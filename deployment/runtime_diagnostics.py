"""Low-rate Linux resource sampling, executed outside the control/recorder processes."""

import json
import os
from pathlib import Path
import signal
import time


def read_process(path):
    raw = (path / 'stat').read_text()
    name = raw[raw.index('(') + 1:raw.rindex(')')]
    fields = raw[raw.rindex(')') + 2:].split()
    result = dict(pid=int(path.name), name=name, state=fields[0],
                  cpu_ticks=int(fields[11]) + int(fields[12]), start_ticks=int(fields[19]),
                  rss_bytes=int(fields[21]) * os.sysconf('SC_PAGE_SIZE'),
                  io_delay_ticks=int(fields[39]) if len(fields) > 39 else 0)
    try:
        io = dict(line.split(':', 1) for line in (path / 'io').read_text().splitlines())
        result.update(read_bytes=int(io['read_bytes']), write_bytes=int(io['write_bytes']))
    except (OSError, ValueError, KeyError):
        pass
    return result


def process_delta(current, previous, elapsed, hz):
    result = {key: current[key] for key in ('pid', 'name', 'state', 'rss_bytes')}
    if previous is None or current['start_ticks'] != previous['start_ticks'] or elapsed <= 0:
        return result
    result['cpu_percent'] = max(0, current['cpu_ticks'] - previous['cpu_ticks']) / hz / elapsed * 100
    result['leader_io_delay_ms'] = max(0, current['io_delay_ticks'] - previous['io_delay_ticks']) / hz * 1000
    for key in ('read_bytes', 'write_bytes'):
        if key in current and key in previous:
            result[key + '_per_s'] = max(0, current[key] - previous[key]) / elapsed
    return result


class ResourceSampler:
    def __init__(self, watched, proc=Path('/proc')):
        self.watched, self.proc = watched, proc
        self.previous, self.threads = {}, {}
        self.last_time = None
        self.last_cpu = None
        self.hz = os.sysconf('SC_CLK_TCK')

    def thread_samples(self, path, identity, elapsed):
        rows, current = [], {}
        for task in (path / 'task').iterdir():
            try:
                values = [int(value) for value in (task / 'schedstat').read_text().split()[:2]]
                stat = (task / 'stat').read_text()
                fields = stat[stat.rindex(')') + 2:].split()
                values.append(int(fields[39]) if len(fields) > 39 else 0)
                key = (identity, int(task.name))
                current[key] = values
                previous = self.threads.get(key)
                if previous is not None and elapsed > 0:
                    rows.append(dict(tid=int(task.name),
                                     name=(task / 'comm').read_text().strip(),
                                     cpu_ms=max(0, values[0] - previous[0]) / 1e6,
                                     runqueue_wait_ms=max(0, values[1] - previous[1]) / 1e6,
                                     io_delay_ms=max(0, values[2] - previous[2]) / self.hz * 1000))
            except (OSError, ValueError):
                continue
        return sorted(rows, key=lambda row: row['runqueue_wait_ms'] + row['io_delay_ms'], reverse=True)[:8], current

    def sample(self, now):
        elapsed = now - self.last_time if self.last_time is not None else 0.
        current, threads, rows = {}, {}, []
        errors = []
        for path in self.proc.iterdir():
            if not path.name.isdigit():
                continue
            try:
                if path.stat().st_uid != os.getuid():
                    continue
                value = read_process(path)
                pid = value['pid']
                current[pid] = value
                row = process_delta(value, self.previous.get(pid), elapsed, self.hz)
                if pid in self.watched:
                    row['role'] = self.watched[pid]
                    row['threads'], counters = self.thread_samples(path, (pid, value['start_ticks']), elapsed)
                    threads.update(counters)
                rows.append(row)
            except (OSError, ValueError, IndexError):
                continue  # Processes may exit during a /proc scan.
        system = {}
        try:
            cpu = [int(v) for v in (self.proc / 'stat').read_text().splitlines()[0].split()[1:9]]
            if self.last_cpu is not None:
                delta = [max(0, a - b) for a, b in zip(cpu, self.last_cpu)]
                total = sum(delta)
                if total:
                    system.update(cpu_busy_percent=(total - delta[3] - delta[4]) / total * 100,
                                  cpu_iowait_percent=delta[4] / total * 100)
            self.last_cpu = cpu
            memory = dict(line.split(':', 1) for line in (self.proc / 'meminfo').read_text().splitlines())
            system['memory_available_bytes'] = int(memory['MemAvailable'].split()[0]) * 1024
            system['swap_free_bytes'] = int(memory['SwapFree'].split()[0]) * 1024
            for kind in ('cpu', 'io', 'memory'):
                try:
                    system[kind + '_pressure'] = {
                        line.split()[0]: dict(item.split('=') for item in line.split()[1:])
                        for line in (self.proc / 'pressure' / kind).read_text().splitlines()}
                except OSError:
                    system[kind + '_pressure'] = None
            for key in ('sched_schedstats', 'task_delayacct'):
                try:
                    system[key] = (self.proc / 'sys/kernel' / key).read_text().strip()
                except OSError:
                    system[key] = None
        except (OSError, ValueError, KeyError, IndexError) as exc:
            errors.append(str(exc))
        # Include watched processes plus the busiest same-user processes, without command arguments.
        selected = {row['pid']: row for row in rows if 'role' in row}
        for key in ('cpu_percent', 'write_bytes_per_s', 'read_bytes_per_s'):
            for row in sorted(rows, key=lambda row: row.get(key, 0), reverse=True)[:8]:
                selected[row['pid']] = row
        self.previous, self.threads, self.last_time = current, threads, now
        return dict(monotonic_s=now, wall_time_ns=time.time_ns(), interval_s=elapsed,
                    system=system, processes=list(selected.values()), errors=errors)


def monitor_resources(parent_pid, recorder_pid, output, status, stop):
    def request_stop(*_):
        stop.value = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    sampler = ResourceSampler({parent_pid: 'bridge', recorder_pid: 'recorder', os.getpid(): 'diagnostics'})
    path = Path(output)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('x', buffering=1) as stream:
        deadline = 0.
        while True:
            now = time.monotonic()
            if now >= deadline or stop.value:
                row = sampler.sample(now)
                try:
                    row['capture'] = status.read()
                except Exception as exc:
                    row['capture_error'] = str(exc)
                stream.write(json.dumps(row, allow_nan=False) + '\n')
                deadline = now + 1.
            if stop.value or os.getppid() != parent_pid:
                break
            time.sleep(.1)
