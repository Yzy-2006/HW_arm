import json
from pathlib import Path
import sys
from concurrent.futures import ThreadPoolExecutor
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'deployment'))
from runtime_diagnostics import ResourceSampler, process_delta, read_process
from ros_shutdown import drain_executor


def test_proc_stat_parses_names_with_spaces_and_parentheses(tmp_path):
    path = tmp_path / '123'
    path.mkdir()
    fields = ['0'] * 40
    fields[0] = 'S'
    fields[11], fields[12], fields[19], fields[21], fields[39] = '20', '30', '100', '2', '4'
    (path / 'stat').write_text('123 (worker (capture)) ' + ' '.join(fields))
    (path / 'io').write_text('read_bytes: 100\nwrite_bytes: 200\n')
    value = read_process(path)
    assert value['name'] == 'worker (capture)' and value['cpu_ticks'] == 50
    old = dict(value, cpu_ticks=0, io_delay_ticks=2, write_bytes=100)
    delta = process_delta(value, old, 2., 100.)
    assert delta['cpu_percent'] == 25.
    assert delta['leader_io_delay_ms'] == 20.
    assert delta['write_bytes_per_s'] == 50.
    assert 'cpu_percent' not in process_delta(value, dict(old, start_ticks=99), 2., 100.)


def test_resource_sampling_is_json_serializable_and_tolerates_missing_proc_fields(tmp_path):
    sampler = ResourceSampler({}, proc=tmp_path)
    first = sampler.sample(10.)
    second = sampler.sample(11.)
    assert first['interval_s'] == 0. and second['interval_s'] == 1.
    assert second['errors'] and second['processes'] == []
    json.dumps(second, allow_nan=False)


def test_thread_scheduler_and_io_wait_deltas_are_separate(tmp_path):
    task = tmp_path / 'task/124'
    task.mkdir(parents=True)
    (task / 'comm').write_text('writer\n')
    fields = ['0'] * 40
    fields[0] = 'S'
    fields[39] = '5'
    (task / 'stat').write_text('124 (writer) ' + ' '.join(fields))
    (task / 'schedstat').write_text('10000000 20000000 1\n')
    sampler = ResourceSampler({})
    sampler.hz = 100
    rows, sampler.threads = sampler.thread_samples(tmp_path, (123, 1), 1.)
    assert rows == []
    fields[39] = '8'
    (task / 'stat').write_text('124 (writer) ' + ' '.join(fields))
    (task / 'schedstat').write_text('14000000 27000000 2\n')
    rows, _ = sampler.thread_samples(tmp_path, (123, 1), 1.)
    assert rows == [dict(tid=124, name='writer', cpu_ms=4., runqueue_wait_ms=7., io_delay_ms=30.)]


def test_shutdown_drains_queued_callbacks_before_destroying_guard():
    events = []
    pool = ThreadPoolExecutor(max_workers=1)
    futures = [pool.submit(lambda: events.append('callback')) for _ in range(20)]

    def shutdown(**kwargs):
        assert len(events) == 20
        events.append('destroy')
        pool.shutdown(wait=True)

    drain_executor(SimpleNamespace(_executor=pool, _futures=futures, shutdown=shutdown))
    assert events[-1] == 'destroy'
