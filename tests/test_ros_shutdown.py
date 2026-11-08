from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import sys
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'deployment'))
from ros_shutdown import drain_executor


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
