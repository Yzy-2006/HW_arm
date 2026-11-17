import os
from pathlib import Path
import subprocess
import sys
import threading
from types import SimpleNamespace as NS

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "collection"))
import ros_capture
from core import REQUIRED, StreamCache


def test_feedback_is_coalesced_per_topic_without_starving_other_topics(monkeypatch):
    pytest.importorskip("rclpy.callback_groups")
    timers = []
    node = NS(create_timer=lambda period, callback, **options: timers.append((period, callback, options)))
    clock = [10.]
    monkeypatch.setattr(ros_capture, "time", NS(monotonic=lambda: clock[0]))
    group = ros_capture.make_feedback_group(node, 100)

    class Entity:
        pass

    first, second = Entity(), Entity()
    group.add_entity(first)
    group.add_entity(second)
    assert group.can_execute(first) and group.beginning_execution(first)
    assert not group.beginning_execution(second)
    group.ending_execution(first)
    assert not group.can_execute(first) and not group.beginning_execution(first)
    assert group.can_execute(second) and group.beginning_execution(second)
    group.ending_execution(second)
    clock[0] += .011
    assert group.can_execute(first) and group.beginning_execution(first)
    group.ending_execution(first)
    assert timers[0][0] == pytest.approx(.01)
    assert timers[0][2]["callback_group"] is not group
    assert timers[0][1]() is None


@pytest.mark.parametrize("rate", [0, -1, True, float("nan"), float("inf"), "120"])
def test_feedback_rate_must_be_valid_before_any_timer_is_created(rate):
    pytest.importorskip("rclpy.callback_groups")
    with pytest.raises(ValueError, match="feedback_rate_hz"):
        ros_capture.make_feedback_group(NS(), rate)


def test_feedback_qos_does_not_change_global_sensor_qos():
    qos_module = pytest.importorskip("rclpy.qos")
    previous_depth = qos_module.qos_profile_sensor_data.depth
    qos = ros_capture.feedback_qos()
    assert qos.depth == 1 and qos is not qos_module.qos_profile_sensor_data
    assert qos_module.qos_profile_sensor_data.depth == previous_depth


def test_feedback_events_disable_only_optional_dds_diagnostics():
    pytest.importorskip("rclpy.event_handler")
    callbacks = ros_capture.feedback_event_callbacks()
    assert not callbacks.use_default_callbacks
    assert callbacks.incompatible_qos is None and callbacks.incompatible_type is None


def test_capture_releases_cache_lock_before_validation(monkeypatch):
    cache = StreamCache()
    for name, size in REQUIRED.items():
        cache.update(name, [0.] * size, 1.)
    validate = cache._snapshot
    entered, release, updated = threading.Event(), threading.Event(), threading.Event()
    snapshots = []

    def blocked_validation(*args):
        entered.set()
        assert release.wait(2)
        return validate(*args)

    monkeypatch.setattr(cache, "_snapshot", blocked_validation)
    reading = threading.Thread(target=lambda: snapshots.append(cache.capture(lambda: 1.01, .2, .1)))

    def update():
        cache.update("q", [1.] * 14, 1.02)
        updated.set()

    updating = threading.Thread(target=update)
    try:
        reading.start()
        assert entered.wait(1)
        updating.start()
        assert updated.wait(1), "Feedback must not wait for snapshot validation"
    finally:
        release.set()
        reading.join(2)
        if updating.ident is not None:
            updating.join(2)
    assert snapshots[0][0] == 1.01 and snapshots[0][1]["q"] == [0.] * 14
    assert cache.items["q"][0] == [1.] * 14


def test_real_executor_coalesces_high_rate_feedback_and_eventually_takes_last_message(tmp_path):
    code = r'''
import threading
import time
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Int64
from ros_capture import make_feedback_group, feedback_qos, feedback_event_callbacks
from ros_shutdown import drain_executor

rclpy.init()
publisher_node, reader = Node("capture_test_publisher"), Node("capture_test_reader")
publisher = publisher_node.create_publisher(Int64, "/capture_feedback_test", 1)
group = make_feedback_group(reader, 120)
observed, published, ticks = [], [], []
reader.create_subscription(Int64, "/capture_feedback_test", lambda msg: observed.append(msg.data),
                           feedback_qos(), callback_group=group,
                           event_callbacks=feedback_event_callbacks())
reader.create_timer(1/30, lambda: ticks.append(time.monotonic()))
stop = threading.Event()
def publish():
    count = 0
    while not stop.is_set():
        count += 1
        publisher.publish(Int64(data=count))
        published.append(count)
        stop.wait(.001)
thread = threading.Thread(target=publish)
executor = MultiThreadedExecutor(num_threads=3)
executor.add_node(reader)
started = time.monotonic()
cpu_started = time.process_time()
try:
    thread.start()
    while time.monotonic() - started < 1.5:
        executor.spin_once(timeout_sec=.02)
    elapsed = time.monotonic() - started
    cpu_seconds = time.process_time() - cpu_started
    print(f"feedback_cpu_seconds={cpu_seconds:.3f} wall_seconds={elapsed:.3f} ticks={len(ticks)}")
    assert cpu_seconds < elapsed * .6, (cpu_seconds, elapsed)
    stop.set()
    thread.join(1)
    deadline = time.monotonic() + .5
    while (not observed or observed[-1] != published[-1]) and time.monotonic() < deadline:
        executor.spin_once(timeout_sec=.02)
    assert 15 < len(observed) <= elapsed * 120 + 5, (len(observed), elapsed)
    assert len(ticks) >= 30, len(ticks)
    assert observed[-1] == published[-1], (observed[-1], published[-1])
    assert len(published) > len(observed) * 2, (len(published), len(observed))
finally:
    stop.set()
    thread.join(1)
    drain_executor(executor)
    reader.destroy_node()
    publisher_node.destroy_node()
    rclpy.shutdown()
'''
    env = dict(os.environ, ROS_DOMAIN_ID="221", ROS_AUTOMATIC_DISCOVERY_RANGE="LOCALHOST",
               ROS_LOG_DIR=str(tmp_path / "ros_logs"),
               PYTHONPATH=os.pathsep.join([str(ROOT / "deployment"), os.environ.get("PYTHONPATH", "")]))
    result = subprocess.run([sys.executable, "-c", code], cwd=ROOT / "collection", env=env,
                            capture_output=True, text=True, timeout=10)
    assert result.returncode == 0, result.stdout + result.stderr
