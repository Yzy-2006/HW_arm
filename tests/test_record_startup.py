import json
import sys
import threading
import os
import subprocess
from pathlib import Path
from types import ModuleType, SimpleNamespace as NS

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'collection'))
import record
from core import REQUIRED, StreamCache


@pytest.fixture
def recorder(monkeypatch, tmp_path):
    clock = [10.0]
    messages = []

    class Node:
        def __init__(self, name):
            self.default_callback_group = object()
            self.subscriptions = []
        def create_publisher(self, *args): return NS(publish=lambda msg: None)
        def create_timer(self, *args, **kwargs): pass
        create_service = create_timer
        def create_subscription(self, *args, **kwargs): self.subscriptions.append(kwargs)
        def get_clock(self): return NS(now=lambda: NS(nanoseconds=int(clock[0] * 1e9)))
        def get_logger(self):
            return NS(info=messages.append, warning=messages.append, error=messages.append)
        def destroy_node(self): pass

    modules = {
        'rclpy': {}, 'rclpy.node': {'Node': Node},
        'rclpy.callback_groups': {'MutuallyExclusiveCallbackGroup': object},
        'rclpy.qos': {'qos_profile_sensor_data': object()},
        'ctl_msgs_hw': {}, 'ctl_msgs_hw.msg': {name: NS for name in (
            'CtlInformation', 'FkDataPub', 'MidNodeCtlMsgRet', 'MidNodeCtlMsgSend')},
        'std_msgs': {}, 'std_msgs.msg': {'Int8MultiArray': NS, 'Bool': NS},
        'sensor_msgs': {}, 'sensor_msgs.msg': {'Image': NS},
        'std_srvs': {}, 'std_srvs.srv': {'SetBool': NS, 'Trigger': NS},
    }
    for name, attributes in modules.items():
        module = ModuleType(name)
        module.__dict__.update(attributes)
        monkeypatch.setitem(sys.modules, name, module)
    monkeypatch.setattr(record, 'time', NS(monotonic=lambda: clock[0], time_ns=lambda: int(clock[0] * 1e9)))
    monkeypatch.setattr(record.cv2, 'imwrite', lambda *args: True)
    config = dict(dataset_root=str(tmp_path), cameras={'head': 'h'}, fps=30,
                  min_episode_frames=1, max_stream_age_s=.2, max_stream_skew_s=.1,
                  max_frame_gap_s=.1, startup_timeout_s=3., writer_finish_timeout_s=1.)

    class Camera:
        stamp = None
        index = 0
        def snapshot(self, now):
            self.index += 1
            return ({'head': np.zeros((2, 2, 3), np.uint8)},
                    {'head': {'frame_number': self.index,
                              'receive_monotonic_s': now if self.stamp is None else self.stamp}})

    camera = Camera()
    node = record.make_node(config, camera)

    def feed(exclude=()):
        for key, size in REQUIRED.items():
            if key not in exclude:
                node.put(key, [0.] * size, NS())

    yield node, clock, camera, feed, messages
    node.close()


def test_slow_initialization_waits_for_new_inputs_and_anchors_first_frame(recorder, monkeypatch):
    node, clock, camera, feed, messages = recorder
    feed()
    writer = record.AsyncEpisodeWriter

    def slow_writer(*args, **kwargs):
        clock[0] += .224
        return writer(*args, **kwargs)

    monkeypatch.setattr(record, 'AsyncEpisodeWriter', slow_writer)
    assert node.record(NS(data=True), NS()).success
    assert node.prepared_at == pytest.approx(10.224)
    node.sample()
    assert node.episode.frames == 0 and node.start_time is None
    assert 'post-startup streams' in node.startup_wait_reason

    clock[0] += .3
    feed(exclude=('tcp',))
    node.sample()
    assert node.start_time is None and 'tcp' in node.startup_wait_reason
    feed()
    camera.stamp = 10.
    node.sample()
    assert node.start_time is None and 'camera frames' in node.startup_wait_reason
    camera.stamp = None
    node.sample()
    assert node.episode.frames == 1 and node.start_time == clock[0]
    assert node.diagnostics.frame_gap_max == 0.
    clock[0] += .033
    feed()
    node.sample()
    assert node.record(NS(data=False), NS()).success
    path = node.retiring_episode._result
    meta = json.loads((path / 'episode.json').read_text())
    rows = [json.loads(line) for line in (path / 'frames.jsonl').read_text().splitlines()]
    assert rows[0]['timestamp_s'] == 0.
    assert rows[1]['timestamp_s'] == pytest.approx(.033)
    assert meta['start_monotonic_s'] == rows[0]['monotonic_s']
    assert meta['requested_monotonic_s'] < meta['start_monotonic_s']
    assert meta['start_wall_time_ns'] == int(rows[0]['monotonic_s'] * 1e9)


def test_startup_timeout_keeps_partial_and_reports_missing_streams(recorder):
    node, clock, camera, feed, messages = recorder
    assert node.record(NS(data=True), NS()).success
    clock[0] += .25
    feed(exclude=('gripper_state',))
    node.sample()
    assert node.start_time is None and node.episode.frames == 0
    assert 'gripper_state' in node.startup_wait_reason
    clock[0] += 3.
    node.sample()
    assert node.episode is None and node.retiring_episode.done.wait(1)
    meta = json.loads((node.retiring_episode.path / 'episode.json').read_text())
    assert meta['status'] == 'aborted' and meta['frames'] == 0
    assert 'startup timed out' in meta['reason'] and 'gripper_state' in meta['reason']


@pytest.mark.parametrize('failure', ['gap', 'stale'])
def test_running_capture_keeps_existing_age_and_gap_checks(recorder, failure):
    node, clock, camera, feed, messages = recorder
    assert node.record(NS(data=True), NS()).success
    clock[0] += .01
    feed()
    node.sample()
    assert node.start_time is not None
    clock[0] += .111 if failure == 'gap' else .21
    if failure == 'gap':
        feed()
    node.sample()
    assert node.episode is None
    assert ('frame gap' if failure == 'gap' else 'stale') in node.last_recording_error
    if failure == 'stale':
        assert 'tcp(age_s=' in node.last_recording_error


def test_independent_feedback_group_and_atomic_joint_snapshot(recorder, monkeypatch):
    node, clock, camera, feed, messages = recorder
    assert node.feedback_group is not node.default_callback_group
    assert all(sub['callback_group'] is node.feedback_group for sub in node.subscriptions)
    feed()
    original = node.put
    first_joint, release, snapshot_done = threading.Event(), threading.Event(), threading.Event()

    def blocked_put(key, values, msg):
        original(key, values, msg)
        if key == 'q':
            first_joint.set()
            assert release.wait(2)

    monkeypatch.setattr(node, 'put', blocked_put)
    feedback = NS(l_arm_pos_fdback=[1.] * 7, r_arm_pos_fdback=[1.] * 7,
                  l_arm_spd_fdback=[2.] * 7, r_arm_spd_fdback=[2.] * 7)
    updating = threading.Thread(target=lambda: node.joints(feedback))
    snapshots = []

    def snapshot():
        snapshots.append(node.snapshot()[1])
        snapshot_done.set()

    reading = threading.Thread(target=snapshot)
    try:
        updating.start()
        assert first_joint.wait(1)
        reading.start()
        assert not snapshot_done.wait(.05)
    finally:
        release.set()
        updating.join(2)
        if reading.ident is not None:
            reading.join(2)
    assert snapshots[0]['q'] == [1.] * 14
    assert snapshots[0]['dq'] == [2.] * 14


def test_cancel_preparing_does_not_leave_writer_or_start_next_capture_early(recorder):
    node, clock, camera, feed, messages = recorder
    assert node.record(NS(data=True), NS()).success
    old = node.episode
    assert node.discard(NS(), NS()).success
    assert old.done.is_set() and not old.path.exists()
    feed()
    assert node.record(NS(data=True), NS()).success
    node.sample()
    assert node.episode.frames == 0 and node.start_time is None


def test_cache_snapshot_does_not_expose_mutable_values():
    cache = StreamCache()
    for key, size in REQUIRED.items():
        cache.update(key, [0.] * size, 1.)
    snapshot = cache.snapshot(1., .2, .1)
    snapshot['q'][0] = 99.
    assert cache.snapshot(1., .2, .1)['q'][0] == 0.


def test_ros_executor_receives_feedback_while_recorder_prepares(tmp_path):
    code = r'''
import threading
import time
from types import SimpleNamespace as NS
import sys
import numpy as np
import rclpy
from rclpy.executors import MultiThreadedExecutor
import record
from core import REQUIRED

rclpy.init()
entered, updated, release, captured = [threading.Event() for _ in range(4)]
original = record.AsyncEpisodeWriter
def slow_writer(*args, **kwargs):
    entered.set()
    assert release.wait(3)
    return original(*args, **kwargs)
record.AsyncEpisodeWriter = slow_writer
class Cameras:
    count = 0
    def snapshot(self, now):
        self.count += 1
        return ({'head': np.zeros((2, 2, 3), np.uint8)},
                {'head': {'frame_number': self.count, 'receive_monotonic_s': now}})
config = dict(dataset_root=sys.argv[1], cameras={'head': 'test'}, fps=30,
              min_episode_frames=1, max_stream_age_s=.2, max_stream_skew_s=.1,
              max_frame_gap_s=.1, startup_timeout_s=3., writer_finish_timeout_s=1.)
node = record.make_node(config, Cameras())
ticks = [0]
def feedback():
    for name, size in REQUIRED.items():
        node.put(name, [0.] * size, NS())
    if entered.is_set() and not release.is_set():
        ticks[0] += 1
        if ticks[0] >= 25:
            updated.set()
feedback_timer = node.create_timer(.01, feedback, callback_group=node.feedback_group)
responses = []
def start():
    start_timer.cancel()
    responses.append(node.record(NS(data=True), NS()))
start_timer = node.create_timer(.05, start)
def observe():
    if node.start_time is not None:
        captured.set()
observe_timer = node.create_timer(.01, observe)
executor = MultiThreadedExecutor(num_threads=3)
executor.add_node(node)
thread = threading.Thread(target=executor.spin)
thread.start()
try:
    assert entered.wait(2), 'Recorder preparation never started'
    assert updated.wait(2), 'Feedback callbacks were blocked by preparation'
    assert node.episode is None
    release.set()
    assert captured.wait(2), 'No fresh first frame after preparation'
    assert responses[0].success and node.last_recording_error == ''
    assert node.start_time >= node.prepared_at
finally:
    release.set()
    executor.shutdown(wait_for_threads=True)
    thread.join(2)
    node.close()
    rclpy.shutdown()
'''
    env = dict(os.environ, ROS_DOMAIN_ID='218', ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST',
               ROS_LOG_DIR=str(tmp_path / 'ros_logs'))
    result = subprocess.run([sys.executable, '-c', code, str(tmp_path / 'episodes')],
                            cwd=Path(record.__file__).parent, env=env,
                            capture_output=True, text=True, timeout=15)
    assert result.returncode == 0, result.stderr + result.stdout
