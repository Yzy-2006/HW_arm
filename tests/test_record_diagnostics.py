import sys
import json
import types
import threading
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'collection'))
import record
from core import REQUIRED


def test_diagnostics_rate_limit_and_timing_summary():
    diagnostics = record.RecordingDiagnostics(10.0)
    diagnostics.tick(10.03, 10.0)
    diagnostics.tick(10.08, 10.03)
    diagnostics.wrote(.02)
    diagnostics.wrote(.04)
    diagnostics.camera_write_s = {'head': .015}
    diagnostics.camera_meta = {'head': {'frame_number': 7, 'receive_monotonic_s': 12.0}}
    diagnostics.previous_ids = {'head': 7}
    diagnostics.duplicate_skips = 1
    diagnostics.motion_skips = 2
    assert diagnostics.report(11.0, 2) is None
    message = diagnostics.report(12.02, 2)
    assert 'callback_interval_ms(avg/max)=50.0/50.0' in message
    assert 'frame_gap_ms(current/max)=50.0/50.0' in message
    assert 'write_ms(last/avg/max)=40.0/30.0/40.0' in message
    assert 'jpeg_write_ms=[head=15.0]' in message
    assert 'duplicate_skips=1' in message
    assert 'motion_skips=2' in message
    assert 'head(frame=7, previous_saved=7, step=0, age_ms=20.0)' in message
    assert diagnostics.report(12.03, 2) is None
    assert diagnostics.report(12.03, 2, force=True) is not None
    diagnostics.camera_observed_at = 12.02
    assert 'age_ms=20.0' in diagnostics.report(12.10, 2, force=True)


def test_callback_wall_time_is_distinct_from_thread_cpu_and_interval():
    diagnostics = record.RecordingDiagnostics(0.)
    diagnostics.tick(.03, 0.)
    diagnostics.tick(.134, .03)
    diagnostics.finished(.020, .002)
    diagnostics.finished(.006, .004)
    message = diagnostics.report(.14, 2, force=True)
    assert 'callback_interval_ms(avg/max)=104.0/104.0' in message
    assert 'callback_exec_ms(last/avg/max)=6.0/13.0/20.0' in message
    assert 'callback_thread_cpu_ms(last/avg/max)=4.0/3.0/4.0' in message


def test_receiver_tracks_host_arrivals_device_timestamps_and_frame_gaps(monkeypatch):
    camera = record.Cameras.__new__(record.Cameras)
    camera.stop, camera.lock = threading.Event(), threading.Lock()
    camera.latest, camera.error = {}, None
    frames = iter([(1, 0.), (2, 33.), (4, 100.)])
    arrivals = iter([1., 1.033, 1.150])
    monkeypatch.setattr(record, 'time', types.SimpleNamespace(monotonic=lambda: next(arrivals)))

    def wait_for_frames(timeout):
        frame, stamp = next(frames)
        if frame == 4:
            camera.stop.set()
        color = types.SimpleNamespace(get_frame_number=lambda: frame, get_timestamp=lambda: stamp,
            get_frame_timestamp_domain=lambda: 'hardware_clock',
            get_data=lambda: np.zeros((2, 2, 3), np.uint8))
        return types.SimpleNamespace(get_color_frame=lambda: color)

    camera._receive('head', types.SimpleNamespace(wait_for_frames=wait_for_frames))
    assert camera.error is None
    meta = camera.frame_metadata()['head']
    assert meta['arrival_interval_ms'] == pytest.approx(117.)
    assert meta['arrival_interval_max_ms'] == pytest.approx(117.)
    assert meta['device_interval_ms'] == 67.
    assert meta['receiver_frame_gaps'] == 1


@pytest.mark.parametrize('slow_write', [True, False])
def test_capture_survives_blocked_writer_but_rejects_repeated_camera(tmp_path, monkeypatch, slow_write):
    ns = types.SimpleNamespace
    messages = []

    class FakeNode:
        def __init__(self, name):
            pass

        def create_publisher(self, *args):
            return ns(publish=lambda msg: None)

        def create_timer(self, *args, **kwargs):
            pass

        create_service = create_subscription = create_timer

        def get_logger(self):
            return ns(**{level: lambda message, level=level: messages.append((level, message))
                         for level in ('info', 'warning', 'error')})

        def get_clock(self):
            return ns(now=lambda: ns(nanoseconds=1_000_000_000))

    modules = {
        'rclpy': {}, 'rclpy.node': {'Node': FakeNode},
        'rclpy.callback_groups': {'MutuallyExclusiveCallbackGroup': object},
        'rclpy.qos': {'qos_profile_sensor_data': object()},
        'ctl_msgs_hw': {}, 'ctl_msgs_hw.msg': {name: ns for name in (
            'CtlInformation', 'FkDataPub', 'MidNodeCtlMsgRet', 'MidNodeCtlMsgSend')},
        'std_msgs': {}, 'std_msgs.msg': {'Int8MultiArray': ns, 'Bool': ns},
        'sensor_msgs': {}, 'sensor_msgs.msg': {'Image': ns},
        'std_srvs': {}, 'std_srvs.srv': {'SetBool': ns, 'Trigger': ns},
    }
    for name, attributes in modules.items():
        module = types.ModuleType(name)
        module.__dict__.update(attributes)
        monkeypatch.setitem(sys.modules, name, module)
    clock = [10.0]
    monkeypatch.setattr(record, 'time', ns(monotonic=lambda: clock[0], time_ns=lambda: 1))

    writing, release = threading.Event(), threading.Event()
    def write_image(*args):
        writing.set()
        if slow_write:
            assert release.wait(3)
        return True

    monkeypatch.setattr(record.cv2, 'imwrite', write_image)
    config = dict(dataset_root=str(tmp_path), cameras={name: name for name in
                  ('head', 'left_wrist', 'right_wrist')}, fps=30, min_episode_frames=1,
                  max_stream_age_s=.2, max_stream_skew_s=.1, max_frame_gap_s=.1,
                  episode_metadata={"source": "policy_intervention", "control_source": "vr"})

    class Camera:
        frame = 1

        def frame_metadata(self):
            return {name: dict(frame_number=self.frame, receive_monotonic_s=clock[0])
                    for name in config['cameras']}

        def snapshot(self, now):
            return ({name: np.zeros((2, 2, 3), np.uint8) for name in config['cameras']},
                    self.frame_metadata())

    camera = Camera()
    node = record.make_node(config, camera)
    for name, size in REQUIRED.items():
        node.cache.update(name, [0] * size, clock[0])
    assert node.record(ns(data=True), ns()).success
    clock[0] += .001
    for name, size in REQUIRED.items():
        node.cache.update(name, [0] * size, clock[0])
    metadata = json.loads((node.episode.path / "episode.json").read_text())
    assert metadata["source"] == "policy_intervention"
    assert metadata["control_source"] == "vr"
    node.sample()
    assert node.episode.frames == 1
    if slow_write:
        result = []
        saving = None
        try:
            assert writing.wait(1)
            for frame in range(2, 9):
                camera.frame = frame
                clock[0] += 1 / 30
                for name, size in REQUIRED.items():
                    node.cache.update(name, [0] * size, clock[0])
                node.sample()
            assert node.episode.frames == 8
            assert node.episode.stats()['written'] == 0
            assert node.diagnostics.frame_gap_max < .1
            saving = threading.Thread(target=lambda: result.append(node.record(ns(data=False), ns())))
            saving.start()
            saving.join(.05)
            assert saving.is_alive() and not result
        finally:
            release.set()
            if saving is not None:
                saving.join(3)
        assert result[0].success and node.episode is None
        saved = next(path for path in tmp_path.glob('episode_*') if not path.name.endswith('.partial'))
        meta = json.loads((saved / 'episode.json').read_text())
        assert meta['frames'] == 8 and meta['status'] == 'complete'
        rows = [json.loads(line) for line in (saved / 'frames.jsonl').read_text().splitlines()]
        assert [row['frame_index'] for row in rows] == list(range(8))
        assert rows[-1]['timestamp_s'] == pytest.approx(7 / 30)
        return
    clock[0] = 10.04
    node.sample()
    assert node.diagnostics.duplicate_skips == 1
    clock[0] = 10.11
    node.sample()
    assert node.episode is None
    assert node.retiring_episode.done.wait(2)
    diagnostic = next(message for level, message in messages if level == 'warning')
    assert 'previous_saved=1' in diagnostic
    assert 'duplicate_skips=1' in diagnostic
    assert 'step=0' in diagnostic
    assert any('gap=' in message and 'limit=100.0 ms' in message
               for level, message in messages if level == 'error')
    assert len(list(tmp_path.glob('*.partial'))) == 1


@pytest.mark.parametrize('camera_timing_checks_enabled', [True, False])
def test_stationary_joint_samples_are_skipped_until_motion_or_gripper_change(tmp_path, monkeypatch, camera_timing_checks_enabled):
    ns = types.SimpleNamespace
    class FakeNode:
        def __init__(self, name): pass
        def create_publisher(self, *args): return ns(publish=lambda msg: None)
        def create_timer(self, *args, **kwargs): pass
        create_service = create_subscription = create_timer
        def get_logger(self): return ns(info=lambda *a: None, warning=lambda *a: None, error=lambda *a: None)
        def get_clock(self): return ns(now=lambda: ns(nanoseconds=1_000_000_000))

    modules = {
        'rclpy': {}, 'rclpy.node': {'Node': FakeNode},
        'rclpy.callback_groups': {'MutuallyExclusiveCallbackGroup': object},
        'rclpy.qos': {'qos_profile_sensor_data': object()},
        'ctl_msgs_hw': {}, 'ctl_msgs_hw.msg': {name: ns for name in (
            'CtlInformation', 'FkDataPub', 'MidNodeCtlMsgRet', 'MidNodeCtlMsgSend')},
        'std_msgs': {}, 'std_msgs.msg': {'Int8MultiArray': ns, 'Bool': ns},
        'sensor_msgs': {}, 'sensor_msgs.msg': {'Image': ns},
        'std_srvs': {}, 'std_srvs.srv': {'SetBool': ns, 'Trigger': ns},
    }
    for name, attributes in modules.items():
        module = types.ModuleType(name)
        module.__dict__.update(attributes)
        monkeypatch.setitem(sys.modules, name, module)
    clock = [10.0]
    monkeypatch.setattr(record, 'time', ns(monotonic=lambda: clock[0], time_ns=lambda: 1))
    monkeypatch.setattr(record.cv2, 'imwrite', lambda *args: True)
    config = dict(dataset_root=str(tmp_path), cameras={name: name for name in ('head', 'left_wrist', 'right_wrist')},
                  fps=30, min_episode_frames=1, max_stream_age_s=.5, max_stream_skew_s=.1,
                  max_camera_age_s=.5, max_camera_skew_s=.1, max_frame_gap_s=.1,
                  min_joint_delta_rad=.1, camera_timing_checks_enabled=camera_timing_checks_enabled,
                  episode_metadata={"source": "policy_intervention", "control_source": "vr"})

    class Camera:
        frame = 0
        def snapshot(self, now):
            self.frame += 1
            return ({name: np.zeros((2, 2, 3), np.uint8) for name in config['cameras']},
                    {name: dict(frame_number=self.frame, receive_monotonic_s=now) for name in config['cameras']})

    camera = Camera()
    node = record.make_node(config, camera)
    for key, size in REQUIRED.items():
        node.cache.update(key, [0] * size, clock[0])
    assert node.record(ns(data=True), ns()).success
    clock[0] += .001
    for key, size in REQUIRED.items():
        node.cache.update(key, [0] * size, clock[0])
    node.sample()
    assert node.episode.frames == 1

    clock[0] = 10.03
    node.sample()
    assert node.episode.frames == 1
    assert node.diagnostics.motion_skips == 1

    node.cache.update('q', [0.2] + [0] * 13, clock[0])
    clock[0] = 10.06
    node.sample()
    assert node.episode.frames == 2

    node.cache.update('q', [0.2] + [0] * 13, clock[0])
    node.cache.update('gripper_state', [1, 0, 0, 0], clock[0])
    clock[0] = 10.09
    node.sample()
    assert node.episode.frames == 3
    response = node.record(ns(data=False), ns())
    assert response.success and node.episode is None
    saved = next(tmp_path.glob('episode_*/episode.json'))
    metadata = json.loads(saved.read_text())
    assert metadata['status'] == 'complete' and metadata['frames'] == 3
    assert metadata['source'] == 'policy_intervention'
    assert metadata['config']['camera_timing_checks_enabled'] is camera_timing_checks_enabled
    assert metadata['config']['max_camera_age_s'] == .5
    json.dumps(metadata, allow_nan=False)


@pytest.mark.parametrize('enabled', [None, True, False])
def test_camera_timing_switch_preserves_readiness_and_receiver_errors(enabled):
    camera = record.Cameras.__new__(record.Cameras)
    camera.lock = threading.Lock()
    camera.config = {'max_camera_age_s': .5, 'max_camera_skew_s': .1}
    if enabled is not None:
        camera.config['camera_timing_checks_enabled'] = enabled
    camera.error = None
    camera.pipelines = {'head': None, 'left_wrist': None}
    camera.latest = {}
    with pytest.raises(ValueError, match='Waiting for all cameras'):
        camera.snapshot(10.)
    camera.latest = {name: (np.zeros((2, 2, 3), np.uint8), {'receive_monotonic_s': stamp})
                     for name, stamp in (('head', 9.8), ('left_wrist', 10.))}
    if enabled is False:
        assert len(camera.snapshot(12.)[0]) == 2
    else:
        with pytest.raises(ValueError, match='skew'):
            camera.snapshot(10.)
        with pytest.raises(ValueError, match='stale'):
            camera.snapshot(12.)
    camera.error = 'receiver disconnected'
    with pytest.raises(ValueError, match='Camera receive failure'):
        camera.snapshot(10.)
