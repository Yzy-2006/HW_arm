import sys
import json
import types
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
    assert 'callback_ms(avg/max)=50.0/50.0' in message
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


@pytest.mark.parametrize('slow_write', [True, False])
def test_abort_diagnostics_distinguish_slow_write_and_repeated_camera(tmp_path, monkeypatch, slow_write):
    ns = types.SimpleNamespace
    messages = []

    class FakeNode:
        def __init__(self, name):
            pass

        def create_publisher(self, *args):
            return ns(publish=lambda msg: None)

        def create_timer(self, *args):
            pass

        create_service = create_subscription = create_timer

        def get_logger(self):
            return ns(**{level: lambda message, level=level: messages.append((level, message))
                         for level in ('info', 'warning', 'error')})

        def get_clock(self):
            return ns(now=lambda: ns(nanoseconds=1_000_000_000))

    modules = {
        'rclpy': {}, 'rclpy.node': {'Node': FakeNode},
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

    def write_image(*args):
        clock[0] += .04 if slow_write else .001
        return True

    monkeypatch.setattr(record.cv2, 'imwrite', write_image)
    config = dict(dataset_root=str(tmp_path), cameras={name: name for name in
                  ('head', 'left_wrist', 'right_wrist')}, fps=30, min_episode_frames=1,
                  max_stream_age_s=.2, max_stream_skew_s=.1, max_frame_gap_s=.1,
                  episode_metadata={"source": "policy_intervention", "control_source": "vr"})

    class Camera:
        frame = 1

        def frame_metadata(self):
            return {name: dict(frame_number=self.frame, receive_monotonic_s=10.0)
                    for name in config['cameras']}

        def snapshot(self, now):
            return ({name: np.zeros((2, 2, 3), np.uint8) for name in config['cameras']},
                    self.frame_metadata())

    camera = Camera()
    node = record.make_node(config, camera)
    for name, size in REQUIRED.items():
        node.cache.update(name, [0] * size, clock[0])
    assert node.record(ns(data=True), ns()).success
    metadata = json.loads((node.episode.path / "episode.json").read_text())
    assert metadata["source"] == "policy_intervention"
    assert metadata["control_source"] == "vr"
    node.sample()
    assert node.episode.frames == 1
    if slow_write:
        camera.frame = 2
    else:
        clock[0] = 10.04
        node.sample()
        assert node.diagnostics.duplicate_skips == 1
        clock[0] = 10.11
    node.sample()
    assert node.episode is None
    diagnostic = next(message for level, message in messages if level == 'warning')
    assert 'previous_saved=1' in diagnostic
    if slow_write:
        assert 'write_ms(last/avg/max)=120.0/120.0/120.0' in diagnostic
        assert 'step=1' in diagnostic
        assert 'head=40.0' in diagnostic
    else:
        assert 'duplicate_skips=1' in diagnostic
        assert 'step=0, age_ms=110.0' in diagnostic
    assert any('gap=' in message and 'limit=100.0 ms' in message
               for level, message in messages if level == 'error')
    assert len(list(tmp_path.glob('*.partial'))) == 1


def test_stationary_joint_samples_are_skipped_until_motion_or_gripper_change(tmp_path, monkeypatch):
    ns = types.SimpleNamespace
    class FakeNode:
        def __init__(self, name): pass
        def create_publisher(self, *args): return ns(publish=lambda msg: None)
        def create_timer(self, *args): pass
        create_service = create_subscription = create_timer
        def get_logger(self): return ns(info=lambda *a: None, warning=lambda *a: None, error=lambda *a: None)
        def get_clock(self): return ns(now=lambda: ns(nanoseconds=1_000_000_000))

    modules = {
        'rclpy': {}, 'rclpy.node': {'Node': FakeNode},
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
                  min_joint_delta_rad=.1)

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
