"""Isolated ROS integration test; synthetic cameras/feedback, no motion publishers."""

import json
import multiprocessing as mp
import os
from pathlib import Path
import sys
import time

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'deployment'))
from recorder_process import RecorderProcess


class FakeCameras:
    def __init__(self, config):
        self.config = config
    def snapshot(self, now):
        frame = int(time.monotonic() * self.config['fps'])
        images = {name: np.full((self.config['height'], self.config['width'], 3), frame % 255, np.uint8)
                  for name in self.config['cameras']}
        metadata = {name: dict(frame_number=frame, receive_monotonic_s=time.monotonic(),
                              device_timestamp_ms=frame * 1000 / self.config['fps'], owner_pid=os.getpid())
                    for name in self.config['cameras']}
        return images, metadata
    def frame_metadata(self):
        return self.snapshot(time.monotonic())[1]
    def close(self): pass


def feed_robot(stop):
    import rclpy
    from ctl_msgs_hw.msg import CtlInformation, FkDataPub, MidNodeCtlMsgRet, MidNodeCtlMsgSend
    from std_msgs.msg import Int8MultiArray
    rclpy.init()
    node = rclpy.create_node('independent_recorder_test_feedback')
    types = [(MidNodeCtlMsgRet, '/dexbotCtl/midMonitor_msg_send'),
             (CtlInformation, '/dexbotCtl/ctl_data_pub'),
             (FkDataPub, '/dexbotCtl/DescPub/Fk_ret_data_pub'),
             (MidNodeCtlMsgSend, '/dexbotCtl/upperMonitor_msg_send'),
             (Int8MultiArray, '/towel/gripper_state')]
    publishers = []
    for kind, topic in types:
        message = kind()
        for field, field_type in message.get_fields_and_field_types().items():
            if field_type == 'sequence<double>':
                setattr(message, field, [0.] * (6 if 'descartes' in field else 7))
            elif field_type == 'sequence<boolean>':
                setattr(message, field, [True] * 7)
        if kind == Int8MultiArray:
            message.data = [0, 1, 0, 1]
        publishers.append((node.create_publisher(kind, topic, 10), message))
    try:
        while not stop.wait(.01):
            for publisher, message in publishers:
                publisher.publish(message)
            rclpy.spin_once(node, timeout_sec=0)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def wait_for(predicate, timeout=5.):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(.01)
    raise AssertionError('Timed out waiting for recorder state')


def main():
    root = Path(sys.argv[1])
    config = dict(dataset_root=str(root), cameras={'head': 'h', 'left_wrist': 'l', 'right_wrist': 'r'},
                  width=4, height=3, fps=30, min_episode_frames=1,
                  max_stream_age_s=.3, max_stream_skew_s=.2,
                  max_camera_age_s=.2, max_camera_skew_s=.1, max_frame_gap_s=.5,
                  startup_timeout_s=3., writer_finish_timeout_s=2., min_joint_delta_rad=0.,
                  episode_metadata={'source': 'policy_intervention', 'control_source': 'vr'})
    owner = RecorderProcess(config, camera_factory=FakeCameras)
    context = mp.get_context('spawn')
    stop = context.Event()
    feedback = context.Process(target=feed_robot, args=(stop,))
    feedback.start()
    try:
        assert owner.process.pid != os.getpid()
        wait_for(lambda: bool(owner.cameras.snapshot(time.monotonic())[0]))
        images, metadata = owner.cameras.snapshot(time.monotonic())
        assert all(meta['owner_pid'] == owner.process.pid for meta in metadata.values())
        assert images['head'].shape == (3, 4, 3)
        assert owner.request(True).result(timeout=5).success
        first_session = owner._expected_session
        wait_for(owner.ready_for_capture)
        before = owner.status.read()['frames']
        interval = sys.getswitchinterval()
        try:
            # Starve the parent GIL. Independent capture and feedback must continue.
            sys.setswitchinterval(1.)
            stalled_at = time.monotonic()
            until = stalled_at + .35
            while time.monotonic() < until:
                pass
        finally:
            sys.setswitchinterval(interval)
        wait_for(lambda: owner.status.read()['frames'] >= before + 5)
        assert owner.request(False).result(timeout=5).success
        assert not owner.ready_for_capture()
        saved = [p for p in root.glob('episode_*') if not p.name.endswith('.partial')]
        assert len(saved) == 1
        rows = [json.loads(line) for line in (saved[0] / 'frames.jsonl').read_text().splitlines()]
        assert len(rows) >= 6 and rows[0]['timestamp_s'] == 0.
        assert [r['frame_index'] for r in rows] == list(range(len(rows)))
        metadata = json.loads((saved[0] / 'episode.json').read_text())
        assert metadata['status'] == 'complete'
        assert sum(stalled_at <= metadata['start_monotonic_s'] + row['timestamp_s'] < until
                   for row in rows) >= 3

        assert owner.request(True).result(timeout=5).success
        assert owner._expected_session != first_session
        wait_for(owner.ready_for_capture)
        stop.set()
        feedback.join(3)
        wait_for(lambda: not owner.status.read().get('active', True))
        try:
            owner.ready_for_capture()
        except RuntimeError as exc:
            assert 'stale' in str(exc)
        else:
            raise AssertionError('Recorder abort was not propagated')
        aborted = root / owner._expected_session / 'episode.json'
        wait_for(lambda: json.loads(aborted.read_text())['status'] == 'aborted')
        owner.process.kill()
        owner.process.join(3)
        try:
            owner.cameras.snapshot(time.monotonic())
        except RuntimeError as exc:
            assert 'exited' in str(exc)
        else:
            raise AssertionError('Dead recorder camera source was accepted')
    finally:
        stop.set()
        feedback.join(3)
        if feedback.is_alive():
            feedback.kill()
            feedback.join(2)
        owner.close()
    assert not owner.process.is_alive()
    assert not owner._monitor.is_alive()
    diagnostics = [json.loads(line) for line in owner.diagnostics_path.read_text().splitlines()]
    assert any(row.get('capture', {}).get('session') for row in diagnostics)
    assert any(p.get('role') == 'bridge' for row in diagnostics for p in row['processes'])

    # Clean shutdown of an unsaved session must not create a completed episode.
    second = RecorderProcess(config, camera_factory=FakeCameras)
    assert second.request(True).result(timeout=5).success
    second.close()
    assert not second.process.is_alive()
    assert not second._monitor.is_alive()
    partials = list(root.glob('*.partial/episode.json'))
    assert len(partials) == 2
    assert all(json.loads(path.read_text())['status'] == 'aborted' for path in partials)
    print('independent_recorder_ok')


if __name__ == '__main__':
    main()
