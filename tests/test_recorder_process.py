import multiprocessing as mp
import os
from pathlib import Path
import subprocess
import sys
import threading
from types import SimpleNamespace

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'deployment'))
from recorder_process import RecorderProcess, RemoteCameras, SharedFrames, SharedJson


def test_shared_images_preserve_metadata_and_do_not_alias():
    frames = SharedFrames(mp.get_context('spawn'), dict(cameras={'head': 'h'}, height=2, width=3))
    original = np.full((2, 3, 3), 7, np.uint8)
    metadata = {'head': {'frame_number': 5, 'receive_monotonic_s': 10.}}
    with pytest.raises(ValueError, match='Waiting'):
        frames.snapshot()
    frames.publish({'head': original}, metadata)
    original[:] = 99
    images, meta = frames.snapshot()
    assert np.all(images['head'] == 7) and meta == metadata
    images['head'][:] = 0
    assert np.all(frames.snapshot()[0]['head'] == 7)
    frames.metadata.write({'error': 'camera disconnected'})
    with pytest.raises(ValueError, match='disconnected'):
        frames.snapshot()


def test_remote_camera_age_uses_capture_timestamp():
    owner = SimpleNamespace(check_health=lambda: None,
        frames=SimpleNamespace(snapshot=lambda: ({}, {'head': {'receive_monotonic_s': 10.}})))
    camera = RemoteCameras(owner, {'max_camera_age_s': .2, 'max_camera_skew_s': .1})
    with pytest.raises(ValueError, match='stale'):
        camera.snapshot(10.3)
    camera.config['camera_timing_checks_enabled'] = False
    assert camera.snapshot(10.3)[1]['head']['receive_monotonic_s'] == 10.


def test_ready_requires_acknowledged_episode_and_propagates_abort():
    owner = RecorderProcess.__new__(RecorderProcess)
    owner._lock = threading.Lock()
    owner._expected_session = 'new'
    state = dict(session='old', active=True, ready=True)
    owner.check_health = lambda: state
    assert not owner.ready_for_capture()
    state['session'] = 'new'
    assert owner.ready_for_capture()
    state.update(active=False, error='disk failure')
    with pytest.raises(RuntimeError, match='disk failure'):
        owner.ready_for_capture()


def test_shared_metadata_rejects_nonfinite_and_oversized_payload():
    shared = SharedJson(mp.get_context('spawn'), capacity=32)
    with pytest.raises(ValueError):
        shared.write({'value': float('inf')})
    with pytest.raises(ValueError, match='capacity'):
        shared.write({'value': 'a' * 100})
    shared.write({'active': False})
    assert shared.read() == {'active': False}


def test_independent_recorder_process_and_parent_stall(tmp_path):
    env = dict(os.environ, ROS_DOMAIN_ID='217', ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST',
               ROS_LOG_DIR=str(tmp_path / 'ros_logs'))
    result = subprocess.run([sys.executable, str(ROOT / 'tests/recorder_process_runtime.py'),
                             str(tmp_path / 'episodes')], env=env, capture_output=True, text=True, timeout=40)
    assert result.returncode == 0, result.stderr + result.stdout
    assert 'independent_recorder_ok' in result.stdout
    assert 'Destroyable' not in result.stderr and 'never retrieved' not in result.stderr
