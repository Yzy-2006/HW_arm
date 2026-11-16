import os
from pathlib import Path
import subprocess
import sys
import pytest


ROOT = Path(__file__).resolve().parents[1]


@pytest.mark.parametrize("workflow", ["shutdown", "roundtrip"])
def test_worker_and_bridge_shutdown_leave_single_threaded_vr_recording(tmp_path, workflow):
    env = dict(os.environ, ROS_DOMAIN_ID="220", ROS_AUTOMATIC_DISCOVERY_RANGE="LOCALHOST",
               ROS_LOG_DIR=str(tmp_path / "ros_logs"))
    result = subprocess.run([sys.executable, str(ROOT / "tests/policy_intervention_runtime.py"), str(tmp_path), workflow],
        env=env, capture_output=True, text=True, timeout=25)
    assert result.returncode == 0, result.stdout + result.stderr
    assert "workflow_passed" in result.stdout
