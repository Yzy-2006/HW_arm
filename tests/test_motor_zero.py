import subprocess
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
JOINTS = ["shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow",
          "wrist_roll", "wrist_pitch", "wrist_yaw"]


@pytest.mark.parametrize("side", ["left", "right"])
@pytest.mark.parametrize("joint", range(1, 8))
def test_zero_motor_selects_one_joint_without_starting_hardware(side, joint):
    # Replace recursive make with echo so the recipe cannot launch a servo.
    result = subprocess.run(["make", "-s", "zero-motor", "MAKE=echo", f"SIDE={side}", f"JOINT={joint}"],
                            cwd=ROOT, capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
    assert f"motor_zero_joint:={side}_{JOINTS[joint - 1]}_joint" in result.stdout


@pytest.mark.parametrize("side,joint", [("", ""), ("both", "2"), ("left", "all"),
                                       ("left", "0"), ("right", "8"), ("left", "2 3")])
def test_zero_motor_rejects_missing_or_multiple_targets(side, joint):
    result = subprocess.run(["make", "-s", "zero-motor", "MAKE=echo", f"SIDE={side}", f"JOINT={joint}"],
                            cwd=ROOT, capture_output=True, text=True)
    assert result.returncode != 0
    assert "motor_zero_joint:=" not in result.stdout
