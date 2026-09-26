import subprocess
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def make_dry_run(target):
    return subprocess.run(
        ["make", "-n", target],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )


def test_removed_one_shot_launch_targets_are_unavailable():
    for target in ("teleop", "experiment"):
        result = make_dry_run(target)
        assert result.returncode == 0
        assert f"make {target} has been removed" in result.stdout
        assert "exit 2" in result.stdout


def test_removed_force_commands_are_not_make_targets():
    for target in ("force", "tare-force"):
        result = make_dry_run(target)
        assert result.returncode != 0


def test_force_sensor_driver_source_is_removed():
    assert not (ROOT / "ros2_ws" / "src" / "encos_force_sensor").exists()


def test_recorder_configuration_and_source_do_not_define_wrench_streams():
    config = (ROOT / "config" / "recording.yaml").read_text()
    recorder = (ROOT / "collection" / "record.py").read_text()
    assert "force_topics:" not in config
    assert "WrenchStamped" not in recorder
    assert 'config.get("force_topics"' not in recorder


def test_vr_and_robot_control_have_no_force_feedforward_topic():
    vr = (ROOT / "teleop/VR_controller/vr_pose_increment_publisher.py").read_text()
    robot = (ROOT / "ros2_ws/src/robot_control_hw/src/robot_control/trajProcessWrap.cpp").read_text()
    assert "right_arm_force_feedforward" not in vr
    assert "right_arm_force_feedforward" not in robot


def test_dual_arm_control_contains_no_sensor_or_admittance_control_path():
    source = (ROOT / "ros2_ws/src/dual_arm_control_hw/src/dual_arm_control/msgPublish.cpp").read_text()
    header = (ROOT / "ros2_ws/src/dual_arm_control_hw/include/dual_arm_control/msgPublish.h").read_text()
    assert "/force_sensor/" not in source
    assert "lArm_TorSensor_msg" not in source
    assert "rArm_TorSensor_msg" not in source
    assert "AdmImpCtlCore" not in source
    assert "WrenchStamped" not in header
    assert not (ROOT / "ros2_ws/src/ctl_msgs_hw/config/admittanceImpPara.yaml").exists()


def test_user_facing_runtime_docs_do_not_advertise_removed_force_workflow():
    paths = [
        ROOT / "README.md",
        ROOT / "docs/REAL_ROBOT_EXPERIMENT.md",
        ROOT / "docs/VR_TELEOP_CHECK.md",
        ROOT / "docs/DEPENDENCIES.md",
        ROOT / "docs/copied_sources.json",
    ]
    text = "\n".join(path.read_text() for path in paths)
    assert "make force" not in text
    assert "tare-force" not in text
    assert "encos_force_sensor" not in text
    assert not (ROOT / "docs/FORCE_SENSOR.md").exists()
