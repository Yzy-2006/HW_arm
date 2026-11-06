import os
import subprocess
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]


@pytest.mark.parametrize("symlink_install", [False, True])
def test_build_sets_ethercat_capabilities_after_build_succeeds(tmp_path, symlink_install):
    """A rebuilt servo binary must receive its required EtherCAT capabilities."""
    sandbox_root = tmp_path / "robot"
    venv = sandbox_root / ".venv-robot"
    (venv / "bin").mkdir(parents=True)
    (venv / "bin" / "activate").write_text("")

    scripts = sandbox_root / "scripts"
    scripts.mkdir()
    build_marker = tmp_path / "build-complete"
    (scripts / "build.sh").write_text(
        "#!/usr/bin/env bash\n"
        "set -eu\n"
        'touch "$TEST_BUILD_MARKER"\n'
    )
    (scripts / "env.sh").write_text("#!/usr/bin/env bash\n")
    (scripts / "build.sh").chmod(0o755)

    servo_node = sandbox_root / "ros2_ws/install/servo_control_hw/lib/servo_control_hw/servo_node"
    servo_node.parent.mkdir(parents=True)
    if symlink_install:
        servo_binary = sandbox_root / "ros2_ws/build/servo_control_hw/servo_node"
        servo_binary.parent.mkdir(parents=True)
        servo_node.symlink_to(servo_binary)
    else:
        servo_binary = servo_node
    servo_binary.write_text("placeholder")
    servo_binary.chmod(0o755)

    fake_bin = tmp_path / "bin"
    fake_bin.mkdir()
    setcap_marker = tmp_path / "setcap-arguments"
    fake_sudo = fake_bin / "sudo"
    fake_sudo.write_text(
        "#!/usr/bin/env bash\n"
        "set -eu\n"
        'test -f "$TEST_BUILD_MARKER"\n'
        'printf "%s\\n" "$*" > "$TEST_SETCAP_MARKER"\n'
    )
    fake_sudo.chmod(0o755)

    environment = os.environ.copy()
    environment.update(
        {
            "PATH": f"{fake_bin}:{environment['PATH']}",
            "TEST_BUILD_MARKER": str(build_marker),
            "TEST_SETCAP_MARKER": str(setcap_marker),
        }
    )
    result = subprocess.run(
        [
            "make",
            "-f",
            str(ROOT / "Makefile"),
            f"ROOT={sandbox_root}",
            f"VENV={venv}",
            "build",
        ],
        capture_output=True,
        text=True,
        env=environment,
    )

    assert result.returncode == 0, result.stderr
    assert setcap_marker.read_text().split() == [
        "setcap",
        "cap_net_raw,cap_net_admin+ep",
        str(servo_binary),
    ]
