#!/usr/bin/env python3
"""Refresh the allowlisted runtime snapshot; delivery docs are maintained separately."""

from pathlib import Path
import shutil
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
DEST = ROOT / "newton_towel_sim"


def main():
    paths = [
        *(Path("simulation") / name for name in
          ("towel_scene.py", "robot_model.py", "vr_control.py", "vr_input.py", "scene.yaml")),
        Path("scripts/check_newton.py"),
        Path("teleop/VR_controller/pose_mapping.py"),
        Path("teleop/VR_controller/config/gripper_pd.yaml"),
        Path("teleop/LICENSE"),
        *(Path("teleop/src/televuer") / name for name in ("__init__.py", "televuer.py", "tv_wrapper.py")),
        *(Path("tests") / name for name in
          ("test_newton_scene.py", "test_newton_vr.py", "check_sim_vr_input.py")),
        Path("dual_arm/urdf/dual_arm.urdf"),
    ]
    robot = ET.parse(ROOT / "dual_arm/urdf/dual_arm.urdf").getroot()
    for mesh in robot.findall(".//mesh"):
        uri = mesh.get("filename")
        if not uri.startswith("package://dual_arm/"):
            raise ValueError(f"Unexpected mesh URI: {uri}")
        path = Path("dual_arm") / uri.removeprefix("package://dual_arm/")
        if not (ROOT / path).resolve().is_relative_to(ROOT / "dual_arm"):
            raise ValueError(f"Mesh outside asset directory: {path}")
        paths.append(path)
    for relative in sorted(set(paths)):
        source, target = ROOT / relative, DEST / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, target)
    print(f"Exported {len(set(paths))} runtime/assets files to {DEST}")


if __name__ == "__main__":
    main()
