"""Offline FK using the same URDF, frames and joint coordinates as arm control."""

from pathlib import Path
import math

import yaml


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONTROLLER_CONFIG = ROOT / "ros2_ws/src/ctl_msgs_hw/config/controllerConfig.yaml"


class ArmKinematics:
    def __init__(self, controller_config: Path = DEFAULT_CONTROLLER_CONFIG):
        import pinocchio as pin

        self.pin = pin
        config = yaml.safe_load(controller_config.read_text())
        loop = config["TransLoopCtlInte"]["dual_arm_ctl_node_loop_ctl"]
        urdf = ROOT / "ros2_ws/src/dual_arm_hw/urdf" / loop["urdf_path_option"]
        self.model = pin.buildModelFromUrdf(str(urdf))
        self.frames = {}
        self.joints = {}
        for side, prefix in (("left", "l"), ("right", "r")):
            base = loop[f"base_link_{prefix}Arm"]
            tip = loop[f"tip_link_{prefix}Arm"]
            if not all(self.model.existFrame(name) for name in (base, tip)):
                raise ValueError(f"Missing {side} arm base or tip frame in URDF")
            self.frames[side] = (self.model.getFrameId(base), self.model.getFrameId(tip))
            self.joints[side] = []
            for index in range(1, 8):
                name = f"arm_{prefix}_joint_{index}"
                if not self.model.existJointName(name):
                    raise ValueError(f"Missing URDF joint: {name}")
                joint = self.model.joints[self.model.getJointId(name)]
                if joint.nq != 1:
                    raise ValueError(f"Expected a single-angle joint: {name}")
                self.joints[side].append(joint.idx_q)

    def pose(self, side: str, angles) -> list[float]:
        if side not in self.joints or len(angles) != 7:
            raise ValueError("Expected a known side and seven joint angles")
        if not all(math.isfinite(value) for value in angles):
            raise ValueError("Joint angles must be finite")
        q = self.pin.neutral(self.model)
        for index, value in zip(self.joints[side], angles, strict=True):
            q[index] = value
        data = self.model.createData()
        self.pin.framesForwardKinematics(self.model, data, q)
        base, tip = self.frames[side]
        transform = data.oMf[base].inverse() * data.oMf[tip]
        pose = [*transform.translation, *self.pin.rpy.matrixToRpy(transform.rotation)]
        if not all(math.isfinite(value) for value in pose):
            raise ValueError("Forward kinematics returned a non-finite pose")
        return [float(value) for value in pose]
