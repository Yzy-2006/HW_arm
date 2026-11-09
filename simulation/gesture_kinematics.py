"""Bounded Cartesian IK for the same model displayed by model_viewer."""

import xml.etree.ElementTree as ET
from pathlib import Path
import sys

import numpy as np
import pinocchio as pin
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
import yaml

from robot_model import ROOT, GRASP_POINTS, OPEN_FRACTION, CLOSED_FRACTION, robot_description

sys.path.insert(0, str(ROOT / "scripts"))
from calibrate_arm_zero import ARM_JOINT_KEYS, DEFAULT_CONFIG


class GestureRobot:
    def __init__(self, joint_speed=0.8, zero_config=DEFAULT_CONFIG):
        self.model = pin.buildModelFromXML(ET.tostring(robot_description(), encoding="unicode"))
        self.data = self.model.createData()
        self.q = pin.neutral(self.model)
        self.names = []
        self.indices = {}
        self.fingers = {}
        self.frames = {}
        self.offsets = {}
        self.joint_speed = joint_speed
        initial = yaml.safe_load(Path(zero_config).read_text())
        for side, short, offset in zip(("left", "right"), ("l", "r"), GRASP_POINTS):
            names = [f"arm_{short}_joint_{index}" for index in range(1, 8)]
            self.indices[side] = np.array([self.model.joints[self.model.getJointId(n)].idx_q for n in names])
            finger = f"arm_{short}_joint_finger"
            self.fingers[side] = self.model.joints[self.model.getJointId(finger)].idx_q
            self.names.extend(names + [finger])
            self.frames[side] = self.model.getFrameId(f"arm_{short}_link_gripper")
            self.offsets[side] = pin.SE3(np.eye(3), np.array(offset))
            # Use the hardware calibration's named joint order, not YAML key order.
            try:
                angles = np.array([initial[f"{side}_arm"][key] for key in ARM_JOINT_KEYS[side]], dtype=float)
            except (KeyError, TypeError, ValueError) as error:
                raise ValueError(f"Invalid {side} arm joint zero in {zero_config}") from error
            indices = self.indices[side]
            if (not np.isfinite(angles).all()
                    or np.any(angles < self.model.lowerPositionLimit[indices])
                    or np.any(angles > self.model.upperPositionLimit[indices])):
                raise ValueError(f"{side} arm joint zero must be finite and within model limits: {zero_config}")
            self.q[indices] = angles
            self.q[self.fingers[side]] = self.finger_angle(side, OPEN_FRACTION)
        self.publish_indices = [self.model.joints[self.model.getJointId(n)].idx_q for n in self.names]
        self.home = self.q.copy()
        self.closed = dict.fromkeys(("left", "right"), False)
        self.homing = False

    def finger_angle(self, side, fraction):
        index = self.fingers[side]
        endpoint = self.model.lowerPositionLimit[index] if side == "left" else self.model.upperPositionLimit[index]
        return float(fraction * endpoint)

    def pose(self, side, q=None):
        pin.framesForwardKinematics(self.model, self.data, self.q if q is None else q)
        return self.data.oMf[self.frames[side]] * self.offsets[side]

    def toggle_gripper(self, side):
        self.closed[side] = not self.closed[side]

    def start_home(self):
        self.homing = True

    def tick(self, dt):
        if self.homing:
            # Return both arms, preserving the current gripper commands.
            for indices in self.indices.values():
                self.q[indices] += np.clip(self.home[indices] - self.q[indices],
                                           -self.joint_speed * dt, self.joint_speed * dt)
            self.homing = any(np.max(np.abs(self.q[i] - self.home[i])) > 1e-6 for i in self.indices.values())
        for side, index in self.fingers.items():
            target = self.finger_angle(side, CLOSED_FRACTION if self.closed[side] else OPEN_FRACTION)
            self.q[index] += np.clip(target - self.q[index], -1.5 * dt, 1.5 * dt)

    def move(self, side, dp, dr, dt):
        if self.homing or (np.linalg.norm(dp) < 1e-10 and np.linalg.norm(dr) < 1e-10):
            return True
        indices = self.indices[side]
        current = self.pose(side)
        target = pin.SE3(Rotation.from_rotvec(dr).as_matrix() @ current.rotation,
                        current.translation + dp)
        seed = self.q[indices].copy()
        candidate = self.q.copy()

        def residual(angles):
            candidate[indices] = angles
            pose = self.pose(side, candidate)
            return np.r_[pose.translation - target.translation,
                         0.15 * pin.log3(target.rotation @ pose.rotation.T),
                         0.001 * (angles - seed)]

        step = self.joint_speed * min(dt, 0.1)
        lower = np.maximum(self.model.lowerPositionLimit[indices], seed - step)
        upper = np.minimum(self.model.upperPositionLimit[indices], seed + step)
        solution = least_squares(residual, seed, bounds=(lower, upper), max_nfev=18,
                                 ftol=1e-5, xtol=1e-5, gtol=1e-6)
        if not np.isfinite(solution.x).all() or np.linalg.norm(residual(solution.x)[:6]) > np.linalg.norm(residual(seed)[:6]) + 1e-8:
            return False
        self.q[indices] = solution.x
        return True

    def snapshot(self):
        poses = {}
        for side in self.indices:
            pose = self.pose(side)
            poses[side] = {"position": pose.translation.tolist(),
                           "orientation": Rotation.from_matrix(pose.rotation).as_quat().tolist()}
        return {"joint_names": self.names, "joint_positions": self.q[self.publish_indices].tolist(),
                "tcp_world": poses, "gripper_closed": self.closed.copy(), "homing": self.homing}
