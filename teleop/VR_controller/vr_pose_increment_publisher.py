#!/usr/bin/env python3
"""Dual-arm VR teleoperation for towel_task.
Left/right squeeze toggle the matching left/right gripper.
Left B toggles recording, left A discards, right A returns arms home, right B toggles VR sensitivity.
Gripper encoder endpoints and motion time: config/grippers.yaml (calibration required).
Motion mapping: config/gripper_pd.yaml.
/towel/gripper_state: [measured_left, measured_right, commanded_left, commanded_right],
0=open, 1=closed, -1=unavailable. Position targets use /gripper/ctl.
"""

import argparse
import os
import time
from pathlib import Path
from multiprocessing import shared_memory

import numpy as np
from pose_mapping import (
    matrix_to_quaternion, quaternion_to_euler_zyx, _head_yaw_rotation, _head_rotation,
    _rotate_xy, _limit_increment, _conservative_deadband,
    robot_frame_position_delta, robot_frame_orientation_delta,
)

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from geometry_msgs.msg import Vector3
from std_msgs.msg import Bool, Float64MultiArray, Int8MultiArray
from std_srvs.srv import SetBool, Trigger
from ctl_msgs_hw.msg import GripperCtl
from ament_index_python.packages import get_package_share_directory

from televuer import TeleVuerWrapper
from grippers import DualGrippers, load_config as load_dual_gripper_config

try:
    import yaml
except ImportError:  # pragma: no cover
    yaml = None


# 左手柄 B 键：开始/结束录制。
LEFT_RECORD_BUTTON_ATTR = "left_bButton"
# 左手柄 X 键：丢弃当前录制（不保存）。物理 X 键由 televuer 暴露为 left_aButton。
LEFT_DISCARD_BUTTON_ATTR = "left_aButton"


def format_controller_debug(snapshot):
    """Render a concise, human-readable controller-input diagnostic line."""
    def yes_no(value):
        return "yes" if value else "no"

    return (
        f"XR controller: events={int(snapshot.get('events', 0))}, "
        f"errors={int(snapshot.get('errors', 0))}; "
        "left("
        f"seen={yes_no(snapshot.get('left_seen', False))}, "
        f"trigger={yes_no(snapshot.get('left_trigger', False))}/"
        f"{float(snapshot.get('left_trigger_value', 0.0)):.2f}, "
        f"left_a={yes_no(snapshot.get('left_a', False))}, "
        f"left_b={yes_no(snapshot.get('left_b', False))}); "
        "right("
        f"seen={yes_no(snapshot.get('right_seen', False))}, "
        f"trigger={yes_no(snapshot.get('right_trigger', False))}/"
        f"{float(snapshot.get('right_trigger_value', 0.0)):.2f}, "
        f"right_a={yes_no(snapshot.get('right_a', False))}, "
        f"right_b={yes_no(snapshot.get('right_b', False))})"
    )


def run_vr_tick(node, tv_wrapper, debug_controller_input=False):
    """Process one VR frame; debug mode intentionally emits no control commands."""
    telemetry = tv_wrapper.get_motion_state_data()
    if debug_controller_input:
        node.report_controller_debug(tv_wrapper.controller_debug_snapshot())
        return
    node.publish_increment(telemetry)
    node.update_gripper(telemetry)


# -----------------------------------------------------------------------------
# 笛卡尔空间零位配置（desc_zeropos.yaml）
# -----------------------------------------------------------------------------

# 零位配置文件路径（可被环境变量 DESC_ZEROPOS_YAML 覆盖）
ZERO_POS_YAML_DEFAULT = str(
    Path(__file__).resolve().parents[2] / "ros2_ws/src/ctl_msgs_hw/config/desc_zeropos.yaml"
)
ZERO_POS_YAML_FALLBACKS = ()


def load_zero_position():
    """读取笛卡尔零位 [x, y, z, roll, pitch, yaw]（顺序与 desc_zeropos.yaml 一致）。

    返回 {"left": np.ndarray(6), "right": np.ndarray(6), "path": str}；
    找不到文件或解析失败时返回 None。
    """
    if yaml is None:
        return None

    candidates = [os.environ.get("DESC_ZEROPOS_YAML"), ZERO_POS_YAML_DEFAULT, *ZERO_POS_YAML_FALLBACKS]
    path = next((c for c in candidates if c and os.path.isfile(c)), None)
    if path is None:
        return None

    try:
        with open(path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f)
    except Exception as e:
        print(f"[WARN] 读取零位配置失败 {path}: {e}")
        return None

    def _extract(side):
        arm = data.get(f"{side}_arm", {})
        pos = arm.get("position", {})
        ori = arm.get("orientation", {})
        return np.array([
            float(pos.get("x", 0.0)),
            float(pos.get("y", 0.0)),
            float(pos.get("z", 0.0)),
            float(ori.get("roll", 0.0)),
            float(ori.get("pitch", 0.0)),
            float(ori.get("yaw", 0.0)),
        ])

    try:
        return {"left": _extract("left"), "right": _extract("right"), "path": path}
    except Exception as e:
        print(f"[WARN] 解析零位配置失败: {e}")
        return None


# -----------------------------------------------------------------------------
# 夹爪 PD 控制配置（config/gripper_pd.yaml）
# -----------------------------------------------------------------------------

# 配置文件路径（可被环境变量 GRIPPER_PD_YAML 覆盖）
GRIPPER_PD_YAML_DEFAULT = str(Path(__file__).resolve().parent / "config/gripper_pd.yaml")

# 默认值：文件缺失/字段缺失时兜底
GRIPPER_PD_DEFAULTS = {
    # 手臂位姿增量的基准坐标系：
    #   "head" —— 以头部为基准（抵消头部/身体平移和水平转向，仅手柄相对头部的移动生效，安全默认）
    #   "room" —— 以房间（世界）坐标系为基准（叠加头部/身体平移，手臂做房间内绝对运动）
    "reference_frame": "head",
    # Cartesian teleoperation gains.  Keep these explicit so there is no
    # hidden scale factor in the ROS2/C++ subscriber.
    "position_scale": 1.0,
    "orientation_scale": 1.0,
    "fast_position_scale": 1.8,
    "fast_orientation_scale": 2.0,
    "fast_position_speed_mps": 1.2,
    "fast_orientation_speed_radps": 6.0,
    "fine_position_scale": 0.45,
    "fine_orientation_scale": 0.60,
    "fine_position_speed_mps": 0.30,
    "fine_orientation_speed_radps": 1.5,
    # Per-axis calibration in robot coordinates [x=front, y=left, z=up].
    "position_axis_scale": [1.0, 1.0, 1.0],
    "position_axis_sign": [1.0, 1.0, 1.0],
    # Horizontal-plane calibration: rotate the commanded translation about the
    # robot vertical axis (z=up).  Compensates for a constant angular offset
    # between the hand's forward/left and the robot's front/left, which makes
    # straight hand motion come out diagonal.  Positive = counterclockwise
    # seen from above.  Unit: degrees.
    "translation_yaw_offset_deg": 0.0,
    # Increment streams are direct by default.  An EMA on increments creates
    # residual motion after a direction reversal because the C++ controller
    # integrates every published sample.
    "position_filter_alpha": 1.0,
    "orientation_filter_alpha": 1.0,
}


def load_gripper_config():
    """读取夹爪 PD 配置，返回 dict。找不到文件/字段时用默认值。"""
    cfg = dict(GRIPPER_PD_DEFAULTS)
    path = os.environ.get("GRIPPER_PD_YAML", GRIPPER_PD_YAML_DEFAULT)
    if yaml is None or not os.path.isfile(path):
        return cfg
    try:
        with open(path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
        for k, default in cfg.items():
            if k not in data:
                continue
            v = data[k]
            if isinstance(default, str):
                # 字符串型字段（如基准坐标系 reference_frame）直接透传
                cfg[k] = str(v)
            elif isinstance(default, (list, tuple)):
                # 列表型字段（如六维前馈力）逐元素转 float
                cfg[k] = [float(x) for x in v]
            else:
                cfg[k] = float(v)
    except Exception as e:
        print(f"[WARN] 读取夹爪 PD 配置失败 {path}: {e}（使用默认值）")
    return cfg


# -----------------------------------------------------------------------------
# 四元数 / 欧拉角工具（取自 demo/vr_ros2_publisher_webtrc.py，无外部依赖）
# -----------------------------------------------------------------------------

# -----------------------------------------------------------------------------
# ROS2 发布节点
# -----------------------------------------------------------------------------

class VRPoseIncrementPublisher(Node):
    """订阅(内部)VR 数据，逐帧计算位姿增量，并按扳机门控发布到 ROS2 话题。"""

    def __init__(self):
        super().__init__('vr_pose_increment_publisher')

        # Increment topics are a latest-sample stream.  A deep reliable queue
        # replays stale per-frame deltas after a scheduling hiccup, producing
        # the exact delayed burst seen during free-space teleoperation.
        vr_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
        )
        # 位置增量
        self.left_pos_delta_pub = self.create_publisher(Vector3, '/vr/left_arm_delta', vr_qos)
        self.right_pos_delta_pub = self.create_publisher(Vector3, '/vr/right_arm_delta', vr_qos)
        # 姿态（欧拉角）增量
        self.left_euler_delta_pub = self.create_publisher(Vector3, '/vr/left_arm_euler_delta', vr_qos)
        self.right_euler_delta_pub = self.create_publisher(Vector3, '/vr/right_arm_euler_delta', vr_qos)

        # 上一帧状态
        self.prev_left_pose = None
        self.prev_right_pose = None
        self.prev_head_pose = None
        # At startup the user faces the robot.  This establishes a stable map
        # from head-local controller axes to the fixed Cartesian IK axes.
        self.head_orientation_reference = np.eye(3)
        self.prev_left_trigger = False
        self.prev_right_trigger = False
        self.first_frame = True

        # Tracking noise thresholds and hard limits on raw frame increments.
        self.position_deadband_m = 0.0004
        self.orientation_deadband_rad = 0.002
        self.max_position_delta_m = 0.05
        self.max_orientation_delta_rad = 0.20
        self._left_pos_residual = np.zeros(3)
        self._right_pos_residual = np.zeros(3)
        self._left_rot_residual = np.zeros(3)
        self._right_rot_residual = np.zeros(3)
        # Downstream integrates displacements once per message.  Elapsed time
        # controls only the speed limit, not the displacement gain.
        self.target_period_s = 1.0 / 50.0
        self._last_sample_time = None
        self.max_position_speed_mps = 0.75
        self.max_orientation_speed_radps = 4.0

        # 笛卡尔零位（仅作日志/参考；真正的回零目标 zeropos.yaml 由下游
        # robot_control_hw 加载并在关节空间执行，这里不再做笛卡尔空间复位）
        self.zero_pos = load_zero_position()
        if self.zero_pos is not None:
            self.get_logger().info(
                f"已加载笛卡尔零位 {self.zero_pos['path']}: "
                f"left={self.zero_pos['left']} right={self.zero_pos['right']}"
            )
        else:
            self.get_logger().warning("未找到笛卡尔零位配置文件（仅影响日志，不影响回零）")

        # A 键回零触发发布器：按下右手柄 A 键（right_aButton）上升沿发布 True，
        # 由下游 robot_control_hw 按 controllerConfig.yaml 的 vrJointReturn
        # 配置，以平滑关节空间轨迹回归参考目标角度。
        self.zero_return_pub = self.create_publisher(Bool, "/vr/zero_return_cmd", 10)

        # A 键上一次状态（用于检测「按下」上升沿）
        self.prev_a_button = False
        self.prev_b_button = False
        self.precision_mode = False
        # ---------- 遥操作坐标映射配置 ----------
        gripper_cfg = load_gripper_config()
        # 手臂位姿增量的基准坐标系：room（房间/世界坐标系）或 head（头部坐标系）
        self.reference_frame = str(gripper_cfg["reference_frame"]).strip().lower()
        if self.reference_frame not in ("room", "head"):
            self.get_logger().warning(
                f"未知基准坐标系 '{self.reference_frame}'，回退为 'head'（仅支持 room / head）"
            )
            self.reference_frame = "head"
        self.position_scale = float(gripper_cfg["position_scale"])
        self.orientation_scale = float(gripper_cfg["orientation_scale"])
        self.fast_position_scale = float(gripper_cfg["fast_position_scale"])
        self.fast_orientation_scale = float(gripper_cfg["fast_orientation_scale"])
        self.fast_position_speed_mps = float(gripper_cfg["fast_position_speed_mps"])
        self.fast_orientation_speed_radps = float(gripper_cfg["fast_orientation_speed_radps"])
        self.fine_position_scale = float(gripper_cfg["fine_position_scale"])
        self.fine_orientation_scale = float(gripper_cfg["fine_orientation_scale"])
        self.fine_position_speed_mps = float(gripper_cfg["fine_position_speed_mps"])
        self.fine_orientation_speed_radps = float(gripper_cfg["fine_orientation_speed_radps"])
        self.position_axis_scale = np.asarray(gripper_cfg["position_axis_scale"], dtype=float)
        self.position_axis_sign = np.asarray(gripper_cfg["position_axis_sign"], dtype=float)
        if self.position_axis_scale.shape != (3,) or self.position_axis_sign.shape != (3,):
            self.get_logger().warning("position_axis_scale/sign 必须是 3 个元素，已回退为 [1,1,1]")
            self.position_axis_scale = np.ones(3)
            self.position_axis_sign = np.ones(3)
        self.position_filter_alpha = float(np.clip(gripper_cfg["position_filter_alpha"], 0.0, 1.0))
        self.orientation_filter_alpha = float(np.clip(gripper_cfg["orientation_filter_alpha"], 0.0, 1.0))
        if self.position_scale <= 0.0 or self.orientation_scale <= 0.0:
            self.get_logger().warning("position_scale/orientation_scale 必须为正数，已回退为 1.0")
            self.position_scale = max(self.position_scale, 1.0)
            self.orientation_scale = max(self.orientation_scale, 1.0)
        if np.any(self.position_axis_scale < 0.0) or not np.all(np.isfinite(self.position_axis_scale)):
            self.get_logger().warning("position_axis_scale 必须为非负有限数，已回退为 [1,1,1]")
            self.position_axis_scale = np.ones(3)
        self.position_axis_sign = np.where(self.position_axis_sign < 0.0, -1.0, 1.0)

        # 水平面平移标定角（度）：绕 z（上）旋转下发平移，补偿手坐标系与机器人
        # 坐标系之间的固定偏航角（表现为直线运动变成斜线）。
        self.translation_yaw_offset_rad = np.deg2rad(float(gripper_cfg["translation_yaw_offset_deg"]))
        if not np.isfinite(self.translation_yaw_offset_rad):
            self.translation_yaw_offset_rad = 0.0

        # EMA state is reset whenever tracking is re-anchored.
        self._left_pos_filter = np.zeros(3)
        self._right_pos_filter = np.zeros(3)
        self._left_rot_filter = np.zeros(3)
        self._right_rot_filter = np.zeros(3)
        self.gripper_ctl_pub = self.create_publisher(
            GripperCtl, "/gripper/ctl", 1)
        self.gripper_fdb_sub = self.create_subscription(
            Float64MultiArray, "/gripper/fdb", self._gripper_fdb_callback, 10
        )
        controller_path = Path(get_package_share_directory('ctl_msgs_hw')) / 'config/controllerConfig.yaml'
        self.dual_grippers = DualGrippers(load_dual_gripper_config(controller_path=controller_path))
        self.gripper_state_pub = self.create_publisher(Int8MultiArray, "/towel/gripper_state", 10)
        self.prev_left_gripper_button = False
        self.prev_right_gripper_button = False

        # 左手柄 B 键上一次状态（用于录制上升沿）。
        self.prev_record_button = False
        # 左手柄 X 键上一次状态（用于丢弃录制上升沿）
        self.prev_discard_button = False
        self._last_controller_debug_log = 0.0

        # 左手柄 B 键录制切换；录制器未启动时不阻塞遥操作。
        self.record_service_name = "/towel/record"
        self.record_service_client = self.create_client(SetBool, self.record_service_name)
        self.recording_active = False
        self.recording_state_sub = self.create_subscription(
            Bool, "/towel/recording", self._recording_state_callback, 10
        )
        self._record_service_warned = False
        self._record_request_pending = False

        # 左手柄 A 键丢弃录制（停止当前 episode 且不保存）。
        self.discard_service_name = "/towel/discard"
        self.discard_service_client = self.create_client(Trigger, self.discard_service_name)
        self._discard_service_warned = False

        self.get_logger().info("双夹爪位控：左/右 squeeze 分别切换左/右夹爪；未标定侧不更新位置目标")
        self.get_logger().info(
            f"手臂位姿增量基准坐标系：{self.reference_frame} "
            f"（room=房间/世界坐标系，head=头部坐标系），"
            f"VR 模式=快速，右 B 切换精细；默认位置缩放={self.position_scale:.2f}，姿态缩放={self.orientation_scale:.2f}"
        )
        if abs(self.translation_yaw_offset_rad) > 1e-6:
            self.get_logger().info(
                f"水平面平移标定角：{float(gripper_cfg['translation_yaw_offset_deg']):.1f} deg（绕 z 旋转）"
            )
        self.get_logger().info(
            "VR 初始化完成：左右 squeeze 控制对应夹爪；左 B 录制、左 A 丢弃、右 A 回零、右 B 切换灵敏度"
        )

    def report_controller_debug(self, snapshot):
        """Log raw XR input at a readable rate; this method publishes nothing."""
        now = time.monotonic()
        if now - self._last_controller_debug_log < 0.5:
            return
        self._last_controller_debug_log = now
        self.get_logger().info(format_controller_debug(snapshot))

    def publish_increment(self, teleData):
        """逐帧计算位姿增量并发布。未扣扳机时发布零增量。"""
        if teleData is None or teleData.head_pose is None:
            return

        state = teleData.tele_state
        now = time.monotonic()
        sample_dt = self.target_period_s
        stale_sample = False
        if self._last_sample_time is not None:
            raw_dt = now - self._last_sample_time
            # If the WebRTC/Python loop was blocked for too long, discard the
            # old displacement instead of replaying it as a delayed jump.
            stale_sample = raw_dt > 0.080
            sample_dt = float(np.clip(raw_dt, 0.010, 0.250))
        self._last_sample_time = now
        left_pose = teleData.left_arm_pose
        right_pose = teleData.right_arm_pose
        head_pose = teleData.head_pose

        # 首帧只记录，不发布（增量需要上一帧做基准）
        if self.first_frame:
            self.prev_left_pose = left_pose.copy()
            self.prev_right_pose = right_pose.copy()
            self.prev_head_pose = head_pose.copy()
            self.head_orientation_reference = _head_yaw_rotation(head_pose)
            self._left_pos_residual.fill(0.0)
            self._right_pos_residual.fill(0.0)
            self._left_rot_residual.fill(0.0)
            self._right_rot_residual.fill(0.0)
            self._left_pos_filter.fill(0.0)
            self._right_pos_filter.fill(0.0)
            self._left_rot_filter.fill(0.0)
            self._right_rot_filter.fill(0.0)
            self.first_frame = False
            return

        # 扳机状态（扣动扳机 = True）
        left_trigger = bool(state.left_trigger_state) if state is not None else False
        right_trigger = bool(state.right_trigger_state) if state is not None else False

        self.prev_left_trigger = left_trigger
        self.prev_right_trigger = right_trigger

        # ---- A 键触发关节空间回零（由下游 robot_control_hw 执行）----
        # 物理 A 键位于右手柄：WebXR gamepad button[4]，vuer 将其映射为 right_aButton。
        # 在「按下」的上升沿发布一次 /vr/zero_return_cmd=True，由 robot_control_hw
        # 以 controllerConfig.yaml 配置的平滑关节轨迹从当前反馈角回归 zeropos.yaml。
        a_button = bool(state.right_aButton) if state is not None else False
        if a_button and not self.prev_a_button:
            self.get_logger().info("检测到 A 键按下，发布关节空间回零触发指令")
            self.zero_return_pub.publish(Bool(data=True))
        self.prev_a_button = a_button
        b_button = bool(getattr(state, "right_bButton", False))
        if b_button and not self.prev_b_button and not a_button:
            self.precision_mode = not self.precision_mode
            self._left_pos_residual.fill(0.0)
            self._right_pos_residual.fill(0.0)
            self._left_rot_residual.fill(0.0)
            self._right_rot_residual.fill(0.0)
            self._left_pos_filter.fill(0.0)
            self._right_pos_filter.fill(0.0)
            self._left_rot_filter.fill(0.0)
            self._right_rot_filter.fill(0.0)
            self.get_logger().info(
                f"检测到右 B 键按下，切换 VR {'精细微调' if self.precision_mode else '快速移动'}模式"
            )
        self.prev_b_button = b_button

        # ---- 基准坐标系选择 ----
        # tv_wrapper 返回的 left_arm_pose / right_arm_pose 已扣除头部平移（头部相对基准），
        # 而 head_pose 是头部在房间（世界）坐标系中的位置。因此：
        #   房间坐标系下的手臂位置 = 手臂位置 + 头部位置
        #   房间坐标下的位置增量 = (手臂增量) + (头部平移增量)
        #   head 模式：不叠加头部平移（抵消身体移动，仅手柄相对头部移动生效）
        #   room 模式：叠加头部平移（手臂跟随身体在房间内的绝对运动）
        head_delta = np.zeros(3)
        if self.reference_frame == "room":
            head_delta = head_pose[:3, 3] - self.prev_head_pose[:3, 3]

        # ---- 左手（臂）增量 ----
        left_pos_delta = np.zeros(3)
        left_euler_delta = np.zeros(3)
        if left_trigger:
            if self.reference_frame == "head":
                left_pos_delta = robot_frame_position_delta(
                    left_pose,
                    self.prev_left_pose,
                    head_pose,
                    self.prev_head_pose,
                    self.head_orientation_reference,
                )
            else:
                left_pos_delta = (left_pose[:3, 3] - self.prev_left_pose[:3, 3]) + head_delta
            left_euler_delta = robot_frame_orientation_delta(
                left_pose,
                self.prev_left_pose,
                head_pose,
                self.prev_head_pose,
                self.head_orientation_reference,
            )

        # ---- 右手（臂）增量 ----
        right_pos_delta = np.zeros(3)
        right_euler_delta = np.zeros(3)
        if right_trigger:
            if self.reference_frame == "head":
                right_pos_delta = robot_frame_position_delta(
                    right_pose,
                    self.prev_right_pose,
                    head_pose,
                    self.prev_head_pose,
                    self.head_orientation_reference,
                )
            else:
                right_pos_delta = (right_pose[:3, 3] - self.prev_right_pose[:3, 3]) + head_delta
            right_euler_delta = robot_frame_orientation_delta(
                right_pose,
                self.prev_right_pose,
                head_pose,
                self.prev_head_pose,
                self.head_orientation_reference,
            )

        left_enabled = left_trigger and not stale_sample
        right_enabled = right_trigger and not stale_sample
        position_speed = self.fine_position_speed_mps if self.precision_mode else self.fast_position_speed_mps
        orientation_speed = self.fine_orientation_speed_radps if self.precision_mode else self.fast_orientation_speed_radps
        position_scale = self.fine_position_scale if self.precision_mode else self.fast_position_scale
        orientation_scale = self.fine_orientation_scale if self.precision_mode else self.fast_orientation_scale
        left_pos_delta = _limit_increment(
            left_pos_delta, left_enabled, self.max_position_delta_m,
            position_speed, sample_dt,
        )
        right_pos_delta = _limit_increment(
            right_pos_delta, right_enabled, self.max_position_delta_m,
            position_speed, sample_dt,
        )
        left_euler_delta = _limit_increment(
            left_euler_delta, left_enabled, self.max_orientation_delta_rad,
            orientation_speed, sample_dt,
        )
        right_euler_delta = _limit_increment(
            right_euler_delta, right_enabled, self.max_orientation_delta_rad,
            orientation_speed, sample_dt,
        )

        # Apply explicit robot-frame axis calibration before filtering.  This
        # keeps x=front, y=left, z=up visible and makes a reversed controller
        # axis a one-line YAML change rather than a code edit.
        axis_gain = self.position_axis_scale * self.position_axis_sign
        left_pos_delta *= position_scale * axis_gain
        right_pos_delta *= position_scale * axis_gain
        left_euler_delta *= orientation_scale
        right_euler_delta *= orientation_scale

        # 水平面偏航标定：把手的「前/左」对齐到机器人的「前/左」。
        if abs(self.translation_yaw_offset_rad) > 1e-6:
            cy = np.cos(self.translation_yaw_offset_rad)
            sy = np.sin(self.translation_yaw_offset_rad)
            left_pos_delta = _rotate_xy(left_pos_delta, cy, sy)
            right_pos_delta = _rotate_xy(right_pos_delta, cy, sy)

        left_pos_delta = _conservative_deadband(
            left_pos_delta, self._left_pos_residual, self.position_deadband_m, left_enabled,
        )
        right_pos_delta = _conservative_deadband(
            right_pos_delta, self._right_pos_residual, self.position_deadband_m, right_enabled,
        )
        left_euler_delta = _conservative_deadband(
            left_euler_delta, self._left_rot_residual, self.orientation_deadband_rad, left_enabled,
        )
        right_euler_delta = _conservative_deadband(
            right_euler_delta, self._right_rot_residual, self.orientation_deadband_rad, right_enabled,
        )

        def filter_delta(delta, previous, alpha, enabled):
            if not enabled:
                previous.fill(0.0)
                return np.zeros(3)
            # Alpha=1 is the recommended setting for integrated increments:
            # direction changes take effect in the same sample.
            filtered = alpha * delta + (1.0 - alpha) * previous
            previous[:] = filtered
            return filtered

        left_pos_delta = filter_delta(
            left_pos_delta, self._left_pos_filter, self.position_filter_alpha, left_enabled
        )
        right_pos_delta = filter_delta(
            right_pos_delta, self._right_pos_filter, self.position_filter_alpha, right_enabled
        )
        left_euler_delta = filter_delta(
            left_euler_delta, self._left_rot_filter, self.orientation_filter_alpha, left_enabled
        )
        right_euler_delta = filter_delta(
            right_euler_delta, self._right_rot_filter, self.orientation_filter_alpha, right_enabled
        )

        # ---- 发布 ----
        self.left_pos_delta_pub.publish(self._to_vec3(left_pos_delta))
        self.right_pos_delta_pub.publish(self._to_vec3(right_pos_delta))
        self.left_euler_delta_pub.publish(self._to_vec3(left_euler_delta))
        self.right_euler_delta_pub.publish(self._to_vec3(right_euler_delta))

        # 更新上一帧（无论是否扣扳机都更新，保证增量始终是「相对上一帧」，
        # 避免扳机重新扣下时出现位置跳变）
        self.prev_left_pose = left_pose.copy()
        self.prev_right_pose = right_pose.copy()
        self.prev_head_pose = head_pose.copy()

    def _gripper_fdb_callback(self, msg):
        self.dual_grippers.feedback(msg.data, time.monotonic())

    def update_gripper(self, teleData):
        state = teleData.tele_state if teleData is not None else None
        record_button = bool(getattr(state, LEFT_RECORD_BUTTON_ATTR, False))
        discard_button = bool(getattr(state, LEFT_DISCARD_BUTTON_ATTR, False))
        left_button = bool(getattr(state, "left_squeeze_ctrl_state", False))
        right_button = bool(getattr(state, "right_squeeze_ctrl_state", False))
        if record_button and not self.prev_record_button:
            self.toggle_recording()
        if discard_button and not self.prev_discard_button:
            self.discard_recording()
        positions, states = self.dual_grippers.step(time.monotonic(), {
            "left": left_button and not self.prev_left_gripper_button,
            "right": right_button and not self.prev_right_gripper_button,
        })
        self.prev_record_button, self.prev_discard_button = record_button, discard_button
        self.prev_left_gripper_button, self.prev_right_gripper_button = left_button, right_button
        ctl = GripperCtl()
        ctl.ctl_time = 0.02
        ctl.l_arm_gripper_pos, ctl.r_arm_gripper_pos = positions
        ctl.l_arm_gripper_tor = ctl.r_arm_gripper_tor = 0.0
        if np.isfinite(positions).all():
            self.gripper_ctl_pub.publish(ctl)
        msg = Int8MultiArray()
        msg.data = states  # measured L/R, commanded L/R; 0=open, 1=closed, -1=invalid
        self.gripper_state_pub.publish(msg)

    def _recording_state_callback(self, message):
        if not self._record_request_pending:
            self.recording_active = bool(message.data)

    def toggle_recording(self):
        """Toggle the passive recorder on a left-hand B-button rising edge."""
        if self._record_request_pending:
            return
        if not self.record_service_client.service_is_ready():
            if not self._record_service_warned:
                self.get_logger().warning(
                    f"左 B 键录制切换不可用：未发现 {self.record_service_name}，请先启动 collection/record.py"
                )
                self._record_service_warned = True
            return
        request = SetBool.Request()
        request.data = not self.recording_active
        self._record_request_pending = True
        future = self.record_service_client.call_async(request)

        def _done(done_future):
            try:
                response = done_future.result()
            except Exception as exc:  # pragma: no cover - ROS transport failure
                self._record_request_pending = False
                self.get_logger().error(f"左 B 键录制切换服务失败: {exc}")
                return
            self._record_request_pending = False
            if response.success:
                self.recording_active = request.data
                self._record_service_warned = False
                self.get_logger().info(
                    "左 B 键录制开始" if self.recording_active else "左 B 键录制结束"
                )
            else:
                self.get_logger().warning(f"左 B 键录制切换被拒绝: {response.message}")

        future.add_done_callback(_done)

    def discard_recording(self):
        """Discard the current episode on a left-hand A-button rising edge."""
        if self._record_request_pending:
            return
        if not self.discard_service_client.service_is_ready():
            if not self._discard_service_warned:
                self.get_logger().warning(
                    f"A 键丢弃录制不可用：未发现 {self.discard_service_name}，请先启动 collection/record.py"
                )
                self._discard_service_warned = True
            return
        self._record_request_pending = True
        future = self.discard_service_client.call_async(Trigger.Request())

        def _done(done_future):
            try:
                response = done_future.result()
            except Exception as exc:  # pragma: no cover - ROS transport failure
                self._record_request_pending = False
                self.get_logger().error(f"A 键丢弃录制服务失败: {exc}")
                return
            self._record_request_pending = False
            if response.success:
                self.recording_active = False
                self._discard_service_warned = False
                self.get_logger().info("已丢弃当前录制")
            else:
                self.get_logger().info(f"丢弃被忽略: {response.message}")

        future.add_done_callback(_done)

    @staticmethod
    def _to_vec3(arr):
        msg = Vector3()
        msg.x = float(arr[0])
        msg.y = float(arr[1])
        msg.z = float(arr[2])
        return msg


# -----------------------------------------------------------------------------
# 主流程
# -----------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--debug-controller-input",
        action="store_true",
        help="print raw XR controller status without publishing control commands",
    )
    args = parser.parse_args()

    # 1. 用一块全零共享内存替代相机图像（位姿数据走 CONTROLLER_MOVE 事件，与图像无关）
    img_shape = (480, 1280, 3)  # 双目（宽 = 2 * 640）
    shm_size = int(np.prod(img_shape) * np.uint8().itemsize)
    image_shm = shared_memory.SharedMemory(create=True, size=shm_size)

    # 2. SSL 证书路径（与 vuer 的 HTTPS/WSS 服务配合，已由 openssl 生成于项目根目录）
    project_root = Path(__file__).resolve().parent.parent
    cert_file = project_root / "cert.pem"
    key_file = project_root / "key.pem"

    # 3. 初始化遥操作封装（手柄模式，不接相机）
    tv_wrapper = TeleVuerWrapper(
        binocular=True,
        use_hand_tracking=False,
        img_shape=img_shape,
        img_shm_name=image_shm.name,
        return_state_data=True,
        cert_file=str(cert_file),
        key_file=str(key_file),
    )

    # 4. ROS2 节点
    rclpy.init()
    node = VRPoseIncrementPublisher()

    try:
        input("请先用 XR 设备浏览器打开 https://<本机IP>:8012?ws=wss://<本机IP>:8012，"
              "点击 'Virtual Reality' 后，回到终端按 Enter 开始...\n")

        if args.debug_controller_input:
            node.get_logger().warning(
                "开始 XR 控制器输入诊断：不会发布机械臂、夹爪、回零或力前馈命令"
            )
        else:
            node.get_logger().info("开始发布位姿增量（扣动扳机生效）")
        target_fps = 50
        frame_interval = 1.0 / target_fps

        while rclpy.ok():
            start_time = time.time()

            run_vr_tick(node, tv_wrapper, args.debug_controller_input)

            rclpy.spin_once(node, timeout_sec=0.0)

            elapsed = time.time() - start_time
            time.sleep(max(0.0, frame_interval - elapsed))

    except KeyboardInterrupt:
        node.get_logger().warning("KeyboardInterrupt，退出...")
    except Exception as e:
        node.get_logger().error(f"运行出错: {e}")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        image_shm.close()
        image_shm.unlink()
        node.get_logger().warning("已清理资源，退出")


if __name__ == '__main__':
    main()
