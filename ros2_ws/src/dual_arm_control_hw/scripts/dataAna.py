#!/usr/bin/env python3
"""
dataAna.py - 数据发布节点
引用 ctl_msgs_hw 中的 DbgInformation 消息类型，每调用一次发送一次。
待发送数据通过函数参数传入。
"""

import sys
import os

sys.path = [p for p in sys.path if 'opt/ros' not in p and '/ros/' not in p]   #强制移除ros2路径

# 与发行版/解释器解耦的路径变量（ROS_DISTRO 由 source setup.bash 设置）
_ros_distro = os.environ.get("ROS_DISTRO", "humble")
_pyver = f"python{sys.version_info.major}.{sys.version_info.minor}"

# 依赖路径由 towel_task/scripts/env.sh 中的 TOWEL_OPENROBOTS_PREFIX 显式设置。
# 不再猜测旧 workspace 相邻的 openrobots_libs_hw 目录，避免混入 Python 3.10 库。
_openrobots_site = os.environ.get("OPENROBOTS_PYTHON_PATH")
if _openrobots_site:
    sys.path.insert(0, _openrobots_site)

sys.path.insert(0, f"/opt/ros/{_ros_distro}/local/lib/{_pyver}/dist-packages")   #导入rclpy
sys.path.insert(0, f"/opt/ros/{_ros_distro}/lib/{_pyver}/site-packages")         #导入其他python包

import rclpy
from rclpy.node import Node
from ctl_msgs_hw.msg import DbgInformation
import numpy as np

# 写日志到文件，方便在 C++ 嵌入 Python 场景下调试
LOG_FILE = "/tmp/dataAna_debug.log"

def _log(msg: str):
    with open(LOG_FILE, "a") as f:
        f.write(f"[dataAna] {msg}\n")


class DbgInfoPublisher(Node):
    """发布 DbgInformation 消息的 ROS 2 节点，通过 publish_once() 发送"""

    def __init__(self):
        super().__init__('dbg_info_publisher')

        self.publisher_ = self.create_publisher(
            DbgInformation,
            'pyDbgPub/dbg_pub_1',
            10,
        )

        self.count_ = 0
        _log("DbgInfoPublisher 节点创建成功")
        self.get_logger().info('DbgInfoPublisher 节点已初始化，等待 publish_once() 调用')

    def publish_once(self, float_reserve, float_reserve_b, int_reserve):
        """
        构造并发布一条 DbgInformation 消息。

        Args:
            float_reserve:   list/ndarray/tuple，长度 7，类型 float，对应 float64[7]
            float_reserve_b: list/ndarray/tuple，长度 7，类型 float，对应 float64[7]
            int_reserve:     list/ndarray/tuple，长度 7，类型 int，  对应 int32[7]

        Returns:
            msg: 已发布的 DbgInformation 消息对象
        """
        msg = DbgInformation()

        for i in range(7):
            msg.float_reserve[i] = float(float_reserve[i])
            msg.float_reserve_b[i] = float(float_reserve_b[i])
            msg.int_reserve[i] = int(int_reserve[i])

        self.publisher_.publish(msg)

        # spin_once 把消息从队列推送到 DDS
        rclpy.spin_once(self, timeout_sec=0.01)

        """
        self.get_logger().info(
            f'#{self.count_} 发布: '
            f'float_reserve[0]={msg.float_reserve[0]:.3f}, '
            f'float_reserve_b[0]={msg.float_reserve_b[0]:.3f}, '
            f'int_reserve[0]={msg.int_reserve[0]}'
        )

        _log(f"#{self.count_} 发布成功")
        """
        self.count_ += 1
        return msg


_rclpy_initialized = False


def ensure_rclpy_init():
    """
    确保 rclpy 已初始化（只能调用一次）。
    返回 True 表示初始化成功，False 表示失败。
    """
    global _rclpy_initialized
    if _rclpy_initialized:
        return True

    _log(f"Python: {sys.executable}")
    _log(f"sys.path: {sys.path[:5]}")
    _log(f"ROS_DOMAIN_ID={os.environ.get('ROS_DOMAIN_ID', 'N/A')}")

    try:
        rclpy.init(args=[])
        _rclpy_initialized = True
        _log("rclpy.init() 成功")
        return True
    except Exception as e:
        _log(f"rclpy.init() 失败: {e}")
        # 可能 rcl 已被 C++ 侧初始化，尝试不传参数
        try:
            rclpy.init(args=None)
            _rclpy_initialized = True
            _log("rclpy.init(args=None) 成功")
            return True
        except Exception as e2:
            _log(f"rclpy.init(args=None) 也失败: {e2}")
            return False
