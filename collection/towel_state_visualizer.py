#!/usr/bin/env python3
"""Read joint feedback and publish a dedicated JointState stream for RViz."""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState

from ctl_msgs_hw.msg import MidNodeCtlMsgRet


class TowelStateVisualizer(Node):
    """No control publishers: this bridge only converts existing feedback for visualization."""

    def __init__(self):
        super().__init__("towel_state_visualizer")
        self.publisher = self.create_publisher(JointState, "/towel/joint_states", qos_profile_sensor_data)
        self.create_subscription(
            MidNodeCtlMsgRet,
            "/dexbotCtl/midMonitor_msg_send",
            self.feedback_callback,
            qos_profile_sensor_data,
        )
        self.get_logger().info("RViz feedback bridge ready: /dexbotCtl/midMonitor_msg_send -> /towel/joint_states")

    def feedback_callback(self, message):
        left_q = list(message.l_arm_pos_fdback)
        right_q = list(message.r_arm_pos_fdback)
        left_dq = list(message.l_arm_spd_fdback)
        right_dq = list(message.r_arm_spd_fdback)
        if min(len(left_q), len(right_q), len(left_dq), len(right_dq)) < 7:
            self.get_logger().warning("Ignoring incomplete joint feedback for RViz")
            return
        joint = JointState()
        joint.header.stamp = message.header.stamp
        joint.name = [*[f"arm_l_joint_{index}" for index in range(1, 8)], *[
            f"arm_r_joint_{index}" for index in range(1, 8)
        ]]
        joint.position = [*left_q[:7], *right_q[:7]]
        joint.velocity = [*left_dq[:7], *right_dq[:7]]
        self.publisher.publish(joint)


def main():
    rclpy.init()
    node = TowelStateVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
