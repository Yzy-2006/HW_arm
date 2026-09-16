# Copyright 2026 encos
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""启动六维力传感器驱动节点。

节点会自动加载包内 config/force_sensor.yaml（按传感器数量与信息配置）。
SocketCAN 后端需要先手动配置接口，例如：
    sudo ip link set can0 up type can bitrate 1000000 dbitrate 5000000 fd on
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='encos_force_sensor',
            executable='force_sensor_node',
            name='encos_force_sensor_node',
            output='screen',
        ),
    ])
