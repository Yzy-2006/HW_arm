# 验证记录（2026-09-15）

主机：x86_64，Python 3.12.3，ROS 2 Jazzy。

## 已通过

- `pytest`：15 个测试全部通过。
  - 左右臂顺序、观测/动作区分、夹爪二值反馈/动作标签。
  - 缺失、过期、时序偏差、非有限数值拒绝。
  - 完成/丢弃/失败片段生命周期与图像写入失败处理。
  - 固定频率重采样保留真实时间，不压缩缺帧。
  - 损坏图像和不连续时间戳拒绝。
  - 导出 API 测试替身验证特征集合、形状、标签和 schema 元信息。
  - 真实生成的 ROS 消息类型配合替身 Node，验证采集回调字段、服务状态及连续片段边界。
- 新工作区编译通过：`dual_arm_control_hw`。该检查覆盖已移除力传感器和阻抗/导纳路径后的双臂位置控制节点。
- 新 Python 代码和复制的 VR/IK Python 语法检查通过；shell 脚本语法检查通过。
- 采集/导出 `--help` 可运行，不需要 ROS 硬件或 LeRobot。

- 双夹爪独立方向、位置轨迹、启动保持、反馈超时保持目标、恢复重锚定、状态滞回和零点换算测试。
- 双端点标定的稳定性、最小间距、备份和失败不覆盖测试。
- VR 按键上升沿与左右状态发布测试。

## 未通过/未执行

- 全控制链编译在 `dual_arm_control_hw` 的 `find_package(pinocchio)` 处停止：本机缺少 C++ Pinocchio。
  `robot_control_hw`、`servo_control_hw` 未完成此轮构建。
- 只读环境检查确认缺少 pyrealsense2、CasADi、Pinocchio/CasADi 绑定、Meshcat、Vuer。
- 未安装完整 LeRobot/PyTorch，所以没有实际生成 Parquet/MP4；导出 API 测试使用替身。
- 没有执行实机夹爪端点采样；配置明确为 calibrated: false。
- 没有启动电机节点、CAN 配置、VR 服务和相机；没有实机采集或训练验证。

本目录中的证书为本任务新生成，仅供本地 VR HTTPS 使用，已在 `.gitignore` 中排除。
