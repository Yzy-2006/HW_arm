# 验证说明与历史记录

[文档索引](README.md)

## 当前检查入口

在已配置的机器人环境中，根目录 `make check` 执行只读依赖检查，`make test` 执行
`tests/`。部分测试需要 C++ 编译器、构建后的 ROS 消息或可选依赖；测试跳过不能视为通过相应功能验收。
修改消息或控制源码后运行 `make build`，它会请求 sudo 设置网卡权限，应在现场终端执行。

Newton 测试和 GPU 验收见[仿真指南](../simulation/README.md#验证与验收)，
手势识别模块测试见[模块说明](../gesture_teleoperation/README.md#tests)，
控制链专项离线检查见[控制安全说明](CONTROL_SAFETY.md#offline-verification)。
真机预检与采集验收见[当天实验总清单](REAL_ROBOT_EXPERIMENT.md)。

离线测试不证明急停、物理回零、真实控制频率或采集吞吐。新增验证结果应注明日期、环境、
命令及通过/失败/跳过项；本次文档整理没有重做以下历史实验。

## 2026-09-15 历史记录

以下内容保留当时的验证结果和缺失项，不表示当前环境状态或当前测试数量。

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
