# 提取范围与依赖

[文档索引](README.md) · [环境与构建](../environments/README.md)

本文保留项目初始提取来源；当前模块分工见[项目架构](ARCHITECTURE.md)。

## 从源码复制

- `ctl_msgs_hw`：双臂控制消息、零位和 PD 配置。
- `servo_control_hw`：电机通信与控制；只保留 Linux SOEM 所需分支，剔除 cpp-readline 和无关平台代码。
- `dual_arm_control_hw`：双臂位置控制与 IK/FK；Python IK 以源码安装，不再 Cython 编译隐藏源码。
- `robot_control_hw`：上层目标和遥操作处理；未编入主节点的底盘硬件实现已剔除。
- `teleop`：现用 VR 入口、实际 televuer 库和配置。调整任务服务名、配置路径和录制状态同步，并添加双夹爪控制与标定模块。

底层运动学算法、增益和电机方向沿用原工程，没有针对新机器人重新调参。

## 从安装目录恢复

- `dual_arm_hw`：只提取 URDF、网格和模型配置，添加资源包 CMake/package.xml；不复制原节点二进制。
- `chassis_control_hw`：提取原始 msg/srv/action，重建消息包。
  原上层控制器仍引用它的升降等消息类型，所以不能直接删掉这个依赖；本目录不提供底盘驱动节点。

目录级来源见 [copied_sources.json](copied_sources.json)。旧 `.git`、`.vscode` 数据库、缓存、build/install/log、
白板控制/标定、旧 demo、白板数据、ACT 输出及 LeRobot 整个仓库均未作为源码带入。
本次本地验证产生的新 build/install/log 已由 `.gitignore` 忽略。

## 必须由运行环境提供

1. ROS 2 Jazzy 的 rclcpp/rclpy、消息生成器、std/geometry/sensor/std_srvs、KDL/URDF、robot_state_publisher。
2. C++ Pinocchio（CMake 包 `pinocchio`）、Python Pinocchio，且 `from pinocchio import casadi` 必须成功。
3. CasADi/IPOPT 插件、NumPy、SciPy、Meshcat，以及编译器、Eigen、Boost、yaml-cpp、TBB。
4. VR：Vuer 0.0.60 等依赖见 `teleop/pyproject.toml`。
5. 相机：pyrealsense2；采集时不需要 PyTorch、LeRobot 或 GPU。
6. 转换/训练：单独安装支持当前数据 API 的 LeRobot；π0.5 另需其模型和训练依赖。

`pinocchio.casadi` 不是所有 Pinocchio 发行包都提供的可选绑定；只安装名字相似的 Python 包不够。
必须使用同时匹配 C++、Python 3.12、CasADi 和处理器架构的构建。

原 `openrobots_libs_hw` 约 282 MB，是 ARM aarch64 库，并含旧机器绝对路径；没有复制到新源码目录。
在原机器人上可通过 `TOWEL_OPENROBOTS_PREFIX` 显式选择修复过路径的兼容依赖。
在 x86 主机需要重新安装/构建依赖，不能用原 ARM `.so`。

## 当前验收边界

- 依赖是否满足以目标主机的 `make check` 和构建结果为准；早期缺依赖记录见[历史验证](VALIDATION.md)。
- 控制链、夹爪标定、PI05/ACT bridge、worker 和回放器已有实现，使用入口见[文档索引](README.md)。
- 实际控制频率、相机吞吐、夹爪端点、零位及策略效果需在目标设备逐项验收；文档整理不代表实机验证通过。
- 训练说明保留示例性质；模型能启动推理不等于本项目已完成训练或真机任务验收。

原工程各包保留其版权/许可声明；若声明为 TODO，不在本次整理中擅自重新授权。
