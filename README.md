# 擦毛巾：ENCOS 双臂 VR 遥操作与数据采集

从原工程提取的任务目录。保留双臂控制链路，使用头部 + 单腕部两路相机；
新采集器记录双臂数据，可离线导出 LeRobot 数据集，供后续 π0.5 微调。
原目录没有修改。本目录不会自动加载旧工作区，也不包含旧示教、ACT 模型、训练框架整库或旧编译产物。

## 目录与入口

```text
towel_task/
├── ros2_ws/src/              双臂控制、驱动、消息和机器人模型
├── teleop/
│   ├── VR_controller/vr_pose_increment_publisher.py  VR 入口
│   ├── VR_controller/config/gripper_pd.yaml         手柄映射/缩放（夹爪参数在 grippers.yaml）
│   └── src/televuer/         项目实际使用的 XR 接收库
├── collection/
│   ├── record.py            双臂、双相机原始数据采集
│   ├── core.py              数据定义与片段存储
│   └── export_lerobot.py     校验和 LeRobot 导出
├── config/recording.yaml     任务文本、相机序列号、帧率、时序阈值
├── scripts/                 环境、编译、分进程启动、只读检查
├── Makefile                 预检、构建、统一启动和数据验收命令
├── training/README_PI05.md   后续训练的数据映射和示例
├── docs/                    数据格式、提取清单、验证结果
└── tests/                   无硬件采集测试
```

## 当前完成范围

- 复用原来的左右臂 VR 位姿增量、轨迹处理、逆解和伺服控制。
- 两路相机默认序列号沿用 `261222075640`（head）、`260422274993`（wrist）。
- 新采集器记录双臂关节反馈、关节目标、末端反馈、末端名义目标、左右夹爪开合状态（反馈/目标）；可选保存原力话题。
- 右 B 开始/结束，左 `left_aButton` 丢弃片段。录制服务独立为 `/towel/*`。
- 原始 PNG + JSONL 保留实际时间戳；导出为 LeRobot Parquet + 视频。

**左右夹爪均已实现独立控制和端点标定流程。** 左 squeeze 切换左夹爪，`left_bButton` 切换右夹爪。
采集只保留夹爪二值开合状态（0=开，1=闭），不保存每帧夹爪角度、电流、温度或力矩。
实机端点尚未测量，未标定侧保持零输出，录制器拒绝未知状态。首次使用先完成 [夹爪标定](docs/GRIPPER_CALIBRATION.md)。

**当前主机不能直接启动完整控制链路：** 缺少 Pinocchio（含 CasADi Python 绑定）、CasADi、
Meshcat、Vuer 和 pyrealsense2。原目录的 OpenRobots 动态库是 ARM aarch64，当前主机为 x86_64，
因此没有复制这些库冒充可用依赖。见 [验证结果](docs/VALIDATION.md) 和 [依赖说明](docs/DEPENDENCIES.md)。

## 1. 准备环境与编译

环境必须分为机器人端和 π0.5 训练端；完整说明与可复现 Conda 文件见
[environments/README.md](environments/README.md)。机器人端使用系统 Python 3.12 + ROS 2 Jazzy，
训练端使用 `towel-pi05` Conda 环境。

以下命令均在 `towel_task` 下执行。机器人端使用 ROS 2 Jazzy / Python 3.12。

```bash
source /opt/ros/jazzy/setup.bash
python3.12 -m venv --system-site-packages .venv
source .venv/bin/activate
python -m pip install -r requirements-record.txt
python -m pip install -e ./teleop
bash scripts/create_certificate.sh
```

证书脚本仅在文件不存在时生成本任务的新证书，不复制发送者的私钥。

在机器人主机准备好匹配架构的 Pinocchio C++/Python、`pinocchio.casadi`、CasADi、Meshcat 后编译：

```bash
# 若机器人上使用原厂 OpenRobots 安装包，显式设置它的前缀；
# 普通系统安装则不设置此变量。
# export TOWEL_OPENROBOTS_PREFIX=/absolute/path/to/openrobots_libs_hw
bash scripts/build.sh
source scripts/env.sh
python3 scripts/preflight.py
```

不能把原目录的 ARM 库路径用于 x86 主机。自定义 OpenRobots 前缀的 CMake 文件也必须可重定位，
原包中存在 `/home/nico/openrobots` 绝对路径，需要提供者修复或重新安装依赖。

`scripts/build.sh` 使用普通安装而不是符号链接安装，避免原始工程遗留的 Python 包目录与符号链接冲突。
RViz bringup 包与双臂控制包分开构建；若控制包因 `pinocchio` 等外部依赖未满足而失败，仍可构建并使用相机/RViz 预览，但不能启动真机控制链路。

## 2. 核对任务配置

- `config/recording.yaml`：两路相机、存储目录和实际任务指令。当前英文描述是可编辑占位，按真实动作修改。
- `teleop/VR_controller/config/gripper_pd.yaml`：遥操作坐标映射和运动缩放。
- `teleop/VR_controller/config/grippers.yaml`：左右夹爪独立开/闭端点、PD、力矩限幅。
- `ros2_ws/src/ctl_msgs_hw/config/`：继承的机器人零位、PD、回零和导纳参数。
- `ros2_ws/src/servo_control_hw/yamlpkg/can_config.yaml`：电机编号和方向。
- `ros2_ws/src/encos_force_sensor/config/force_sensor.yaml`：两路 CAN 力传感器配置。

这些是原机器人配置，不是针对新毛巾任务重新标定的结果。导纳开关继承为 `2`，
因此控制链路仍需原有力传感器；采集器中力数据是否可选不等于控制器可以省略力传感器。
原中层代码还存在左右力话题交叉订阅，原始记录按话题名保留，不擅自纠正硬件对应关系。

## 3. 一键启动与 RViz 预览

明天真机实验请优先按 [真机实验流程](docs/REAL_ROBOT_EXPERIMENT.md) 执行；
它包含预检、相机预览、夹爪标定、力传感器去皮、试录和收尾步骤。

统一启动可使用：

```bash
make experiment VISUALIZE=true
```

完整 Launch 默认不启动硬件节点，只有 `make experiment` 会显式启动它们。
RViz 可显示独立 TF 树、双臂模型和采集器的两路 RGB 图像；它不发送机器人命令。

## 4. 分终端启动

每个终端先进入本目录并运行：`source scripts/env.sh`。
按原机器人已调通的硬件上电、网口/CAN 设置流程准备好设备，然后分别运行：

```bash
# 终端 1：电机层
bash scripts/run.sh servo
# 终端 2：双臂控制层
bash scripts/run.sh arms
# 终端 3：上层 VR 指令处理
bash scripts/run.sh robot
# 终端 4：力传感器
bash scripts/run.sh force
# 终端 5：VR
bash scripts/run.sh vr
# 终端 6：采集（可在 VR 操作稳定后再启动）
bash scripts/run.sh record
```

VR 设备与主机同网，在设备浏览器访问：

```text
https://<主机IP>:8012?ws=wss://<主机IP>:8012
```

进入 Virtual Reality 后，在 VR 程序终端按 Enter。
同一台机器人上仅运行一套这些控制进程，避免旧、新工作区同时发布同名控制话题。

### 按键（以库字段名为准）

| 输入 | 行为 |
|---|---|
| 左/右扳机 | 控制对应机械臂的移动和旋转 |
| `right_aButton` | 触发原双臂回零过程 |
| `right_bButton` | 开始录制 / 结束并保存 |
| 左 squeeze | 左夹爪开合切换 |
| `left_bButton` | 右夹爪开合切换 |
| `left_aButton` | 丢弃当前片段 |
| 右 squeeze | 原右臂力前馈 |

原项目的注释对物理 X/Y 名称有冲突，所以此处列库字段，实机核对按键再使用。

也可在 `source scripts/env.sh` 后直接调用：

```bash
ros2 service call /towel/record std_srvs/srv/SetBool '{data: true}'
ros2 service call /towel/record std_srvs/srv/SetBool '{data: false}'
ros2 service call /towel/discard std_srvs/srv/Trigger
```

采集必须收到完整、新鲜的反馈和指令才能开始。正常结束保存片段；Ctrl+C 或断流留下 `.partial`，
不会当成成功演示导出。恢复后再次按 B 开始新片段。原始 PNG 占用较大，导出后仍建议保留原始数据用于排查时序。

## 5. 校验与导出

```bash
python3 collection/export_lerobot.py \
  --root data/raw/towel_wipe --validate-only
```

导出需要额外的 LeRobot 数据集依赖，放在训练/转换环境中即可，不需要 ROS：

```bash
# 在安装了 LeRobot 的 Python 环境中执行
python3 collection/export_lerobot.py \
  --root data/raw/towel_wipe \
  --output data/lerobot/towel_wipe \
  --repo-id local/towel_wipe
```

导出目录必须是新目录；原始目录允许继续增加独立片段。相机或机器人配置变化的片段分开导出。
数据字段和单位见 [DATASET.md](docs/DATASET.md)；训练接入见 [README_PI05.md](training/README_PI05.md)。

## 6. 无硬件验证

```bash
source scripts/env.sh
python3 -m pytest -q tests
```

不启动控制节点、不发送电机指令。需要已编译的 `ctl_msgs_hw` 才会执行真实消息定义相关测试。
