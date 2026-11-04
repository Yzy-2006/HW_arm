# 同一设备更换上位机：迁移与操作交接

核对日期：2026-10-09。本文依据当前工作目录中的代码和配置编写，面向接替机器人控制、VR 遥操和数据采集的操作者。参数表是交接快照，接收后以实际 YAML 为准。

## 1. 本次迁移范围与最小修改清单

- **已确认：同一套机械臂、夹爪、相机，只更换上位机。** 机器人侧机构、编码器零点和接线保持不变；本次不是新机器人适配。
- 原样迁移现有零位、夹爪端点、电机映射、限位、PD、URDF 和 VR 参数，**不需要因为换电脑重新标定或重新调增益**。接收后只做一致性检查与空载复核。
- 本项目控制端使用 Linux、ROS 2 Jazzy 和系统 Python 3.12。新电脑优先使用与此组合匹配的 Ubuntu 24.04；Windows、WSL 或不同 ROS/Python 版本不属于本文的直接迁移流程。
- 控制和采集不需要 PyTorch、CUDA 或 LeRobot。训练环境另见 [训练说明](../training/README_PI05.md)，不要和机器人控制环境混装。

新上位机主要处理以下项目，其余机器人参数保持原值：

| 项目 | 是否需要修改 |
|---|---|
| 操作系统、ROS、Python、SDK、venv | 在新电脑重新安装，并重新构建工作区 |
| `controllerConfig.yaml` 的 `netCard_name` | 核对新电脑实际 EtherCAT 网口名；与原值不同时修改 |
| SDK 路径与项目目录 | 按新电脑安装位置配置；项目无需沿用原用户名 |
| 头显连接 IP、TLS 证书、网络权限 | 使用新电脑 LAN IP，重新生成证书并确认头显访问 |
| `recording.yaml` 的数据目录 | 确认可写、空间足够；按需改为新电脑路径 |
| 三相机序列号 | 同一批相机，**保留原值**；核对设备枚举与画面位置即可 |
| 机械臂/夹爪标定、PD、限位、VR 模式 | **全部保留当前值**，不执行重新标定命令 |

若迁移时另外改变了接线、重置编码器或拆装机构，应暂停上述“保留标定”的假设，按实际变化重新核对；这些不属于本次已确认的换上位机场景。

**硬件运动边界：** `make servo` 普通启动会使机械臂回零，`make robot` 会输出遥操初始目标。`Ctrl+C` 只停止当前进程，不等同于物理急停；停止 VR 后夹爪可能继续保持最后的位置目标。首次启动前需有人在现场确认支撑、活动空间和急停。

## 2. 原电脑需要交付什么

交付当前完整源码，而不只是旧 Git 提交：现场参数和修改可能尚未提交。至少包含 `ros2_ws/src/`、`teleop/`、`collection/`、`config/`、`scripts/`、`environments/`、`tests/`、`docs/`、`training/`、`Makefile` 和依赖清单。

必须保留当前有效的 `controllerConfig.yaml`、`zeropos.yaml`、`desc_zeropos.yaml`、`grippers.yaml`、`gripper_pd.yaml` 和 `recording.yaml`。标定 `.backup` 可单独归档；数据集按需单独迁移，避免混入新机器的首次测试数据。

不要复用原电脑的 `.venv-robot`、`ros2_ws/build`、`ros2_ws/install`、`ros2_ws/log` 或 Python 缓存：它们包含解释器、架构或绝对路径信息。TLS 私钥在新电脑重新生成。

原电脑项目根目录可用以下命令制作当前工作目录的源码包，包含尚未提交的修改：

```bash
tar --exclude='./.git' --exclude='./.venv*' \
    --exclude='./ros2_ws/build' --exclude='./ros2_ws/install' --exclude='./ros2_ws/log' \
    --exclude='./data' --exclude='./.pytest_cache' --exclude='__pycache__' \
    --exclude='*.egg-info' --exclude='./teleop/cert.pem' --exclude='./teleop/key.pem' \
    -czf ../towel_task_source.tar.gz .
```

新电脑解压到自己的目录，例如 `~/projects/HW_arm/towel_task`。不必创建原操作者的 `/home/yanziyu` 用户目录；运行脚本会根据项目所在位置确定根目录。

代码以外还需交接：

| 物料 | 要记录的内容 |
|---|---|
| OpenRobots/Pinocchio SDK | 安装包或源码、版本、构建选项、安装路径、CPU 架构，尤其是 CasADi Python 绑定 |
| 硬件接线 | EtherCAT 专用网口、网关接线、左右 CAN 通道、电机 ID 与方向 |
| 相机 | 三台设备序列号与头部/左腕/右腕的对应关系 |
| 现场数据 | 有效标定 YAML、机器人身份、采集数据存放位置 |

SDK **不包含在这份代码中**；仅复制仓库或执行 pip 安装不能保证具备完整的运动学环境。

## 3. 安装环境

### 3.1 系统与 ROS

先安装 ROS 2 Jazzy，确保 `/opt/ros/jazzy/setup.bash` 存在。以下命令以已经配置好 ROS 软件源的 Ubuntu 24.04 为前提：

```bash
sudo apt update
sudo apt install build-essential cmake git pkg-config openssl libcap2-bin \
    python3.12-venv python3.12-dev python3-numpy cython3 \
    python3-colcon-common-extensions python3-rosdep \
    libeigen3-dev libboost-all-dev libyaml-cpp-dev libtbb-dev liborocos-kdl-dev \
    ros-jazzy-desktop ros-jazzy-kdl-parser ros-jazzy-tf2-kdl \
    ros-jazzy-urdf ros-jazzy-robot-state-publisher
```

如 rosdep 尚未初始化，先运行一次 `sudo rosdep init`，然后 `rosdep update`。已初始化的机器不要重复初始化。进入项目后可检查工作区依赖：

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths ros2_ws/src --ignore-src -r -y
```

rosdep 不能代替下面的专用 SDK 检查；尤其是安装了普通 Pinocchio 不代表具有 `pinocchio.casadi`。

### 3.2 OpenRobots 与运动学依赖

需要同时提供 C++ Pinocchio、Python Pinocchio/CasADi 绑定、CasADi/IPOPT、SciPy、Meshcat。当前默认 SDK 前缀为 `/opt/openrobots`。

- `uname -m` 为 `x86_64` 时使用 x86_64 库，不能复用 aarch64 的 `.so`；反之亦然。
- Python 绑定须匹配系统 Python 3.12，C++ 库与 Python 绑定须来自兼容构建。
- 要求 `from pinocchio import casadi` 成功，不能只检查 `import pinocchio`。
- 仓库没有自动重建整套 SDK 的脚本。原机器管理员应提供可复现的 SDK 安装材料；缺少此项时先补齐，不能跳过后直接启动控制。

`scripts/env.sh` 假定 SDK Python 绑定在 `<前缀>/lib/python3.12/site-packages`，库在 `<前缀>/lib`。如果安装布局不同，需要按实际布局修改该脚本的 `OPENROBOTS_PYTHON_PATH`、`PYTHONPATH`、`LD_LIBRARY_PATH`、`CASADIPATH`。

`Makefile` 默认 `OPENROBOTS_PREFIX=/opt/openrobots`。若前缀不同，修改这一默认值，或者**所有相关 make 命令**都传入 `OPENROBOTS_PREFIX=/实际前缀`。只在终端设置 `TOWEL_OPENROBOTS_PREFIX` 会被 Makefile 的环境初始化覆盖。

### 3.3 创建机器人 Python 环境

在未激活 Conda 的终端、项目根目录运行：

```bash
/usr/bin/python3.12 -m venv --system-site-packages .venv-robot
source .venv-robot/bin/activate
python -m pip install --upgrade pip
python -m pip install -r environments/robot-system-requirements.txt
python -m pip install -e teleop
python -m pip install pytest
bash scripts/create_certificate.sh
```

机器人 Python 依赖清单包括 NumPy `<2`、PyYAML、OpenCV、pyrealsense2。`teleop/pyproject.toml` 固定 Vuer 为 `0.0.60`。不要通过 pip 安装替代 ROS 自带的 `rclpy` 或消息包，也不要为“修依赖”切换到 Conda 的 Python。

RealSense 另需与新电脑系统匹配的 USB 权限/udev 规则。安装 Python 包不等于设备已经可访问；后面用 `make camera-list` 实际验证。三台相机应使用可靠的 USB 连接和足够的带宽、供电。

### 3.4 构建和只读检查

```bash
make build
make env
make check-control
make check-record
make check-visualize
make test
```

`make build` 在编译后会请求 sudo 密码，为安装的 `servo_node` 授予 `cap_net_raw` 和 `cap_net_admin`。不能把这个授权步骤跳过。重新链接或替换伺服二进制后需重新授权；不要把 `sudo make servo` 作为常规启动方式，以免 root 与普通用户的 ROS 通信环境不同。

```bash
getcap ros2_ws/install/servo_control_hw/lib/servo_control_hw/servo_node
```

预期包含 `cap_net_admin,cap_net_raw=ep`。`make check*` 主要检查模块、文件和 ROS 包是否存在；通过不代表 CAN 接线、相机图像和机器人运动已验证。部分无依赖测试可能被跳过，需查看原因。

## 4. 必须核对的配置文件

以下路径均相对于项目根目录。配置文件中的历史值不是适用于所有硬件的通用默认值。

| 文件/字段 | 新操作者要做的事 | 生效方式 |
|---|---|---|
| `Makefile`：`VENV`、`OPENROBOTS_PREFIX` | 确认新电脑 venv 与 SDK 路径；常规使用项目下 `.venv-robot` | 后续 make 命令 |
| `scripts/env.sh` | 确认 Jazzy、SDK Python/lib 路径，不加载旧工作区 | 新终端或重启进程 |
| `ros2_ws/src/ctl_msgs_hw/config/controllerConfig.yaml`：`TransLoopCtlInte.servo_node_loop_ctl.netCard_name` | **必须改为接 EtherCAT 的实际有线网卡**；当前为 `enp129s0` | 同步配置后重启 servo |
| 同文件：运动学链、DOF、周期、限位、补偿 | 本次原样保留，核对复制前后数据一致 | 同步配置后重启相关控制节点 |
| 同文件：`gripperControl` | 核对位置模式、两侧 PD、编码器限位与零偏，覆盖对应开闭目标 | 同步配置后重启 servo 和 VR |
| 同文件：`vrOrientationControl`、`vrTeleoperationLimit`、`vrJointReturn` | 核对工具轴锁定、末端限幅、右 A 回零时间 | 同步配置后重启 robot |
| `ros2_ws/src/ctl_msgs_hw/config/zeropos.yaml` | 启动及右 A 的双臂关节零位，单位 rad；原样保留，无需重标定 | 同步配置后重启控制节点 |
| `ros2_ws/src/ctl_msgs_hw/config/desc_zeropos.yaml` | 遥操初始 TCP 位姿，位置 m、RPY rad；与关节零位对应 | 同步配置后重启控制及 VR |
| 同目录 `initialPD.yaml`、`jointLink_config.yaml`、`robot_config.yaml` | 核对关节增益、硬件/模型配置；不是换电脑就需要重新调参 | 同步配置后重启使用它的节点 |
| `ros2_ws/src/dual_arm_hw/urdf/dual_arm.urdf` 及 meshes | 原样保留当前模型 | 在新电脑重新构建资源包 |
| `teleop/VR_controller/config/grippers.yaml` | 实际左右开闭端点、标定状态、运动时间；单位 deg | 重启 VR |
| `teleop/VR_controller/config/gripper_pd.yaml` | VR 坐标轴、快速/精细模式、倍率、滤波 | 重启 VR，无需编译 |
| `config/recording.yaml` | 保留三相机序列号与采集参数，核对设备可见性；按需修改数据目录 | 重启录制器/预览 |
| `teleop/cert.pem`、`teleop/key.pem` | 新电脑生成证书，头显重新信任和连接 | 重启 VR |

底层 CAN 通道、ID、方向还涉及 `ros2_ws/src/servo_control_hw/` 的源码实现。若改过接线，需要结合网关与电机协议核对，不能假设只修改 YAML 就完成硬件迁移。

### 当前参数快照

| 配置项 | 当前值 |
|---|---|
| 左/右夹爪 PD | 两侧均 `kp=10.0`、`kd=2.2` |
| 左夹爪编码器位置范围 | `[-10.0, 100.32378762208262]` deg |
| 右夹爪编码器位置范围 | `[-34.70927483203165, 16.961438085103716]` deg |
| 两侧 `zero_pos_deg` | `0.0` |
| 两侧开闭运动时间 | `0.6 s`，目标以 `grippers.yaml` 的实际标定为准 |
| 工具轴自转锁定 | `vrOrientationControl.lock_tool_twist: false`，允许双臂工具轴自转 |
| VR 末端位置/姿态限幅开关 | `position_enabled: false`、`orientation_enabled: false`，对应末端限幅不生效 |
| VR 基准与方向 | `head`，`position_axis_sign: [1,1,1]`，水平校正角 `0°` |
| 快速档平移/旋转倍率 | `fast_position_scale: 1.2`、`fast_orientation_scale: 1.5` |
| 快速档原始增量限速 | `1.2 m/s`、`6.0 rad/s` |
| 精细档平移/旋转倍率 | `fine_position_scale: 0.6`、`fine_orientation_scale: 0.60` |
| 精细档原始增量限速 | `0.30 m/s`、`1.5 rad/s` |
| 位置/姿态滤波 alpha | 两者 `1.0`，即这一层不做平滑滤波 |

当前实际运动缩放使用 `fast_*` / `fine_*`，不要只改遗留的 `position_scale` / `orientation_scale` 就认为已修改两档行为。算法先限制原始增量速度再乘倍率，因此这里的速度参数不是最终机器人速度的严格上限。位置模式使用电机内部 PD；旧软件力矩限制不等于电机 PD 的输出力矩限制。

### 配置同步规则

ROS 节点通常读取 `ros2_ws/install/.../share/.../config`，并非直接读源码配置。修改 `ctl_msgs_hw/config` 后：

```bash
# SDK 前缀按本机实际情况设置；这一命令只安装配置，不启动机器人。
export TOWEL_OPENROBOTS_PREFIX=/opt/openrobots
bash scripts/build.sh --packages-select ctl_msgs_hw
```

然后在合适的现场状态下重启使用该配置的进程。修改 C++ 源码则执行 `make build`。不要同时修改源码和安装目录两份文件，避免下次构建覆盖；也不要认为修改 YAML 会自动更新正在运行的节点。

## 5. 网络、相机与证书

EtherCAT 网口和头显局域网是不同用途。先用 `ip -br link` 找到实际连接网关的网口，把名称填入 `netCard_name`。链路应有 `LOWER_UP`；必要时按现场网络管理方式启用该接口。EtherCAT 使用原始以太网帧，不是把网卡 IP 改成头显 IP 就能连接。

头显与新电脑需要能在局域网互通。用 `hostname -I` 找到头显可达的 LAN 地址，访问：

```text
https://<新电脑局域网IP>:8012?ws=wss://<新电脑局域网IP>:8012
```

检查 TCP 8012 的防火墙规则、无线 AP 的客户端隔离与浏览器证书信任。`scripts/create_certificate.sh` 检测到已有证书或私钥会直接退出，不会替换过期/不完整文件；如需重建，先将旧文件移到备份位置后再运行。不要继续使用旧电脑 IP，也不要把启动输出的 `https://vuer.ai` 当成本地机器人访问地址。

所有 ROS 控制、标定、采集进程使用同一普通用户和一致的 `ROS_DOMAIN_ID`、`RMW_IMPLEMENTATION` 等通信环境。关闭旧电脑上仍连接同一机器人的控制进程，避免两个主机同时发命令。

相机配置当前为：

| 位置 | 序列号 |
|---|---|
| `head` | `261222075640` |
| `left_wrist` | `260422271922` |
| `right_wrist` | `260422274509` |

```bash
make camera-list
make preview
```

先确保枚举到三台配置中的相机，再逐个遮挡镜头确认命名和实际位置一致。预览完成后退出，再启动录制器；不要让 preview、record 和其他 RealSense 软件同时占用相机。`make record` 必须能启动全部配置相机，少一台就会失败。

## 6. 首次运行与按键

完成环境检查、网卡权限和标定核对后，每个终端进入新电脑自己的项目目录。逐个启动，等待上一层初始化完成：

| 终端 | 命令 | 说明 |
|---|---|---|
| 1 | `make servo` | 底层伺服；普通模式会执行机械臂初始回零 |
| 2 | `make arms` | IK/FK 与轨迹控制 |
| 3 | `make robot` | 上层遥操目标 |
| 4 | `make record` | 三路相机与数据采集；仅遥操可不启动 |
| 5 | `make vr` | VR 输入；进入头显 VR 页面后，在本终端按 Enter |

`make record` 已经同时打开三路 OpenCV 画面；`make record-opencv` 只是兼容别名，不要两个一起启动。

| 操作 | 当前行为 |
|---|---|
| 左/右扳机 | 对应机械臂跟随手柄位姿增量 |
| 左/右 squeeze | 对应夹爪开闭切换 |
| 左 B | 开始/停止录制 |
| 左 A | 丢弃当前录制 |
| 右 A | 双臂关节回零 |
| 右 B | 快速/精细模式切换，默认快速档；不再去预设姿态 |

右 B 的旧话题 `/vr/preset_pose_cmd` 已从运行订阅链路移除。仓库中仍可能保留 `vr_preset_pose.yaml`、`TARGET=preset` 和旧内部实现，用于历史兼容/数据保存；它们不是当前右 B 的配置入口。

独立检查 VR 连接可用 `make vr-debug`：它不发布控制指令，但仍会初始化 VR 节点并校验夹爪配置。不要与正式 `make vr` 同时运行。

同一套机械臂只换电脑通常不需要重新标定。确需标定时先阅读 [机械臂标定](ARM_ZERO_CALIBRATION.md) 和 [夹爪标定](GRIPPER_CALIBRATION.md)：机械臂被动标定会让所选侧零电流，需要支撑；保存命令只采样，不会自动启动反馈发布者。新的夹爪端点必须位于伺服限位内，标定成功并不自动同步或扩大限位。

## 7. 常见故障与验收

| 现象 | 优先检查 |
|---|---|
| `No socket connection on ...` | 网卡名、链路、`getcap`；不要直接认定网关损坏 |
| `pinocchio.casadi` 导入失败或 `.so` 架构错误 | SDK 架构、Python 3.12、CasADi 绑定、Conda/旧工作区污染 |
| CMake 找不到 Pinocchio 或运行缺动态库 | SDK 前缀、CMAKE_PREFIX_PATH、库路径，是否在新路径重新构建 |
| RealSense `No device connected` | `make camera-list` 与三相机序列号逐项对照，检查 USB/权限/占用 |
| `closed_deg is outside servo position limits` | 两侧实际开闭端点与安装配置的上下限；不能只检查报错的第一侧 |
| `resource_tracker ... leaked shared_memory` | 先看前面的真正异常；目前 VR 初始化异常可绕过共享内存清理 |
| 标定超时 | 标定伺服是否运行、ROS 域是否一致、反馈是否连续；`make watch-arm SIDE=both RATE=3` |
| 手柄方向不对 | `reference_frame`、轴符号、水平偏航校正、工具坐标与模型；勿用调增益代替方向核对 |
| 右 B 参数修改后无变化 | 改的是 `fast_*` / `fine_*` 吗，是否重启 VR，终端是否显示模式切换 |

交接验收至少记录以下结果：

- 系统版本/架构、ROS/Python、SDK 来源和前缀，构建及依赖检查结果。
- 网卡名、文件能力、ROS 通信环境；新机器有且只有一套控制进程。
- 三路相机序列号及实物对应关系、预览正常、数据目录可写。
- 双臂反馈正确，空载小幅平移/旋转方向正确，右 B 两档手感不同且不触发预设运动。
- 夹爪左右独立开闭，端点在限位内，握持和温升经现场确认；右 A 回零路径经过检查。
- 录制一小段有效示教并停止，检查数据目录；运行 `make validate-data DATASET_ROOT=/实际原始数据目录`。

将本次确认后的参数文件、验证时间、操作者和异常记录一起归档。本文提供迁移步骤，不代表新电脑已经通过硬件测试。
