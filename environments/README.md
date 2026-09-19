# 环境划分

这个项目需要两套彼此独立的运行环境。

| 环境 | 用途 | Python | 不应放入的依赖 |
|---|---|---|---|
| `robot-system` | 电机、双臂控制、VR、标定、采集 | 系统 Python 3.12，与 ROS 2 Jazzy 一致 | PyTorch、CUDA、LeRobot π0.5 |
| `towel-pi05` | 原始数据导出、数据检查、π0.5 微调 | Conda Python 3.12 | ROS 2、rclpy、EtherCAT/CAN 驱动 |

## 机器人端：系统 Python，不创建 Conda Python 环境

ROS 2 Jazzy 的 `rclpy`、生成消息和 C++ 节点以系统 Python 3.12 构建。
`dual_arm_control_hw` 还在 C++ 节点内嵌该 Python 解释器；使用 Conda 自带的
`libpython`、`libstdc++` 或不同 Pinocchio/CasADi 二进制，常见结果是导入失败或运行时崩溃。

因此建议把“机器人端环境”设为一个带系统包的 venv，而不是 Conda：

```bash
cd /home/yanziyu/projects/HW_arm/towel_task
/usr/bin/python3.12 -m venv --system-site-packages .venv-robot
source .venv-robot/bin/activate
python -m pip install --upgrade pip
python -m pip install -r environments/robot-system-requirements.txt
python -m pip install -e teleop
bash scripts/create_certificate.sh
```

每个控制终端都应按这个顺序进入环境：

```bash
conda deactivate 2>/dev/null || true
source /home/yanziyu/projects/HW_arm/towel_task/.venv-robot/bin/activate
source /home/yanziyu/projects/HW_arm/towel_task/scripts/env.sh
```

不要从已激活的 Conda Python 环境中构建或运行 ROS 节点。`scripts/build.sh` 已固定
使用 `/usr/bin/python3.12`。

还必须由系统或厂商 SDK 提供：

- ROS 2 Jazzy：`rclpy`、rclcpp、KDL、URDF、TF2、robot_state_publisher、ament。
- C++ Pinocchio；Python `pinocchio` 且 `from pinocchio import casadi` 可导入。
- CasADi/IPOPT、Meshcat、SciPy、yaml-cpp、Eigen、Boost、TBB。
- RealSense 的系统权限/udev 规则和两台相机；`pyrealsense2`。
- 与机器人处理器架构一致的 OpenRobots/Pinocchio SDK。原收到的二进制是 aarch64，不能在当前 x86_64 主机使用。

安装完成并编译后，运行只读检查：

```bash
python scripts/preflight.py --scope control
python scripts/preflight.py --scope record
```

## EtherCAT 网卡权限

`servo_node` 使用 EtherCAT 原始以太网帧，并会把配置中的网卡设为混杂模式；这需要
Linux 的 `CAP_NET_RAW` 和 `CAP_NET_ADMIN` 权限。没有这两个权限时，日志会显示
`No socket connection on <网卡名>`，即使网卡名称正确也无法扫描从站。

先确认 `ros2_ws/src/ctl_msgs_hw/config/controllerConfig.yaml` 中的
`netCard_name` 是实际接入 EtherCAT 的有线网卡。按厂家流程给网关和机械臂上电、接好
网线后，以下命令应显示 `LOWER_UP`，且不应包含 `NO-CARRIER`：

```bash
ip -br link show enp129s0   # 将 enp129s0 替换为 netCard_name 的实际值
```

`servo_node` 已嵌入工作区和 ROS 的 RPATH，因此在带文件能力的安全加载模式下仍能找到
所需动态库。`make build` 会在 ROS 构建成功后自动调用 `sudo setcap`，授予已安装伺服
可执行文件 `CAP_NET_RAW` 和 `CAP_NET_ADMIN`。因此每次运行 `make build` 都会提示输入
sudo 密码；成功时会打印 `getcap` 结果，其中应包含
`cap_net_admin,cap_net_raw=ep`。

必须从可交互输入 sudo 密码的本机终端运行 `make build`。若取消、输错密码或授权失败，
`make build` 会返回失败；重新运行它完成授权后，才可直接以当前用户运行 `make servo`、
`make teleop` 或 `make experiment VISUALIZE=true`，使所有 ROS 节点处于同一用户和 DDS
通信域。

`sudo -E make servo` 仍可作为临时诊断命令：`sudo` 让伺服节点获得原始网卡权限，`-E`
请求保留当前 ROS 域、虚拟环境和 OpenRobots 等环境变量。但 root 伺服与普通用户节点
可能无法通过 DDS 共享内存通信，不能将它作为常规标定、VR 或采集流程。

## π0.5 训练端：Conda

先创建基础 Conda 环境：

```bash
cd /home/yanziyu/projects/HW_arm/towel_task
conda env create -f environments/towel-pi05.yml
conda activate towel-pi05
python -m pip install --upgrade pip
```

然后先确认 GPU 驱动和可见显卡：

```bash
nvidia-smi
```

当前机器的 `nvidia-smi` 无法与驱动通信，不能在这里决定 PyTorch 的 CUDA 轮子。
驱动恢复后，按 PyTorch 官网与 `nvidia-smi` 显示的驱动版本选择兼容的 wheel index，再安装。
例如确认驱动满足 CUDA 12.8 的要求时：

```bash
python -m pip install torch torchvision --index-url https://download.pytorch.org/whl/cu128
python -m pip install 'lerobot[pi,training]==0.6.2'
```

`lerobot[pi,training]` 会拉入 π0.5 需要的 Transformers、SciPy、Accelerate、数据集、
PyArrow、PyAV 和 TorchCodec。训练前还需要：

```bash
hf auth login
```

并在 Hugging Face 网页接受 `google/paligemma-3b-pt-224` 的访问许可。执行 `python -c`
导入 Torch/LeRobot、检查 `torch.cuda.is_available()` 和显存后，再运行数据导出或训练。

该环境可以处理数据导出；不应 source ROS，不应连接电机、CAN 或启动 VR。
