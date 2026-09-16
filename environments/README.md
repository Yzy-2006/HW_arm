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
