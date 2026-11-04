# Newton 仿真环境（Conda）

这一阶段配置独立的 `towel-newton` 环境，运行官方 Franka 与布料示例。
示例使用 Franka 和一件衣物，不是本项目的双臂叠毛巾场景。
项目双臂、夹爪与矩形毛巾已另行接入，使用 `make sim`，见 [场景说明](../simulation/README.md)。

Conda 管理 Python 环境，环境内的 pip 安装 Newton 官方 Python 包。
版本固定为 Newton 1.7.0、Warp 1.18.0，见 `newton-requirements.txt`。
官方安装说明：https://newton-physics.github.io/newton/latest/guide/installation.html

## 1. 检查主机与 GPU

在项目根目录、可访问 GPU 的桌面终端执行：

```bash
nvidia-smi
conda --version
```

本机已确认：Ubuntu 24.04 x86_64、RTX 5080 Laptop GPU（约 16 GB）、驱动 595.99.02。
Warp 的 wheel 已带所需运行组件，本流程不额外安装系统 CUDA Toolkit，也不修改驱动。
沙箱内可能看不到 `/dev/nvidia*`，应以主机终端和 CUDA 检查结果为准。

## 2. 创建 Conda 环境

以下创建步骤只需执行一次，已经存在时直接激活：

```bash
conda env create -f environments/towel-newton.yml
conda activate towel-newton
python --version
```

使用 Python 3.12。仿真终端不需要激活 `.venv-robot` 或 source `scripts/env.sh`。

## 3. 安装 Newton 与示例依赖

```bash
python -I -m pip install -r environments/newton-requirements.txt
python -I -m pip check
```

`newton[examples]` 包含官方示例所需的可视化、URDF/USD 导入等依赖。
首次安装需要下载较大的网格处理库。这里不安装策略训练用的 PyTorch。

## 4. 验证 CUDA 计算

```bash
make sim-check
```

命令通过 `conda run -n towel-newton` 调用专用环境，不要求先手动激活。
它会导入 Newton 和 USD，编译一个 Warp CUDA kernel，在 GPU 上运行并检查计算结果。
最后应看到 `PASS: Newton/USD imports and compiled CUDA kernel ...`。
首次执行需要编译，后续会复用 Warp 缓存。

## 5. 运行官方机器人布料示例

先进行无窗口的计算验证：

```bash
make sim-demo ARGS="--viewer null --test"
```

这会运行示例默认的 3850 帧并执行官方状态检查，完成后自动退出。
然后在桌面终端打开交互窗口：

```bash
make sim-demo
```

首次运行会下载 Franka 模型并编译物理内核，加载时间长于之后的启动。
应看到机械臂、桌面和布料，仿真中布料发生运动。关闭窗口即可退出。
示例用于验证依赖、求解器和渲染链路，不代表本项目已完成叠毛巾任务。

手动激活环境后的等价命令：

```bash
conda activate towel-newton
python -I -m newton.examples cloth_franka --device cuda:0
```

自定义 Conda 环境名时使用 `make sim-check SIM_ENV=你的环境名`，`sim-demo` 同理。
`python -I` 用来隔离外部 `PYTHONPATH` 和用户 Python 包。

## 本机验证记录

2026-11-03，Conda `towel-newton`：Python 3.12.16、Newton 1.7.0、Warp 1.18.0。

- `pip check`：依赖无冲突。
- `make sim-check`：Newton/USD 导入及 GPU kernel 结果检查通过。
- `make sim-demo ARGS="--viewer null --test"`：3850 帧官方完整测试通过。
- OpenGL 窗口运行 120 帧，机械臂、桌面、布料可见；布料状态和画面变化检查通过。

以上验证的是官方示例和安装环境；本项目双臂与毛巾的独立场景验证见 [场景说明](../simulation/README.md)。
