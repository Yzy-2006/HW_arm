# Thor PI05 测速环境

本文配置的是已发布 OpenPI/JAX/Orbax PI05 的离线推理和测速环境，不需要 ROS、
RealSense、EtherCAT 或 LeRobot。Thor 是 aarch64，不要复制笔记本的 x86_64 Conda 环境。

验证状态：测速程序的计时、预热、输入与输出检查已在本地测试；本文的 Thor 安装路线需在目标
机上执行下述 GPU 和模型验收，尚未是已实测的 Thor 环境锁文件。

## 需要哪些软件

| 组件 | 要求 |
| --- | --- |
| 系统 | Thor 对应的 JetPack/Jetson Linux，Linux aarch64 |
| NVIDIA 驱动和 CUDA | 使用与 JetPack 匹配的组合，不要在 Jetson 上按普通 PC 方式替换显卡驱动 |
| Python | 3.11 或 3.12；NVIDIA 容器使用自带的 Python |
| JAX/JAXlib/GPU plugin | 需匹配 aarch64、CUDA 和 Thor GPU；必须完成 GPU BF16 矩阵乘法自检 |
| Flax NNX | 需有 `nnx.Module` / `nnx.bridge.ToNNX` / `nnx.state` / `nnx.split` / `nnx.merge` |
| Orbax / TensorStore | 需能读取该模型的 OCDBT checkpoint |
| PyTorch / torchvision | 模型运行时的导入依赖；PI05 的 GPU 计算在 JAX，此处可用 CPU 版 PyTorch |
| 应用依赖 | 见 `environments/pi05-thor-extra-requirements.txt`，包含 tokenizer、图像、YAML 和运行时依赖 |
| 模型文件 | 完整的 `inference.py` / `runtime/` / `tokenizer/` / checkpoint `params/` 和 `assets/` |

发布模型的原环境依赖是 JAX 0.5.3 + CUDA 12、Flax 0.10.2、Orbax 0.11.13、NumPy 1.26.4。
这是原模型代码的兼容性参考，不是对 Thor 可直接使用的保证。

## 路线 A：NVIDIA JAX 容器

适合已配置 Docker 和 NVIDIA Container Toolkit 的 Thor。先查看本机信息：

```bash
uname -m
cat /etc/nv_tegra_release
nvidia-smi
nvcc --version
docker --version
```

使用兼容本机 JetPack/驱动的 NVIDIA JAX ARM64 镜像。例如官方 25.10 发布使用
JAX 0.7.2、Flax 0.11.2、CUDA 13.0.2、Python 3.12，可作为固定版本的候选基底：

```bash
docker pull nvcr.io/nvidia/jax:25.10-py3
docker image inspect nvcr.io/nvidia/jax:25.10-py3 --format '{{.Architecture}}'
```

确认是 `arm64`，并查看
[NVIDIA 发布说明](https://docs.nvidia.com/deeplearning/frameworks/jax-release-notes/rel-25-10.html)
的驱动要求。若本机不匹配或该标签没有可用 ARM64 manifest，改用官方支持当前 Thor 系统的固定版本
镜像，不要通过 x86 模拟运行。NVIDIA 的
[JAX Toolbox](https://github.com/NVIDIA/JAX-Toolbox) 提供了更新的镜像和 Thor 测试信息。

在仓库根目录执行；模型目录挂载为只读，结果保存到挂载的仓库：

```bash
docker run --rm -it --gpus all --ipc=host \
  -v "$PWD:/workspace/towel_task" \
  -v "$HOME/models/pi05-encos-fold-towel-93:/models/pi05:ro" \
  -w /workspace/towel_task \
  nvcr.io/nvidia/jax:25.10-py3 bash
```

如果本机使用的 NVIDIA runtime 配置需要 `--runtime nvidia`，按本机 Container Toolkit 配置添加。
容器内先执行下文的 GPU 自检，再安装应用依赖。本例为临时容器，退出会删除容器内的安装；
验证成功后应用 Dockerfile 固化镜像和依赖，并记录镜像 digest。

## 路线 B：独立 Python 环境

不使用容器时，可在 Thor 的 ARM64 Conda/Miniforge 中创建环境：

```bash
conda create -n pi05-openpi python=3.11 -y
conda activate pi05-openpi
python -m pip install --upgrade pip
```

对兼容 CUDA 13 的驱动/系统，可以从官方 JAX ARM64 wheel 路线开始：

```bash
python -m pip install 'jax[cuda13]==0.7.2' 'flax==0.11.2' 'orbax-checkpoint>=0.11.13,<0.12'
```

这是候选组合，需验证旧 OpenPI 运行时的 NNX/Orbax API，不是已验证的锁版组合。若安装器提示
版本冲突，不要用 `--no-deps` 强行忽略，应选择相容的 JAX/Flax/Orbax 版本组合。
具体驱动、CUDA/cuDNN 要求和使用本机 CUDA 的 `cuda13-local` 路线见
[JAX 安装文档](https://docs.jax.dev/en/latest/installation.html)。

## GPU 自检

在选定的容器或 Conda 环境内执行：

```bash
python - <<'PY'
import jax
import jax.numpy as jnp
print("JAX:", jax.__version__, "devices:", jax.devices())
assert any(d.platform == "gpu" for d in jax.devices()), "No JAX GPU"
x = jnp.ones((1024, 1024), dtype=jnp.bfloat16)
y = (x @ x).block_until_ready()
print("GPU BF16 matmul OK:", y.shape, y.dtype)
PY
```

必须同时看到 GPU 设备和 `GPU BF16 matmul OK`。仅 `nvidia-smi` 成功不代表 JAX 与 CUDA 已兼容。

## 安装应用依赖

先记录基底环境作为约束，避免应用包静默替换已能运行的 GPU 栈：

```bash
python -m pip freeze > /tmp/pi05-thor-base-constraints.txt
python -m pip install -c /tmp/pi05-thor-base-constraints.txt \
  -r environments/pi05-thor-extra-requirements.txt
```

约束冲突时应检查该包与基底版本，不要直接删除约束让 pip 替换 JAX。依赖文件不固定
JAX、Flax、Orbax、TensorStore、NumPy、ml-dtypes 和 CUDA plugin，这些由基底环境提供。

若环境已有可导入的成对 PyTorch/torchvision，保留即可。如果没有，可安装 CPU 版以满足模型的导入依赖：

```bash
python -m pip install -c /tmp/pi05-thor-base-constraints.txt \
  torch==2.9.1 torchvision==0.24.1 --index-url https://download.pytorch.org/whl/cpu
python -m pip check
python -c 'import torch, torchvision, yaml; print(torch.__version__, torchvision.__version__)'
```

这里 `torch.cuda.is_available()` 为 False 可以是正常的；实际 PI05 计算使用 JAX，以 JAX GPU 自检为准。

## 模型加载与一次推理验收

在仓库根目录执行一次小规模测试。使用容器时模型路径是 `/models/pi05`；宿主机上换成实际目录：

```bash
python deployment/benchmark_pi05.py \
  --config config/pi05_deployment.yaml \
  --model /models/pi05 \
  --checkpoint step-49999 \
  --mode policy --warmup 1 --iterations 1 \
  --output outputs/thor-pi05-smoke.json
```

验收标准：权重加载成功，推理返回有限数值的 `[50,16]` 动作，并保存 JSON。之后才开始
[100 次推理和持续负载测速](../docs/PI05_THOR_BENCHMARK.md)。

如果报 `flax.nnx` / `ToNNX` / `State` / Orbax metadata 或布局 API 错误，这是发布模型运行时与基底包
的兼容性问题，不能得出 Thor 上模型速度的结论，需先配齐版本或适配相应 API。

通过验收后，固化完整依赖以便复现：

```bash
mkdir -p outputs
python -m pip freeze > outputs/thor-pi05-requirements-tested.txt
sudo nvpmodel -q --verbose
```

容器环境中的 `nvpmodel` 如不可用，在 Thor 宿主机运行该查询命令。同时保留镜像 digest、
JetPack 版本、功耗模式和测速 JSON，不要只保留一个延迟数字。
