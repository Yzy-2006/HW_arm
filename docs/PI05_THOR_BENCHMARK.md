# Thor PI05 推理测速

目标是测量当前 OpenPI/JAX、BF16、batch=1 推理实现在 Thor 上的串行速度。
测速不启动 ROS、相机或机械臂，不发送动作。这不是量化、TensorRT 或并发 batch 的性能上限。

## 环境前提

完整依赖表、NVIDIA 容器与 Conda 安装路线、GPU 自检和模型验收步骤见
[Thor 推理环境文档](../environments/PI05_THOR.md)。

先在 Thor 记录以下输出：

```bash
uname -m
cat /etc/nv_tegra_release
nvidia-smi
nvcc --version
docker --version
```

Thor 需要兼容其 JetPack、CUDA 和 aarch64 的 JAX 运行环境；不能直接复制笔记本 x86_64
的 Conda 环境。模型的 `requirements-inference.txt` 固定了 `jax[cuda12]==0.5.3`、
Flax 0.10.2 和 Orbax 0.11.13。选择新版 JAX/CUDA 13 环境时，还需验证随模型发布的 OpenPI
代码与新版 Flax、Orbax 的兼容性；不要在 NVIDIA 已配好的容器中盲目安装整份旧依赖覆盖 GPU 栈。

安装路线应根据上面的系统信息选定。可参考
[JAX 安装文档](https://docs.jax.dev/en/latest/installation.html)和
[NVIDIA JAX Toolbox](https://github.com/NVIDIA/JAX-Toolbox)。

将仓库和完整的模型根目录放到 Thor；模型需包含 `inference.py`、`runtime/`、
`tokenizer/`、checkpoint 的 `params/` 和 `assets/` 归一化统计。测试不需要编译 ROS 工作空间。

在选定的推理环境中，先验证非 CPU 的真实 GPU 计算：

```bash
python - <<'PY'
import jax
import jax.numpy as jnp
print("JAX:", jax.__version__, "devices:", jax.devices())
assert any(d.platform == "gpu" for d in jax.devices()), "JAX cannot see a GPU"
x = jnp.ones((1024, 1024), dtype=jnp.bfloat16)
y = (x @ x).block_until_ready()
print("GPU matmul OK:", y.shape, y.dtype)
PY
```

## 功耗与温度

记录功耗模式并保持同一模式比较：

```bash
sudo nvpmodel -q --verbose
sudo jetson_clocks --show
```

需要测试锁频性能时，先选好适合自己硬件与散热的功耗模式，再执行：

```bash
sudo jetson_clocks --store
sudo jetson_clocks
```

NVIDIA 文档说明锁频上限取决于 nvpmodel，执行后切换功耗模式可能需要重启；
MAXN 也不保证持续最快，需要比较实际降频和温度。见
[Thor 功耗与性能文档](https://docs.nvidia.com/jetson/archives/r38.4/DeveloperGuide/SD/PlatformPowerAndPerformance/JetsonThor.html)。

另开终端记录持续监控，完成后 Ctrl+C：

```bash
tegrastats --interval 1000 --logfile /tmp/pi05-thor-tegrastats.log
```

## 基准测试

### U 盘传输模型与脚本

在模型所在的电脑上执行，不要 source 该脚本：

```bash
bash scripts/pack_pi05_thor.sh /media/yanziyu/6745-2301
```

脚本先在本机 `outputs/pi05-thor-transfer.*` 目录打包并生成 SHA256，再复制到 U 盘
`pi05-thor-offline/`。只有 U 盘文件与本地校验和一致且同步成功才显示 `TRANSFER COMPLETE`。
超大文件复制期间可能暂时没有新输出，需等待。本机和 U 盘都需有足够空间容纳约 12 GB 权重包。

出错时保留本地打包结果和 `transfer.log`，U 盘未验证的文件使用 `.partial` 后缀，不要将其用于部署。
只读挂载或 I/O 错误需先排查 U 盘和连接。取下 U 盘前使用文件管理器的安全弹出功能。

到 Thor 后先在 `pi05-thor-offline/` 目录执行 `sha256sum -c SHA256SUMS`，两项都为 OK 后
将模型包解压到模型根目录，代码包解压到仓库根目录。该传输包含环境文档和依赖清单，
不包含可离线安装的 ARM64 wheel 或 Docker 镜像，需另行准备。

### 运行测速

先停止其他 worker 和训练进程。不要启动 `make pi05` 或 `make motion`；测速程序自己加载模型。

如果 Thor 使用名为 `pi05-openpi` 的 Conda 环境，在仓库根目录运行：

```bash
make pi05-bench PI05_CHECKPOINT=step-49999
```

模型不在默认位置时增加 `PI05_MODEL=/absolute/model/root`；环境名称不同时增加 `PI05_ENV=...`。
默认读取 YAML 的 `worker.num_steps` 和 `worker.action_horizon`，每个模式首次调用后再预热 5 次，
连续测量 100 次，保存到 `outputs/pi05_benchmark.json`。每次运行会覆盖同名报告，比较测试时指定不同的 `--output`。

如果使用容器或已激活的推理环境，直接执行以下命令，不需要 Conda：

```bash
python deployment/benchmark_pi05.py \
  --config config/pi05_deployment.yaml \
  --model "$HOME/models/pi05-encos-fold-towel-93" \
  --checkpoint step-49999 \
  --warmup 5 --iterations 100 --mode both \
  --output outputs/thor-pi05-baseline.json
```

两种模式的范围：

| 模式 | 计时包含 | 不包含 |
| --- | --- | --- |
| `policy` | 图像缩放、文本 tokenization、归一化、GPU 推理、后处理和结果返回 CPU | JPEG 编解码、TCP/JSON、相机、ROS |
| `worker` | 现有 `PolicyRunner.infer` 路径，包括 Base64/JPEG 解码、policy 推理和动作列表转换 | JPEG 编码、TCP/JSON、相机、ROS |

默认是固定的三路 RGB 测试图像，尺寸读取相机配置，状态为零，只用于速度测量，不代表任务效果。
这些输入不会发送到机器人。有真实观测 NPZ 时可增加 `--input /path/observation.npz`，需包含：

```text
observation.state                 float32[16]
observation.images.head           uint8[H,W,3], RGB
observation.images.left_wrist     uint8[H,W,3], RGB
observation.images.right_wrist    uint8[H,W,3], RGB
prompt                            optional scalar string
```

## 积分步数与持续负载

先测 YAML 原有的积分步数，再比较 20、10、5 步。下面依次启动和退出独立进程，每次都释放显存：

```bash
for steps in 20 10 5; do
  make pi05-bench PI05_CHECKPOINT=step-49999 \
    BENCH_ARGS="--num-steps $steps --warmup 5 --iterations 100 --min-seconds 60 --mode both --output outputs/thor-pi05-steps-$steps.json" || break
done
```

`--min-seconds 60` 表示每个模式至少测 60 秒，且至少达到 `--iterations` 指定的次数。
长时测试可使用 `--min-seconds 300 --mode worker`。不同积分步数并不保证相同的动作质量，
不应将最少步数的最快结果当作原配置的速度。所有测试保持 action horizon=50。

## 结果解读

- `model_load_s`：加载权重的时间，不计入稳态推理。
- `first_call_ms`：该模式的首次调用，可能包含 JAX 编译；`both` 的第二个模式会复用已编译的模型。
- `mean_ms` / `p50_ms` / `p95_ms` / `p99_ms`：完整一次推理返回 50 步动作的耗时，不是单个动作的耗时。
- `serial_inferences_per_s`：连续串行调用的实测频率，是本次配置的主要速度指标。

例如完整推理为 100 ms，串行约为 10 次/秒；不能用 `50 / 0.1 = 500` 来声称闭环推理达到 500 Hz。
不要只看最小耗时，也要结合 p95/p99、温度和功耗日志判断持续性能。

结果转换为 NumPy 后才停止计时，等待 GPU 计算完成，避免 JAX 异步提交导致假快。见
[JAX 异步调度文档](https://docs.jax.dev/en/latest/async_dispatch.html)。

bridge 的 `replan_interval_s`、`rate_hz` 和相机同步都不参与此测速。最后再用真实相机和
`make bridge` 检查系统延迟，它包含通信和调度开销，不等同于模型本身的极限速度。
