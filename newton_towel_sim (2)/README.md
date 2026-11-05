# Newton 双臂叠毛巾仿真交付包

此目录可以完整复制到另一台电脑，独立运行。无需原项目、ROS 工作区、真机驱动或 Isaac Sim。
交付快照日期：2026-11-05。使用新版 `dual_arm/urdf/dual_arm.urdf`，包含双臂、旋转夹爪和相机外壳。

## 从这里开始

1. 按 [环境配置](docs/ENVIRONMENT.md) 创建 Conda 环境并安装依赖。
2. 在本目录执行 `make verify`、`make check`，确认文件和 CUDA 正常。
3. 执行 `make sim` 查看模型，或执行 `make sim-fold` 自动完成一次双臂对折。
4. VR 使用额外的 `towel-vr` 环境，连接步骤见 [使用说明](docs/USAGE.md)。

```bash
make sim          # 桌面交互、关节滑块、夹爪及三路相机
make sim-fold     # 自动抓角，执行一次对折
make sim-vr       # 仅 VR 动作输入，不向头显发送画面
make test         # GPU 检查 + 5 项场景测试 + 6 项 VR 测试
```

## 包含什么

| 目录 | 内容 |
|---|---|
| `simulation/` | Newton 场景、自动折叠、VR 控制、模型加载和 `scene.yaml` |
| `dual_arm/` | 新版 URDF 和其实际引用的全部 STL；不需要 USD |
| `teleop/` | 仿真使用的 WebXR 输入适配、坐标映射及配置；不含真机发布器 |
| `environments/` | 两套 Conda 环境定义、固定直接依赖和已验证环境的依赖快照 |
| `scripts/` | GPU 检查、证书生成、校验和打包 |
| `tests/` | 场景、VR 映射与完整对折、WebXR 通信测试 |
| `docs/` | 环境、操作、实现边界与验收记录 |
| `MANIFEST.json` | 交付文件 SHA-256 清单 |

毛巾为 30 × 30 cm，机器人采用限速运动学控制，布料使用 VBD 求解，抓取采用节点绑定辅助。
自动策略读取已知网格角点并按规则对折，没有视觉识别、训练策略或纯摩擦抓持。
仿真回零使用 `scene.yaml` 的初始姿态，三个 VR 旋转自由度均开放。

详细说明：[环境](docs/ENVIRONMENT.md)、[操作](docs/USAGE.md)、[程序结构](docs/ARCHITECTURE.md)、
[验收记录](docs/VALIDATION.md)、[第三方来源](THIRD_PARTY_NOTICES.md)。

## 再次交付

修改后运行 `make test`，更新验收记录，再执行 `make package`。
将生成在上一级目录的 `newton_towel_sim.tar.gz` 和 `.sha256` 文件一并交付。
打包自动排除运行输出、缓存和证书私钥；接收方在自己的机器生成证书。
该包包含源代码和模型，不包含 Conda 本体、Python wheel 或 GPU 驱动，因此首次安装需要联网。
