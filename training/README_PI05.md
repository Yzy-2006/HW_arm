# 后续 π0.5 微调

目前交付的是采集与转换接口。没有下载权重、启动训练或接管机器人。

## 数据为何这样保留

π0.5 可以从图像、机器人状态和任务指令学习动作，LeRobot 提供 `pi05` 训练入口。
本项目导出 `observation.state`、`action`、两路图像和任务文本，具体字段见 ../docs/DATASET.md。

默认动作是 14 个关节位置目标 + 2 个夹爪二值目标（0=开，1=闭）。
观测中夹爪位是反馈分类，动作中是操作者要求的目标，二者可能在运动中不同。
夹爪位不做连续角度差分；未来若关节使用相对动作，应排除两个 gripper 维度并重算统计。
原始 TCP 目标仍保留，可用于另外选择末端动作表示。

## 训练环境

使用独立 GPU 训练环境，安装 LeRobot 的 π 依赖，固定实际使用的版本/commit：

```bash
pip install 'lerobot[pi]'
```

不要为训练升级正在控制硬件的 ROS Python 环境。本地原目录的 LeRobot HEAD 为
`3f2c29ef7e44b1ddccbcda3b6a63939e53639e9e`，但其工作树可能有本地改动，不能用该 SHA 代表整个原目录。
本目录没有隐式依赖该副本。导出工具采用公开的 `LeRobotDataset.create/add_frame/save_episode/finalize` API。

## 示例（待真实数据与 GPU 验证）

先根据主 README 导出并检查数据，在训练环境中用新版本 CLI 的 `--help` 核对参数。
以下示例从基础权重初始化，让模型特征从自定义数据集构建；不是已完成训练的证明。

```bash
lerobot-train \
  --dataset.repo_id=local/towel_wipe \
  --dataset.root=/absolute/path/to/towel_task/data/lerobot/towel_wipe \
  --policy.type=pi05 \
  --policy.pretrained_path=lerobot/pi05_base \
  --policy.empty_cameras=1 \
  --policy.normalization_mapping='{"ACTION":"MEAN_STD","STATE":"MEAN_STD","VISUAL":"IDENTITY"}' \
  --policy.gradient_checkpointing=true \
  --policy.device=cuda \
  --policy.dtype=bfloat16 \
  --policy.push_to_hub=false \
  --batch_size=4 \
  --steps=30000 \
  --output_dir=outputs/pi05_towel \
  --job_name=pi05_towel
```

这里 mean/std 避免依赖旧导出器未必生成的分位数统计。
默认只把 `observation.state` 和图像用于模型输入；其他原始/辅助字段用于检查或未来适配。
按整段演示保留验证集；示例中的步数和 batch size 需按数据量与硬件调整。

将来部署需要编写 ENCOS 策略执行适配器，把模型的关节目标和夹爪开合目标接入相应控制层（模型开合输出需阈值化，映射到已标定的开/闭端点），
并处理控制模式、限幅、命令中断和状态反馈；不能把这 16 维动作直接发给原 6 维右臂白板桥接器。

## 参考（2026-09-15 查阅）

- [LeRobot π0.5 文档](https://huggingface.co/docs/lerobot/pi05)：安装、基础权重微调、统计与特征映射。
- [Physical Intelligence openpi](https://github.com/Physical-Intelligence/openpi)：π0.5 基础模型和自定义数据适配路径。

这里提供的是数据接入准备；换成 openpi 训练还需按其版本实现数据重打包、动作映射和归一化配置。
