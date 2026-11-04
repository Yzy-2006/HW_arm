# 部署时 VR 接管与补采

适用于当前 `pi05_bridge.py` 控制路径，包括通过该桥部署的 PI05/ACT worker。
正常接管时模型和 VR 不必重启。左 B 键暂停模型、切换 VR 并开始补采；再次按左 B 键停止 VR、
保存 episode，再以最新观测恢复模型。普通部署和普通 VR 的行为保持原有入口。

## 启动

控制消息和三层控制器均已修改，需要一起构建并重启。不要在机器人正在运动时重启。
底层故障会进入 `fault`，禁止接管和恢复；复位条件、阻尼输出的限制见
[控制安全说明](CONTROL_SAFETY.md)。以下仅是构建命令，不启动硬件：

```bash
source scripts/env.sh
colcon build --base-paths ros2_ws/src --build-base ros2_ws/build --install-base ros2_ws/install --packages-select ctl_msgs_hw dual_arm_control_hw robot_control_hw servo_control_hw
```

按现有流程启动 `make servo`、`make arms`、`make robot` 和模型 worker。在两个独立终端执行：

```bash
# 终端 1：先启动 VR 并连接头显；在提示处按 Enter。
make vr ARGS="--intervention"

# 终端 2：部署桥和内置补采器，共享同一组相机。
make motion ARGS="--intervention --intervention-dir data/raw/interventions"
```

不要同时运行普通 `make vr`、`make record`、回放节点或其他动作发布器。
内置补采器提供原有 `/towel/record`、`/towel/discard` 服务；单独启动采集器会争抢相机和服务。

干预部署初始状态为 `paused`，不会自动运行模型。确认反馈和相机就绪后，在已加载 ROS 环境的终端执行：

```bash
ros2 service call /towel/intervene std_srvs/srv/SetBool '{data: false}'
ros2 topic echo /towel/intervention/state
```

服务返回的是“已接受切换”，以状态话题进入 `policy` 或 `vr` 为完成标志。

## 现场操作

| 操作 | 结果 |
| --- | --- |
| 模型运行时按左 B | 停止模型输出、作废缓存轨迹；控制器从当前末端反馈重新锚定 VR；补采成功启动后进入 `vr` |
| VR 中扣动左右扳机 | 沿用现有增量遥操作 |
| VR 中按左右 squeeze | 沿用现有夹爪开闭控制 |
| VR 中再次按左 B | 控制器先保持当前位置，保存本段补采，随后模型从新观测重新推理 |
| 干预模式的左 A / 右 A | 禁用丢弃和回零快捷键，避免误删除补采或离开失败现场 |
| 右 B | 沿用快/精细灵敏度切换 |

也可用服务替代左 B：`data: true` 请求接管补采，`data: false` 请求保存并恢复模型。
补采失败后状态停在 `paused`，先查看部署日志；修复后显式重新请求接管或恢复。
超时请求未收到最终应答前不接受另一项切换，避免迟到的服务改变控制权。

VR 接管时会清除旧的手柄位姿基准和关节插值缓存。控制器拒绝过期、非有限或超出 VR
工作空间限位的末端反馈，不会为了接管直接回零或将姿态裁到边界。
VR 权限心跳超过 0.3 秒未刷新时，手柄不再发布增量和夹爪目标。软件切换不是硬件急停，
也不承诺硬实时停机延迟；首次验证应在受控环境中低速进行。

## 数据与导出

默认补采目录为 `data/raw/interventions/`，每次成功接管生成一个独立 episode：

- 三路相机图像、14 维关节反馈及控制目标、TCP、夹爪状态继续使用原始采集格式。
- `episode.json` 标记 `source: policy_intervention`、`control_source: vr`，保留部署配置和标定。
- 补采固定保留静止帧，便于学习等待、夹爪动作和恢复动作的时序。
- 保存过短的片段、相机/反馈/写盘异常或直接退出时保留 `.partial`，不会被正常导出。
- `Ctrl+C` 不等同于保存；正常结束补采使用左 B 或 `data: false`。

```bash
make validate-data DATASET_ROOT="$PWD/data/raw/interventions"
make export-data DATASET_ROOT="$PWD/data/raw/interventions" OUTPUT="$PWD/data/lerobot/interventions"
```

导出前查看日志中的保存路径、episode 帧数和采样诊断。成功接管仅说明切换和采集已就绪，
恢复动作是否适合加入训练集仍需按原有数据检查流程筛选。
