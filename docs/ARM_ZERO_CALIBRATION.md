# 机械臂零位与预设姿态标定

本流程用于此前没有建立零位的单臂或双臂。它把你手动摆好的姿态记录为后续
`make servo` 的自动回零目标和 `make robot` 的遥操作初始位姿。默认同时保存关节与
笛卡尔零位，VR A 键也回到同一关节零位。只在有现场人员、可立即触及急停、且确认机器人机械
状态安全时执行。

右手 B 键的预设姿态也使用本流程采样，保存时选择 `TARGET=preset`。
它只更新独立的 `vr_preset_pose.yaml`，不修改启动零位、右 A 回零或遥操作初始末端位姿。

> **危险：** `make calibrate-arm-zero SIDE=both` 会让双臂 14 个关节进入**零电流**，
> 不执行自动回零，也不会保持位置。双臂可能因重力下垂或突然转动；必须由两人分别
> 托住左右臂、移除碰撞风险并保持手在急停附近。单臂模式下，`SIDE=left` 仅让左臂
> 零电流、右臂以启动反馈位置低刚度保持；`SIDE=right` 则相反。不要手动推拉保持侧。
> 单臂模式要求保持侧至少有一项有效的非零角度反馈；若反馈全为零，程序会拒绝启动，
> 避免把该侧误保持到全零位。此时如需仅保存另一侧的零位，可在两人扶稳双臂的前提下
> 使用 `SIDE=both` 启动，再用相应的 `save-arm-zero SIDE=left` 或 `SIDE=right` 单独保存。

## 开始前

1. 确认现场急停可用，两臂运动范围没有人、线缆或桌面障碍物；双臂标定时安排两人
   分别全程托住左右臂。不要靠软件退出或 `Ctrl+C` 作为急停。
2. 给双臂和 EtherCAT-8CAN 网关上电。确认网线连接正确、EtherCAT 网卡已经启用，
   并确认双臂全部电机的**绿灯**亮起；若绿灯不亮，先排查供电、急停、线缆和网关，
   不要运行本流程。
3. 在项目根目录运行一次 `make build`。构建结束会请求 sudo 密码，为新编译的
   `servo_node` 配置 EtherCAT 所需能力。
4. 标定前先停止遥操作终端中的 `make servo`、`make arms`、`make robot`、`make record`、
   `make vr`，以及 VR 调试程序或其他向
   `/dexbotCtl/ctl_data_pub` 发布的节点。必要时先在已有终端 `Ctrl+C` 退出。

## CAN 通道定位（反馈异常时）

若不知道左臂接到网关哪个 CAN 接口，先停止所有 `make servo` 和标定进程，确保双臂
有人托住，再单独运行：

```bash
make scan-arm-can
```

该命令持续约两秒，扫描协议支持的 **CAN0–CAN2**，对每个通道的 ID 1–8 仅发送
**零电流**请求并统计回包，例如 `CAN1: ID1=... ID8=...`。它不发送位置目标、不回零、
**不保存零位**，结束后会自动退出。只有一个 `servo_node` 可以运行；若结果中某通道的
各 ID 都有回包，该通道连接着对应的电机链。扫描期间仍可能因重力下垂，必须持续托住
双臂并保持急停可用。

## 标定步骤

使用三个终端，均从项目根目录执行。

```bash
cd /home/yanziyu/projects/HW_arm/towel_task
```

保存时用 `TARGET` 选择修改哪个文件；`SIDE` 仍表示修改哪一侧机械臂：

| 保存选项 | 修改文件 | 用途 |
| --- | --- | --- |
| `TARGET=both`（默认） | `zeropos.yaml` 和 `desc_zeropos.yaml` | 统一初始回零与遥操作零位 |
| `TARGET=joint` | 仅 `zeropos.yaml` | 修改伺服启动回零和 VR A 键关节回零 |
| `TARGET=cartesian` | 仅 `desc_zeropos.yaml` | 修改 `make robot` 的遥操作初始末端位姿 |
| `TARGET=preset` | 仅 `vr_preset_pose.yaml` | 修改 VR 右 B 键的双臂预设关节姿态 |

例如 `make save-arm-zero SIDE=right TARGET=cartesian` 只修改右臂的末端零位。
只更新一个文件会允许两种零位不同；需要保持一致时使用 `TARGET=both`。
`TARGET` 在保存命令上选择，启动被动标定模式的命令不变。

两种零位由同一批稳定关节反馈计算：关节角直接写入 `zeropos.yaml`，末端位姿使用
控制器相同的 Pinocchio 和 URDF 做正运动学后写入 `desc_zeropos.yaml`。坐标系和
末端链配置读取 `controllerConfig.yaml`，默认是 `arm_base_link` 到
`arm_l_link_8` / `arm_r_link_8`，单位为米和 RPY 弧度。
此过程只需被动伺服反馈，不需要启动 `arms` 或 `robot`。

1. **终端 1：启动被动标定模式。** 两人仍分别托住双臂，运行：

   ```bash
   make calibrate-arm-zero SIDE=both
   ```

   启动日志必须包含 `both-arm calibration: zero-current`。该模式仍会发布
   `/dexbotCtl/dualArmCtl_fdback`，但忽略普通上位机控制和自动回零。如果看到
   EtherCAT 没有从站、任一侧电机绿灯熄灭或任一臂反馈始终全为零，立刻停止并修复
   供电/通讯，不要保存零位。

2. **终端 2：确认反馈。** 运行：

   ```bash
   make watch-arm SIDE=both RATE=3
   ```

   终端每秒显示约三行，每行分别包含左右臂各七个关节角度（rad）。在两人分别托住
   双臂的情况下轻微移动各一关节，确认两侧对应数值都会变化。任一侧数据不变化、长度
   不是七，或全部为零时，返回“开始前”检查该侧上电、绿灯和 CAN 通道。

3. **手动摆臂。** 双臂模式下两臂均为零电流，两人持续托住各自机械臂；单臂模式下只
   能摆动被选中的零电流侧，另一侧保持不动。缓慢摆到希望普通
   `make servo` 启动后回到的初始姿态；标定右 B 时则摆到希望按 B 后到达的姿态。
   确认关节与线缆没有干涉，然后把双臂完全保持静止。

4. **终端 3：采样并保存。** 标定双臂时，双臂均保持静止后运行：

   ```bash
   make save-arm-zero SIDE=both TARGET=both
   ```

   这会同时更新两个文件。若只想更新一个文件，在同一个保存步骤中改用以下
   **其中一条**命令，不要把不同保存命令连续执行：

   ```bash
   # 只修改初始关节回零及 VR A 键回零目标
   make save-arm-zero SIDE=both TARGET=joint

   # 只修改遥操作初始末端位姿
   make save-arm-zero SIDE=both TARGET=cartesian

   # 只修改右 B 键预设姿态
   make save-arm-zero SIDE=both TARGET=preset
   ```

   要分别标定时，可用下面对应的一组命令。启动命令在终端 1 运行，保存命令在
   终端 3 运行；不要在标定节点尚未退出的终端中接着输入保存命令：

   ```bash
   # 左臂零电流，右臂保持；同时更新两个文件的 left_arm
   make calibrate-arm-zero SIDE=left
   make save-arm-zero SIDE=left TARGET=both

   # 右臂零电流，左臂保持；同时更新两个文件的 right_arm
   make calibrate-arm-zero SIDE=right
   make save-arm-zero SIDE=right TARGET=both
   ```

   单臂保存也可将 `TARGET` 改成 `joint`、`cartesian` 或 `preset`。保存的 `SIDE` 必须是
   当前标定模式下的零电流侧；以 `SIDE=both` 启动时，可以只保存左侧或右侧。

   命令会采集至少 1 秒、至少 20 个连续反馈。任何被保存关节在窗口内变化超过 0.2 rad、
   消息中断超过 0.2 秒、位置不是七个有限数，或该侧七个位置全为零时，命令会失败且
   **不会修改**配置。`SIDE=left` 与 `SIDE=right` 只修改对应侧；`SIDE=both` 会在两侧
   都验证通过后一次写入。失败后保持托举，排除原因，再重新运行保存命令。

5. 成功后，终端会打印采集的关节角、保存的末端位姿以及每个文件的备份路径。
   默认新值写入这两个源码配置：

   ```text
   ros2_ws/src/ctl_msgs_hw/config/zeropos.yaml
   ros2_ws/src/ctl_msgs_hw/config/desc_zeropos.yaml
   ```

   保存前的原文件会位于同目录，文件名类似
   `zeropos.yaml.1770000000000000000.backup`。两文件同时保存时使用相同备份时间戳。
   所有采样、正运动学和 YAML 校验完成后才写入，文件逐个原子替换；遇到替换错误会
   回滚已替换文件。两文件不是文件系统级原子事务，断电或强制杀进程后需核对二者或
   恢复同一时间戳的备份。运行中的标定节点不会读取新值或自动移动。

   `TARGET=preset` 直接保存关节反馈，不需要正运动学。它只写入
   `ros2_ws/src/ctl_msgs_hw/config/vr_preset_pose.yaml`，并生成同名带时间戳的 `.backup`；
   单侧保存会保留另一侧的原有预设。

## 保存后验证

1. 先在终端 1 按 `Ctrl+C` 停止被动标定模式，再确认双臂均已被安全支撑或处在可靠位置。
2. 运行 `make build`，使安装空间中的标定配置与源码配置同步，并重新为
   新的 `servo_node` 配置 EtherCAT 能力。
   如果本次仅保存 `TARGET=preset`，可改用
   `bash scripts/build.sh --packages-select ctl_msgs_hw`，只同步配置包。
3. 保持急停可用并清空双臂周围，运行普通 `make servo`。普通模式会恢复原有的
   自动回零行为；`TARGET=joint/both` 时确认向刚保存的姿态运动，
   `TARGET=cartesian/preset` 时仍使用原来的关节回零目标。出现方向、幅度或碰撞风险
   异常时立即急停，不要继续遥操。
4. 通过后保持终端 1 的普通 `make servo` 运行，再在其他独立终端依次启动：

   | 顺序 | 命令 | 作用 |
   | --- | --- | --- |
   | 终端 1（已运行） | `make servo` | 底层伺服和初始回零，不要重复启动 |
   | 终端 2 | `make arms` | 双臂运动学与轨迹控制 |
   | 终端 3 | `make robot` | 上层控制和遥操作初始位姿 |
   | 终端 4 | `make vr` | VR 手柄输入 |

   如果终端 2 仍在运行 `make watch-arm`，先停止观察命令再启动 `make arms`。
   每一步等待前一个节点初始化完成。`TARGET=both` 时，
   `robot` 的末端目标应与初始回零位置一致；实际精度取决于模型和关节反馈。
   A 键使用 `zeropos.yaml`，不再自动记录或读取 `refTargetAngle.yaml` 作为回零目标。
   B 键使用 `vr_preset_pose.yaml`，该文件在 `make robot` 启动时加载。
5. 头显连接页面并进入 **Virtual Reality** 后，回到终端 4 按 **Enter**。
   出现“开始发布位姿增量（扣动扳机生效）”后，按住对应手柄的扳机、小幅移动手柄。
   `make vr-debug` 只显示诊断数据，不能替代 `make vr`。
   连接地址和检查方法见 [VR 遥操作检查](VR_TELEOP_CHECK.md)。
   验证预设时，松开扳机，按一下右 B：双臂应平滑到达保存的姿态，完成后恢复遥操。

## 恢复上一次零位

必须先停止所有伺服程序。选择该次标定打印出的具体备份文件，用其覆盖源码配置，
然后重新构建：

```bash
cp ros2_ws/src/ctl_msgs_hw/config/zeropos.yaml.<时间戳>.backup \
   ros2_ws/src/ctl_msgs_hw/config/zeropos.yaml
make build
```

若该次选择 `TARGET=both`，也应恢复相同时间戳的
`desc_zeropos.yaml.<时间戳>.backup` 到 `desc_zeropos.yaml`，再执行 `make build`。

恢复右 B 预设时，使用对应的 `vr_preset_pose.yaml.<时间戳>.backup` 覆盖
`vr_preset_pose.yaml`，同步配置包并重启 `make robot`。

不要用通配符猜测备份文件；先核对时间戳和内容。恢复后仍按“保存后验证”的安全条件
进行一次普通 `make servo` 检查。
