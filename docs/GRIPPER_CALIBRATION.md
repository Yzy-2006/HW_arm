# 双夹爪控制与端点标定

## 按键与编码

- 左侧 squeeze 上升沿切换左夹爪，保持按住不会重复切换。
- `left_bButton` 上升沿切换右夹爪。
- 右 A 回零、右 B 录制、左 `left_aButton` 丢弃、右 squeeze 力前馈沿用原分配。
- 0=打开，1=闭合。VR 发布 `[左反馈,右反馈,左目标,右目标]` 到 `/towel/gripper_state`。
- 观测是实际角度相对标定行程的二值分类，目标是按键切换的状态，二者分别采集。

## 端点文件

`teleop/VR_controller/config/grippers.yaml` 为每侧独立保存：
`open_deg`、`closed_deg`、`calibrated`、`kp/kd`、`motion_time_s`、`torque_limit_nm`。
允许左/右电机方向相反，直接用实际开、闭角度计算，不需要猜测符号。
左侧初始无端点；右侧原 -30/45 度仅作参考，两个侧均需本次实际采样才能启用。
PD 初始参数沿用原右侧数值，尚未完成左侧实机增益调节。

## 操作步骤

在机器人上，停止 VR 和采集进程；按硬件原有流程让待测夹爪处于可安全手动调整/释放力矩的状态，
同时保持 `/gripper/fdb` 正常发布。**只停止 VR 不保证伺服清除上一条力矩命令。**
下面的工具仅订阅反馈，不会发送力矩，也不会自动寻找机械限位。

每条命令前，先把对应夹爪放到所选的正常开或闭位置并保持静止（不要用硬限位顶死作为运动目标）：

```bash
cd /path/to/towel_task
source scripts/env.sh
# 左夹爪放到打开位置，再执行
python3 scripts/calibrate_grippers.py --side left --endpoint open
# 左夹爪放到闭合位置，再执行
python3 scripts/calibrate_grippers.py --side left --endpoint closed
# 右夹爪放到打开位置，再执行
python3 scripts/calibrate_grippers.py --side right --endpoint open
# 右夹爪放到闭合位置，再执行
python3 scripts/calibrate_grippers.py --side right --endpoint closed
```

每次取至少 1 秒且至少 20 个反馈样本的中位数；角度极差不得大于 1 度，角速度不超过 0.05 rad/s。
端点间距至少 5 度；不满足则拒绝写入。成功写入前生成 `.backup`，配置通过原子替换保存。
两端都由工具采过后，才将该侧标记为 `calibrated: true`。若更换夹爪或编码器零位，应重新采两端。

随后重新启动 VR。每侧从首次反馈角度保持，第一次按键才向对应端点运动；
到达端点仍使用 PD 保持，反馈超时会输出零力矩并将状态标为未知，恢复后重新以当前位置保持。

```bash
ros2 topic echo /towel/gripper_state
```

检查左右单独切换、方向、行程和夹持毛巾的效果，再启动采集器。
未知状态 -1 不会进入数据集。运动中滞回阈值为行程的 40%/60%，避免临界点抖动。
这种二值反馈不表示接触检测或夹持成功；毛巾较厚时，反馈状态与闭合目标可能不同。

标定过程记录角度用于控制；正式逐帧采集只保存开合状态，不保存夹爪角度、速度、电流、温度或力矩。
