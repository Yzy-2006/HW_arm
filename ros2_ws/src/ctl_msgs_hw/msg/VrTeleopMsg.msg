# VR 遥操作消息类型 — VR 端下发到机器人上层节点 (robot_control_hw)

# 时间戳头
std_msgs/Header header

#----- 左臂关节角增量（rad，7 关节，顺序与 zeropos.yaml 一致）-----
float64[] l_arm_jnt_delta

#----- 右臂关节角增量（rad，7 关节）-----
float64[] r_arm_jnt_delta

#----- 状态切换变量（控制模式枚举）-----
# 0 = 遥操作跟随（累加关节角增量并跟随）
# 1 = 回归零位（平滑回到零位）
# 其他值预留扩展
int32 ctl_mode
