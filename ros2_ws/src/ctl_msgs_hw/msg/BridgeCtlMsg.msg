# 通讯桥接控制消息 — 从其他节点接受机械臂控制参量
# 每接受一次该消息，ctlTransmit就执行一次发送代码
# 控制代码类型及包含信息参照demoTraj

# 时间戳头
std_msgs/Header header

#===== 机械臂轨迹控制信息 (参照 armTrajMsg_Trans / demoTraj DotOutpWrap) =====

# 运动控制模式:
#   0-归零 1-关节模式 2-点位控制 3-关节轨迹控制
#   4-点位轨迹控制 5-零力拖动 6-MDP模式
int32 ctl_trans_mod

# 力前馈控制: 0-无力前馈 1-笛卡尔空间 2-关节空间
int32 torque_feedforward_ctl

# 是否到达目标位置
bool if_reach_end

# Continuous joint stream (ctl_trans_mod=3): capture and hold the current joint
# feedback once. Release only when a new command with hold_position=false arrives.
bool hold_position

#----- 关节限位控制 (左臂) -----
bool l_arm_if_use_jntlim
int32 l_arm_limjnt_num
int32[] l_arm_limjnt_id
float64[] l_arm_limjnt_ang

#----- 关节限位控制 (右臂) -----
bool r_arm_if_use_jntlim
int32 r_arm_limjnt_num
int32[] r_arm_limjnt_id
float64[] r_arm_limjnt_ang

#----- 目标位姿信息 (参照 ctlDesMsg_trans) -----
# 笛卡尔模式(ctlTransMod=2,4): 前3位末端位置，后4位姿态四元数(w,x,y,z) — 共7维
# 关节模式(ctlTransMod=1,3): 依次为各关节角度 — LARMDOF/RARMDOF维
# MDP模式(ctlTransMod=6): 关节角度 — LARMDOF/RARMDOF维
float64[] l_arm_des_msg
float64[] r_arm_des_msg

# 目标前馈力矩信息
float64[] l_arm_des_tor_msg
float64[] r_arm_des_tor_msg

# MDP接口目标关节速度
float64[] l_arm_mdp_des_jnt_vel
float64[] r_arm_mdp_des_jnt_vel

# 是否使用返回值初始化
int32 if_init_use_ret

#----- 刚度等级 -----
int32[] l_arm_rigidity_mod
int32[] r_arm_rigidity_mod

#----- PD参数 -----
float64[] l_arm_jnt_kp
float64[] l_arm_jnt_kd
float64[] r_arm_jnt_kp
float64[] r_arm_jnt_kd

#----- 关节静止开关 (参照demoTraj中 l_ifJntStab / r_ifJntStab) -----
int32[] l_arm_if_jnt_stab
int32[] r_arm_if_jnt_stab
