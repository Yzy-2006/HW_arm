# CtlInformation

#时间戳头
std_msgs/Header header

#----------控制代码----------

#控制周期
float64 temp_time

#插补模式代码
int32 runin_mod

#判断是否刷新底层插补器
int32 if_runin_renew

#判断是否使用返回值作为第一轮插补输入值
int32 if_useret_to_start_runin

#判断是否使用调试模式
int32 if_use_debugmod

#下发的初始值（位置量，其他的均为0，Kp、Kd不参与插补）
float64[] l_position_at_runin_st
float64[] r_position_at_runin_st

#位置保持模式控制代码：0--终止位置保持模式；1--开启位置保持模式
int32 place_hold

# Already sampled position/velocity from one validated joint trajectory.
bool trajectory_sample

#底层控制模式选择代码：
int32[] base_rigidity_ctl_mode_l       #左臂各个关节底层控制模式选择
int32[] base_rigidity_ctl_mode_r       #右臂各个关节底层控制模式选择

#----------数据传输代码----------

float64[] l_position    # 目标位置 (rad)
float64[] l_kp          # 比例增益
float64[] l_kd          # 微分增益
float64[] l_speed       # 目标速度 (rad/s)
float64[] l_torque_grav_comp       # 目标重力补偿力矩 (Nm)
float64[] l_torque_interia_comp    # 目标惯性补偿力矩 (Nm)
float64[] l_torque_feedforward     # 目标前馈力矩 (Nm)

float64[] r_position    # 目标位置 (rad)
float64[] r_kp          # 比例增益
float64[] r_kd          # 微分增益
float64[] r_speed       # 目标速度 (rad/s)
float64[] r_torque_grav_comp        # 目标重力补偿力矩 (Nm)
float64[] r_torque_interia_comp    # 目标惯性补偿力矩 (Nm)
float64[] r_torque_feedforward     # 目标前馈力矩 (Nm)

#----------reserve----------

#保留量
float64[] float_reserve
int32[] int_reserve
