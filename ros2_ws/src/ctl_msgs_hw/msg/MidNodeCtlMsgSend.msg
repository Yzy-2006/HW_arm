#上层节点与中层节点通讯代码
#上层节点向中层节点下发消息的代码
#该信息字段较长，有可能会出现通讯延时过久的bug

#----------控制消息发送代码----------

#时间戳头
std_msgs/Header header

#-----数据传输与插补模式选择-----

int32 ctl_trans_mod         #数据传输模式
int32 runin_mod             #插补模式
int32 tor_feedforward_ctl   #力前馈控制变量

#-----求解模式信息-----

int32 if_use_ret            #是否使用返回值
bool if_use_temp            #是否使用循环输出
int32[] rigidity_ctl_mod_l     #左臂各个关节底层控制模式选择
int32[] rigidity_ctl_mod_r     #右臂各个关节底层控制模式选择

#搭配自定义刚度模式的控制代码
float64[] kp_trans_l       #左臂关节Kp
float64[] kd_trans_l       #左臂关节Kd
float64[] kp_trans_r       #右臂关节Kp
float64[] kd_trans_r       #右臂关节Kd

#-----插补信息包装-----

float64 upper_monitor_time   #上位机迭代时间

# Original bridge sample identity; repeated forwarding must not restart interpolation.
std_msgs/Header trajectory_header
bool bridge_joint_stream
bool policy_speed_checks_disabled
bool derivatives_valid
float64[] l_arm_des_acc
float64[] r_arm_des_acc

float64[] traj_time_l_arm  #左臂轨迹时间
float64[] traj_time_r_arm  #右臂轨迹时间

float64[] acc_rate_base_l_arm  #左臂加速阶段占比
float64[] acc_rate_base_r_arm  #右臂加速阶段占比

bool[] l_arm_if_use_stab  #左臂插补过程中是否使用静止阶段
bool[] r_arm_if_use_stab  #右臂插补过程中是否使用静止阶段

int32[] l_arm_start_stab_times     #左臂起始阶段静止轮数
int32[] r_arm_start_stab_times     #右臂起始阶段静止轮数

int32[] l_arm_des_stab_times       #左臂静止阶段静止轮数
int32[] r_arm_des_stab_times       #右臂静止阶段静止轮数

#-----关节控制恒定位置输出开关-----

int32[] l_arm_if_jnt_stab      #左臂关节是否开启恒定位置模式
int32[] r_arm_if_jnt_stab      #右臂关节是否开启恒定位置模式

bool if_redefine_stab_jnt_ang   #判断量 -- 是否重新定义静止关节的角度

#-----关节限制信息-----

bool l_arm_if_use_jntlim       #左臂是否限制关节信息
int32 l_arm_jntlim_num          #左臂受限关节数量
string[] l_arm_limjnt_label    #左臂受限关节标签
float64[] l_arm_limjnt_ang        #左臂受限关节角度

bool r_arm_if_use_jntlim       #右臂是否限制关节信息
int32 r_arm_jntlim_num          #右臂受限关节数量
string[] r_arm_limjnt_label    #右臂受限关节标签
float64[] r_arm_limjnt_ang        #右臂受限关节角度

bool if_update_lim             #是否更新限制信息


#-----目标控制数据-----

#左右臂笛卡尔空间
float64[] l_arm_des_pos_descartes
float64[] r_arm_des_pos_descartes

float64[] l_arm_des_tor_descartes
float64[] r_arm_des_tor_descartes

#左右臂关节空间
float64[] l_arm_des_pos_jntspace
float64[] r_arm_des_pos_jntspace

float64[] l_arm_des_tor_jntspace
float64[] r_arm_des_tor_jntspace

#适配与MDP接口的预留量
float64[] l_arm_des_vel_jntspace
float64[] r_arm_des_vel_jntspace
