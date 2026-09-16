#RetInformation

#时间戳头
std_msgs/Header header

float64[] l_position    # 反馈位置 (rad)
float64[] l_speed       # 反馈速度 (rad/s)
float64[] l_iq      # 反馈电流

float64[] r_position    # 反馈位置 (rad)
float64[] r_speed       # 反馈速度 (rad/s)
float64[] r_iq      # 反馈电流

int32 if_first_time   #标志量：是否是第一轮 1--是 0--否

float64[] float_reserve
int32[] int_reserve