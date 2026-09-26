#夹爪控制接口消息类型

float64 ctl_time

float64 l_arm_gripper_tor
float64 r_arm_gripper_tor

# Position offsets from gripperControl.<side>.zero_pos_deg, in degrees.
# NaN means hold the previous target for that side.
float64 l_arm_gripper_pos
float64 r_arm_gripper_pos
