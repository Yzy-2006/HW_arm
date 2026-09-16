#ifndef __CALLBACK__HPP__
#define __CALLBACK__HPP__

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "pdos.hpp"




#define   Cmd_Disable 																	0x00
#define   Cmd_Closed_loop_Mode 															0x11
#define   Cmd_Trajectory_Planning_Mode 													0x22
#define   Cmd_Impedance_Control_Mode  													0x33
#define   Cmd_Force_Control_Mode  														0x44
#define   Cmd_New_Reset_Mode  															0x55
#define   Cmd_Calibration 																0x66
#define   Cmd_Open_Loop_Mode 															0x77
#define   Cmd_Clear_Errors  															0x88
#define   Cmd_New_Force_Control_Mode  													0xAA
#define   Cmd_Find_Zero  													            0xBB
#define   Cmd_Set_Zero  													            0xCC
#define   Cmd_Reset_Force_Zero  													    0xDD
#define   Cmd_Drag_Zero  													    		0xEE
#define   Cmd_New_Calibration 															0xFF



#define   Run_Mode_Disable 																0x00
#define   Run_Mode_Closed_loop_Mode 													0x11
#define   Run_Mode_Trajectory_Planning_Mode 											0x22
#define   Run_Mode_Impedance_Control_Mode  												0x33
#define   Run_Mode_Force_Control_Mode  													0x44
#define   Run_Mode_New_Reset_Mode  														0x55
#define   Run_Mode_Calibration 															0x66
#define   Run_Mode_Open_Loop_Mode 														0x77
#define   Run_Mode_New_Force_Control_Mode  												0xAA
#define   Run_Mode_Find_Zero  													        0xBB
#define   Run_Mode_Drag_Zero  													    	0xEE
#define   Run_Mode_New_Calibration 														0xFF


#ifndef UINT8
#define UINT8                                     unsigned char
#endif

#ifndef UINT16
#define UINT16                                    unsigned short
#endif

#ifndef UINT32
#define UINT32                                    unsigned int
#endif

#ifndef USHORT
#define USHORT                                    unsigned short
#endif

#ifndef INT8
#define INT8                                      char
#endif

#ifndef INT16
#define INT16                                     short
#endif

#ifndef INT32
#define INT32                                    int32_t
#endif



typedef struct _G2HITFINGERCONTROL {
	int FingerCtrl;
	int FingerMotPWM;
	int BaseMot1PWM;
	int BaseMot2PWM;
	int Reserved[6];
} G2HITFINGERCONTROL, *LPG2HITFINGERCONTROL;



typedef struct _G2HITHANDCONTROL {
	G2HITFINGERCONTROL Thumb;
	G2HITFINGERCONTROL FirstFinger;
	G2HITFINGERCONTROL MiddleFinger;
	G2HITFINGERCONTROL RingFinger;
	G2HITFINGERCONTROL LittleFinger;
} G2HITHANDCONTROL, *LPG2HITHANDCONTROL;



typedef struct _FINGERPARAPOS
{
    float Kp;
    float Ki;
    float Kd;
    float Fo;
}FINGERPARAPOS;


typedef struct _FINGERPARAIMP
{
    float Kst;
    float Kdp;
    float Kdi;
}FINGERPARAIMP;



typedef struct {
	uint8_t    	    run_flag;       	
	uint8_t     	direction;      
	float    		position;       
	float       	pwm;          
	uint8_t   		hall_state_now; 
	uint8_t    	    hall_state_last;    
	uint8_t    	    reset_flag;       	
	float    		position_angle; 
	uint8_t    	    start_flag;         
	float    		last_position;
	float           speed;
} MOTOR_STATE;


typedef struct {	
	UINT16 run_mode;
	INT16  p_tip_pitch;
	INT16  p_joint_pitch;
	INT16  p_joint_yaw;
	UINT16 plan_time;
	UINT16 k_tip_pitch;
	UINT16 k_joint_pitch;
	UINT16 k_joint_yaw;
	UINT16 d_tip_pitch;
	UINT16 d_joint_pitch;
	UINT16 d_joint_yaw;
	UINT16 j_tip_pitch;
	UINT16 j_joint_pitch;
	UINT16 j_joint_yaw;
	INT16  pwm_tip_pitch;
	INT16  pwm_joint_pitch;
	INT16  pwm_joint_yaw;
	UINT16 finger_enable[2];
	UINT16 finger_reserve1;
	UINT16 finger_reserve2;
	UINT16 finger_reserve3;
	UINT16 finger_reserve4;
	UINT16 finger_reserve5;
	} Before_Conversion_Finger_R_Struct;
	
	typedef struct {	
	UINT16 run_mode;
	float  p_tip_pitch;
	float p_joint_pitch;
	float p_joint_yaw;
	float plan_time;
	float k_tip_pitch;
	float k_joint_pitch;
	float k_joint_yaw;
	float d_tip_pitch;
	float d_joint_pitch;
	float d_joint_yaw;
	float j_tip_pitch;
	float j_joint_pitch;
	float j_joint_yaw;
	INT16  pwm_tip_pitch;
	INT16  pwm_joint_pitch;
	INT16  pwm_joint_yaw;
	UINT16 finger_enable[2];
	UINT16 finger_reserve1;
	UINT16 finger_reserve2;
	UINT16 finger_reserve3;
	UINT16 finger_reserve4;
	UINT16 finger_reserve5;
	} Finger_R_Struct;
	typedef struct {	
	
		UINT16 state;
		UINT16 run_mode;
		UINT16 initial_force_finger1_up_pitch;
		UINT16 initial_force_finger1_joint_pitch;
		UINT16 initial_force_finger1_joint_yaw;
			
		UINT16 initial_hall_finger1_up;
		UINT16 initial_hall1_finger1_joint;
		UINT16 initial_hall2_finger1_joint;
		UINT16 initial_hall3_finger1_joint;
			
		UINT16 tactile[20];
		
		float p_finger1_up_pitch;
		float p_finger1_joint_pitch;
		float p_finger1_joint_yaw;
			
		float v_finger1_up_pitch;
		float v_finger1_joint_pitch;
		float v_finger1_joint_yaw;
		
		float f_finger1_up_pitch;
		float f_finger1_joint_pitch;
		float f_finger1_joint_yaw;
		
		INT16 pwm_finger1_up;
		INT16 pwm_finger1_joint1;
		INT16 pwm_finger1_joint2;
		
		UINT16 reserve[7];
			
	
		
		} Before_Conversion_Finger_T_Struct;



		typedef struct {	
	
			unsigned short state;
			unsigned short run_mode;

			float motor1_position;
			float motor2_position;
			float motor3_position;

			float initial_force_finger_up_pitch;
			float initial_force_finger_joint_pitch;
			float initial_force_finger_joint_yaw;
				
			float initial_hall_finger_up;

			float initial_hall1_finger_joint;
			float initial_hall2_finger_joint;
			float initial_hall3_finger_joint;
			
			float p_finger_up_pitch;
			float p_finger_joint_pitch;
			float p_finger_joint_yaw;
				
			float v_finger_up_pitch;
			float v_finger_joint_pitch;
			float v_finger_joint_yaw;
			
			float f_finger_up_pitch;
			float f_finger_joint_pitch;
			float f_finger_joint_yaw;
			
			float pwm_finger_up;
			float pwm_finger_joint1;
			float pwm_finger_joint2;

			double finger_position_tosend[3];
			double palm_position_tosend[3];
			
			
			} Shm_Finger_T_Struct;
			
		
		typedef struct 
		{
			int external_control;
			int data_ready;               		
			int flag_dkr;
			int left_right;
			float pos[3][5];
			Finger_R_Struct finger_r_struct[5];  
			Shm_Finger_T_Struct finger_t_struct[5];
        } SharedMemory;	
        

void finger_data_convert();
void targetset();
void upload_data();



#endif
