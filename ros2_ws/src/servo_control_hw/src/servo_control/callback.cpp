#include "servo_control/callback.hpp"
#include "servo_control/pdos.hpp"

// ==========================================
// [新增] 定义解算需要的全局变量 (分配内存)
// ==========================================
ECATBAGHANDDATA hand_info_v;
ECATBAGHANDTARGET hand_info_send_v;

// 为了防止 callback.cpp 中遗留的 upload_data 等函数对 shared_var 操作导致段错误，
// 我们定义一个虚拟的共享内存给它绑定（外层 transmit.cpp 会通过 finger_t_struct 阵列去拿数据）
// SharedMemory dummy_shm;
// SharedMemory* shared_var = &dummy_shm; 
SharedMemory* shared_var = nullptr;

// ==========================================


Finger_R_Struct last_finger_r_struct[5];

Finger_R_Struct finger_r_struct[5];
Finger_T_Struct finger_t_struct[5];


Before_Conversion_Finger_R_Struct convert_finger_r_struct[5];
Before_Conversion_Finger_T_Struct convert_finger_t_struct[5];




float r_convert[10]={1, 0.01, 0.01, 0.01, 0.001, 0.001, 0.001, 0.001, 1, 1 };
float t_convert[20]={1, 1, 0.01, 0.01, 0.01, 0.01,0.01, 0.01 ,1, 1, 1, 1, 1, 1, 1, 1, 1, 1 ,1 , 1};


int select_finger=0;


void finger_data_convert(void)
{
	if(select_finger==0)
	{
	convert_finger_r_struct[0].run_mode 					= 		  (1/r_convert[0])	* 	finger_r_struct[0].run_mode 		;
    convert_finger_r_struct[0].p_tip_pitch 					=         (1/r_convert[1])	* 	finger_r_struct[0].p_tip_pitch 		;
    convert_finger_r_struct[0].p_joint_pitch 				=         (1/r_convert[2])	* 	finger_r_struct[0].p_joint_pitch 	;
    convert_finger_r_struct[0].p_joint_yaw 					=         (1/r_convert[3])	* 	finger_r_struct[0].p_joint_yaw 		;
    convert_finger_r_struct[0].plan_time 					=         (1/r_convert[4])	* 	finger_r_struct[0].plan_time 		;
    convert_finger_r_struct[0].k_tip_pitch 					=         (1/r_convert[5])	* 	finger_r_struct[0].k_tip_pitch 		;
    convert_finger_r_struct[0].k_joint_pitch 				=         (1/r_convert[6])	* 	finger_r_struct[0].k_joint_pitch 	;
    convert_finger_r_struct[0].k_joint_yaw 					=         (1/r_convert[7])	* 	finger_r_struct[0].k_joint_yaw 		;
    convert_finger_r_struct[0].finger_reserve1 				=         (1/r_convert[8])	* 	finger_r_struct[0].finger_reserve1  ;
    convert_finger_r_struct[0].finger_reserve2 				=         (1/r_convert[9])	* 	finger_r_struct[0].finger_reserve2  ;

	convert_finger_t_struct[0].state						=			t_convert[0]	*	hand_info_v.ThumbData.state;		    	
	convert_finger_t_struct[0].run_mode						=			t_convert[1]	*	hand_info_v.ThumbData.run_mode;		    	  
	convert_finger_t_struct[0].p_finger1_up_pitch			=			t_convert[2]	*	hand_info_v.ThumbData.p_finger1_up_pitch;	
	convert_finger_t_struct[0].p_finger1_joint_pitch		=			t_convert[3]	*	hand_info_v.ThumbData.p_finger1_joint_pitch;
	convert_finger_t_struct[0].p_finger1_joint_yaw			=			t_convert[4]	*	hand_info_v.ThumbData.p_finger1_joint_yaw;
	convert_finger_t_struct[0].v_finger1_up_pitch			=			t_convert[5]	*	hand_info_v.ThumbData.v_finger1_up_pitch;	
	convert_finger_t_struct[0].v_finger1_joint_pitch		=			t_convert[6]	*	hand_info_v.ThumbData.v_finger1_joint_pitch;
	convert_finger_t_struct[0].v_finger1_joint_yaw			=			t_convert[7]	*	hand_info_v.ThumbData.v_finger1_joint_yaw;
	convert_finger_t_struct[0].f_finger1_up_pitch			=			t_convert[8]	*	hand_info_v.ThumbData.f_finger1_up_pitch;	
	convert_finger_t_struct[0].f_finger1_joint_pitch		=			t_convert[9]	*	hand_info_v.ThumbData.f_finger1_joint_pitch;
	convert_finger_t_struct[0].f_finger1_joint_yaw			=			t_convert[10]	*	hand_info_v.ThumbData.f_finger1_joint_yaw;
	convert_finger_t_struct[0].pwm_finger1_up				=			t_convert[11]	*	hand_info_v.ThumbData.pwm_finger1_up;		
	convert_finger_t_struct[0].pwm_finger1_joint1			=			t_convert[12]	*	hand_info_v.ThumbData.pwm_finger1_joint1;	
	convert_finger_t_struct[0].pwm_finger1_joint2			=			t_convert[13]	*	hand_info_v.ThumbData.pwm_finger1_joint2;	
	}
	else
	{
        int finger_index = select_finger - 1;
	convert_finger_r_struct[select_finger].run_mode 					= 		  (1/r_convert[0])	* 	finger_r_struct[select_finger].run_mode 		    ;
    convert_finger_r_struct[select_finger].p_joint_pitch 				=         (1/r_convert[2])	* 	finger_r_struct[select_finger].p_joint_pitch 	    ;
    convert_finger_r_struct[select_finger].plan_time 					=         (1/r_convert[4])	* 	finger_r_struct[select_finger].plan_time 		    ;
    convert_finger_r_struct[select_finger].k_joint_pitch 				=         (1/r_convert[6])	* 	finger_r_struct[select_finger].k_joint_pitch 	    ;
    convert_finger_r_struct[select_finger].finger_reserve1 			    =         (1/r_convert[8])	* 	finger_r_struct[select_finger].finger_reserve1    ;
    convert_finger_r_struct[select_finger].finger_reserve2 			    =         (1/r_convert[9])	* 	finger_r_struct[select_finger].finger_reserve2    ;

	convert_finger_t_struct[select_finger].state						=			t_convert[0]	*	hand_info_v.FigData[finger_index].state;		    	
	convert_finger_t_struct[select_finger].run_mode					    =			t_convert[1]	*	hand_info_v.FigData[finger_index].run_mode;		    	  
	convert_finger_t_struct[select_finger].p_finger1_up_pitch			=			0;	
	convert_finger_t_struct[select_finger].p_finger1_joint_pitch		=			t_convert[3]	*	hand_info_v.FigData[finger_index].p_finger1_joint_pitch;
	convert_finger_t_struct[select_finger].p_finger1_joint_yaw		    =			0;
	convert_finger_t_struct[select_finger].v_finger1_up_pitch			=			0;	
	convert_finger_t_struct[select_finger].v_finger1_joint_pitch		=			t_convert[6]	*	hand_info_v.FigData[finger_index].v_finger1_joint_pitch;
	convert_finger_t_struct[select_finger].v_finger1_joint_yaw		    =			0;
	convert_finger_t_struct[select_finger].f_finger1_up_pitch			=			0;	
	convert_finger_t_struct[select_finger].f_finger1_joint_pitch		=			t_convert[9]	*	hand_info_v.FigData[finger_index].f_finger1_joint_pitch;
	convert_finger_t_struct[select_finger].f_finger1_joint_yaw		    =			0;
	convert_finger_t_struct[select_finger].pwm_finger1_up				=			0;		
	convert_finger_t_struct[select_finger].pwm_finger1_joint1			=			t_convert[12]	*	hand_info_v.FigData[finger_index].pwm_finger1_joint1;	
	convert_finger_t_struct[select_finger].pwm_finger1_joint2			=			0;	
	}
}


void targetset()
{    // 拇指目标设置
    if (select_finger == 0) {
        hand_info_send_v.thumb_r_struct.run_mode      = convert_finger_r_struct[0].run_mode;
        hand_info_send_v.thumb_r_struct.p_tip_pitch   = convert_finger_r_struct[0].p_tip_pitch;
        hand_info_send_v.thumb_r_struct.p_joint_pitch = convert_finger_r_struct[0].p_joint_pitch;
        hand_info_send_v.thumb_r_struct.p_joint_yaw   = convert_finger_r_struct[0].p_joint_yaw;
        hand_info_send_v.thumb_r_struct.plan_time     = convert_finger_r_struct[0].plan_time;
        hand_info_send_v.thumb_r_struct.k_tip_pitch   = convert_finger_r_struct[0].k_tip_pitch;
        hand_info_send_v.thumb_r_struct.k_joint_pitch = convert_finger_r_struct[0].k_joint_pitch;
        hand_info_send_v.thumb_r_struct.k_joint_yaw   = convert_finger_r_struct[0].k_joint_yaw;
        hand_info_send_v.thumb_r_struct.stop_force    = convert_finger_r_struct[0].finger_reserve1;
    }
    // 其他四个手指目标设置
    else {
        int finger_index = select_finger - 1; // 转换为0-3的索引
        
        hand_info_send_v.fig_r_struct[finger_index].run_mode      = convert_finger_r_struct[select_finger].run_mode;
        hand_info_send_v.fig_r_struct[finger_index].p_joint_pitch = convert_finger_r_struct[select_finger].p_joint_pitch;
        hand_info_send_v.fig_r_struct[finger_index].plan_time     = convert_finger_r_struct[select_finger].plan_time;
        hand_info_send_v.fig_r_struct[finger_index].k_joint_pitch = convert_finger_r_struct[select_finger].k_joint_pitch;
        hand_info_send_v.fig_r_struct[finger_index].stop_force    = convert_finger_r_struct[select_finger].finger_reserve1;
    }


}


void upload_data()
{
    // 拇指数据上传
    if (select_finger == 0) {
        shared_var->finger_t_struct[0].state                = convert_finger_t_struct[0].state;
        shared_var->finger_t_struct[0].run_mode             = convert_finger_t_struct[0].run_mode;
        shared_var->finger_t_struct[0].p_finger_up_pitch    = convert_finger_t_struct[0].p_finger1_up_pitch;
        shared_var->finger_t_struct[0].p_finger_joint_pitch = convert_finger_t_struct[0].p_finger1_joint_pitch;
        shared_var->finger_t_struct[0].p_finger_joint_yaw   = convert_finger_t_struct[0].p_finger1_joint_yaw;
        shared_var->finger_t_struct[0].v_finger_up_pitch    = convert_finger_t_struct[0].v_finger1_up_pitch;
        shared_var->finger_t_struct[0].v_finger_joint_pitch = convert_finger_t_struct[0].v_finger1_joint_pitch;
        shared_var->finger_t_struct[0].v_finger_joint_yaw   = convert_finger_t_struct[0].v_finger1_joint_yaw;
        shared_var->finger_t_struct[0].f_finger_up_pitch    = convert_finger_t_struct[0].f_finger1_up_pitch;
        shared_var->finger_t_struct[0].f_finger_joint_pitch = convert_finger_t_struct[0].f_finger1_joint_pitch;
        shared_var->finger_t_struct[0].f_finger_joint_yaw   = convert_finger_t_struct[0].f_finger1_joint_yaw;
        shared_var->finger_t_struct[0].pwm_finger_up        = convert_finger_t_struct[0].pwm_finger1_up;
        shared_var->finger_t_struct[0].pwm_finger_joint1    = convert_finger_t_struct[0].pwm_finger1_joint1;
        shared_var->finger_t_struct[0].pwm_finger_joint2    = convert_finger_t_struct[0].pwm_finger1_joint2;
    }
    // 其他四个手指数据上传
    else {
        shared_var->finger_t_struct[select_finger].state                = convert_finger_t_struct[select_finger].state;
        shared_var->finger_t_struct[select_finger].run_mode             = convert_finger_t_struct[select_finger].run_mode;
        shared_var->finger_t_struct[select_finger].p_finger_up_pitch    = 0;
		shared_var->finger_t_struct[select_finger].p_finger_joint_pitch = convert_finger_t_struct[select_finger].p_finger1_joint_pitch;
		shared_var->finger_t_struct[select_finger].p_finger_joint_yaw   = 0;
		shared_var->finger_t_struct[select_finger].v_finger_up_pitch    = 0;
        shared_var->finger_t_struct[select_finger].v_finger_joint_pitch = convert_finger_t_struct[select_finger].v_finger1_joint_pitch;
        shared_var->finger_t_struct[select_finger].v_finger_joint_yaw   = 0;
		shared_var->finger_t_struct[select_finger].f_finger_up_pitch    = 0;
		shared_var->finger_t_struct[select_finger].f_finger_joint_pitch = convert_finger_t_struct[select_finger].f_finger1_joint_pitch;
        shared_var->finger_t_struct[select_finger].f_finger_joint_yaw   = 0;
		shared_var->finger_t_struct[select_finger].pwm_finger_up        = 0;
		shared_var->finger_t_struct[select_finger].pwm_finger_joint1    = convert_finger_t_struct[select_finger].pwm_finger1_joint1;
		shared_var->finger_t_struct[select_finger].pwm_finger_joint2    = 0;
    }
    
    // 设置系统状态标志
    shared_var->left_right = hand_info_v.left_right_hand_mode;
    shared_var->data_ready = 1;
}
