#ifndef __PDOS__HPP__
#define __PDOS__HPP__
#include <cstdint>


typedef struct 
{	
		
	volatile	short state;
	volatile	short run_mode;
	
	volatile	short p_finger1_up_pitch;
	volatile	short p_finger1_joint_pitch;
	volatile	short p_finger1_joint_yaw;

	volatile	short v_finger1_up_pitch;
	volatile	short v_finger1_joint_pitch;
	volatile	short v_finger1_joint_yaw;
	
	volatile	short f_finger1_up_pitch;
	volatile	short f_finger1_joint_pitch;
	volatile	short f_finger1_joint_yaw;
	
	volatile	short pwm_finger1_up;
	volatile	short pwm_finger1_joint1;
	volatile	short pwm_finger1_joint2;
	
	volatile	short reserve[6];

} Thumb_T_Struct;

	typedef struct 
	{		
		volatile	short state;
		volatile	short run_mode;
		volatile	short p_finger1_joint_pitch;  
		volatile	short v_finger1_joint_pitch;
		volatile	short f_finger1_joint_pitch;
		volatile	short pwm_finger1_joint1;
		volatile	short reserve[5];
	} Finger_T_Struct;

typedef struct _ECATBAGHANDDATA
{
	Thumb_T_Struct ThumbData;      //20
	Finger_T_Struct FigData[4];    //44
	volatile	short left_right_hand_mode;
	volatile	short reserve[7];  //8

} ECATBAGHANDDATA, *LPECATBAGHANDDATA;


typedef struct 
{	
	volatile	short run_mode;
	volatile	short  p_tip_pitch;
	volatile	short  p_joint_pitch;
	volatile	short  p_joint_yaw;
	volatile	short plan_time;
	volatile	short k_tip_pitch;
	volatile	short k_joint_pitch;
	volatile	short k_joint_yaw;
	volatile	short stop_force;
	volatile	short finger_reserve1;
} Thumb_R_Struct;


typedef struct 
{	
	volatile	short run_mode;	
	volatile	short p_joint_pitch;
	volatile    short plan_time;
	volatile	short k_joint_pitch;
	volatile    short stop_force;
	volatile	short finger_reserve1;

} finger_r_struct_r;


typedef struct _ECATBAGHANDTARGET
{
	Thumb_R_Struct  thumb_r_struct;   //10
	finger_r_struct_r fig_r_struct[4];  //24
	volatile	short left_right;	  //1
	volatile	short Reserve;		  //1
} ECATBAGHANDTARGET, *LPECATBAGHANDTARGET;


#endif
