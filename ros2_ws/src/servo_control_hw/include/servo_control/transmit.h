/*
 * @Description:
 * @Author: kx zhang
 * @Date: 2022-09-20 11:17:58
 * @LastEditTime: 2022-11-13 17:16:18
 */
#ifndef TRANSMIT_H
#define TRANSMIT_H

//#define SLAVE_NUMBER 2
//#define SLAVE_NUMBER 12 //可该最大从机数

//#include "encos_joint_test/rosLoopMsgDefine.h"

#include <inttypes.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include "sys/time.h"
#include <pthread.h>
#include <utility>

#include "ros_controller/rosLoopMsgDefine.h"

#ifdef __cplusplus
extern "C" {
#endif

#include "ethercat.h"
#include "servo_control/config.h"
#include "servo_control/motor_control.h"

#ifdef __cplusplus
}
#endif

#define feedbackANAMOD (1)      //回传信息解析模式选择：类型1

/*      by 霜云喵
-----------------------
    关节信息传输代码
-----------------------
协议：
1. 注意关节信息传输结构体中，必须保证其传输的顺序与机器人结构匹配

每个传输结构体包含两层vector容器嵌套：
    外层 -- 对应每个CAN板
    内层 -- 依据关节排列顺序，对应每个CAN板上的关节
-----------------------

-----------------------
*/

//----------数据传输用结构体----------

//反馈值接收包装
typedef struct
{
    uint8_t JntID;          //关节ID
    uint8_t type;           //报文类别
    uint8_t error_type;     //错误信息
    float pos;              //电机位置
    float vel;              //电机速度
    float Iq;               //电机电流
    float temperature_Motor;    //电机温度
    float temperature_MOS;      //MOS温度
    uint8_t config_code;    //配置代码
    uint8_t config_state;   //配置状态    
}fdbackRes;
typedef struct 
{
    uint8_t JntNum;
    fdbackRes JntFdBack[14];
}SlaveChannel_fdBack;

//定义辅助结构体 -- 获取CAN板数量以及每个CAN板上关节数量的结构体
typedef struct {
    int CAN_board_num;                          //CAN板数量
    std::vector<int> Jnt_num_on_each_board;     //每个CAN板上关节的数量
}CAN_JNT_numStruct;


//----------------------------------

#ifdef __cplusplus
extern "C" {
#endif

extern int CommandMODE;     //控制模式全局变量，默认为1，可以在其他函数中修改以实现不同的控制律

extern servo_struct ServoStruct;
extern CAN_JNT_numStruct CAN_and_Jnt_num;

void ecatcheck();

void EtherCAT_Transmit();
void EtherCAT_Init(char *ifname);       //EtherCAT初始化代码

//----------位置控制（三环控制）参数配置函数----------
/*
    注意：三环控制中位置环PD参数、速度环PI参数对应取值范围参考值为(0 ~ 0.1)
*/

//位置环PD参数
void EtherCAT_SetPosLoop_PD(std::vector<std::vector<double>> posKp, std::vector<std::vector<double>> posKd);

//PD参数打印 -- 必须在有获取相关操作后使用
std::pair<std::vector<std::vector<double>>, std::vector<std::vector<double>>> Get_posLoop_PD();

//速度环PI参数
void EtherCAT_SetVelLoop_PI(std::vector<std::vector<double>> velKp, std::vector<std::vector<double>> velKi);

//PI参数打印 -- 必须在有获取相关操作后使用
std::pair<std::vector<std::vector<double>>, std::vector<std::vector<double>>> Get_velLoop_PI();

//-----------------------------------

void EtherCAT_direGetFdback();    //直接获取EtherCAT返回值的函数

void EtherCAT_Run();                    //EtherCAT运行主要流程代码
void EtherCAT_Command_Set();            //EtherCAT运行主要流程成员函数 -- 设定EtherCAT命令

void EtherCAT_transInp(std::vector<std::vector<joint_desMsg>> allJnt_desMsg);    //EtherCAT输入值传入函数
std::vector<std::vector<joint_retMsg>> EtherCAT_transOutp();                     //EtherCAT返回值输出函数

//EtherCAT运行流程包装 -- 仅接收运行流程
std::vector<std::vector<joint_retMsg>> runMsgGet();
//EtherCAT运行流程包装 -- 完整收发流程
std::vector<std::vector<joint_retMsg>> runImpl(std::vector<std::vector<joint_desMsg>> allJnt_desMsg); 

void startRun();                        //启动EtherCAT运行函数

// 夹爪控制接口 (力位混控)
void setLeftGripperCmd(double tor);
void setRightGripperCmd(double tor);

CAN_JNT_numStruct CAN_JNT_arrange_outp();       //辅助函数 -- 返回CAN板数量以及各个CAN板上关节数量的结构体
std::vector<double> joint_Kt_get();             //辅助函数 -- 获取关节Kt

extern float IqRange[2];
extern SlaveChannel_fdBack fbBack_Message[SLAVE_NUMBER];
extern pthread_mutex_t data_mutex;
// 缓存上一次的有效数据（可选）
extern SlaveChannel_fdBack last_fbBack_Message[SLAVE_NUMBER];
extern bool last_data_valid;

//---------------调试辅助函数---------------

//输出容器中关节目标信息
std::vector<std::vector<joint_desMsg>> JntDesMsg_inContainer_Outp_v1();      //版本1 -- 直接按照原始结构输出

//打印容器中发送的顺序

//----------------------------------------

#ifdef __cplusplus
}
#endif

#endif // PROJECT_RT_ETHERCAT_H