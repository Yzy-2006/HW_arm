#ifndef __msgDefine__H__
#define __msgDefine__H__

#include <iostream>
#include <vector>
#include <string>
#include <Eigen/Dense>

//--------------------求解器关键变量--------------------

extern double TEMPTIME;
extern int ALLJNTNUM;
extern int LARMDOF;
extern int RARMDOF;

//加载机械臂urdf文件路径

extern std::vector<std::string> path_options;
extern std::vector<std::string> base_link;
extern std::vector<std::string> tip_link;

// ----------------------------------------
// 数据类型包装

//单关节限制信息
struct JntLimit{
    double lower;          // 角度下限 (rad)
    double upper;          // 角度上限 (rad)
    double velocity;       // 角速度限制 (rad/s)
    double acceleration;   // 角加速度限制 (rad/s²)
    double torque;         // 力矩限制 (N·m)
};

//机器人机械臂关节运动信息包装
struct RobotJntMsg{
    std::vector<std::string> lArm_frame_names;      //左臂连杆名称列表
    std::vector<std::string> rArm_frame_names;      //右臂连杆名称列表

    std::vector<std::string> lArm_JntRotation_axis;     //左臂关节旋转轴
    std::vector<std::string> rArm_JntRotation_axis;     //右臂关节旋转轴

    int lArm_JntNum;                //左臂关节数目
    int rArm_JntNum;                //右臂关节数目
};

//结构体 -- 储存机械臂模型配置的结构体
struct RobotStructure{
    //基座名称
    std::string base_name;
    //基座相对于世界坐标系的转换矩阵
    std::vector<double> base_to_world;

    std::vector<std::string> lArm_joint_names;      //左臂连杆名称列表
    std::vector<std::string> rArm_joint_names;      //右臂连杆名称列表

    std::vector<JntLimit> lArm_JntLimits;               //左臂关节限制
    std::vector<JntLimit> rArm_JntLimits;               //右臂关节限制

    std::vector<double> lArm_jnt_Kt;                    //左臂电流-力矩常数
    std::vector<double> rArm_jnt_Kt;                    //右臂电流-力矩常数

    int lArm_JntNum;                //左臂关节数目
    int rArm_JntNum;                //右臂关节数目

    //左右臂工具链定义参数
    std::string lArm_tool_base;
    std::vector<std::string> lArm_tool_chain;

    std::string rArm_tool_base;
    std::vector<std::string> rArm_tool_chain;
};


//机器人底盘运动
struct RobotBaseMovement{
    std::vector<double> base_pos;
    std::vector<double> base_vel;
    std::vector<double> base_acc;
};

//机器人关节运动包装
struct RobotJntMovement{
    std::vector<double> lArm_Joint_pos;
    std::vector<double> lArm_Joint_vel;
    std::vector<double> lArm_Joint_acc;

    std::vector<double> rArm_Joint_pos;
    std::vector<double> rArm_Joint_vel;
    std::vector<double> rArm_Joint_acc;
};

//机器人各个关节-连杆体系空间运动包装
struct RobotLinkMovement{
    std::vector<std::vector<double>> lArm_vel;          //线速度
    std::vector<std::vector<double>> lArm_omega;        //角速度
    std::vector<std::vector<double>> lArm_acc;          //线加速度
    std::vector<std::vector<double>> lArm_domega;       //角加速度
    std::vector<std::vector<double>> lArm_acc_center;   //质心加速度

    std::vector<std::vector<double>> rArm_vel;          //线速度
    std::vector<std::vector<double>> rArm_omega;        //角速度
    std::vector<std::vector<double>> rArm_acc;          //线加速度
    std::vector<std::vector<double>> rArm_domega;       //角加速度
    std::vector<std::vector<double>> rArm_acc_center;   //质心加速度
};

//机器人力矩包装
struct RobotJntTor{
    std::vector<double> lArm_JntTor;
    std::vector<double> rArm_JntTor;
};
struct RobotFrameTor{
    std::vector<double> lArm_EndTor;
    std::vector<double> rArm_EndTor;
};

//-----------------------------------------

//结构体：低通滤波器参数
struct lowpassFtrInte{
    double sampRate;
    double cutoffRate;
};

//结构体：卡尔曼滤波器参数
struct kalmanFtrInte{
    double Q_gain;
    double R_gain;
};

//-----------------------------------------

//结构体：机器人重力补偿滤波器参数包装
struct gravCompFtr{
    std::vector<lowpassFtrInte> lArm_posInpFtr;
    std::vector<lowpassFtrInte> lArm_velInpFtr;
    std::vector<lowpassFtrInte> lArm_torInpFtr;
    std::vector<lowpassFtrInte> lArm_torOutpFtr;

    std::vector<lowpassFtrInte> rArm_posInpFtr;
    std::vector<lowpassFtrInte> rArm_velInpFtr;
    std::vector<lowpassFtrInte> rArm_torInpFtr;
    std::vector<lowpassFtrInte> rArm_torOutpFtr;

    //关节角度/角速度输出低通滤波器（新增）
    std::vector<lowpassFtrInte> lArm_posOutpFtr;
    std::vector<lowpassFtrInte> lArm_velOutpFtr;
    std::vector<lowpassFtrInte> rArm_posOutpFtr;
    std::vector<lowpassFtrInte> rArm_velOutpFtr;
};

//结构体：机器人重力补偿参数包装
struct gravCompInte{

    //开关变量
    int ifUseGravComp;      //是否打开重力补偿
    int ifUseInpFtr;        //是否使用输入滤波
    int ifUseOutpFtr;       //是否使用输出滤波
    int ifUseFricComp;      //是否使用摩擦补偿
    int ifUsePosVelOutpFtr; //是否使用关节角度/角速度输出低通滤波器

    //模式控制变量
    int InpDataMod;         //数据输入模式

    //初始化平滑输出控制变量
    int GravCompStZeroTimes;            //平滑输出初始阶段零输出轮数
    int GravCompStSmoothTimes;          //平滑输出初始阶段平滑输出迭代轮数
    int JntCtlStStabTimes;              //机器人控制初始化完成后空跑输出轮数

    //参数变量
    std::vector<double> gravCompInte;
    std::vector<double> coriousCompInte;
    std::vector<double> InteriaCompInte;

    std::vector<kalmanFtrInte> acc_ftr;
    gravCompFtr gravComp_InpOutpFtr;
};

// ----------------------------------------
// 基于优化器的逆解参数包装

//信息打印参数
typedef struct{
    bool dispJntInfo;               //开关量——是否打印
    bool use_visualization;
    bool disp_messages;
}optiPrintMsgCtl;

//优化器控制类：
typedef struct{
    double OptiWeight[5];           //控制器优化权重——四个权重分别对应位置误差权重、旋转误差权重、正则化成本权重(避免角度过大)、以及平滑项权重
    int template_times;             //控制器优化迭代次数
    double errTol;                  //误差容差
    int ifPrintLevel;               //是否打印优化日志
    bool ifPrintOptiTime;           //是否打印计算时间
}optimizerCtl;

// ----------------------------------------
//总控流程参数
typedef struct{

    //中层节点控制参数
    double midNode_tempTime;
    int lArm_Dof;
    int rArm_Dof;

    std::string urdf_path_option;
    std::string base_link_lArm;
    std::string base_link_rArm;
    std::string tip_link_lArm;
    std::string tip_link_rArm;

    //位置反馈值通用滤波参数 -- 采用卡尔曼滤波
    std::vector<kalmanFtrInte> lArm_posFeedback_uniFtr;
    std::vector<kalmanFtrInte> rArm_posFeedback_uniFtr;

    //电力反馈值通用滤波函数 -- 采用卡尔曼滤波
    std::vector<kalmanFtrInte> lArm_IqFeedback_uniFtr;
    std::vector<kalmanFtrInte> rArm_IqFeedback_uniFtr;

}TransLoopCtl;

// ----------------------------------------

#endif
