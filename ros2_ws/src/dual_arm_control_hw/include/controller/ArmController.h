#ifndef __ArmControllerNew__H__
#define __ArmControllerNew__H__
#include <vector>
#include <stdexcept>
#include <tuple>
#include <algorithm>
#include <cmath>

#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/jntarray.hpp>

#include <pinocchio/multibody/fwd.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/rnea.hpp> // 用于逆动力学（含重力项）
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <Eigen/Dense>

#include <fstream>
#include <string>
#include <random>

#include "robot_config/msgDefine.h"
#include "robot_config/config.h"
#include "controller/IKFKsolver.h"

/*
        by 霜云喵
================================
整体控制代码 -- 改进重构版
本代码包为正逆解、机械臂力补偿以及基础点位规划的封装，后续在此进一步封装阻抗控制等
============修改日志============

==============协议==============
臂杆顺序：
    0--左臂
    1--右臂
关节标号：
    0~6 -- 左臂7个关节
    7~13 -- 右臂7个关节
关节角度默认采用弧度制，角度制需要转化成弧度制再运用
注意传入的关节角度数量需要时刻保证与关节数量相等

这里电流常数的定义为：电流乘以这个常数，可以得到电机输出力矩

正解中末端姿态量采用旋转矩阵描述，既R=R_z(Yaw)*R_y(Pitch)*R_x(Roll)

==============封装内容==============

一. 一级控制器：
    1. 重力补偿、惯性力补偿、科氏力补偿
    2. 力前馈（预留接口）
    3. 插补 -- 3种插补模式

二. 综合控制器：
    1. 输入：
        控制模式
            对应控制模式，有不同的插补模式
            对应控制模式，有不同的输入代码
        控制参数
        运动参数

    2. 输出：
        5个控制量
        控制时间

================================
*/

//--------------------------------------------------------------------------------

/*
注意 -- 以下数据不是全都需要传输的，不同控制模式必须传输的数据是不同的
--------------------------------
对于不同的数据传输类型，对于不必须传输的数据，可以直接空着或者传输一个占位的数据
*/

/*  辅助类 -- 插补过程中有关静止阶段信息的容器*/
typedef struct{
    bool ifUseStab;         //判断量 -- 是否在初始阶段静止
    int start_stabTimes;    //初始阶段的静止次数
    int des_stabTimes;      //运动到目标阶段的静止次数
}runinStabMsg;

/*  数据传输控制：
-------------------------
数据含义：
    数据传输模式选择：
        0 -- 归零控制
        1 -- 关节模式（直接传输关节指令）
        2 -- 点位控制（传输点位指令）
        3 -- 关节轨迹控制（连续的关节位置）
        4 -- 点位轨迹控制（连续的点位轨迹）
        5 -- 零力拖动模式
        6 -- MDP接口适配模式
    插补模式选择
        -1 -- 不插补 -- 直接输出逆解结果
        1 -- 线性插补 -- 连续两点间进行，速度近似连续
        2 -- 二次插补 -- 连续两点间进行，速度近似连续
        3 -- 点位插补 -- 包含完整的加减速过程，适配点位控制
        **插补在关节空间进行**
        **使用时一定要看好插补模式是否匹配**
    动力学前馈模式控制
        0 -- 不使用动力学前馈
        1 -- 笛卡尔空间动力学前馈
        2 -- 关节空间动力学前馈
    
    底层关节控制模式选择
        0 -- 保护待机模式
        1 -- 低刚度
        2 -- 中刚度
        3 -- 高刚度
        4 -- 纯位置模式
        5 -- 纯电流模式
-------------------------
    **注意：插补模式必须与点位控制模式适配**
    **注意：该类中采用vector嵌套格式传输数据，
      嵌套外层 -- 对应各个机械臂
      嵌套内层 -- 对应机械臂中的各个关节
    **
    **注意：在使用归零控制后，下一轮不管使用什么控制，ifUseRet必须设定为true（1）**
*/
typedef struct{
    int transMod;               //数据传输模式
    int runinMod;               //插补模式  
    int TorFeedforwardCtl;      //动力学前馈模式控制
    //------求解模式信息------
    int ifUseRet;       //求解模式 -- 是否使用返回值 0--否；1--是
    bool ifUseTemp;     //求解模式 -- 是否使用循环输出
    std::vector<int> base_rigiditiCtl_mod_l;   //求解模式 -- 左臂底层关节控制模式
    std::vector<int> base_rigiditiCtl_mod_r;   //求解模式 -- 右臂底层关节控制模式
    //------插补信息包装------
    double upperMonitor_time;                            //上位机轨迹迭代时间（适配于连续规划）
    std::vector<std::vector<double>> TrajTime;           //各个关节的运动时间（适配于点位规划）
    std::vector<std::vector<double>> accRateBase;        //各个关节加速阶段时间占比
    std::vector<std::vector<runinStabMsg>> m_stabMsg;    //辅助类：静止阶段信息

    /*只有机器人处于关节控制模式下时，这个输入量有意义，该输入量含义为是否令关节处于静止输出模式
    -------------------------------------------------------------------------------
    当处于静止输出模式时，关节位置输出恒定为初始位置
    该判断值： 0--否 ； 1--是
    */
    std::vector<std::vector<int>> ifJntStab;
}modCtl;

//--------------------------------------------------------------------------------

//插补信息类内传递
/*  一般插补模式信息：
-------------------------
    适配于插补模式1、2
*/
typedef struct{
    double mPosDes;          //目标位置
    double mPosSt;           //初始位置
    double mVelSt;           //初始速度
    double mTrajTime;        //目标运动时间
    int runinMod;           //插补模式（填写1或者2,对应插补模式1、2）
}runinMsg;
/*  点位控制插补信息：
-----------------------
    适配插补模式3
*/
typedef struct{
    double mPosDes;                  //目标位置
    double mPosSt;                   //初始位置
    double mTrajTime;                //总运动时间
    double accTimeRateBase;          //加速度占比
}runinMsgDotMod;

/*插补运动轨迹参数*/
typedef struct{
    double a;
    double c;
    double C;
    double K;
    double t;
    double posEnd;
}MotionParams;

/*插补轨迹结果参数*/
struct TrajResult {
    std::vector<double> positionIncrements;
    std::vector<double> VelocityTraj;
};

//类定义结构体——逆解控制输出量
struct ctlOutp {
    double posDes;      //目标位置
    double velDes;      //目标速度

    double grav_TorDes;         //目标重力补偿力矩
    double interia_TorDes;      //目标惯性力补偿力矩
    double feedforward_TorDes;  //目标前馈力矩

    bool ifReachEnd;    //是否达到目标位置
};

//======================================================================
/*一级控制算法*/


//插补类：
class runiner {
    /*插补器*/
    /*注意总运动时间必须是control_T的整数倍*/
    private:
        double control_T;       //控制周期

        double lastTempPosOutp;
        double lastTempVelOutp;

    public:
        /**拟合代码初始化代码
         * deg -- 样条阶数
         * bar_len -- 窗口长度
        */
        void Init(){
            control_T = TEMPTIME;
            lastTempPosOutp = 0.0;
            lastTempVelOutp = 0.0;
        }
        //一次插补
        TrajResult dPos_queue_stabTime_single(double mposDes, double mTrajTime);
        //二次插补
        TrajResult dPos_queue_stabTime_double(double mposDes, double velSt, double mtrajTime);
        //点位插补
        TrajResult dPos_queue_stabTime_dot(double mposDes, double mTrajTime, double accTimeRateBase = 1.2);
        /*备注：最后一个参量的含义为加速时间占到达总位移一半时的总运动时间的比例，这个数值必须大于1*/

        //辅助函数 -- 输出最后一轮的速度
        //上述计算过程中的mposDes指的是相对于初始位置的位置变化量，因此这里引入初始位置将位置量还原
        std::pair<double, double> lastPosVelGet(double posSt = 0.0);
};

//力补偿类：


//辅助函数：力与电流的转换
std::vector<double> Tor2Iq(double Kt[], std::vector<double> Tor, int Len);
std::vector<double> Iq2Tor(double Kt[], std::vector<double> Tor, int Len);

//力补偿类 -- 带有浮动基座的动力学补偿类
// 模型来自 robotProcessedModel (FreeFlyer浮基 + 双臂)
//
// 外部接口采用六维 RPY:
//   base_q_ [x, y, z, roll, pitch, yaw]
//   base_v_ [vx, vy, vz, droll/dt, dpitch/dt, dyaw/dt]
class ForceComp_floatBase{
    private:

        //Pinocchio模型与数据
        pinocchio::Model model_;
        pinocchio::Data data_;

        gravCompInte mRobotGravCompInte;

        //机器人状态矩阵 —— 外部接口采用六维 RPY
        Eigen::VectorXd base_q_;    //基座配置 (6维: x,y,z,roll,pitch,yaw)
        Eigen::VectorXd base_v_;    //基座速度 (6维: vx,vy,vz, dr/dt, dp/dt, dy/dt)
        Eigen::VectorXd q_, v_, a_;                     //全状态（Pinocchio内部布局）
        Eigen::VectorXd v_lst_;                 //上一轮速度

        double temp_T;               //控制周期
        int base_nq_ = 6;       //基座配置维度 (RPY: x,y,z,roll,pitch,yaw)
        int base_nv_ = 6;       //基座速度维度 (RPY导数)
        int arm_nj_  = 0;   //手臂关节数 — 由 init() 根据模型动态计算
        int arm_q_offset_;      //手臂关节在 q_ 中的起始索引
        int arm_v_offset_;      //手臂关节在 v_ 中的起始索引

        //机器人加速度滤波器定义 -- 采用卡尔曼滤波器处理加速度
        std::vector<Kernel::Filter> acc_ftr;

    public:

        //输入输出开关 (1=启用, 0=禁用补偿)
        int inte_inp[14];
        int inte_outp[14];

        //初始化 — 从 robotProcessedModel 复制模型并初始化状态
        bool init(
            std::vector<double> base_pos,
            KDL::JntArray stPos_lArm, KDL::JntArray stPos_rArm
        );

        //更新基座状态 (由外部IMU/里程计提供)
        // base_pos:   [x, y, z, roll, pitch, yaw]   基座在世界中的位姿
        // base_twist: [vx, vy, vz, droll/dt, dpitch/dt, dyaw/dt]  基座速度
        void baseStateUpdate(const std::vector<double>& base_pos,
                             const std::vector<double>& base_twist);

        //重力综合补偿 (重力项，含科氏力耦合)
        // 注意：必须保证补偿函数调用顺序，先调用重力补偿，再调用惯性补偿
        std::vector<double> gravComp(std::vector<double> lArm_pos, std::vector<double> lArm_vel,
                      std::vector<double> rArm_pos, std::vector<double> rArm_vel);

        //惯性综合补偿 (惯性力 + 科氏力)
        std::vector<double> InteriaMulComp(
            std::vector<double> lArm_pos, std::vector<double> lArm_vel,
            std::vector<double> rArm_pos, std::vector<double> rArm_vel);

        //打印模型信息
        void printModelInfo();
};

//======================================================================
/*单关节点位控制代码
-------------------------------------------
  单关节点位控制代码仅包含定时轨迹控制代码
*/

//静止关节代码
class jnt_stabOutp{
    //机器人关节静息输出
    private:
        double JntDeg;

    public:
        void init(double mJntDeg);
        ctlOutp StabOutp();
};

//单关节代码控制代码封装
class moveJntCtl_stabTime{
    private:

        //插补器
        runiner mlocater;
        //控制参数：
        double temp_T;        //控制时间
        int stabRanges;                 //在初始位置静止时长
        int stabTimes;                  //在目标位置停留的时间
        bool ifUseStab;          //是否静息
        int TempTimes;              //迭代次数

        //----------------------------------------
        //过程参数：
        double startPos;     //初始位置
        double endPos;       //末端位置

        //轨迹参数：
        //用于搭配定时轨迹控制
        std::vector<double> posTraj_stabTime;
        std::vector<double> velTraj_stabTime;
        std::vector<double> TorTraj_stabTime;        //力矩输出占位，设定为0
        std::vector<double> posTraj_stabTime_half;
        std::vector<double> velTraj_stabTime_half;
        std::vector<double> TorTraj_stabTime_half;   //力矩输出占位，设定为0

        //----------------------------------------
        //搭配连续轨迹规划的变量
        double posTraj_last;

    public:
        int ID;

        //初始化函数 -- 对应两种控制模式，有两种初始化函数：
        //模式1 -- 适配非连续运动，点到点模式
        /*  求解函数
        --------------------
        输入量含义：
            mCtlMsg -- 求解信息包装
            mstabMsg -- 静止信息包装
        
        */
        void controllerInit(){
            mlocater.Init();
            temp_T = TEMPTIME;
            ifUseStab = true;
            TempTimes = 0;
        }

        void init_stabTime(runinMsgDotMod mCtlMsg, runinStabMsg mstabMsg);
        //输出函数——每调用一次就输出一个相应的输出值
        ctlOutp desPos_get_stabTime();          //做一个从位置A到位置B的往复运动
        ctlOutp desPos_get_stabTimeStop();      //做一个从位置A到位置B的运动并停止在B点
        int TrajLen;    //停留阶段轨迹长度

        //模式2 -- 适配连续运动
        void init_stabTime_continue(runinMsg mCtlMsg);
        ctlOutp desPos_get_stab_Continue();
        int TrajLen_Continue;

        //模式3 -- 静息输出
        jnt_stabOutp mStabOutp;

        /*  辅助函数 -- 输出最后一轮运动的速度
        ------------------------------------
        输出量依据顺序依次为位置、速度
        */
        std::pair<double, double> lastTempPosVel();

        //刷新函数：
        void clear(){
            posTraj_stabTime.clear();
            velTraj_stabTime.clear();
            TorTraj_stabTime.clear();
            posTraj_stabTime_half.clear();
            velTraj_stabTime_half.clear();
            TorTraj_stabTime_half.clear();

            this->TempTimes = 0;
            this->ifUseStab = true;
        }
};

//======================================================================
/*综合控制算法*/
double double2Float(double inp);                 //快捷转换函数 -- double变为float      

/*  针对机器人整体直接传输关节参数的求解代码封装
---------------------------------------------
    注意：力轨迹与点位控制不适配，其对应的力输出默认为0
    仅当关节处于轨迹模式时，才会涉及到力前馈的控制
*/
class JntTrackSol{
    private:
        //机器人关节求解器：
        std::vector<moveJntCtl_stabTime> lArmJntTrajSol;
        std::vector<moveJntCtl_stabTime> rArmJntTrajSol;
        std::vector<jnt_stabOutp> lArmStab;
        std::vector<jnt_stabOutp> rArmStab;

        //机器人力矩求解器 -- 力矩轨迹插补
        std::vector<moveJntCtl_stabTime> lArmJntTorTrajSol;
        std::vector<moveJntCtl_stabTime> rArmJntTorTrajSol;

        //过程量：
        KDL::JntArray posLst_lArm;      //上一轮关节角度 -- 左臂
        KDL::JntArray posLst_rArm;      //上一轮关节角度 -- 右臂

        KDL::JntArray velLst_lArm;      //上一轮角速度 -- 左臂
        KDL::JntArray velLst_rArm;      //上一轮角速度 -- 右臂

        std::vector<double> torLst_lArm;    //上一轮力矩输出 -- 左臂
        std::vector<double> torLst_rArm;    //上一轮力矩输出 -- 右臂

        int outp_mod;       //计算控制量--用于输出模式的控制
        bool ifUseTemp = false; //控制量：是否使用循环输出
        bool ifFirstTemp = true;    //控制量：是否为第一轮输出

        std::vector<std::vector<int>> ifJntStab;    //关节输出模式 -- 关节是否静止

    public:

        //初始化：
        void init(KDL::JntArray posSt_lArm, KDL::JntArray posSt_rArm);

        //刷新初始位置接口
        void updateStPos(KDL::JntArray posSt_lArm, KDL::JntArray posSt_rArm){
            this->posLst_lArm = posSt_lArm;
            this->posLst_rArm = posSt_rArm;
        }

        void JntTrajCal(
            //目标量
            KDL::JntArray posDes_lArm, KDL::JntArray posDes_rArm,
            std::vector<double> JntTorDes_l, std::vector<double> JntTorDes_r,

            //反馈量
            KDL::JntArray posRet_lArm, KDL::JntArray posRet_rArm,
            KDL::JntArray velRet_lArm, KDL::JntArray velRet_rArm,
            std::vector<double> JntTorRet_l, std::vector<double> JntTorRet_r,

            modCtl mod_ctl_msg,
            bool ifRedefineStab = false         //是否刷新关节静止值
        );
        //这里放一个轨迹长度参数，放在类外便于交互
        std::vector<double> lArmTrajLen;
        std::vector<double> rArmTrajLen;

        std::vector<std::vector<ctlOutp>> solveResOutp(std::vector<double> JntTorRet_l, std::vector<double> JntTorRet_r);

        
};

/*逆解轨迹与正解算法封装*/
//辅助函数
std::vector<double> Frame_to_vector(KDL::Frame frameInp);           //Frame转化成vector类型变量
KDL::Frame vector_to_Frame(std::vector<double> vecInp);             //vector类型变量转化成Frame

//算法封装
class FrameTrackSol{
    private:
        //逆解正解求解器：
        IkFk_solverPino mSol_Pino;

        //----------插补器----------

        //适配于连续轨迹的插值计算
        std::vector<moveJntCtl_stabTime> lArm_JntTrajSol;
        std::vector<moveJntCtl_stabTime> rArm_JntTrajSol;

        //适配于点到点的插值计算
        std::vector<moveJntCtl_stabTime> lArm_DescTrajSol;
        std::vector<moveJntCtl_stabTime> rArm_DescTrajSol;

        //--------------------------
        //求解过程控制代码
        modCtl modCtlMsg;
        KDL::JntArray posInp_l, posInp_r;
        KDL::JntArray velInp_l, velInp_r;

        //----------

        //过程参量：
        KDL::JntArray lArmLstPos;
        KDL::JntArray rArmLstPos;
        KDL::JntArray lArmLstVel;
        KDL::JntArray rArmLstVel;

        //--------------------------

        //适配恒定输出的过程参量：
        KDL::JntArray lArmDesPos;
        KDL::JntArray rArmDesPos;

        //修正参量
        std::vector<double> lArm_posLst_velCor;
        std::vector<double> rArm_posLst_velCor;

        //过程控制参量   
        double tempT;
        bool if_not_firstTemp;  //过程控制量--用于限制第一轮研究中不能重新优化
        bool ifUseTemp;     //过程控制量--用于点到点模式，用于表明是不是循环输出模式
        int outp_mod;       //计算控制量--用于输出模式的控制

        int stabTimesCount;
        int allStabTimes_l;
        int allStabTimes_r;

        bool ifUseSolve;     //判断量：是否求解过

        //关节静止模式：
        bool ifFirstTemp = true;
        std::vector<std::vector<int>> ifJntStab;    //关节输出模式 -- 关节是否静止
        //关节静止控制器
        std::vector<jnt_stabOutp> lArmStab;         
        std::vector<jnt_stabOutp> rArmStab;

        //---------------
        //权重控制代码
        //点到点权重控制
        std::vector<double> p2p_optiWeight = {
            50.0, 1.0, 0.001, 0.001, 0.00002
        };
        //上一轮控制模式
        int controlMod_lastTemp = 0;

        //----------
        //轨迹修正代码 -- 描述站立情况下torso相对于universe的位姿
        std::vector<double> torso_pos_l = {
            0.0, 0.0, 0.0
        };
         std::vector<double> torso_pos_r = {
            0.0, 0.0, 0.0
        };

        //成员函数 -- 求解单次逆解的成员函数
        void IK_Traj_solve(
            KDL::Frame lArm_desPos, KDL::Frame rArm_desPos,
            KDL::JntArray lArmPosRet, KDL::JntArray rArmPosRet,
            bool ifUseRet,
            double TrajTime,
            int runinMod
        );

    public:

        /*初始化求解器 -- 返回 true 表示成功，false 表示失败*/
        bool init_solver(
            JntLimMsg lArm_LimJnt, JntLimMsg rArm_LimJnt,       //关节限制配置
            std::vector<double> JntAngConf_l, std::vector<double> JntAngConf_r,     //关节初始角度配置
            optiPrintMsgCtl mPrintMsgPkg,           //程序打印控制参数
            optimizerCtl m_opti_ctl_msg        //优化器参数
        );

        //更新初始位置
        void updateStPos(KDL::JntArray posSt_lArm, KDL::JntArray posSt_rArm){
            this->lArmLstPos = posSt_lArm;
            this->rArmLstPos = posSt_rArm;
        }

        /*  求解代码
        ---------------
        这里添加力反馈接口，用于集成力控
        */
        void FrameTrack(
            bool ifUpdateLim,               //判断量 -- 是否更新限制信息
            JntLimMsg allArm_LimJnt,        //求解关节限制信息，将所有的关节信息都放在这里面

            KDL::Frame lArm_desPos, KDL::Frame rArm_desPos,         //求解目标位置
            std::vector<double> lArm_desTor, std::vector<double> rArm_desTor,   //目标前馈力矩

            KDL::JntArray lArmPosRet, KDL::JntArray rArmPosRet,     //位置返回值
            KDL::JntArray lArmVelRet, KDL::JntArray rArmVelRet,     //速度返回值
            std::vector<double> lArmTorRet, std::vector<double> rArmTorRet,     //关节力矩反馈值

            modCtl mod_ctl_msg          //数据传输控制过程
        );
        //这里放一个轨迹长度参数，放在类外便于交互
        std::vector<double> lArm_TrajLen;
        std::vector<double> rArm_TrajLen;

        /*  逆解轨迹输出
        -----------------
        同时输出双臂的轨迹，先左臂，后右臂
        */
        std::vector<std::vector<ctlOutp>> IKsolveResOutp(std::vector<double> JntTorRet_l, std::vector<double> JntTorRet_r);

        //正解结果输出接口函数
        std::vector<KDL::Frame> FkSolver(KDL::JntArray lArm_JntArray, KDL::JntArray rArm_JntArray){
            return mSol_Pino.Fk_pinoSolver(lArm_JntArray, rArm_JntArray);
        }

        // -------- 动力学映射求解 -------

        //末端力到关节力的映射
        RobotJntTor EndTor_To_Jnt(
            std::vector<double> TorEnd_l, std::vector<double> TorEnd_r,
            KDL::JntArray lArm_JntPos, KDL::JntArray rArm_JntPos
        ){
            return mSol_Pino.EndTor_To_Jnt(
                TorEnd_l, TorEnd_r,
                lArm_JntPos, rArm_JntPos
            );
        }

        //关节力到末端力的映射
        RobotFrameTor JntTor_to_End(
            RobotJntTor JntTor,
            KDL::JntArray lArm_JntPos, KDL::JntArray rArm_JntPos,
            double damping_lambda
        ){
            return mSol_Pino.JntTor_to_End(
                JntTor,
                lArm_JntPos, rArm_JntPos,
                damping_lambda
            );
        }

        //------------------------------
        double reserve[14] = {
            0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0
        };

        double dbg_a = 0;
};


#endif
