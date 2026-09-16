#ifndef __demoTraj__H__
#define __demoTraj__H__

#include<iostream>
#include<vector>
#include<string>
#include<math.h>

#include "robot_control/config.h"

#include "robot_control/msgType.h"
#include "messageType/upper_msgType.h"
#include "messageType/chassis_msgType.h"

#include "debugPublish/debugPublish.h"

/*
        by 霜云喵
==========================
测试轨迹代码包
==========================
本代码包包含所有预设的测试轨迹
未来其他控制代码书写规范与方法可以参照本代码包裹

注意增添输出过程中修改输出参数的代码段

==========================
*/

//===============================================================================================

/*插补器*/
class runiner {
    /*插补器*/
    /*注意总运动时间必须是control_T的整数倍*/
    private:
        float control_T = TempTime;//控制周期

        float lastTempPosOutp = 0.0f;
        float lastTempVelOutp = 0.0f;

    public:
        /**拟合代码初始化代码
         * deg -- 样条阶数
         * bar_len -- 窗口长度
        */

        void Init(){
            control_T = TempTime;
            lastTempPosOutp = 0.0f;
            lastTempVelOutp = 0.0f;
        };

        //一次插补
        TrajResult dPos_queue_stabTime_single(float mposDes, float mTrajTime);
        //二次插补
        TrajResult dPos_queue_stabTime_double(float mposDes, float velSt, float mtrajTime);
        //点位插补
        TrajResult dPos_queue_stabTime_dot(float mposDes, float mTrajTime, float accTimeRateBase = 1.2);
        /*备注：最后一个参量的含义为加速时间占到达总位移一半时的总运动时间的比例，这个数值必须大于1*/

        //辅助函数 -- 输出最后一轮的速度
        //上述计算过程中的mposDes指的是相对于初始位置的位置变化量，因此这里引入初始位置将位置量还原
        std::pair<float, float> lastPosVelGet(float posSt = 0.0f);
};

//核心类 -- 用于储存所有的演示轨迹
//注意针对每个初始化、输出函数刷一遍，看看有没有轨迹编号与变量标号不匹配的情况
//注意这些轨迹的参数暂时还没有精调，需要进一步精调以保障轨迹质量
class allDemoTraj{
    public:
        int m_rigidityCtlMod = 1;   //底层控制模式，注意，纯位置模式和电流模式不要管这个，直接设就行

        //----------轨迹1----------
        //纯点位控制轨迹（可以兼顾用作负载测试轨迹）
        /**/
        void init_v1();
        DotOutpWrap TrajOutp_v1();
        /**/

        //----------轨迹2----------
        //定位精度测试轨迹 -- 先抬起到目标位置，再作往复运动
        void init_v2();
        DotOutpWrap TrajOutp_v2();

        //过程控制参数
        bool ifFirstSend_v2 = false;
        bool ifSecondSend_v2 = false;

        //----------轨迹3----------
        //极限速度测试轨迹 -- 先抬起至起始位置，再开始运动
        void init_v3();
        DotOutpWrap TrajOutp_v3();

        //过程控制参数
        bool ifFirstSend_v3 = false;
        bool ifSecondSend_v3 = false;


        //----------轨迹4----------
        //轨迹控制轨迹
        void init_v4();
        DotOutpWrap TrajOutp_v4();

        //----------轨迹5----------
        //零空间控制轨迹 -- 先用点位控制运动到目标点位，再执行零空间轨迹

        void init_v5();
        DotOutpWrap TrajOutp_v5();
        bool ifReachStartPos_v5 = false;    //控制变量 -- 是否到达初始位置
        bool initCtl_v5 = false;

        //----------轨迹6---------
        //三点点位控制代码
        void init_v6();
        DotOutpWrap TrajOutp_v6();

        //----------轨迹7---------
        //零力拖动demo
        void init_v7();
        DotOutpWrap TrajOutp_v7();

        //----------轨迹8----------
        //关节空间连续轨迹发送测试
        void init_v8();
        DotOutpWrap TrajOutp_v8();

        //----------轨迹9----------
        //余弦扫频轨迹测试 — 关节空间连续轨迹控制模式
        void init_v9();
        DotOutpWrap TrajOutp_v9();

        //----------轨迹10----------
        //正弦扫频轨迹测试 — MDP传输模式，关节空间位置+速度透传
        void init_v10();
        DotOutpWrap TrajOutp_v10();

        //----------轨迹11----------
        //控制模式切换测试轨迹 — 点位与连续轨迹混合切换
        //模式1: 笛卡尔空间 点位→轨迹→点位 循环
        //模式2: 关节空间   点位→轨迹→点位 循环
        void init_v11();
        DotOutpWrap TrajOutp_v11();

        //----------demo操作界面-----------

    private:
        DotOutpWrap mdataSend;     //输出数据包装

        //----------轨迹1参数----------
        /**/
        ctlDesMsgWrap ctlStMsg_v1;
        ctlDesMsgWrap ctlDesMsg_v1;
        /**/

        //----------轨迹2参数----------

        ctlDesMsgWrap ctlStMsg_v2;

        ctlDesMsgWrap ctlDesMsg_v2_p1;
        std::vector<int> lArm_ifJntStab_v2_1, rArm_ifJntStab_v2_1;

        ctlDesMsgWrap ctlDesMsg_v2_p2;
        std::vector<int> lArm_ifJntStab_v2_2, rArm_ifJntStab_v2_2;

        bool ifSecondPoint_v2;      //是否为第2个点对应的运动过程

        //----------轨迹3参数----------
        ctlDesMsgWrap ctlStMsg_v3;

        ctlDesMsgWrap ctlDesMsg_v3_p1;
        std::vector<int> lArm_ifJntStab_v3_1, rArm_ifJntStab_v3_1;

        ctlDesMsgWrap ctlDesMsg_v3_p2;
        std::vector<int> lArm_ifJntStab_v3_2, rArm_ifJntStab_v3_2;

        bool ifSecondPoint_v3;      //是否为第2个点对应的运动过程

        bool ifRedefineStabJntAng = true;

        double traj_time_v3;

        //----------轨迹4参数----------
        ctlDesMsgWrap ctlStMsg_v4;      //初始位置包装
        ctlDesMsgWrap ctlDesMsg_v4;     //目标位置包装
        std::vector<ctlDesMsgWrap> ctlDesTraj_v4;  //关节限制轨迹

        int trajLen;        //单条运动轨迹长度
        int all_len;        //轨迹总长度
        int temp_times;     //迭代次数

        std::vector<runiner> lArm_DescRuniner_v4;
        std::vector<runiner> rArm_DescRuniner_v4;

        std::vector<runiner> lArm_JntRuniner_v4;
        std::vector<runiner> rArm_JntRuniner_v4;

        //轨迹4新增控制参数 -- 让机械臂高速动三次后强制到目标点进入中等阻抗
        int counter_1 = 0;      //计数器1 -- 用于衡量是否到达目标位置的变量
        int counter_2 = 0;      //计数器2 -- 每当计temp_times大小等于轨迹长度，该数值加1,并请零counter_1
        int all_count = 3;      //计数器3 -- 默认为3,用于控制轨迹总长度
        bool ifReachDesPos = false;     //判断变量 -- 是否到达目标位置

        //----------轨迹5参数----------
        
        ctlDesMsgWrap ctlStMsg_v5;
        ctlDesMsgWrap ctlDesMsg_v5;

        JntLimMsgWrap stJntLimMsg_l;                //轨迹初始端受限关节信息
        JntLimMsgWrap endJntLimMsg_l;               //轨迹终点端受限关节信息
        JntLimMsgWrap stJntLimMsg_r;                //轨迹初始端受限关节信息
        JntLimMsgWrap endJntLimMsg_r;               //轨迹终点端受限关节信息
        std::vector<JntLimMsgWrap> lArmJntLimMsg;
        std::vector<JntLimMsgWrap> rArmJntLimMsg;

        int trajLen_v5;     //单条运动轨迹长度
        int all_len_v5;     //运动轨迹总长度
        int temp_times_v5;  //迭代次数

        //----------轨迹6参数----------
        ctlDesMsgWrap ctlStMsg_v6;      //初始位置包装
        std::vector<ctlDesMsgWrap> ctlDesPoints_v6;  //关节限制轨迹
        std::vector<std::vector<ctlDesMsgWrap>> ctlDesTraj_v6;    //完整的轨迹序列（需要添加）

        std::vector<int> all_len_v6;                    //轨迹总长度（需要添加）
        int all_singleJntTempTimes_v6 = 0;
        int singleJntTempTimes_v6 = 0;      //单个点到达的次数
        int temp_times_v6 = 0;
        int dotID_v6 = 0;
        bool ifFw = true;
        int tol = 2;

        int point_choose_v6 = 0;

        std::vector<runiner> lArm_DescRuniner_v6;
        std::vector<runiner> rArm_DescRuniner_v6;

        std::vector<runiner> lArm_JntRuniner_v6;
        std::vector<runiner> rArm_JntRuniner_v6;

        //----------轨迹7参数----------

        ctlDesMsgWrap ctlStMsg_v7;
        ctlDesMsgWrap ctlDesMsg_v7;

        //----------轨迹8参数----------
        ctlDesMsgWrap ctlStMsg_v8;      //初始位置包装
        ctlDesMsgWrap ctlDesMsg_v8;     //目标位置包装
        std::vector<ctlDesMsgWrap> ctlDesTraj_v8;  //关节空间连续轨迹

        int trajLen_v8;         //单条运动轨迹长度
        int all_len_v8;         //轨迹总长度
        int temp_times_v8;      //迭代次数

        std::vector<runiner> lArm_JntRuniner_v8;
        std::vector<runiner> rArm_JntRuniner_v8;

        //轨迹8新增控制参数 -- 循环控制
        int counter_1_v8 = 0;       //计数器1 -- 用于衡量是否到达目标位置的变量
        int counter_2_v8 = 0;       //计数器2 -- 每当计temp_times大小等于轨迹长度，该数值加1
        int all_count_v8 = 3;       //计数器3 -- 默认为3，用于控制轨迹总循环次数
        bool ifReachDesPos_v8 = false;  //判断变量 -- 是否到达目标位置

        //----------轨迹9参数----------
        //正弦轨迹 — 每个关节独立的正弦信号 (固定频率)
        ctlDesMsgWrap ctlStMsg_v9;      //初始位置包装
        std::vector<ctlDesMsgWrap> ctlDesTraj_v9;  //预计算的正弦轨迹序列

        int trajLen_v9;             //轨迹序列总长度
        int temp_times_v9;          //当前输出迭代次数
        int all_count_v9;           //循环总次数
        int counter_2_v9;           //已完成循环次数计数器

        // 正弦轨迹可调参数 — 每关节独立配置
        // pos(t) = center + amplitude * sin(2π * f * t)
        std::vector<double> v9_lArm_center;     //左臂各关节中心位置 (rad)
        std::vector<double> v9_rArm_center;     //右臂各关节中心位置 (rad)
        std::vector<double> v9_lArm_amplitude;  //左臂各关节振幅 (rad)
        std::vector<double> v9_rArm_amplitude;  //右臂各关节振幅 (rad)
        std::vector<double> v9_lArm_freq;       //左臂各关节正弦频率 (Hz)
        std::vector<double> v9_rArm_freq;       //右臂各关节正弦频率 (Hz)
        double v9_traj_duration;                //单次轨迹持续时间 (s)
        double v9_sample_dt;                    //轨迹采样间隔 (s)，默认 = TempTime

        //----------轨迹11参数----------
        //控制模式切换测试轨迹
        int v11_mode;                       // 1=笛卡尔空间切换, 2=关节空间切换
        int v11_phase;                      // 0=点位→中继点, 1=轨迹→目标点, 2=点位→初始位
        int v11_allCount;                   // 总循环次数
        int v11_counter;                    // 已完成循环次数
        bool v11_justEnteredPhase;          // 刚进入新阶段标记

        ctlDesMsgWrap v11_ctlStMsg;         // 初始位置包装
        ctlDesMsgWrap v11_pointA;           // 笛卡尔空间 - 中继点A
        ctlDesMsgWrap v11_pointB;           // 笛卡尔空间 - 目标点B
        ctlDesMsgWrap v11_pointC;           // 关节空间 - 中继点C
        ctlDesMsgWrap v11_pointD;           // 关节空间 - 目标点D

        std::vector<ctlDesMsgWrap> v11_traj; // 预计算轨迹序列
        int v11_trajLen;                     // 轨迹序列长度
        int v11_trajIdx;                     // 当前轨迹索引

        //----------轨迹10参数----------
        //正弦 MDP 模式 — 预计算位置+速度轨迹 (固定频率)
        std::vector<std::vector<double>> v10_lArm_PosTraj;   // [timestep][joint]
        std::vector<std::vector<double>> v10_rArm_PosTraj;
        std::vector<std::vector<double>> v10_lArm_VelTraj;   // [timestep][joint]
        std::vector<std::vector<double>> v10_rArm_VelTraj;

        int v10_trajLen;                        //轨迹序列总长度
        int v10_tempTimes;                      //当前输出迭代次数
        int v10_allCount;                       //循环总次数
        int v10_counter2;                       //已完成循环次数计数器

        // 正弦轨迹可调参数 — 每关节独立配置
        // pos(t) = center + amplitude * sin(2π * f * t)
        // vel(t) = amplitude * 2π * f * cos(2π * f * t)
        std::vector<double> v10_lArm_center;    //中心位置 = lArm_zeroPos
        std::vector<double> v10_rArm_center;    //中心位置 = rArm_zeroPos
        std::vector<double> v10_lArm_amplitude; //振幅 (rad)
        std::vector<double> v10_rArm_amplitude;
        std::vector<double> v10_lArm_freq;      //正弦频率 (Hz)
        std::vector<double> v10_rArm_freq;
        double v10_trajDuration;                //单次轨迹持续时间 (s)
        double v10_sampleDt;                    //采样间隔 (s)
};

//===============================================================================================


#endif