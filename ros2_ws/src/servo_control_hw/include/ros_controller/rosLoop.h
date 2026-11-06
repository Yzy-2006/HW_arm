#ifndef SERVO_CONTROL_HW__ROS_CONTROLLER__ROSLOOP_H_
#define SERVO_CONTROL_HW__ROS_CONTROLLER__ROSLOOP_H_

#include <cstdio>
#include <csignal>
#include <cstring>
#include <atomic>
#include <string>
#include <vector>
#include <iostream>
#include <fstream>
#include <thread>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "servo_control/zero_pos_config.h"
#include "servo_control/command.h"
#include "servo_control/config.h"
#include "servo_control/motor_control.h"
#include "servo_control/transmit.h"
#include "servo_control/queue.h"

#include <memory>
#include <chrono>
#include <functional>
#include "rclcpp/executors/single_threaded_executor.hpp"

#include "debugPublish/debugPublish.h"

#include "ros_controller/ctlConfig.h"
#include "ros_controller/rosLoopMsgDefine.h"

#include "JntController/runinController.h"
#include "JntController/JntTestCtl.h"
#include "JntController/ForceComp.h"
#include "JntController/retZeroControl.h"
#include "servo_control/homingHandoff.h"

#include "ctl_msgs_hw/msg/ctl_information.hpp"
#include "ctl_msgs_hw/msg/handoff_status.hpp"
#include "ctl_msgs_hw/msg/ret_information.hpp"
#include "ctl_msgs_hw/msg/jnt_comp_res_visu.hpp"
#include "ctl_msgs_hw/msg/dbg_information.hpp"
#include "ctl_msgs_hw/msg/gripper_ctl.hpp"

extern "C" {
#include "ethercat.h"
}

/*
        by 霜云喵
================================
通讯主函数代码（下位机）
============修改日志============

==============协议==============
本代码为下位机节点主函数代码
控制流程为：
    1. 先获取机械臂关节返回值，并发送
    2. 然后获取上位机发送的指令
    3. 将上位机发送的指令传输给关节

通讯成功后，当第一轮时，发送的返回值均为0,并发送初始轮标志
若通讯失败，未能成功获取上位机指令，则进入零刚度高阻尼模式，回零位

注意：Kp, Kd不参与插补；除了位置量以外其他量在开始插补时默认为0

==============通讯协议1==============

中层节点与下位机的通讯。下位机：关节通讯节点

传输信息：

1. 控制信息：
    temp_time -- 本中层节点的循环周期，用于与下层节点协调周期，并用于插补
    runin_mod -- 下位机（关节通讯节点）插补模式
        0 -- 不插补
        1 -- 线性插补
        2 -- 二次插补
    if_runin_renew -- 是否刷新下位机插补器
        0 -- 否
        1 -- 是
    if_useret_to_start_runin -- 是否使用返回值作开始插补时第一轮输入
        0 -- 否
        1 -- 是
    if_use_debugMod -- 是否使用调试模式
        0 -- 否
        1 -- 是
    l_position_at_runin_st 、r_position_at_runin_st -- 下发的插补开始时初始值
    
    place_hold -- 位置保持模式控制参量：
        1 -- 开启位置保持模式
        0 -- 关闭位置保持模式

2. 左右臂目标位置，速度、三类目标力矩、目标kp, kd值
    此处略，不再赘述。

3. reserve量（保留量）

三种传输模式的传输协议：
1. 力位混合模式
  关键的传输量有三个：
  期望位置、期望速度、期望力前馈。力位混合控制有不同的刚度模式，其在下层对应不同的Kp、Kd配置。

2. 位置控制模式
  关键的传输量有两个：
  期望位置、期望速度。其他的控制量置0占位或合理填充即可。

3. 电流控制模式
  关键的传输量只有1个：
  期望电流。期望电流通过力前馈传输通道赋值。其他量置0或者合理填充即可


================================

排列顺序协议：传输过程中，
    前7个关节 -- 对应左臂
    第8到第14个关节 -- 对应右臂

================================
*/

extern std::vector<double> joint_pos;
extern std::vector<double> joint_kp;
extern std::vector<double> joint_kd;
extern std::vector<double> joint_spd;
extern std::vector<double> joint_tor;



class ServoNode : public rclcpp::Node {
    public:
        //构造函数（初始化用）
        ServoNode();

        //析构函数（performance_test 由 unique_ptr 自动管理）
        ~ServoNode() = default;

        //ros循环通讯变量初始化函数
        void rosLoopVar_Init();

        //ros消息参量发送模块 -- 反馈值发送函数
        void msg_publish();

        //ros消息回调函数 -- 上位机控制消息接收模块
        void ctlmsg_get();

        //消息传输辅助函数 -- 仅仅进行一次消息的接收(与底层关节的通讯)
        void EtherCATmsg_Get();

        //消息传输辅助函数 -- 进行一次消息的收发(与底层关节的通讯)
        void EtherCATrun();

        //主要循环函数
        void run();

    private:
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr safety_pub_;
        rclcpp::Publisher<ctl_msgs_hw::msg::HandoffStatus>::SharedPtr handoff_pub_;
        std_msgs::msg::Header last_source_header_;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr safety_stop_;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr safety_reset_;
        rclcpp::TimerBase::SharedPtr safety_timer_;
        //====================消息发送变量====================

        //----------反馈信息发送----------
        rclcpp::Time global_timestamp_;

        //上位机通讯消息发送
        rclcpp::Publisher<ctl_msgs_hw::msg::RetInformation>::SharedPtr RetData_pub;    //返回值发送
        rclcpp::Subscription<ctl_msgs_hw::msg::CtlInformation>::SharedPtr CtlData;     //控制量获取

        //debug消息发送接口
        rclcpp::Publisher<ctl_msgs_hw::msg::DbgInformation>::SharedPtr debugDataPub_comparison;    //比对用debug消息发送

        //夹爪反馈发布 (Float64MultiArray: [pos_deg, spd_rad, current_A, temperature] × 左右)
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr gripper_fdb_pub;
        void gripper_fdb_publish();

        //夹爪控制参量接受
        rclcpp::Subscription<ctl_msgs_hw::msg::GripperCtl>::SharedPtr gripper_des_get;
        void gripper_desMsg_get(const ctl_msgs_hw::msg::GripperCtl::SharedPtr msg);

        //添加标志位，标记是否接收到指令
        bool has_received_command_;     //当该标志位为false -- 非使能状态； 当标志位为true -- 使能状态
        bool ifFirstTemp;
        bool trajectory_sample_ = false;
        HomingHandoff homing_handoff_;
        bool startup_homing_ = true;

        // Arm zero calibration is an explicit, isolated mode.  It must never
        // fall through to the normal homing or upper-controller path.
        bool arm_zero_calibration_ = false;
        bool left_arm_zero_calibration_ = false;
        bool right_arm_zero_calibration_ = false;
        bool both_arm_zero_calibration_ = false;
        bool can_channel_scan_ = false;
        bool feedback_only_ = false;
        std::string motor_zero_joint_;
        std::vector<double> left_arm_calibration_hold_positions_;
        std::vector<double> right_arm_calibration_hold_positions_;
        void apply_arm_zero_calibration_commands(
            const std::vector<double>& left_hold_positions,
            const std::vector<double>& right_hold_positions
        );

        //----------ROS2回调执行器----------
        //显式 executor（替代每次调用都会临时创建 executor 的全局 rclcpp::spin_some）
        //注意：EtherCAT 主循环与回调仍在同一线程执行，避免共享数据竞态
        rclcpp::Executor::SharedPtr executor_;

        //调试模式标志 -- 无EtherCAT从站时回传下发指令
        bool ethercat_debug_mode_ = false;

        //----------控制调试信息发送----------
        std::unique_ptr<performanceTestSeal> performance_test;

        //==================================================
        //消息传递辅助变量 -- CAN板数量，以及各个CAN板上关节数量
        CAN_JNT_numStruct servo_CAN_Jnt_arrange;

        //消息传递变量
        std::vector<std::vector<joint_desMsg>> servo_desMsg;
        std::vector<std::vector<joint_retMsg>> servo_retMsg;

        int vector_len = 0;
        //解包前目标变量
        std::vector<joint_desMsg> servo_desMsg_orig;
        //解包后反馈变量
        std::vector<joint_retMsg> servo_retMsg_unpacked;
        //电机力控参数
        std::vector<double> can_jnt_Kt_unpacked;

        //----------插补器变量----------
        //滤波器初始化触发器 -- 在初次接收到中层节点后要进行滤波器初始化

        //左右臂插补位置初始化刷新量
        std::vector<double> l_posReceive_atRuninSt;
        std::vector<double> r_posReceive_atRuninSt;

        //----------模式控制变量----------
        double tempTime_upperMonitor;       //上位机迭代时间
        int runin_mod;                      //插补模式
        int if_runin_renew;                 //是否刷新插补
        int msg_get_ifUseDebugMod;          //是否使用调试模式
        int if_useret_to_start_runin;       //判断量 -- 是否使用返回值作为第一轮插补的输入值

        //----------过程控制变量----------
        std::vector<double> servo_desVel_last;       //上一轮的目标速度

        //控制消息获取辅助变量
        std::vector<int> lArm_transMod;

        std::vector<double> lArm_pos_receive;
        std::vector<double> lArm_vel_receive;

        std::vector<double> lArm_torComp_grav_receive;
        std::vector<double> lArm_torComp_interia_receive;
        std::vector<double> lArm_torComp_feedforward_receive;

        std::vector<double> lArm_Kp_receive;
        std::vector<double> lArm_Kd_receive;

        std::vector<int> rArm_transMod;

        std::vector<double> rArm_pos_receive;
        std::vector<double> rArm_vel_receive;

        std::vector<double> rArm_torComp_grav_receive;
        std::vector<double> rArm_torComp_interia_receive;
        std::vector<double> rArm_torComp_feedforward_receive;

        std::vector<double> rArm_Kp_receive;
        std::vector<double> rArm_Kd_receive;

        //夹爪夹紧力
        //夹紧力获取目标值
        double l_gripper_tor_get;
        double r_gripper_tor_get;
        double l_gripper_pos_get = 0.0;
        double r_gripper_pos_get = 0.0;
        bool l_gripper_pos_received = false;
        bool r_gripper_pos_received = false;

        //每轮输出量
        double l_gripper_tor;
        double r_gripper_tor;

        //插补过程辅助变量
        bool ifFirstRunin = true;

        //回零控制变量
        JointPos_Init retZero_Init;

        //------------------------------

        /*  目标值包装协议
        --------------------
        包装协议 -- 把目标数据直接包装，直接按照原始顺序依次对应
        */
        void desMsg_pack_func();

        /*  反馈值配套解包函数
        -----------------------
        解包协议 -- 直接把反馈数据堆叠即可
        */
        void retMsg_unpack_func();

        //==================================================
        //控制器及其配套过程控制变量

        //-----插补器-----
        runin_control runin_control_core;

        int temp_len = 0;

        //控制量 -- 插补器是否初化过
        bool ifRuninerInited = false;

        //-----计时器-----
        double time_res = 0.0;
        double time_last = 0.0;
        int winlen = 0;     //计时器窗口长度

        //-----夹爪插补控制-----
        double time_res_gripper = 0.0;
        double time_last_gripper = 0.0;
        int winlen_gripper = 1;     //夹爪计时器窗口长度
        bool if_gripper_firstGet = true;

        // 夹爪插补变量
        CubicSpline l_gripper_runiner;
        CubicSpline r_gripper_runiner;
        runin_core gripper_pos_runiner;
        std::vector<double> l_gripper_posTraj;
        std::vector<double> r_gripper_posTraj;
        std::vector<double> l_gripper_torTraj;
        std::vector<double> r_gripper_torTraj;
        int gripper_Tor_trajLen;

        //夹爪消息接受辅助变量
        double gripper_LastTorRec_l;
        double gripper_LastTorRec_r;
        bool ifGetGripperMsg = false;

        //夹爪插补函数
        void gripperTrajRunin();

        //插补结果输出函数 -- 直接把插补结果输入到全局变量中
        int gripper_otputTimes;
        void gripperTorOutp();


        //-----夹爪限幅-----

        //限幅包装
        gripper_RangeLimit l_gripper_limit;
        gripper_RangeLimit r_gripper_limit;

        //-----动力学补偿-----
        
        //刚度补偿
        K_comp pos_k_comp_l;
        K_comp pos_k_comp_r;

        //位置、速度输出滤波器
        runinOutpFtr posOutpFtr;
        runinOutpFtr velOutpFtr;
        runinOutpFtr TorOutpFtr;

        //初始段输出控制代码

        //摩擦补偿
        Stribeck_FricCompWrap stribeckComp_wrap;

        //==================================================
        //上位机变量传输函数组

        //函数1 -- 传输信息回调函数
        void message_callback(const ctl_msgs_hw::msg::CtlInformation::SharedPtr msg);

        //函数2 -- 执行插补函数
        bool ifRunin = false;   //插补启动标志
        void ctlMsg_runin();

        //函数3 -- 数据处理与转换函数
        void outpTraj_process();

        //==================================================
        //debug辅助函数

        //debug函数1 -- 比对容器中直接获取的传输值与实际的传输值
        void comp_desMsg_containerMsg();
};

#endif  // SERVO_CONTROL_HW__ROS_CONTROLLER__ROSLOOP_H_
