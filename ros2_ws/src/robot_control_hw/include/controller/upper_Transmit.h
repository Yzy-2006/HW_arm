#ifndef __upper_Transmit__H__
#define __upper_Transmit__H__

#include <cstdio>
#include <csignal>
#include <cstring>
#include <atomic>
#include <string>
#include <vector>
#include <iostream>
#include <thread>

#include <deque>
#include <algorithm>
#include <numeric>

#include <unistd.h> // for read()
#include <sys/select.h> // for select()

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "robot_control/msgType.h"

#include "messageType/chassis_msgType.h"
#include "messageType/upper_msgType.h"

#include <mutex>
#include <functional>                
#include <future>                  
#include <memory> 
#include "chassis_control_hw/msg/lifter_cmd.hpp"
#include "chassis_control_hw/srv/save_preset.hpp"
#include "chassis_control_hw/action/move_to_preset.hpp"       

#include <geometry_msgs/msg/twist.hpp>

/*
        by 霜云喵
==========================
上位机交互代码 -- 负责总控层与上位机的局域网通讯解析
==========================

==========================
*/


//--------------------------------------------------
//辅助函数

//四元数乘法
std::vector<double> multiplyQuaternions(const std::vector<double>& q1, const std::vector<double>& q2);

// 欧拉角转四元数（ZYX顺序 - 先绕Z轴，再Y轴，最后X轴）
std::vector<double> eulerToQuaternionZYX(const std::vector<double>& euler);

// 四元数转欧拉角（ZYX顺序）
std::vector<double> quaternionToEulerZYX(const std::vector<double>& q);

//--------------------------------------------------
//数据处理核心代码1 -- 机械臂顶层下发轨迹处理 -- 包含针对两种轨迹的处理
class arm_traj_ctl{
    private:
        //发送变量包装
        dataSendWrap mdataSend;   

        //---------------------------------------
        // 默认控制参量参数定义

        //默认刚度参数
        std::vector<int> rigidity_mod_l = { 2, 2, 2, 2, 2, 2, 2};
        std::vector<int> rigidity_mod_r = { 2, 2, 2, 2, 2, 2, 2};

        //默认PD参数传输
        std::vector<double> Kp_l_default;
        std::vector<double> Kd_l_default;

        std::vector<double> Kp_r_default;
        std::vector<double> Kd_r_default;

        //---------------------------------------

    public:
        void init();

        //关节静止情况修改代码
        void JntStabMod(std::vector<int> l_JntStab, std::vector<int> r_JntStab); 

        /*  数据处理与传输代码
        --------------------
        传输协议
        输入内容：
            1. 关节限位控制量 -- JntStabMod
            2. 运动控制量 -- ctlTransMod
                0 -- 归零控制；1 -- 关节模式；2 -- 点位控制； 3 -- 关节轨迹控制； 4 -- 点位轨迹控制； 5 -- 零力拖动模式; 6 -- MDP模式
            3. 力前馈控制量 -- TorFeedforwardCtl：0 -- 无力前馈，1 -- 笛卡尔空间力前馈，2 -- 关节空间力前馈，注意力前馈与控制模式的连动
            4. 目标力位 -- ctlDesMsg
            5. 是否到达该段轨迹终点（该项应当取自反馈值）-- ifReachEnd
        */
        dataSendWrap trans_and_outp(
            JntLimMsg_trans JntStabMod_l, 
            JntLimMsg_trans JntStabMod_r, 
            int ctlTransMod, 
            int TorFeedforwardCtl,
            ctlDesMsg_trans ctlDesMsg, 
            bool ifReachEnd
        );
};

//------------------------------
//数据处理核心代码2 -- 底盘通讯代码

//类封装 -- 升降台控制代码
using MoveToPreset = chassis_control_hw::action::MoveToPreset;

class elevator_transmit {
    public:
        void init(rclcpp::Node* node);
        void publish_lifter_cmd(int8_t move_cmd);
        
        void saveCurrentPresetAsync(std::function<void(bool, const std::string&)> callback);
        
        // 正确声明：不要加类名限定符
        void moveToHeight(int target_height,
                        std::function<void(bool, const std::string&)> on_result = nullptr,
                        std::function<void(int, float)> on_feedback = nullptr);
        
        void cancelCurrentMove();

    private:
        //处理升降动作的回调函数
        void handleSaveResponse(rclcpp::Client<chassis_control_hw::srv::SavePreset>::SharedFuture future);
        
        // Action 回调函数
        void goal_cb(const rclcpp_action::ClientGoalHandle<MoveToPreset>::SharedPtr& handle);
        void feedback_cb(const rclcpp_action::ClientGoalHandle<MoveToPreset>::SharedPtr& handle,
                        const std::shared_ptr<const MoveToPreset::Feedback>& feedback);
        void result_cb(const rclcpp_action::ClientGoalHandle<MoveToPreset>::WrappedResult& result);
        
        // ROS2 接口
        rclcpp::Node* node_ptr_;
        rclcpp::Publisher<chassis_control_hw::msg::LifterCmd>::SharedPtr lifter_cmd_pub_;
        rclcpp::Client<chassis_control_hw::srv::SavePreset>::SharedPtr save_preset_cli_;  // 注意变量名
        rclcpp_action::Client<MoveToPreset>::SharedPtr move_action_client_;  // 注意变量名
        
        // 当前活动的 goal handle
        rclcpp_action::ClientGoalHandle<MoveToPreset>::SharedPtr current_goal_handle_;
        
        // 回调函数存储
        std::function<void(bool, const std::string&)> result_callback_;
        std::function<void(int, float)> feedback_callback_;
};

//类封装 -- 小车控制代码
class chassis_transmit {
    public:
        void init(rclcpp::Node* node);
        void publish_chassis_cmd_vel(double linear_x, double linear_y, double angular_z);

    private:
        rclcpp::Node* node_ptr_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

};

//类封装 -- 底盘整体控制代码
class base_transmit{
    private:
        //升降机通讯封装
        elevator_transmit elevator_;
        //底盘小车通讯封装
        chassis_transmit chassis_;

    public:
        //初始化函数
        void init(rclcpp::Node* node);

        //升降台位置查询接口 -- 待补充
        double get_elevatorPos();

        //整体控制接口封装
        void base_control(base_control_msg ctl_msg);

};

//--------------------------------------------------

#endif