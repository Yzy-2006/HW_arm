#ifndef __ctlTransmit__H__
#define __ctlTransmit__H__

#include <cstdio>
#include <string>
#include <vector>
#include <iostream>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "ctl_msgs_hw/msg/bridge_ctl_msg.hpp"
#include "ctl_msgs_hw/msg/mid_node_ctl_msg_send.hpp"

#include "robot_control/msgType.h"
#include "messageType/upper_msgType.h"
#include "messageType/chassis_msgType.h"

#include "debugPublish/debugPublish.h"
#include "controller/jointStreamGuard.h"

/*
        by 霜云喵
==========================
通讯桥接代码 — 从其他节点接受控制参量
==========================
本代码包负责从其他ROS2节点接受机械臂控制参量(BridgeCtlMsg)，
每接受一次就执行一次发送代码(转化为MidNodeCtlMsgSend并发布)。

控制代码类型及包含信息参照demoTraj。
==========================
*/

class CtlTransmit {
public:
    // 初始化函数 — 传入ROS2节点指针以创建发布器和订阅器
    void init(rclcpp::Node* node);

    DotOutpWrap bridgeMsg_getAndPub(const std::vector<double>& left_feedback,
                                  const std::vector<double>& right_feedback);

    // Once a bridge message arrives, robot_control stays on the bridge stream
    // until this process exits, so an idle bridge cannot compete with VR.
    bool active() const { return bridge_active_; }

private:
    DotOutpWrap bridge_data;

    rclcpp::Node* node_ptr_;

    // 桥接消息订阅器 — 从其他节点接收 BridgeCtlMsg
    rclcpp::Subscription<ctl_msgs_hw::msg::BridgeCtlMsg>::SharedPtr bridge_msg_sub_;

    // 桥接消息回调函数 — 接受一次BridgeCtlMsg，并将其转化为DotOutpWrap消息类型的数据保存到bridge_data中
    void bridge_msg_callback(const ctl_msgs_hw::msg::BridgeCtlMsg::SharedPtr msg);

    // 尝试接受一次消息 — 非阻塞地处理当前可用的回调（若队列中有 BridgeCtlMsg 则触发一次 bridge_msg_callback）
    void tryReceiveBridgeMsg();

    // ---------------------------------------------------
    //过程控制变量
    bool ifReceiveSuccess;
    bool bridge_active_;
    int zero_count;
    JointStreamGuard joint_stream_;
    bool joint_stream_mode_ = false;

};

#endif
