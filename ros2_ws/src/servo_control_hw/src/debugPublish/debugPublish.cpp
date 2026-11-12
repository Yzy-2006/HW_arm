#include"debugPublish/debugPublish.h"

debugMsgPub::debugMsgPub(rclcpp::Node* node, ServoRosPublisher* async) : node_ptr_(node), async_(async){

    //初始化消息发送函数
    debug_msg_send = node_ptr_->create_publisher<ctl_msgs_hw::msg::DbgInformation>(
        "debugPub_servo/debug_publish_1",
        10
    );

    debug_msg_send_2 = node_ptr_->create_publisher<ctl_msgs_hw::msg::DbgInformation>(
        "debugPub_servo/debug_publish_2",
        10
    );

    debug_msg_send_3 = node_ptr_->create_publisher<ctl_msgs_hw::msg::DbgInformation>(
        "debugPub_servo/debug_publish_3",
        10
    );

}

void debugMsgPub::debug_msg_publish_1(std::vector<double> l_reserve, std::vector<double> r_reserve){
    auto temp_debug_publish = ctl_msgs_hw::msg::DbgInformation();

    temp_debug_publish.float_reserve.resize(l_reserve.size());
    temp_debug_publish.float_reserve_b.resize(r_reserve.size());
    for(int i = 0; i < l_reserve.size(); i++){
        temp_debug_publish.float_reserve[i] = l_reserve[i];
    }
    for(int i = 0; i < r_reserve.size(); i++){
        temp_debug_publish.float_reserve_b[i] = r_reserve[i];
    }

    if (async_) async_->publish(ServoPublishChannel::debug1, debug_msg_send, std::move(temp_debug_publish));
    else debug_msg_send->publish(temp_debug_publish);
}

void debugMsgPub::debug_msg_publish_2(std::vector<double> l_reserve, std::vector<double> r_reserve){
    auto temp_debug_publish = ctl_msgs_hw::msg::DbgInformation();

    temp_debug_publish.float_reserve.resize(l_reserve.size());
    temp_debug_publish.float_reserve_b.resize(r_reserve.size());
    for(int i = 0; i < l_reserve.size(); i++){
        temp_debug_publish.float_reserve[i] = l_reserve[i];
    }
    for(int i = 0; i < r_reserve.size(); i++){
        temp_debug_publish.float_reserve_b[i] = r_reserve[i];
    }

    if (async_) async_->publish(ServoPublishChannel::debug2, debug_msg_send_2, std::move(temp_debug_publish));
    else debug_msg_send_2->publish(temp_debug_publish);
}

void debugMsgPub::debug_msg_publish_3(std::vector<double> l_reserve, std::vector<double> r_reserve){
    auto temp_debug_publish = ctl_msgs_hw::msg::DbgInformation();

    temp_debug_publish.float_reserve.resize(l_reserve.size());
    temp_debug_publish.float_reserve_b.resize(r_reserve.size());
    for(int i = 0; i < l_reserve.size(); i++){
        temp_debug_publish.float_reserve[i] = l_reserve[i];
    }
    for(int i = 0; i < r_reserve.size(); i++){
        temp_debug_publish.float_reserve_b[i] = r_reserve[i];
    }

    if (async_) async_->publish(ServoPublishChannel::debug3, debug_msg_send_3, std::move(temp_debug_publish));
    else debug_msg_send_3->publish(temp_debug_publish);
}

//-----------------------------------------------------------------------------------

debugMsgPub* global_debugPub = nullptr;
