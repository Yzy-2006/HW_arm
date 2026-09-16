#include"debugPublish/debugPublish.h"

debugMsgPub_upper::debugMsgPub_upper(rclcpp::Node* node) : node_ptr_(node){

    //初始化消息发送函数
    debug_msg_send = node_ptr_->create_publisher<ctl_msgs_hw::msg::DbgInformation>(
        "debugPub_upper/debug_publish_1",
        10
    );

    debug_msg_send_2 = node_ptr_->create_publisher<ctl_msgs_hw::msg::DbgInformation>(
        "debugPub_upper/debug_publish_2",
        10
    );

}

void debugMsgPub_upper::debug_msg_publish_1(std::vector<double> l_reserve, std::vector<double> r_reserve){
    auto temp_debug_publish = ctl_msgs_hw::msg::DbgInformation();

    temp_debug_publish.float_reserve.resize(l_reserve.size());
    temp_debug_publish.float_reserve_b.resize(r_reserve.size());
    for(int i = 0; i < l_reserve.size(); i++){
        temp_debug_publish.float_reserve[i] = l_reserve[i];
    }
    for(int i = 0; i < r_reserve.size(); i++){
        temp_debug_publish.float_reserve_b[i] = r_reserve[i];
    }

    debug_msg_send->publish(temp_debug_publish);
}

void debugMsgPub_upper::debug_msg_publish_2(std::vector<double> l_reserve, std::vector<double> r_reserve){
    auto temp_debug_publish = ctl_msgs_hw::msg::DbgInformation();

    temp_debug_publish.float_reserve.resize(l_reserve.size());
    temp_debug_publish.float_reserve_b.resize(r_reserve.size());
    for(int i = 0; i < l_reserve.size(); i++){
        temp_debug_publish.float_reserve[i] = l_reserve[i];
    }
    for(int i = 0; i < r_reserve.size(); i++){
        temp_debug_publish.float_reserve_b[i] = r_reserve[i];
    }

    debug_msg_send_2->publish(temp_debug_publish);
}

//-----------------------------------------------------------------------------------

debugMsgPub_upper* global_debugPub = nullptr;