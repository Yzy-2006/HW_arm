#include "robot_control/msgPublisher.h"

/*
        by 霜云喵
================================
上位机输出轨迹处理代码
============修改日志============

==============协议==============

================================
*/


//轨迹输出
/*  
注意这个轨迹输出函数并不会检测是否下位机所有关节到达目标位置，只要调用一次这个函数，就进行依次计算并输出一次轨迹
是否应当输出一次轨迹，既下位机所有关节是否到达目标位置应当由外部函数检测
*/
void RobotCtlNode::TrajOutp(){
    upperMonitor_dataSend = armTrajTransCtl.trans_and_outp(armTrajMsg.JntStabMod_l, armTrajMsg.JntStabMod_r, armTrajMsg.ctlTransMod, armTrajMsg.TorFeedforwardCtl, armTrajMsg.ctlDesMsg, armTrajMsg.ifReachEnd);
    baseTransCtl.base_control(baseCtlMsg);

    global_debugPub->debug_msg_publish_2(upperMonitor_dataSend.ctlDesMsg.desPos_l_JointSpace, upperMonitor_dataSend.ctlDesMsg.desPos_r_JointSpace);
}

//---------------演示轨迹函数---------------

void RobotCtlNode::demoTrajInit(){
    std::cout << "请选择执行的轨迹" << std::endl;
    std::cout << "  1 -- 点位控制测试轨迹；" << std::endl
              << "  2 -- 精度测试轨迹1；" << std::endl
              << "  3 -- 速度测试轨迹1；" << std::endl
              << "  4 -- 直线演示轨迹；" << std::endl
              << "  5 -- 零空间控制演示轨迹" << std::endl
              << "  6 -- 三点控制演示轨迹" << std::endl
              << "  7 -- 零力拖动演示轨迹" << std::endl
              << "  8 -- 关节空间连续轨迹测试" << std::endl
              << "  9 -- 余弦扫频轨迹测试" << std::endl
              << " 10 -- 正弦扫频MDP模式测试" << std::endl
              << " 11 -- 控制模式切换测试" << std::endl
              ;
              /**/
    int m_Choose;
    std::cin >> m_Choose;
    this->trajChoose = m_Choose;

    std::cout << "请选择底层刚度与控制模式（**注意，请保证选择模式的匹配**)" << std::endl;
    std::cout << "  1 -- 低刚度模式；" << std::endl
            << "  2 -- 中等刚度模式；" << std::endl
            << "  3 -- 高刚度模式；" << std::endl
            << "  7 -- 超高刚度模式" << std::endl;
            /**/
    int control_mod;
    std::cin >> control_mod; 
    if(control_mod > 0 && control_mod < 10){
        demoTrajWrap.m_rigidityCtlMod = control_mod;
    }else{
        std::cout << "输入有误，默认进入保护待机模式" << std::endl;
        demoTrajWrap.m_rigidityCtlMod = 0;
    }

    std::cout << "执行轨迹：" << trajChoose << std::endl;

    if(trajChoose == 1){
        demoTrajWrap.init_v1();
    }else if(trajChoose == 2){
        demoTrajWrap.init_v2();
    }else if(trajChoose == 3){
        demoTrajWrap.init_v3();
    }else if(trajChoose == 4){
        demoTrajWrap.init_v4();
    }else if(trajChoose == 5){
        demoTrajWrap.init_v5();
        demoTrajWrap.initCtl_v5 = true;
    }else if(trajChoose == 6){
        demoTrajWrap.init_v6();
    }else if(trajChoose == 7){
        demoTrajWrap.init_v7();
    }else if(trajChoose == 8){
        demoTrajWrap.init_v8();
    }else if(trajChoose == 9){
        demoTrajWrap.init_v9();
    }else if(trajChoose == 10){
        demoTrajWrap.init_v10();
    }else if(trajChoose == 11){
        demoTrajWrap.init_v11();
    }
    else{
        std::cout << "输入有误，默认执行轨迹1" << std::endl;
        demoTrajWrap.init_v1();
    }

}

void RobotCtlNode::demoTrajOutp(){
    DotOutpWrap temp_dotOutp;

    if(trajChoose == 1){
        temp_dotOutp = demoTrajWrap.TrajOutp_v1();
    }else if(trajChoose == 2){
        temp_dotOutp = demoTrajWrap.TrajOutp_v2();
    }else if(trajChoose == 3){
        temp_dotOutp = demoTrajWrap.TrajOutp_v3();
    }else if(trajChoose == 4){
        temp_dotOutp = demoTrajWrap.TrajOutp_v4();
    }else if(trajChoose == 5){
        temp_dotOutp = demoTrajWrap.TrajOutp_v5();
    }else if(trajChoose == 6){
        temp_dotOutp = demoTrajWrap.TrajOutp_v6();
    }else if(trajChoose == 7){
        temp_dotOutp = demoTrajWrap.TrajOutp_v7();
    }else if(trajChoose == 8){
        temp_dotOutp = demoTrajWrap.TrajOutp_v8();
    }else if(trajChoose == 9){
        temp_dotOutp = demoTrajWrap.TrajOutp_v9();
    }else if(trajChoose == 10){
        temp_dotOutp = demoTrajWrap.TrajOutp_v10();
    }else if(trajChoose == 11){
        temp_dotOutp = demoTrajWrap.TrajOutp_v11();
    }
    else{
        temp_dotOutp = demoTrajWrap.TrajOutp_v1();
    }
    //数据传递函数赋值
    this->armTrajMsg = temp_dotOutp.armTrajMsg;
    this->baseCtlMsg = temp_dotOutp.baseCtlMsg;

    //调用限位刷新函数
    armTrajTransCtl.JntStabMod(
        temp_dotOutp.l_ifJntStab, temp_dotOutp.r_ifJntStab
    );

}

//----------------------------------------


//-----------------------------------------------------------
//遥操作处理代码

//遥操作信息收发初始化
void RobotCtlNode::xr_teleComu_init(){
    // VR publishes per-frame increments.  Keep only the newest sample so a
    // delayed upper-loop cycle cannot replay a backlog as one large motion.
    const auto vr_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
    lArm_endPos = this->create_subscription<geometry_msgs::msg::Vector3>(
        "/vr/left_arm_euler_delta",
        vr_qos,
        std::bind(&RobotCtlNode::lArm_xrTele_endPos_callback, this, std::placeholders::_1)
    );

    rArm_endPos = this->create_subscription<geometry_msgs::msg::Vector3>(
        "/vr/right_arm_euler_delta",
        vr_qos,
        std::bind(&RobotCtlNode::rArm_xrTele_endPos_callback, this, std::placeholders::_1)
    );

    lArm_end_delta = this->create_subscription<geometry_msgs::msg::Vector3>(
        "/vr/left_arm_delta",
        vr_qos,
        std::bind(&RobotCtlNode::lArm_xrTele_endDelta_callback, this, std::placeholders::_1)
    );
    rArm_end_delta = this->create_subscription<geometry_msgs::msg::Vector3>(
        "/vr/right_arm_delta",
        vr_qos,
        std::bind(&RobotCtlNode::rArm_xrTele_endDelta_callback, this, std::placeholders::_1)
    );

    zeroReturnCmd_sub = this->create_subscription<std_msgs::msg::Bool>(
        "/vr/zero_return_cmd",
        10,
        std::bind(&RobotCtlNode::zeroReturnCmd_callback, this, std::placeholders::_1)
    );

    endDescPub_sub = this->create_subscription<ctl_msgs_hw::msg::EndDescPub>(
        "/feedback_get/end_desc_traj_pub",
        10,
        std::bind(&RobotCtlNode::endDescPub_callback, this, std::placeholders::_1)
    );

    xrTeleComuCtl.init();
    safety_subscription_ = create_subscription<std_msgs::msg::String>("/towel/safety/state",
        rclcpp::QoS(1).transient_local(), [this](std_msgs::msg::String::ConstSharedPtr msg) {
            safety_received_ = std::chrono::steady_clock::now();
            safety_ready_ = msg->data == "ready";
            safety_fault_ = safety_fault_ || msg->data.rfind("fault:", 0) == 0;
        });
    vr_override_service = this->create_service<std_srvs::srv::SetBool>(
        "/dexbotCtl/vr_override",
        [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
               std::shared_ptr<std_srvs::srv::SetBool::Response> response){
            if (!motion_ready()) {
                response->success = false;
                response->message = "Handoff blocked by servo safety state; reset fault and restart controller";
                return;
            }
            const double age = std::chrono::duration<double>(
                std::chrono::steady_clock::now() - last_desc_feedback).count();
            if(request->data && (age > 0.25 || !xrTeleComuCtl.reanchor_from_feedback())){
                response->success = false;
                response->message = "VR handoff requires fresh finite FK feedback within VR limits";
                return;
            }
            bridge_transmit.set_vr_override(request->data);
            response->success = true;
            response->message = request->data ? "VR anchored to feedback" : "Bridge holding; awaiting fresh policy";
        });
}

//遥操作数据转化并输出
void RobotCtlNode::xr_teleComu_outp(){
    upperMonitor_dataSend = xrTeleComuCtl.trans_and_outp();
}

//四个遥操作消息回调函数
void RobotCtlNode::lArm_xrTele_endPos_callback(const geometry_msgs::msg::Vector3::SharedPtr msg){
    if(!motion_ready() || bridge_transmit.active()){ return; }

    xrTeleComuCtl.apply_left_orientation_delta({
        msg->x, msg->y, msg->z
    });
 
}
void RobotCtlNode::rArm_xrTele_endPos_callback(const geometry_msgs::msg::Vector3::SharedPtr msg){
    if(!motion_ready() || bridge_transmit.active()){ return; }

    xrTeleComuCtl.apply_right_orientation_delta({
        msg->x, msg->y, msg->z
    });

}
void RobotCtlNode::lArm_xrTele_endDelta_callback(const geometry_msgs::msg::Vector3::SharedPtr msg){
    if(!motion_ready() || bridge_transmit.active() || !xrTeleComuCtl.accepts_vr_input()){ return; }

    xrTeleComuCtl.l_arm_end_delta[0] += msg->x;
    xrTeleComuCtl.l_arm_end_delta[1] += msg->y;
    xrTeleComuCtl.l_arm_end_delta[2] += msg->z;
    
}
void RobotCtlNode::rArm_xrTele_endDelta_callback(const geometry_msgs::msg::Vector3::SharedPtr msg){
    if(!motion_ready() || bridge_transmit.active() || !xrTeleComuCtl.accepts_vr_input()){ return; }


    xrTeleComuCtl.r_arm_end_delta[0] += msg->x;
    xrTeleComuCtl.r_arm_end_delta[1] += msg->y;
    xrTeleComuCtl.r_arm_end_delta[2] += msg->z;
    
}

//A 键回零触发回调：收到 True 后置位回零请求标志
void RobotCtlNode::zeroReturnCmd_callback(const std_msgs::msg::Bool::SharedPtr msg){
    if(!motion_ready() || bridge_transmit.active()){ return; }
    if(msg->data){
        std::cout << " xr message get: A 键回零触发指令" << std::endl;
        xrTeleComuCtl.trigger_joint_return();
    }
}

//末端笛卡尔位姿反馈回调：缓存当前末端位姿（用于回零完成后重新锚定）
void RobotCtlNode::endDescPub_callback(const ctl_msgs_hw::msg::EndDescPub::SharedPtr msg){
    if(msg->l_desc_pos_ret.size() != 6 || msg->r_desc_pos_ret.size() != 6) return;
    std::vector<double> l_desc_ret(6, 0.0), r_desc_ret(6, 0.0);
    for(int i = 0; i < 6; i++){
        l_desc_ret[i] = msg->l_desc_pos_ret[i];
        r_desc_ret[i] = msg->r_desc_pos_ret[i];
    }
    xrTeleComuCtl.update_desc_ret(l_desc_ret, r_desc_ret);
    last_desc_feedback = std::chrono::steady_clock::now();
}

//-----------------------------------------------------------

//桥接输出代码
void RobotCtlNode::bridge_transmitOutp(){
    DotOutpWrap temp_dotOutp;
    temp_dotOutp = bridge_transmit.bridgeMsg_getAndPub(lArm_pos_fdback, rArm_pos_fdback);

    //数据传递函数赋值
    this->armTrajMsg = temp_dotOutp.armTrajMsg;
    this->baseCtlMsg = temp_dotOutp.baseCtlMsg;

    //调用限位刷新函数
    armTrajTransCtl.JntStabMod(
        temp_dotOutp.l_ifJntStab, temp_dotOutp.r_ifJntStab
    );
}

//-----------------------------------------------------------
