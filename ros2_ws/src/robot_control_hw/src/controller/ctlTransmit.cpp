#include "controller/ctlTransmit.h"
#include "robot_control/config.h"

//===============================================================================================

void CtlTransmit::init(rclcpp::Node* node)
{
    node_ptr_ = node;
    if (!node_ptr_) return;

    // 创建桥接消息订阅器 — 从其他节点接收 BridgeCtlMsg
    bridge_msg_sub_ = node_ptr_->create_subscription<ctl_msgs_hw::msg::BridgeCtlMsg>(
        "/dexbotCtl/bridge_ctl_msg",
        10,
        std::bind(&CtlTransmit::bridge_msg_callback, this, std::placeholders::_1)
    );

    // 初始化消息传输包装变量 -- bridge_data

    //关节限位控制模式
    bridge_data.armTrajMsg.JntStabMod_l.ifUseJntLim = 0;
    bridge_data.armTrajMsg.JntStabMod_l.limJntNum = 1;
    bridge_data.armTrajMsg.JntStabMod_l.limJntID = {3};
    bridge_data.armTrajMsg.JntStabMod_l.limJntAng = {0.1};

    bridge_data.armTrajMsg.JntStabMod_r.ifUseJntLim = 0;
    bridge_data.armTrajMsg.JntStabMod_r.limJntNum = 1;
    bridge_data.armTrajMsg.JntStabMod_r.limJntID = {3};
    bridge_data.armTrajMsg.JntStabMod_r.limJntAng = {-0.1};

    //运动控制模式
    bridge_data.armTrajMsg.ctlTransMod = 2;
    bridge_data.armTrajMsg.TorFeedforwardCtl = 0;
    bridge_data.armTrajMsg.ifReachEnd = false;

    //控制量初始化
    bridge_data.armTrajMsg.ctlDesMsg.lArm_rigidity_mod.assign(LARMDOF, 1);
    bridge_data.armTrajMsg.ctlDesMsg.rArm_rigidity_mod.assign(RARMDOF, 1);
    bridge_data.armTrajMsg.ctlDesMsg.lArm_JntKp = lArm_defaultKp_low;
    bridge_data.armTrajMsg.ctlDesMsg.lArm_JntKd = lArm_defaultKd_low;
    bridge_data.armTrajMsg.ctlDesMsg.rArm_JntKp = rArm_defaultKp_low;
    bridge_data.armTrajMsg.ctlDesMsg.rArm_JntKd = rArm_defaultKd_low;

    //单点控制初始化
    bridge_data.armTrajMsg.ctlDesMsg.m_SingleDotCtlMsg.ifUseTemp = 0;   //是否使用循环输出

    double dotTrajTime = 2.0;                                           //轨迹时间2.0s
    double accRate = 1.2;                                               //加速时间占比
    bridge_data.armTrajMsg.ctlDesMsg.m_SingleDotCtlMsg.JntTrajTime_L.assign(LARMDOF, dotTrajTime);
    bridge_data.armTrajMsg.ctlDesMsg.m_SingleDotCtlMsg.JntTrajTime_R.assign(RARMDOF, dotTrajTime);
    bridge_data.armTrajMsg.ctlDesMsg.m_SingleDotCtlMsg.JntAccRate_L.assign(LARMDOF, accRate);
    bridge_data.armTrajMsg.ctlDesMsg.m_SingleDotCtlMsg.JntAccRate_R.assign(RARMDOF, accRate);

    //单点位静止消息控制
    bridge_data.armTrajMsg.ctlDesMsg.m_SingleDotCtlMsg.ifReDefineStabJnt = false;
    runinStabMsgWrap runin_stab_msg;
    runin_stab_msg.ifUseStab = true;
    runin_stab_msg.start_stabTimes = 50;
    runin_stab_msg.des_stabTimes = 50;
    bridge_data.armTrajMsg.ctlDesMsg.m_SingleDotCtlMsg.lArm_stabMsg.assign(LARMDOF, runin_stab_msg);
    bridge_data.armTrajMsg.ctlDesMsg.m_SingleDotCtlMsg.rArm_stabMsg.assign(RARMDOF, runin_stab_msg);

    //控制量赋值
    bridge_data.armTrajMsg.ctlDesMsg.ifInitUseRet = 0;  //是否使用反馈值

    //目标位置初始化为关节空间/笛卡尔空间零位
    std::vector<double> lArm_stJntPos = lArm_zeroPos;
    std::vector<double> rArm_stJntPos = rArm_zeroPos;
    std::vector<double> lArm_stDescPos = l_descZeroPos;
    std::vector<double> rArm_stDescPos = r_descZeroPos;

    if(bridge_data.armTrajMsg.ctlTransMod == 1 || bridge_data.armTrajMsg.ctlTransMod == 3){
        bridge_data.armTrajMsg.ctlDesMsg.lArm_desMsg = lArm_stJntPos;
        bridge_data.armTrajMsg.ctlDesMsg.rArm_desMsg = rArm_stJntPos;
    }else{
        bridge_data.armTrajMsg.ctlDesMsg.lArm_desMsg = lArm_stDescPos;
        bridge_data.armTrajMsg.ctlDesMsg.rArm_desMsg = rArm_stDescPos;
    }
    bridge_data.armTrajMsg.ctlDesMsg.lArm_desTorMsg.assign(LARMDOF, 0.0);
    bridge_data.armTrajMsg.ctlDesMsg.rArm_desTorMsg.assign(RARMDOF, 0.0);
    bridge_data.armTrajMsg.ctlDesMsg.lArm_MDP_desJntVel.assign(LARMDOF, 0.0);
    bridge_data.armTrajMsg.ctlDesMsg.rArm_MDP_desJntVel.assign(RARMDOF, 0.0);

    //关节静止控制量
    std::vector<int> lArm_ifJntStab(LARMDOF, 0);
    std::vector<int> rArm_ifJntStab(RARMDOF, 0);

    //针对底盘控制进行赋值
    bridge_data.baseCtlMsg.chassis_cmd.linear_x = 0.0;
    bridge_data.baseCtlMsg.chassis_cmd.linear_y = 0.0;
    bridge_data.baseCtlMsg.chassis_cmd.angular_z = 0.0;

    bridge_data.baseCtlMsg.lifter_cmd.control_mod = 0;   //默认处于遥操作模式
    bridge_data.baseCtlMsg.lifter_cmd.move_cmd = 0;
    bridge_data.baseCtlMsg.lifter_cmd.target_position = 1;
    bridge_data.baseCtlMsg.lifter_cmd.save_preset = false;

    bridge_data.l_ifJntStab = lArm_ifJntStab;
    bridge_data.r_ifJntStab = rArm_ifJntStab;

    ifReceiveSuccess = false;
    zero_count = 0;

    RCLCPP_INFO(node_ptr_->get_logger(), "CtlTransmit bridge initialized.");
}

//尝试接受一次消息（非阻塞）
void CtlTransmit::tryReceiveBridgeMsg()
{
    //处理当前可用的回调：若订阅队列中有 BridgeCtlMsg，则触发一次 bridge_msg_callback
    rclcpp::spin_some(node_ptr_->get_node_base_interface());
}

//每次调用：先尝试接受一次消息，再下发一次 bridge_data（不论是否收到新消息）
DotOutpWrap CtlTransmit::bridgeMsg_getAndPub()
{
    if (!node_ptr_) return bridge_data;

    //尝试接受一次消息
    tryReceiveBridgeMsg();

    global_debugPub->debug_msg_publish_1(bridge_data.armTrajMsg.ctlDesMsg.lArm_desMsg, bridge_data.armTrajMsg.ctlDesMsg.rArm_desMsg);

    if(!ifReceiveSuccess){
        zero_count += 1;

        if(zero_count > 10){
            //若没有成功接收到消息，速度要刷零
            bridge_data.armTrajMsg.ctlDesMsg.lArm_MDP_desJntVel.clear();
            bridge_data.armTrajMsg.ctlDesMsg.rArm_MDP_desJntVel.clear();

            bridge_data.armTrajMsg.ctlDesMsg.lArm_MDP_desJntVel.assign(LARMDOF, 0.0);
            bridge_data.armTrajMsg.ctlDesMsg.rArm_MDP_desJntVel.assign(RARMDOF, 0.0);
        }
    }else{
        zero_count = 0;
    }

    //不论成功与否，都下发一次 bridge_data
    ifReceiveSuccess = false;
    return bridge_data;
}

//===============================================================================================

// 桥接消息回调 — 每接受一次BridgeCtlMsg就更新一次bridge_data
void CtlTransmit::bridge_msg_callback(const ctl_msgs_hw::msg::BridgeCtlMsg::SharedPtr msg)
{
    if (!msg) return;

    RCLCPP_INFO(node_ptr_->get_logger(),
        "BridgeCtlMsg received: ctl_trans_mod=%d", msg->ctl_trans_mod);

    //将接收到的双臂控制信息装入bridge_data（此处无底盘控制信息）
    ifReceiveSuccess = true;

    //运动控制模式
    bridge_data.armTrajMsg.ctlTransMod = msg->ctl_trans_mod;
    bridge_data.armTrajMsg.TorFeedforwardCtl = msg->torque_feedforward_ctl;
    bridge_data.armTrajMsg.ifReachEnd = msg->if_reach_end;

    //关节限位控制（左臂）
    bridge_data.armTrajMsg.JntStabMod_l.ifUseJntLim = msg->l_arm_if_use_jntlim;
    bridge_data.armTrajMsg.JntStabMod_l.limJntNum = msg->l_arm_limjnt_num;
    bridge_data.armTrajMsg.JntStabMod_l.limJntID = msg->l_arm_limjnt_id;
    bridge_data.armTrajMsg.JntStabMod_l.limJntAng = msg->l_arm_limjnt_ang;

    //关节限位控制（右臂）
    bridge_data.armTrajMsg.JntStabMod_r.ifUseJntLim = msg->r_arm_if_use_jntlim;
    bridge_data.armTrajMsg.JntStabMod_r.limJntNum = msg->r_arm_limjnt_num;
    bridge_data.armTrajMsg.JntStabMod_r.limJntID = msg->r_arm_limjnt_id;
    bridge_data.armTrajMsg.JntStabMod_r.limJntAng = msg->r_arm_limjnt_ang;

    //目标位姿信息
    bridge_data.armTrajMsg.ctlDesMsg.lArm_desMsg = msg->l_arm_des_msg;
    bridge_data.armTrajMsg.ctlDesMsg.rArm_desMsg = msg->r_arm_des_msg;

    //目标前馈力矩
    bridge_data.armTrajMsg.ctlDesMsg.lArm_desTorMsg = msg->l_arm_des_tor_msg;
    bridge_data.armTrajMsg.ctlDesMsg.rArm_desTorMsg = msg->r_arm_des_tor_msg;

    //MDP接口目标速度
    bridge_data.armTrajMsg.ctlDesMsg.lArm_MDP_desJntVel = msg->l_arm_mdp_des_jnt_vel;
    bridge_data.armTrajMsg.ctlDesMsg.rArm_MDP_desJntVel = msg->r_arm_mdp_des_jnt_vel;

    //是否使用返回值初始化
    bridge_data.armTrajMsg.ctlDesMsg.ifInitUseRet = msg->if_init_use_ret;

    //刚度等级
    bridge_data.armTrajMsg.ctlDesMsg.lArm_rigidity_mod = msg->l_arm_rigidity_mod;
    bridge_data.armTrajMsg.ctlDesMsg.rArm_rigidity_mod = msg->r_arm_rigidity_mod;

    //PD参数
    bridge_data.armTrajMsg.ctlDesMsg.lArm_JntKp = msg->l_arm_jnt_kp;
    bridge_data.armTrajMsg.ctlDesMsg.lArm_JntKd = msg->l_arm_jnt_kd;
    bridge_data.armTrajMsg.ctlDesMsg.rArm_JntKp = msg->r_arm_jnt_kp;
    bridge_data.armTrajMsg.ctlDesMsg.rArm_JntKd = msg->r_arm_jnt_kd;

    //关节静止开关
    bridge_data.l_ifJntStab = msg->l_arm_if_jnt_stab;
    bridge_data.r_ifJntStab = msg->r_arm_if_jnt_stab;
    

    //bridge_data 已更新，交由 bridgeMsg_getAndPub 统一下发
}

//===============================================================================================
