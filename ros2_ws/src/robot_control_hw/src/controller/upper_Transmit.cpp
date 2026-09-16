#include "controller/upper_Transmit.h"
#include "robot_control/config.h"

//--------------------------------------------------
//输入数据处理代码：


//----------------------------------------------------------------------------------
//辅助函数新添加
// 四元数乘法（保持不变）
std::vector<double> multiplyQuaternions(const std::vector<double>& q1, const std::vector<double>& q2) {
    std::vector<double> result(4);
    result[0] = q1[0]*q2[0] - q1[1]*q2[1] - q1[2]*q2[2] - q1[3]*q2[3]; // w
    result[1] = q1[0]*q2[1] + q1[1]*q2[0] + q1[2]*q2[3] - q1[3]*q2[2]; // x
    result[2] = q1[0]*q2[2] - q1[1]*q2[3] + q1[2]*q2[0] + q1[3]*q2[1]; // y
    result[3] = q1[0]*q2[3] + q1[1]*q2[2] - q1[2]*q2[1] + q1[3]*q2[0]; // z
    return result;
}

// 欧拉角转四元数（ZYX顺序 - 先绕Z轴，再Y轴，最后X轴）
std::vector<double> eulerToQuaternionZYX(const std::vector<double>& euler) {
    double yaw = euler[0];   // Z轴旋转
    double pitch = euler[1]; // Y轴旋转  
    double roll = euler[2];  // X轴旋转
    
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);
    
    std::vector<double> q(4);
    q[0] = cr * cp * cy + sr * sp * sy; // w
    q[1] = sr * cp * cy - cr * sp * sy; // x
    q[2] = cr * sp * cy + sr * cp * sy; // y
    q[3] = cr * cp * sy - sr * sp * cy; // z
    
    return q;
}

// 四元数转欧拉角（ZYX顺序）
std::vector<double> quaternionToEulerZYX(const std::vector<double>& q) {
    double w = q[0], x = q[1], y = q[2], z = q[3];
    
    std::vector<double> euler(3);
    
    // 绕Z轴旋转（yaw）
    double siny_cosp = 2.0 * (w * z + x * y);
    double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    euler[0] = atan2(siny_cosp, cosy_cosp);
    
    // 绕Y轴旋转（pitch）
    double sinp = 2.0 * (w * y - z * x);
    if (fabs(sinp) >= 1.0) {
        euler[1] = copysign(M_PI / 2.0, sinp); // 使用90度如果超出范围
    } else {
        euler[1] = asin(sinp);
    }
    
    // 绕X轴旋转（roll）
    double sinr_cosp = 2.0 * (w * x + y * z);
    double cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
    euler[2] = atan2(sinr_cosp, cosr_cosp);
    
    return euler;
}
//----------------------------------------------------------------------------------


//----------------------------------------------------------------------------------
//数据处理核心代码1 -- 机械臂顶层下发轨迹处理 -- 包含针对两种轨迹的处理

void arm_traj_ctl::init(){

    //初始化默认参数
    Kp_l_default = lArm_defaultKp_low;
    Kd_l_default = lArm_defaultKd_low;
    Kp_r_default = rArm_defaultKp_low;
    Kd_r_default = rArm_defaultKd_low;

    //--------------------初始化输出包装变量--------------------
    //初始化角度输出模式是否为静止的
    std::vector<int> lArm_ifJntStab, rArm_ifJntStab;
    lArm_ifJntStab = { 0, 0, 0, 0, 0, 0, 0 };
    rArm_ifJntStab = { 0, 0, 0, 0, 0, 0, 0 };

    //初始信息包装
    //关节空间：
    std::vector<double> lArm_stJntPos, rArm_stJntPos;
    lArm_stJntPos = lArm_zeroPos;
    rArm_stJntPos = rArm_zeroPos;
    //笛卡尔空间：
    std::vector<double> lArm_stDescPos, rArm_stDescPos;
    lArm_stDescPos = l_descZeroPos;
    rArm_stDescPos = r_descZeroPos;

    //初始化目标位置
    ctlDesMsgWrap ctlStMsg;
    ctlStMsg.desPos_l_Descartes = lArm_stDescPos;
    ctlStMsg.desPos_r_Descartes = rArm_stDescPos;
    ctlStMsg.desPos_l_JointSpace = lArm_stJntPos;
    ctlStMsg.desPos_r_JointSpace = rArm_stJntPos;

    ctlStMsg.desTor_l_Descartes.assign(6, 0.0);
    ctlStMsg.desTor_r_Descartes.assign(6, 0.0);
    ctlStMsg.desTor_l_JointSpace.assign(LARMDOF, 0.0);
    ctlStMsg.desTor_r_JointSpace.assign(RARMDOF, 0.0);

    ctlStMsg.desVel_l_MDPjointSpace.assign(LARMDOF, 0.0);
    ctlStMsg.desVel_r_MDPjointSpace.assign(RARMDOF, 0.0);

    //控制值初始化
    //-----数据传输与插补模式-----
    mdataSend.ctl_transMod = 4;
    mdataSend.runinMod = 1;
    mdataSend.TorCompMod = 0;

    //-----求解模式信息-----
    mdataSend.ifUseRet = 0;
    mdataSend.ifUseTemp = 0;

    mdataSend.rigidityCtl_mod_l.resize(LARMDOF);
    mdataSend.rigidityCtl_mod_r.resize(RARMDOF);
    mdataSend.KpTrans_l.resize(LARMDOF);
    mdataSend.KdTrans_l.resize(LARMDOF);
    mdataSend.KpTrans_r.resize(RARMDOF);
    mdataSend.KdTrans_r.resize(RARMDOF);

    for(int i = 0; i < LARMDOF; i++){
        mdataSend.rigidityCtl_mod_l[i] = this->rigidity_mod_l[i];
        mdataSend.rigidityCtl_mod_r[i] = this->rigidity_mod_r[i];
    }

    std::cout << "kp_l_default_len = " << Kp_l_default.size() << std::endl;
    for(int i = 0; i < LARMDOF; i++){
        mdataSend.KpTrans_l[i] = Kp_l_default[i];
        mdataSend.KdTrans_l[i] = Kd_l_default[i];

        mdataSend.KpTrans_r[i] = Kp_r_default[i];
        mdataSend.KdTrans_r[i] = Kd_r_default[i];
    }

    //-----插补信息包装-----
    //发送变量初始化
    double trajTime = 2.0;
    double accRate = 1.2;
    runinStabMsgWrap stabMsg;
    stabMsg.ifUseStab = true;
    stabMsg.start_stabTimes = 50;
    stabMsg.des_stabTimes = 50;

    mdataSend.TrajTime.resize(2);
    mdataSend.TrajTime[0].assign(LARMDOF, trajTime);
    mdataSend.TrajTime[1].assign(RARMDOF, trajTime);

    mdataSend.accRateBase.resize(2);
    mdataSend.accRateBase[0].assign(LARMDOF, accRate);
    mdataSend.accRateBase[1].assign(RARMDOF, accRate);

    mdataSend.m_stabMsg.resize(2);
    mdataSend.m_stabMsg[0].assign(LARMDOF, stabMsg);
    mdataSend.m_stabMsg[1].assign(RARMDOF, stabMsg);

    //-----开关量包装-----
    mdataSend.ifJntStab.resize(2);
    mdataSend.ifJntStab[0] = lArm_ifJntStab;
    mdataSend.ifJntStab[1] = rArm_ifJntStab;

    mdataSend.ifRedefineJntTrajStab = false;

    //-----关节限位信息-----
    mdataSend.lArm_JntLim.ifUseJntLim = false;
    mdataSend.lArm_JntLim.limJntNum = 1;
    mdataSend.lArm_JntLim.limJntLabel.resize(mdataSend.lArm_JntLim.limJntNum);
    mdataSend.lArm_JntLim.limJntAng.resize(mdataSend.lArm_JntLim.limJntNum);
    mdataSend.lArm_JntLim.limJntLabel[0] = "l_joint_3";
    mdataSend.lArm_JntLim.limJntAng[0] = 0.2;

    mdataSend.rArm_JntLim.ifUseJntLim = false;
    mdataSend.rArm_JntLim.limJntNum = 1;
    mdataSend.rArm_JntLim.limJntAng.resize(mdataSend.rArm_JntLim.limJntNum);
    mdataSend.rArm_JntLim.limJntLabel.resize(mdataSend.rArm_JntLim.limJntNum);
    mdataSend.rArm_JntLim.limJntLabel[0] = "r_joint_3";
    mdataSend.rArm_JntLim.limJntAng[0] = 0.2;

    mdataSend.ifUpdateLim = false;

    //-----关节初始化信息-----

    mdataSend.ctlDesMsg = ctlStMsg;
    //-------------------------------------------------------
}

void arm_traj_ctl::JntStabMod(std::vector<int> l_JntStab, std::vector<int> r_JntStab){
    for(int i = 0; i < LARMDOF; i++){
        mdataSend.ifJntStab[0][i] = l_JntStab[i];
        mdataSend.ifJntStab[1][i] = r_JntStab[i];
    }
}

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
dataSendWrap arm_traj_ctl::trans_and_outp(
    JntLimMsg_trans JntStabMod_l, 
    JntLimMsg_trans JntStabMod_r, 

    int ctlTransMod, 
    int TorFeedforwardCtl,

    ctlDesMsg_trans ctlDesMsg, 
    bool ifReachEnd
){
    
    //--------------------
    // 控制量赋值
    mdataSend.ctl_transMod = ctlTransMod;
    int runinMod;
    if(ctlTransMod == 3 || ctlTransMod == 4){
        runinMod = 1;
    }else{
        runinMod = 3;
    }
    mdataSend.runinMod = runinMod;
    mdataSend.TorCompMod = TorFeedforwardCtl;

    //关节限位控制量
    JntLimMsgWrap temp_JntLimMsg_l = mdataSend.lArm_JntLim;
    JntLimMsgWrap temp_JntLimMsg_r = mdataSend.rArm_JntLim;

    temp_JntLimMsg_l.ifUseJntLim = JntStabMod_l.ifUseJntLim;
    temp_JntLimMsg_l.limJntNum = JntStabMod_l.limJntNum;
    temp_JntLimMsg_l.limJntAng = JntStabMod_l.limJntAng;
    temp_JntLimMsg_l.limJntLabel = JntID_to_label(JntStabMod_l.limJntID);

    temp_JntLimMsg_r.ifUseJntLim = JntStabMod_r.ifUseJntLim;
    temp_JntLimMsg_r.limJntNum = JntStabMod_r.limJntNum;
    temp_JntLimMsg_r.limJntAng = JntStabMod_r.limJntAng;
    temp_JntLimMsg_r.limJntLabel = JntID_to_label(JntStabMod_r.limJntID);

    //是否采用反馈值作为初始化
    mdataSend.ifUseRet = ctlDesMsg.ifInitUseRet;

    //单点控制指令刷新
    mdataSend.ifUseTemp = ctlDesMsg.m_SingleDotCtlMsg.ifUseTemp;
    mdataSend.TrajTime.resize(2);
    mdataSend.TrajTime[0] = ctlDesMsg.m_SingleDotCtlMsg.JntTrajTime_L;
    mdataSend.TrajTime[1] = ctlDesMsg.m_SingleDotCtlMsg.JntTrajTime_R;
    mdataSend.accRateBase.resize(2);
    mdataSend.accRateBase[0] = ctlDesMsg.m_SingleDotCtlMsg.JntAccRate_L;
    mdataSend.accRateBase[1] = ctlDesMsg.m_SingleDotCtlMsg.JntAccRate_R;

    mdataSend.ifRedefineJntTrajStab = ctlDesMsg.m_SingleDotCtlMsg.ifReDefineStabJnt;
    mdataSend.m_stabMsg.resize(2);
    mdataSend.m_stabMsg[0] = ctlDesMsg.m_SingleDotCtlMsg.lArm_stabMsg;
    mdataSend.m_stabMsg[1] = ctlDesMsg.m_SingleDotCtlMsg.rArm_stabMsg;
    
    //控制模式更新
    mdataSend.lArm_JntLim = temp_JntLimMsg_l;
    mdataSend.rArm_JntLim = temp_JntLimMsg_r;
    mdataSend.ifUpdateLim = JntStabMod_l.ifUseJntLim || JntStabMod_r.ifUseJntLim;
    //** 力控模式刷新尚未开放 **//

    //--------------------
    // 输入数据长度校验与补齐
    // 根据控制模式确定各消息向量的期望长度，不足时用零补齐并报错
    {
        auto pad_and_check = [](std::vector<double>& vec, size_t expected,
                                const char* name, int side) {
            if (vec.size() < expected) {
                std::cerr << "[ERROR] trans_and_outp(): " << name
                          << " (side=" << side << ") size=" << vec.size()
                          << " < expected=" << expected
                          << ", padding with zeros" << std::endl;
                vec.resize(expected, 0.0);
            }
        };

        // desMsg: Cartesian 模式期望 6 维，Joint/MDP 模式期望 LARMDOF/RARMDOF
        if (ctlTransMod == 2 || ctlTransMod == 4) {
            pad_and_check(ctlDesMsg.lArm_desMsg, 6, "lArm_desMsg", 0);
            pad_and_check(ctlDesMsg.rArm_desMsg, 6, "rArm_desMsg", 1);
        } else if (ctlTransMod == 1 || ctlTransMod == 3 || ctlTransMod == 6) {
            pad_and_check(ctlDesMsg.lArm_desMsg, LARMDOF, "lArm_desMsg", 0);
            pad_and_check(ctlDesMsg.rArm_desMsg, RARMDOF, "rArm_desMsg", 1);
        }

        // desTorMsg: 力前馈模式1期望6维(Cartesian), 模式2期望LARMDOF(Joint),
        // MDP模式也期望LARMDOF
        if (TorFeedforwardCtl == 1) {
            pad_and_check(ctlDesMsg.lArm_desTorMsg, 6, "lArm_desTorMsg", 0);
            pad_and_check(ctlDesMsg.rArm_desTorMsg, 6, "rArm_desTorMsg", 1);
        } else if (TorFeedforwardCtl == 2 || ctlTransMod == 6) {
            pad_and_check(ctlDesMsg.lArm_desTorMsg, LARMDOF, "lArm_desTorMsg", 0);
            pad_and_check(ctlDesMsg.rArm_desTorMsg, RARMDOF, "rArm_desTorMsg", 1);
        }

        // MDP_desJntVel: 仅在 MDP 模式下使用
        if (ctlTransMod == 6) {
            pad_and_check(ctlDesMsg.lArm_MDP_desJntVel, LARMDOF,
                          "lArm_MDP_desJntVel", 0);
            pad_and_check(ctlDesMsg.rArm_MDP_desJntVel, RARMDOF,
                          "rArm_MDP_desJntVel", 1);
        }
    }

    //针对目标量进行处理
    ctlDesMsgWrap ctlDesMsg_processed;

    ctlDesMsg_processed.desPos_l_Descartes.assign(6, 0.0);
    ctlDesMsg_processed.desPos_r_Descartes.assign(6, 0.0);
    ctlDesMsg_processed.desPos_l_JointSpace.assign(LARMDOF, 0.0);
    ctlDesMsg_processed.desPos_r_JointSpace.assign(RARMDOF, 0.0);

    ctlDesMsg_processed.desTor_l_Descartes.assign(6, 0.0);
    ctlDesMsg_processed.desTor_r_Descartes.assign(6, 0.0);
    ctlDesMsg_processed.desTor_l_JointSpace.assign(LARMDOF, 0.0);
    ctlDesMsg_processed.desTor_r_JointSpace.assign(RARMDOF, 0.0);

    ctlDesMsg_processed.desVel_l_MDPjointSpace.assign(LARMDOF, 0.0);
    ctlDesMsg_processed.desVel_r_MDPjointSpace.assign(RARMDOF, 0.0);

    if(ctlTransMod == 2 || ctlTransMod == 4){
        //传输笛卡尔空间轨迹
        //输入量姿态采用欧拉角描述

        for(int i = 0; i < 6; i++){
            ctlDesMsg_processed.desPos_l_Descartes[i] = ctlDesMsg.lArm_desMsg[i];
            ctlDesMsg_processed.desPos_r_Descartes[i] = ctlDesMsg.rArm_desMsg[i];
        }

        //针对力前馈进行处理
        if(TorFeedforwardCtl == 1){
            //笛卡尔空间力前馈
            for(int i = 0; i < 6; i++){
                ctlDesMsg_processed.desTor_l_Descartes[i] = ctlDesMsg.lArm_desTorMsg[i];
                ctlDesMsg_processed.desTor_r_Descartes[i] = ctlDesMsg.rArm_desTorMsg[i];
            }
        }else if(TorFeedforwardCtl == 2){
            for(int i = 0; i < LARMDOF; i++){
                ctlDesMsg_processed.desTor_l_JointSpace[i] = ctlDesMsg.lArm_desTorMsg[i];
            }
            for(int i = 0; i < RARMDOF; i++){
                ctlDesMsg_processed.desTor_r_JointSpace[i] = ctlDesMsg.rArm_desTorMsg[i];
            }
        }else{
            //力前馈为0
        }
        
    }else if(ctlTransMod == 1 || ctlTransMod == 3){
        int ctlMod = 0;
        if(ctlTransMod == 1){
            ctlMod = 0;
        }else{
            ctlMod = 1;
        }

        ctlDesMsg_processed.desPos_l_JointSpace = ctlDesMsg.lArm_desMsg;
        ctlDesMsg_processed.desPos_r_JointSpace = ctlDesMsg.rArm_desMsg;

        //针对力前馈进行处理
        if(TorFeedforwardCtl == 1){
            //笛卡尔空间力前馈
            for(int i = 0; i < 6; i++){
                ctlDesMsg_processed.desTor_l_Descartes[i] = ctlDesMsg.lArm_desTorMsg[i];
                ctlDesMsg_processed.desTor_r_Descartes[i] = ctlDesMsg.rArm_desTorMsg[i];
            }
        }else if(TorFeedforwardCtl == 2){
            for(int i = 0; i < LARMDOF; i++){
                ctlDesMsg_processed.desTor_l_JointSpace[i] = ctlDesMsg.lArm_desTorMsg[i];
            }
            for(int i = 0; i < RARMDOF; i++){
                ctlDesMsg_processed.desTor_r_JointSpace[i] = ctlDesMsg.rArm_desTorMsg[i];
            }
        }else{
            //力前馈为0
        }

    }else if(ctlTransMod == 6){
        //搭配MDP模式，此时直接传输关节空间变量
        for(int i = 0; i < LARMDOF; i++){
            ctlDesMsg_processed.desPos_l_JointSpace[i] = ctlDesMsg.lArm_desMsg[i];
            ctlDesMsg_processed.desVel_l_MDPjointSpace[i] = ctlDesMsg.lArm_MDP_desJntVel[i];
            ctlDesMsg_processed.desTor_l_JointSpace[i] = ctlDesMsg.lArm_desTorMsg[i];
        }
        for(int i = 0; i < RARMDOF; i++){
            ctlDesMsg_processed.desPos_r_JointSpace[i] = ctlDesMsg.rArm_desMsg[i];
            ctlDesMsg_processed.desVel_r_MDPjointSpace[i] = ctlDesMsg.rArm_MDP_desJntVel[i];
            ctlDesMsg_processed.desTor_r_JointSpace[i] = ctlDesMsg.rArm_desTorMsg[i];
        }
    }else{
        //零力拖动模式，不用传输任何参量

    }

    mdataSend.ctlDesMsg = ctlDesMsg_processed;

    //更新关节刚度模式
    for(int i = 0; i < LARMDOF; i++){
        mdataSend.rigidityCtl_mod_l[i] = ctlDesMsg.lArm_rigidity_mod[i];
        mdataSend.rigidityCtl_mod_r[i] = ctlDesMsg.rArm_rigidity_mod[i];
    }
    //更新PD参数
    for(int i = 0; i < LARMDOF; i++){
        mdataSend.KpTrans_l[i] = ctlDesMsg.lArm_JntKp[i];
        mdataSend.KdTrans_l[i] = ctlDesMsg.lArm_JntKd[i];

        mdataSend.KpTrans_r[i] = ctlDesMsg.rArm_JntKp[i];
        mdataSend.KdTrans_r[i] = ctlDesMsg.rArm_JntKd[i];
    }

    return mdataSend;
}

//----------------------------------------------------------------------------------
//底盘通讯代码

// 初始化函数
void elevator_transmit::init(rclcpp::Node* node) {
    node_ptr_ = node;
    if (!node_ptr_) return;
    
    lifter_cmd_pub_ = node_ptr_->create_publisher<chassis_control_hw::msg::LifterCmd>(
        "/lifter/cmd", rclcpp::QoS(10));
    
    save_preset_cli_ = node_ptr_->create_client<chassis_control_hw::srv::SavePreset>(
        "lifter/save_preset");
    
    move_action_client_ = rclcpp_action::create_client<MoveToPreset>(
        node_ptr_, "lifter/move_to_preset");
}

// 发送升降指令
void elevator_transmit::publish_lifter_cmd(int8_t move_cmd) {
    if (!lifter_cmd_pub_) return;
    auto msg = std::make_shared<chassis_control_hw::msg::LifterCmd>();
    msg->move_cmd.data = move_cmd;
    lifter_cmd_pub_->publish(*msg);
}

// 保存预设（修正变量名）
void elevator_transmit::saveCurrentPresetAsync(
    std::function<void(bool, const std::string&)> callback) {
    
    if (!save_preset_cli_) {  // 修正：save_preset_cli_ 不是 save_preset_client_
        if (callback) callback(false, "Service client not initialized");
        return;
    }
    
    if (!save_preset_cli_->wait_for_service(std::chrono::seconds(2))) {
        if (callback) callback(false, "Service not available");
        return;
    }
    
    auto request = std::make_shared<chassis_control_hw::srv::SavePreset::Request>();
    
    save_preset_cli_->async_send_request(
        request,
        [callback](rclcpp::Client<chassis_control_hw::srv::SavePreset>::SharedFuture future) {
            if (callback) {
                auto response = future.get();
                callback(response->success, response->message);
            }
        });
}

// ========== Action 回调函数实现 ==========

void elevator_transmit::goal_cb(
    const rclcpp_action::ClientGoalHandle<MoveToPreset>::SharedPtr& handle) {
    if (!handle) {
        if (result_callback_) result_callback_(false, "goal rejected");
    }
}

void elevator_transmit::feedback_cb(
    const rclcpp_action::ClientGoalHandle<MoveToPreset>::SharedPtr&,
    const std::shared_ptr<const MoveToPreset::Feedback>& feedback) {
    if (feedback_callback_) {
        feedback_callback_(feedback->current_height, feedback->progress);
    }
}

void elevator_transmit::result_cb(
    const rclcpp_action::ClientGoalHandle<MoveToPreset>::WrappedResult& result) {
    if (!result_callback_) return;
    
    switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            result_callback_(true, result.result->message);
            break;
        case rclcpp_action::ResultCode::ABORTED:
            result_callback_(false, "aborted: " + result.result->message);
            break;
        case rclcpp_action::ResultCode::CANCELED:
            result_callback_(false, "canceled");
            break;
        default:
            result_callback_(false, "unknown error");
    }
}

// 移动函数（修正变量名和占位符）
void elevator_transmit::moveToHeight(
    int target_height,
    std::function<void(bool, const std::string&)> on_result,
    std::function<void(int, float)> on_feedback) {
    
    result_callback_ = on_result;
    feedback_callback_ = on_feedback;
    
    // 修正：使用 move_action_client_ 而不是 action_client_
    if (!move_action_client_) {
        if (result_callback_) result_callback_(false, "Action client not initialized");
        return;
    }
    
    if (!move_action_client_->wait_for_action_server(std::chrono::seconds(2))) {
        if (result_callback_) result_callback_(false, "server timeout");
        return;
    }
    
    auto goal = MoveToPreset::Goal();
    goal.target_height = target_height;
    
    rclcpp_action::Client<MoveToPreset>::SendGoalOptions opt;
    // 修正：使用 std::placeholders::_1
    opt.goal_response_callback = std::bind(&elevator_transmit::goal_cb, this, 
                                           std::placeholders::_1);
    opt.feedback_callback = std::bind(&elevator_transmit::feedback_cb, this,
                                      std::placeholders::_1, std::placeholders::_2);
    opt.result_callback = std::bind(&elevator_transmit::result_cb, this,
                                   std::placeholders::_1);
    
    move_action_client_->async_send_goal(goal, opt);  // 修正变量名
    
    if (node_ptr_) {
        RCLCPP_INFO(node_ptr_->get_logger(), "Sent move goal: %d mm", target_height);
    }
}

// 取消移动
void elevator_transmit::cancelCurrentMove() {
    if (current_goal_handle_ && move_action_client_) {
        move_action_client_->async_cancel_goal(current_goal_handle_);
        if (node_ptr_) {
            RCLCPP_INFO(node_ptr_->get_logger(), "Cancel request sent");
        }
    }
}
//----------------------------------------------------------------------------------
//小车通讯代码
void chassis_transmit::init(rclcpp::Node* node) {
    node_ptr_ = node;
    if (!node_ptr_) return;
    
    // 创建速度控制发布者
    // 话题名称通常为 "/cmd_vel"，根据实际需要修改
    cmd_vel_pub_ = node_ptr_->create_publisher<geometry_msgs::msg::Twist>(
        "/sub_cmd_vel", 
    rclcpp::QoS(10));
    
    RCLCPP_INFO(node_ptr_->get_logger(), "Chassis transmit initialized");
}

void chassis_transmit::publish_chassis_cmd_vel(double linear_x, double linear_y, double angular_z) {
    if (!cmd_vel_pub_) {
        if (node_ptr_) {
            RCLCPP_WARN(node_ptr_->get_logger(), "Cmd_vel publisher not initialized");
        }
        return;
    }
    
    // 创建 Twist 消息
    auto twist_msg = std::make_shared<geometry_msgs::msg::Twist>();
    
    // 设置线速度 (m/s)
    twist_msg->linear.x = linear_x;   // 前进速度
    twist_msg->linear.y = linear_y;   // 横向移动速度（用于全向移动）
    twist_msg->linear.z = 0.0;        // 垂直方向通常为0
    
    // 设置角速度 (rad/s)
    twist_msg->angular.x = 0.0;
    twist_msg->angular.y = 0.0;
    twist_msg->angular.z = angular_z; // 旋转速度
    
    // 发布消息
    cmd_vel_pub_->publish(*twist_msg);
    
    // 可选的调试信息
    if (node_ptr_) {
        RCLCPP_DEBUG(node_ptr_->get_logger(), 
                     "Publishing cmd_vel: linear=(%.2f, %.2f), angular_z=%.2f",
                     linear_x, linear_y, angular_z);
    }
}

//----------------------------------------------------------------------------------
//底盘整体控制代码
void base_transmit::init(rclcpp::Node* node) {
    if (!node) return;

    //初始化通讯对象
    elevator_.init(node);
    chassis_.init(node);
}

//升降台位置查询接口
double base_transmit::get_elevatorPos(){

    return 0.0;
}

//整体控制接口封装
void base_transmit::base_control(base_control_msg ctl_msg){
    //每一个控制周期中，都要进行一次升降台控制通讯 + 一次底盘控制通讯

    //升降机通讯
    if(ctl_msg.lifter_cmd.control_mod == 0){
        elevator_.publish_lifter_cmd(ctl_msg.lifter_cmd.move_cmd);
    }else{
        elevator_.moveToHeight(ctl_msg.lifter_cmd.target_position, 
            [](bool success, const std::string& message) {
                if (success) {
                    RCLCPP_INFO(rclcpp::get_logger("base_transmit"), "Move successful: %s", message.c_str());
                } else {
                    RCLCPP_ERROR(rclcpp::get_logger("base_transmit"), "Move failed: %s", message.c_str());
                }
            },
            [](int current_height, float progress) {
                RCLCPP_INFO(rclcpp::get_logger("base_transmit"), "Move feedback: current height = %d mm, progress = %.2f%%", current_height, progress * 100);
            }
        );
    }

    //判断是否记录当前位置
    if(ctl_msg.lifter_cmd.save_preset){
        elevator_.saveCurrentPresetAsync([](bool success, const std::string& message) {
            if (success) {
                RCLCPP_INFO(rclcpp::get_logger("base_transmit"), "Preset saved successfully: %s", message.c_str());
            } else {
                RCLCPP_ERROR(rclcpp::get_logger("base_transmit"), "Failed to save preset: %s", message.c_str());   
            };
        });
    }

    //下发底盘控制指令
    chassis_.publish_chassis_cmd_vel(ctl_msg.chassis_cmd.linear_x, ctl_msg.chassis_cmd.linear_y, ctl_msg.chassis_cmd.angular_z);
}

//----------------------------------------------------------------------------------