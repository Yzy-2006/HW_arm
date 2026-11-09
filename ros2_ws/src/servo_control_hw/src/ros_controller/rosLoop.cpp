#include "ros_controller/rosLoop.h"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "std_msgs/msg/float64_multi_array.hpp"
#include <algorithm>    // std::clamp / std::max（夹爪限幅用）
#include <stdexcept>
#include <cmath>
#include <limits>
#include "servo_control/homingMonitor.h"
#include "servo_control/motorZeroSession.h"
#include "servo_control/shutdownAwareRate.h"

//全局变量 -- 数据传输
std::vector<double> joint_pos(SERVODOF, 0.0);
std::vector<double> joint_kp(SERVODOF, 0.0);
std::vector<double> joint_kd(SERVODOF, 0.0);
std::vector<double> joint_spd(SERVODOF, 0.0);
std::vector<double> joint_tor(SERVODOF, 0.0);

double TEMPTIME;        //迭代时间
int SERVODOF;            //伺服控制自由度
//双臂自由度
int LARMDOF;
int RARMDOF;

double ZERO_POS_THRESHOLD;   //回零判定阈值 (rad)

ServoNode::ServoNode() : Node ("servo_node"),
    has_received_command_(false),
    ifFirstTemp(true)
{

    const auto calibration_side = this->declare_parameter<std::string>(
        "arm_zero_calibration_side", ""
    );
    if (
        !calibration_side.empty()
        && calibration_side != "left"
        && calibration_side != "right"
        && calibration_side != "both"
    ) {
        throw std::runtime_error(
            "arm_zero_calibration_side must be left, right, both, or empty"
        );
    }
    arm_zero_calibration_ = !calibration_side.empty();
    left_arm_zero_calibration_ = calibration_side == "left";
    right_arm_zero_calibration_ = calibration_side == "right";
    both_arm_zero_calibration_ = calibration_side == "both";
    can_channel_scan_ = this->declare_parameter<bool>("can_channel_scan", false);
    feedback_only_ = this->declare_parameter<bool>("feedback_only", false);
    motor_zero_joint_ = this->declare_parameter<std::string>("motor_zero_joint", "");
    if (!motor_zero_joint_.empty()) {
        MotorZeroTarget::from_name(motor_zero_joint_);  // Validate before opening hardware.
        if (feedback_only_ || can_channel_scan_ || arm_zero_calibration_)
            throw std::runtime_error("motor_zero_joint cannot be combined with other service modes");
    }
    if (feedback_only_ && (can_channel_scan_ || arm_zero_calibration_))
        throw std::runtime_error("feedback_only cannot be combined with scan or calibration");
    if (can_channel_scan_ && arm_zero_calibration_) {
        throw std::runtime_error(
            "can_channel_scan cannot be combined with arm zero calibration"
        );
    }

    //显式 executor 替代全局 rclcpp::spin_some
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(this->get_node_base_interface());

    //----------初始化零位全局变量----------
    yaml_path_zeroPos = ament_index_cpp::get_package_share_directory("ctl_msgs_hw") + "/config/zeropos.yaml";
    yaml_path_initialPD = ament_index_cpp::get_package_share_directory("ctl_msgs_hw") + "/config/initialPD.yaml";
    controllerConfig_yamlPath = ament_index_cpp::get_package_share_directory("ctl_msgs_hw") + "/config/controllerConfig.yaml";

    if(!loadServoNodeCtlFromYaml(controllerConfig_yamlPath)){
        throw std::runtime_error("Cannot load servo controller configuration");
    }

    //初始化关键全局变量
    TEMPTIME = ctl_inteMsg_config.TempTime;
    LARMDOF = static_cast<int>(ctl_inteMsg_config.LARMDOF);
    RARMDOF = static_cast<int>(ctl_inteMsg_config.RARMDOF);
    SERVODOF = LARMDOF + RARMDOF;
    const auto controller_config = YAML::LoadFile(controllerConfig_yamlPath);
    const auto smoothing = controller_config["servoCommandSmoothing"];
    bool command_lowpass_enabled = true;
    if (smoothing) {
        command_interpolation_mode_ = smoothing["interpolation_mode"].as<int>(2);
        command_lowpass_enabled = smoothing["lowpass_enabled"].as<bool>(true);
        command_output_kalman_enabled_ = smoothing["output_kalman_enabled"].as<bool>(true);
    }
    if (command_interpolation_mode_ != 1 && command_interpolation_mode_ != 2)
        throw std::runtime_error("servoCommandSmoothing.interpolation_mode must be 1 or 2");
    runin_control_core.set_lowpass_enabled(command_lowpass_enabled);
    RCLCPP_INFO(get_logger(), "Command smoothing: interpolation=%s lowpass=%s output_kalman=%s",
                command_interpolation_mode_ == 1 ? "linear" : "B-spline",
                command_lowpass_enabled ? "enabled" : "disabled",
                command_output_kalman_enabled_ ? "enabled" : "disabled");
    homing_handoff_.load(controller_config);
    motor_position_offsets.load(controller_config);
    RCLCPP_INFO(get_logger(), "CAN 0 ID 2 position offset: %+.6f deg; feedback uses inverse offset",
                motor_position_offsets.offset(0, 2) * 180.0 / MotorPositionOffsets::pi);
    const auto safety = controller_config["armSafety"];
    if (!safety || SERVODOF != 14) throw std::runtime_error("armSafety configuration required for 14 joints");
    const auto protection_mode = safety["protection_mode"] ? safety["protection_mode"].as<std::string>() : "full";
    if (protection_mode != "full" && protection_mode != "speed_only")
        throw std::runtime_error("armSafety.protection_mode must be full or speed_only");
    arm_safety.limits.speed_only = protection_mode == "speed_only";
    RCLCPP_INFO(get_logger(), "Servo protection mode: %s", protection_mode.c_str());
    arm_safety.limits.check_command_position_limits = safety["check_command_position_limits"].as<bool>(true);
    arm_safety.limits.check_feedback_position_limits = safety["check_feedback_position_limits"].as<bool>(true);
    RCLCPP_INFO(get_logger(), "Position bounds checks: command=%s feedback=%s",
                arm_safety.limits.check_command_position_limits ? "enabled" : "disabled",
                arm_safety.limits.check_feedback_position_limits ? "enabled" : "disabled");
    auto read_limit = [&](const char* key, double& value) {
        value = safety[key].as<double>();
        if (!std::isfinite(value) || value <= 0) throw std::runtime_error(std::string("Invalid armSafety.") + key);
    };
    read_limit("feedback_timeout_s", arm_safety.limits.feedback_timeout);
    read_limit("command_timeout_s", arm_safety.limits.command_timeout);
    read_limit("max_speed_rad_s", arm_safety.limits.max_speed);
    if (const auto mask = safety["speed_limit_enabled_per_arm"]) {
        if (!mask.IsSequence() || mask.size() != 7)
            throw std::runtime_error("armSafety.speed_limit_enabled_per_arm must contain 7 booleans");
        for (size_t i = 0; i < 7; ++i)
            arm_safety.limits.speed_limit_enabled_per_arm[i] = mask[i].as<bool>();
    }
    for (size_t i = 0; i < 7; ++i)
        RCLCPP_INFO(get_logger(), "Both arms joint %zu software speed limit: %s", i + 1,
                    arm_safety.speed_limited(i) ? "enabled" : "disabled");
    arm_safety.limits.feedback_speed_trip_samples = safety["feedback_speed_trip_samples"].as<int>(1);
    if (arm_safety.limits.feedback_speed_trip_samples < 1)
        throw std::runtime_error("armSafety.feedback_speed_trip_samples must be a positive integer");
    RCLCPP_INFO(get_logger(), "Feedback overspeed trip: |v|>%.3f rad/s for %d consecutive feedback updates",
                arm_safety.limits.max_speed, arm_safety.limits.feedback_speed_trip_samples);
    read_limit("max_feedforward", arm_safety.limits.max_feedforward);
    read_limit("max_feedforward_rate", arm_safety.limits.max_feedforward_rate);
    read_limit("tracking_error_rad", arm_safety.limits.tracking_error);
    read_limit("tracking_duration_s", arm_safety.limits.tracking_duration);
    read_limit("fault_damping", arm_safety.limits.damping);
    read_limit("feedback_position_tolerance_rad", arm_safety.limits.feedback_position_tolerance);
    if (arm_safety.limits.damping > 50) throw std::runtime_error("Fault damping exceeds protocol range");
    if (safety["lower"].size() != 14 || safety["upper"].size() != 14)
        throw std::runtime_error("armSafety position limits must contain 14 values");
    for (size_t i = 0; i < 14; ++i) {
        const double lo = safety["lower"][i].as<double>(), hi = safety["upper"][i].as<double>();
        if (!std::isfinite(lo) || !std::isfinite(hi) || lo >= hi)
            throw std::runtime_error("Invalid armSafety position limits");
        arm_safety.limits.lower[i] = lo;
        arm_safety.limits.upper[i] = hi;
    }
    safety_pub_ = create_publisher<std_msgs::msg::String>("/towel/safety/state", rclcpp::QoS(1).transient_local());
    handoff_pub_ = create_publisher<ctl_msgs_hw::msg::HandoffStatus>("/towel/control/handoff", 1);
    safety_stop_ = create_subscription<std_msgs::msg::String>("/towel/safety/stop", 1,
        [](std_msgs::msg::String::ConstSharedPtr msg) { arm_safety.trip("upstream: " + msg->data); });
    safety_timer_ = create_wall_timer(std::chrono::milliseconds(50), [this] {
        std_msgs::msg::String msg;
        msg.data = arm_safety.fault.empty()
            ? (!startup_homing_ && arm_safety.ready(arm_safety_now()) ? "ready" : "waiting") : "fault:" + arm_safety.fault;
        safety_pub_->publish(msg);
        ctl_msgs_hw::msg::HandoffStatus handoff;
        handoff.header.stamp = now();
        handoff.source_header = last_source_header_;
        handoff.handoff_epoch = homing_handoff_.epoch();
        handoff.complete = !startup_homing_ && !arm_zero_calibration_ &&
            arm_safety.ready(arm_safety_now()) && !homing_handoff_.transitioning();
        for (size_t i = 0; i < std::min(size_t{14}, servo_desMsg_orig.size()); ++i)
            handoff.commanded_position.push_back(servo_desMsg_orig[i].joint_pos);
        handoff_pub_->publish(handoff);
        if (!arm_safety.fault.empty()) RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000,
            "Latched arm fault: %s; damping only, no load holding guarantee", arm_safety.fault.c_str());
    });
    safety_reset_ = create_service<std_srvs::srv::Trigger>("/towel/safety/reset",
        [this](std_srvs::srv::Trigger::Request::SharedPtr, std_srvs::srv::Trigger::Response::SharedPtr response) {
            if (startup_homing_) {
                response->success = false;
                response->message = "Startup homing has not finished";
                return;
            }
            response->success = arm_safety.reset(arm_safety_now());
            response->message = response->success ? "Fault reset; restart upstream controllers from feedback"
                : "Stop upstream controllers for 1s; require fresh stationary feedback within limits";
            if (response->success) {
                last_source_header_ = std_msgs::msg::Header{};
                ifRuninerInited = false;
                ifRunin = false;
                trajectory_sample_ = false;
                ifFirstRunin = true;
                posOutpFtr = runinOutpFtr{};
                velOutpFtr = runinOutpFtr{};
                TorOutpFtr = runinOutpFtr{};
                ifGetGripperMsg = false;
                l_gripper_pos_received = r_gripper_pos_received = false;
                l_gripper_posTraj.clear();
                r_gripper_posTraj.clear();
                l_gripper_torTraj.assign(1, 0.0);
                r_gripper_torTraj.assign(1, 0.0);
                l_gripper_tor_get = r_gripper_tor_get = 0.0;
                l_gripper_tor = r_gripper_tor = 0.0;
                gripper_LastTorRec_l = gripper_LastTorRec_r = 0.0;
                gripper_otputTimes = 0;
                setLeftGripperCmd(gripperCmd{});
                setRightGripperCmd(gripperCmd{});
                for (auto& c : servo_desMsg_orig) {
                    c.transMod = 2; c.joint_pos = c.joint_spd = c.joint_tor = c.joint_Kp = 0;
                    c.joint_Kd = arm_safety.limits.damping;
                }
                homing_handoff_.begin(servo_desMsg_orig, arm_safety);
            }
        });
    ZERO_POS_THRESHOLD = ctl_inteMsg_config.ZeroPos_Threshold;
    if (!std::isfinite(ZERO_POS_THRESHOLD) || ZERO_POS_THRESHOLD <= 0)
        throw std::runtime_error("ZeroPos_Threshold must be finite and positive");
    netCard_name = ctl_inteMsg_config.NetCardName;

    loadZeroPosFromYaml(yaml_path_zeroPos);
    loadInitialPDFromYaml(yaml_path_initialPD, "low");      //获取刚度配置，默认低刚度

    //-----------------------------------

    //反馈值发送器初始化
    RetData_pub = this->create_publisher<ctl_msgs_hw::msg::RetInformation>(
        "/dexbotCtl/dualArmCtl_fdback",
        200
    );

    //上位机接收器初始化
    CtlData = this->create_subscription<ctl_msgs_hw::msg::CtlInformation>(
        "/dexbotCtl/ctl_data_pub",
        1,
        std::bind(&ServoNode::message_callback, this, std::placeholders::_1)
    );

    //----------debug消息类型初始化-----------
    debugDataPub_comparison = this->create_publisher<ctl_msgs_hw::msg::DbgInformation>(
        "/debug_msgs/debug_of_comparison",
        10
    );

    //----------夹爪反馈发布----------
    gripper_fdb_pub = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/gripper/fdb",
        10
    );

    //----------夹爪控制订阅----------
    gripper_des_get = this->create_subscription<ctl_msgs_hw::msg::GripperCtl>(
        "/gripper/ctl",
        200,
        std::bind(&ServoNode::gripper_desMsg_get, this, std::placeholders::_1)
    );

    //----------控制调试信息发送初始化----------
    performance_test = std::make_unique<performanceTestSeal>(this);  // 传入当前节点指针

    //初始化debug发布器
    global_debugPub = new debugMsgPub(this);

}

//ros循环通讯变量初始化函数
void ServoNode::rosLoopVar_Init(){

    //-----传输变量初始化-----

    servo_CAN_Jnt_arrange = CAN_JNT_arrange_outp();
    can_jnt_Kt_unpacked = joint_Kt_get();
    servo_desMsg.resize(servo_CAN_Jnt_arrange.CAN_board_num);
    servo_retMsg.resize(servo_CAN_Jnt_arrange.CAN_board_num);
    for(int i = 0; i < servo_CAN_Jnt_arrange.CAN_board_num; i++){
        joint_desMsg temp_desMsg;
        joint_retMsg temp_retMsg;

        temp_desMsg.transMod = 1;
        temp_desMsg.joint_pos = 0.0;
        temp_desMsg.joint_spd = 0.0;
        temp_desMsg.joint_tor = 0.0;
        temp_desMsg.joint_Kp = 0.0;
        temp_desMsg.joint_Kd = 0.0;

        temp_retMsg.joint_posRet = 0.0;
        temp_retMsg.joint_spdRet = 0.0;
        temp_retMsg.joint_currentRet = 0.0;
        temp_retMsg.joint_temperature = 0.0;

        servo_desMsg[i].assign(servo_CAN_Jnt_arrange.Jnt_num_on_each_board[i], temp_desMsg);
        servo_retMsg[i].assign(servo_CAN_Jnt_arrange.Jnt_num_on_each_board[i], temp_retMsg);
    }

    int m_vector_len = 0;
    for(int i = 0; i < servo_CAN_Jnt_arrange.CAN_board_num; i++){
        m_vector_len += servo_CAN_Jnt_arrange.Jnt_num_on_each_board[i];
    }
    this->vector_len = m_vector_len;
    servo_desMsg_orig.resize(this->vector_len);
    servo_retMsg_unpacked.resize(this->vector_len);
    for(int i = 0; i < this->vector_len; i++){
        servo_desMsg_orig[i].joint_pos = 0.0;
        servo_desMsg_orig[i].joint_spd = 0.0;
        servo_desMsg_orig[i].joint_tor = 0.0;
        servo_desMsg_orig[i].joint_Kp = 0.0;
        servo_desMsg_orig[i].joint_Kd = 0.0;

        servo_retMsg_unpacked[i].joint_posRet = 0.0;
        servo_retMsg_unpacked[i].joint_spdRet = 0.0;
        servo_retMsg_unpacked[i].joint_currentRet = 0.0;
        servo_retMsg_unpacked[i].joint_temperature = 0.0;
    }


    //-----过程控制变量初始化-----

    servo_desVel_last.assign(this->vector_len, 0.0);

    lArm_transMod.assign(LARMDOF, 1);
    
    lArm_pos_receive = lArm_zeroPos;
    lArm_vel_receive.assign(LARMDOF, 0.0);

    lArm_torComp_feedforward_receive.assign(LARMDOF, 0.0);
    lArm_torComp_grav_receive.assign(LARMDOF, 0.0);
    lArm_torComp_interia_receive.assign(LARMDOF, 0.0);

    lArm_Kp_receive = lArm_defaultKp;
    lArm_Kd_receive = lArm_defaultKd;

    rArm_transMod.assign(RARMDOF, 1);

    rArm_pos_receive = rArm_zeroPos;
    rArm_vel_receive.assign(RARMDOF, 0.0);

    rArm_torComp_feedforward_receive.assign(RARMDOF, 0.0);
    rArm_torComp_grav_receive.assign(RARMDOF, 0.0);
    rArm_torComp_interia_receive.assign(RARMDOF, 0.0);

    rArm_Kp_receive = rArm_defaultKp;
    rArm_Kd_receive = rArm_defaultKd;

    //插补过程辅助变量
    ifFirstRunin = true;

    temp_len = 0;

    time_res = this->now().seconds() + this->now().nanoseconds() * 1e-9;
    time_last = this->now().seconds() + this->now().nanoseconds() * 1e-9;

    //-----插补器变量初始化-----
    l_posReceive_atRuninSt.assign(LARMDOF, 0.0);
    r_posReceive_atRuninSt.assign(RARMDOF, 0.0);

    ifRunin = false;  // Wait for a real upstream command before starting interpolation.

    //-----模式控制变量初始化-----
    tempTime_upperMonitor = 0.02;
    runin_mod = 2;
    if_runin_renew = 0;
    msg_get_ifUseDebugMod = 0;
    if_useret_to_start_runin = 1;

    //-----夹爪控制变量初始化-----
    l_gripper_tor_get = 0.0;
    r_gripper_tor_get = 0.0;

    l_gripper_tor = 0.0;
    r_gripper_tor = 0.0;

    time_res_gripper = 0.0;
    time_last_gripper = 0.0;
    winlen_gripper = 1;

    setLeftGripperCmd(gripperCmd{});
    setRightGripperCmd(gripperCmd{});

    l_gripper_torTraj.assign(1, 0.0);
    r_gripper_torTraj.assign(1, 0.0);

    gripper_Tor_trajLen = 1;

    gripper_LastTorRec_l = 0.0;
    gripper_LastTorRec_r = 0.0;

    gripper_otputTimes = 0;

    //夹爪力限幅初始化
    l_gripper_limit.Init(ctl_inteMsg_config.GripperLeft_Tor_Thr, ctl_inteMsg_config.GripperLeft_Temp_Thr,
                         ctl_inteMsg_config.Gripper_TempRecover_Hys);
    r_gripper_limit.Init(ctl_inteMsg_config.GripperRight_Tor_Thr, ctl_inteMsg_config.GripperRight_Temp_Thr,
                         ctl_inteMsg_config.Gripper_TempRecover_Hys);


}

//ros消息参量发送模块 -- 反馈值发送函数
void ServoNode::msg_publish(){
    auto tempRetData_pub = ctl_msgs_hw::msg::RetInformation();
    tempRetData_pub.header.stamp = this->now();

    //分配反馈值数组
    tempRetData_pub.l_position.resize(LARMDOF);
    tempRetData_pub.l_speed.resize(LARMDOF);
    tempRetData_pub.l_iq.resize(LARMDOF);
    tempRetData_pub.r_position.resize(RARMDOF);
    tempRetData_pub.r_speed.resize(RARMDOF);
    tempRetData_pub.r_iq.resize(RARMDOF);

    //获取并发送反馈值 — 左臂
    for(int i = 0; i < LARMDOF; i++){
        tempRetData_pub.l_position[i] = servo_retMsg_unpacked[i].joint_posRet;
        tempRetData_pub.l_speed[i]   = servo_retMsg_unpacked[i].joint_spdRet;
        tempRetData_pub.l_iq[i]      = servo_retMsg_unpacked[i].joint_currentRet;
    }
    //获取并发送反馈值 — 右臂
    for(int i = 0; i < RARMDOF; i++){
        tempRetData_pub.r_position[i] = servo_retMsg_unpacked[i + LARMDOF].joint_posRet;
        tempRetData_pub.r_speed[i]   = servo_retMsg_unpacked[i + LARMDOF].joint_spdRet;
        tempRetData_pub.r_iq[i]      = servo_retMsg_unpacked[i + LARMDOF].joint_currentRet;
    }

    if(ifFirstTemp){
        tempRetData_pub.if_first_time = 1;
        ifFirstTemp = false;
    }else{
        tempRetData_pub.if_first_time = 0;
    }

    RetData_pub->publish(tempRetData_pub);
}

//ros消息参量发送模块 -- 反馈值发送函数
void ServoNode::ctlmsg_get(){
    executor_->spin_some();  // 非阻塞检查回调（显式 executor，与主循环同线程）
}

//消息传输辅助函数 -- 仅仅进行一次消息的接收(与底层关节的通讯)
void ServoNode::EtherCATmsg_Get(){
    servo_retMsg = runMsgGet();
    retMsg_unpack_func();
}

//消息传输辅助函数 -- 进行一次消息的收发(与底层关节的通讯)
void ServoNode::EtherCATrun(){
    //进行一轮数据转换与收发
    desMsg_pack_func();
    servo_retMsg = runImpl(servo_desMsg);
    retMsg_unpack_func();
}

void ServoNode::apply_arm_zero_calibration_commands(
    const std::vector<double>& left_hold_positions,
    const std::vector<double>& right_hold_positions
){
    if (static_cast<int>(left_hold_positions.size()) != LARMDOF) {
        throw std::runtime_error("Missing left-arm calibration hold positions");
    }
    if (static_cast<int>(right_hold_positions.size()) != RARMDOF) {
        throw std::runtime_error("Missing right-arm calibration hold positions");
    }

    for(int i = 0; i < LARMDOF; i++){
        if (left_arm_zero_calibration_ || both_arm_zero_calibration_) {
            servo_desMsg_orig[i].transMod = 5;
            servo_desMsg_orig[i].joint_pos = 0.0;
            servo_desMsg_orig[i].joint_spd = 0.0;
            servo_desMsg_orig[i].joint_tor = 0.0;
            servo_desMsg_orig[i].joint_Kp = 0.0;
            servo_desMsg_orig[i].joint_Kd = 0.0;
            continue;
        }
        servo_desMsg_orig[i].transMod = 1;
        servo_desMsg_orig[i].joint_pos = left_hold_positions[i];
        servo_desMsg_orig[i].joint_spd = 0.0;
        servo_desMsg_orig[i].joint_tor = 0.0;
        servo_desMsg_orig[i].joint_Kp = lArm_defaultKp[i];
        servo_desMsg_orig[i].joint_Kd = lArm_defaultKd[i];
    }
    for(int i = 0; i < RARMDOF; i++){
        const int index = i + LARMDOF;
        if (right_arm_zero_calibration_ || both_arm_zero_calibration_) {
            servo_desMsg_orig[index].transMod = 5;
            servo_desMsg_orig[index].joint_pos = 0.0;
            servo_desMsg_orig[index].joint_spd = 0.0;
            servo_desMsg_orig[index].joint_tor = 0.0;
            servo_desMsg_orig[index].joint_Kp = 0.0;
            servo_desMsg_orig[index].joint_Kd = 0.0;
            continue;
        }
        servo_desMsg_orig[index].transMod = 1;
        servo_desMsg_orig[index].joint_pos = right_hold_positions[i];
        servo_desMsg_orig[index].joint_spd = 0.0;
        servo_desMsg_orig[index].joint_tor = 0.0;
        servo_desMsg_orig[index].joint_Kp = rArm_defaultKp[i];
        servo_desMsg_orig[index].joint_Kd = rArm_defaultKd[i];
    }
}

//主要循环函数
void ServoNode::run(){
    std::cout << "SOEM主站代码 -- ENCOS关节控制与测试代码" << std::endl;

    //初始化ros迭代频率
    double loopRate_unrounded = 1 / TEMPTIME;
    int loopRate = static_cast<int>(loopRate_unrounded);
    rclcpp::Rate loop_rate(loopRate);
    std::cout << "执行频率：" << loopRate << " Hz。" << std::endl;

    //初始化EtherCAT通讯
    EtherCAT_Init(netCard_name.data());        //输入量 -- 电脑网卡名称
    std::cout << "slave count = " << ec_slavecount << std::endl;
    if(ec_slavecount <= 0){
        std::cout << "未找到从站，进入调试模式" << std::endl;
        ethercat_debug_mode_ = true;
    }

    //开启transmit过程
    startRun();

    if (!motor_zero_joint_.empty()) {
        const auto target = MotorZeroTarget::from_name(motor_zero_joint_);
        MotorZeroSession session(target.motor_id, arm_safety_now());
        RCLCPP_WARN(get_logger(), "Internal zero: ONLY %s (CAN %d ID %d). Mechanism must be supported at its reference pose; no other master may run.",
                    motor_zero_joint_.c_str(), target.channel, target.motor_id);
        RCLCPP_WARN(get_logger(), "This operation does not disable motors. Software offset remains %+.6f deg; raw feedback is used for verification.",
                    motor_position_offsets.offset(target.channel, target.motor_id) * 180 / MotorPositionOffsets::pi);
        try {
            while (rclcpp::ok() && session.phase != MotorZeroSession::Phase::done &&
                   session.phase != MotorZeroSession::Phase::failed) {
                const auto request = session.next(arm_safety_now());
                if (session.phase == MotorZeroSession::Phase::failed) break;
                if (request.dlc == 4)
                    RCLCPP_WARN(get_logger(), "Sending ONE zero request to CAN %d ID %d; original raw angle=%.6f deg",
                                target.channel, target.motor_id, session.before_deg);
                if (!rclcpp::ok()) break;
                bool valid_wkc = false;
                const auto reply = MotorSingleExchange(target.channel, target.motor_id, request, valid_wkc);
                session.receive(reply, valid_wkc, arm_safety_now());
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
            }
        } catch (...) {
            ec_close();
            throw;
        }
        ec_close();
        if (session.phase != MotorZeroSession::Phase::done)
            throw std::runtime_error("Single-motor zero failed/interrupted: " + session.error +
                (session.sent ? "; zero request issued, motor zero may have changed; inspect raw feedback before retrying"
                              : "; no zero request issued"));
        RCLCPP_INFO(get_logger(), "Internal zero ACK + raw readback verified for %s: before=%.6f deg after=%.6f deg. Power-cycle persistence NOT verified. Review software offsets/home poses before control.",
                    motor_zero_joint_.c_str(), session.before_deg, session.after_deg);
        return;
    }

    if (feedback_only_) {
        RCLCPP_INFO(get_logger(), "Feedback-only: parameter queries, no homing or motion commands. Stop other EtherCAT masters first.");
        RCLCPP_WARN(get_logger(), "Queries do not disable motors or cancel previous commands; support the mechanism as needed.");
        double last_print = -1;
        while (rclcpp::ok()) {
            const double now = arm_safety_now();
            const bool print = last_print < 0 || now - last_print >= 1.0 / 3;
            MotorFeedbackQueryStep(print);
            if (print) last_print = now;
            // Do not execute motion, reset or safety-ready callbacks in query mode.
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        ec_close();
        return;
    }

    //初始化传输相关变量
    rosLoopVar_Init();

    if (can_channel_scan_) {
        if (ec_slavecount <= 0) {
            throw std::runtime_error(
                "Cannot scan CAN channels without an EtherCAT gateway"
            );
        }
        RCLCPP_WARN(
            this->get_logger(),
            "CAN scan: zero-current only; support both arms and do not run another servo node"
        );
        CanChannelScanBegin();
        constexpr int scan_cycles = 2000;
        for (int cycle = 0; rclcpp::ok() && cycle < scan_cycles; ++cycle) {
            CanChannelScanStep();
            sleep_while_running(loop_rate);
        }
        CanChannelScanPrintResult();
        return;
    }


    //-----------------------------------------------
    //初始位置获取
    for(int i = 0; i < 20; i++){
        EtherCATmsg_Get();
        rclcpp::sleep_for(std::chrono::milliseconds(1));
    }

    if (arm_zero_calibration_) {
        if (static_cast<int>(servo_retMsg_unpacked.size()) < LARMDOF + RARMDOF) {
            throw std::runtime_error(
                "Cannot start arm calibration without full arm feedback"
            );
        }
        left_arm_calibration_hold_positions_.resize(LARMDOF);
        for (int i = 0; i < LARMDOF; i++) {
            left_arm_calibration_hold_positions_[i] =
                servo_retMsg_unpacked[i].joint_posRet;
        }
        right_arm_calibration_hold_positions_.resize(RARMDOF);
        for (int i = 0; i < RARMDOF; i++) {
            right_arm_calibration_hold_positions_[i] =
                servo_retMsg_unpacked[i + LARMDOF].joint_posRet;
        }

        const auto has_nonzero_feedback = [](const std::vector<double>& positions) {
            return std::any_of(
                positions.begin(),
                positions.end(),
                [](double position) { return std::abs(position) > 1e-6; }
            );
        };
        if (left_arm_zero_calibration_
            && !has_nonzero_feedback(right_arm_calibration_hold_positions_)) {
            throw std::runtime_error(
                "Cannot hold right arm without feedback; use both-arm calibration instead"
            );
        }
        if (right_arm_zero_calibration_
            && !has_nonzero_feedback(left_arm_calibration_hold_positions_)) {
            throw std::runtime_error(
                "Cannot hold left arm without feedback; use both-arm calibration instead"
            );
        }

        if (both_arm_zero_calibration_) {
            RCLCPP_WARN(
                this->get_logger(),
                "both-arm calibration: zero-current; support both arms before moving them"
            );
        } else if (left_arm_zero_calibration_) {
            RCLCPP_WARN(
                this->get_logger(),
                "left arm calibration: zero-current; support the arm before moving it"
            );
        } else {
            RCLCPP_WARN(
                this->get_logger(),
                "right arm calibration: zero-current; support the arm before moving it"
            );
        }
    }

    if (!arm_zero_calibration_) {
        // Acquire complete ACK1 feedback using damping, never position commands.
        for (auto& c : servo_desMsg_orig) {
            c = {1, 0, 0, 0, 0, arm_safety.limits.damping};
        }
        const double feedback_deadline = arm_safety_now() + 1.0;
        while (rclcpp::ok() && arm_safety.fault.empty() && !arm_safety.ready(arm_safety_now()) &&
               arm_safety_now() < feedback_deadline) {
            EtherCATrun();
            msg_publish();
            executor_->spin_some();
            sleep_while_running(loop_rate);
        }
        if (!arm_safety.ready(arm_safety_now())) arm_safety.trip("homing_feedback_not_ready");

        std::vector<double> lArm_stPos(7), rArm_stPos(7);
        std::array<double, 14> homing_target{};
        for (size_t i = 0; i < 7; ++i) {
            lArm_stPos[i] = arm_safety.feedback[i].position;
            rArm_stPos[i] = arm_safety.feedback[i + 7].position;
        }
        if (arm_safety.fault.empty()) {
            const auto error = retZero_Init.retZero_runin(lArm_stPos, rArm_stPos, arm_safety);
            if (!error.empty()) arm_safety.trip(error);
        }
        if (arm_safety.fault.empty()) {
            for (size_t i = 0; i < 7; ++i) {
                homing_target[i] = lArm_zeroPos[i];
                homing_target[i + 7] = rArm_zeroPos[i];
            }
        }
        HomingMonitor homing_monitor(homing_target, ZERO_POS_THRESHOLD);

        //回零控制
        bool needRetZero = false;
        if(arm_safety.fault.empty()){
            for(int i = 0; i < LARMDOF; i++){
                if(std::abs(lArm_stPos[i] - lArm_zeroPos[i]) > ZERO_POS_THRESHOLD){
                    needRetZero = true;
                    break;
                }
            }
            for(int i = 0; i < RARMDOF; i++){
                if(std::abs(rArm_stPos[i] - rArm_zeroPos[i]) > ZERO_POS_THRESHOLD){
                    needRetZero = true;
                    break;
                }
            }
        }

        //若达成回零控制条件，则开启回零控制
        if(needRetZero){
        RCLCPP_INFO(get_logger(), "Homing preflight passed; executing validated trajectory");

        //执行回零控制循环
        bool retZero_done = false;
        while(!retZero_done && rclcpp::ok() && arm_safety.fault.empty()){
            executor_->spin_some();
            if (!arm_safety.fault.empty()) break;
            //获取下一帧回零轨迹输出
            TrajCtlOutp retZero_outp = retZero_Init.retZero_trajOutp();

            //将回零轨迹装入目标消息
            for(int i = 0; i < LARMDOF; i++){
                servo_desMsg_orig[i].transMod = 1;                          //力位混合控制模式
                servo_desMsg_orig[i].joint_pos = retZero_outp.lArm_desPos[i];
                servo_desMsg_orig[i].joint_spd = retZero_outp.lArm_desVel[i];
                servo_desMsg_orig[i].joint_tor = 0.0;
                servo_desMsg_orig[i].joint_Kp  = retZero_outp.lArm_Kp[i];
                servo_desMsg_orig[i].joint_Kd  = retZero_outp.lArm_Kd[i];
            }
            for(int i = 0; i < RARMDOF; i++){
                servo_desMsg_orig[i + LARMDOF].transMod = 1;
                servo_desMsg_orig[i + LARMDOF].joint_pos = retZero_outp.rArm_desPos[i];
                servo_desMsg_orig[i + LARMDOF].joint_spd = retZero_outp.rArm_desVel[i];
                servo_desMsg_orig[i + LARMDOF].joint_tor = 0.0;
                servo_desMsg_orig[i + LARMDOF].joint_Kp  = retZero_outp.rArm_Kp[i];
                servo_desMsg_orig[i + LARMDOF].joint_Kd  = retZero_outp.rArm_Kd[i];
            }

            //----------------------
            //debug
            std::vector<double> lArm_pos, rArm_pos;
            std::vector<double> lArm_vel, rArm_vel;
            std::vector<double> lArm_tor, rArm_tor;

            lArm_pos.assign(LARMDOF, 0.0);  rArm_pos.assign(RARMDOF, 0.0);
            lArm_vel.assign(LARMDOF, 0.0);  rArm_vel.assign(RARMDOF, 0.0);
            lArm_tor.assign(LARMDOF, 0.0);  rArm_tor.assign(RARMDOF, 0.0);
            for(int i = 0; i < LARMDOF; i++){
                lArm_pos[i] = servo_desMsg_orig[i].joint_pos;
                lArm_vel[i] = servo_desMsg_orig[i].joint_spd;
                lArm_tor[i] = servo_desMsg_orig[i].joint_tor;
            }
            for(int i = 0; i < RARMDOF; i++){
                rArm_pos[i] = servo_desMsg_orig[i + LARMDOF].joint_pos;
                rArm_vel[i] = servo_desMsg_orig[i + LARMDOF].joint_spd;
                rArm_tor[i] = servo_desMsg_orig[i + LARMDOF].joint_tor;
            }
            global_debugPub->debug_msg_publish_1(lArm_pos, rArm_pos);
            global_debugPub->debug_msg_publish_2(lArm_vel, rArm_vel);
            global_debugPub->debug_msg_publish_3(lArm_tor, rArm_tor);

            //发送一轮EtherCAT指令并接收反馈
            EtherCATrun();

            //发送一轮反馈值
            msg_publish();

            const auto result = homing_monitor.update(arm_safety, arm_safety_now(), retZero_outp.ifOutpReachEnd);
            if (result == HomingMonitor::Result::fault) break;
            retZero_done = result == HomingMonitor::Result::reached ||
                           result == HomingMonitor::Result::finished_unverified;

            //尾处理代码
            executor_->spin_some();
            sleep_while_running(loop_rate);
        }

            if (retZero_done && arm_safety.fault.empty() && rclcpp::ok()) {
                if (arm_safety.limits.speed_only)
                    RCLCPP_WARN(get_logger(), "Homing trajectory finished; arrival NOT verified (speed_only mode)");
                else
                    RCLCPP_INFO(get_logger(), "Homing complete: fresh feedback within %.6f rad and |v|<=0.05 rad/s for 0.2s",
                                ZERO_POS_THRESHOLD);
            }
        }else if (arm_safety.fault.empty()){
            // Even the no-motion path needs sustained, fresh at-target feedback.
            while (rclcpp::ok() && arm_safety.fault.empty()) {
                EtherCATrun();
                msg_publish();
                executor_->spin_some();
                const auto result = homing_monitor.update(arm_safety, arm_safety_now(), true);
                if (result == HomingMonitor::Result::finished_unverified) {
                    RCLCPP_WARN(get_logger(), "Homing skipped near target; settled position NOT verified (speed_only mode)");
                    break;
                }
                if (result == HomingMonitor::Result::reached) {
                    RCLCPP_INFO(get_logger(), "Already home: fresh stationary feedback confirmed for 0.2s");
                    break;
                }
                if (result == HomingMonitor::Result::fault) break;
                sleep_while_running(loop_rate);
            }
        }
        if (!rclcpp::ok()) arm_safety.trip("homing_interrupted");
        if (!arm_safety.fault.empty()) {
            // Send damping immediately, including faults discovered after the last exchange.
            EtherCATrun();
            RCLCPP_ERROR(get_logger(), "Homing aborted; zero position NOT confirmed: %s", arm_safety.fault.c_str());
        }
    }
    if (!arm_zero_calibration_ && arm_safety.fault.empty() && rclcpp::ok()) {
        homing_handoff_.begin(servo_desMsg_orig, arm_safety);
        ifRunin = false;
        ifRuninerInited = false;
        ifFirstRunin = true;
        trajectory_sample_ = false;
        posOutpFtr = runinOutpFtr{};
        velOutpFtr = runinOutpFtr{};
        TorOutpFtr = runinOutpFtr{};
        RCLCPP_INFO(get_logger(), "Homing handoff: holding last output until first upstream command; limited joints takeover speed<=%.3f rad/s, acceleration<=%.3f rad/s^2, gains/feedforward ramped",
                    std::min(homing_handoff_.limits.max_speed, arm_safety.limits.max_speed),
                    homing_handoff_.limits.max_acceleration);
    }
    startup_homing_ = false;

    //-----------------------------------------------
    //开启ros2循环
    while (rclcpp::ok()){
        ecatcheck();

        if (arm_zero_calibration_) {
            apply_arm_zero_calibration_commands(
                left_arm_calibration_hold_positions_,
                right_arm_calibration_hold_positions_
            );
            msg_publish();
            gripper_fdb_publish();
            EtherCATrun();
            executor_->spin_some();
            sleep_while_running(loop_rate);
            continue;
        }

        //发送一轮反馈值
        msg_publish();

        //发送夹爪反馈值
        gripper_fdb_publish();

        //获取一轮控制量
        ctlmsg_get();

        //-----轨迹计算-----
        //解包、插补并计算 -- 待修改：在插补过程中添加归零模式和阻尼回零模式
        ctlMsg_runin();

        //-----轨迹装入-----

        //数据处理过程量定义

        //位置量、速度量初始值
        std::vector<double> lArm_posOrig, rArm_posOrig;
        std::vector<double> lArm_velOrig, rArm_velOrig;
        std::vector<double> lArm_TorOrig, rArm_TorOrig;

        //滤波结果
        std::vector<std::vector<double>> pos_ftrRes;
        std::vector<std::vector<double>> vel_ftrRes;
        std::vector<std::vector<double>> Tor_ftrRes;

        //初始化
        lArm_posOrig.assign(LARMDOF, 0.0);
        rArm_posOrig.assign(RARMDOF, 0.0);
        lArm_velOrig.assign(LARMDOF, 0.0);
        rArm_velOrig.assign(RARMDOF, 0.0);
        lArm_TorOrig.assign(LARMDOF, 0.0);
        rArm_TorOrig.assign(RARMDOF, 0.0);
        pos_ftrRes.resize(2);
        pos_ftrRes[0].assign(LARMDOF, 0.0);
        pos_ftrRes[1].assign(RARMDOF, 0.0);
        vel_ftrRes.resize(2);
        vel_ftrRes[0].assign(LARMDOF, 0.0);
        vel_ftrRes[1].assign(RARMDOF, 0.0);
        Tor_ftrRes.resize(2);
        Tor_ftrRes[0].assign(LARMDOF, 0.0);
        Tor_ftrRes[1].assign(RARMDOF, 0.0);

        // Grippers can run even before the arm receives its first command.
        if(ifGetGripperMsg){
            ifGetGripperMsg = false;
            gripperTrajRunin();
        }

        //获取本轮插补值
        if (trajectory_sample_) {
            // Preserve the sampled trajectory's q/v pair, including on startup.
            for (int i = 0; i < 7; ++i) {
                auto& l = servo_desMsg_orig[i];
                auto& r = servo_desMsg_orig[i + 7];
                l.transMod = lArm_transMod[i]; r.transMod = rArm_transMod[i];
                l.joint_pos = lArm_pos_receive[i]; r.joint_pos = rArm_pos_receive[i];
                l.joint_spd = lArm_vel_receive[i]; r.joint_spd = rArm_vel_receive[i];
                l.joint_tor = lArm_torComp_grav_receive[i] + lArm_torComp_interia_receive[i] + lArm_torComp_feedforward_receive[i];
                r.joint_tor = rArm_torComp_grav_receive[i] + rArm_torComp_interia_receive[i] + rArm_torComp_feedforward_receive[i];
                l.joint_Kp = lArm_Kp_receive[i]; r.joint_Kp = rArm_Kp_receive[i];
                l.joint_Kd = lArm_Kd_receive[i]; r.joint_Kd = rArm_Kd_receive[i];
            }
        } else if(ifRuninerInited){    // --- 若进行了插补的操作 ---
            //-----获取初始轨迹-----
            outpTraj_process();

            //-----数据处理以及补偿量计算-----

            //滤波输入量装入
            for(int i = 0; i < LARMDOF; i++){
                lArm_posOrig[i] = servo_desMsg_orig[i].joint_pos;
                lArm_velOrig[i] = servo_desMsg_orig[i].joint_spd;
                lArm_TorOrig[i] = servo_desMsg_orig[i].joint_tor;
            }
            for(int i = 0; i < RARMDOF; i++){
                rArm_posOrig[i] = servo_desMsg_orig[i + LARMDOF].joint_pos;
                rArm_velOrig[i] = servo_desMsg_orig[i + LARMDOF].joint_spd;
                rArm_TorOrig[i] = servo_desMsg_orig[i + LARMDOF].joint_tor;
            }

            //滤波计算
            pos_ftrRes = command_output_kalman_enabled_ ? posOutpFtr.Outp_ftr(
                lArm_posOrig, rArm_posOrig
            ) : std::vector<std::vector<double>>{lArm_posOrig, rArm_posOrig};

            for(int i = 0; i < LARMDOF; i++){
                servo_desMsg_orig[i].joint_pos = pos_ftrRes[0][i];
            }
            for(int i = 0; i < RARMDOF; i++){
                servo_desMsg_orig[i + LARMDOF].joint_pos = pos_ftrRes[1][i];
            }

            vel_ftrRes = command_output_kalman_enabled_ ? velOutpFtr.Outp_ftr(
                lArm_velOrig, rArm_velOrig
            ) : std::vector<std::vector<double>>{lArm_velOrig, rArm_velOrig};
            for(int i = 0; i < LARMDOF; i++){
                servo_desMsg_orig[i].joint_spd = vel_ftrRes[0][i];
            }
            for(int i = 0; i < RARMDOF; i++){
                servo_desMsg_orig[i + LARMDOF].joint_spd = vel_ftrRes[1][i];
            }

            Tor_ftrRes = TorOutpFtr.Outp_ftr(
                lArm_TorOrig, rArm_TorOrig
            );
            for(int i = 0; i < LARMDOF; i++){
                servo_desMsg_orig[i].joint_tor = Tor_ftrRes[0][i];
            }
            for(int i = 0; i < RARMDOF; i++){
                servo_desMsg_orig[i + LARMDOF].joint_tor = Tor_ftrRes[1][i];
            }

            //刚度补偿 -- 暂时不做

            //摩擦补偿 -- 暂时不做

        }else{
            for(int i = 0; i < LARMDOF; i++){
                servo_desMsg_orig[i].transMod = 1;

                servo_desMsg_orig[i].joint_pos = lArm_zeroPos[i];
                servo_desMsg_orig[i].joint_spd = 0.0;
                servo_desMsg_orig[i].joint_tor = 0.0;

                servo_desMsg_orig[i].joint_Kp = 0.0;
                servo_desMsg_orig[i].joint_Kd = arm_safety.limits.damping;
            }

            for(int i = 0; i < RARMDOF; i++){
                servo_desMsg_orig[i + LARMDOF].transMod = 1;

                servo_desMsg_orig[i + LARMDOF].joint_pos = rArm_zeroPos[i];
                servo_desMsg_orig[i + LARMDOF].joint_spd = 0.0;
                servo_desMsg_orig[i + LARMDOF].joint_tor = 0.0;

                servo_desMsg_orig[i + LARMDOF].joint_Kp = 0.0;
                servo_desMsg_orig[i + LARMDOF].joint_Kd = arm_safety.limits.damping;
            }
        }

        const bool was_handoff = homing_handoff_.transitioning();
        homing_handoff_.apply(servo_desMsg_orig, arm_safety, TEMPTIME);
        if (was_handoff && !homing_handoff_.transitioning() && arm_safety.fault.empty())
            RCLCPP_INFO(get_logger(), "Controller handoff complete: command position/velocity/gains aligned; physical arrival not implied");

        //状态监测 -- 输出底层状态

        //辅助量赋值
        std::vector<double> lArm_joint_tor, rArm_joint_tor;

        std::vector<double> lArm_pos_ret, rArm_pos_ret;
        std::vector<double> lArm_vel_ret, rArm_vel_ret;
        std::vector<double> lArm_current_ret, rArm_current_ret;
        std::vector<double> lArm_temperature, rArm_temperature;
        std::vector<double> l_Kp, r_Kp, l_Kd, r_Kd;

        lArm_joint_tor.assign(LARMDOF, 0.0);    rArm_joint_tor.assign(RARMDOF, 0.0);

        lArm_pos_ret.assign(LARMDOF, 0.0);      rArm_pos_ret.assign(RARMDOF, 0.0);
        lArm_vel_ret.assign(LARMDOF, 0.0);      rArm_vel_ret.assign(RARMDOF, 0.0);
        lArm_current_ret.assign(LARMDOF, 0.0);  rArm_current_ret.assign(RARMDOF, 0.0);
        lArm_temperature.assign(LARMDOF, 0.0);  rArm_temperature.assign(RARMDOF, 0.0);
        l_Kp.assign(LARMDOF, 0.0);              r_Kp.assign(RARMDOF, 0.0);
        l_Kd.assign(LARMDOF, 0.0);              r_Kd.assign(RARMDOF, 0.0);

        for(int i = 0; i < LARMDOF; i++){
            lArm_joint_tor[i] = servo_desMsg_orig[i].joint_tor;

            lArm_pos_ret[i] = servo_retMsg_unpacked[i].joint_posRet;
            lArm_vel_ret[i] = servo_retMsg_unpacked[i].joint_spdRet;
            lArm_current_ret[i] = servo_retMsg_unpacked[i].joint_currentRet;
            lArm_temperature[i] = servo_retMsg_unpacked[i].joint_temperature;

            l_Kp[i] = servo_desMsg_orig[i].joint_Kp;
            l_Kd[i] = servo_desMsg_orig[i].joint_Kd;
        };
        for(int i = 0; i < RARMDOF; i++){
            rArm_joint_tor[i] = servo_desMsg_orig[i + LARMDOF].joint_tor;

            rArm_pos_ret[i] = servo_retMsg_unpacked[i + LARMDOF].joint_posRet;
            rArm_vel_ret[i] = servo_retMsg_unpacked[i + LARMDOF].joint_spdRet;
            rArm_current_ret[i] = servo_retMsg_unpacked[i + LARMDOF].joint_currentRet;
            rArm_temperature[i] = servo_retMsg_unpacked[i + LARMDOF].joint_temperature;

            r_Kp[i] = servo_desMsg_orig[i + LARMDOF].joint_Kp;
            r_Kd[i] = servo_desMsg_orig[i + LARMDOF].joint_Kd;
        };

        auto m_now = this->now();
        double m_timeSpot = m_now.seconds() + m_now.nanoseconds() * 1e-9;
        performance_test->comprehensive_msg_pub->ctl_res_pub_PVTmod(
            lArm_posOrig, rArm_posOrig,
            pos_ftrRes[0], pos_ftrRes[1],
            lArm_velOrig, rArm_velOrig,
            vel_ftrRes[0], vel_ftrRes[1],
            lArm_joint_tor, rArm_joint_tor,

            l_Kp, r_Kp,
            l_Kd, r_Kd,

            lArm_pos_ret, rArm_pos_ret,
            lArm_vel_ret, rArm_vel_ret,
            lArm_current_ret, rArm_current_ret,

            m_timeSpot
        );


        //刷新逻辑（未使用）
        if(has_received_command_){
            has_received_command_ = false;
        }else{

        }

        //发送夹爪指令
        gripperTorOutp();

        //----------------------
        //debug
        std::vector<double> lArm_pos, rArm_pos;
        std::vector<double> lArm_vel, rArm_vel;
        std::vector<double> lArm_tor, rArm_tor;

        lArm_pos.assign(LARMDOF, 0.0);  rArm_pos.assign(RARMDOF, 0.0);
        lArm_vel.assign(LARMDOF, 0.0);  rArm_vel.assign(RARMDOF, 0.0);
        lArm_tor.assign(LARMDOF, 0.0);  rArm_tor.assign(RARMDOF, 0.0);
        for(int i = 0; i < LARMDOF; i++){
            lArm_pos[i] = servo_desMsg_orig[i].joint_pos;
            lArm_vel[i] = servo_desMsg_orig[i].joint_spd;
            lArm_tor[i] = servo_desMsg_orig[i].joint_tor;
        }
        for(int i = 0; i < RARMDOF; i++){
            rArm_pos[i] = servo_desMsg_orig[i + LARMDOF].joint_pos;
            rArm_vel[i] = servo_desMsg_orig[i + LARMDOF].joint_spd;
            rArm_tor[i] = servo_desMsg_orig[i + LARMDOF].joint_tor;
        }
        global_debugPub->debug_msg_publish_1(lArm_pos, rArm_pos);
        global_debugPub->debug_msg_publish_2(lArm_vel, rArm_vel);
        global_debugPub->debug_msg_publish_3(lArm_tor, rArm_tor);

        //----------------------
        //进行一轮EtherCAT发送
        EtherCATrun();

        //尾处理代码
        executor_->spin_some();
        sleep_while_running(loop_rate);
    }

    //-----------------------------------------------
    RCLCPP_INFO(this->get_logger(), "程序已安全退出");
}

//目标值包装协议
void ServoNode::desMsg_pack_func(){
    /**
    int temp = 0;
    for(int i = 0; i < servo_CAN_Jnt_arrange.CAN_board_num; i++){
        for(int j = 0; j < servo_CAN_jnt_arrange.Jnt_num_on_each_board[i]; j++){
            servo_desMsg[i][j] = servo_desMsg_orig[temp];
            temp += 1;
        }
    }
    /**/

    // ECAT-8CAN: linear mapping, skip gripper (CAN ID 8)
    int temp = 0;
    for(int i = 0; i < servo_CAN_Jnt_arrange.CAN_board_num; i++){
        for(int j = 0; j < servo_CAN_Jnt_arrange.Jnt_num_on_each_board[i]; j++){
            if(ServoStruct.CAN_boards[i].can_jnt_ID[j] == 8) continue;
            servo_desMsg[i][j] = servo_desMsg_orig[temp];
            temp += 1;
        }
    }
}

//反馈值解包协议
void ServoNode::retMsg_unpack_func(){
    servo_retMsg_unpacked.clear();

    /**
    for(int i = 0; i < servo_CAN_Jnt_arrange.CAN_board_num; i++){
        for(int j = 0; j < servo_CAN_jnt_arrange.Jnt_num_on_each_board[i]; j++){
            servo_retMsg_unpacked.push_back(servo_retMsg[i][j]);
        }
    }
    /**/

    // ECAT-8CAN: linear unpacking, skip gripper (CAN ID 8)
    for(int i = 0; i < servo_CAN_Jnt_arrange.CAN_board_num; i++){
        for(int j = 0; j < servo_CAN_Jnt_arrange.Jnt_num_on_each_board[i]; j++){
            if(ServoStruct.CAN_boards[i].can_jnt_ID[j] == 8) continue;
            servo_retMsg_unpacked.push_back(servo_retMsg[i][j]);
        }
    }
}

//传输信息回调函数 -- 接收消息并执行插补任务
void ServoNode::message_callback(const ctl_msgs_hw::msg::CtlInformation::SharedPtr msg){
    if (startup_homing_) return;
    arm_safety.command_received(arm_safety_now());
    if (!arm_safety.fault.empty()) return;
    const auto valid = [](const auto& values) {
        return values.size() == 7 && std::all_of(values.begin(), values.end(),
            [](double v) { return std::isfinite(v); });
    };
    if (!std::isfinite(msg->temp_time) || msg->temp_time <= 0 || msg->temp_time > 0.25 ||
        !valid(msg->base_rigidity_ctl_mode_l) || !valid(msg->base_rigidity_ctl_mode_r) ||
        !valid(msg->l_position_at_runin_st) || !valid(msg->r_position_at_runin_st) ||
        !valid(msg->l_position) || !valid(msg->r_position) || !valid(msg->l_speed) || !valid(msg->r_speed) ||
        !valid(msg->l_torque_grav_comp) || !valid(msg->r_torque_grav_comp) ||
        !valid(msg->l_torque_interia_comp) || !valid(msg->r_torque_interia_comp) ||
        !valid(msg->l_torque_feedforward) || !valid(msg->r_torque_feedforward) ||
        !valid(msg->l_kp) || !valid(msg->r_kp) || !valid(msg->l_kd) || !valid(msg->r_kd)) {
        arm_safety.trip("invalid_controller_packet"); return;
    }
    for (size_t i = 0; !arm_safety.limits.speed_only && i < 7; ++i) {
        if (std::abs(msg->l_torque_grav_comp[i]) + std::abs(msg->l_torque_interia_comp[i]) +
                std::abs(msg->l_torque_feedforward[i]) > arm_safety.limits.max_feedforward ||
            std::abs(msg->r_torque_grav_comp[i]) + std::abs(msg->r_torque_interia_comp[i]) +
                std::abs(msg->r_torque_feedforward[i]) > arm_safety.limits.max_feedforward) {
            arm_safety.trip("raw_feedforward_limit"); return;
        }
    }

    //--------------------接收消息--------------------

    //检查时间戳是否合理（例如，延迟不超过 5ms）
    auto now = this->now();
    auto latency = now - msg->header.stamp;
    if (!arm_safety.limits.speed_only && (latency.seconds() > 0.1 || latency.seconds() < -0.05)) {
        arm_safety.trip("controller_packet_timestamp"); return;
    }
    if (latency.seconds() > 0.005) {
        RCLCPP_WARN(this->get_logger(), "High latency detected: %f seconds", latency.seconds());
    }

    const bool speed_override_before = arm_safety.policy_speed_override_active();
    arm_safety.policy_speed_override(
        msg->policy_speed_checks_disabled && msg->trajectory_header.frame_id == "pi05_policy",
        (now - msg->trajectory_header.stamp).seconds(), arm_safety_now());
    if (speed_override_before != arm_safety.policy_speed_override_active())
        RCLCPP_WARN(get_logger(), "Policy software speed checks: %s (source-scoped override)",
                    arm_safety.policy_speed_override_active() ? "disabled" : "restored");

    if (homing_handoff_.receive_epoch(msg->handoff_epoch, servo_desMsg_orig, arm_safety)) {
        ifFirstRunin = true;
        ifRuninerInited = false;
        posOutpFtr = runinOutpFtr{};
        velOutpFtr = runinOutpFtr{};
        TorOutpFtr = runinOutpFtr{};
        RCLCPP_INFO(get_logger(), "Controller handoff epoch=%llu: restarting transition from last emitted command",
                    static_cast<unsigned long long>(msg->handoff_epoch));
    }

    //模式控制变量赋值
    if (trajectory_sample_ != msg->trajectory_sample || msg->if_useret_to_start_runin) {
        ifFirstRunin = true;
        posOutpFtr = runinOutpFtr{};
        velOutpFtr = runinOutpFtr{};
        TorOutpFtr = runinOutpFtr{};
    }
    trajectory_sample_ = msg->trajectory_sample;
    tempTime_upperMonitor = msg->temp_time;
    arm_safety.command_period = msg->temp_time;
    runin_mod = msg->runin_mod;
    if_runin_renew = msg->if_runin_renew;
    msg_get_ifUseDebugMod = msg->if_use_debugmod;
    if_useret_to_start_runin = msg->if_useret_to_start_runin;

    //关节底层控制模式变量赋值
    for(int i = 0; i < LARMDOF; i++){
        lArm_transMod[i] = msg->base_rigidity_ctl_mode_l[i];
    }
    for(int i = 0; i < RARMDOF; i++){
        rArm_transMod[i] = msg ->base_rigidity_ctl_mode_r[i];
    }

    //左右臂插补位置初始化变量刷新
    for(int i = 0; i < LARMDOF; i++){
        l_posReceive_atRuninSt[i] = msg->l_position_at_runin_st[i];
    }
    for(int i = 0; i < RARMDOF; i++){
        r_posReceive_atRuninSt[i] = msg->r_position_at_runin_st[i];
    }

    //控制消息获取
    for(int i = 0; i < LARMDOF; i++){
        lArm_pos_receive[i] = msg->l_position[i];
        lArm_vel_receive[i] = msg->l_speed[i];

        lArm_torComp_grav_receive[i] = msg->l_torque_grav_comp[i];
        lArm_torComp_interia_receive[i] = msg->l_torque_interia_comp[i];
        lArm_torComp_feedforward_receive[i] = msg->l_torque_feedforward[i];

        lArm_Kp_receive[i] = msg->l_kp[i];
        lArm_Kd_receive[i] = msg->l_kd[i];
    }

    for(int i = 0; i < RARMDOF; i++){
        rArm_pos_receive[i] = msg->r_position[i];
        rArm_vel_receive[i] = msg->r_speed[i];

        rArm_torComp_grav_receive[i] = msg->r_torque_grav_comp[i];
        rArm_torComp_interia_receive[i] = msg->r_torque_interia_comp[i];
        rArm_torComp_feedforward_receive[i] = msg->r_torque_feedforward[i];

        rArm_Kp_receive[i] = msg->r_kp[i];
        rArm_Kd_receive[i] = msg->r_kd[i];
    }

    //--------------------标志位更新--------------------


    //更新插补启动标志
    ifRunin = true;

    //更新获取反馈值标志位
    has_received_command_ = true;
    last_source_header_ = msg->trajectory_header;
    if (homing_handoff_.accept_command()) {
        double max_target_delta = 0, max_feedback_error = 0;
        size_t target_joint = 0, feedback_joint = 0;
        for (size_t i = 0; i < 14; ++i) {
            const double held = homing_handoff_.position(i);
            const double target = i < 7 ? lArm_pos_receive[i] : rArm_pos_receive[i - 7];
            if (std::abs(target - held) > max_target_delta) {
                max_target_delta = std::abs(target - held); target_joint = i;
            }
            if (std::abs(held - arm_safety.feedback[i].position) > max_feedback_error) {
                max_feedback_error = std::abs(held - arm_safety.feedback[i].position); feedback_joint = i;
            }
        }
        RCLCPP_INFO(get_logger(), "Controller takeover started: max target change=%.6f rad (joint_%zu), max held/feedback error=%.6f rad (joint_%zu); indices zero-based",
                    max_target_delta, target_joint, max_feedback_error, feedback_joint);
    }
}

//函数2 -- 执行插补函数
void ServoNode::ctlMsg_runin(){

    if (trajectory_sample_) { ifRunin = false; return; }

    if(ifRunin){
        //更新插补启动标志
        ifRunin = false;

        //更新插补初始位置
        if (homing_handoff_.transitioning()) {
            for (int i = 0; i < 7; ++i) {
                l_posReceive_atRuninSt[i] = homing_handoff_.position(i);
                r_posReceive_atRuninSt[i] = homing_handoff_.position(i + 7);
            }
        } else if(if_useret_to_start_runin){
            //若采用反馈值作为插补初始值，则需要更新插补初始关键量
            for(int i = 0; i < LARMDOF; i++){
                l_posReceive_atRuninSt[i] = servo_retMsg_unpacked[i].joint_posRet;
            }
            for(int i = 0; i < RARMDOF; i++){
                r_posReceive_atRuninSt[i] = servo_retMsg_unpacked[i + LARMDOF].joint_posRet;
            }
        }

        //判断是否要更新插补
        bool if_runin_renew_bool = false;
        if(if_runin_renew == 1){
            if_runin_renew_bool = true;
        }
        bool if_temp_runin_renew = ifFirstRunin || if_runin_renew_bool;
        ifFirstRunin = false;

        //执行刷新代码
        if(if_temp_runin_renew){
            runin_control_core.init(
                l_posReceive_atRuninSt, r_posReceive_atRuninSt,
                20
            );
            ifRuninerInited = true;
        }

        temp_len = 0;

        //获取插补长度
        winlen = std::clamp(static_cast<int>(std::round(tempTime_upperMonitor / TEMPTIME)), 1, 100);

        // The configured command interpolator is independent of upstream modes.
        runin_control_core.ModelRunin(
            lArm_pos_receive,
            lArm_vel_receive,
            lArm_torComp_grav_receive,
            lArm_torComp_interia_receive,
            lArm_torComp_feedforward_receive,

            rArm_pos_receive,
            rArm_vel_receive,
            rArm_torComp_grav_receive,
            rArm_torComp_interia_receive,
            rArm_torComp_feedforward_receive,

            lArm_Kp_receive,
            lArm_Kd_receive,
            rArm_Kp_receive,
            rArm_Kd_receive,

            command_interpolation_mode_,
            winlen
        );
    }
}

//函数3 -- 数据处理与转换函数
void ServoNode::outpTraj_process(){
    //获取轨迹输出解包
    runinTrajOutp trajOutp = runin_control_core.dataOutp();

    //进行数据装入
    for(int i = 0; i < LARMDOF; i++){
        servo_desMsg_orig[i].transMod = lArm_transMod[i];

        servo_desMsg_orig[i].joint_pos = trajOutp.runinData.lArm_posDot[i];
        servo_desMsg_orig[i].joint_spd = trajOutp.runinData.lArm_velDot[i];

        servo_desMsg_orig[i].joint_tor = trajOutp.runinData.lArm_FeedForwardCompDot[i] + trajOutp.runinData.lArm_gravCompDot[i] + trajOutp.runinData.lArm_InteriaCompDot[i];
        
        servo_desMsg_orig[i].joint_Kp = trajOutp.lArmJnt_Kp[i];
        servo_desMsg_orig[i].joint_Kd = trajOutp.lArmJnt_Kd[i];
    }

    for(int i = 0; i < RARMDOF; i++){
        servo_desMsg_orig[i + LARMDOF].transMod = rArm_transMod[i];

        servo_desMsg_orig[i + LARMDOF].joint_pos = trajOutp.runinData.rArm_posDot[i];
        servo_desMsg_orig[i + LARMDOF].joint_spd = trajOutp.runinData.rArm_velDot[i];

        servo_desMsg_orig[i + LARMDOF].joint_tor = trajOutp.runinData.rArm_FeedForwardCompDot[i] + trajOutp.runinData.rArm_gravCompDot[i] + trajOutp.runinData.rArm_InteriaCompDot[i];

        servo_desMsg_orig[i + LARMDOF].joint_Kp = trajOutp.rArmJnt_Kp[i];
        servo_desMsg_orig[i + LARMDOF].joint_Kd = trajOutp.rArmJnt_Kd[i];
    }
}

//==================================================
//debug辅助函数

//debug函数1 -- 比对容器中直接获取的传输值与实际的传输值
void ServoNode::comp_desMsg_containerMsg(){
    std::vector<std::vector<joint_desMsg>> container_desMsg_ori = JntDesMsg_inContainer_Outp_v1();

    std::vector<joint_desMsg> container_desMsg_l, container_desMsg_r;

    for(int i = 0; i < container_desMsg_ori[1].size(); i++){
        container_desMsg_l.push_back(container_desMsg_ori[1][i]);
    }
    for(int i = 0; i < container_desMsg_ori[0].size(); i++){
        container_desMsg_l.push_back(container_desMsg_ori[0][i]);
    }

    for(int i = 0; i < container_desMsg_ori[3].size(); i++){
        container_desMsg_r.push_back(container_desMsg_ori[3][i]);
    }
    for(int i = 0; i < container_desMsg_ori[2].size(); i++){
        container_desMsg_r.push_back(container_desMsg_ori[2][i]);
    }

    //消息发送对比
    auto comp_pub = ctl_msgs_hw::msg::DbgInformation();
    for(int i = 0; i < 7; i++){
        comp_pub.float_reserve[i] = container_desMsg_l[i].joint_pos;
        comp_pub.float_reserve_b[i] = servo_desMsg_orig[i].joint_pos;
    }

    debugDataPub_comparison->publish(comp_pub);
}

//==================================================
//夹爪控制函数

//插补函数
void ServoNode::gripperTrajRunin(){
    // Interpolate from the last emitted setpoint; restarting from feedback on
    // every packet would repeatedly pull the target back under load.
    const auto positionStart = [](const gripperCmd& cmd, const gripperFdb& feedback, int channel){
        return cmd.joint_mode == 1
            ? gripperDireComp(channel) * cmd.joint_pos / 0.017453292519943295
            : feedback.pos_deg;
    };
    if(ctl_inteMsg_config.GripperLeft_Control.mode == 1){
        if(l_gripper_pos_received && left_gripper_fdb.valid){
            l_gripper_posTraj = gripper_pos_runiner.liner_runin(
                positionStart(left_gripper_cmd, left_gripper_fdb, 0), l_gripper_pos_get, winlen_gripper);
        }
    }else{
        l_gripper_torTraj = l_gripper_runiner.cubicSplineCal(
            gripper_LastTorRec_l, l_gripper_tor_get, winlen_gripper);
    }
    if(ctl_inteMsg_config.GripperRight_Control.mode == 1){
        if(r_gripper_pos_received && right_gripper_fdb.valid){
            r_gripper_posTraj = gripper_pos_runiner.liner_runin(
                positionStart(right_gripper_cmd, right_gripper_fdb, 1), r_gripper_pos_get, winlen_gripper);
        }
    }else{
        r_gripper_torTraj = r_gripper_runiner.cubicSplineCal(
            gripper_LastTorRec_r, r_gripper_tor_get, winlen_gripper);
    }

    gripper_Tor_trajLen = winlen_gripper;
    gripper_otputTimes = 0;

    gripper_LastTorRec_l = l_gripper_tor;
    gripper_LastTorRec_r = r_gripper_tor;
}

//插补结果输出函数
void ServoNode::gripperTorOutp(){
    const auto sample = [this](const std::vector<double>& trajectory){
        return trajectory[std::min(static_cast<size_t>(gripper_otputTimes), trajectory.size() - 1)];
    };
    const auto positionCommand = [&sample](const std::vector<double>& trajectory,
            const GripperControlConfig& config, const gripperFdb& feedback,
            gripper_RangeLimit& limit, int channel){
        if(trajectory.empty()){
            return gripperCmd{};
        }
        return limit.PositionOutp(sample(trajectory), config, gripperDireComp(channel), feedback.temperature);
    };

    gripperCmd left, right;
    if(ctl_inteMsg_config.GripperLeft_Control.mode == 1){
        left = positionCommand(l_gripper_posTraj, ctl_inteMsg_config.GripperLeft_Control,
                               left_gripper_fdb, l_gripper_limit, 0);
    }else{
        l_gripper_tor = sample(l_gripper_torTraj);
        left.joint_tor = l_gripper_limit.LimitOutp(l_gripper_tor, left_gripper_fdb.temperature,
            left_gripper_fdb.current_A * ctl_inteMsg_config.GripperLeft_Kt);
    }
    if(ctl_inteMsg_config.GripperRight_Control.mode == 1){
        right = positionCommand(r_gripper_posTraj, ctl_inteMsg_config.GripperRight_Control,
                                right_gripper_fdb, r_gripper_limit, 1);
    }else{
        r_gripper_tor = sample(r_gripper_torTraj);
        right.joint_tor = r_gripper_limit.LimitOutp(r_gripper_tor, right_gripper_fdb.temperature,
            right_gripper_fdb.current_A * ctl_inteMsg_config.GripperRight_Kt);
    }
    setLeftGripperCmd(left);
    setRightGripperCmd(right);
    // Windows are bounded at 100; saturating also avoids overflow while holding.
    gripper_otputTimes = std::min(gripper_otputTimes + 1, 100);
}


//夹爪反馈值发布 (topic: /gripper/fdb)
//data: [left_pos_deg, left_spd_rad, left_current_A, left_temperature,
//       right_pos_deg, right_spd_rad, right_current_A, right_temperature]
void ServoNode::gripper_fdb_publish(){
    auto msg = std_msgs::msg::Float64MultiArray();
    msg.data.resize(8);
    msg.data[0] = left_gripper_fdb.valid ? left_gripper_fdb.pos_deg : std::numeric_limits<double>::quiet_NaN();
    msg.data[1] = left_gripper_fdb.spd_rad;
    msg.data[2] = left_gripper_fdb.current_A;
    msg.data[3] = left_gripper_fdb.temperature;
    msg.data[4] = right_gripper_fdb.valid ? right_gripper_fdb.pos_deg : std::numeric_limits<double>::quiet_NaN();
    msg.data[5] = right_gripper_fdb.spd_rad;
    msg.data[6] = right_gripper_fdb.current_A;
    msg.data[7] = right_gripper_fdb.temperature;
    gripper_fdb_pub->publish(msg);
}

//夹爪目标控制量接收
void ServoNode::gripper_desMsg_get(const ctl_msgs_hw::msg::GripperCtl::SharedPtr msg){
    if (startup_homing_) return;
    if (!arm_safety.fault.empty()) return;
    l_gripper_tor_get = std::isfinite(msg->l_arm_gripper_tor) ? msg->l_arm_gripper_tor : 0.0;
    r_gripper_tor_get = std::isfinite(msg->r_arm_gripper_tor) ? msg->r_arm_gripper_tor : 0.0;

    const auto& left = ctl_inteMsg_config.GripperLeft_Control;
    const auto& right = ctl_inteMsg_config.GripperRight_Control;
    const double left_target = left.zero_pos_deg + msg->l_arm_gripper_pos;
    const double right_target = right.zero_pos_deg + msg->r_arm_gripper_pos;
    if(std::isfinite(left_target) && left_gripper_fdb.valid){
        l_gripper_pos_get = std::clamp(left_target, left.pos_min_deg, left.pos_max_deg);
        l_gripper_pos_received = true;
    }
    if(std::isfinite(right_target) && right_gripper_fdb.valid){
        r_gripper_pos_get = std::clamp(right_target, right.pos_min_deg, right.pos_max_deg);
        r_gripper_pos_received = true;
    }

    //获取时间戳并计算窗口长度
    if(if_gripper_firstGet){
        rclcpp::Time current_time = this->now();
        time_res_gripper = current_time.seconds();
        time_last_gripper = current_time.seconds();
        const double period = std::isfinite(msg->ctl_time) ? msg->ctl_time : TEMPTIME;
        winlen_gripper = static_cast<int>(std::clamp(std::round(period / TEMPTIME), 1.0, 100.0));

        if_gripper_firstGet = false;
    }else{
        rclcpp::Time current_time = this->now();
        time_res_gripper = current_time.seconds();

        winlen_gripper = static_cast<int>(std::clamp(
            std::round((time_res_gripper - time_last_gripper)/TEMPTIME), 1.0, 100.0));
        time_last_gripper = time_res_gripper;
    }

    ifGetGripperMsg = true;
}
