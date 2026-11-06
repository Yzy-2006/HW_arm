#include "dual_arm_control/msgPublish.h"
#include <ament_index_cpp/get_package_share_directory.hpp>

// 全局变量（定义，不在此初始化 —— 初始化在 rosloop_var_init() 中完成）

//--------------------求解器关键变量--------------------

double TEMPTIME;
int ALLJNTNUM;
int LARMDOF;
int RARMDOF;

//加载机械臂urdf文件路径
std::vector<std::string> path_options;
std::vector<std::string> base_link;
std::vector<std::string> tip_link;


//---------------中层节点与下位机通讯全局变量---------------

//控制量发送：
int runin_mod;
int if_runin_renew;
int ifUseRet_toStRunin;
int ifUseDebugMod;
std::vector<int> base_rigidityCtl_mod_l;
std::vector<int> base_rigidityCtl_mod_r;

//左右臂初始值刷新
std::vector<double> lArm_pos_atRuninSt;
std::vector<double> rArm_pos_atRuninSt;

//目标参量发送：
std::vector<double> lArm_joint_pos;
std::vector<double> lArm_joint_kp;
std::vector<double> lArm_joint_kd;
std::vector<double> lArm_joint_spd;

std::vector<double> lArm_joint_tor_grav;
std::vector<double> lArm_joint_tor_interia;
std::vector<double> lArm_joint_tor_feedforward;

std::vector<double> rArm_joint_pos;
std::vector<double> rArm_joint_kp;
std::vector<double> rArm_joint_kd;
std::vector<double> rArm_joint_spd;

std::vector<double> rArm_joint_tor_grav;
std::vector<double> rArm_joint_tor_interia;
std::vector<double> rArm_joint_tor_feedforward;

//控制状态向上位机反馈：
std::vector<bool> lArm_ifReachEnd;
std::vector<bool> rArm_ifReachEnd;

//---------------中层节点与上位机通讯全局变量---------------

//-----控制信息-----

//综合控制信息
modCtl modCtlMsg_get;

//程序打印控制参数
optiPrintMsgCtl printMsgPkg_get;

//优化器参数打包
optimizerCtl optimizerPara_get;

//关节限制信息打包
JntLimMsg lArm_LimMsg_get;
JntLimMsg rArm_LimMsg_get;
JntLimMsg allArm_LimMsg_get;

//控制过程信息打包
bool ifReInit_get;
bool ifUpdateLim_get;
bool ifRedefineJntTrajStab_get;


//-----目标位置等信息-----
std::vector<double> lArm_desPos_descartes;
std::vector<double> lArm_desPos_JntSpace;


std::vector<double> rArm_desPos_descartes;
std::vector<double> rArm_desPos_JntSpace;


//适配MDP接口的速度传输变量
std::vector<double> lArm_desVel_MDP_JntSpace;
std::vector<double> rArm_desVel_MDP_JntSpace;

//搭配自定义PD参数模式使用 -- PD参数接收模块
std::vector<double> lArm_Kp_;
std::vector<double> lArm_Kd_;
std::vector<double> rArm_Kp_;
std::vector<double> rArm_Kd_;

//-----------------------------------------------------

//反馈接收：
std::vector<double> lArm_pos_fdback;
std::vector<double> lArm_spd_fdback;
std::vector<double> lArm_iq_fdback;

std::vector<double> rArm_pos_fdback;
std::vector<double> rArm_spd_fdback;
std::vector<double> rArm_iq_fdback;

//反馈接收 -- 基座位姿
std::vector<double> waist_desc_pos;
std::vector<double> waist_desc_spd;

int if_lower_firstTime;
int if_middle_firstTime;

//debug用的辅助变量
std::vector<double> vec_dbg_1;
std::vector<double> vec_dbg_2;

//===================================================================================
void signalHandler(int sig) {
    shutdown_flag = 1;
}
bool is_key_pressed(int timeout_ms) {
    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(STDIN_FILENO, &fds);
    struct timeval timeout = {0, timeout_ms * 1000};
    return select(STDIN_FILENO + 1, &fds, nullptr, nullptr, &timeout) > 0;
}

//============================================================
//调试用辅助函数
std::vector<double> JntPDoutpDebug(double posDes, double velDes, double posRet, double Kp, double Kd)
{
    std::vector<double> mRes;
    double mRes_all = (posDes - posRet) * Kp + velDes * Kd;
    double mRes_kp = (posDes - posRet) * Kp;
    double mRes_kd = velDes * Kd;
    mRes.push_back(mRes_all);
    mRes.push_back(mRes_kp);
    mRes.push_back(mRes_kd);

    return mRes;
}

//===================================================================================

//构造函数
ArmCtlNode::ArmCtlNode() : Node("dual_arm_control_node") , 
    has_received_command_(false),
    has_received_upperMonitor_command_(false)
{
    yaml_path_gravCompConfig = ament_index_cpp::get_package_share_directory("ctl_msgs_hw") + "/config/controllerConfig.yaml";

    //初始化机器人数据传输流程参数 — 必须最先加载，后续代码依赖 dataTransLoopCtl 的配置值
    std::cout << " ===== 初始化机器人数据传输流程参数 ===== " << std::endl;
    if(!loadTransLoopCtlFromYaml(yaml_path_gravCompConfig)){
        std::cerr << "错误：controllerConfig.yaml 加载失败，无法获取控制流程参数" << std::endl;
        return;
    }

    //配置全局变量 — 必须在 loadTransLoopCtlFromYaml 之后
    TEMPTIME = dataTransLoopCtl.midNode_tempTime;
    LARMDOF = dataTransLoopCtl.lArm_Dof;
    RARMDOF = dataTransLoopCtl.rArm_Dof;
    ALLJNTNUM = LARMDOF + RARMDOF;


    std::string urdf_dir = ament_index_cpp::get_package_share_directory("dual_arm_hw") + "/urdf/";
    path_options.resize(1);
    path_options[0] = urdf_dir + dataTransLoopCtl.urdf_path_option;
    base_link.resize(2);
    base_link[0] = dataTransLoopCtl.base_link_lArm;
    base_link[1] = dataTransLoopCtl.base_link_rArm;
    tip_link.resize(2);
    tip_link[0] = dataTransLoopCtl.tip_link_lArm;
    tip_link[1] = dataTransLoopCtl.tip_link_rArm;

    std::cout << path_options[0] << std::endl;
    std::string urdf_path = path_options[0];
    std::ifstream urdf_file(urdf_path);

    //初始化机器人配置代码
    yaml_path_zeroPos = ament_index_cpp::get_package_share_directory("ctl_msgs_hw") + "/config/zeropos.yaml";
    yaml_path_desc_zeroPos = ament_index_cpp::get_package_share_directory("ctl_msgs_hw") + "/config/desc_zeropos.yaml";
    yaml_path_initialPD = ament_index_cpp::get_package_share_directory("ctl_msgs_hw") + "/config/initialPD.yaml";
    yaml_path_jointLink = ament_index_cpp::get_package_share_directory("ctl_msgs_hw") + "/config/jointLink_config.yaml";
    yaml_path_robotStructConfig = ament_index_cpp::get_package_share_directory("ctl_msgs_hw") + "/config/robot_config.yaml";
    std::cout << " ===== 初始化关节层零位配置 ===== " << std::endl;
    loadZeroPosFromYaml(yaml_path_zeroPos);
    std::cout << " ===== 初始化笛卡尔空间零位配置 ===== " << std::endl;
    loadDescZeroPosFromYaml(yaml_path_desc_zeroPos);
    std::cout << " ===== 初始化PD参数配置 ===== " << std::endl;
    loadInitialPDFromYaml(yaml_path_initialPD);
    std::cout << " ===== 初始化关节连杆结构配置 ===== " << std::endl;
    loadJointLinkFromYaml(yaml_path_jointLink);

     //初始化机器人逆解求解器
    std::cout << " ===== 初始化机器人求解器配置 ===== " << std::endl;
    if(!loadSolverControlFromYaml(yaml_path_gravCompConfig)){
        std::cerr << "警告：controllerConfig.yaml 加载失败，逆解求解器将无法工作" << std::endl;
    }

    // 初始化机器人模型
    std::cout << " ===== 初始化机器人重力补偿配置 ===== " << std::endl;
    if(!loadGravCompFromYaml(yaml_path_gravCompConfig)){
        std::cerr << "警告：controllerConfig.yaml 加载失败，重力补偿将无法工作" << std::endl;
    }

    // 初始化机器人结构模型 — 从 robot_config.yaml + dual_arm.urdf 构建，填充 robotModelStructConfig
    std::cout << " ===== 初始化机器人结构模型 ===== " << std::endl;
    if(!loadRobotConfigFromYaml(yaml_path_robotStructConfig, urdf_path)){
        std::cerr << "警告：robot_config.yaml 加载失败，机器人结构模型将为空" << std::endl;
    }

    //--------------------------------------------------------------------

    //加载urdf文件
    try {
        RCLCPP_INFO(this->get_logger(), "URDF directory found: %s", path_options[0].c_str());
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error finding URDF directory: %s", e.what());
        return;
    }

    //读取urdf文件内容
    if (!urdf_file.is_open()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open URDF file: %s", urdf_path.c_str());
        //rclcpp::shutdown();
        return;
    }

    robot_description_ = std::string((std::istreambuf_iterator<char>(urdf_file)), std::istreambuf_iterator<char>());
    if (robot_description_.empty()) {
        RCLCPP_ERROR(this->get_logger(), "URDF file is empty!");
        //rclcpp::shutdown();
        return;
    }

    //加载机器人模型
    robot_model_ = std::make_shared<urdf::Model>();
    if (!robot_model_->initString(robot_description_)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to parse URDF model");
        //rclcpp::shutdown();
        return;
    }

    //初始化KDL树
    if (!kdl_parser::treeFromString(robot_description_, kdl_tree_)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to construct KDL tree");
        return;
    }

    //提取KDL链
    if (!kdl_tree_.getChain(base_link[0], tip_link[0], kdl_lArm_chain_)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to extract Left Arm KDL chain");
        return;
    }
    if(!kdl_tree_.getChain(base_link[1], tip_link[1], kdl_rArm_chain_)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to extract Right Arm KDL chain");
        return;
    }

    // 初始化 RobotStatePublisher
    rclcpp::NodeOptions options;
    options.parameter_overrides({
        {"robot_description", robot_description_},
        {"publish_frequency", 1000.0}  // 与循环频率匹配
    });
    robot_state_publisher_ = std::make_shared<robot_state_publisher::RobotStatePublisher>(options);

    // 初始化TF广播器
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // 服务：允许外部修改关节角度
    set_joint_service_ = this->create_service<std_srvs::srv::SetBool>(
        "set_joint_angles",
        std::bind(&ArmCtlNode::set_joint_angles, this, std::placeholders::_1, std::placeholders::_2));

    //初始化发布与接受的消息类型

    mCtlData_pub = this->create_publisher<ctl_msgs_hw::msg::CtlInformation>(
        "/dexbotCtl/ctl_data_pub",
        10
    );
    mRetData = this->create_subscription<ctl_msgs_hw::msg::RetInformation>(
        "/dexbotCtl/dualArmCtl_fdback",			// 订阅的话题（必须与发布者一致)
        10,								// QoS历史深度
        std::bind(&ArmCtlNode::message_callback, this, std::placeholders::_1)
    );

    upperMonitorMsg_get = this->create_subscription<ctl_msgs_hw::msg::MidNodeCtlMsgSend>(
        "/dexbotCtl/upperMonitor_msg_send",
        10,
        std::bind(&ArmCtlNode::upperMonitor_msg_callback, this, std::placeholders::_1, std::placeholders::_2)
    );
    upperMonitorMsg_send = this->create_publisher<ctl_msgs_hw::msg::MidNodeCtlMsgRet>(
        "/dexbotCtl/midMonitor_msg_send",
        10
    );

    mFkDesRes_pub = this->create_publisher<ctl_msgs_hw::msg::FkDataPub>(
        "/dexbotCtl/DescPub/Fk_des_data_pub",
        10
    );
    mFkRetRes_pub = this->create_publisher<ctl_msgs_hw::msg::FkDataPub>(
        "/dexbotCtl/DescPub/Fk_ret_data_pub",
        10
    );

    //初始化画图代码
    endPosTrajPlot = new EndPos_plot(this);

    //初始化debug发布器
    global_debugPub = new debugMsgPub(this);

    //-----------------------------------------
    //过程控制变量初始化

    //初始化控制变量
    ifInit = true;
    ifCheckTemp = true;

    //重力补偿控制变量
    ifUse_gravComp = robotGravCompInte.ifUseGravComp ||
        robotGravCompInte.ifUseInertiaComp || robotGravCompInte.ifUseCoriolisComp;
    gravInpMode = robotGravCompInte.InpDataMod;

    ifGravCompInit = true;

    ctl_JntStStabTimes = robotGravCompInte.JntCtlStStabTimes;
    ctl_GravCompStZeroTimes = robotGravCompInte.GravCompStZeroTimes;
    ctl_GravCompStSmoothTimes = robotGravCompInte.GravCompStSmoothTimes;

    recent_tempTimes = 0; 

}


//------------------------------
//控制参量发送模块
void ArmCtlNode::msg_publish()
{
    auto tempCtlData_pub = ctl_msgs_hw::msg::CtlInformation();
    tempCtlData_pub.header.stamp = this->now();  // 添加当前时间戳


    tempCtlData_pub.temp_time = TEMPTIME;   //发送迭代时间
    tempCtlData_pub.trajectory_sample = stream_sample_ && stream_sample_->derivatives_valid;
    tempCtlData_pub.handoff_epoch = control_handoff_.value();
    if (stream_sample_) tempCtlData_pub.trajectory_header = stream_sample_->trajectory_header;
    tempCtlData_pub.policy_speed_checks_disabled = stream_sample_ &&
        stream_sample_->policy_speed_checks_disabled &&
        stream_sample_->trajectory_header.frame_id == "pi05_policy" && modCtlMsg_get.transMod == 3;

    //发送控制量
    tempCtlData_pub.runin_mod = runin_mod;
    tempCtlData_pub.if_runin_renew = if_runin_renew;
    tempCtlData_pub.if_useret_to_start_runin = ifUseRet_toStRunin;
    tempCtlData_pub.if_use_debugmod = ifUseDebugMod;

    tempCtlData_pub.base_rigidity_ctl_mode_l.resize(LARMDOF);
    tempCtlData_pub.base_rigidity_ctl_mode_r.resize(RARMDOF);
    for(int i = 0; i < LARMDOF; i++){
        tempCtlData_pub.base_rigidity_ctl_mode_l[i] = base_rigidityCtl_mod_l[i];
    }
    for(int i = 0; i < RARMDOF; i++){
        tempCtlData_pub.base_rigidity_ctl_mode_r[i] = base_rigidityCtl_mod_r[i];
    }

    //发送左右臂初始值刷新
    tempCtlData_pub.l_position_at_runin_st.resize(LARMDOF);
    tempCtlData_pub.r_position_at_runin_st.resize(RARMDOF);
    for(int i = 0; i < LARMDOF; i++){
        tempCtlData_pub.l_position_at_runin_st[i] = lArm_pos_atRuninSt[i];
    }
    for(int i = 0; i < RARMDOF; i++){
        tempCtlData_pub.r_position_at_runin_st[i] = rArm_pos_atRuninSt[i];
    }

    //发送控制来唵
    tempCtlData_pub.l_position.resize(LARMDOF);
    tempCtlData_pub.l_speed.resize(LARMDOF);
    tempCtlData_pub.l_torque_grav_comp.resize(LARMDOF);
    tempCtlData_pub.l_torque_interia_comp.resize(LARMDOF);
    tempCtlData_pub.l_torque_feedforward.resize(LARMDOF);
    tempCtlData_pub.l_kp.resize(LARMDOF);
    tempCtlData_pub.l_kd.resize(LARMDOF);

    tempCtlData_pub.r_position.resize(RARMDOF);
    tempCtlData_pub.r_speed.resize(RARMDOF);
    tempCtlData_pub.r_torque_grav_comp.resize(RARMDOF);
    tempCtlData_pub.r_torque_interia_comp.resize(RARMDOF);
    tempCtlData_pub.r_torque_feedforward.resize(RARMDOF);
    tempCtlData_pub.r_kp.resize(RARMDOF);
    tempCtlData_pub.r_kd.resize(RARMDOF);

    for(int i = 0; i < LARMDOF; i++){
        /**/
        tempCtlData_pub.l_position[i] = lArm_joint_pos[i];
        tempCtlData_pub.l_speed[i] = lArm_joint_spd[i];

        tempCtlData_pub.l_torque_grav_comp[i] = lArm_joint_tor_grav[i];
        tempCtlData_pub.l_torque_interia_comp[i] = lArm_joint_tor_interia[i];
        tempCtlData_pub.l_torque_feedforward[i] = lArm_joint_tor_feedforward[i];

        tempCtlData_pub.l_kp[i] = lArm_joint_kp[i];
        tempCtlData_pub.l_kd[i] = lArm_joint_kd[i];
    }

    for(int i = 0; i < RARMDOF; i++){
        tempCtlData_pub.r_position[i] = rArm_joint_pos[i];
        tempCtlData_pub.r_speed[i] = rArm_joint_spd[i];

        tempCtlData_pub.r_torque_grav_comp[i] = rArm_joint_tor_grav[i];
        tempCtlData_pub.r_torque_interia_comp[i] = rArm_joint_tor_interia[i];
        tempCtlData_pub.r_torque_feedforward[i] = rArm_joint_tor_feedforward[i];

        tempCtlData_pub.r_kp[i] = rArm_joint_kp[i];
        tempCtlData_pub.r_kd[i] = rArm_joint_kd[i];
    }

    tempCtlData_pub.float_reserve.resize(LARMDOF + RARMDOF);
    tempCtlData_pub.int_reserve.resize(LARMDOF + RARMDOF);
    for(int i = 0; i < LARMDOF + RARMDOF; i++){
        tempCtlData_pub.float_reserve[i] = 0;
        tempCtlData_pub.int_reserve[i] = 0;
    }

    mCtlData_pub->publish(tempCtlData_pub);
}

//上位机指令发送模块
void ArmCtlNode::midMonitor_msg_publish(){

    auto tempCtlData_pub = ctl_msgs_hw::msg::MidNodeCtlMsgRet();
    tempCtlData_pub.header.stamp = this->now();     //添加当前时间戳

    //发送控制反馈量

    tempCtlData_pub.if_first_temp_in_base = if_lower_firstTime;
    tempCtlData_pub.if_first_temp_in_middle = if_middle_firstTime;

    //发送反馈值
    tempCtlData_pub.l_arm_pos_fdback.resize(LARMDOF);
    tempCtlData_pub.l_arm_spd_fdback.resize(LARMDOF);
    tempCtlData_pub.l_arm_iq_fdback.resize(LARMDOF);
    
    tempCtlData_pub.r_arm_pos_fdback.resize(RARMDOF);
    tempCtlData_pub.r_arm_spd_fdback.resize(RARMDOF);
    tempCtlData_pub.r_arm_iq_fdback.resize(RARMDOF);

    for(int i = 0; i < LARMDOF; i++){
        tempCtlData_pub.l_arm_pos_fdback[i] = lArm_pos_fdback[i];
        tempCtlData_pub.l_arm_spd_fdback[i] = lArm_spd_fdback[i];
        tempCtlData_pub.l_arm_iq_fdback[i] = lArm_iq_fdback[i];
    }
    for(int i = 0; i < RARMDOF; i++){
        tempCtlData_pub.r_arm_pos_fdback[i] = rArm_pos_fdback[i];
        tempCtlData_pub.r_arm_spd_fdback[i] = rArm_spd_fdback[i];
        tempCtlData_pub.r_arm_iq_fdback[i] = rArm_iq_fdback[i];
    }

    //发送计算状态反馈
    tempCtlData_pub.l_arm_if_jnt_reach_despos.resize(LARMDOF);
    tempCtlData_pub.r_arm_if_jnt_reach_despos.resize(RARMDOF);
    for(int i = 0; i < LARMDOF; i++){
        tempCtlData_pub.l_arm_if_jnt_reach_despos[i] = lArm_ifReachEnd[i];
    }
    for(int i = 0; i < RARMDOF; i++){
        tempCtlData_pub.r_arm_if_jnt_reach_despos[i] = rArm_ifReachEnd[i];
    }

    tempCtlData_pub.l_arm_jntpos_des.resize(LARMDOF);
    tempCtlData_pub.l_arm_jntspd_des.resize(LARMDOF);
    tempCtlData_pub.l_arm_jnttor_gravcomp.resize(LARMDOF);
    tempCtlData_pub.l_arm_jnttor_interiacomp.resize(LARMDOF);
    tempCtlData_pub.l_arm_jnttor_feedforward.resize(LARMDOF);

    tempCtlData_pub.r_arm_jntpos_des.resize(RARMDOF);
    tempCtlData_pub.r_arm_jntspd_des.resize(RARMDOF);
    tempCtlData_pub.r_arm_jnttor_gravcomp.resize(RARMDOF);
    tempCtlData_pub.r_arm_jnttor_interiacomp.resize(RARMDOF);
    tempCtlData_pub.r_arm_jnttor_feedforward.resize(RARMDOF);

    for(int i = 0; i < LARMDOF; i++){
        tempCtlData_pub.l_arm_jntpos_des[i] = lArm_joint_pos[i];
        tempCtlData_pub.l_arm_jntspd_des[i] = lArm_joint_spd[i];
        
        tempCtlData_pub.l_arm_jnttor_gravcomp[i] = lArm_joint_tor_grav[i];
        tempCtlData_pub.l_arm_jnttor_interiacomp[i] = lArm_joint_tor_interia[i];
        tempCtlData_pub.l_arm_jnttor_feedforward[i] = lArm_joint_tor_feedforward[i];
    }
    for(int i = 0; i < RARMDOF; i++){
        tempCtlData_pub.r_arm_jntpos_des[i] = rArm_joint_pos[i];
        tempCtlData_pub.r_arm_jntspd_des[i] = rArm_joint_spd[i];

        tempCtlData_pub.r_arm_jnttor_gravcomp[i] = rArm_joint_tor_grav[i];
        tempCtlData_pub.r_arm_jnttor_interiacomp[i] = rArm_joint_tor_interia[i];
        tempCtlData_pub.r_arm_jnttor_feedforward[i] = rArm_joint_tor_feedforward[i];
    }

    upperMonitorMsg_send->publish(tempCtlData_pub);

}

//反馈值接受模块
void ArmCtlNode::fdback_get() {
    rclcpp::spin_some(this->get_node_base_interface());  // 非阻塞检查回调
}

//------------------------------
//仿真信息与debug信息发送模块

// 发送关节角度到RViz
// 使用KDL计算正向运动学并发布TF
void ArmCtlNode::publish_tf_from_joint_states(KDL::Chain mkdl_chain_, std::vector<double> mjoint_pos) {
    // 创建KDL关节数组
    KDL::JntArray jnt_array(mkdl_chain_.getNrOfJoints());
    
    // 填充当前关节位置
    for (size_t i = 0; i < mjoint_pos.size() && i < static_cast<size_t>(mkdl_chain_.getNrOfJoints()); ++i) {
        jnt_array(i) = mjoint_pos[i];
    }

    // 创建KDL帧数组来存储各连杆的位姿
    std::vector<KDL::Frame> frames(mkdl_chain_.getNrOfSegments() + 1);
    frames[0] = KDL::Frame::Identity(); // 基座坐标系

    // 计算各连杆的位姿
    int joint_id = 0;
    KDL::ChainFkSolverPos_recursive fk_solver(mkdl_chain_);
    for (size_t i = 0; i < mkdl_chain_.getNrOfSegments(); ++i) {
        const KDL::Segment& segment = mkdl_chain_.getSegment(i);
        const KDL::Joint& joint = segment.getJoint();
        
        // 如果是关节（不是固定连接）
        if (joint.getType() != KDL::Joint::None) {
            // 获取当前关节索引
            int joint_idx = mkdl_chain_.getNrOfJoints() - 1; // 默认最后一个关节
            for (int j = 0; j < mkdl_chain_.getNrOfJoints(); ++j) {
                if (mkdl_chain_.getSegment(j).getJoint().getName() == joint.getName()) {
                    joint_idx = j;
                    break;
                }
            }

            // 计算前一个位姿到当前位姿的变换
            //frames[i + 1] = frames[i] * segment.getFrameToTip() * joint_frame;
            //已知关节角度，计算父连杆末端到关节坐标系的变换矩阵
            double joint_angle = jnt_array(joint_id);
            KDL::Frame joint_frame;
            switch (joint.getType())
            {
            case KDL::Joint::RotAxis:
                //旋转轴关节 -- 需要获取关节旋转方向    
                joint_frame = KDL::Frame(KDL::Rotation::Rot(joint.JointAxis(), joint_angle));
                break;
            case KDL::Joint::RotX:
                joint_frame = KDL::Frame(KDL::Rotation::RotX(joint_angle));
                break;
            case KDL::Joint::RotY:
                joint_frame = KDL::Frame(KDL::Rotation::RotY(joint_angle));
                break;
            case KDL::Joint::RotZ:
                joint_frame = KDL::Frame(KDL::Rotation::RotZ(joint_angle));
                break;
            default:
                break;
            }

            KDL::Frame tempFrame = segment.getFrameToTip();     //计算关节坐标系到连杆末端的变换矩阵
            frames[i + 1] = tempFrame * joint_frame;
            joint_id = joint_id + 1;

        } else {
            // 固定连接，直接应用segment的变换
            frames[i + 1] = segment.getFrameToTip();
        }
    }

    // 发布所有连杆的TF
    auto now = global_timestamp_;
    for (size_t i = 0; i < frames.size(); ++i) {
        if (i == 0) continue; // 跳过基座
        
        const KDL::Segment& segment = mkdl_chain_.getSegment(i - 1);
        std::string parent_frame = (i == 1) ? base_link[0] : mkdl_chain_.getSegment(i - 2).getName();
        std::string child_frame = segment.getName();

        // 创建TF变换消息
        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = now;
        transform.header.frame_id = parent_frame;
        transform.child_frame_id = child_frame;
        
        // 从KDL Frame转换
        transform.transform.translation.x = frames[i].p.x();
        transform.transform.translation.y = frames[i].p.y();
        transform.transform.translation.z = frames[i].p.z();
        
        double roll, pitch, yaw;
        frames[i].M.GetRPY(roll, pitch, yaw);
        tf2::Quaternion quat;
        quat.setRPY(roll, pitch, yaw);
        transform.transform.rotation.x = quat.x();
        transform.transform.rotation.y = quat.y();
        transform.transform.rotation.z = quat.z();
        transform.transform.rotation.w = quat.w();
        
        // 发布TF
        tf_broadcaster_->sendTransform(transform);
    }
}


void ArmCtlNode::fk_publish(
    KDL::Frame lArm_retDescPos, KDL::Frame rArm_retDescPos,
    KDL::Frame lArm_desDescPos, KDL::Frame rArm_desDescPos
){
    auto tempDesData_pub = ctl_msgs_hw::msg::FkDataPub();
    auto tempRetData_pub = ctl_msgs_hw::msg::FkDataPub();

    // ---- 目标位姿（左臂）----
    tempDesData_pub.x_l = lArm_desDescPos.p[0];
    tempDesData_pub.y_l = lArm_desDescPos.p[1];
    tempDesData_pub.z_l = lArm_desDescPos.p[2];
    lArm_desDescPos.M.GetRPY(tempDesData_pub.roll_l, tempDesData_pub.pitch_l, tempDesData_pub.yaw_l);

    // ---- 目标位姿（右臂）----
    tempDesData_pub.x_r = rArm_desDescPos.p[0];
    tempDesData_pub.y_r = rArm_desDescPos.p[1];
    tempDesData_pub.z_r = rArm_desDescPos.p[2];
    rArm_desDescPos.M.GetRPY(tempDesData_pub.roll_r, tempDesData_pub.pitch_r, tempDesData_pub.yaw_r);

    // ---- 反馈位姿（左臂）----
    tempRetData_pub.x_l = lArm_retDescPos.p[0];
    tempRetData_pub.y_l = lArm_retDescPos.p[1];
    tempRetData_pub.z_l = lArm_retDescPos.p[2];
    lArm_retDescPos.M.GetRPY(tempRetData_pub.roll_l, tempRetData_pub.pitch_l, tempRetData_pub.yaw_l);

    // ---- 反馈位姿（右臂）----
    tempRetData_pub.x_r = rArm_retDescPos.p[0];
    tempRetData_pub.y_r = rArm_retDescPos.p[1];
    tempRetData_pub.z_r = rArm_retDescPos.p[2];
    rArm_retDescPos.M.GetRPY(tempRetData_pub.roll_r, tempRetData_pub.pitch_r, tempRetData_pub.yaw_r);

    // 发布
    mFkDesRes_pub->publish(tempDesData_pub);
    mFkRetRes_pub->publish(tempRetData_pub);
}

//------------------------------
//ArmInterface接口交互模块

//初始化模块
void ArmCtlNode::InterfaceInit(){
    //初始化模块，采用返回值进行初始化

    //初始化量解包
    KDL::JntArray lArm_stPos, rArm_stPos;
    lArm_stPos = vec_to_JntArray(lArm_zeroPos);
    rArm_stPos = vec_to_JntArray(rArm_zeroPos);

    //求解器初始化
    std::vector<double> waist_stPos(6, 0.0);  // 固定基座，零位姿
    if (!solverInterface.init(
        waist_stPos,
        lArm_stPos, rArm_stPos,

        lArm_LimMsg_get, rArm_LimMsg_get,
        modCtlMsg_get,

        msgPringControlParam,
        optimizerControParam
    )) {
        std::cerr << "\n======================================================" << std::endl;
        std::cerr << "FATAL: InterfaceInit() FAILED" << std::endl;
        std::cerr << "The solver interface could not be initialized." << std::endl;
        std::cerr << "This usually means the Python 'ik_solver' module failed to load." << std::endl;
        std::cerr << "Check:" << std::endl;
        std::cerr << "  1. Has 'source install/setup.bash' been run?" << std::endl;
        std::cerr << "  2. Are the Cython .so files under install/dual_arm_control/share/dual_arm_control/scripts/?" << std::endl;
        std::cerr << "  3. Do the .so files match the Python version (Python 3.10)?" << std::endl;
        std::cerr << "  4. See the error messages above for the specific cause." << std::endl;
        std::cerr << "======================================================" << std::endl;
        ifIKsolverInited = false;
    } else {
        ifIKsolverInited = true;
        std::cout << "初始化完成" << std::endl;
    }
}

//进行一轮控制计算的模块
void ArmCtlNode::controlCal(){
    KDL::Frame lArm_desPos_Frame, rArm_desPos_Frame;
    KDL::JntArray lArm_desPos_Jnt, rArm_desPos_Jnt;
    KDL::JntArray lArm_posRet_Jnt, rArm_posRet_Jnt;
    KDL::JntArray lArm_velRet_Jnt, rArm_velRet_Jnt;
    const std::vector<double> zero_cartesian_force(6, 0.0);
    const std::vector<double> zero_left_joint_force(LARMDOF, 0.0);
    const std::vector<double> zero_right_joint_force(RARMDOF, 0.0);

    lArm_desPos_Frame = vec_to_Frame(lArm_desPos_descartes);
    rArm_desPos_Frame = vec_to_Frame(rArm_desPos_descartes);
    
    lArm_desPos_Jnt = vec_to_JntArray(lArm_desPos_JntSpace);
    rArm_desPos_Jnt = vec_to_JntArray(rArm_desPos_JntSpace);

    lArm_posRet_Jnt = vec_to_JntArray(lArm_pos_fdback);
    rArm_posRet_Jnt = vec_to_JntArray(rArm_pos_fdback);
    lArm_velRet_Jnt = vec_to_JntArray(lArm_spd_fdback);
    rArm_velRet_Jnt = vec_to_JntArray(rArm_spd_fdback);

    //进行一轮计算
    solverInterface.Traj_cal(
        ifUpdateLim_get,
        allArm_LimMsg_get,

        lArm_desPos_Frame, rArm_desPos_Frame,
        zero_cartesian_force, zero_cartesian_force,

        lArm_desPos_Jnt, rArm_desPos_Jnt,
        zero_left_joint_force, zero_right_joint_force,

        lArm_posRet_Jnt, rArm_posRet_Jnt,
        lArm_velRet_Jnt, rArm_velRet_Jnt,
        lArm_iq_fdback, rArm_iq_fdback,

        modCtlMsg_get,
        ifRedefineJntTrajStab_get
    );

    ifCalculated = true;
}

// 回零控制初始化计算代码
void ArmCtlNode::retZeroControlInit(){
    // ---------- 刷新控制参量 ---------- 
    allArm_LimMsg_get.ifUseJntLim = false;

    modCtl temp_modMsg = modCtlMsg_get;
    temp_modMsg.transMod = 1;
    temp_modMsg.runinMod = 3;
    temp_modMsg.TorFeedforwardCtl = 0;

    temp_modMsg.ifUseTemp = false;
    for(int i = 0; i < LARMDOF; i++){
        temp_modMsg.base_rigiditiCtl_mod_l[i] = 1;
        temp_modMsg.base_rigiditiCtl_mod_r[i] = 1;
    }

    std::vector<std::vector<double>> TrajTime, accRateBase;
    TrajTime.resize(2);
    TrajTime[0].assign(LARMDOF, 2.0f);
    TrajTime[1].assign(RARMDOF, 2.0f);
    accRateBase.resize(2);
    accRateBase[0].assign(LARMDOF, 1.2f);
    accRateBase[1].assign(RARMDOF, 1.2f);
    temp_modMsg.TrajTime = TrajTime;
    temp_modMsg.accRateBase = accRateBase;
    for(int i = 0; i < LARMDOF; i++){
        temp_modMsg.m_stabMsg[0][i].ifUseStab = true;
        temp_modMsg.m_stabMsg[0][i].start_stabTimes = 50;
        temp_modMsg.m_stabMsg[0][i].des_stabTimes = 30;
    }
    for(int i = 0; i < RARMDOF; i++){
        temp_modMsg.m_stabMsg[1][i].ifUseStab = true;
        temp_modMsg.m_stabMsg[1][i].start_stabTimes = 50;
        temp_modMsg.m_stabMsg[1][i].des_stabTimes = 50;
    }

    std::vector<std::vector<int>> temp_ifJntStab;
    temp_ifJntStab.resize(2);
    temp_ifJntStab[0].assign(LARMDOF, 0);
    temp_ifJntStab[1].assign(RARMDOF, 0);
    temp_modMsg.ifJntStab = temp_ifJntStab;

    // ---------- 执行计算任务 ----------
    KDL::Frame lArm_desPos_Frame, rArm_desPos_Frame;
    KDL::JntArray lArm_desPos_Jnt, rArm_desPos_Jnt;
    KDL::JntArray lArm_posRet_Jnt, rArm_posRet_Jnt;
    KDL::JntArray lArm_velRet_Jnt, rArm_velRet_Jnt;
    const std::vector<double> zero_cartesian_force(6, 0.0);
    const std::vector<double> zero_left_joint_force(LARMDOF, 0.0);
    const std::vector<double> zero_right_joint_force(RARMDOF, 0.0);

    lArm_desPos_Frame = vec_to_Frame(l_descZeroPos);
    rArm_desPos_Frame = vec_to_Frame(r_descZeroPos);

    lArm_desPos_Jnt = vec_to_JntArray(lArm_zeroPos);
    rArm_desPos_Jnt = vec_to_JntArray(rArm_zeroPos);

    lArm_posRet_Jnt = vec_to_JntArray(lArm_pos_fdback);
    rArm_posRet_Jnt = vec_to_JntArray(rArm_pos_fdback);
    lArm_velRet_Jnt = vec_to_JntArray(lArm_spd_fdback);
    rArm_velRet_Jnt = vec_to_JntArray(rArm_spd_fdback);

    // 进行一轮计算
    solverInterface.Traj_cal(
        false,
        allArm_LimMsg_get,

        lArm_desPos_Frame, rArm_desPos_Frame,
        zero_cartesian_force, zero_cartesian_force,

        lArm_desPos_Jnt, rArm_desPos_Jnt,
        zero_left_joint_force, zero_right_joint_force,

        lArm_posRet_Jnt, rArm_posRet_Jnt,
        lArm_velRet_Jnt, rArm_velRet_Jnt,
        lArm_iq_fdback, rArm_iq_fdback,

        temp_modMsg,
        false
    );

    //--------------------------------------------------
    //控制变量刷新
    lArm_ifReachEnd.assign(LARMDOF, false);
    rArm_ifReachEnd.assign(RARMDOF, false);
}

//控制封装代码
void ArmCtlNode::control_seal(){

    //如果没有接收到下位机的反馈值，强制不使用返回值，并强制各个初始化过程需要注入的参数刷零
    if(!has_received_command_){
        modCtlMsg_get.ifUseRet = 0;
    }

    // 保存本周期由上层发送的“名义目标”。导纳输出会在本函数内覆盖
    // lArm/rArm_desPos_descartes，但该修正后的位姿不能作为下一周期
    // 名义目标的历史，否则下一帧遥操作增量会被误算成很大的速度，表现
    // 为接触时抖动和卡顿。
    const std::vector<double> nominal_l_pose = lArm_desPos_descartes;
    const std::vector<double> nominal_r_pose = rArm_desPos_descartes;

    //先进行一轮初始位置刷新 -- 注意执行第一轮要确保强制刷新
    if(ifFirstTemp){
        solverInterface.updatePosLst(vec_to_JntArray(lArm_pos_fdback), vec_to_JntArray(rArm_pos_fdback));
    }

    //再进行一轮控制计算 -- 注意满足条件或第一轮情况下计算
    bool all_lArmEnd = std::all_of(lArm_ifReachEnd.begin(), lArm_ifReachEnd.end(), [](bool val) { return val; });
    bool all_rArmEnd = std::all_of(rArm_ifReachEnd.begin(), rArm_ifReachEnd.end(), [](bool val) { return val; });
    bool ifAllEnd = all_lArmEnd && all_rArmEnd;

    //前馈力置零
    lArm_joint_tor_feedforward.clear();
    rArm_joint_tor_feedforward.clear();
    lArm_joint_tor_feedforward.assign(LARMDOF, 0.0);
    rArm_joint_tor_feedforward.assign(RARMDOF, 0.0);

    //位置控制计算
    if(modCtlMsg_get.transMod != 0 && modCtlMsg_get.transMod != 6){
        //其他控制模式
        // 连续笛卡尔遥操作（runinMod=1）是一个 50 Hz 的目标流，不能等待
        // 上一段插补完成才重新求逆解；否则 Pico 增量会在控制器内部堆积，
        // 造成明显滞后以及改变方向时的过冲。每周期用反馈位置重规划，
        // IK 仍以当前关节角为种子，保持解的连续性。
        if(ifFirstTemp || ifAllEnd || modCtlMsg_get.runinMod == 1){
            controlCal();
        }

        //进行输出
        controlOutp();


        //向着上位机输出反馈值
        midMonitor_msg_publish();

        has_received_upperMonitor_command_ = false;
    }else if(modCtlMsg_get.transMod == 6){
        //MDP控制模式
        if(ifFirstTemp || ifAllEnd){
            MDP_runin();
            ifFirstTemp = false;
        }

        //MDP控制模式下，PD参数强制使用自定义PD参数，底层插补强制采用线性插补
        MDP_trans MDP_runinRes = MDP_outp();
        lArm_joint_pos = MDP_runinRes.lArm_pos;
        lArm_joint_spd = MDP_runinRes.lArm_spd;
        rArm_joint_pos = MDP_runinRes.rArm_pos;
        rArm_joint_spd = MDP_runinRes.rArm_spd;

        //Kp，Kd采集 -- 强制采用自定义PD参数
        lArm_joint_kp = lArm_Kp_;
        lArm_joint_kd = lArm_Kd_;
        rArm_joint_kp = rArm_Kp_;
        rArm_joint_kd = rArm_Kd_;

        //位置到达标志反馈


        //向着上位机输出反馈值
        midMonitor_msg_publish();

        has_received_upperMonitor_command_ = false;
    }else{
        //回零位模式
        // 确保已收到下位机反馈后再进行回零轨迹规划，避免使用未初始化的零位值导致跳变
        if(has_received_command_ && (ifFirstTemp || ifAllEnd)){
            retZeroControlInit();
        }

        //进行输出
        controlOutp();

        //向着上位机输出反馈值
        midMonitor_msg_publish();

        has_received_upperMonitor_command_ = false;
    }

    // Position-only teleoperation does not accept external torque feedforward.
    lArm_joint_tor_feedforward.assign(LARMDOF, 0.0);
    rArm_joint_tor_feedforward.assign(RARMDOF, 0.0);

    ifFirstTemp = false;
}

//注意修改执行逻辑，有的时候这个ifReachEnd输出的不正确，例子：轨迹2,另外要添加
//上层规划器令中层规划器刷新的机制。（强制令中层规划器重新初始化）

//进行一轮输出
void ArmCtlNode::controlOutp(){
    KDL::JntArray lArm_posRet, rArm_posRet;
    KDL::JntArray lArm_velRet, rArm_velRet;
    
    lArm_posRet = vec_to_JntArray(lArm_pos_fdback);
    rArm_posRet = vec_to_JntArray(rArm_pos_fdback);

    lArm_velRet = vec_to_JntArray(lArm_spd_fdback);
    rArm_velRet = vec_to_JntArray(rArm_spd_fdback);

    std::vector<outpWrap> mRes;
    mRes = solverInterface.Traj_temp_outp(
        waist_desc_pos, waist_desc_spd,
        lArm_posRet, rArm_posRet,
        lArm_velRet, rArm_velRet,
        lArm_iq_fdback, rArm_iq_fdback
    );

    //输出解包
    lArm_joint_pos = mRes[0].posDes;
    lArm_joint_spd = mRes[0].velDes;


    lArm_joint_tor_grav = mRes[0].gravComp_TorDes;
    lArm_joint_tor_interia = mRes[0].InteriaComp_TorDes;
    lArm_joint_tor_feedforward = mRes[0].FeedForward_TorDes;
    
    lArm_joint_kp = mRes[0].KpDes;
    lArm_joint_kd = mRes[0].KdDes;

    rArm_joint_pos = mRes[1].posDes;
    rArm_joint_spd = mRes[1].velDes;

    rArm_joint_tor_grav = mRes[1].gravComp_TorDes;
    rArm_joint_tor_interia = mRes[1].InteriaComp_TorDes;
    rArm_joint_tor_feedforward = mRes[1].FeedForward_TorDes;

    rArm_joint_kp = mRes[1].KpDes;
    rArm_joint_kd = mRes[1].KdDes;

    lArm_ifReachEnd = mRes[0].ifComToEnd;
    rArm_ifReachEnd = mRes[1].ifComToEnd;

    //若使用了自定义的PD参数，则需要覆盖原PD参数
    for(int i = 0; i < LARMDOF; i++){
        if(modCtlMsg_get.base_rigiditiCtl_mod_l[i] == -1){
            lArm_joint_kp[i] = lArm_Kp_[i];
            lArm_joint_kd[i] = lArm_Kd_[i];
        }

        if(modCtlMsg_get.base_rigiditiCtl_mod_r[i] == -1){
            rArm_joint_kp[i] = rArm_Kp_[i];
            rArm_joint_kd[i] = rArm_Kd_[i];
        }
    }
}

//------------------------------
//ros传输变量初始化模块

void ArmCtlNode::rosloop_var_init(){
    //---------------中层节点与下位机通讯全局变量---------------

    //控制量发送：
    runin_mod = -1;
    if_runin_renew = 0;
    ifUseRet_toStRunin = 0;
    ifUseDebugMod = 0;
    base_rigidityCtl_mod_l.resize(LARMDOF);
    base_rigidityCtl_mod_r.resize(RARMDOF);
    for(int i = 0; i < LARMDOF; i++){
        base_rigidityCtl_mod_l[i] = 1;
    }
    for(int i = 0; i < RARMDOF; i++){
        base_rigidityCtl_mod_r[i] = 1;
    }

    //左右臂初始值刷新
    lArm_pos_atRuninSt = lArm_zeroPos;
    rArm_pos_atRuninSt = rArm_zeroPos;

    //目标参量发送：
    lArm_joint_pos = lArm_zeroPos;
    lArm_joint_kp = lArm_defaultKp_low;
    lArm_joint_kd = lArm_defaultKd_low;
    lArm_joint_spd.assign(LARMDOF, 0.0);
    lArm_joint_tor_grav.assign(LARMDOF, 0.0);
    lArm_joint_tor_interia.assign(LARMDOF, 0.0);
    lArm_joint_tor_feedforward.assign(LARMDOF, 0.0);

    rArm_joint_pos = rArm_zeroPos;
    rArm_joint_kp = rArm_defaultKp_low;
    rArm_joint_kd = rArm_defaultKd_low;
    rArm_joint_spd.assign(RARMDOF, 0.0);
    rArm_joint_tor_grav.assign(RARMDOF, 0.0);
    rArm_joint_tor_interia.assign(RARMDOF, 0.0);
    rArm_joint_tor_feedforward.assign(RARMDOF, 0.0);

    //反馈接收：
    lArm_pos_fdback = lArm_zeroPos;
    lArm_spd_fdback.assign(LARMDOF, 0.0);
    lArm_iq_fdback.assign(LARMDOF, 0.0);
    rArm_pos_fdback = rArm_zeroPos;
    rArm_spd_fdback.assign(RARMDOF, 0.0);
    rArm_iq_fdback.assign(RARMDOF, 0.0);

    waist_desc_pos.assign(6, 0.0);
    waist_desc_spd.assign(6, 0.0);

    if_lower_firstTime = true;
    if_middle_firstTime = true;

    //控制状态向上位机反馈：
    lArm_ifReachEnd.assign(LARMDOF, true);
    rArm_ifReachEnd.assign(RARMDOF, true);

    //---------------中层节点与上位机通讯全局变量---------------

    //控制过程信息打包
    ifReInit_get = false;
    ifUpdateLim_get = false;
    ifRedefineJntTrajStab_get = false;
    ifCalculated = false;

    //目标位置等信息
    lArm_desPos_descartes = l_descZeroPos;
    lArm_desPos_JntSpace = lArm_zeroPos;

    rArm_desPos_descartes = r_descZeroPos;
    rArm_desPos_JntSpace = rArm_zeroPos;

    lArm_desVel_MDP_JntSpace.assign(LARMDOF, 0.0);
    rArm_desVel_MDP_JntSpace.assign(RARMDOF, 0.0);

    //搭配自定义PD参数模式使用
    lArm_Kp_ = lArm_defaultKp_low;
    lArm_Kd_ = lArm_defaultKd_low;
    rArm_Kp_ = rArm_defaultKp_low;
    rArm_Kd_ = rArm_defaultKd_low;

    //debug用的辅助变量
    vec_dbg_1.assign(LARMDOF, 0.0);
    vec_dbg_2.assign(LARMDOF, 0.0);

    //动力学递推差分变量初始化
    lArm_JntPosLast.resize(LARMDOF);
    rArm_JntPosLast.resize(RARMDOF);
    lArm_JntVelLast.assign(LARMDOF, 0.0);
    rArm_JntVelLast.assign(RARMDOF, 0.0);
    base_lastPos.assign(6, 0.0);
    base_lastVel.assign(6, 0.0);

    for(int i = 0; i < LARMDOF; i++){
        lArm_JntPosLast(i) = lArm_zeroPos[i];
    }
    for(int i = 0; i < RARMDOF; i++){
        rArm_JntPosLast(i) = rArm_zeroPos[i];
    }

    //---------------求解器状态控制变量初始化---------------
    //初始时处于静息状态
    lArm_LimMsg_get.ifUseJntLim = false;
    lArm_LimMsg_get.limJntNum = 0;
    lArm_LimMsg_get.limJntLabel = {"l_joint_3"};
    lArm_LimMsg_get.limJntAng = {0.1};

    rArm_LimMsg_get.ifUseJntLim = false;
    rArm_LimMsg_get.limJntNum = 0;
    rArm_LimMsg_get.limJntLabel = {"r_joint_3"};
    rArm_LimMsg_get.limJntAng = {0.1};

    modCtlMsg_get.transMod = 1;
    modCtlMsg_get.runinMod = 1;
    modCtlMsg_get.TorFeedforwardCtl = 0;

    modCtlMsg_get.ifUseRet = 0;
    modCtlMsg_get.ifUseTemp = false;

    //刚度模式初始话，默认处于低刚度模式
    modCtlMsg_get.base_rigiditiCtl_mod_l.resize(LARMDOF);
    modCtlMsg_get.base_rigiditiCtl_mod_r.resize(RARMDOF);
    for(int i = 0; i < LARMDOF; i++){
        modCtlMsg_get.base_rigiditiCtl_mod_l[i] = 1;
    }
    for(int i = 0; i < RARMDOF; i++){
        modCtlMsg_get.base_rigiditiCtl_mod_r[i] = 1;
    }

    modCtlMsg_get.upperMonitor_time = TEMPTIME;
    modCtlMsg_get.TrajTime.resize(2);
    modCtlMsg_get.accRateBase.resize(2);
    modCtlMsg_get.TrajTime[0].assign(LARMDOF, 1.0);
    modCtlMsg_get.TrajTime[1].assign(RARMDOF, 1.0);
    modCtlMsg_get.accRateBase[0].assign(LARMDOF, 1.2);
    modCtlMsg_get.accRateBase[1].assign(RARMDOF, 1.2);
    modCtlMsg_get.m_stabMsg.resize(2);
    modCtlMsg_get.m_stabMsg[0].resize(LARMDOF);
    modCtlMsg_get.m_stabMsg[1].resize(RARMDOF);
    for(int i = 0; i < LARMDOF; i++){
        modCtlMsg_get.m_stabMsg[0][i].ifUseStab = true;
        modCtlMsg_get.m_stabMsg[0][i].start_stabTimes = 50;
        modCtlMsg_get.m_stabMsg[0][i].des_stabTimes = 50;
    }
    for(int i = 0; i < RARMDOF; i++){
        modCtlMsg_get.m_stabMsg[1][i].ifUseStab = true;
        modCtlMsg_get.m_stabMsg[1][i].start_stabTimes = 50;
        modCtlMsg_get.m_stabMsg[1][i].des_stabTimes = 50;
    }
    modCtlMsg_get.ifJntStab.resize(2);
    modCtlMsg_get.ifJntStab[0].assign(LARMDOF, 0);
    modCtlMsg_get.ifJntStab[1].assign(RARMDOF, 0);

    //----------------------------------------
    //MDP求解变量初始化
    MDP_posOutp_lArm.resize(1);
    MDP_posOutp_lArm[0] = lArm_zeroPos;
    MDP_velOutp_lArm.resize(1);
    MDP_velOutp_lArm[0].assign(LARMDOF, 0.0);

    MDP_posOutp_rArm.resize(1);
    MDP_posOutp_rArm[0] = rArm_zeroPos;
    MDP_velOutp_rArm.resize(1);
    MDP_velOutp_rArm[0].assign(RARMDOF, 0.0);

    // MDP 关节静止保持：记录上一轮输出位置，当关节静止标识为1时保持该位置
    MDP_lastPos_lArm = lArm_zeroPos;
    MDP_lastPos_rArm = rArm_zeroPos;

    MDP_trajLen = 1;
    MDP_tempTimes = 0;

    lArmLastPos = lArm_zeroPos;
    rArmLastPos = rArm_zeroPos;

    //-----------------------------------
    //重力补偿初始化量
    zeroSpeed_count = 0;
    ifTrigger = false;
    ifInitTrigger = true;

    std::cout << "全局变量初始化完毕 (rosloop_var_init)" << std::endl;
}

//------------------------------
//ros2 循环函数

void ArmCtlNode::run(){
    //用户交互
    std::cout << "开始执行中层节点封装程序" << std::endl;

    //初始化全局变量
    rosloop_var_init();

    //初始化python解释器：
    PyGILState_STATE gstate = PyGILState_Ensure();

    //初始化执行频率
    double loopRate_unRound = 1 / TEMPTIME;
    int loopRate = std::round(loopRate_unRound);
    std::cout << "轨迹执行频率为：" << loopRate <<" Hz。" << std::endl;
    rclcpp::Rate loop_rate(loopRate); 

    //初始化阶段 -- 初始化求解器与重力补偿
    while(rclcpp::ok() && ifInit){
        //第一轮执行时，自动检测位置
        if(ifCheckTemp){
            ifCheckTemp = false;

            //检测是否通讯成功
            if(has_received_upperMonitor_command_){
                std::cout << "警告：第一轮控制中中层节点需要在上层节点之前打开，退出程序以保证安全" << std::endl;
                break;
            }

            for(int i = 0; i < 10; i++){
                fdback_get();
            }
            for(int i = 0; i < LARMDOF; i++){
                std::cout << "lr pos in " << i << " = " << lArm_pos_fdback[i] << " || " << rArm_pos_fdback[i] << std::endl;
            }

            //初始化求解器
            InterfaceInit();

            //初始化滤波器，并执行一次计算
            GlobalFilterInit();
            GlobalFilterCal();

            //初始化重力补偿器
            if(ifUse_gravComp == 1){
                gravCompInit(); }

        }

        //更新时间戳
        global_timestamp_ = this->now();
        //获取一次反馈值
        fdback_get();
        //进行一次反馈值全局滤波
        GlobalFilterCal();

        //控制参量赋值
        lArm_joint_pos = lArm_zeroPos;
        lArm_joint_spd.assign(LARMDOF, 0.0);
        lArm_joint_tor_feedforward.assign(LARMDOF, 0.0);
        lArm_joint_tor_grav.assign(LARMDOF, 0.0);
        lArm_joint_tor_interia.assign(LARMDOF, 0.0);

        rArm_joint_pos = rArm_zeroPos;
        rArm_joint_spd.assign(RARMDOF, 0.0);
        rArm_joint_tor_feedforward.assign(RARMDOF, 0.0);
        rArm_joint_tor_grav.assign(RARMDOF, 0.0);
        rArm_joint_tor_interia.assign(RARMDOF, 0.0);

        //重力补偿计算
        if(ifUse_gravComp == 1){
            std::vector<double> lArm_stVel, rArm_stVel;
            lArm_stVel.assign(LARMDOF, 0.0);
            rArm_stVel.assign(RARMDOF, 0.0);
            TorCompWrap floatbase_gravCompRes;
            floatbase_gravCompRes = gravCompCalOutp(
                waist_desc_pos, waist_desc_spd,
                lArm_zeroPos, lArm_stVel,
                rArm_zeroPos, rArm_stVel,
                0
            );

            lArm_joint_tor_grav = floatbase_gravCompRes.lArm_Comp_grav;
            rArm_joint_tor_grav = floatbase_gravCompRes.rArm_comp_grav;
        }

        //向下位机输出并向上位机发送反馈值
        midMonitor_msg_publish();           //向上位机发送反馈值
        msg_publish();                      //向底层发送控制值

        //判断是否退出并开始后续控制
        if(recent_tempTimes > ctl_JntStStabTimes){
            ifInit = false;

        }
        recent_tempTimes += 1;

        //后处理代码
        rclcpp::spin_some(this->get_node_base_interface());
        loop_rate.sleep();
    }
    std::cout << "全部初始化流程完成，开始运行代码" << std::endl;
    //重置底层循环指令
    loop_rate.reset();

    //开始ros2循环
    while(rclcpp::ok()){
        //更新时间戳
        global_timestamp_ = this->now();
        //获取一次反馈值
        fdback_get();
        //进行一次反馈值全局滤波
        GlobalFilterCal();

        global_debugPub->debug_msg_publish_2(lArm_desPos_JntSpace, rArm_desPos_JntSpace);

        //若能接受到上位机的反馈值时，开始计算
        if(has_received_upperMonitor_command_){
            control_seal();  
        }else{
            /*
            此时应当进入位置保持模式，因此位置、Kp、Kd、重力补偿不刷新，
            前馈力矩刷新成上一轮的控制器下发目标力矩
            */

            //进行输出
            if(ifIKsolverInited && ifCalculated){
                controlOutp();
            }

            //强制令目标速度输出为0
            //lArm_joint_spd.assign(LARMDOF, 0.0);
            //lArm_joint_tor_interia.assign(LARMDOF, 0.0);

            //rArm_joint_spd.assign(RARMDOF, 0.0);0
            //rArm_joint_tor_interia.assign(RARMDOF, .0);

            lArm_joint_tor_feedforward.assign(LARMDOF, 0.0);
            rArm_joint_tor_feedforward.assign(RARMDOF, 0.0);

            //向着上位机输出反馈值
            midMonitor_msg_publish();
        }

        


        // Bridge samples already contain a coherent trajectory state. Preserve
        // its derivatives instead of differentiating the sparse position stream.
        stream_acceleration_.clear();
        if (stream_sample_ && stream_sample_->derivatives_valid && modCtlMsg_get.transMod == 3) {
            const auto& s = *stream_sample_;
            const double age = (now() - s.trajectory_header.stamp).seconds();
            const bool fresh = age >= 0 && age <= s.upper_monitor_time * 2.0;
            lArm_joint_pos = s.l_arm_des_pos_jntspace;
            rArm_joint_pos = s.r_arm_des_pos_jntspace;
            lArm_joint_spd = fresh ? s.l_arm_des_vel_jntspace : std::vector<double>(7, 0);
            rArm_joint_spd = fresh ? s.r_arm_des_vel_jntspace : std::vector<double>(7, 0);
            stream_acceleration_.assign(14, 0);
            if (fresh) {
                std::copy(s.l_arm_des_acc.begin(), s.l_arm_des_acc.end(), stream_acceleration_.begin());
                std::copy(s.r_arm_des_acc.begin(), s.r_arm_des_acc.end(), stream_acceleration_.begin() + 7);
            }
        }

        if(ifIKsolverInited){
            //----------执行末端轨迹画图任务----------
            //先获取返回值
            KDL::JntArray lArm_JntPosFdback_kdl, rArm_JntPosFdback_kdl;
            KDL::JntArray lArm_JntPosDes_kdl, rArm_JntPosDes_kdl;

            std::vector<KDL::Frame> Arm_descPosFdback_kdl;
            std::vector<KDL::Frame> Arm_desDescPos_kdl;

            lArm_JntPosFdback_kdl.resize(LARMDOF);
            rArm_JntPosFdback_kdl.resize(RARMDOF);
            lArm_JntPosDes_kdl.resize(LARMDOF);
            rArm_JntPosDes_kdl.resize(RARMDOF);

            for(int i = 0; i < LARMDOF; i++){
                lArm_JntPosFdback_kdl(i) = lArm_pos_fdback[i];
                rArm_JntPosFdback_kdl(i) = rArm_pos_fdback[i];
                lArm_JntPosDes_kdl(i) = lArm_joint_pos[i];
                rArm_JntPosDes_kdl(i) = rArm_joint_pos[i];
            }

            Arm_descPosFdback_kdl = solverInterface.FKsolver(
                lArm_JntPosFdback_kdl, rArm_JntPosFdback_kdl
            );
            Arm_desDescPos_kdl = solverInterface.FKsolver(
                lArm_JntPosDes_kdl, rArm_JntPosDes_kdl
            );

            fk_publish(
                Arm_descPosFdback_kdl[0], Arm_descPosFdback_kdl[1],
                Arm_desDescPos_kdl[0], Arm_desDescPos_kdl[1]
            );

        }

        //计算重力补偿 -- 此时需要区分是采用反馈位置还是采用目标位置
        if(ifUse_gravComp == 1){
            TorCompWrap floatbase_gravCompRes;
            floatbase_gravCompRes = gravCompCalWrap();

            lArm_joint_tor_grav = floatbase_gravCompRes.lArm_Comp_grav;
            lArm_joint_tor_interia = floatbase_gravCompRes.lArm_Comp_InteriaComb;

            rArm_joint_tor_grav = floatbase_gravCompRes.rArm_comp_grav;
            rArm_joint_tor_interia = floatbase_gravCompRes.rArm_Comp_InteriaComb;
        }

        //向下位机发送数据，同步给rviz2发送数据
        msg_publish();

        publish_tf_from_joint_states(kdl_lArm_chain_, lArm_joint_pos);
        publish_tf_from_joint_states(kdl_rArm_chain_, rArm_joint_pos);

        has_received_command_ = false;

        //末处理：
        rclcpp::spin_some(this->get_node_base_interface());
        loop_rate.sleep();
    }

    PyGILState_Release(gstate);
    RCLCPP_INFO(this->get_logger(), "程序已安全退出");
}

//------------------------------
//全局通用滤波器成员函数

void ArmCtlNode::GlobalFilterInit(){
    //滤波器初始化
    lArm_posFdback_dKalmanFtr.resize(LARMDOF);
    rArm_posFdback_dKalmanFtr.resize(RARMDOF);
    lArm_IqFdback_dKalmanFtr.resize(LARMDOF);
    rArm_IqFdback_dKalmanFtr.resize(RARMDOF);

    lArm_JointKt = robotModelStructConfig.lArm_jnt_Kt;
    rArm_JointKt = robotModelStructConfig.rArm_jnt_Kt;

    for(int i = 0; i < LARMDOF; i++){
        lArm_posFdback_dKalmanFtr[i].FilterInit(lArm_pos_fdback[i], dataTransLoopCtl.lArm_posFeedback_uniFtr[i].Q_gain, dataTransLoopCtl.lArm_posFeedback_uniFtr[i].R_gain, TEMPTIME);
        lArm_IqFdback_dKalmanFtr[i].FilterInit(lArm_iq_fdback[i], dataTransLoopCtl.lArm_IqFeedback_uniFtr[i].Q_gain, dataTransLoopCtl.lArm_IqFeedback_uniFtr[i].R_gain, TEMPTIME);
    }
    for(int i = 0; i < RARMDOF; i++){
        rArm_posFdback_dKalmanFtr[i].FilterInit(rArm_pos_fdback[i], dataTransLoopCtl.rArm_posFeedback_uniFtr[i].Q_gain, dataTransLoopCtl.rArm_posFeedback_uniFtr[i].R_gain, TEMPTIME);
        rArm_IqFdback_dKalmanFtr[i].FilterInit(rArm_iq_fdback[i], dataTransLoopCtl.rArm_IqFeedback_uniFtr[i].Q_gain, dataTransLoopCtl.rArm_IqFeedback_uniFtr[i].R_gain, TEMPTIME);
    }

    lArm_posFdback_Ftred.assign(LARMDOF, 0.0);
    rArm_posFdback_Ftred.assign(RARMDOF, 0.0);

    lArm_IqFdback_Ftred.assign(LARMDOF, 0.0);
    rArm_IqFdback_Ftred.assign(RARMDOF, 0.0);
    lArm_TorFdback_Ftred.assign(LARMDOF, 0.0);
    rArm_TorFdback_Ftred.assign(RARMDOF, 0.0);
}

void ArmCtlNode::GlobalFilterCal(){
    
    //执行滤波操作
    for(int i = 0; i < LARMDOF; i++){
        lArm_posFdback_Ftred[i] = lArm_posFdback_dKalmanFtr[i].FilteredResCal(lArm_pos_fdback[i]);
        lArm_IqFdback_Ftred[i] = lArm_IqFdback_dKalmanFtr[i].FilteredResCal(lArm_iq_fdback[i]);
    }

    for(int i = 0; i < RARMDOF; i++){
        rArm_posFdback_Ftred[i] = rArm_posFdback_dKalmanFtr[i].FilteredResCal(rArm_pos_fdback[i]);
        rArm_IqFdback_Ftred[i] = rArm_IqFdback_dKalmanFtr[i].FilteredResCal(rArm_iq_fdback[i]);
    }

    //执行数据转换操作
    for(int i = 0; i < LARMDOF; i++){
        lArm_TorFdback_Ftred[i] = lArm_IqFdback_Ftred[i]/lArm_JointKt[i];
    }
    for(int i = 0; i < RARMDOF; i++){
        rArm_TorFdback_Ftred[i] = rArm_IqFdback_Ftred[i]/rArm_JointKt[i];
    }
}

//------------------------------
//重力补偿相关计算函数

void ArmCtlNode::gravCompInit(){
    //重力补偿初始化 -- 注意该全局重力补偿函数只能在程序初始化阶段调用一次，注意必须与求解器同步初始化
    if(ifGravCompInit){
        ifGravCompInit = false;

        //初始化重力补偿求解器 -- 采用零位初始化重力补偿
        KDL::JntArray l_stPosArr, r_stPosArr;
        l_stPosArr = vec_to_JntArray(lArm_zeroPos);
        r_stPosArr = vec_to_JntArray(rArm_zeroPos);
        gravComp_floatBase.init_GravComp(
            waist_desc_pos,
            l_stPosArr, r_stPosArr
        );

        //初始化重力补偿缓输出控制
        smoothOutp.smootherInit(ctl_GravCompStSmoothTimes, ctl_GravCompStZeroTimes);
    }
}

TorCompWrap ArmCtlNode::gravCompCalOutp(
    std::vector<double> waist_pos, std::vector<double> waist_spd,
    std::vector<double> lArm_Jnt_pos, std::vector<double> lArm_Jnt_vel,
    std::vector<double> rArm_Jnt_pos, std::vector<double> rArm_Jnt_vel,
    int compMode        //补偿模式 -- 是采用纯重力补偿还是采用综合的动力学补偿（0 -- 纯重力补偿， 1 -- 完整动力学补偿）
){
    std::vector<double> lArm_JntVelCor, rArm_JntVelCor;
    if(ifTrigger){
        lArm_JntVelCor.assign(LARMDOF, 0.0);
        rArm_JntVelCor.assign(RARMDOF, 0.0);

        zeroSpeed_count += 1;
        if(zeroSpeed_count > zeroSpeed_trajLen || zeroSpeed_count == zeroSpeed_trajLen){
            ifTrigger = false;
        }
    }else{
        lArm_JntVelCor = lArm_Jnt_vel;
        rArm_JntVelCor = rArm_Jnt_vel;
    }

    //重力补偿计算
    TorCompWrap dyCompRes;
    if(compMode == 0){
        //纯重力补偿
        dyCompRes = gravComp_floatBase.onlyGravComp(waist_pos, waist_spd, lArm_Jnt_pos, lArm_JntVelCor, rArm_Jnt_pos, rArm_JntVelCor);
    }else{
        //完整的动力学补偿
        dyCompRes = gravComp_floatBase.GravCompOutp(waist_pos, waist_spd, lArm_Jnt_pos, lArm_JntVelCor,
            rArm_Jnt_pos, rArm_JntVelCor, gravInpMode == 1 && !ifTrigger ? stream_acceleration_ : std::vector<double>{});
    }

    //重力补偿修正
    TorCompWrap dyCompRes_revise = dyCompRes;
    double integer_revise = smoothOutp.smootherOutp();
    for(int i = 0; i < dyCompRes.lArm_Comp_grav.size(); i++){
        dyCompRes_revise.lArm_Comp_grav[i] = integer_revise * dyCompRes.lArm_Comp_grav[i];
        dyCompRes_revise.lArm_Comp_InteriaComb[i] = integer_revise * dyCompRes.lArm_Comp_InteriaComb[i];
        dyCompRes_revise.lArm_feedForward[i] = integer_revise * dyCompRes.lArm_feedForward[i];
    }
    for(int i = 0; i < dyCompRes.rArm_comp_grav.size(); i++){
        dyCompRes_revise.rArm_comp_grav[i] = integer_revise * dyCompRes.rArm_comp_grav[i];
        dyCompRes_revise.rArm_Comp_InteriaComb[i] = integer_revise * dyCompRes.rArm_Comp_InteriaComb[i];
        dyCompRes_revise.rArm_feedForward[i] = integer_revise * dyCompRes.rArm_feedForward[i];
    }

    return dyCompRes_revise;
}

//包装函数 -- 重力补偿传参与输出包装
TorCompWrap ArmCtlNode::gravCompCalWrap(){
    std::vector<double> lArm_jointPos_Inp, rArm_jointPos_Inp;
    std::vector<double> lArm_jointVel_Inp, rArm_jointVel_Inp;
    lArm_jointPos_Inp.assign(LARMDOF, 0.0);
    rArm_jointPos_Inp.assign(RARMDOF, 0.0);
    lArm_jointVel_Inp.assign(LARMDOF, 0.0);
    rArm_jointVel_Inp.assign(RARMDOF, 0.0);

    if(gravInpMode == 1){
        //全部采用目标数据作为输入
        lArm_jointPos_Inp = lArm_joint_pos;
        rArm_jointPos_Inp = rArm_joint_pos;
        lArm_jointVel_Inp = lArm_joint_spd;
        rArm_jointVel_Inp = rArm_joint_spd;
    }else if(gravInpMode == 2){
        //全部采用反馈值作为输入
        lArm_jointPos_Inp = lArm_pos_fdback;
        rArm_jointPos_Inp = rArm_pos_fdback;
        lArm_jointVel_Inp = lArm_spd_fdback;
        rArm_jointVel_Inp = rArm_spd_fdback;
    }else{
        //仅位置采用反馈值作为输入
        lArm_jointPos_Inp = lArm_pos_fdback;
        rArm_jointPos_Inp = rArm_pos_fdback;
        lArm_jointVel_Inp = lArm_joint_spd;
        rArm_jointVel_Inp = rArm_joint_spd;
    }

    //针对补偿模式进行处理
    int compMod = 0;        //默认纯重力补偿
    if(modCtlMsg_get.transMod == 5){
        //处于零力拖动模式
        compMod = 0;
    }else{
        compMod = 1;
    }

    //机器人基座必须采用反馈值作为输入
    TorCompWrap mRes;
    mRes = gravCompCalOutp(
        waist_desc_pos, waist_desc_spd,
        lArm_jointPos_Inp, lArm_jointVel_Inp,
        rArm_jointPos_Inp, rArm_jointVel_Inp,
        compMod
    );

    return mRes;
}

//------------------------------
//MDP接口函数定义

//MDP插补求解器
void ArmCtlNode::MDP_runin(){
    //先计算轨迹长度
    MDP_trajLen = std::round(modCtlMsg_get.upperMonitor_time / TEMPTIME);
    MDP_tempTimes = 0;
    
    std::vector<double> MDP_posOutp_last_l, MDP_posOutp_last_r;
    std::vector<double> MDP_velOutp_last_l, MDP_velOutp_last_r;

    MDP_posOutp_last_l = MDP_posOutp_lArm.back();
    MDP_posOutp_last_r = MDP_posOutp_rArm.back();

    MDP_velOutp_last_l = MDP_velOutp_lArm.back();
    MDP_velOutp_last_r = MDP_velOutp_rArm.back();

    MDP_posOutp_lArm.clear();
    MDP_posOutp_lArm.resize(MDP_trajLen);
    MDP_velOutp_lArm.clear();
    MDP_velOutp_lArm.resize(MDP_trajLen);

    MDP_posOutp_rArm.clear();
    MDP_posOutp_rArm.resize(MDP_trajLen);
    MDP_velOutp_rArm.clear();
    MDP_velOutp_rArm.resize(MDP_trajLen);

    std::vector<double> derPos_l, derPos_r;
    std::vector<double> derVel_l, derVel_r;

    //global_debugPub->debug_msg_publish_1(lArm_desPos_JntSpace, rArm_desPos_JntSpace);

    for(int i = 0; i < LARMDOF; i++){
        derPos_l.push_back((lArm_desPos_JntSpace[i] - MDP_posOutp_last_l[i])/static_cast<double>(MDP_trajLen));
        derVel_l.push_back((lArm_desVel_MDP_JntSpace[i] - MDP_velOutp_last_l[i])/static_cast<double>(MDP_trajLen));
    }
    for(int i = 0; i < RARMDOF; i++){
        derPos_r.push_back((rArm_desPos_JntSpace[i] - MDP_posOutp_last_r[i])/static_cast<double>(MDP_trajLen));
        derVel_r.push_back((rArm_desVel_MDP_JntSpace[i] - MDP_velOutp_last_r[i])/static_cast<double>(MDP_trajLen)); 
    }

    for(int i = 0; i < MDP_trajLen; i++){
        MDP_posOutp_lArm[i].resize(LARMDOF);
        MDP_velOutp_lArm[i].resize(LARMDOF);
        MDP_posOutp_rArm[i].resize(RARMDOF);
        MDP_velOutp_rArm[i].resize(RARMDOF);

        for(int j = 0; j < LARMDOF; j++){
            MDP_posOutp_lArm[i][j] = MDP_posOutp_last_l[j] + (i+1)*derPos_l[j];
            MDP_velOutp_lArm[i][j] = MDP_velOutp_last_l[j] + (i+1)*derVel_l[j];
        }
        for(int j = 0; j < RARMDOF; j++){
            MDP_posOutp_rArm[i][j] = MDP_posOutp_last_r[j] + (i+1)*derPos_r[j];
            MDP_velOutp_rArm[i][j] = MDP_velOutp_last_r[j] + (i+1)*derVel_r[j];
        }
    }

    //控制变量修改
    lArm_ifReachEnd.assign(LARMDOF, false);
    rArm_ifReachEnd.assign(RARMDOF, false);
}

//MDP输出函数
MDP_trans ArmCtlNode::MDP_outp(){
    MDP_trans mRes;

    if(MDP_tempTimes < MDP_trajLen){
        //正常输出
        mRes.lArm_pos = MDP_posOutp_lArm[MDP_tempTimes];
        mRes.lArm_spd = MDP_velOutp_lArm[MDP_tempTimes];
        mRes.rArm_pos = MDP_posOutp_rArm[MDP_tempTimes];
        mRes.rArm_spd = MDP_velOutp_rArm[MDP_tempTimes];

        if(MDP_tempTimes == (MDP_trajLen - 1)){
            lArm_ifReachEnd.assign(LARMDOF, true);
            rArm_ifReachEnd.assign(RARMDOF, true);
        }
        MDP_tempTimes += 1;
    }else{
        mRes.lArm_pos = MDP_posOutp_lArm.back();
        mRes.lArm_spd = MDP_velOutp_lArm.back();
        mRes.rArm_pos = MDP_posOutp_rArm.back();
        mRes.rArm_spd = MDP_velOutp_rArm.back();

        lArm_ifReachEnd.assign(LARMDOF, true);
        rArm_ifReachEnd.assign(RARMDOF, true);
    }

    // 关节静止判定：若 ifJntStab 为 1，强制输出上一轮目标位置，速度置零
    for(int i = 0; i < LARMDOF; i++){
        if(modCtlMsg_get.ifJntStab[0][i] == 1){
            mRes.lArm_pos[i] = MDP_lastPos_lArm[i];
            mRes.lArm_spd[i] = 0.0;
        }else{
            double tempPos = mRes.lArm_pos[i];
            MDP_lastPos_lArm[i] = tempPos;
        }
    }
    for(int i = 0; i < RARMDOF; i++){
        if(modCtlMsg_get.ifJntStab[1][i] == 1){
            mRes.rArm_pos[i] = MDP_lastPos_rArm[i];
            mRes.rArm_spd[i] = 0.0;
        }else{
            double tempPos = mRes.rArm_pos[i];
            MDP_lastPos_rArm[i] = tempPos;
        }
    }

    return mRes;
}

//回调函数
void ArmCtlNode::set_joint_angles(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                        std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
    if (request->data) {
        // 这里可以添加设置关节角度的逻辑
        RCLCPP_INFO(this->get_logger(), "Received request to set joint angles");
        response->success = true;
        response->message = "Joint angles set successfully";
    } else {
        response->success = false;
        response->message = "Request not confirmed";
    }
}

void ArmCtlNode::message_callback(const ctl_msgs_hw::msg::RetInformation::SharedPtr msg) {

    // 检查时间戳是否合理
    auto now = this->now();
    auto latency = now - msg->header.stamp;
    if (latency.seconds() > 0.05) {
        RCLCPP_WARN(this->get_logger(), "High latency detected: %f seconds", latency.seconds());
    }

    for(int i = 0; i < LARMDOF; i++){
        lArm_pos_fdback[i] = msg->l_position[i];
        lArm_spd_fdback[i] = msg->l_speed[i];
        lArm_iq_fdback[i] = msg->l_iq[i];
    }

    for(int i = 0; i < RARMDOF; i++){
        rArm_pos_fdback[i] = msg->r_position[i];
        rArm_spd_fdback[i] = msg->r_speed[i];
        rArm_iq_fdback[i] = msg->r_iq[i];
    }

    this->has_received_command_ = true;
}

//从上位机获取的控制数据
void ArmCtlNode::upperMonitor_msg_callback(const ctl_msgs_hw::msg::MidNodeCtlMsgSend::SharedPtr msg,
                                          const rclcpp::MessageInfo& info)
{
    const auto& gid = info.get_rmw_message_info().publisher_gid.data;
    const ControlSource source{{std::begin(gid), std::end(gid)}, msg->ctl_trans_mod, msg->runin_mod,
        msg->bridge_joint_stream, msg->derivatives_valid, msg->rigidity_ctl_mod_l, msg->rigidity_ctl_mod_r};
    if (msg->bridge_joint_stream) {
        const auto valid = [](const auto& values) {
            return values.size() == 7 && std::all_of(values.begin(), values.end(),
                [](double v) { return std::isfinite(v); });
        };
        if (!std::isfinite(msg->upper_monitor_time) || msg->upper_monitor_time < TEMPTIME * 0.99 ||
            msg->upper_monitor_time > 0.25 || !valid(msg->l_arm_des_pos_jntspace) ||
            !valid(msg->r_arm_des_pos_jntspace) || (msg->derivatives_valid &&
            (!valid(msg->l_arm_des_vel_jntspace) || !valid(msg->r_arm_des_vel_jntspace) ||
             !valid(msg->l_arm_des_acc) || !valid(msg->r_arm_des_acc)))) {
            RCLCPP_ERROR(get_logger(), "Rejected malformed bridge trajectory sample");
            return;
        }
        if (!control_handoff_.changed(source) && stream_sample_ &&
            stream_sample_->trajectory_header.stamp == msg->trajectory_header.stamp) return;
        stream_sample_ = msg;
    } else {
        stream_sample_.reset();
    }
    //----------综合模式控制信息解包----------

    RCLCPP_DEBUG(this->get_logger(), "upper monitor data get success");

    if(ifInitTrigger){
        ifTrigger = true;
        ifInitTrigger = false;
    }

    //数据传输模式
    modCtlMsg_get.transMod = msg->ctl_trans_mod;
    modCtlMsg_get.runinMod = msg->runin_mod;

    //求解模式
    modCtlMsg_get.ifUseRet = msg->if_use_ret;
    modCtlMsg_get.ifUseTemp = msg->if_use_temp;

    for(int i = 0; i < LARMDOF; i++){
        modCtlMsg_get.base_rigiditiCtl_mod_l[i] = msg->rigidity_ctl_mod_l[i];
        modCtlMsg_get.base_rigiditiCtl_mod_r[i] = msg->rigidity_ctl_mod_r[i];

        base_rigidityCtl_mod_l[i] = msg->rigidity_ctl_mod_l[i];
        base_rigidityCtl_mod_r[i] = msg->rigidity_ctl_mod_r[i];
    }
    //std::cout << "base_rigidityCtl_mod_l[0] = " <<  << std::endl;

    //插补信息包装
    modCtlMsg_get.upperMonitor_time = msg->upper_monitor_time;
    modCtlMsg_get.TrajTime.resize(2);
    modCtlMsg_get.accRateBase.resize(2);
    modCtlMsg_get.m_stabMsg.resize(2);

    modCtlMsg_get.TrajTime[0].assign(LARMDOF, 0.0);
    modCtlMsg_get.accRateBase[0].assign(LARMDOF, 0.0);
    modCtlMsg_get.m_stabMsg[0].resize(LARMDOF);

    modCtlMsg_get.TrajTime[1].assign(RARMDOF, 0.0);
    modCtlMsg_get.accRateBase[1].assign(RARMDOF, 0.0);
    modCtlMsg_get.m_stabMsg[1].resize(RARMDOF);

    for(int i = 0; i < LARMDOF; i++){
        modCtlMsg_get.TrajTime[0][i] = msg->traj_time_l_arm[i];
        modCtlMsg_get.accRateBase[0][i] = msg->acc_rate_base_l_arm[i];

        modCtlMsg_get.m_stabMsg[0][i].ifUseStab = msg->l_arm_if_use_stab[i];
        modCtlMsg_get.m_stabMsg[0][i].des_stabTimes = msg->l_arm_des_stab_times[i];
        modCtlMsg_get.m_stabMsg[0][i].start_stabTimes = msg->l_arm_start_stab_times[i];
    }

    for(int i = 0; i < RARMDOF; i++){
        modCtlMsg_get.TrajTime[1][i] = msg->traj_time_r_arm[i];
        modCtlMsg_get.accRateBase[1][i] = msg->acc_rate_base_r_arm[i];

        modCtlMsg_get.m_stabMsg[1][i].ifUseStab = msg->r_arm_if_use_stab[i];
        modCtlMsg_get.m_stabMsg[1][i].start_stabTimes = msg->r_arm_start_stab_times[i];
        modCtlMsg_get.m_stabMsg[1][i].des_stabTimes = msg->r_arm_des_stab_times[i];
    }

    //控制量包装 -- 各个关节是否使用静止输出模式
    modCtlMsg_get.ifJntStab.resize(2);
    modCtlMsg_get.ifJntStab[0].assign(LARMDOF, 0);
    modCtlMsg_get.ifJntStab[1].assign(RARMDOF, 0);
    for(int i = 0; i < LARMDOF; i++){
        modCtlMsg_get.ifJntStab[0][i] = msg->l_arm_if_jnt_stab[i];
        modCtlMsg_get.ifJntStab[1][i] = msg->r_arm_if_jnt_stab[i];
    }

    ifUpdateLim_get = msg->if_update_lim;
    ifRedefineJntTrajStab_get = msg->if_redefine_stab_jnt_ang;

    //----------关节控制信息参数解包----------

    int lArmLimJntNumGet = msg->l_arm_jntlim_num;
    int rArmLimJntNumGet = msg->r_arm_jntlim_num;
    int allArmLimJntNumGet = lArmLimJntNumGet + rArmLimJntNumGet;

    //左臂
    lArm_LimMsg_get.ifUseJntLim = msg->l_arm_if_use_jntlim;
    lArm_LimMsg_get.limJntNum = msg->l_arm_jntlim_num;
    lArm_LimMsg_get.limJntLabel.resize(msg->l_arm_jntlim_num);
    lArm_LimMsg_get.limJntAng.resize(msg->l_arm_jntlim_num);
    for(int i = 0; i < lArmLimJntNumGet; i++){
        lArm_LimMsg_get.limJntLabel[i] = msg->l_arm_limjnt_label[i];
        lArm_LimMsg_get.limJntAng[i] = msg->l_arm_limjnt_ang[i];
    }

    //右臂
    rArm_LimMsg_get.ifUseJntLim = msg->r_arm_if_use_jntlim;
    rArm_LimMsg_get.limJntNum = msg->r_arm_jntlim_num;
    rArm_LimMsg_get.limJntLabel.resize(msg->r_arm_jntlim_num);
    rArm_LimMsg_get.limJntAng.resize(msg->r_arm_jntlim_num);
    for(int i = 0; i < rArmLimJntNumGet; i++){
        rArm_LimMsg_get.limJntLabel[i] = msg->r_arm_limjnt_label[i];
        rArm_LimMsg_get.limJntAng[i] = msg->r_arm_limjnt_ang[i];
    }

    //双臂 -- 将左右臂信息依次装入到双臂中
    allArm_LimMsg_get.ifUseJntLim = lArm_LimMsg_get.ifUseJntLim || rArm_LimMsg_get.ifUseJntLim;
    allArm_LimMsg_get.limJntNum = allArmLimJntNumGet;
    allArm_LimMsg_get.limJntLabel.resize(allArmLimJntNumGet);
    allArm_LimMsg_get.limJntAng.resize(allArmLimJntNumGet);
    int m_temp = 0;
    if(lArm_LimMsg_get.ifUseJntLim){
        for(int i = 0; i < lArmLimJntNumGet; i++){
            allArm_LimMsg_get.limJntLabel[m_temp] = lArm_LimMsg_get.limJntLabel[i];
            allArm_LimMsg_get.limJntAng[m_temp] = lArm_LimMsg_get.limJntAng[i];
            m_temp += 1;
        }
    }
    if(rArm_LimMsg_get.ifUseJntLim){
        for(int i = 0; i < rArmLimJntNumGet; i++){
            allArm_LimMsg_get.limJntLabel[m_temp] = rArm_LimMsg_get.limJntLabel[i];
            allArm_LimMsg_get.limJntAng[m_temp] = rArm_LimMsg_get.limJntAng[i];
            m_temp += 1;
        }
    }

    //----------关节目标位置等目标量信息----------
    
    for(int i = 0; i < 6; i++){
        lArm_desPos_descartes[i] = msg->l_arm_des_pos_descartes[i];
        rArm_desPos_descartes[i] = msg->r_arm_des_pos_descartes[i];
    }

    for(int i = 0; i < LARMDOF; i++){
        lArm_desPos_JntSpace[i] = msg->l_arm_des_pos_jntspace[i];
    }
    for(int i = 0; i < RARMDOF; i++){
        rArm_desPos_JntSpace[i] = msg->r_arm_des_pos_jntspace[i];
    }

    for(int i = 0; i < LARMDOF; i++){
        lArm_Kp_[i] = msg->kp_trans_l[i];
        lArm_Kd_[i] = msg->kd_trans_l[i];
    }
    for(int i = 0; i < RARMDOF; i++){
        rArm_Kp_[i] = msg->kp_trans_r[i];
        rArm_Kd_[i] = msg->kd_trans_r[i];
    }

    for(int i = 0; i < LARMDOF; i++){
        lArm_desVel_MDP_JntSpace[i] = msg->l_arm_des_vel_jntspace[i];
        rArm_desVel_MDP_JntSpace[i] = msg->r_arm_des_vel_jntspace[i];
    }

    if (control_handoff_.observe(source)) {
        ifFirstTemp = true;
        RCLCPP_INFO(get_logger(), "Upper controller handoff epoch=%llu mode=%d runin=%d; servo transition requested",
                    static_cast<unsigned long long>(control_handoff_.value()), msg->ctl_trans_mod, msg->runin_mod);
    }
    has_received_upperMonitor_command_ = true;
}

//===================================================================================

EndPos_plot::EndPos_plot(rclcpp::Node* node): node_ptr_(node){
    //初始化消息发送变量：
    pvTrajPub = node_ptr_->create_publisher<ctl_msgs_hw::msg::EndDescPub>(
        "/feedback_get/end_desc_traj_pub",
        30
    );

    temp_time = TEMPTIME;
    //初始化成员变量：
    lArm_DescPos_last.assign(LARMDOF, 0.0);
    rArm_DescPos_last.assign(RARMDOF, 0.0);
}

void EndPos_plot::end_pos_plot(
    std::vector<double> lArm_desDescPos, std::vector<double> rArm_desDescPos,
    std::vector<double> lArm_retDescPos, std::vector<double> rArm_retDescPos
){
    auto tempPV_pub = ctl_msgs_hw::msg::EndDescPub();

    //发送末端位置
    for(int i = 0; i < 6; i++){
        tempPV_pub.l_desc_pos_des[i] = lArm_desDescPos[i];
        tempPV_pub.r_desc_pos_des[i] = rArm_desDescPos[i];

        tempPV_pub.l_desc_pos_ret[i] = lArm_retDescPos[i];
        tempPV_pub.r_desc_pos_ret[i] = rArm_retDescPos[i];
    }

    //发送末端速度
    for(int i = 0; i < 6; i++){
        tempPV_pub.l_desc_vel_ret[i] = (lArm_retDescPos[i] - lArm_DescPos_last[i]) / temp_time;
        tempPV_pub.r_desc_vel_ret[i] = (rArm_retDescPos[i] - rArm_DescPos_last[i]) / temp_time;
    }

    //发送末端综合线速度大小
    double l_linerVel, r_linerVel;
    l_linerVel = std::sqrt(std::pow(tempPV_pub.l_desc_vel_ret[0], 2) + std::pow(tempPV_pub.l_desc_vel_ret[1], 2) + std::pow(tempPV_pub.l_desc_vel_ret[2], 2));
    r_linerVel = std::sqrt(std::pow(tempPV_pub.r_desc_vel_ret[0], 2) + std::pow(tempPV_pub.r_desc_vel_ret[1], 2) + std::pow(tempPV_pub.r_desc_vel_ret[2], 2));

    //更新上一轮位置
    lArm_DescPos_last = lArm_retDescPos;
    rArm_DescPos_last = rArm_retDescPos;

    tempPV_pub.l_desc_liner_vel = l_linerVel;
    tempPV_pub.r_desc_liner_vel = r_linerVel;

    pvTrajPub->publish(tempPV_pub);
}
