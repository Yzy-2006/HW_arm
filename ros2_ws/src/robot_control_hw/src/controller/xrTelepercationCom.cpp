#include"controller/xrTelepercationCom.h"
#include"robot_control/config.h"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>
#include <Eigen/Geometry>
#include <iomanip>
#include <sstream>

namespace {

constexpr std::size_t kCartesianAxes = 3;

std::vector<double> tool_swing_increment(
    const std::vector<double>& delta, const std::vector<double>& accumulated,
    const std::vector<double>& base_rpy, bool lock_twist){
    const auto dq = eulerToQuaternionZYX(delta);
    if(!lock_twist){
        return dq;
    }
    const auto base = eulerToQuaternionZYX({base_rpy[2], base_rpy[1], base_rpy[0]});
    const auto current = multiplyQuaternions(accumulated, base);
    const Eigen::Quaterniond orientation(current[0], current[1], current[2], current[3]);
    const Eigen::Quaterniond increment(dq[0], dq[1], dq[2], dq[3]);
    // Both URDF tool tips extend along local -Z.  Decompose the world-frame
    // increment as swing * twist about that current tool axis, then keep swing.
    const Eigen::Vector3d axis = orientation.normalized() * Eigen::Vector3d::UnitZ();
    const Eigen::Vector3d projection = axis * increment.vec().dot(axis);
    Eigen::Quaterniond twist(increment.w(), projection.x(), projection.y(), projection.z());
    if(twist.norm() < 1e-12){
        return dq;  // A 180-degree perpendicular swing has no defined twist.
    }
    const Eigen::Quaterniond swing = (increment * twist.normalized().conjugate()).normalized();
    return {swing.w(), swing.x(), swing.y(), swing.z()};
}

bool read_limit_vector(const YAML::Node& arm_config, const char* key, std::vector<double>& value){
    const YAML::Node node = arm_config[key];
    if(!node || !node.IsSequence() || node.size() != kCartesianAxes){
        return false;
    }
    std::vector<double> parsed(kCartesianAxes, 0.0);
    try{
        for(std::size_t i = 0; i < kCartesianAxes; ++i){
            parsed[i] = node[i].as<double>();
            if(!std::isfinite(parsed[i])){
                return false;
            }
        }
    } catch(const YAML::Exception&){
        return false;
    }
    value = parsed;
    return true;
}

bool valid_limits(const std::vector<double>& minimum, const std::vector<double>& maximum){
    if(minimum.size() != kCartesianAxes || maximum.size() != kCartesianAxes){
        return false;
    }
    for(std::size_t i = 0; i < kCartesianAxes; ++i){
        if(!std::isfinite(minimum[i]) || !std::isfinite(maximum[i]) || minimum[i] > maximum[i]){
            return false;
        }
    }
    return true;
}

bool read_finite_scalar(const YAML::Node& node, const char* key, double& value){
    const YAML::Node scalar = node[key];
    if(!scalar || !scalar.IsScalar()){
        return false;
    }
    try{
        const double parsed = scalar.as<double>();
        if(!std::isfinite(parsed)){
            return false;
        }
        value = parsed;
        return true;
    } catch(const YAML::Exception&){
        return false;
    }
}

bool read_nonnegative_integer(const YAML::Node& node, const char* key, int& value){
    const YAML::Node scalar = node[key];
    if(!scalar || !scalar.IsScalar()){
        return false;
    }
    try{
        const int parsed = scalar.as<int>();
        if(parsed < 0){
            return false;
        }
        value = parsed;
        return true;
    } catch(const YAML::Exception&){
        return false;
    }
}

bool read_boolean(const YAML::Node& node, const char* key, bool& value){
    const YAML::Node scalar = node[key];
    if(!scalar || !scalar.IsScalar()){
        return false;
    }
    try{
        value = scalar.as<bool>();
        return true;
    } catch(const YAML::Exception&){
        return false;
    }
}

bool clamp_vector(std::vector<double>& values,
                  const std::vector<double>& minimum,
                  const std::vector<double>& maximum){
    bool limited = false;
    for(std::size_t i = 0; i < kCartesianAxes; ++i){
        const double clamped = std::clamp(values[i], minimum[i], maximum[i]);
        limited = limited || clamped != values[i];
        values[i] = clamped;
    }
    return limited;
}

std::vector<double> inverse_unit_quaternion(const std::vector<double>& quaternion){
    return {quaternion[0], -quaternion[1], -quaternion[2], -quaternion[3]};
}

} // namespace

std::vector<double> dbg_reserve(7, 0.0);

//------------------------------------------------------------
//辅助类：插补器
//点位插补
TrajResult stTraj_runiner::dPos_queue_stabTime_dot(float mposDes, float mTrajTime, float accTimeRateBase)
{
    //轨迹一半的时间用来加速
    //由曲线推到可得，当加速过程时间占到时间的一半时，按照钟型曲线加速过程和匀速过程走过距离比例为1：2

    float t_half = mTrajTime / 2;
    float posDes_half = mposDes / 2;

    //计算半切轨迹及其增量
    std::vector<float> acc_traj;
    std::vector<float> acc_vel;

    //轨迹参数计算a, b, 和v_max
    if (accTimeRateBase <= 1.0) {
        std::cerr << "accTimeRateBase must be greater than 0.5 to avoid division by zero" << std::endl;
        accTimeRateBase = 1.05; // 设置默认安全值
    }
    float t_acc = t_half / accTimeRateBase;
    float t_stab_half = t_half - t_acc;
    float denominator = std::pow(t_half, 4) * (2*accTimeRateBase - 1);
    float a = -12 * std::pow(accTimeRateBase, 4) * posDes_half / denominator;
    float b = a * t_half / accTimeRateBase;
    //float v_max = (1.0/3.0)*a*std::pow(t_acc, 3) - (1.0/2.0)*b*std::pow(t_acc,2);

    float acc_moveLen = (1.0/12)*a*std::pow(t_acc, 4) - (1.0/6)*b*std::pow(t_acc, 3);
    //计算加减速段轨迹
    float temp_pos_lst = 0;
    float temp_T = 0;
    while (true){
        float temp_pos;
        if(temp_T < t_acc || temp_T == t_acc){
            temp_pos = (1.0/12)*a*std::pow(temp_T, 4) - (1.0/6)*b*std::pow(temp_T, 3);

            acc_traj.push_back(temp_pos);
            acc_vel.push_back((temp_pos - temp_pos_lst)/TempTime);

            temp_pos_lst = temp_pos;
            temp_T = temp_T + TempTime;
        }else{
            break;
        }
    }

    //计算匀速阶段轨迹 -- 修改以补偿
    float stabVel_trajLen = mposDes - 2*acc_traj.back();
    float stabVel_time = mTrajTime - 2*(temp_T - TempTime);
    int stabVelTrmpTimes = std::round(stabVel_time/TempTime);
    float stabVel_derPos = stabVel_trajLen / (stabVelTrmpTimes);
    float stabVel = stabVel_trajLen / (stabVelTrmpTimes * TempTime);

    //据此计算整个轨迹
    std::vector<float> whole_traj;
    std::vector<float> whole_traj_vel;

    //输入加速段轨迹
    for(int i = 0; i < acc_traj.size(); i++){
        whole_traj.push_back(acc_traj[i]);
        whole_traj_vel.push_back(acc_vel[i]);
    }

    //输入匀速段轨迹
    float temp_pos_2 = acc_traj.back();
    for(int i = 0; i < stabVelTrmpTimes; i++){
        temp_pos_2 += stabVel_derPos;
        whole_traj.push_back(temp_pos_2);
        whole_traj_vel.push_back(stabVel);
    }

    //输入减速段轨迹
    for(int i = 0; i < acc_traj.size(); i++){
        float element = *std::next(acc_traj.rbegin(), i);
        float element_vel = *std::next(acc_vel.rbegin(), i);
        float m_temp_pos = mposDes - element;

        whole_traj.push_back(m_temp_pos);
        whole_traj_vel.push_back(element_vel);
    }

    TrajResult mRes;
    mRes.positionIncrements = whole_traj;
    mRes.VelocityTraj = whole_traj_vel;

    //获取最后一轮的位置和速度
    lastTempPosOutp = mposDes;
    lastTempVelOutp = 0.0f;

    return mRes;
}

std::pair<float, float> stTraj_runiner::lastPosVelGet(float posSt)
{
    float posOutpCor = this->lastTempPosOutp + posSt;
    std::pair<float, float> mRes = {posOutpCor, this->lastTempVelOutp};
    return mRes;
}

//--------------------------------------------------
//通讯核心类：

std::vector<double> quaternionToEuler(const std::vector<double>& quaternion);

static void unwrapEulerNear(std::vector<double>& angles,
                            const std::vector<double>& previous){
    if(angles.size() != 3 || previous.size() != 3){
        return;
    }
    for(size_t i = 0; i < 3; ++i){
        while(angles[i] - previous[i] > M_PI){ angles[i] -= 2.0 * M_PI; }
        while(angles[i] - previous[i] < -M_PI){ angles[i] += 2.0 * M_PI; }
    }
}

void xr_telepercation_ctl::load_teleoperation_limits(){
    // Conservative defaults keep IK targets within the nominal dual-arm workspace.
    l_pos_min = {-0.50, -0.10, -0.90};
    l_pos_max = { 0.70,  0.80,  0.20};
    r_pos_min = {-0.50, -0.80, -0.90};
    r_pos_max = { 0.80,  0.10,  0.30};
    l_rpy_min = {-1.20, -1.20, -1.57};
    l_rpy_max = { 1.20,  1.20,  1.57};
    r_rpy_min = l_rpy_min;
    r_rpy_max = l_rpy_max;
    position_limit_enabled = true;
    orientation_limit_enabled = true;
    joint_return_trajectory_time_sec = 6.0;
    joint_return_acceleration_time_ratio = 1.10;
    joint_return_start_stable_cycles = 15;
    joint_return_end_stable_cycles = 25;

    try{
        const std::string path = ament_index_cpp::get_package_share_directory("ctl_msgs_hw") +
            "/config/controllerConfig.yaml";
        const YAML::Node root = YAML::LoadFile(path);
        const YAML::Node orientation_control = root["vrOrientationControl"];
        if(orientation_control && orientation_control["lock_tool_twist"] &&
           !read_boolean(orientation_control, "lock_tool_twist", lock_tool_twist)){
            std::cerr << "[遥操作] lock_tool_twist 配置无效，保持工具轴自转锁定" << std::endl;
            lock_tool_twist = true;
        }
        const YAML::Node limits = root["vrTeleoperationLimit"];
        if(!limits){
            std::cerr << "[遥操作] 未配置 vrTeleoperationLimit，使用内置安全限位" << std::endl;
            return;
        }
        if(limits["position_enabled"] &&
           !read_boolean(limits, "position_enabled", position_limit_enabled)){
            std::cerr << "[遥操作] position_enabled 配置无效，使用位置限位" << std::endl;
            position_limit_enabled = true;
        }
        if(limits["orientation_enabled"] &&
           !read_boolean(limits, "orientation_enabled", orientation_limit_enabled)){
            std::cerr << "[遥操作] orientation_enabled 配置无效，使用 RPY 姿态限位" << std::endl;
            orientation_limit_enabled = true;
        }
        auto load_arm = [](const YAML::Node& node,
                           std::vector<double>& pos_min, std::vector<double>& pos_max,
                           std::vector<double>& rpy_min, std::vector<double>& rpy_max,
                           bool position_limit_enabled,
                           bool orientation_limit_enabled,
                           const char* name){
            std::vector<double> candidate_pos_min, candidate_pos_max;
            std::vector<double> candidate_rpy_min, candidate_rpy_max;
            const bool valid_position_limits = !position_limit_enabled ||
                (read_limit_vector(node, "position_min", candidate_pos_min) &&
                 read_limit_vector(node, "position_max", candidate_pos_max) &&
                 valid_limits(candidate_pos_min, candidate_pos_max));
            const bool valid_orientation_limits = !orientation_limit_enabled ||
                (read_limit_vector(node, "rpy_min", candidate_rpy_min) &&
                 read_limit_vector(node, "rpy_max", candidate_rpy_max) &&
                 valid_limits(candidate_rpy_min, candidate_rpy_max));
            const bool valid = node && valid_position_limits && valid_orientation_limits;
            if(!valid){
                std::cerr << "[遥操作] " << name << "臂限位配置无效，使用内置安全限位" << std::endl;
                return;
            }
            if(position_limit_enabled){
                pos_min = candidate_pos_min;
                pos_max = candidate_pos_max;
            }
            if(orientation_limit_enabled){
                rpy_min = candidate_rpy_min;
                rpy_max = candidate_rpy_max;
            }
        };
        load_arm(limits["left_arm"], l_pos_min, l_pos_max, l_rpy_min, l_rpy_max,
                 position_limit_enabled, orientation_limit_enabled, "左");
        load_arm(limits["right_arm"], r_pos_min, r_pos_max, r_rpy_min, r_rpy_max,
                 position_limit_enabled, orientation_limit_enabled, "右");
        const YAML::Node return_config = root["vrJointReturn"];
        if(return_config){
            double trajectory_time = 0.0;
            double acceleration_ratio = 0.0;
            int start_stable_cycles = 0;
            int end_stable_cycles = 0;
            const bool valid_return_config =
                read_finite_scalar(return_config, "trajectory_time_sec", trajectory_time) &&
                read_finite_scalar(return_config, "acceleration_time_ratio", acceleration_ratio) &&
                read_nonnegative_integer(return_config, "start_stable_cycles", start_stable_cycles) &&
                read_nonnegative_integer(return_config, "end_stable_cycles", end_stable_cycles) &&
                trajectory_time >= 2.0 && trajectory_time <= 60.0 &&
                acceleration_ratio > 1.05 && acceleration_ratio <= 5.0 &&
                start_stable_cycles <= 500 && end_stable_cycles <= 500;
            if(valid_return_config){
                joint_return_trajectory_time_sec = trajectory_time;
                joint_return_acceleration_time_ratio = acceleration_ratio;
                joint_return_start_stable_cycles = start_stable_cycles;
                joint_return_end_stable_cycles = end_stable_cycles;
            }else{
                std::cerr << "[遥操作] vrJointReturn 配置无效，使用保守默认值" << std::endl;
            }
        }
        std::cout << "[遥操作] VR 限位：位置="
                  << (position_limit_enabled ? "开启" : "关闭")
                  << "，RPY 姿态="
                  << (orientation_limit_enabled ? "开启" : "关闭") << std::endl;
    } catch(const std::exception& exception){
        std::cerr << "[遥操作] 读取 VR 限位配置失败，使用内置安全限位: "
                  << exception.what() << std::endl;
    }
}

void xr_telepercation_ctl::limit_and_reanchor_target(
    std::vector<double>& target_rpy,
    std::vector<double>& position_delta,
    std::vector<double>& orientation_accumulator,
    const std::vector<double>& base_position,
    const std::vector<double>& base_rpy,
    const std::vector<double>& position_min,
    const std::vector<double>& position_max,
    const std::vector<double>& rpy_min,
    const std::vector<double>& rpy_max,
    bool position_limit_enabled,
    bool orientation_limit_enabled,
    bool& position_limit_active,
    bool& orientation_limit_active,
    const char* arm_name)
{
    const bool position_limited = position_limit_enabled &&
        clamp_vector(target_rpy, position_min, position_max);
    for(std::size_t i = 0; i < kCartesianAxes; ++i){
        // Do not retain movement past the boundary in the accumulated state.
        position_delta[i] = target_rpy[i] - base_position[i];
    }

    std::vector<double> target_orientation = {target_rpy[3], target_rpy[4], target_rpy[5]};
    const bool orientation_limited = orientation_limit_enabled &&
        clamp_vector(target_orientation, rpy_min, rpy_max);
    for(std::size_t i = 0; i < kCartesianAxes; ++i){
        target_rpy[i + 3] = target_orientation[i];
    }

    // world_orientation = accumulator * base_orientation. Rebuild accumulator
    // from the clamped target so subsequent reverse motion has no accumulated lag.
    const std::vector<double> base_quaternion =
        eulerToQuaternionZYX({base_rpy[2], base_rpy[1], base_rpy[0]});
    const std::vector<double> target_quaternion =
        eulerToQuaternionZYX({target_orientation[2], target_orientation[1], target_orientation[0]});
    orientation_accumulator = multiplyQuaternions(
        target_quaternion, inverse_unit_quaternion(base_quaternion));

    if(position_limited && !position_limit_active){
        std::cerr << "[遥操作] " << arm_name << "臂末端位置达到配置限位" << std::endl;
    }
    if(orientation_limited && !orientation_limit_active){
        std::cerr << "[遥操作] " << arm_name << "臂末端 RPY 姿态达到配置限位" << std::endl;
    }
    position_limit_active = position_limited;
    orientation_limit_active = orientation_limited;
}

void xr_telepercation_ctl::load_preset_pose(){
    preset_pose_valid = false;
    try{
        const auto path = ament_index_cpp::get_package_share_directory("ctl_msgs_hw") +
                          "/config/vr_preset_pose.yaml";
        const YAML::Node root = YAML::LoadFile(path);
        const std::vector<std::string> joints = {
            "shoulder_pitch_joint", "shoulder_roll_joint", "shoulder_yaw_joint",
            "elbow_joint", "wrist_roll_joint", "wrist_pitch_joint", "wrist_yaw_joint"
        };
        std::vector<double> left(joints.size()), right(joints.size());
        for(std::size_t i = 0; i < joints.size(); ++i){
            if(!read_finite_scalar(root["left_arm"], ("left_" + joints[i]).c_str(), left[i]) ||
               !read_finite_scalar(root["right_arm"], ("right_" + joints[i]).c_str(), right[i])){
                std::cerr << "[遥操作] vr_preset_pose.yaml 关节角缺失或无效，禁用右 B 预设姿态" << std::endl;
                return;
            }
        }
        if(left.size() != static_cast<std::size_t>(LARMDOF) ||
           right.size() != static_cast<std::size_t>(RARMDOF)){
            std::cerr << "[遥操作] 预设姿态关节数量与机械臂不符，禁用右 B" << std::endl;
            return;
        }
        preset_left_joints = left;
        preset_right_joints = right;
        preset_pose_valid = true;
    }catch(const std::exception& exception){
        std::cerr << "[遥操作] 无法加载 vr_preset_pose.yaml，禁用右 B: "
                  << exception.what() << std::endl;
    }
}

void xr_telepercation_ctl::init(){

    l_arm_end_pos.assign(3, 0.0);
    r_arm_end_pos.assign(3, 0.0);
    l_arm_quat_accum = {1.0, 0.0, 0.0, 0.0};
    r_arm_quat_accum = {1.0, 0.0, 0.0, 0.0};
    last_l_arm_zyx.assign(3, 0.0);
    last_r_arm_zyx.assign(3, 0.0);
    last_arm_zyx_valid = false;
    l_arm_end_delta.assign(3, 0.0);
    r_arm_end_delta.assign(3, 0.0);
    lArm_end_pos_rpy.assign(6, 0.0);
    rArm_end_pos_rpy.assign(6, 0.0);

    //初始化末端位姿偏置量
    st_pos_l.assign(3, 0.0);
    st_pos_r.assign(3, 0.0);
    st_gesture_l.assign(3, 0.0);
    st_gesture_r.assign(3, 0.0);
    for(int i = 0; i < 3; i++){
        st_pos_l[i] = l_descZeroPos[i];
        st_pos_r[i] = r_descZeroPos[i];

        st_gesture_l[i] = l_descZeroPos[i + 3];
        st_gesture_r[i] = r_descZeroPos[i + 3];
    }

    load_teleoperation_limits();
    load_preset_pose();

    //角度输出模式是否为静止的
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

    //-------------------------

    //控制值初始化
    //-----数据传输与插补模式-----
    mdataSend.ctl_transMod = 4;
    mdataSend.runinMod = 1;
    mdataSend.TorCompMod = 0;

    //-----求解模式信息-----
    // 连续笛卡尔遥操作使用上一条规划状态作为轨迹起点。
    // 以当前反馈(ifUseRet=1)重规划会把通信/反馈延迟直接变成追赶运动，
    // 表现为忽快忽慢；底层伺服仍会闭环跟踪这条平滑目标轨迹。
    mdataSend.ifUseRet = 0;
    mdataSend.ifUseTemp = 0;

    mdataSend.rigidityCtl_mod_l.resize(LARMDOF);
    mdataSend.rigidityCtl_mod_r.resize(RARMDOF);
    for(int i = 0; i < LARMDOF; i++){
        mdataSend.rigidityCtl_mod_l[i] = this->rigidity_mod;
        mdataSend.rigidityCtl_mod_r[i] = this->rigidity_mod;
    }

    mdataSend.KpTrans_l.resize(LARMDOF);
    mdataSend.KdTrans_l.resize(LARMDOF);
    mdataSend.KpTrans_r.resize(RARMDOF);
    mdataSend.KdTrans_r.resize(RARMDOF);

    //-----插补信息包装-----
    //发送变量初始化
    // The target is refreshed at 50 Hz.  A one-second interpolation window
    // makes each re-plan chase a stale target and causes visible lag during
    // left/right reversals.  Keep a short smooth window for direct following.
    double trajTime = 0.20;
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

    mdataSend.ctlDesMsg = ctlStMsg;
}

void xr_telepercation_ctl::trans_and_outp_core(){
    for(std::size_t i = 0; i < kCartesianAxes; ++i){
        if(!std::isfinite(l_arm_end_delta[i])){ l_arm_end_delta[i] = 0.0; }
        if(!std::isfinite(r_arm_end_delta[i])){ r_arm_end_delta[i] = 0.0; }
    }
    const auto valid_quaternion = [](const std::vector<double>& quaternion){
        return quaternion.size() == 4 &&
            std::all_of(quaternion.begin(), quaternion.end(), [](double value){ return std::isfinite(value); });
    };
    if(!valid_quaternion(l_arm_quat_accum)){ l_arm_quat_accum = {1.0, 0.0, 0.0, 0.0}; }
    if(!valid_quaternion(r_arm_quat_accum)){ r_arm_quat_accum = {1.0, 0.0, 0.0, 0.0}; }

    // st_gesture_* 为 RPY [roll, pitch, yaw]（来自 desc_zeropos.yaml），
    // 而 eulerToQuaternionZYX 期望 ZYX [yaw, pitch, roll]，故交换首尾两项
    std::vector<double> base_quat_l = eulerToQuaternionZYX({st_gesture_l[2], st_gesture_l[1], st_gesture_l[0]});
    std::vector<double> base_quat_r = eulerToQuaternionZYX({st_gesture_r[2], st_gesture_r[1], st_gesture_r[0]});

    // l/r_arm_quat_accum stores deltas in the fixed robot frame.  Prepending
    // those deltas to the zero pose keeps a physical left/right turn on the
    // same robot axis; base * accum would rotate the axes by the zero pose.
    std::vector<double> lArm_quat_world = multiplyQuaternions(l_arm_quat_accum, base_quat_l);
    std::vector<double> rArm_quat_world = multiplyQuaternions(r_arm_quat_accum, base_quat_r);

    // 将四元数转换回 ZYX 顺序的欧拉角（返回 [yaw, pitch, roll]）
    std::vector<double> lArm_zyx_world = quaternionToEulerZYX(lArm_quat_world);
    std::vector<double> rArm_zyx_world = quaternionToEulerZYX(rArm_quat_world);
    if(last_arm_zyx_valid){
        unwrapEulerNear(lArm_zyx_world, last_l_arm_zyx);
        unwrapEulerNear(rArm_zyx_world, last_r_arm_zyx);
    }
    last_l_arm_zyx = lArm_zyx_world;
    last_r_arm_zyx = rArm_zyx_world;
    last_arm_zyx_valid = true;

    // 数据点包装
    // 下游 desPos_*_Descartes 的姿态部分为 RPY [roll, pitch, yaw]，
    // 而 quaternionToEulerZYX 返回 ZYX [yaw, pitch, roll]，故交换首尾两项
    for(int i = 0; i < 3; i++){
        lArm_end_pos_rpy[i] = l_arm_end_delta[i] + st_pos_l[i];
        rArm_end_pos_rpy[i] = r_arm_end_delta[i] + st_pos_r[i];
    }
    lArm_end_pos_rpy[3] = lArm_zyx_world[2];  // roll
    lArm_end_pos_rpy[4] = lArm_zyx_world[1];  // pitch
    lArm_end_pos_rpy[5] = lArm_zyx_world[0];  // yaw
    rArm_end_pos_rpy[3] = rArm_zyx_world[2];  // roll
    rArm_end_pos_rpy[4] = rArm_zyx_world[1];  // pitch
    rArm_end_pos_rpy[5] = rArm_zyx_world[0];  // yaw

    limit_and_reanchor_target(
        lArm_end_pos_rpy, l_arm_end_delta, l_arm_quat_accum,
        st_pos_l, st_gesture_l, l_pos_min, l_pos_max, l_rpy_min, l_rpy_max, position_limit_enabled,
        orientation_limit_enabled,
        l_position_limit_active, l_orientation_limit_active, "左");
    limit_and_reanchor_target(
        rArm_end_pos_rpy, r_arm_end_delta, r_arm_quat_accum,
        st_pos_r, st_gesture_r, r_pos_min, r_pos_max, r_rpy_min, r_rpy_max, position_limit_enabled,
        orientation_limit_enabled,
        r_position_limit_active, r_orientation_limit_active, "右");
    last_l_arm_zyx = {lArm_end_pos_rpy[5], lArm_end_pos_rpy[4], lArm_end_pos_rpy[3]};
    last_r_arm_zyx = {rArm_end_pos_rpy[5], rArm_end_pos_rpy[4], rArm_end_pos_rpy[3]};
    
    ctlDesMsg = ctlStMsg;
    ctlDesMsg.desPos_l_Descartes = lArm_end_pos_rpy;
    ctlDesMsg.desPos_r_Descartes = rArm_end_pos_rpy;

    mdataSend.TorCompMod = 0;

    mdataSend.ctlDesMsg = ctlDesMsg;
}

void xr_telepercation_ctl::trigger_joint_return(){
    use_preset_target = false;
    returnRequested = true;
}

bool xr_telepercation_ctl::reanchor_from_feedback(std::string* rejection_reason){
    if(rejection_reason) rejection_reason->clear();
    const auto reject = [rejection_reason](const std::string& reason){
        if(rejection_reason) *rejection_reason = reason;
        return false;
    };
    if(!desc_ret_valid) return reject("fk_feedback_missing: no valid six-element pose received");
    static constexpr const char* axes[] = {"x", "y", "z", "roll", "pitch", "yaw"};
    const auto detail = [](const char* reason, const char* side, int axis, double value){
        std::ostringstream out;
        out << std::setprecision(10) << reason << " arm=" << side
            << " axis=" << axes[axis] << " value=" << value
            << " unit=" << (axis < 3 ? "m" : "rad");
        return out.str();
    };
    for(int i = 0; i < 6; ++i){
        if(!std::isfinite(l_desc_ret_cache[i]))
            return reject(detail("fk_feedback_nonfinite", "left", i, l_desc_ret_cache[i]));
        if(!std::isfinite(r_desc_ret_cache[i]))
            return reject(detail("fk_feedback_nonfinite", "right", i, r_desc_ret_cache[i]));
    }
    // Refuse a handoff that would clamp the measured pose to a distant VR limit.
    const auto check_bound = [&](const char* side, int axis, double value, double lower, double upper){
        if(value >= lower && value <= upper) return true;
        std::ostringstream out;
        out << detail(axis < 3 ? "vr_position_limit" : "vr_orientation_limit", side, axis, value)
            << std::setprecision(10) << " bounds=[" << lower << "," << upper << "]";
        return reject(out.str());
    };
    for(int i = 0; i < 3; ++i){
        if(position_limit_enabled &&
            (!check_bound("left", i, l_desc_ret_cache[i], l_pos_min[i], l_pos_max[i]) ||
             !check_bound("right", i, r_desc_ret_cache[i], r_pos_min[i], r_pos_max[i]))) return false;
        if(orientation_limit_enabled &&
            (!check_bound("left", i+3, l_desc_ret_cache[i+3], l_rpy_min[i], l_rpy_max[i]) ||
             !check_bound("right", i+3, r_desc_ret_cache[i+3], r_rpy_min[i], r_rpy_max[i]))) return false;
    }
    for(int i = 0; i < 3; ++i){
        st_pos_l[i] = l_desc_ret_cache[i];
        st_pos_r[i] = r_desc_ret_cache[i];
        st_gesture_l[i] = l_desc_ret_cache[i+3];
        st_gesture_r[i] = r_desc_ret_cache[i+3];
    }
    l_arm_end_delta.assign(3, 0.0);
    r_arm_end_delta.assign(3, 0.0);
    l_arm_end_pos.assign(3, 0.0);
    r_arm_end_pos.assign(3, 0.0);
    l_arm_quat_accum = {1.0, 0.0, 0.0, 0.0};
    r_arm_quat_accum = {1.0, 0.0, 0.0, 0.0};
    last_arm_zyx_valid = false;
    returnRequested = returning = preset_holding = false;
    ifFirstOutp = false;
    zero_outputTimes = total_zeroTimes;
    mdataSend.ctl_transMod = 4;
    mdataSend.runinMod = 1;
    mdataSend.ifUseTemp = 0;
    mdataSend.TrajTime[0].assign(LARMDOF, 0.20);
    mdataSend.TrajTime[1].assign(RARMDOF, 0.20);
    handoff_pending = true;
    return true;
}

void xr_telepercation_ctl::trigger_preset_pose(){
    if(!preset_pose_valid){
        std::cerr << "[遥操作] 右 B 请求被忽略：预设姿态配置无效" << std::endl;
        return;
    }
    use_preset_target = true;
    returnRequested = true;
}

void xr_telepercation_ctl::apply_left_orientation_delta(const std::vector<double>& delta_zyx){
    if(!accepts_vr_input()){ return; }
    if(delta_zyx.size() != 3 || l_arm_quat_accum.size() != 4){
        return;
    }
    if(!std::all_of(delta_zyx.begin(), delta_zyx.end(), [](double value){ return std::isfinite(value); })){
        return;
    }
    // Deltas are expressed in the fixed head/robot frame, so pre-multiply.
    // Post-multiplication would interpret them in the changing tool-local
    // frame and swap the perceived left/right rotation axes.
    l_arm_quat_accum = multiplyQuaternions(
        tool_swing_increment(delta_zyx, l_arm_quat_accum, st_gesture_l, lock_tool_twist),
        l_arm_quat_accum
    );
}

void xr_telepercation_ctl::apply_right_orientation_delta(const std::vector<double>& delta_zyx){
    if(!accepts_vr_input()){ return; }
    if(delta_zyx.size() != 3 || r_arm_quat_accum.size() != 4){
        return;
    }
    if(!std::all_of(delta_zyx.begin(), delta_zyx.end(), [](double value){ return std::isfinite(value); })){
        return;
    }
    r_arm_quat_accum = multiplyQuaternions(
        tool_swing_increment(delta_zyx, r_arm_quat_accum, st_gesture_r, lock_tool_twist),
        r_arm_quat_accum
    );
}

void xr_telepercation_ctl::update_desc_ret(const std::vector<double>& l_desc_pose, const std::vector<double>& r_desc_pose){
    if(l_desc_pose.size() == 6 && r_desc_pose.size() == 6){
        l_desc_ret_cache = l_desc_pose;
        r_desc_ret_cache = r_desc_pose;
        desc_ret_valid = true;
    }
}

void xr_telepercation_ctl::build_joint_return_cmd(){
    //关节空间轨迹回零：mode 1（关节模式）+ 点位插补（完整加减速）
    mdataSend.ctl_transMod = 1;
    mdataSend.runinMod = 3;
    mdataSend.TorCompMod = 0;

    //起点使用当前关节反馈（ifUseRet=1），A/B 共用平滑轨迹。
    mdataSend.ifUseRet = 1;
    mdataSend.ifUseTemp = 0;

    // 回零时间和曲线形状均由 YAML 配置。较长时间和接近 1 的比例会
    // 延长加、减速段，避免从遥操作直接切换到回零时的速度突变。
    mdataSend.TrajTime[0].assign(LARMDOF, joint_return_trajectory_time_sec);
    mdataSend.TrajTime[1].assign(RARMDOF, joint_return_trajectory_time_sec);
    mdataSend.accRateBase[0].assign(LARMDOF, joint_return_acceleration_time_ratio);
    mdataSend.accRateBase[1].assign(RARMDOF, joint_return_acceleration_time_ratio);

    // 先保持当前关节位置，再在目标位置驻留，给中层轨迹规划和伺服稳定时间。
    for(int i = 0; i < LARMDOF; i++){
        mdataSend.m_stabMsg[0][i].ifUseStab = true;
        mdataSend.m_stabMsg[0][i].start_stabTimes = joint_return_start_stable_cycles;
        mdataSend.m_stabMsg[0][i].des_stabTimes = joint_return_end_stable_cycles;
    }
    for(int i = 0; i < RARMDOF; i++){
        mdataSend.m_stabMsg[1][i].ifUseStab = true;
        mdataSend.m_stabMsg[1][i].start_stabTimes = joint_return_start_stable_cycles;
        mdataSend.m_stabMsg[1][i].des_stabTimes = joint_return_end_stable_cycles;
    }

    mdataSend.ctlDesMsg.desPos_l_JointSpace = use_preset_target ? preset_left_joints : lArm_zeroPos;
    mdataSend.ctlDesMsg.desPos_r_JointSpace = use_preset_target ? preset_right_joints : rArm_zeroPos;
}


dataSendWrap xr_telepercation_ctl::trans_and_outp(){
    if(handoff_pending){
        handoff_pending = false;
        mdataSend.ifUseRet = 1;
        use_feedback_once_after_return = true;
        trans_and_outp_core();
        return mdataSend;
    }
    if(ifFirstOutp){
        std::cout << "准备开始遥操作" << std::endl;
        ifFirstOutp = false;
        for(int i = 0; i < l_arm_end_pos.size(); i++){
            l_arm_end_pos[i] = 0.0;
        }
        for(int i = 0; i < l_arm_end_delta.size(); i++){
            l_arm_end_delta[i] = 0.0;
        }

        for(int i = 0; i < r_arm_end_pos.size(); i++){
            r_arm_end_pos[i] = 0.0;
        }
        l_arm_quat_accum = {1.0, 0.0, 0.0, 0.0};
        r_arm_quat_accum = {1.0, 0.0, 0.0, 0.0};
        last_arm_zyx_valid = false;
        for(int i = 0; i < r_arm_end_delta.size(); i++){
            r_arm_end_delta[i] = 0.0;
        }
        ifUseStFtr = true;
        stabtimecount = 0;
    }
    
    //---------- A 键关节空间回零状态机 ----------
    //触发：构建关节空间回零指令，清零增量
    if(returnRequested){
        returnRequested = false;
        preset_holding = false;
        use_feedback_once_after_return = false;
        // An explicit A/B target supersedes the startup zero-pose hold.
        zero_outputTimes = total_zeroTimes;
        std::cout << "[遥操作] 以 " << joint_return_trajectory_time_sec
                  << "s 平滑关节轨迹到达 "
                  << (use_preset_target ? "右 B 预设 vr_preset_pose.yaml" : "右 A 零位 zeropos.yaml")
                  << std::endl;
        build_joint_return_cmd();
        returning = true;
        const double cycle_time = (std::isfinite(TempTime) && TempTime > 0.0) ? TempTime : 0.02;
        joint_return_remaining_cycles = static_cast<int>(std::ceil(
            joint_return_trajectory_time_sec / cycle_time)) +
            joint_return_start_stable_cycles + joint_return_end_stable_cycles + 2;
        std::fill(l_arm_end_delta.begin(), l_arm_end_delta.end(), 0.0);
        std::fill(r_arm_end_delta.begin(), r_arm_end_delta.end(), 0.0);
        std::fill(l_arm_end_pos.begin(), l_arm_end_pos.end(), 0.0);
        std::fill(r_arm_end_pos.begin(), r_arm_end_pos.end(), 0.0);
        l_arm_quat_accum = {1.0, 0.0, 0.0, 0.0};
        r_arm_quat_accum = {1.0, 0.0, 0.0, 0.0};
        last_arm_zyx_valid = false;
        return mdataSend;
    }

    if(preset_holding){
        std::fill(l_arm_end_delta.begin(), l_arm_end_delta.end(), 0.0);
        std::fill(r_arm_end_delta.begin(), r_arm_end_delta.end(), 0.0);
        return mdataSend;
    }

    // 在整段点到点轨迹完成前，持续输出同一关节回零指令。此前下一周期就
    // 切回笛卡尔模式，导致规划轨迹被中断，表现为回零突跳。
    if(returning){
        if(joint_return_remaining_cycles-- > 0){
            return mdataSend;
        }
        if(use_preset_target){
            returning = false;
            preset_holding = true;
            std::fill(l_arm_end_delta.begin(), l_arm_end_delta.end(), 0.0);
            std::fill(r_arm_end_delta.begin(), r_arm_end_delta.end(), 0.0);
            std::cout << "[遥操作] 预设轨迹时段结束：保持预设关节目标，暂停 VR 位姿控制；右 A 回零后恢复" << std::endl;
            return mdataSend;
        }
        // 完成：用末端反馈的当前笛卡尔位姿重新锚定遥操作基准，再恢复笛卡尔遥操作。
        std::cout << "[遥操作] 关节姿态运动完成，恢复笛卡尔遥操作" << std::endl;
        returning = false;

        if(desc_ret_valid){
            for(int i = 0; i < 3; i++){
                st_pos_l[i] = l_desc_ret_cache[i];
                st_pos_r[i] = r_desc_ret_cache[i];
                st_gesture_l[i] = l_desc_ret_cache[i + 3];
                st_gesture_r[i] = r_desc_ret_cache[i + 3];
            }
        }

        //恢复笛卡尔遥操作模式
        mdataSend.ctl_transMod = 4;
        mdataSend.runinMod = 1;
        mdataSend.TorCompMod = 0;
        // 恢复遥操作后继续使用上一条规划状态作为连续轨迹起点。
        mdataSend.ifUseRet = 0;
        mdataSend.ifUseTemp = 0;
        mdataSend.TrajTime[0].assign(LARMDOF, 0.20);
        mdataSend.TrajTime[1].assign(RARMDOF, 0.20);
        mdataSend.accRateBase[0].assign(LARMDOF, 1.2);
        mdataSend.accRateBase[1].assign(RARMDOF, 1.2);
        for(int i = 0; i < LARMDOF; i++){
            mdataSend.m_stabMsg[0][i].ifUseStab = true;
            mdataSend.m_stabMsg[0][i].start_stabTimes = 50;
            mdataSend.m_stabMsg[0][i].des_stabTimes = 50;
        }
        for(int i = 0; i < RARMDOF; i++){
            mdataSend.m_stabMsg[1][i].ifUseStab = true;
            mdataSend.m_stabMsg[1][i].start_stabTimes = 50;
            mdataSend.m_stabMsg[1][i].des_stabTimes = 50;
        }
        // The first Cartesian cycle must synchronize to the just-finished
        // joint-return feedback; subsequent cycles use planner state.
        mdataSend.ifUseRet = 1;
        use_feedback_once_after_return = true;

        //清零增量，重新开始
        std::fill(l_arm_end_delta.begin(), l_arm_end_delta.end(), 0.0);
        std::fill(r_arm_end_delta.begin(), r_arm_end_delta.end(), 0.0);
        std::fill(l_arm_end_pos.begin(), l_arm_end_pos.end(), 0.0);
        std::fill(r_arm_end_pos.begin(), r_arm_end_pos.end(), 0.0);
        l_arm_quat_accum = {1.0, 0.0, 0.0, 0.0};
        r_arm_quat_accum = {1.0, 0.0, 0.0, 0.0};
        last_arm_zyx_valid = false;

        trans_and_outp_core();
        return mdataSend;
    }

    if(use_feedback_once_after_return){
        // One-cycle synchronization is enough.  Keeping ifUseRet=1 would
        // re-plan from delayed feedback every cycle and reintroduce speed
        // modulation during normal teleoperation.
        mdataSend.ifUseRet = 0;
        use_feedback_once_after_return = false;
    }
    trans_and_outp_core();

    if(zero_outputTimes < total_zeroTimes){
        mdataSend.ctlDesMsg.desPos_l_Descartes = l_descZeroPos;
        mdataSend.ctlDesMsg.desPos_r_Descartes = r_descZeroPos;
        mdataSend.ctlDesMsg.desPos_l_JointSpace = lArm_zeroPos;
        mdataSend.ctlDesMsg.desPos_r_JointSpace = rArm_zeroPos;

        zero_outputTimes += 1;
    }

    return mdataSend;
}
