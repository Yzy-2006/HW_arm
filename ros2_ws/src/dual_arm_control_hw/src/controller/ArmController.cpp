#include "controller/ArmController.h"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <fstream>
#include <sstream>

// 读取文件并解析 package:// 路径
static std::string loadAndResolveURDF(const std::string& filepath) {
    std::ifstream f(filepath);
    if (!f.is_open()) {
        std::cerr << "Cannot open URDF: " << filepath << std::endl;
        return "";
    }
    std::stringstream buf;
    buf << f.rdbuf();
    std::string content = buf.str();

    // 替换 package://xxx/ → file://实际路径/
    const std::string prefix = "package://";
    size_t pos = 0;
    while ((pos = content.find(prefix, pos)) != std::string::npos) {
        size_t slash = content.find('/', pos + prefix.length());
        std::string pkg = content.substr(pos + prefix.length(), slash - pos - prefix.length());
        try {
            std::string share = ament_index_cpp::get_package_share_directory(pkg);
            content.replace(pos, slash - pos, "file://" + share);
        } catch (...) {
            pos++;  // 找不到包，跳过
        }
    }
    return content;
}


//======================================================================
/*一级控制算法*/

std::vector<double> dbg_reserve_extern(14, 0.0);
std::vector<double> dbg_reserve_extern_2(14, 0.0);
std::vector<double> dbg_reserve_extern_3(14, 0.0);

//--------------------插补类：--------------------

//一次插补
TrajResult runiner::dPos_queue_stabTime_single(double mposDes, double mTrajTime)
{
    //圆整时间
    int tempTimes = std::round(mTrajTime/this->control_T);
    if(tempTimes < 1){
        tempTimes = 1;
    }
    double trajTime_round = this->control_T * tempTimes;

    //计算位置增量与速度
    double derPos = mposDes / tempTimes;
    double vel = mposDes / trajTime_round;

    //装入结构体
    TrajResult mRes;
    double tempPos = 0;
    for(int i = 0; i < tempTimes; i++){
    //for(int i = 0; i < 1; i++){

        tempPos = tempPos + derPos;
        mRes.positionIncrements.push_back(tempPos);
        mRes.VelocityTraj.push_back(vel);
    }

    //记录最后一轮的速度
    this->lastTempPosOutp = mRes.positionIncrements.back();
    this->lastTempVelOutp = mRes.VelocityTraj.back();
    return mRes;
}

//二次插补
TrajResult runiner::dPos_queue_stabTime_double(double mposDes, double velSt, double mtrajTime)
{
    //注意：输入量均为弧度制
    double posDes_half;
    double trajTime;
    //计算半切轨迹
    posDes_half = mposDes;
    trajTime = mtrajTime;

    //计算半切轨迹
    //核心——保证始末加速度为0（或近似0），末位置恰好达到目标位置
    double a = 12 * (velSt*trajTime - posDes_half) / std::pow(trajTime, 4);
    double b = a * trajTime;

    //据此生成增量轨迹
    std::vector<double> mResTraj;
    std::vector<double> mVelTraj;
    int m_tempLen = std::floor(trajTime / control_T);
    m_tempLen = std::max(1, m_tempLen);

    double temp_T = this->control_T;
    for(int i = 0; i < m_tempLen; i++){
        double temp_pos = std::pow(temp_T, 4)*a*1.0/12 - std::pow(temp_T, 3)*b*1.0/6 + velSt*temp_T;
        double temp_vel = a * std::pow(temp_T, 3) - b * std::pow(temp_T, 2) / 2 + velSt;
        mResTraj.push_back(temp_pos);
        //mVelTraj.push_back(temp_vel);
        mVelTraj.push_back(0.0);
        temp_T += control_T;
    }

    TrajResult mRes;
    mRes.positionIncrements = mResTraj;
    mRes.VelocityTraj = mVelTraj;

    this->lastTempPosOutp = mRes.positionIncrements.back();
    this->lastTempVelOutp = mRes.VelocityTraj.back();

    //return mRes_Interpolated;
    return mRes;
}

//点位插补
TrajResult runiner::dPos_queue_stabTime_dot(double mposDes, double mTrajTime, double accTimeRateBase)
{
    //轨迹一半的时间用来加速
    //由曲线推到可得，当加速过程时间占到时间的一半时，按照钟型曲线加速过程和匀速过程走过距离比例为1：2

    double t_half = mTrajTime / 2;
    double posDes_half = mposDes / 2;

    // ----- 计算轨迹及其增量结果储存 -----
    //加速过程
    std::vector<double> acc_traj_pos;
    std::vector<double> acc_traj_derPos;
    std::vector<double> acc_traj_vel;

    //减速过程
    std::vector<double> dAcc_traj_pos;
    std::vector<double> dAcc_traj_derPos;
    std::vector<double> dAcc_traj_vel;

    //匀速过程
    std::vector<double> stab_traj_pos;
    std::vector<double> stab_traj_derPos;
    std::vector<double> stab_traj_vel;

    //轨迹参数计算a, b, 和v_max
    if (accTimeRateBase <= 1.05) {
        std::cerr << "accTimeRateBase must be greater than 1.0 to avoid division by zero" << std::endl;
        accTimeRateBase = 1.05; // 设置默认安全值
    }
    double t_acc = t_half / accTimeRateBase;
    double t_stab = mTrajTime - 2*t_acc;
    double denominator = std::pow(t_half, 4) * (2*accTimeRateBase - 1);
    double a = -12 * std::pow(accTimeRateBase, 4) * posDes_half / denominator;
    double b = a * t_half / accTimeRateBase;

    //计算加速段轨迹
    int m_accTempLen = std::round(t_acc/this->control_T);
    double temp_T = this->control_T;
    double temp_pos_last = 0;
    for(int i = 0; i < m_accTempLen; i++){
        double temp_pos;
        temp_pos = (1.0/12.0)*a*std::pow(temp_T, 4) - (1.0/6.0)*b*std::pow(temp_T, 3);
        acc_traj_pos.push_back(temp_pos);
        acc_traj_derPos.push_back(temp_pos - temp_pos_last);
        acc_traj_vel.push_back((temp_pos - temp_pos_last)/control_T);

        temp_pos_last = temp_pos;
        temp_T = temp_T + this->control_T;
    }

    //计算匀速段轨迹
    int stab_tempTime = std::round((mTrajTime - 2*(m_accTempLen*this->control_T)) / this->control_T);
    double stabTime = this->control_T * stab_tempTime;
    double stab_trajLen = mposDes - 2*(acc_traj_pos.back());
    double stab_vel = stab_trajLen / stabTime;
    double stab_derTraj = stab_trajLen / stab_tempTime;
    for(int i = 0; i < stab_tempTime; i++){
        double temp_pos = temp_pos_last + stab_derTraj;

        stab_traj_pos.push_back(temp_pos);
        stab_traj_derPos.push_back(stab_derTraj);
        stab_traj_vel.push_back(stab_vel);

        temp_pos_last = temp_pos;
    }

    //计算减速段位移 -- 加速段位移的镜像
    for(int i = m_accTempLen - 1; i >= 0; i--){
        double temp_pos = temp_pos_last + acc_traj_derPos[i];
        double temp_vel = acc_traj_vel[i];
        dAcc_traj_pos.push_back(temp_pos);
        dAcc_traj_derPos.push_back(acc_traj_derPos[i]);
        dAcc_traj_vel.push_back(temp_vel);

        temp_pos_last = temp_pos;
    }

    //融合三段轨迹得到结果
    TrajResult mRes;
    mRes.positionIncrements = acc_traj_pos;
    mRes.VelocityTraj = acc_traj_vel;

    mRes.positionIncrements.insert(mRes.positionIncrements.end(), stab_traj_pos.begin(), stab_traj_pos.end());
    mRes.VelocityTraj.insert(mRes.VelocityTraj.end(), stab_traj_vel.begin(), stab_traj_vel.end());

    mRes.positionIncrements.insert(mRes.positionIncrements.end(), dAcc_traj_pos.begin(), dAcc_traj_pos.end());
    mRes.VelocityTraj.insert(mRes.VelocityTraj.end(), dAcc_traj_vel.begin(), dAcc_traj_vel.end());

    this->lastTempPosOutp = mRes.positionIncrements.back();
    this->lastTempVelOutp = mRes.VelocityTraj.back();

    return mRes;
}

std::pair<double, double> runiner::lastPosVelGet(double posSt)
{
    double posOutpCor = this->lastTempPosOutp + posSt;
    std::pair<double, double> mRes = {posOutpCor, this->lastTempVelOutp};
    return mRes;
}

//--------------------力补偿类：--------------------



std::vector<double> Tor2Iq(double Kt[], std::vector<double> Tor, int Len)
{
    //输入量：Kt -- 电流常数； Tor -- 力矩； Len -- 数组长度，注意长度的匹配，不匹配就报错
    //力矩转化成电流
    std::vector<double> mres;
    for(int i = 0; i < Len; i++){
        mres.push_back(Tor.at(i)/Kt[i]);
    }
    return mres;
}

std::vector<double> Iq2Tor(double Kt[], std::vector<double> Tor, int Len)
{
    //输入量：Kt -- 电流常数； Tor -- 力矩； Len -- 数组长度，注意长度的匹配，不匹配就报错
    //力矩转化成电流
    std::vector<double> mres;
    for(int i = 0; i < Len; i++){
        mres.push_back(Tor.at(i) * Kt[i]);
    }
    return mres;
}

void printAllJointsInfo(const pinocchio::Model& model) {
    std::cout << "\n========== 模型关节信息 ==========" << std::endl;
    std::cout << "模型总关节数 (model.njoints): " << model.njoints << std::endl;
    std::cout << "配置空间维度 (model.nq): " << model.nq << std::endl;
    std::cout << "速度空间维度 (model.nv): " << model.nv << std::endl;
    
    std::cout << "\n========== 配置空间映射 ==========" << std::endl;
    
    // 创建一个向量来跟踪配置索引
    int current_nq_index = 0;  // 当前配置向量的起始索引
    int current_nv_index = 0;  // 当前速度向量的起始索引
    
    std::cout << "\n关节详细信息：" << std::endl;
    std::cout << "-------------------------------------------------------" << std::endl;
    
    for (pinocchio::JointIndex i = 0; i < model.njoints; ++i) {
        const std::string& joint_name = model.names[i];
        const pinocchio::JointModel& joint = model.joints[i];
        int joint_nq = joint.nq();
        int joint_nv = joint.nv();
        
        std::cout << "关节索引: " << i << std::endl;
        std::cout << "  名称: " << joint_name << std::endl;
        std::cout << "  关节类型: " << joint.shortname() << std::endl;
        std::cout << "  父关节索引: " << model.parents[i] << std::endl;
        
        // 打印配置空间映射信息
        if (joint_nq > 0) {
            std::cout << "  配置维度 (nq): " << joint_nq << std::endl;
            std::cout << "  在q向量中的索引: [" << current_nq_index 
                     << " 到 " << current_nq_index + joint_nq - 1 << "]" << std::endl;
            
            // 解释每个配置变量的含义
            std::cout << "  配置变量含义: ";
            if (joint.shortname().find("Revolute") != std::string::npos) {
                std::cout << "旋转角度 (弧度)" << std::endl;
            } else if (joint.shortname().find("Prismatic") != std::string::npos) {
                std::cout << "平移距离 (米)" << std::endl;
            } else if (joint.shortname().find("FreeFlyer") != std::string::npos) {
                std::cout << "位置(3) + 四元数姿态(4)" << std::endl;
            } else if (joint.shortname().find("Spherical") != std::string::npos) {
                std::cout << "四元数姿态 (x,y,z,w)" << std::endl;
            } else if (joint.shortname().find("Planar") != std::string::npos) {
                std::cout << "平面位置(2) + 旋转角度(1)" << std::endl;
            } else {
                std::cout << "未知类型" << std::endl;
            }
        } else {
            std::cout << "  配置维度 (nq): 0 (固定关节)" << std::endl;
        }
        
        if (joint_nv > 0) {
            std::cout << "  速度维度 (nv): " << joint_nv << std::endl;
            std::cout << "  在v向量中的索引: [" << current_nv_index 
                     << " 到 " << current_nv_index + joint_nv - 1 << "]" << std::endl;
        } else {
            std::cout << "  速度维度 (nv): 0 (固定关节)" << std::endl;
        }
        
        // 输出关节限制信息
        if (i < model.upperPositionLimit.size() && i < model.lowerPositionLimit.size() && joint_nq > 0) {
            bool has_limits = false;
            for (int j = 0; j < joint_nq; ++j) {
                int idx = current_nq_index + j;
                if (idx < model.lowerPositionLimit.size() && 
                    idx < model.upperPositionLimit.size()) {
                    double lower = model.lowerPositionLimit[idx];
                    double upper = model.upperPositionLimit[idx];
                    if (upper > lower) {  // 有效的限制
                        has_limits = true;
                        std::cout << "  配置变量 " << j << " 限制: [" 
                                 << lower << ", " << upper << "]" << std::endl;
                    }
                }
            }
            if (!has_limits) {
                std::cout << "  位置限制: 无限制" << std::endl;
            }
        }
        
        // 输出关节ID映射（如果存在）
        if (model.joints[i].id() >= 0) {
            std::cout << "  关节ID: " << model.joints[i].id() << std::endl;
        }
        
        // 输出坐标系信息
        if (i < model.frames.size()) {
            for (size_t frame_idx = 0; frame_idx < model.frames.size(); ++frame_idx) {
                if (model.frames[frame_idx].parentJoint == i) {
                    std::cout << "  附加框架: " << model.frames[frame_idx].name 
                             << " (类型: " << model.frames[frame_idx].type << ")" << std::endl;
                }
            }
        }
        
        std::cout << "-------------------------------------------------------" << std::endl;
        
        // 更新索引
        current_nq_index += joint_nq;
        current_nv_index += joint_nv;
    }
    
    // 验证总数
    std::cout << "\n========== 配置空间摘要 ==========" << std::endl;
    std::cout << "总配置变量数 (应等于nq): " << current_nq_index 
              << " (model.nq = " << model.nq << ")" << std::endl;
    std::cout << "总速度变量数 (应等于nv): " << current_nv_index 
              << " (model.nv = " << model.nv << ")" << std::endl;
    
    if (current_nq_index != model.nq) {
        std::cerr << "警告：计算的总配置变量数与model.nq不匹配！" << std::endl;
    }
    if (current_nv_index != model.nv) {
        std::cerr << "警告：计算的总速度变量数与model.nv不匹配！" << std::endl;
    }
    
    // 添加完整的配置向量结构说明
    std::cout << "\n========== 配置向量q的结构 ==========" << std::endl;
    current_nq_index = 0;
    current_nv_index = 0;
    
    std::cout << "q向量 (size = " << model.nq << "):" << std::endl;
    std::cout << "索引 | 关节名称 | 类型 | 变量含义" << std::endl;
    std::cout << "-----|----------|------|----------" << std::endl;
    
    for (pinocchio::JointIndex i = 0; i < model.njoints; ++i) {
        const pinocchio::JointModel& joint = model.joints[i];
        int joint_nq = joint.nq();
        
        if (joint_nq > 0) {
            for (int j = 0; j < joint_nq; ++j) {
                std::cout << std::setw(3) << current_nq_index << "  | "
                         << std::setw(10) << model.names[i] << " | "
                         << std::setw(6) << joint.shortname();
                
                if (joint.shortname().find("Revolute") != std::string::npos) {
                    std::cout << " | 角度 θ" << (j+1) << " (rad)" << std::endl;
                } else if (joint.shortname().find("Prismatic") != std::string::npos) {
                    std::cout << " | 距离 d" << (j+1) << " (m)" << std::endl;
                } else if (joint.shortname().find("FreeFlyer") != std::string::npos) {
                    if (j < 3) std::cout << " | 位置 p" << (j+1) << " (m)" << std::endl;
                    else std::cout << " | 姿态四元数 q" << (j-2) << std::endl;
                } else {
                    std::cout << " | 变量 " << (j+1) << std::endl;
                }
                
                current_nq_index++;
            }
        }
    }
    
    std::cout << "\nv向量 (size = " << model.nv << "):" << std::endl;
    std::cout << "索引 | 关节名称 | 类型 | 变量含义" << std::endl;
    std::cout << "-----|----------|------|----------" << std::endl;
    
    current_nv_index = 0;
    for (pinocchio::JointIndex i = 0; i < model.njoints; ++i) {
        const pinocchio::JointModel& joint = model.joints[i];
        int joint_nv = joint.nv();
        
        if (joint_nv > 0) {
            for (int j = 0; j < joint_nv; ++j) {
                std::cout << std::setw(3) << current_nv_index << "  | "
                         << std::setw(10) << model.names[i] << " | "
                         << std::setw(6) << joint.shortname();
                
                if (joint.shortname().find("Revolute") != std::string::npos) {
                    std::cout << " | 角速度 ω" << (j+1) << " (rad/s)" << std::endl;
                } else if (joint.shortname().find("Prismatic") != std::string::npos) {
                    std::cout << " | 速度 v" << (j+1) << " (m/s)" << std::endl;
                } else if (joint.shortname().find("FreeFlyer") != std::string::npos) {
                    if (j < 3) std::cout << " | 线速度 v" << (j+1) << " (m/s)" << std::endl;
                    else std::cout << " | 角速度 ω" << (j-2) << " (rad/s)" << std::endl;
                } else {
                    std::cout << " | 速度 " << (j+1) << std::endl;
                }
                
                current_nv_index++;
            }
        }
    }
    
    std::cout << "=======================================================" << std::endl;
}



//======================================================================
/*单关节点位控制代码
-------------------------------------------
  单关节点位控制代码仅包含定时轨迹控制代码
*/


//静止关节代码
void jnt_stabOutp::init(double mJntDeg){
    this->JntDeg = mJntDeg;
}
ctlOutp jnt_stabOutp::StabOutp()
{
    //输出计算
    ctlOutp mOutp;
    mOutp.posDes = this->JntDeg;
    mOutp.velDes = 0.0;

    mOutp.grav_TorDes = 0.0;
    mOutp.interia_TorDes = 0.0;
    mOutp.feedforward_TorDes = 0.0;

    mOutp.ifReachEnd = true;

    return mOutp;  
}

//运动关节代码
void moveJntCtl_stabTime::init_stabTime(runinMsgDotMod mCtlMsg, runinStabMsg mstabMsg)
{
    clear();

    this->startPos = mCtlMsg.mPosSt;
    this->endPos = mCtlMsg.mPosDes;
    this->ifUseStab = mstabMsg.ifUseStab;

    this->ifUseStab = mstabMsg.ifUseStab;       //是否在初始位置停留
    this->stabTimes = mstabMsg.des_stabTimes;   //在目标位置停留时间
    double mstabRanges = mstabMsg.start_stabTimes;     //在初始位置停留时间

    double dertaPos = mCtlMsg.mPosDes - mCtlMsg.mPosSt;
    TrajResult res_traj = mlocater.dPos_queue_stabTime_dot(dertaPos, mCtlMsg.mTrajTime, mCtlMsg.accTimeRateBase);   //计算出的全切轨迹——从A点到达B点的轨迹
    std::vector<double> derPos_traj = res_traj.positionIncrements;
    derPos_traj.push_back(dertaPos);
    std::vector<double> derPos_traj_bf;      //依据全切轨迹计算往复轨迹
    derPos_traj_bf = derPos_traj;   //先正序复制
    for(int i = 0; i < this->stabTimes; i++){
        derPos_traj_bf.push_back(dertaPos);
    }
    std::vector<double> traj_temp(derPos_traj.rbegin(), derPos_traj.rend());
    derPos_traj_bf.insert(derPos_traj_bf.end(), traj_temp.begin(), traj_temp.end());
    derPos_traj_bf.push_back(0);

    //计算位置、速度与力矩
    //先装入一定量的静止值
    if(this->ifUseStab){
        posTraj_stabTime.assign(mstabRanges, startPos);
        velTraj_stabTime.assign(mstabRanges, 0.0);
        TorTraj_stabTime.assign(mstabRanges, 0.0);
        posTraj_stabTime_half.assign(mstabRanges, startPos);
        velTraj_stabTime_half.assign(mstabRanges, 0.0);
        TorTraj_stabTime_half.assign(mstabRanges, 0.0);
    }
    posTraj_stabTime.push_back(startPos);
    velTraj_stabTime.push_back(0.0);
    TorTraj_stabTime.push_back(0.0);
    posTraj_stabTime_half.push_back(startPos);
    velTraj_stabTime_half.push_back(0.0);
    TorTraj_stabTime_half.push_back(0.0);

    //再开始装入计算结果
    int all_tempLen = derPos_traj.size();
    int all_tempLen_bf = derPos_traj_bf.size();
    double pos_lst = startPos;
    for(int i = 0; i < all_tempLen_bf; i++){
        double temp_pos = derPos_traj_bf.at(i) + startPos;
        double temp_vel = (temp_pos - pos_lst)/this->temp_T;
        posTraj_stabTime.push_back(temp_pos);
        velTraj_stabTime.push_back(temp_vel);
        pos_lst = temp_pos;

        TorTraj_stabTime.push_back(0.0);
    }

    double pos_lst_2 = startPos;
    for(int i = 0; i < all_tempLen; i++){
        double temp_pos_2 = derPos_traj.at(i) + startPos;
        double temp_vel_2 = (temp_pos_2 - pos_lst_2)/this->temp_T;
        posTraj_stabTime_half.push_back(temp_pos_2);
        velTraj_stabTime_half.push_back(temp_vel_2);
        pos_lst_2 = temp_pos_2;

        TorTraj_stabTime_half.push_back(0.0);
    }
    // 在目标位置驻留 stabTimes 个周期，使稳定性延迟由轨迹自身承载
    for(int i = 0; i < this->stabTimes; i++){
        posTraj_stabTime_half.push_back(endPos);
        velTraj_stabTime_half.push_back(0.0);
        TorTraj_stabTime_half.push_back(0.0);
    }
    TrajLen = posTraj_stabTime_half.size();
}

ctlOutp moveJntCtl_stabTime::desPos_get_stabTime()
{
    ctlOutp mOutp;
    mOutp.posDes = this->posTraj_stabTime.at(this->TempTimes);
    mOutp.velDes = this->velTraj_stabTime.at(this->TempTimes);

    //所有目标力矩均用占位符填充
    mOutp.grav_TorDes = this->TorTraj_stabTime.at(this->TempTimes);
    mOutp.feedforward_TorDes = this->TorTraj_stabTime.at(this->TempTimes);
    mOutp.interia_TorDes = this->TorTraj_stabTime.at(this->TempTimes);

    mOutp.ifReachEnd = false;

    this->TempTimes += 1;
    int tempTimeMaxSize = this->posTraj_stabTime.size();
    if(tempTimeMaxSize < 1){
        tempTimeMaxSize = 1;
    }
    if(this->TempTimes >= tempTimeMaxSize - 1){
        this->TempTimes = 0;
    }
    return mOutp;
}
ctlOutp moveJntCtl_stabTime::desPos_get_stabTimeStop()
{
    ctlOutp mOutp;
    if(TempTimes < this->posTraj_stabTime_half.size()){
        mOutp.posDes = this->posTraj_stabTime_half.at(this->TempTimes);
        mOutp.velDes = this->velTraj_stabTime_half.at(this->TempTimes);

        //力输出占位填充
        mOutp.grav_TorDes = this->TorTraj_stabTime_half.at(this->TempTimes);
        mOutp.interia_TorDes = this->TorTraj_stabTime_half.at(this->TempTimes);
        mOutp.feedforward_TorDes = this->TorTraj_stabTime_half.at(this->TempTimes);

        mOutp.ifReachEnd = false;
        if(TempTimes >= this->posTraj_stabTime_half.size()-1){
            mOutp.ifReachEnd = true;
        }
        this->TempTimes += 1;
    }else{
        // 轨迹已完成：保持在终点位置，ifReachEnd 持续为 true，
        // TempTimes 不再递增，等待上层调用 init 重置
        mOutp.posDes = this->endPos;
        mOutp.velDes = 0.0;

        mOutp.grav_TorDes = 0.0;
        mOutp.interia_TorDes = 0.0;
        mOutp.feedforward_TorDes = 0.0;

        mOutp.ifReachEnd = true;
    }

    return mOutp;
}

void moveJntCtl_stabTime::init_stabTime_continue(runinMsg mCtlMsg)
{
    clear();

    //控制模式解包
    this->startPos = mCtlMsg.mPosSt;
    this->endPos = mCtlMsg.mPosDes;
    double derPos = this->endPos - this->startPos;
    this->posTraj_last = this->startPos;
    //计算轨迹
    TrajResult mRes;
    if(mCtlMsg.runinMod == 1){
        //插补模式1：线性插补
        mRes = mlocater.dPos_queue_stabTime_single(derPos, mCtlMsg.mTrajTime);
    }else{
        //插补模式2：二次插补
        mRes = mlocater.dPos_queue_stabTime_double(derPos, mCtlMsg.mVelSt, mCtlMsg.mTrajTime);
    }

    //装入计算结果
    for(int i = 0; i < mRes.positionIncrements.size(); i++){
        posTraj_stabTime.push_back(mRes.positionIncrements[i] + this->startPos);
        velTraj_stabTime.push_back(mRes.VelocityTraj[i]);
        posTraj_last = posTraj_stabTime[i];

        TorTraj_stabTime.push_back(0.0);
    }
    TrajLen_Continue = mRes.positionIncrements.size();
}

ctlOutp moveJntCtl_stabTime::desPos_get_stab_Continue(){
    ctlOutp mOutp;

    if(TempTimes < this->posTraj_stabTime.size()){
        mOutp.posDes = this->posTraj_stabTime.at(this->TempTimes);
        mOutp.velDes = this->velTraj_stabTime.at(this->TempTimes);

        //占位填充
        mOutp.grav_TorDes = this->TorTraj_stabTime.at(this->TempTimes);
        mOutp.interia_TorDes = this->TorTraj_stabTime.at(this->TempTimes);
        mOutp.feedforward_TorDes = this->TorTraj_stabTime.at(this->TempTimes);

        mOutp.ifReachEnd = false;
        if(TempTimes >= this->posTraj_stabTime.size()-1){
            mOutp.ifReachEnd = true;
        }
        this->TempTimes += 1;
    }else{
        // 轨迹已完成：保持在终点位置，ifReachEnd 持续为 true，
        // TempTimes 不再递增，等待上层调用 init 重置
        mOutp.posDes = this->endPos;
        mOutp.velDes = 0.0;

        mOutp.grav_TorDes = 0.0;
        mOutp.interia_TorDes = 0.0;
        mOutp.feedforward_TorDes = 0.0;

        mOutp.ifReachEnd = true;
    }

    return mOutp;
}

std::pair<double, double> moveJntCtl_stabTime::lastTempPosVel(){
    std::pair<double, double> mRes = mlocater.lastPosVelGet(this->startPos);
    return mRes;
}

//======================================================================
/*综合控制算法*/

double double2Float(double inp){
    double mres = static_cast<double>(inp);
    return mres;
}

/*针对机器人整体直接传输关节参数的求解代码封装*/

void JntTrackSol::init(KDL::JntArray posSt_lArm, KDL::JntArray posSt_rArm){
    posLst_lArm = posSt_lArm;
    posLst_rArm = posSt_rArm;

    ifUseTemp = false; //控制量：是否使用循环输出
    ifFirstTemp = true;    //控制量：是否为第一轮输出
    
    velLst_lArm.resize(posSt_lArm.rows());
    velLst_rArm.resize(posSt_rArm.rows());
    for(int i = 0; i < posSt_lArm.rows(); i++){
        velLst_lArm(i) = 0.0f;
    }
    for(int i = 0; i < posSt_rArm.rows(); i++){
        velLst_rArm(i) = 0.0f;
    }

    lArmJntTrajSol.resize(posSt_lArm.rows());
    rArmJntTrajSol.resize(posSt_rArm.rows());
    lArmStab.resize(posSt_lArm.rows());
    rArmStab.resize(posSt_rArm.rows());

    lArmJntTorTrajSol.resize(posSt_lArm.rows());
    rArmJntTorTrajSol.resize(posSt_rArm.rows());

    for(int i = 0; i < LARMDOF; i++){
        lArmJntTrajSol[i].controllerInit();
        lArmJntTorTrajSol[i].controllerInit();
    }
    for(int i = 0; i < RARMDOF; i++){
        rArmJntTrajSol[i].controllerInit();
        rArmJntTorTrajSol[i].controllerInit();
    }

    //初始化 ifJntStab，防止 solveResOutp 访问 size=0 的 vector
    ifJntStab.resize(2);
    ifJntStab[0].assign(posSt_lArm.rows(), 0);
    ifJntStab[1].assign(posSt_rArm.rows(), 0);
}

void JntTrackSol::JntTrajCal(
    //目标量
    KDL::JntArray posDes_lArm, KDL::JntArray posDes_rArm,
    std::vector<double> JntTorDes_l, std::vector<double> JntTorDes_r,

    //反馈量
    KDL::JntArray posRet_lArm, KDL::JntArray posRet_rArm,
    KDL::JntArray velRet_lArm, KDL::JntArray velRet_rArm,
    std::vector<double> JntTorRet_l, std::vector<double> JntTorRet_r,

    modCtl mod_ctl_msg,
    bool ifRedefineStab
){
    lArmTrajLen.clear();
    rArmTrajLen.clear();

    //先获取初始角度与角速度输入值
    KDL::JntArray posInp_l, posInp_r;
    KDL::JntArray velInp_l, velInp_r;

    if(mod_ctl_msg.ifUseRet){
        //采用返回值作为上一轮的输入
        posInp_l = posRet_lArm;
        posInp_r = posRet_rArm;
        velInp_l = velRet_lArm;
        velInp_r = velRet_rArm;
    }else{
        //采用上一轮输出作为本轮输入
        posInp_l = posLst_lArm;
        posInp_r = posLst_rArm;
        velInp_l = velLst_lArm;
        velInp_r = velLst_rArm;
    }

    this->ifUseTemp = mod_ctl_msg.ifUseTemp;

    //设定静止控制参量：
    bool m_ifRedefineStab = ifFirstTemp || ifRedefineStab;
    ifFirstTemp = false;

    //更新输出控制参量
    ifJntStab = mod_ctl_msg.ifJntStab;

    //再计算轨迹
    //依据输入模式计算插补轨迹
    if(mod_ctl_msg.runinMod == 3){
        //点到点模式

        //先包装参量
        std::vector<runinMsgDotMod> runin_ctl_msg_dot_l;
        std::vector<runinMsgDotMod> runin_ctl_msg_dot_r;
        runin_ctl_msg_dot_l.resize(posRet_lArm.rows());
        runin_ctl_msg_dot_r.resize(posRet_rArm.rows());
        for(int i = 0; i < posRet_lArm.rows(); i++){
            runin_ctl_msg_dot_l[i].mPosSt = posInp_l(i);
            runin_ctl_msg_dot_l[i].mPosDes = posDes_lArm(i);
            runin_ctl_msg_dot_l[i].accTimeRateBase = mod_ctl_msg.accRateBase[0][i];
            runin_ctl_msg_dot_l[i].mTrajTime = mod_ctl_msg.TrajTime[0][i];
        }
        for(int i = 0; i < posRet_rArm.rows(); i++){
            runin_ctl_msg_dot_r[i].mPosSt = posInp_r(i);
            runin_ctl_msg_dot_r[i].mPosDes = posDes_rArm(i);
            runin_ctl_msg_dot_r[i].accTimeRateBase = mod_ctl_msg.accRateBase[1][i];
            runin_ctl_msg_dot_r[i].mTrajTime = mod_ctl_msg.TrajTime[1][i];
        }

        //再计算输出值
        for(int i = 0; i < lArmJntTrajSol.size(); i++){
            lArmJntTrajSol.at(i).init_stabTime(runin_ctl_msg_dot_l[i], mod_ctl_msg.m_stabMsg[0][i]);
            lArmTrajLen.push_back(lArmJntTrajSol.at(i).TrajLen);

            if(m_ifRedefineStab){
                lArmStab[i].init(runin_ctl_msg_dot_l[i].mPosSt);
            }
        }
        for(int i = 0; i < rArmJntTrajSol.size(); i++){
            rArmJntTrajSol.at(i).init_stabTime(runin_ctl_msg_dot_r[i], mod_ctl_msg.m_stabMsg[1][i]);
            rArmTrajLen.push_back(rArmJntTrajSol.at(i).TrajLen);

            if(m_ifRedefineStab){
                rArmStab[i].init(runin_ctl_msg_dot_r[i].mPosSt);
            }
        }

        //储存输出模式
        this->outp_mod = mod_ctl_msg.runinMod;
    }else{
        //连续求解插补模式

        //先包装参量
        std::vector<runinMsg> runin_ctl_msg_l;
        std::vector<runinMsg> runin_ctl_msg_r;
        runin_ctl_msg_l.resize(posRet_lArm.rows());
        runin_ctl_msg_r.resize(posRet_rArm.rows());

        for(int i = 0; i < posRet_lArm.rows(); i++){
            runin_ctl_msg_l[i].mPosSt = posInp_l(i);
            runin_ctl_msg_l[i].mPosDes = posDes_lArm(i);
            runin_ctl_msg_l[i].mVelSt = velInp_l(i);
            runin_ctl_msg_l[i].mTrajTime = mod_ctl_msg.upperMonitor_time;
            runin_ctl_msg_l[i].runinMod = mod_ctl_msg.runinMod;
        }
        for(int i = 0; i < posRet_rArm.rows(); i++){
            runin_ctl_msg_r[i].mPosSt = posInp_r(i);
            runin_ctl_msg_r[i].mPosDes = posDes_rArm(i);
            runin_ctl_msg_r[i].mVelSt = velInp_r(i);
            runin_ctl_msg_r[i].mTrajTime = mod_ctl_msg.upperMonitor_time;
            runin_ctl_msg_r[i].runinMod = mod_ctl_msg.runinMod;
        }

        //再计算输出值
        for(int i = 0; i < lArmJntTrajSol.size(); i++){
            lArmJntTrajSol.at(i).init_stabTime_continue(runin_ctl_msg_l[i]);
            lArmTrajLen.push_back(lArmJntTrajSol.at(i).TrajLen);

            if(m_ifRedefineStab){
                lArmStab[i].init(runin_ctl_msg_l[i].mPosSt);
            }
        }
        for(int i = 0; i < rArmJntTrajSol.size(); i++){
            rArmJntTrajSol.at(i).init_stabTime_continue(runin_ctl_msg_r[i]);
            rArmTrajLen.push_back(rArmJntTrajSol.at(i).TrajLen);

            if(m_ifRedefineStab){
                rArmStab[i].init(runin_ctl_msg_r[i].mPosSt);
            }
        }

        //储存输出模式
        this->outp_mod = mod_ctl_msg.runinMod;
    }

    /*  力前馈控制接口
    -------------------
    在这里添加力前馈控制函数
    这部分先空着，留着接口，以后做编辑
    */
    std::vector<double> TorFeedForward_l, TorFeedForward_r;
    TorFeedForward_l.assign(LARMDOF, 0.0);
    TorFeedForward_r.assign(RARMDOF, 0.0);

    if(mod_ctl_msg.runinMod == 3){
        //点到点控制 -- 
    }else{

    }

    //更新控制参量
    for(int i = 0; i < this->lArmJntTrajSol.size(); i++){
        std::pair<double, double> mRes = lArmJntTrajSol[i].lastTempPosVel();
        posLst_lArm(i) = mRes.first;
        velLst_lArm(i) = mRes.second;
    }
    for(int i = 0; i < this->rArmJntTrajSol.size(); i++){
        std::pair<double, double> mRes = rArmJntTrajSol[i].lastTempPosVel();
        posLst_rArm(i) = mRes.first;
        velLst_rArm(i) = mRes.second;
    }

}

std::vector<std::vector<ctlOutp>> JntTrackSol::solveResOutp(
    std::vector<double> JntTorRet_l, std::vector<double> JntTorRet_r
){
    std::vector<ctlOutp> mRes_l;
    std::vector<ctlOutp> mRes_r;
    std::vector<std::vector<ctlOutp>> mRes;


    if(this->outp_mod == 3){
        //获取左右臂杆的输出
        if(this->ifUseTemp){
            //循环输出
            for(int i = 0; i < lArmJntTrajSol.size(); i++){
                if(ifJntStab[0][i] != 1){
                    mRes_l.push_back(lArmJntTrajSol[i].desPos_get_stabTime());
                }else{
                    mRes_l.push_back(lArmStab[i].StabOutp());
                }
            }
            for(int i = 0; i < rArmJntTrajSol.size(); i++){
                if(ifJntStab[1][i] != 1){
                    mRes_r.push_back(rArmJntTrajSol[i].desPos_get_stabTime());
                }else{
                    mRes_r.push_back(rArmStab[i].StabOutp());
                }
            }            
        }else{
            //正常输出
            for(int i = 0; i < lArmJntTrajSol.size(); i++){
                if(ifJntStab[0][i] != 1){
                    mRes_l.push_back(lArmJntTrajSol[i].desPos_get_stabTimeStop());
                }else{
                    mRes_l.push_back(lArmStab[i].StabOutp());
                }
            }
            for(int i = 0; i < rArmJntTrajSol.size(); i++){
                if(ifJntStab[1][i] != 1){
                    mRes_r.push_back(rArmJntTrajSol[i].desPos_get_stabTimeStop());
                }else{
                    mRes_r.push_back(rArmStab[i].StabOutp());
                }
            }
        }
    }else{
        //获取左右臂杆的输出
        for(int i = 0; i < lArmJntTrajSol.size(); i++){
            if(ifJntStab[0][i] != 1){
                mRes_l.push_back(lArmJntTrajSol[i].desPos_get_stab_Continue());
            }else{
                mRes_l.push_back(lArmStab[i].StabOutp());
            }
        }
        for(int i = 0; i < rArmJntTrajSol.size(); i++){
            if(ifJntStab[1][i] != 1){
                mRes_r.push_back(rArmJntTrajSol[i].desPos_get_stab_Continue());
            }else{
                mRes_r.push_back(rArmStab[i].StabOutp());
            }
        }
    }

    /*  左右臂力控结果叠加部分
    --------------------------
    力控部分尚未开发，因此这一部分先空着
    */

    mRes.push_back(mRes_l);
    mRes.push_back(mRes_r);
    return mRes;
}

//-------------------------------------------------------------------------------------

//-----------------------------------------
//辅助函数 -- 将Frame与vector之间转化
std::vector<double> Frame_to_vector(KDL::Frame frameInp){
    std::vector<double> mRes;
    mRes.resize(6);
    for(int i = 0; i < 3; i++){
        mRes[i] = frameInp.p[i];
    }
    frameInp.M.GetRPY(mRes[3], mRes[4], mRes[5]);
    return mRes;
}

KDL::Frame vector_to_Frame(std::vector<double> vecInp){
    KDL::Frame mRes;
    for(int i = 0; i < 3; i++){
        mRes.p[i] = vecInp[i];
    }
    mRes.M = KDL::Rotation::RPY(vecInp[3], vecInp[4], vecInp[5]);

    return mRes;
}

//-----------------------------------------

bool FrameTrackSol::limitIKJointStep(
    KDL::JntArray& solution,
    const KDL::JntArray& seed,
    std::size_t joint_offset)
{
    if(!joint_step_limit_enabled || solution.rows() != seed.rows()){
        return false;
    }

    bool limited = false;
    for(unsigned int i = 0; i < solution.rows(); ++i){
        const std::size_t global_index = joint_offset + i;
        const double max_step = global_index < max_joint_step_rad.size()
            ? max_joint_step_rad[global_index] : 0.025;
        const double seed_value = seed(i);
        const double solution_value = solution(i);
        if(!std::isfinite(max_step) || max_step <= 0.0 || !std::isfinite(seed_value)){
            continue;
        }
        if(!std::isfinite(solution_value)){
            solution(i) = seed_value;
            limited = true;
            continue;
        }
        const double delta = solution_value - seed_value;
        if(std::abs(delta) > max_step){
            solution(i) = seed_value + std::copysign(max_step, delta);
            limited = true;
        }
    }

    if(limited && !joint_step_limit_reported){
        std::cerr << "[IK安全] 检测到关节目标跳变，已启用逐关节步长限幅；"
                  << "后续周期将继续追踪目标" << std::endl;
        joint_step_limit_reported = true;
    }
    return limited;
}

//核心成员函数 -- 逆解求解方法
void FrameTrackSol::IK_Traj_solve(
    KDL::Frame lArm_desPos, KDL::Frame rArm_desPos,
    KDL::JntArray lArmPosRet, KDL::JntArray rArmPosRet,
    bool ifUseRet,
    double TrajTime,
    int runinMod
){

    KDL::Frame lArm_desPos_cor, rArm_desPos_cor;
    lArm_desPos_cor = lArm_desPos;
    rArm_desPos_cor = rArm_desPos;
    for(int i = 0; i < 3; i++){
        lArm_desPos_cor.p[i] += torso_pos_l[i];
        rArm_desPos_cor.p[i] += torso_pos_r[i];
    }

    // ---------- 计算逆解 ----------
    std::vector<KDL::JntArray> all_posRes = mSol_Pino.IK_pinoSolver(
        lArm_desPos_cor, rArm_desPos_cor,
        lArmPosRet, rArmPosRet,
        ifUseRet
    );

    // The optimizer can switch IK branches near a singularity. Clamp the
    // candidate before creating a joint trajectory so the branch switch is
    // approached over multiple control cycles instead of one burst.
    if(all_posRes.size() >= 2){
        limitIKJointStep(all_posRes[0], posInp_l, 0);
        limitIKJointStep(all_posRes[1], posInp_r, LARMDOF);
    }

    // ---------- 针对连续轨迹进行插值计算 ----------

    lArm_TrajLen.clear();
    rArm_TrajLen.clear();

    //先包装参量
    std::vector<runinMsg> runin_ctl_msg_l;
    std::vector<runinMsg> runin_ctl_msg_r;
    runin_ctl_msg_l.resize(lArmPosRet.rows());
    runin_ctl_msg_r.resize(rArmPosRet.rows());

    for(int i = 0; i < lArmPosRet.rows(); i++){
        runin_ctl_msg_l[i].mPosSt = posInp_l(i);
        runin_ctl_msg_l[i].mPosDes = all_posRes[0](i);
        runin_ctl_msg_l[i].mVelSt = velInp_l(i);
        runin_ctl_msg_l[i].mTrajTime = TrajTime;
        runin_ctl_msg_l[i].runinMod = runinMod;
    }
    for(int i = 0; i < rArmPosRet.rows(); i++){
        runin_ctl_msg_r[i].mPosSt = posInp_r(i);
        runin_ctl_msg_r[i].mPosDes = all_posRes[1](i);
        runin_ctl_msg_r[i].mVelSt = velInp_r(i);
        runin_ctl_msg_r[i].mTrajTime = TrajTime;
        runin_ctl_msg_r[i].runinMod = runinMod;
    }

    //再计算输出值
    for(int i = 0; i < lArm_JntTrajSol.size(); i++){
        runinMsg temp_runinMsg = runin_ctl_msg_l[i];
        temp_runinMsg.mTrajTime = runin_ctl_msg_l[i].mTrajTime;
        lArm_JntTrajSol.at(i).init_stabTime_continue(temp_runinMsg);
        lArm_TrajLen.push_back(lArm_JntTrajSol.at(i).TrajLen);
    }
    for(int i = 0; i < rArm_JntTrajSol.size(); i++){
        runinMsg temp_runinMsg = runin_ctl_msg_r[i];
        temp_runinMsg.mTrajTime = runin_ctl_msg_r[i].mTrajTime;
        rArm_JntTrajSol.at(i).init_stabTime_continue(temp_runinMsg);
        rArm_TrajLen.push_back(rArm_JntTrajSol.at(i).TrajLen);
    }
}


/*逆解轨迹与正解算法封装*/
bool FrameTrackSol::init_solver(
    JntLimMsg lArm_LimJnt, JntLimMsg rArm_LimJnt,       //关节限制配置
    std::vector<double> JntAngConf_l, std::vector<double> JntAngConf_r,     //关节初始角度配置
    optiPrintMsgCtl mPrintMsgPkg,           //程序打印控制参数
    optimizerCtl m_opti_ctl_msg         //优化器参数
){
    joint_step_limit_enabled = m_opti_ctl_msg.joint_step_limit_enabled;
    max_joint_step_rad = m_opti_ctl_msg.max_joint_step_rad;
    joint_step_limit_reported = false;
    if(max_joint_step_rad.size() < static_cast<std::size_t>(JntAngConf_l.size() + JntAngConf_r.size())){
        max_joint_step_rad.assign(
            static_cast<std::size_t>(JntAngConf_l.size() + JntAngConf_r.size()), 0.025);
    }

    //初始化求解器配置
    PathChoose mChoose_l, mChoose_r;
    mChoose_l.urdf_choose = 0;      mChoose_r.urdf_choose = 0;
    mChoose_l.base_choose = 0;      mChoose_r.base_choose = 1;
    mChoose_l.tip_choose = 0;       mChoose_r.tip_choose = 1;

    this->tempT = TEMPTIME;
    if_not_firstTemp = false;
    stabTimesCount = 0;
    allStabTimes_l = 0;
    allStabTimes_r = 0;

    controlMod_lastTemp = 0;

    if (!mSol_Pino.Init(
        mChoose_l, mChoose_r,
        lArm_LimJnt, rArm_LimJnt,
        JntAngConf_l, JntAngConf_r,
        mPrintMsgPkg,
        m_opti_ctl_msg
    )) {
        std::cerr << "FATAL: FrameTrackSol::init_solver() FAILED — "
                  << "IkFk_solverPino::Init() returned false. "
                  << "The Python-based IK/FK solver could not be initialized." << std::endl;
        return false;
    }

    //初始化过程参量
    lArmLstPos.resize(JntAngConf_l.size());
    rArmLstPos.resize(JntAngConf_r.size());
    lArmLstVel.resize(JntAngConf_l.size());
    rArmLstVel.resize(JntAngConf_r.size());
    for(int i = 0; i < JntAngConf_l.size(); i++){
        lArmLstPos(i) = JntAngConf_l[i];
        lArmLstVel(i) = 0.0;
    }
    for(int j = 0; j < JntAngConf_r.size(); j++){
        rArmLstPos(j) = JntAngConf_r[j];
        rArmLstVel(j) = 0.0;
    }

    lArm_JntTrajSol.resize(JntAngConf_l.size());
    rArm_JntTrajSol.resize(JntAngConf_r.size());

    lArm_DescTrajSol.resize(6);
    rArm_DescTrajSol.resize(6);

    for(int i = 0; i < LARMDOF; i++){
        lArm_JntTrajSol[i].controllerInit();
    }
    for(int i = 0; i < RARMDOF; i++){
        rArm_JntTrajSol[i].controllerInit();
    }
    for(int i = 0; i < 6; i++){
        lArm_DescTrajSol[i].controllerInit();
        rArm_DescTrajSol[i].controllerInit();
    }

    if_not_firstTemp = false;
    ifUseSolve = false;

    //修正参量初始化
    lArm_posLst_velCor = JntAngConf_l;
    rArm_posLst_velCor = JntAngConf_r;

    //初始化 ifJntStab，防止 solveResOutp 访问 size=0 的 vector
    ifJntStab.resize(2);
    ifJntStab[0].assign(LARMDOF, 0);
    ifJntStab[1].assign(RARMDOF, 0);

    return true;
}

void FrameTrackSol::FrameTrack(
    bool ifUpdateLim,               //判断量 -- 是否更新限制信息
    JntLimMsg allArm_LimJnt,        //求解关节限制信息，将所有的关节信息都放在这里面

    KDL::Frame lArm_desPos, KDL::Frame rArm_desPos,         //求解目标位置
    std::vector<double> lArm_desTor, std::vector<double> rArm_desTor,   //目标前馈力矩

    KDL::JntArray lArmPosRet, KDL::JntArray rArmPosRet,     //位置返回值
    KDL::JntArray lArmVelRet, KDL::JntArray rArmVelRet,     //速度返回值
    std::vector<double> lArmTorRet, std::vector<double> rArmTorRet,     //关节力矩反馈值

    modCtl mod_ctl_msg          //数据传输控制过程
){
    //储存数据传输控制过程
    modCtlMsg = mod_ctl_msg;

    //模型返回值解包
    if(mod_ctl_msg.ifUseRet){
        //使用返回值
        posInp_l = lArmPosRet;
        posInp_r = rArmPosRet;
        velInp_l = lArmVelRet;
        velInp_r = rArmVelRet;
    }else{
        //不使用返回值
        posInp_l = lArmLstPos;
        posInp_r = rArmLstPos;
        velInp_l = lArmLstVel;
        velInp_r = rArmLstVel;
    }

    //更新模型封装
    if(ifUpdateLim && if_not_firstTemp){
        mSol_Pino.solverUpdate(allArm_LimJnt);
    }
    if_not_firstTemp = true;
    this->ifUseTemp = mod_ctl_msg.ifUseTemp;
    controlMod_lastTemp = mod_ctl_msg.runinMod;

    //依据输入模式计算插补轨迹
    if(mod_ctl_msg.runinMod == 3){
        //点到点模式

        //定义数据包装
        std::vector<runinMsgDotMod> lArm_runinMsgDot, rArm_runinMsgDot;
        std::vector<runinStabMsg> lArm_runinStabMsg, rArm_runinStabMsg;
        lArm_runinMsgDot.resize(6);         rArm_runinMsgDot.resize(6);
        lArm_runinStabMsg.resize(6);        rArm_runinStabMsg.resize(6);
        
        //获取目标值，反馈值
        std::vector<double> lArm_desPos_vec, rArm_desPos_vec;
        std::vector<double> lArm_posSt_vec, rArm_posSt_vec;
        lArm_desPos_vec = Frame_to_vector(lArm_desPos);
        rArm_desPos_vec = Frame_to_vector(rArm_desPos);
        std::vector<KDL::Frame> posInp_frame;
        posInp_frame = mSol_Pino.Fk_pinoSolver(posInp_l, posInp_r);
        lArm_posSt_vec = Frame_to_vector(posInp_frame[0]);
        rArm_posSt_vec = Frame_to_vector(posInp_frame[1]);

        //进行六个维度的目标数据包装
        for(int i = 0; i < 6; i++){
            lArm_runinMsgDot[i].mPosDes = lArm_desPos_vec[i];
            lArm_runinMsgDot[i].mPosSt = lArm_posSt_vec[i];
            lArm_runinMsgDot[i].accTimeRateBase = mod_ctl_msg.accRateBase[0][0];
            lArm_runinMsgDot[i].mTrajTime = mod_ctl_msg.TrajTime[0][0];

            rArm_runinMsgDot[i].mPosDes = rArm_desPos_vec[i];
            rArm_runinMsgDot[i].mPosSt = rArm_posSt_vec[i];
            rArm_runinMsgDot[i].accTimeRateBase = mod_ctl_msg.accRateBase[1][0];
            rArm_runinMsgDot[i].mTrajTime = mod_ctl_msg.TrajTime[1][0];

            lArm_runinStabMsg[i] = mod_ctl_msg.m_stabMsg[0][0];
            rArm_runinStabMsg[i] = mod_ctl_msg.m_stabMsg[1][0];
        }

        //计算六维度笛卡尔空间插值
        for(int i = 0; i < 6; i++){
            lArm_DescTrajSol[i].clear();
            rArm_DescTrajSol[i].clear();

            lArm_DescTrajSol[i].init_stabTime(lArm_runinMsgDot[i], lArm_runinStabMsg[i]);
            rArm_DescTrajSol[i].init_stabTime(rArm_runinMsgDot[i], rArm_runinStabMsg[i]);
        }

        //储存输出模式
        this->outp_mod = mod_ctl_msg.runinMod;
    }else{
        //连续求解插补模式

        IK_Traj_solve(
            lArm_desPos, rArm_desPos,
            posInp_l, posInp_r,
            mod_ctl_msg.ifUseRet,
            mod_ctl_msg.upperMonitor_time,
            mod_ctl_msg.runinMod
        );

        //储存输出模式
        this->outp_mod = mod_ctl_msg.runinMod;
    }

    //关节静止模式初始化
    lArmStab.resize(LARMDOF);
    rArmStab.resize(RARMDOF);
    ifJntStab = mod_ctl_msg.ifJntStab;
    if(ifFirstTemp){
        for(int i = 0; i < ifJntStab[0].size(); i++){
            if(ifJntStab[0][i] == 1){
                lArmStab[i].init(lArmLstPos(i));
            }
        }
        for(int i = 0; i < ifJntStab[1].size(); i++){
            if(ifJntStab[1][i] == 1){
                rArmStab[i].init(rArmLstPos(i));
            }
        }
    }
    ifFirstTemp = false;

    /**/
    if(this->outp_mod != -1){
        //更新控制参量
        for(int i = 0; i < this->lArm_JntTrajSol.size(); i++){
            std::pair<double, double> mRes = lArm_JntTrajSol[i].lastTempPosVel();
            lArmLstPos(i) = mRes.first;
            lArmLstVel(i) = mRes.second;
        }
        for(int i = 0; i < this->rArm_JntTrajSol.size(); i++){
            std::pair<double, double> mRes = rArm_JntTrajSol[i].lastTempPosVel();
            rArmLstPos(i) = mRes.first;
            rArmLstVel(i) = mRes.second;
        }
    }
    /**/
}

std::vector<std::vector<ctlOutp>> FrameTrackSol::IKsolveResOutp(
    std::vector<double> lArm_TorRet, std::vector<double> rArm_TorRet
){
    //输出逆解轨迹函数 -- 依据求解模式

    std::vector<ctlOutp> mRes_l;
    std::vector<ctlOutp> mRes_r;
    std::vector<std::vector<ctlOutp>> mRes;

    if(this->outp_mod == 3){
        //获取当前笛卡尔空间点位
        std::vector<ctlOutp> l_descRes, r_descRes;      //左右臂点位
        std::vector<double> l_temp_desPosDesc, r_temp_desPosDesc;
        std::vector<double> l_temp_desVelDesc, r_temp_desVelDesc;
        l_descRes.resize(6);            r_descRes.resize(6);
        l_temp_desPosDesc.resize(6);    r_temp_desPosDesc.resize(6);
        l_temp_desVelDesc.resize(6);    r_temp_desVelDesc.resize(6);

        if(this->ifUseTemp){
            //循环输出
            for(int i = 0; i < 6; i++){
                l_descRes[i] = lArm_DescTrajSol[i].desPos_get_stabTime();
                r_descRes[i] = rArm_DescTrajSol[i].desPos_get_stabTime();
            }
        }else{
            //正常输出
            for(int i = 0; i < 6; i++){
                l_descRes[i] = lArm_DescTrajSol[i].desPos_get_stabTimeStop();
                r_descRes[i] = rArm_DescTrajSol[i].desPos_get_stabTimeStop();
            }
        }

        //静止轮计数器
        // 轨迹 posTraj_stabTime_half 已包含 stabTimes 驻留段，
        // 各DOF的 ifReachEnd=true 表示轨迹（含驻留）已完成，直接同步到输出
        bool temp_ifReachEnd_l = true;
        bool temp_ifReachEnd_r = true;
        for(int i = 0; i < 6; i++){
            if(!l_descRes[i].ifReachEnd){
                temp_ifReachEnd_l = false;
            }
            if(!r_descRes[i].ifReachEnd){
                temp_ifReachEnd_r = false;
            }
        }

        //输出解包
        for(int i = 0; i < 6; i++){
            l_temp_desPosDesc[i] = l_descRes[i].posDes;
            r_temp_desPosDesc[i] = r_descRes[i].posDes;

            l_temp_desVelDesc[i] = l_descRes[i].velDes;
            r_temp_desVelDesc[i] = r_descRes[i].velDes;
        }

        //计算逆解 —— 先用最新关节位置刷新种子，避免IK被过期状态拉偏
        posInp_l = lArmLstPos;
        posInp_r = rArmLstPos;
        velInp_l = lArmLstVel;
        velInp_r = rArmLstVel;

        IK_Traj_solve(
            vector_to_Frame(l_temp_desPosDesc), vector_to_Frame(r_temp_desPosDesc),
            lArmLstPos, rArmLstPos,
            true,
            TEMPTIME,
            1
        );

        std::vector<double> lArm_JntPosDes, rArm_JntPosDes;

        for(int i = 0; i < LARMDOF; i++){
            ctlOutp temp_res_l;
            temp_res_l = lArm_JntTrajSol[i].desPos_get_stab_Continue();
            temp_res_l.ifReachEnd = temp_ifReachEnd_l;
            mRes_l.push_back(temp_res_l);

            lArm_JntPosDes.push_back(temp_res_l.posDes);
        }
        for(int i = 0; i < RARMDOF; i++){
            ctlOutp temp_res_r;
            temp_res_r = rArm_JntTrajSol[i].desPos_get_stab_Continue();
            temp_res_r.ifReachEnd = temp_ifReachEnd_r;
            mRes_r.push_back(temp_res_r);

            rArm_JntPosDes.push_back(temp_res_r.posDes);
        }


    }else if(this->outp_mod == -1){
        for(int i = 0; i < lArm_JntTrajSol.size(); i++){
            if(ifJntStab[0][i] != 1){
                ctlOutp temp_res_l;
                temp_res_l.posDes = lArmDesPos(i);
                if(ifUseSolve){
                    temp_res_l.velDes = (lArmDesPos(i) - lArmLstPos(i))/tempT;
                }else{
                    temp_res_l.velDes = 0.0;
                }

                temp_res_l.grav_TorDes = 0.0;
                temp_res_l.interia_TorDes = 0.0;
                temp_res_l.feedforward_TorDes = 0.0;

                temp_res_l.ifReachEnd = true;
                mRes_l.push_back(temp_res_l);
            }else{
                mRes_l.push_back(lArmStab[i].StabOutp());
            }
        }
        for(int i = 0; i < rArm_JntTrajSol.size(); i++){
            if(ifJntStab[1][i] != 1){
                ctlOutp temp_res_r;
                temp_res_r.posDes = rArmDesPos(i);
                if(ifUseSolve){
                    temp_res_r.velDes = (rArmDesPos(i) - rArmLstPos(i))/tempT;
                }else{
                    temp_res_r.velDes = 0.0;
                }

                temp_res_r.grav_TorDes = 0.0;
                temp_res_r.interia_TorDes = 0.0;
                temp_res_r.feedforward_TorDes = 0.0;

                temp_res_r.ifReachEnd = true;
                mRes_r.push_back(temp_res_r);
            }else{
                mRes_r.push_back(rArmStab[i].StabOutp());
            }
        }
        if(ifUseSolve){
            ifUseSolve = false;
        }

    }else{
        //获取左右臂杆的输出
        for(int i = 0; i < lArm_JntTrajSol.size(); i++){
            if(ifJntStab[0][i] != 1){
                mRes_l.push_back(lArm_JntTrajSol[i].desPos_get_stab_Continue());
            }else{
                mRes_l.push_back(lArmStab[i].StabOutp());
            }
        }
        for(int i = 0; i < rArm_JntTrajSol.size(); i++){
            if(ifJntStab[1][i] != 1){
                mRes_r.push_back(rArm_JntTrajSol[i].desPos_get_stab_Continue());
            }else{
                mRes_r.push_back(rArmStab[i].StabOutp());
            }
        }
    }

    //修正速度输出 -- 治标不治本
    for(int i = 0; i < lArm_JntTrajSol.size(); i++){
        if(ifJntStab[0][i] != 1){
            //mRes_l[i].velDes = (mRes_l[i].posDes - lArm_posLst_velCor[i]) / tempT;
        }
    }
    for(int i = 0; i < rArm_JntTrajSol.size(); i++){
        if(ifJntStab[1][i] != 1){
            //mRes_r[i].velDes = (mRes_r[i].posDes - rArm_posLst_velCor[i]) / tempT;
        }
    }

    for(int i = 0; i < lArm_JntTrajSol.size(); i++){
        if(ifJntStab[0][i] != 1){
            lArm_posLst_velCor[i] = mRes_l[i].posDes;
        }
    }
    for(int i = 0; i < rArm_JntTrajSol.size(); i++){
        if(ifJntStab[1][i] != 1){
            rArm_posLst_velCor[i] = mRes_r[i].posDes;
        }
    }

    for(int i = 0; i < this->lArm_JntTrajSol.size(); i++){
        std::pair<double, double> mResFrame_l = lArm_JntTrajSol[i].lastTempPosVel();
        lArmLstPos(i) = mResFrame_l.first;
        lArmLstVel(i) = mResFrame_l.second;
    }
    for(int i = 0; i < this->rArm_JntTrajSol.size(); i++){
        std::pair<double, double> mResFrame_r = rArm_JntTrajSol[i].lastTempPosVel();
        rArmLstPos(i) = mResFrame_r.first;
        rArmLstVel(i) = mResFrame_r.second;
    }

    //直接输出

    /*  力控叠加部分
    ------------------
    此部分尚未作开发，因此先空着
    */

    mRes.push_back(mRes_l);
    mRes.push_back(mRes_r);
    return mRes;
}

//======================================================================
// ForceComp_floatBase — 带有浮动基座的动力学补偿类 实现
//======================================================================

// 辅助函数：RPY角速度导数 → 空间角速度的转换矩阵
// [wx, wy, wz]^T = E(r,p,y) * [dr/dt, dp/dt, dy/dt]^T
inline Eigen::Matrix3d rpyDot_to_omega(double roll, double pitch, double yaw){
    double cr = std::cos(roll), sr = std::sin(roll);
    double cp = std::cos(pitch), sp = std::sin(pitch);
    double cy = std::cos(yaw),   sy = std::sin(yaw);

    Eigen::Matrix3d E;
    E <<  cy*cp,  -sy,  0.0,
          sy*cp,   cy,  0.0,
          -sp,    0.0,  1.0;
    return E;
}

bool ForceComp_floatBase::init(
    std::vector<double> base_pos,
    KDL::JntArray stPos_lArm, KDL::JntArray stPos_rArm
){
    // 销毁旧模型
    if(model_.njoints > 0){
        model_ = pinocchio::Model();
        data_ = pinocchio::Data();
    }

    // 从全局变量 robotProcessedModel 复制模型
    model_ = robotProcessedModel;
    data_  = pinocchio::Data(model_);

    // 计算手臂关节在 q/v 中的偏移量
    // 模型结构：universe(0) → floating_base_joint(1, FreeFlyer) → arm_joints(2...)
    if(model_.njoints < 3){
        std::cerr << "错误：浮动基座模型结构异常，关节数=" << model_.njoints << std::endl;
        return false;
    }
    arm_q_offset_ = model_.joints[1].nq();  // 手臂 q 起始索引
    arm_v_offset_ = model_.joints[1].nv();  // 手臂 v 起始索引
    arm_nj_       = model_.nv - arm_v_offset_;  // 手臂关节数 (从模型动态计算)

    // 分配外部状态向量 (RPY 6维)
    base_q_ = Eigen::VectorXd::Zero(base_nq_);  // 6维 RPY
    base_v_ = Eigen::VectorXd::Zero(base_nv_);  // 6维 RPY导数
    this->temp_T = TEMPTIME;

    // 分配 Pinocchio 内部全状态
    q_ = Eigen::VectorXd::Zero(model_.nq);
    v_ = Eigen::VectorXd::Zero(model_.nv);
    a_ = Eigen::VectorXd::Zero(model_.nv);
    v_lst_ = Eigen::VectorXd::Zero(model_.nv);

    // 初始化输入输出开关
    for(int i = 0; i < arm_nj_; i++){
        inte_inp[i]  = 1;
        inte_outp[i] = 1;
    }

    // 从全局变量拷贝重力补偿参数到成员变量，防止全局变量被意外修改
    mRobotGravCompInte = robotGravCompInte;

    // ===== 初始化基座位姿 (来源于输入参数 base_pos) =====
    for(int i = 0; i < base_nq_ && i < static_cast<int>(base_pos.size()); i++){
        base_q_(i) = base_pos[i];
    }
    base_v_.setZero();  // 初始速度为零

    // RPY → Quaternion 转换，填入 Pinocchio q_ (FreeFlyer关节: 7维)
    double roll  = base_q_(3);
    double pitch = base_q_(4);
    double yaw   = base_q_(5);
    Eigen::Quaterniond base_quat(
        Eigen::AngleAxisd(yaw,   Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(roll,  Eigen::Vector3d::UnitX())
    );
    q_(0) = base_q_(0);     // x
    q_(1) = base_q_(1);     // y
    q_(2) = base_q_(2);     // z
    q_(3) = base_quat.x();
    q_(4) = base_quat.y();
    q_(5) = base_quat.z();
    q_(6) = base_quat.w();
    v_.head(arm_v_offset_).setZero();

    // ===== 初始化手臂关节位置 (来源于输入参数 stPos_lArm, stPos_rArm) =====
    // 模型关节顺序遵循URDF定义；此处按左臂在前、右臂在后的惯例填充
    int lArm_start = arm_q_offset_;
    int rArm_start = arm_q_offset_ + LARMDOF;
    for(unsigned int i = 0; i < stPos_lArm.rows() && (lArm_start + i) < static_cast<unsigned int>(q_.size()); i++){
        q_(lArm_start + i) = stPos_lArm(i);
    }
    for(unsigned int i = 0; i < stPos_rArm.rows() && (rArm_start + i) < static_cast<unsigned int>(q_.size()); i++){
        q_(rArm_start + i) = stPos_rArm(i);
    }

    pinocchio::forwardKinematics(model_, data_, q_);

    //初始化加速度滤波器
    acc_ftr.resize(LARMDOF + RARMDOF);
    for(int i = 0; i < LARMDOF + RARMDOF; i++){
        acc_ftr[i].FilterInit(0.0, mRobotGravCompInte.acc_ftr[i].Q_gain, mRobotGravCompInte.acc_ftr[i].R_gain, TEMPTIME);
    }

    printModelInfo();
    return true;
}

void ForceComp_floatBase::baseStateUpdate(
    const std::vector<double>& base_pos,      // [x, y, z, roll, pitch, yaw]
    const std::vector<double>& base_twist)    // [vx, vy, vz, droll, dpitch, dyaw]
{
    // 从全局变量抓取重力补偿配置
    mRobotGravCompInte = robotGravCompInte;


    // ===== 1. 外部 RPY 表示: base_q_ (6D) =====
    for(int i = 0; i < base_nq_; i++){
        base_q_(i) = base_pos[i];
    }
    for(int i = 0; i < base_nv_; i++){
        base_v_(i) = base_twist[i];
    }

    double roll  = base_q_(3);
    double pitch = base_q_(4);
    double yaw   = base_q_(5);

    // ===== 2. RPY → Quaternion, 填入 Pinocchio q_ (7D) =====
    Eigen::Quaterniond base_quat(
        Eigen::AngleAxisd(yaw,   Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(roll,  Eigen::Vector3d::UnitX())
    );
    q_(0) = base_q_(0);
    q_(1) = base_q_(1);
    q_(2) = base_q_(2);
    q_(3) = base_quat.x();
    q_(4) = base_quat.y();
    q_(5) = base_quat.z();
    q_(6) = base_quat.w();

    // ===== 3. RPY导数 → 角速度, 填入 Pinocchio v_ (6D) =====
    Eigen::Vector3d rpyDot(base_twist[3], base_twist[4], base_twist[5]);
    Eigen::Vector3d omega = rpyDot_to_omega(roll, pitch, yaw) * rpyDot;

    v_(0) = base_twist[0];
    v_(1) = base_twist[1];
    v_(2) = base_twist[2];
    v_(3) = omega(0);
    v_(4) = omega(1);
    v_(5) = omega(2);
}

std::vector<double> ForceComp_floatBase::gravComp(
    std::vector<double> lArm_pos, std::vector<double> lArm_vel,
    std::vector<double> rArm_pos, std::vector<double> rArm_vel)
{
    if (!mRobotGravCompInte.ifUseGravComp) return std::vector<double>(arm_nj_, 0.0);
    // ---- 填入手臂关节位置 (从 arm_q_offset_ 开始) ----
    int idx = arm_q_offset_;
    for(int i = 0; i < LARMDOF; i++){
        q_(idx) = lArm_pos[i] * inte_inp[i];
        idx++;
    }
    for(int i = 0; i < RARMDOF; i++){
        q_(idx) = rArm_pos[i] * inte_inp[i + LARMDOF];
        idx++;
    }

    // ---- 计算广义重力 (含基座+手臂) ----
    Eigen::VectorXd tau = pinocchio::computeGeneralizedGravity(model_, data_, q_);

    // ---- 仅提取手臂关节力矩 (跳过基座 FreeFlyer 的6维) ----
    std::vector<double> tau_vector(arm_nj_);
    for(int i = 0; i < arm_nj_; i++){
        tau_vector[i] = tau[arm_v_offset_ + i] / mRobotGravCompInte.gravCompInte[i] * inte_outp[i];
    }

    return tau_vector;
}

std::vector<double> ForceComp_floatBase::InteriaMulComp(
    std::vector<double> lArm_pos, std::vector<double> lArm_vel,
    std::vector<double> rArm_pos, std::vector<double> rArm_vel,
    const std::vector<double>& acceleration)
{
    if (!mRobotGravCompInte.ifUseInertiaComp && !mRobotGravCompInte.ifUseCoriolisComp)
        return std::vector<double>(arm_nj_, 0.0);
    // ---- 填入手臂关节位置 ----
    int idx_q = arm_q_offset_;
    for(int i = 0; i < LARMDOF; i++){
        q_(idx_q) = lArm_pos[i] * inte_inp[i];
        idx_q++;
    }
    for(int i = 0; i < RARMDOF; i++){
        q_(idx_q) = rArm_pos[i] * inte_inp[i + LARMDOF];
        idx_q++;
    }

    // ---- 填入手臂关节速度 ----
    int idx_v = arm_v_offset_;
    for(int i = 0; i < LARMDOF; i++){
        v_(idx_v) = lArm_vel[i] * inte_inp[i];
        idx_v++;
    }
    for(int i = 0; i < RARMDOF; i++){
        v_(idx_v) = rArm_vel[i] * inte_inp[i + LARMDOF];
        idx_v++;
    }

    // ---- 通过差分计算关节加速度 ----
    for(int i = arm_v_offset_; i < model_.nv; i++){
        a_(i) = (v_(i) - v_lst_(i)) / temp_T;
        v_lst_(i) = v_(i);

        //对加速度进行滤波处理并输出 (注意: acc_ftr 索引从0开始, 需减去 arm_v_offset_)
        if (acceleration.size() == static_cast<size_t>(arm_nj_)) {
            a_(i) = acceleration[i - arm_v_offset_] * inte_inp[i - arm_v_offset_];
        } else {
            a_(i) = mRobotGravCompInte.ifUseInertiaComp
                ? acc_ftr[i - arm_v_offset_].FilteredResCal(a_(i)) : 0.0;
        }
    }

    //针对速度进行画图分析
    std::vector<double> vec_l, vec_r;
    std::vector<double> acc_ll, acc_rr;
    acc_ll.resize(LARMDOF);
    acc_rr.resize(RARMDOF);
    for(int i = 0; i < LARMDOF; i++){
        acc_ll[i] = a_(i + 6);
    }
    for(int i = 0; i < RARMDOF; i++){
        acc_rr[i] = a_(i + LARMDOF + 6);
    }

    // ---- 科氏力项 ----
    Eigen::VectorXd coriolis = Eigen::VectorXd::Zero(model_.nv);
    if (mRobotGravCompInte.ifUseCoriolisComp)
        coriolis = pinocchio::computeCoriolisMatrix(model_, data_, q_, v_) * v_;

    // ---- 惯性力项 (质量矩阵 × 加速度) ----
    Eigen::VectorXd inertia = Eigen::VectorXd::Zero(model_.nv);
    if (mRobotGravCompInte.ifUseInertiaComp) {
        Eigen::MatrixXd mass_matrix = pinocchio::crba(model_, data_, q_);
        mass_matrix.triangularView<Eigen::StrictlyLower>() =
            mass_matrix.transpose().triangularView<Eigen::StrictlyLower>();
        inertia = mass_matrix * a_;
    }

    // ---- 仅提取手臂关节力矩 (跳过基座6维) ----
    std::vector<double> tau_vec(arm_nj_);
    for(int i = 0; i < arm_nj_; i++){
        int j = arm_v_offset_ + i;
        tau_vec[i] = coriolis[j] / mRobotGravCompInte.coriousCompInte[i] * inte_outp[i]
                   + inertia[j]  / mRobotGravCompInte.InteriaCompInte[i]  * inte_outp[i];
    }

    return tau_vec;
}


void ForceComp_floatBase::printModelInfo(){
    std::cout << "\n========== 浮动基座 Pinocchio 模型摘要 ==========" << std::endl;
    std::cout << "模型名称: " << model_.name << std::endl;
    std::cout << "总关节数: " << model_.njoints << std::endl;
    std::cout << "配置空间维度 (nq): " << model_.nq << std::endl;
    std::cout << "速度空间维度 (nv): " << model_.nv << std::endl;
    std::cout << "外部基座维度 (RPY): q=" << base_nq_
              << "  v=" << base_nv_ << std::endl;;
    std::cout << "手臂关节数: " << arm_nj_ << std::endl;
    std::cout << "手臂偏移: q=" << arm_q_offset_
              << "  v 偏移: " << arm_v_offset_ << std::endl;
    std::cout << "基座位姿 [x,y,z,roll,pitch,yaw]: ";
    for(int i = 0; i < base_q_.size(); i++)
        std::cout << base_q_(i) << " ";
    std::cout << std::endl;
    std::cout << "基座速度 [vx,vy,vz, dr/dt, dp/dt, dy/dt]: ";
    for(int i = 0; i < base_v_.size(); i++)
        std::cout << base_v_(i) << " ";
    std::cout << std::endl;
    std::cout << "==================================================\n" << std::endl;
}
