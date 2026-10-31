#include "robot_config/config.h"

#include <Eigen/Dense>
#include <algorithm>
#include <fstream>
#include <sstream>
#include <cmath>
#include <pinocchio/multibody/fwd.hpp>
#include <pinocchio/multibody/joint/joint-free-flyer.hpp>

std::string yaml_path_zeroPos;
std::string yaml_path_desc_zeroPos;
std::string yaml_path_initialPD;
std::string yaml_path_jointLink;
std::string yaml_path_robotStructConfig;
std::string yaml_path_gravCompConfig;

//默认位置定义
std::vector<double> lArm_zeroPos;
std::vector<double> rArm_zeroPos;

std::vector<double> l_descZeroPos;
std::vector<double> r_descZeroPos;

//默认PD参数定义
std::vector<double> lArm_defaultKp_low;
std::vector<double> lArm_defaultKd_low;
std::vector<double> rArm_defaultKp_low;
std::vector<double> rArm_defaultKd_low;

std::vector<double> lArm_defaultKp_mid;
std::vector<double> lArm_defaultKd_mid;
std::vector<double> rArm_defaultKp_mid;
std::vector<double> rArm_defaultKd_mid;

std::vector<double> lArm_defaultKp_high;
std::vector<double> lArm_defaultKd_high;
std::vector<double> rArm_defaultKp_high;
std::vector<double> rArm_defaultKd_high;

std::vector<double> lArm_defaultKp_superhigh;
std::vector<double> lArm_defaultKd_superhigh;
std::vector<double> rArm_defaultKp_superhigh;
std::vector<double> rArm_defaultKd_superhigh;

std::vector<double> lArm_defaultKp_tag;
std::vector<double> lArm_defaultKd_tag;
std::vector<double> rArm_defaultKp_tag;
std::vector<double> rArm_defaultKd_tag;

//默认机器人关节连杆体系配置参数
RobotJntMsg robotJointLinkConfig;

//默认机器人模型结构配置参数
RobotStructure robotModelStructConfig;
pinocchio::Model robotProcessedModel;

//默认机器人重力补偿配置参数
gravCompInte robotGravCompInte;

//基于优化器的逆解求解器参数
optimizerCtl optimizerControParam;
optiPrintMsgCtl msgPringControlParam;

//总控流程参数集合
TransLoopCtl dataTransLoopCtl;

//==============================================================================
// 构造函数
//==============================================================================

ZeroPosConfig::ZeroPosConfig()
    : lArm_zeroPos(LARMDOF, 0.0)
    , rArm_zeroPos(RARMDOF, 0.0)
    , yaml_path_("")
    , loaded_(false)
{
}

ZeroPosConfig::ZeroPosConfig(const std::string& yaml_path)
    : lArm_zeroPos(LARMDOF, 0.0)
    , rArm_zeroPos(RARMDOF, 0.0)
    , yaml_path_(yaml_path)
    , loaded_(false)
{
    loadFromYaml(yaml_path);
}

//==============================================================================
// 从 YAML 文件加载零位配置
//==============================================================================

bool ZeroPosConfig::loadFromYaml(const std::string& yaml_path){
    yaml_path_ = yaml_path;

    try{
        // 加载 YAML 文件
        YAML::Node config = YAML::LoadFile(yaml_path_);

        // 读取左臂零位
        if(config["left_arm"]){
            const YAML::Node& left = config["left_arm"];
            lArm_zeroPos.clear();
            lArm_zeroPos.push_back(left["left_shoulder_pitch_joint"].as<double>(0.0));
            lArm_zeroPos.push_back(left["left_shoulder_roll_joint"].as<double>(0.0));
            lArm_zeroPos.push_back(left["left_shoulder_yaw_joint"].as<double>(0.0));
            lArm_zeroPos.push_back(left["left_elbow_joint"].as<double>(0.0));
            lArm_zeroPos.push_back(left["left_wrist_roll_joint"].as<double>(0.0));
            lArm_zeroPos.push_back(left["left_wrist_pitch_joint"].as<double>(0.0));
            lArm_zeroPos.push_back(left["left_wrist_yaw_joint"].as<double>(0.0));
        }else{
            std::cerr << "警告：zeropos.yaml 中未找到 left_arm 配置节点，"
                      << "使用默认零位 0.0" << std::endl;
            lArm_zeroPos.assign(LARMDOF, 0.0);
        }

        // 读取右臂零位
        if(config["right_arm"]){
            const YAML::Node& right = config["right_arm"];
            rArm_zeroPos.clear();
            rArm_zeroPos.push_back(right["right_shoulder_pitch_joint"].as<double>(0.0));
            rArm_zeroPos.push_back(right["right_shoulder_roll_joint"].as<double>(0.0));
            rArm_zeroPos.push_back(right["right_shoulder_yaw_joint"].as<double>(0.0));
            rArm_zeroPos.push_back(right["right_elbow_joint"].as<double>(0.0));
            rArm_zeroPos.push_back(right["right_wrist_roll_joint"].as<double>(0.0));
            rArm_zeroPos.push_back(right["right_wrist_pitch_joint"].as<double>(0.0));
            rArm_zeroPos.push_back(right["right_wrist_yaw_joint"].as<double>(0.0));
        }else{
            std::cerr << "警告：zeropos.yaml 中未找到 right_arm 配置节点，"
                      << "使用默认零位 0.0" << std::endl;
            rArm_zeroPos.assign(RARMDOF, 0.0);
        }

        // 检查读取的维度是否正确
        if(lArm_zeroPos.size() != LARMDOF){
            std::cerr << "错误：左臂零位配置维度 " << lArm_zeroPos.size()
                      << " 与 LARMDOF=" << LARMDOF << " 不匹配" << std::endl;
            lArm_zeroPos.assign(LARMDOF, 0.0);
            loaded_ = false;
            return false;
        }
        if(rArm_zeroPos.size() != RARMDOF){
            std::cerr << "错误：右臂零位配置维度 " << rArm_zeroPos.size()
                      << " 与 RARMDOF=" << RARMDOF << " 不匹配" << std::endl;
            rArm_zeroPos.assign(RARMDOF, 0.0);
            loaded_ = false;
            return false;
        }

        loaded_ = true;
        std::cout << "===============零位配置加载完毕===============" << std::endl;
        printDefaultPositions();

        return true;
    }
    catch(const YAML::BadFile& e){
        std::cerr << "错误：无法打开零位配置文件 " << yaml_path_ << std::endl;
        std::cerr << "  " << e.what() << std::endl;
        loaded_ = false;
        return false;
    }
    catch(const YAML::Exception& e){
        std::cerr << "错误：解析零位配置文件失败 " << yaml_path_ << std::endl;
        std::cerr << "  " << e.what() << std::endl;
        loaded_ = false;
        return false;
    }
}

//==============================================================================
// 获取零位位置
//==============================================================================

std::vector<double> ZeroPosConfig::getLeftArmZeroPos() const{
    return lArm_zeroPos;
}

std::vector<double> ZeroPosConfig::getRightArmZeroPos() const{
    return rArm_zeroPos;
}

//==============================================================================
// 打印默认位置
//==============================================================================

void ZeroPosConfig::printDefaultPositions() const{
    // 关节名称列表（与 YAML 顺序一致）
    const std::vector<std::string> left_joint_names = {
        "left_shoulder_pitch_joint",
        "left_shoulder_roll_joint",
        "left_shoulder_yaw_joint",
        "left_elbow_joint",
        "left_wrist_roll_joint",
        "left_wrist_pitch_joint",
        "left_wrist_yaw_joint"
    };
    const std::vector<std::string> right_joint_names = {
        "right_shoulder_pitch_joint",
        "right_shoulder_roll_joint",
        "right_shoulder_yaw_joint",
        "right_elbow_joint",
        "right_wrist_roll_joint",
        "right_wrist_pitch_joint",
        "right_wrist_yaw_joint"
    };

    std::cout << "========================================" << std::endl;
    std::cout << "  机械臂默认零位配置 (rad)" << std::endl;
    std::cout << "========================================" << std::endl;

    // 左臂
    std::cout << "--- 左臂 (Left Arm) ---" << std::endl;
    for(size_t i = 0; i < lArm_zeroPos.size() && i < left_joint_names.size(); i++){
        printf("  %-32s  %+8.4f\n",
               left_joint_names[i].c_str(), lArm_zeroPos[i]);
    }

    // 右臂
    std::cout << "--- 右臂 (Right Arm) ---" << std::endl;
    for(size_t i = 0; i < rArm_zeroPos.size() && i < right_joint_names.size(); i++){
        printf("  %-32s  %+8.4f\n",
               right_joint_names[i].c_str(), rArm_zeroPos[i]);
    }

    std::cout << "========================================" << std::endl;
}

//==============================================================================
// 检查是否已成功加载
//==============================================================================

bool ZeroPosConfig::isLoaded() const{
    return loaded_;
}



//==============================================================================
// DescZeroPosConfig — 笛卡尔空间零位配置
//==============================================================================

DescZeroPosConfig::DescZeroPosConfig()
    : lArm_descPos(6, 0.0)
    , rArm_descPos(6, 0.0)
    , yaml_path_("")
    , loaded_(false)
{
}

DescZeroPosConfig::DescZeroPosConfig(const std::string& yaml_path)
    : lArm_descPos(6, 0.0)
    , rArm_descPos(6, 0.0)
    , yaml_path_(yaml_path)
    , loaded_(false)
{
    loadFromYaml(yaml_path);
}

bool DescZeroPosConfig::loadFromYaml(const std::string& yaml_path){
    yaml_path_ = yaml_path;

    try{
        YAML::Node config = YAML::LoadFile(yaml_path_);

        // 读取左臂笛卡尔零位
        if(config["left_arm"]){
            const YAML::Node& left = config["left_arm"];
            lArm_descPos.clear();
            lArm_descPos.push_back(left["position"]["x"].as<double>(0.0));
            lArm_descPos.push_back(left["position"]["y"].as<double>(0.0));
            lArm_descPos.push_back(left["position"]["z"].as<double>(0.0));
            lArm_descPos.push_back(left["orientation"]["roll"].as<double>(0.0));
            lArm_descPos.push_back(left["orientation"]["pitch"].as<double>(0.0));
            lArm_descPos.push_back(left["orientation"]["yaw"].as<double>(0.0));
        }else{
            std::cerr << "警告：desc_zeropos.yaml 中未找到 left_arm 配置节点，"
                      << "使用默认零位 [0,0,0,0,0,0]" << std::endl;
            lArm_descPos.assign(6, 0.0);
        }

        // 读取右臂笛卡尔零位
        if(config["right_arm"]){
            const YAML::Node& right = config["right_arm"];
            rArm_descPos.clear();
            rArm_descPos.push_back(right["position"]["x"].as<double>(0.0));
            rArm_descPos.push_back(right["position"]["y"].as<double>(0.0));
            rArm_descPos.push_back(right["position"]["z"].as<double>(0.0));
            rArm_descPos.push_back(right["orientation"]["roll"].as<double>(0.0));
            rArm_descPos.push_back(right["orientation"]["pitch"].as<double>(0.0));
            rArm_descPos.push_back(right["orientation"]["yaw"].as<double>(0.0));
        }else{
            std::cerr << "警告：desc_zeropos.yaml 中未找到 right_arm 配置节点，"
                      << "使用默认零位 [0,0,0,0,0,0]" << std::endl;
            rArm_descPos.assign(6, 0.0);
        }

        // 检查维度
        if(lArm_descPos.size() != 6){
            std::cerr << "错误：左臂笛卡尔零位配置维度 " << lArm_descPos.size()
                      << " 与期望维度 6 不匹配" << std::endl;
            lArm_descPos.assign(6, 0.0);
            loaded_ = false;
            return false;
        }
        if(rArm_descPos.size() != 6){
            std::cerr << "错误：右臂笛卡尔零位配置维度 " << rArm_descPos.size()
                      << " 与期望维度 6 不匹配" << std::endl;
            rArm_descPos.assign(6, 0.0);
            loaded_ = false;
            return false;
        }

        loaded_ = true;
        std::cout << "===============笛卡尔空间零位配置加载完毕===============" << std::endl;
        printDefaultPositions();

        return true;
    }
    catch(const YAML::BadFile& e){
        std::cerr << "错误：无法打开笛卡尔零位配置文件 " << yaml_path_ << std::endl;
        std::cerr << "  " << e.what() << std::endl;
        loaded_ = false;
        return false;
    }
    catch(const YAML::Exception& e){
        std::cerr << "错误：解析笛卡尔零位配置文件失败 " << yaml_path_ << std::endl;
        std::cerr << "  " << e.what() << std::endl;
        loaded_ = false;
        return false;
    }
}

std::vector<double> DescZeroPosConfig::getLeftArmDescZeroPos() const{
    return lArm_descPos;
}

std::vector<double> DescZeroPosConfig::getRightArmDescZeroPos() const{
    return rArm_descPos;
}

void DescZeroPosConfig::printDefaultPositions() const{
    const std::vector<std::string> field_names = {
        "x (m)", "y (m)", "z (m)", "roll (rad)", "pitch (rad)", "yaw (rad)"
    };

    std::cout << "========================================" << std::endl;
    std::cout << "  笛卡尔空间默认零位配置" << std::endl;
    std::cout << "========================================" << std::endl;

    // 左臂
    std::cout << "--- 左臂 (Left Arm) ---" << std::endl;
    for(size_t i = 0; i < lArm_descPos.size() && i < field_names.size(); i++){
        printf("  %-14s  %+10.6f\n",
               field_names[i].c_str(), lArm_descPos[i]);
    }

    // 右臂
    std::cout << "--- 右臂 (Right Arm) ---" << std::endl;
    for(size_t i = 0; i < rArm_descPos.size() && i < field_names.size(); i++){
        printf("  %-14s  %+10.6f\n",
               field_names[i].c_str(), rArm_descPos[i]);
    }

    std::cout << "========================================" << std::endl;
}

bool DescZeroPosConfig::isLoaded() const{
    return loaded_;
}


//==============================================================================
//  InitialPDConfig 实现
//==============================================================================

// 构造函数
InitialPDConfig::InitialPDConfig()
    : lArm_Kp(LARMDOF, 0.0)
    , lArm_Kd(LARMDOF, 0.0)
    , rArm_Kp(RARMDOF, 0.0)
    , rArm_Kd(RARMDOF, 0.0)
    , yaml_path_("")
    , rigidity_level_("")
    , loaded_(false)
{
}

InitialPDConfig::InitialPDConfig(const std::string& yaml_path,
                                 const std::string& rigidity_level)
    : lArm_Kp(LARMDOF, 0.0)
    , lArm_Kd(LARMDOF, 0.0)
    , rArm_Kp(RARMDOF, 0.0)
    , rArm_Kd(RARMDOF, 0.0)
    , yaml_path_(yaml_path)
    , rigidity_level_(rigidity_level)
    , loaded_(false)
{
    loadFromYaml(yaml_path, rigidity_level);
}

//==============================================================================
// 从 YAML 文件加载指定刚度等级的 PD 配置
//==============================================================================

bool InitialPDConfig::loadFromYaml(const std::string& yaml_path,
                                   const std::string& rigidity_level){
    yaml_path_ = yaml_path;
    rigidity_level_ = rigidity_level;

    // 构建 YAML 节点名称: left_arm_Kp_<level>, left_arm_Kd_<level>, ...
    std::string left_Kp_key  = "left_arm_Kp_"  + rigidity_level_;
    std::string left_Kd_key  = "left_arm_Kd_"  + rigidity_level_;
    std::string right_Kp_key = "right_arm_Kp_" + rigidity_level_;
    std::string right_Kd_key = "right_arm_Kd_" + rigidity_level_;

    // 关节子键名称列表（与 YAML 顺序一致）
    const std::vector<std::string> left_joint_kp_keys = {
        "left_shoulder_pitch_joint_kp",
        "left_shoulder_roll_joint_kp",
        "left_shoulder_yaw_joint_kp",
        "left_elbow_joint_kp",
        "left_wrist_roll_joint_kp",
        "left_wrist_pitch_joint_kp",
        "left_wrist_yaw_joint_kp"
    };
    const std::vector<std::string> left_joint_kd_keys = {
        "left_shoulder_pitch_joint_kd",
        "left_shoulder_roll_joint_kd",
        "left_shoulder_yaw_joint_kd",
        "left_elbow_joint_kd",
        "left_wrist_roll_joint_kd",
        "left_wrist_pitch_joint_kd",
        "left_wrist_yaw_joint_kd"
    };
    const std::vector<std::string> right_joint_kp_keys = {
        "right_shoulder_pitch_joint_kp",
        "right_shoulder_roll_joint_kp",
        "right_shoulder_yaw_joint_kp",
        "right_elbow_joint_kp",
        "right_wrist_roll_joint_kp",
        "right_wrist_pitch_joint_kp",
        "right_wrist_yaw_joint_kp"
    };
    const std::vector<std::string> right_joint_kd_keys = {
        "right_shoulder_pitch_joint_kd",
        "right_shoulder_roll_joint_kd",
        "right_shoulder_yaw_joint_kd",
        "right_elbow_joint_kd",
        "right_wrist_roll_joint_kd",
        "right_wrist_pitch_joint_kd",
        "right_wrist_yaw_joint_kd"
    };

    try{
        // 加载 YAML 文件
        YAML::Node config = YAML::LoadFile(yaml_path_);

        // ---- 读取左臂 Kp ----
        if(config[left_Kp_key]){
            const YAML::Node& node = config[left_Kp_key];
            lArm_Kp.clear();
            for(size_t i = 0; i < left_joint_kp_keys.size(); i++){
                lArm_Kp.push_back(node[left_joint_kp_keys[i]].as<double>(0.0));
            }
        }else{
            std::cerr << "警告：initialPD.yaml 中未找到 " << left_Kp_key
                      << " 配置节点，使用默认 Kp=0.0" << std::endl;
            lArm_Kp.assign(LARMDOF, 0.0);
        }

        // ---- 读取左臂 Kd ----
        if(config[left_Kd_key]){
            const YAML::Node& node = config[left_Kd_key];
            lArm_Kd.clear();
            for(size_t i = 0; i < left_joint_kd_keys.size(); i++){
                lArm_Kd.push_back(node[left_joint_kd_keys[i]].as<double>(0.0));
            }
        }else{
            std::cerr << "警告：initialPD.yaml 中未找到 " << left_Kd_key
                      << " 配置节点，使用默认 Kd=0.0" << std::endl;
            lArm_Kd.assign(LARMDOF, 0.0);
        }

        // ---- 读取右臂 Kp ----
        if(config[right_Kp_key]){
            const YAML::Node& node = config[right_Kp_key];
            rArm_Kp.clear();
            for(size_t i = 0; i < right_joint_kp_keys.size(); i++){
                rArm_Kp.push_back(node[right_joint_kp_keys[i]].as<double>(0.0));
            }
        }else{
            std::cerr << "警告：initialPD.yaml 中未找到 " << right_Kp_key
                      << " 配置节点，使用默认 Kp=0.0" << std::endl;
            rArm_Kp.assign(RARMDOF, 0.0);
        }

        // ---- 读取右臂 Kd ----
        if(config[right_Kd_key]){
            const YAML::Node& node = config[right_Kd_key];
            rArm_Kd.clear();
            for(size_t i = 0; i < right_joint_kd_keys.size(); i++){
                rArm_Kd.push_back(node[right_joint_kd_keys[i]].as<double>(0.0));
            }
        }else{
            std::cerr << "警告：initialPD.yaml 中未找到 " << right_Kd_key
                      << " 配置节点，使用默认 Kd=0.0" << std::endl;
            rArm_Kd.assign(RARMDOF, 0.0);
        }

        // 检查读取的维度是否正确
        if(lArm_Kp.size() != LARMDOF || lArm_Kd.size() != LARMDOF){
            std::cerr << "错误：左臂 PD 配置维度不匹配 (Kp=" << lArm_Kp.size()
                      << ", Kd=" << lArm_Kd.size()
                      << ")，期望 " << LARMDOF << std::endl;
            lArm_Kp.assign(LARMDOF, 0.0);
            lArm_Kd.assign(LARMDOF, 0.0);
            loaded_ = false;
            return false;
        }
        if(rArm_Kp.size() != RARMDOF || rArm_Kd.size() != RARMDOF){
            std::cerr << "错误：右臂 PD 配置维度不匹配 (Kp=" << rArm_Kp.size()
                      << ", Kd=" << rArm_Kd.size()
                      << ")，期望 " << RARMDOF << std::endl;
            rArm_Kp.assign(RARMDOF, 0.0);
            rArm_Kd.assign(RARMDOF, 0.0);
            loaded_ = false;
            return false;
        }

        loaded_ = true;
        std::cout << "===============PD 配置加载完毕===============" << std::endl;
        std::cout << "  刚度等级: " << rigidity_level_ << std::endl;
        printPDConfig();

        return true;
    }
    catch(const YAML::BadFile& e){
        std::cerr << "错误：无法打开 PD 配置文件 " << yaml_path_ << std::endl;
        std::cerr << "  " << e.what() << std::endl;
        loaded_ = false;
        return false;
    }
    catch(const YAML::Exception& e){
        std::cerr << "错误：解析 PD 配置文件失败 " << yaml_path_ << std::endl;
        std::cerr << "  " << e.what() << std::endl;
        loaded_ = false;
        return false;
    }
}

//==============================================================================
// 获取 PD 参数
//==============================================================================

std::vector<double> InitialPDConfig::getLeftArmKp() const{
    return lArm_Kp;
}

std::vector<double> InitialPDConfig::getLeftArmKd() const{
    return lArm_Kd;
}

std::vector<double> InitialPDConfig::getRightArmKp() const{
    return rArm_Kp;
}

std::vector<double> InitialPDConfig::getRightArmKd() const{
    return rArm_Kd;
}

//==============================================================================
// 获取当前刚度等级
//==============================================================================

std::string InitialPDConfig::getRigidityLevel() const{
    return rigidity_level_;
}

//==============================================================================
// 打印当前 PD 配置
//==============================================================================

void InitialPDConfig::printPDConfig() const{
    // 关节名称列表（与 YAML 顺序一致）
    const std::vector<std::string> left_joint_names = {
        "left_shoulder_pitch_joint",
        "left_shoulder_roll_joint",
        "left_shoulder_yaw_joint",
        "left_elbow_joint",
        "left_wrist_roll_joint",
        "left_wrist_pitch_joint",
        "left_wrist_yaw_joint"
    };
    const std::vector<std::string> right_joint_names = {
        "right_shoulder_pitch_joint",
        "right_shoulder_roll_joint",
        "right_shoulder_yaw_joint",
        "right_elbow_joint",
        "right_wrist_roll_joint",
        "right_wrist_pitch_joint",
        "right_wrist_yaw_joint"
    };

    std::cout << "========================================" << std::endl;
    std::cout << "  机械臂 PD 增益配置  [" << rigidity_level_ << "]"
              << std::endl;
    std::cout << "========================================" << std::endl;

    // 左臂
    std::cout << "--- 左臂 (Left Arm) ---" << std::endl;
    std::cout << "  " << std::left << std::setw(32) << "关节名称"
              << std::right << std::setw(10) << "Kp"
              << std::setw(10) << "Kd" << std::endl;
    std::cout << "  " << std::string(52, '-') << std::endl;
    for(size_t i = 0; i < lArm_Kp.size() && i < left_joint_names.size(); i++){
        printf("  %-32s  %8.0f  %8.0f\n",
               left_joint_names[i].c_str(), lArm_Kp[i], lArm_Kd[i]);
    }

    // 右臂
    std::cout << "--- 右臂 (Right Arm) ---" << std::endl;
    std::cout << "  " << std::left << std::setw(32) << "关节名称"
              << std::right << std::setw(10) << "Kp"
              << std::setw(10) << "Kd" << std::endl;
    std::cout << "  " << std::string(52, '-') << std::endl;
    for(size_t i = 0; i < rArm_Kp.size() && i < right_joint_names.size(); i++){
        printf("  %-32s  %8.0f  %8.0f\n",
               right_joint_names[i].c_str(), rArm_Kp[i], rArm_Kd[i]);
    }

    std::cout << "========================================" << std::endl;
}

//==============================================================================
// 检查是否已成功加载
//==============================================================================

bool InitialPDConfig::isLoaded() const{
    return loaded_;
}


//==============================================================================
//  JointLinkConfig — 关节连杆结构配置
//==============================================================================

// 构造函数
JointLinkConfig::JointLinkConfig()
    : lArm_frame_names()
    , lArm_JntRotation_axis()
    , lArm_jointNum(0)
    , rArm_frame_names()
    , rArm_JntRotation_axis()
    , rArm_jointNum(0)
    , yaml_path_("")
    , loaded_(false)
{
}

JointLinkConfig::JointLinkConfig(const std::string& yaml_path)
    : lArm_frame_names()
    , lArm_JntRotation_axis()
    , lArm_jointNum(0)
    , rArm_frame_names()
    , rArm_JntRotation_axis()
    , rArm_jointNum(0)
    , yaml_path_(yaml_path)
    , loaded_(false)
{
    loadFromYaml(yaml_path);
}

//==============================================================================
// 从 YAML 文件加载关节连杆配置
//==============================================================================

bool JointLinkConfig::loadFromYaml(const std::string& yaml_path){
    yaml_path_ = yaml_path;

    try{
        YAML::Node config = YAML::LoadFile(yaml_path_);

        // ---- 读取左臂关节连杆结构 ----
        if(config["lArm"]){
            const YAML::Node& lArm = config["lArm"];

            // 左臂连杆名称列表
            lArm_frame_names.clear();
            if(lArm["frame_names"]){
                for(size_t i = 0; i < lArm["frame_names"].size(); i++){
                    lArm_frame_names.push_back(lArm["frame_names"][i].as<std::string>());
                }
            }

            // 左臂关节旋转轴列表
            lArm_JntRotation_axis.clear();
            if(lArm["jnt_rotation_axis"]){
                for(size_t i = 0; i < lArm["jnt_rotation_axis"].size(); i++){
                    lArm_JntRotation_axis.push_back(lArm["jnt_rotation_axis"][i].as<std::string>());
                }
            }

            // 左臂关节数目
            lArm_jointNum = lArm["joint_num"].as<int>(0);

            // ---- 维度一致性校验 ----
            if(static_cast<int>(lArm_frame_names.size()) != lArm_jointNum){
                std::cerr << "错误：左臂 frame_names 数量 " << lArm_frame_names.size()
                          << " 与 joint_num=" << lArm_jointNum << " 不匹配" << std::endl;
                loaded_ = false;
                return false;
            }
            if(static_cast<int>(lArm_JntRotation_axis.size()) != lArm_jointNum){
                std::cerr << "错误：左臂 jnt_rotation_axis 数量 " << lArm_JntRotation_axis.size()
                          << " 与 joint_num=" << lArm_jointNum << " 不匹配" << std::endl;
                loaded_ = false;
                return false;
            }
        }else{
            std::cerr << "警告：jointLink_config.yaml 中未找到 lArm 配置节点" << std::endl;
            loaded_ = false;
            return false;
        }

        // ---- 读取右臂关节连杆结构 ----
        if(config["rArm"]){
            const YAML::Node& rArm = config["rArm"];

            // 右臂连杆名称列表
            rArm_frame_names.clear();
            if(rArm["frame_names"]){
                for(size_t i = 0; i < rArm["frame_names"].size(); i++){
                    rArm_frame_names.push_back(rArm["frame_names"][i].as<std::string>());
                }
            }

            // 右臂关节旋转轴列表
            rArm_JntRotation_axis.clear();
            if(rArm["jnt_rotation_axis"]){
                for(size_t i = 0; i < rArm["jnt_rotation_axis"].size(); i++){
                    rArm_JntRotation_axis.push_back(rArm["jnt_rotation_axis"][i].as<std::string>());
                }
            }

            // 右臂关节数目
            rArm_jointNum = rArm["joint_num"].as<int>(0);

            // ---- 维度一致性校验 ----
            if(static_cast<int>(rArm_frame_names.size()) != rArm_jointNum){
                std::cerr << "错误：右臂 frame_names 数量 " << rArm_frame_names.size()
                          << " 与 joint_num=" << rArm_jointNum << " 不匹配" << std::endl;
                loaded_ = false;
                return false;
            }
            if(static_cast<int>(rArm_JntRotation_axis.size()) != rArm_jointNum){
                std::cerr << "错误：右臂 jnt_rotation_axis 数量 " << rArm_JntRotation_axis.size()
                          << " 与 joint_num=" << rArm_jointNum << " 不匹配" << std::endl;
                loaded_ = false;
                return false;
            }
        }else{
            std::cerr << "警告：jointLink_config.yaml 中未找到 rArm 配置节点" << std::endl;
            loaded_ = false;
            return false;
        }

        loaded_ = true;
        std::cout << "===============关节连杆结构配置加载完毕===============" << std::endl;
        printJointLinkConfig();

        return true;
    }
    catch(const YAML::BadFile& e){
        std::cerr << "错误：无法打开关节连杆配置文件 " << yaml_path_ << std::endl;
        std::cerr << "  " << e.what() << std::endl;
        loaded_ = false;
        return false;
    }
    catch(const YAML::Exception& e){
        std::cerr << "错误：解析关节连杆配置文件失败 " << yaml_path_ << std::endl;
        std::cerr << "  " << e.what() << std::endl;
        loaded_ = false;
        return false;
    }
}

//==============================================================================
// 获取关节连杆结构数据
//==============================================================================

std::vector<std::string> JointLinkConfig::getLeftArmFrameNames() const{
    return lArm_frame_names;
}

std::vector<std::string> JointLinkConfig::getLeftArmRotationAxis() const{
    return lArm_JntRotation_axis;
}

int JointLinkConfig::getLeftArmJointNum() const{
    return lArm_jointNum;
}

std::vector<std::string> JointLinkConfig::getRightArmFrameNames() const{
    return rArm_frame_names;
}

std::vector<std::string> JointLinkConfig::getRightArmRotationAxis() const{
    return rArm_JntRotation_axis;
}

int JointLinkConfig::getRightArmJointNum() const{
    return rArm_jointNum;
}

//==============================================================================
// 打印关节连杆结构
//==============================================================================

void JointLinkConfig::printJointLinkConfig() const{
    std::cout << "========================================" << std::endl;
    std::cout << "  关节连杆结构配置" << std::endl;
    std::cout << "========================================" << std::endl;

    // 左臂
    std::cout << "--- 左臂 (Left Arm)  joint_num = " << lArm_jointNum << " ---" << std::endl;
    std::cout << "  " << std::left << std::setw(16) << "frame_name"
              << std::setw(16) << "rotation_axis" << std::endl;
    std::cout << "  " << std::string(32, '-') << std::endl;
    for(size_t i = 0; i < lArm_frame_names.size() && i < lArm_JntRotation_axis.size(); i++){
        std::cout << "  " << std::left << std::setw(16) << lArm_frame_names[i]
                  << std::setw(16) << lArm_JntRotation_axis[i] << std::endl;
    }

    // 右臂
    std::cout << "--- 右臂 (Right Arm)  joint_num = " << rArm_jointNum << " ---" << std::endl;
    std::cout << "  " << std::left << std::setw(16) << "frame_name"
              << std::setw(16) << "rotation_axis" << std::endl;
    std::cout << "  " << std::string(32, '-') << std::endl;
    for(size_t i = 0; i < rArm_frame_names.size() && i < rArm_JntRotation_axis.size(); i++){
        std::cout << "  " << std::left << std::setw(16) << rArm_frame_names[i]
                  << std::setw(16) << rArm_JntRotation_axis[i] << std::endl;
    }

    std::cout << "========================================" << std::endl;
}

//==============================================================================
// 检查是否已成功加载
//==============================================================================

bool JointLinkConfig::isLoaded() const{
    return loaded_;
}


//==============================================================================
// 机器人配置文件读取
//==============================================================================

void loadRobotCofig::yamlRead(std::string yaml_filePath){
    try{
        YAML::Node config = YAML::LoadFile(yaml_filePath);

        // ===== 1. 读取腰部（基座）配置 =====
        if(config["waist"]){
            const YAML::Node& waist = config["waist"];

            // 基座名称
            if(waist["arm_base_name"]){
                robotStructConfig.base_name = waist["arm_base_name"].as<std::string>();
            }else{
                std::cerr << "警告：robot_config.yaml 中未找到 waist.arm_base_name" << std::endl;
                robotStructConfig.base_name = "arm_base_link";
            }

            // 基座相对于世界坐标系的位姿 (可选字段)
            if(waist["base_to_world"] && waist["base_to_world"].IsSequence()){
                robotStructConfig.base_to_world.clear();
                for(const auto& v : waist["base_to_world"]){
                    robotStructConfig.base_to_world.push_back(v.as<double>());
                }
            }else{
                // 默认：原点无偏移
                robotStructConfig.base_to_world = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
            }
        }else{
            std::cerr << "警告：robot_config.yaml 中未找到 waist 配置节点" << std::endl;
            robotStructConfig.base_name = "arm_base_link";
            robotStructConfig.base_to_world = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        }

        // ===== 2. 读取左臂配置 =====
        if(config["lArm"]){
            const YAML::Node& lArm = config["lArm"];

            // 关节数目
            robotStructConfig.lArm_JntNum = lArm["joint_num"].as<int>(0);

            // 关节名称列表
            robotStructConfig.lArm_joint_names.clear();
            if(lArm["joint_names"] && lArm["joint_names"].IsSequence()){
                for(const auto& name : lArm["joint_names"]){
                    robotStructConfig.lArm_joint_names.push_back(name.as<std::string>());
                }
            }

            // 关节限制列表 [lower, upper, velocity, acceleration, torque]
            robotStructConfig.lArm_JntLimits.clear();
            if(lArm["jnt_limit"] && lArm["jnt_limit"].IsSequence()){
                for(const auto& limit_node : lArm["jnt_limit"]){
                    JntLimit lim;
                    lim.lower        = limit_node[0].as<double>(0.0);
                    lim.upper        = limit_node[1].as<double>(0.0);
                    lim.velocity     = limit_node[2].as<double>(0.0);
                    lim.acceleration = limit_node[3].as<double>(0.0);
                    lim.torque       = limit_node[4].as<double>(0.0);
                    robotStructConfig.lArm_JntLimits.push_back(lim);
                }
            }

            // 关节电流-力矩常数 Kt
            robotStructConfig.lArm_jnt_Kt.clear();
            if(lArm["jnt_Kt"] && lArm["jnt_Kt"].IsSequence()){
                for(const auto& kt_node : lArm["jnt_Kt"]){
                    for(const auto& val : kt_node){
                        robotStructConfig.lArm_jnt_Kt.push_back(val.as<double>(1.0));
                    }
                }
            }

            // 工具基座
            if(lArm["tool_base"]){
                robotStructConfig.lArm_tool_base = lArm["tool_base"].as<std::string>();
            }else{
                robotStructConfig.lArm_tool_base = "";
            }

            // 工具链
            robotStructConfig.lArm_tool_chain.clear();
            if(lArm["tool_chain"] && lArm["tool_chain"].IsSequence()){
                for(const auto& link : lArm["tool_chain"]){
                    robotStructConfig.lArm_tool_chain.push_back(link.as<std::string>());
                }
            }
        }else{
            std::cerr << "错误：robot_config.yaml 中未找到 lArm 配置节点" << std::endl;
            return;
        }

        // ===== 3. 读取右臂配置 =====
        if(config["rArm"]){
            const YAML::Node& rArm = config["rArm"];

            // 关节数目
            robotStructConfig.rArm_JntNum = rArm["joint_num"].as<int>(0);

            // 关节名称列表
            robotStructConfig.rArm_joint_names.clear();
            if(rArm["joint_names"] && rArm["joint_names"].IsSequence()){
                for(const auto& name : rArm["joint_names"]){
                    robotStructConfig.rArm_joint_names.push_back(name.as<std::string>());
                }
            }

            // 关节限制列表 [lower, upper, velocity, acceleration, torque]
            robotStructConfig.rArm_JntLimits.clear();
            if(rArm["jnt_limit"] && rArm["jnt_limit"].IsSequence()){
                for(const auto& limit_node : rArm["jnt_limit"]){
                    JntLimit lim;
                    lim.lower        = limit_node[0].as<double>(0.0);
                    lim.upper        = limit_node[1].as<double>(0.0);
                    lim.velocity     = limit_node[2].as<double>(0.0);
                    lim.acceleration = limit_node[3].as<double>(0.0);
                    lim.torque       = limit_node[4].as<double>(0.0);
                    robotStructConfig.rArm_JntLimits.push_back(lim);
                }
            }

            // 关节电流-力矩常数 Kt
            robotStructConfig.rArm_jnt_Kt.clear();
            if(rArm["jnt_Kt"] && rArm["jnt_Kt"].IsSequence()){
                for(const auto& kt_node : rArm["jnt_Kt"]){
                    for(const auto& val : kt_node){
                        robotStructConfig.rArm_jnt_Kt.push_back(val.as<double>(1.0));
                    }
                }
            }

            // 工具基座
            if(rArm["tool_base"]){
                robotStructConfig.rArm_tool_base = rArm["tool_base"].as<std::string>();
            }else{
                robotStructConfig.rArm_tool_base = "";
            }

            // 工具链
            robotStructConfig.rArm_tool_chain.clear();
            if(rArm["tool_chain"] && rArm["tool_chain"].IsSequence()){
                for(const auto& link : rArm["tool_chain"]){
                    robotStructConfig.rArm_tool_chain.push_back(link.as<std::string>());
                }
            }
        }else{
            std::cerr << "错误：robot_config.yaml 中未找到 rArm 配置节点" << std::endl;
            return;
        }

        // ===== 4. 维度一致性校验 =====
        if(static_cast<int>(robotStructConfig.lArm_joint_names.size()) != robotStructConfig.lArm_JntNum){
            std::cerr << "错误：左臂 joint_names 数量 " << robotStructConfig.lArm_joint_names.size()
                      << " 与 joint_num=" << robotStructConfig.lArm_JntNum << " 不匹配" << std::endl;
        }
        if(static_cast<int>(robotStructConfig.lArm_JntLimits.size()) != robotStructConfig.lArm_JntNum){
            std::cerr << "错误：左臂 jnt_limit 数量 " << robotStructConfig.lArm_JntLimits.size()
                      << " 与 joint_num=" << robotStructConfig.lArm_JntNum << " 不匹配" << std::endl;
        }

        if(static_cast<int>(robotStructConfig.rArm_joint_names.size()) != robotStructConfig.rArm_JntNum){
            std::cerr << "错误：右臂 joint_names 数量 " << robotStructConfig.rArm_joint_names.size()
                      << " 与 joint_num=" << robotStructConfig.rArm_JntNum << " 不匹配" << std::endl;
        }
        if(static_cast<int>(robotStructConfig.rArm_JntLimits.size()) != robotStructConfig.rArm_JntNum){
            std::cerr << "错误：右臂 jnt_limit 数量 " << robotStructConfig.rArm_JntLimits.size()
                      << " 与 joint_num=" << robotStructConfig.rArm_JntNum << " 不匹配" << std::endl;
        }
        if(static_cast<int>(robotStructConfig.lArm_jnt_Kt.size()) != robotStructConfig.lArm_JntNum){
            std::cerr << "错误：左臂 jnt_Kt 数量 " << robotStructConfig.lArm_jnt_Kt.size()
                      << " 与 joint_num=" << robotStructConfig.lArm_JntNum << " 不匹配" << std::endl;
        }
        if(static_cast<int>(robotStructConfig.rArm_jnt_Kt.size()) != robotStructConfig.rArm_JntNum){
            std::cerr << "错误：右臂 jnt_Kt 数量 " << robotStructConfig.rArm_jnt_Kt.size()
                      << " 与 joint_num=" << robotStructConfig.rArm_JntNum << " 不匹配" << std::endl;
        }

        std::cout << "===============机器人结构配置加载完毕===============" << std::endl;
        std::cout << "  base_name: " << robotStructConfig.base_name << std::endl;
        std::cout << "  左臂关节数: " << robotStructConfig.lArm_JntNum
                  << "  右臂关节数: " << robotStructConfig.rArm_JntNum << std::endl;
        // 打印关节限制摘要
        std::cout << "  左臂关节限制 (lower, upper, vel, acc, tor):" << std::endl;
        for(size_t i = 0; i < robotStructConfig.lArm_JntLimits.size(); i++){
            const auto& lim = robotStructConfig.lArm_JntLimits[i];
            printf("    J%zu: [%+7.2f, %+7.2f, %5.2f, %5.2f, %6.1f]\n",
                   i+1, lim.lower, lim.upper, lim.velocity, lim.acceleration, lim.torque);
        }
        std::cout << "  右臂关节限制 (lower, upper, vel, acc, tor):" << std::endl;
        for(size_t i = 0; i < robotStructConfig.rArm_JntLimits.size(); i++){
            const auto& lim = robotStructConfig.rArm_JntLimits[i];
            printf("    J%zu: [%+7.2f, %+7.2f, %5.2f, %5.2f, %6.1f]\n",
                   i+1, lim.lower, lim.upper, lim.velocity, lim.acceleration, lim.torque);
        }
        // 打印电流-力矩常数
        std::cout << "  左臂 Kt:" << std::endl << "    ";
        for(size_t i = 0; i < robotStructConfig.lArm_jnt_Kt.size(); i++){
            printf("[%.1f] ", robotStructConfig.lArm_jnt_Kt[i]);
        }
        std::cout << std::endl;
        std::cout << "  右臂 Kt:" << std::endl << "    ";
        for(size_t i = 0; i < robotStructConfig.rArm_jnt_Kt.size(); i++){
            printf("[%.1f] ", robotStructConfig.rArm_jnt_Kt[i]);
        }
        std::cout << std::endl;

    }
    catch(const YAML::BadFile& e){
        std::cerr << "错误：无法打开机器人配置文件 " << yaml_filePath << std::endl;
        std::cerr << "  " << e.what() << std::endl;
    }
    catch(const YAML::Exception& e){
        std::cerr << "错误：解析机器人配置文件失败 " << yaml_filePath << std::endl;
        std::cerr << "  " << e.what() << std::endl;
    }
}

void loadRobotCofig::getModelFromYaml(std::string yaml_filePath, std::string urdf_filepath){
    // ===== 1. 读取.yaml文件到配置结构体中 =====
    yamlRead(yaml_filePath);

    // ===== 2. 读取.urdf并创建原始的pinocchio机器人模型 =====
    // 每个机械臂运动链为：基座 -> 关节1 -> 关节2 -> ... -> 关节n -> 工具基座

    std::ifstream f(urdf_filepath);
    if(!f.is_open()){
        std::cerr << "错误：无法打开 URDF 文件 " << urdf_filepath << std::endl;
        return;
    }
    std::stringstream buf;
    buf << f.rdbuf();
    std::string urdf_xml = buf.str();

    // ===== 3. 提取基座到世界坐标系的转换矩阵 =====
    // 按照[x, y, z, roll, pitch, yaw]格式转换为 pinocchio::SE3
    auto& bw = robotStructConfig.base_to_world;
    if(bw.size() < 6){
        std::cerr << "警告：base_to_world 维度不足，使用默认零位姿" << std::endl;
        bw.resize(6, 0.0);
    }

    // R = Rz(yaw) * Ry(pitch) * Rx(roll)
    Eigen::Quaterniond base_quat(
        Eigen::AngleAxisd(bw[5], Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(bw[4], Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(bw[3], Eigen::Vector3d::UnitX())
    );
    Eigen::Vector3d base_trans(bw[0], bw[1], bw[2]);
    pinocchio::SE3 basePlacement(base_quat.toRotationMatrix(), base_trans);

    // ===== 4. 添加六维自由飞行关节（浮动基座）到pinocchio模型中 =====
    // 使得模型能够处理基座在世界坐标系中的六自由度运动
    pinocchio::JointModelFreeFlyer root_joint;
    whole_model_ = pinocchio::Model();

    pinocchio::urdf::buildModelFromXML(
        urdf_xml,
        root_joint,
        "floating_base_joint",
        whole_model_
    );
    // 模型结构：universe -> floating_base_joint(FreeFlyer, 6DOF) -> arm_base_link -> ...

    // 将 base_to_world 写入 FreeFlyer 关节的初始位姿
    // FreeFlyer 为模型中的第1号关节（universe 为第0号）
    whole_model_.jointPlacements[1] = basePlacement;

    // ===== 5. 将工具链添加到模型中 =====
    // 工具链中所有对象通过 FIXED_JOINT 帧串联，固连在一起
    // Pinocchio 中 FIXED_JOINT 不是单独的关节模型，而是通过 addFrame() 添加的帧类型

    // ---- 左臂工具链 ----
    if(!robotStructConfig.lArm_tool_chain.empty() && !robotStructConfig.lArm_tool_base.empty()){
        if(!whole_model_.existJointName(robotStructConfig.lArm_tool_base)){
            std::cerr << "警告：左臂工具基座关节 \"" << robotStructConfig.lArm_tool_base
                      << "\" 在模型中不存在，跳过工具链添加" << std::endl;
        }else{
            pinocchio::JointIndex parent_joint =
                whole_model_.getJointId(robotStructConfig.lArm_tool_base);
            int parent_frame = -1;  // 初始挂载在父关节自身帧上

            for(size_t i = 0; i < robotStructConfig.lArm_tool_chain.size(); i++){
                const std::string& link_name = robotStructConfig.lArm_tool_chain[i];

                pinocchio::Frame f(
                    link_name,
                    parent_joint,
                    parent_frame,
                    pinocchio::SE3::Identity(),
                    pinocchio::FIXED_JOINT
                );
                pinocchio::FrameIndex new_frame = whole_model_.addFrame(f, true);
                parent_frame = static_cast<int>(new_frame);  // 下一级链到此帧上
            }
            std::cout << "左臂工具链已添加，" << robotStructConfig.lArm_tool_chain.size()
                      << " 个固连连杆" << std::endl;
        }
    }

    // ---- 右臂工具链 ----
    if(!robotStructConfig.rArm_tool_chain.empty() && !robotStructConfig.rArm_tool_base.empty()){
        if(!whole_model_.existJointName(robotStructConfig.rArm_tool_base)){
            std::cerr << "警告：右臂工具基座关节 \"" << robotStructConfig.rArm_tool_base
                      << "\" 在模型中不存在，跳过工具链添加" << std::endl;
        }else{
            pinocchio::JointIndex parent_joint =
                whole_model_.getJointId(robotStructConfig.rArm_tool_base);
            int parent_frame = -1;

            for(size_t i = 0; i < robotStructConfig.rArm_tool_chain.size(); i++){
                const std::string& link_name = robotStructConfig.rArm_tool_chain[i];

                pinocchio::Frame f(
                    link_name,
                    parent_joint,
                    parent_frame,
                    pinocchio::SE3::Identity(),
                    pinocchio::FIXED_JOINT
                );
                pinocchio::FrameIndex new_frame = whole_model_.addFrame(f, true);
                parent_frame = static_cast<int>(new_frame);
            }
            std::cout << "右臂工具链已添加，" << robotStructConfig.rArm_tool_chain.size()
                      << " 个固连连杆" << std::endl;
        }
    }

    // ===== 6. 保存处理后的模型 =====
    processed_model = whole_model_;

    std::cout << "===============机器人模型构建完毕===============" << std::endl;
    std::cout << "  URDF 文件: " << urdf_filepath << std::endl;
    std::cout << "  基座名称: " << robotStructConfig.base_name << std::endl;
    std::cout << "  基座位姿: [" << bw[0] << ", " << bw[1] << ", " << bw[2]
              << ", " << bw[3] << ", " << bw[4] << ", " << bw[5] << "]" << std::endl;
    std::cout << "  关节总数: " << whole_model_.njoints << std::endl;
    std::cout << "  配置维度 (nq): " << whole_model_.nq
              << "  速度维度 (nv): " << whole_model_.nv << std::endl;
}

//==============================================================================
//  GravCompConfig — 重力补偿配置读取
//==============================================================================

// 辅助函数：从 YAML node 读取双臂滤波器 (7个 lowpassFtrInte)
std::vector<lowpassFtrInte> GravCompConfig::readFilterGroup(
    const YAML::Node& node,
    const std::string& sampRate_key,
    const std::string& cutoffRate_key)
{
    std::vector<lowpassFtrInte> result;
    if(node && node[sampRate_key] && node[cutoffRate_key]){
        for(size_t i = 0; i < node[sampRate_key].size(); i++){
            lowpassFtrInte ftr;
            ftr.sampRate   = node[sampRate_key][i].as<double>(10000.0);
            ftr.cutoffRate = node[cutoffRate_key][i].as<double>(20.0);
            result.push_back(ftr);
        }
    } else {
        // 默认值
        for(int i = 0; i < LARMDOF; i++){
            lowpassFtrInte ftr;
            ftr.sampRate   = 10000.0;
            ftr.cutoffRate = 20.0;
            result.push_back(ftr);
        }
    }
    return result;
}

// 读取加速度卡尔曼滤波器 (14维 → 存为14个 kalmanFtrInte)
void GravCompConfig::readAccFtr(const YAML::Node& accFtr_node){
    mGravCompData.acc_ftr.clear();
    if(accFtr_node && accFtr_node["Qgain"] && accFtr_node["Rgain"]){
        for(size_t i = 0; i < accFtr_node["Qgain"].size() && i < static_cast<size_t>(ALLJNTNUM); i++){
            kalmanFtrInte ftr;
            ftr.Q_gain = accFtr_node["Qgain"][i].as<double>(12000.0);
            ftr.R_gain = accFtr_node["Rgain"][i].as<double>(200.0);
            mGravCompData.acc_ftr.push_back(ftr);
        }
    } else {
        // 默认值
        for(int i = 0; i < ALLJNTNUM; i++){
            kalmanFtrInte ftr;
            ftr.Q_gain = 12000.0;
            ftr.R_gain = 200.0;
            mGravCompData.acc_ftr.push_back(ftr);
        }
    }
}

// 读取 gravCompFtr 总节点
void GravCompConfig::readGravCompFtr(const YAML::Node& ftr_root){
    auto& ftr = mGravCompData.gravComp_InpOutpFtr;

    // ---- 位置输入滤波器 ----
    if(ftr_root["posInpFtr"]){
        const auto& pos = ftr_root["posInpFtr"];
        ftr.lArm_posInpFtr = readFilterGroup(pos["lArm"], "sampRate", "cutoffRate");
        ftr.rArm_posInpFtr = readFilterGroup(pos["rArm"], "sampRate", "cutoffRate");
    }
    // ---- 速度输入滤波器 ----
    if(ftr_root["velInpFtr"]){
        const auto& vel = ftr_root["velInpFtr"];
        ftr.lArm_velInpFtr = readFilterGroup(vel["lArm"], "sampRate", "cutoffRate");
        ftr.rArm_velInpFtr = readFilterGroup(vel["rArm"], "sampRate", "cutoffRate");
    }
    // ---- 力矩输入滤波器 ----
    if(ftr_root["torInpFtr"]){
        const auto& tor = ftr_root["torInpFtr"];
        ftr.lArm_torInpFtr = readFilterGroup(tor["lArm"], "sampRate", "cutoffRate");
        ftr.rArm_torInpFtr = readFilterGroup(tor["rArm"], "sampRate", "cutoffRate");
    }
    // ---- 力矩输出滤波器 ----
    if(ftr_root["torOutpFtr"]){
        const auto& tout = ftr_root["torOutpFtr"];
        ftr.lArm_torOutpFtr = readFilterGroup(tout["lArm"], "sampRate", "cutoffRate");
        ftr.rArm_torOutpFtr = readFilterGroup(tout["rArm"], "sampRate", "cutoffRate");
    }
    // ---- 关节角度输出滤波器 ----
    if(ftr_root["posOutpFtr"]){
        const auto& pout = ftr_root["posOutpFtr"];
        ftr.lArm_posOutpFtr = readFilterGroup(pout["lArm"], "sampRate", "cutoffRate");
        ftr.rArm_posOutpFtr = readFilterGroup(pout["rArm"], "sampRate", "cutoffRate");
    }
    // ---- 关节角速度输出滤波器 ----
    if(ftr_root["velOutpFtr"]){
        const auto& vout = ftr_root["velOutpFtr"];
        ftr.lArm_velOutpFtr = readFilterGroup(vout["lArm"], "sampRate", "cutoffRate");
        ftr.rArm_velOutpFtr = readFilterGroup(vout["rArm"], "sampRate", "cutoffRate");
    }
}

// 构造函数
GravCompConfig::GravCompConfig()
    : yaml_path_("")
    , loaded_(false)
{
    mGravCompData.ifUseGravComp = 1;
    mGravCompData.ifUseInpFtr   = 1;
    mGravCompData.ifUseOutpFtr  = 1;
    mGravCompData.ifUseFricComp = 1;
    mGravCompData.ifUsePosVelOutpFtr = 1;
    mGravCompData.InpDataMod    = 1;
    mGravCompData.GravCompStZeroTimes   = 10;
    mGravCompData.GravCompStSmoothTimes = 75;
    mGravCompData.JntCtlStStabTimes     = 100;
}

GravCompConfig::GravCompConfig(const std::string& yaml_path)
    : yaml_path_(yaml_path)
    , loaded_(false)
{
    mGravCompData.ifUseGravComp = 1;
    mGravCompData.ifUseInpFtr   = 1;
    mGravCompData.ifUseOutpFtr  = 1;
    mGravCompData.ifUseFricComp = 1;
    mGravCompData.ifUsePosVelOutpFtr = 1;
    mGravCompData.InpDataMod    = 1;
    mGravCompData.GravCompStZeroTimes   = 10;
    mGravCompData.GravCompStSmoothTimes = 75;
    mGravCompData.JntCtlStStabTimes     = 100;
    loadFromYaml(yaml_path);
}

bool GravCompConfig::loadFromYaml(const std::string& yaml_path){
    yaml_path_ = yaml_path;

    try{
        YAML::Node config = YAML::LoadFile(yaml_path_);

        // ===== 1. 读取 gravComp 节点 =====
        if(config["gravComp"]){
            const auto& gc = config["gravComp"];

            // 开关量
            mGravCompData.ifUseGravComp = gc["ifUseGravComp"].as<int>(1);
            mGravCompData.ifUseInpFtr   = gc["ifUseInpFtr"].as<int>(1);
            mGravCompData.ifUseOutpFtr  = gc["ifUseOutpFtr"].as<int>(1);
            mGravCompData.ifUseFricComp = gc["ifUseFricComp"].as<int>(1);
            mGravCompData.ifUsePosVelOutpFtr = gc["ifUsePosVelOutpFtr"].as<int>(1);

            // 数据输入模式
            mGravCompData.InpDataMod = gc["InpDataMode"].as<int>(1);

            // 初始化平滑输出控制变量
            mGravCompData.GravCompStZeroTimes   = gc["GravCompStZeroTimes"].as<int>(10);
            mGravCompData.GravCompStSmoothTimes = gc["GravCompStSmoothTimes"].as<int>(75);
            mGravCompData.JntCtlStStabTimes     = gc["JntCtlStStabTimes"].as<int>(100);

            // 补偿修正系数 (14维)
            mGravCompData.gravCompInte.clear();
            if(gc["GravCompInte"] && gc["GravCompInte"].IsSequence()){
                for(const auto& v : gc["GravCompInte"]){
                    mGravCompData.gravCompInte.push_back(v.as<double>(1.0));
                }
            }
            mGravCompData.coriousCompInte.clear();
            if(gc["CoriousCompInte"] && gc["CoriousCompInte"].IsSequence()){
                for(const auto& v : gc["CoriousCompInte"]){
                    mGravCompData.coriousCompInte.push_back(v.as<double>(1.0));
                }
            }
            mGravCompData.InteriaCompInte.clear();
            if(gc["InteriaCompInte"] && gc["InteriaCompInte"].IsSequence()){
                for(const auto& v : gc["InteriaCompInte"]){
                    mGravCompData.InteriaCompInte.push_back(v.as<double>(1.0));
                }
            }

            // 加速度滤波器
            if(gc["accFtr"]){
                readAccFtr(gc["accFtr"]);
            }
        } else {
            std::cerr << "警告：controllerConfig.yaml 中未找到 gravComp 配置节点" << std::endl;
        }

        // ===== 2. 读取 gravCompFtr 节点 =====
        if(config["gravCompFtr"]){
            readGravCompFtr(config["gravCompFtr"]);
        } else {
            std::cerr << "警告：controllerConfig.yaml 中未找到 gravCompFtr 配置节点" << std::endl;
        }

        // ===== 3. 维度校验 =====
        if(static_cast<int>(mGravCompData.gravCompInte.size()) != ALLJNTNUM){
            std::cerr << "错误：GravCompInte 维度 " << mGravCompData.gravCompInte.size()
                      << " 与 ALLJNTNUM=" << ALLJNTNUM << " 不匹配" << std::endl;
            loaded_ = false;
            return false;
        }
        if(static_cast<int>(mGravCompData.coriousCompInte.size()) != ALLJNTNUM){
            std::cerr << "错误：CoriousCompInte 维度 " << mGravCompData.coriousCompInte.size()
                      << " 与 ALLJNTNUM=" << ALLJNTNUM << " 不匹配" << std::endl;
            loaded_ = false;
            return false;
        }
        if(static_cast<int>(mGravCompData.InteriaCompInte.size()) != ALLJNTNUM){
            std::cerr << "错误：InteriaCompInte 维度 " << mGravCompData.InteriaCompInte.size()
                      << " 与 ALLJNTNUM=" << ALLJNTNUM << " 不匹配" << std::endl;
            loaded_ = false;
            return false;
        }
        if(static_cast<int>(mGravCompData.acc_ftr.size()) != ALLJNTNUM){
            std::cerr << "错误：acc_ftr 维度 " << mGravCompData.acc_ftr.size()
                      << " 与 ALLJNTNUM=" << ALLJNTNUM << " 不匹配" << std::endl;
            loaded_ = false;
            return false;
        }

        // gravCompFtr 各滤波器向量维度校验 (每臂 LARMDOF 个)
        const auto& ftr = mGravCompData.gravComp_InpOutpFtr;
        auto checkFtrVec = [&](const std::vector<lowpassFtrInte>& vec, const char* name) -> bool {
            if(static_cast<int>(vec.size()) != LARMDOF){
                std::cerr << "错误：" << name << " 维度 " << vec.size()
                          << " 与 LARMDOF=" << LARMDOF << " 不匹配" << std::endl;
                loaded_ = false;
                return false;
            }
            return true;
        };

        checkFtrVec(ftr.lArm_posInpFtr,  "gravCompFtr.lArm_posInpFtr");
        checkFtrVec(ftr.lArm_velInpFtr,  "gravCompFtr.lArm_velInpFtr");
        checkFtrVec(ftr.lArm_torInpFtr,  "gravCompFtr.lArm_torInpFtr");
        checkFtrVec(ftr.lArm_torOutpFtr, "gravCompFtr.lArm_torOutpFtr");
        checkFtrVec(ftr.rArm_posInpFtr,  "gravCompFtr.rArm_posInpFtr");
        checkFtrVec(ftr.rArm_velInpFtr,  "gravCompFtr.rArm_velInpFtr");
        checkFtrVec(ftr.rArm_torInpFtr,  "gravCompFtr.rArm_torInpFtr");
        checkFtrVec(ftr.rArm_torOutpFtr, "gravCompFtr.rArm_torOutpFtr");
        checkFtrVec(ftr.lArm_posOutpFtr, "gravCompFtr.lArm_posOutpFtr");
        checkFtrVec(ftr.lArm_velOutpFtr, "gravCompFtr.lArm_velOutpFtr");
        checkFtrVec(ftr.rArm_posOutpFtr, "gravCompFtr.rArm_posOutpFtr");
        checkFtrVec(ftr.rArm_velOutpFtr, "gravCompFtr.rArm_velOutpFtr");

        loaded_ = true;
        std::cout << "===============重力补偿配置加载完毕===============" << std::endl;
        printConfig();
        return true;
    }
    catch(const YAML::BadFile& e){
        std::cerr << "错误：无法打开重力补偿配置文件 " << yaml_path_ << std::endl;
        std::cerr << "  " << e.what() << std::endl;
        loaded_ = false;
        return false;
    }
    catch(const YAML::Exception& e){
        std::cerr << "错误：解析重力补偿配置文件失败 " << yaml_path_ << std::endl;
        std::cerr << "  " << e.what() << std::endl;
        loaded_ = false;
        return false;
    }
}

gravCompInte GravCompConfig::getGravCompData() const{
    return mGravCompData;
}

bool GravCompConfig::isLoaded() const{
    return loaded_;
}

void GravCompConfig::printConfig() const{
    std::cout << "========================================" << std::endl;
    std::cout << "  重力补偿配置摘要" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "  重力补偿   : " << (mGravCompData.ifUseGravComp ? "开启" : "关闭") << std::endl;
    std::cout << "  输入滤波器 : " << (mGravCompData.ifUseInpFtr   ? "开启" : "关闭") << std::endl;
    std::cout << "  输出滤波器 : " << (mGravCompData.ifUseOutpFtr  ? "开启" : "关闭") << std::endl;
    std::cout << "  数据输入模式 : " << mGravCompData.InpDataMod << std::endl;
    std::cout << "  摩擦补偿     : " << (mGravCompData.ifUseFricComp ? "开启" : "关闭") << std::endl;
    std::cout << "  重力补偿零输出轮数 : " << mGravCompData.GravCompStZeroTimes << std::endl;
    std::cout << "  重力补偿平滑输出轮数 : " << mGravCompData.GravCompStSmoothTimes << std::endl;
    std::cout << "  关节控制初始化稳定轮数 : " << mGravCompData.JntCtlStStabTimes << std::endl;
    std::cout << "  GravCompInte    [" << mGravCompData.gravCompInte.size() << "] :";
    for(size_t i = 0; i < mGravCompData.gravCompInte.size(); i++){
        printf(" %.1f", mGravCompData.gravCompInte[i]);
    }
    std::cout << std::endl;
    std::cout << "  CoriousCompInte [" << mGravCompData.coriousCompInte.size() << "] :";
    for(size_t i = 0; i < mGravCompData.coriousCompInte.size(); i++){
        printf(" %.1f", mGravCompData.coriousCompInte[i]);
    }
    std::cout << std::endl;
    std::cout << "  InteriaCompInte [" << mGravCompData.InteriaCompInte.size() << "] :";
    for(size_t i = 0; i < mGravCompData.InteriaCompInte.size(); i++){
        printf(" %.1f", mGravCompData.InteriaCompInte[i]);
    }
    std::cout << std::endl;
    std::cout << "  加速度卡尔曼滤波器 [" << mGravCompData.acc_ftr.size() << "] :" << std::endl;
    std::cout << "    Q_gain:";
    for(const auto& f : mGravCompData.acc_ftr){
        printf(" %.0f", f.Q_gain);
    }
    std::cout << std::endl;
    std::cout << "    R_gain:";
    for(const auto& f : mGravCompData.acc_ftr){
        printf(" %.0f", f.R_gain);
    }
    std::cout << std::endl;
    std::cout << "  位置输入滤波器 (左臂) :";
    for(const auto& f : mGravCompData.gravComp_InpOutpFtr.lArm_posInpFtr){
        printf(" [%.0f/%.0f]", f.sampRate, f.cutoffRate);
    }
    std::cout << std::endl;
    std::cout << "  力矩输出滤波器 (左臂) :";
    for(const auto& f : mGravCompData.gravComp_InpOutpFtr.lArm_torOutpFtr){
        printf(" [%.0f/%.0f]", f.sampRate, f.cutoffRate);
    }
    std::cout << std::endl;
    std::cout << "========================================" << std::endl;
}

//==============================================================================
//  solverControlParam — 求解器控制配置读取
//==============================================================================

// 构造函数
solverControlParam::solverControlParam()
    : yaml_path_("")
    , loaded_(false)
{
    // optimizerCtl 默认值
    mOptiCtlData.OptiWeight[0] = 50.0;
    mOptiCtlData.OptiWeight[1] = 1.0;
    mOptiCtlData.OptiWeight[2] = 0.002;
    mOptiCtlData.OptiWeight[3] = 0.01;
    mOptiCtlData.OptiWeight[4] = 0.0002;
    mOptiCtlData.template_times  = 100;
    mOptiCtlData.errTol          = 1e-5;
    mOptiCtlData.ifPrintLevel    = 0;
    mOptiCtlData.ifPrintOptiTime = false;
    mOptiCtlData.joint_step_limit_enabled = true;
    mOptiCtlData.max_joint_step_rad.assign(14, 0.025);

    // optiPrintMsgCtl 默认值
    mPrintMsgData.dispJntInfo       = false;
    mPrintMsgData.use_visualization = false;
    mPrintMsgData.disp_messages     = false;
}

solverControlParam::solverControlParam(const std::string& yaml_path)
    : yaml_path_(yaml_path)
    , loaded_(false)
{
    // 默认值
    mOptiCtlData.OptiWeight[0] = 50.0;
    mOptiCtlData.OptiWeight[1] = 1.0;
    mOptiCtlData.OptiWeight[2] = 0.002;
    mOptiCtlData.OptiWeight[3] = 0.01;
    mOptiCtlData.OptiWeight[4] = 0.0002;
    mOptiCtlData.template_times  = 100;
    mOptiCtlData.errTol          = 1e-5;
    mOptiCtlData.ifPrintLevel    = 0;
    mOptiCtlData.ifPrintOptiTime = false;
    mOptiCtlData.joint_step_limit_enabled = true;
    mOptiCtlData.max_joint_step_rad.assign(14, 0.025);

    mPrintMsgData.dispJntInfo       = false;
    mPrintMsgData.use_visualization = false;
    mPrintMsgData.disp_messages     = false;

    loadFromYaml(yaml_path);
}

bool solverControlParam::loadFromYaml(const std::string& yaml_path){
    yaml_path_ = yaml_path;

    try{
        YAML::Node config = YAML::LoadFile(yaml_path_);

        // ===== 读取 IKoptimizeSolverPara 节点 =====
        if(config["IKoptimizeSolverPara"]){
            const auto& ik = config["IKoptimizeSolverPara"];

            // ---- 优化器权重 (5维) ----
            if(ik["optiWeights"] && ik["optiWeights"].IsSequence()){
                size_t wcount = std::min(
                    static_cast<size_t>(ik["optiWeights"].size()),
                    static_cast<size_t>(5)
                );
                for(size_t i = 0; i < wcount; i++){
                    mOptiCtlData.OptiWeight[i] = ik["optiWeights"][i].as<double>();
                }
            }

            // ---- 迭代次数 ----
            mOptiCtlData.template_times = ik["templateTimes"].as<int>(100);

            // ---- 误差容差 (带合法性校验，与 ik_solver.py 中一致) ----
            mOptiCtlData.errTol = ik["errorTolerance"].as<double>(1e-6);
            if(mOptiCtlData.errTol <= 0.0 || mOptiCtlData.errTol < 1e-8){
                std::cout << "Warning(IK-tol): YAML errorTolerance="
                          << mOptiCtlData.errTol
                          << " is invalid for IPOPT (requires > 1e-08). "
                          << "Using default tol=1e-5 instead." << std::endl;
                mOptiCtlData.errTol = 1e-5;
            }

            // ---- 信息打印控制 (0/1 整数 → bool/int) ----
            mOptiCtlData.ifPrintLevel    = ik["ifPrintLevel"].as<int>(0);
            mOptiCtlData.ifPrintOptiTime = (ik["ifPrintOptiTime"].as<int>(0) != 0);

            mPrintMsgData.dispJntInfo       = (ik["ifDispJntInfo"].as<int>(0) != 0);
            mPrintMsgData.use_visualization = (ik["ifUse_visulization"].as<int>(0) != 0);
            mPrintMsgData.disp_messages     = (ik["ifDispMessages"].as<int>(0) != 0);
        } else {
            std::cerr << "警告：controllerConfig.yaml 中未找到 IKoptimizeSolverPara "
                      << "配置节点，使用默认求解器参数" << std::endl;
        }

        // ---- IK 关节目标跳变保护 ----
        // A scalar applies to every joint; a 14-element sequence allows
        // joints with different safe speeds to be tuned independently.
        if(config["jointTargetSafety"]){
            const auto& safety = config["jointTargetSafety"];
            mOptiCtlData.joint_step_limit_enabled = safety["enabled"].as<bool>(true);
            if(safety["max_delta_rad"]){
                std::vector<double> limits;
                const auto& node = safety["max_delta_rad"];
                if(node.IsSequence()){
                    limits.reserve(node.size());
                    for(std::size_t i = 0; i < node.size(); ++i){
                        limits.push_back(node[i].as<double>());
                    }
                }else{
                    limits.assign(14, node.as<double>());
                }
                if(limits.size() == 14 && std::all_of(
                       limits.begin(), limits.end(),
                       [](double value){ return std::isfinite(value) && value > 0.0; })){
                    mOptiCtlData.max_joint_step_rad = std::move(limits);
                }else{
                    std::cerr << "警告：jointTargetSafety.max_delta_rad 无效，使用默认 0.025 rad" << std::endl;
                }
            }
        }

        loaded_ = true;
        std::cout << "===============求解器控制配置加载完毕==============="
                  << std::endl;
        printConfig();
        return true;
    }
    catch(const YAML::BadFile& e){
        std::cerr << "错误：无法打开求解器控制配置文件 " << yaml_path_ << std::endl;
        std::cerr << "  " << e.what() << std::endl;
        loaded_ = false;
        return false;
    }
    catch(const YAML::Exception& e){
        std::cerr << "错误：解析求解器控制配置文件失败 " << yaml_path_ << std::endl;
        std::cerr << "  " << e.what() << std::endl;
        loaded_ = false;
        return false;
    }
}

optimizerCtl solverControlParam::getOptimizerCtl() const{
    return mOptiCtlData;
}

optiPrintMsgCtl solverControlParam::getPrintMsgCtl() const{
    return mPrintMsgData;
}

bool solverControlParam::isLoaded() const{
    return loaded_;
}

void solverControlParam::printConfig() const{
    std::cout << "========================================" << std::endl;
    std::cout << "  求解器控制配置摘要" << std::endl;
    std::cout << "========================================" << std::endl;

    // ---- 优化器参数 ----
    std::cout << "  --- 优化器控制参数 ---" << std::endl;
    std::cout << "  优化权重 (optiWeight): [";
    for(int i = 0; i < 5; i++){
        std::cout << mOptiCtlData.OptiWeight[i];
        if(i < 4) std::cout << ", ";
    }
    std::cout << "]" << std::endl;
    std::cout << "  最大迭代次数 (templateTimes):  "
              << mOptiCtlData.template_times << std::endl;
    std::cout << "  误差容差 (errorTolerance):     "
              << mOptiCtlData.errTol << std::endl;
    std::cout << "  关节目标跳变保护: "
              << (mOptiCtlData.joint_step_limit_enabled ? "开启" : "关闭")
              << ", 步长(rad): ";
    if(mOptiCtlData.max_joint_step_rad.empty()){
        std::cout << "无";
    }else{
        std::cout << mOptiCtlData.max_joint_step_rad.front();
        if(mOptiCtlData.max_joint_step_rad.size() > 1){
            std::cout << " ... (" << mOptiCtlData.max_joint_step_rad.size() << " joints)";
        }
    }
    std::cout << std::endl;

    // ---- 打印控制参数 ----
    std::cout << "  --- 信息打印控制参数 ---" << std::endl;
    std::cout << "  打印优化日志 (ifPrintLevel):       "
              << mOptiCtlData.ifPrintLevel << std::endl;
    std::cout << "  打印计算时间 (ifPrintOptiTime):   "
              << (mOptiCtlData.ifPrintOptiTime ? "是" : "否") << std::endl;
    std::cout << "  打印关节配置 (ifDispJntInfo):     "
              << (mPrintMsgData.dispJntInfo ? "是" : "否") << std::endl;
    std::cout << "  Meshcat可视化 (ifUse_visulization): "
              << (mPrintMsgData.use_visualization ? "是" : "否") << std::endl;
    std::cout << "  打印优化器信息 (ifDispMessages):  "
              << (mPrintMsgData.disp_messages ? "是" : "否") << std::endl;

    std::cout << "========================================" << std::endl;
}

//==============================================================================
//  TransLoopCtlConfig — 总控流程配置读取
//==============================================================================

// 辅助函数：从 YAML node 读取单臂卡尔曼滤波器组 (7个 kalmanFtrInte)
std::vector<kalmanFtrInte> TransLoopCtlConfig::readKalmanFilterGroup(
    const YAML::Node& node,
    const std::string& Qgain_key,
    const std::string& Rgain_key)
{
    std::vector<kalmanFtrInte> result;
    if(node && node[Qgain_key] && node[Rgain_key]){
        size_t count = std::min(node[Qgain_key].size(), node[Rgain_key].size());
        for(size_t i = 0; i < count; i++){
            kalmanFtrInte ftr;
            ftr.Q_gain = node[Qgain_key][i].as<double>(1.0);
            ftr.R_gain = node[Rgain_key][i].as<double>(0.01);
            result.push_back(ftr);
        }
    } else {
        // 默认值
        for(int i = 0; i < LARMDOF; i++){
            kalmanFtrInte ftr;
            ftr.Q_gain = 1.0;
            ftr.R_gain = 0.01;
            result.push_back(ftr);
        }
    }
    return result;
}

// 构造函数
TransLoopCtlConfig::TransLoopCtlConfig()
    : yaml_path_("")
    , loaded_(false)
{
}

TransLoopCtlConfig::TransLoopCtlConfig(const std::string& yaml_path)
    : yaml_path_(yaml_path)
    , loaded_(false)
{
    loadFromYaml(yaml_path);
}

bool TransLoopCtlConfig::loadFromYaml(const std::string& yaml_path){
    yaml_path_ = yaml_path;

    try{
        YAML::Node config = YAML::LoadFile(yaml_path_);

        // ===== 读取 TransLoopCtlInte 节点 =====
        if(config["TransLoopCtlInte"]){
            const auto& tl = config["TransLoopCtlInte"];

            // ---- 读取中层节点控制参数 ----
            if(tl["dual_arm_ctl_node_loop_ctl"]){
                const auto& mid = tl["dual_arm_ctl_node_loop_ctl"];

                mTransLoopCtlData.midNode_tempTime = mid["TempTime"].as<double>(0.02);
                mTransLoopCtlData.lArm_Dof        = mid["lArm_DOF"].as<int>(7);
                mTransLoopCtlData.rArm_Dof        = mid["rArm_DOF"].as<int>(7);

                // 机器人求解链定义
                mTransLoopCtlData.urdf_path_option = mid["urdf_path_option"].as<std::string>("dual_arm.urdf");
                mTransLoopCtlData.base_link_lArm   = mid["base_link_lArm"].as<std::string>("arm_base_link");
                mTransLoopCtlData.tip_link_lArm    = mid["tip_link_lArm"].as<std::string>("l_link_8");
                mTransLoopCtlData.base_link_rArm   = mid["base_link_rArm"].as<std::string>("arm_base_link");
                mTransLoopCtlData.tip_link_rArm    = mid["tip_link_rArm"].as<std::string>("r_link_8");
            } else {
                std::cerr << "警告：TransLoopCtlInte 中未找到 dual_arm_ctl_node_loop_ctl "
                          << "配置节点，使用默认中层节点参数" << std::endl;
                mTransLoopCtlData.midNode_tempTime = 0.02;
                mTransLoopCtlData.lArm_Dof         = 7;
                mTransLoopCtlData.rArm_Dof         = 7;
                mTransLoopCtlData.urdf_path_option = "dual_arm.urdf";
                mTransLoopCtlData.base_link_lArm   = "arm_base_link";
                mTransLoopCtlData.tip_link_lArm    = "l_link_8";
                mTransLoopCtlData.base_link_rArm   = "arm_base_link";
                mTransLoopCtlData.tip_link_rArm    = "r_link_8";
            }

            // ---- 读取位置反馈值通用卡尔曼滤波参数 ----
            if(tl["posRetKalmanFilterInte"]){
                const auto& kf = tl["posRetKalmanFilterInte"];

                // 左臂位置反馈卡尔曼滤波
                mTransLoopCtlData.lArm_posFeedback_uniFtr =
                    readKalmanFilterGroup(kf, "l_JntPos_Qgain", "l_JntPos_Rgain");

                // 右臂位置反馈卡尔曼滤波
                mTransLoopCtlData.rArm_posFeedback_uniFtr =
                    readKalmanFilterGroup(kf, "r_JntPos_Qgain", "r_JntPos_Rgain");
            } else {
                std::cerr << "警告：TransLoopCtlInte 中未找到 posRetKalmanFilterInte "
                          << "配置节点，使用默认卡尔曼滤波参数" << std::endl;
                mTransLoopCtlData.lArm_posFeedback_uniFtr =
                    readKalmanFilterGroup(YAML::Node(), "", "");
                mTransLoopCtlData.rArm_posFeedback_uniFtr =
                    readKalmanFilterGroup(YAML::Node(), "", "");
            }

            // ---- 读取电流反馈值通用卡尔曼滤波参数 ----
            if(tl["IqRetKalmanFilterInte"]){
                const auto& kf_iq = tl["IqRetKalmanFilterInte"];

                // 左臂电流反馈卡尔曼滤波
                mTransLoopCtlData.lArm_IqFeedback_uniFtr = 
                    readKalmanFilterGroup(kf_iq, "l_JntIq_Qgain", "l_JntIq_Rgain");

                // 右臂位置反馈卡尔曼滤波
                mTransLoopCtlData.rArm_IqFeedback_uniFtr = 
                    readKalmanFilterGroup(kf_iq, "r_JntIq_Qgain", "r_JntIq_Rgain");
            } else {
                std::cerr << "警告：TransLoopCtlInte 中未找到 IqRetKalmanFilterInte "
                          << "配置节点，使用默认卡尔曼滤波参数" << std::endl;
                mTransLoopCtlData.lArm_IqFeedback_uniFtr =
                    readKalmanFilterGroup(YAML::Node(), "", "");
                mTransLoopCtlData.rArm_IqFeedback_uniFtr =
                    readKalmanFilterGroup(YAML::Node(), "", "");
            }

        } else {
            std::cerr << "警告：controllerConfig.yaml 中未找到 TransLoopCtlInte "
                      << "配置节点，使用默认卡尔曼滤波参数" << std::endl;
            
            mTransLoopCtlData.lArm_posFeedback_uniFtr =
                readKalmanFilterGroup(YAML::Node(), "", "");
            mTransLoopCtlData.rArm_posFeedback_uniFtr =
                readKalmanFilterGroup(YAML::Node(), "", "");

            mTransLoopCtlData.lArm_IqFeedback_uniFtr =
                readKalmanFilterGroup(YAML::Node(), "", "");
            mTransLoopCtlData.rArm_IqFeedback_uniFtr =
                readKalmanFilterGroup(YAML::Node(), "", "");
        }

        // ===== 维度校验 =====
        if(static_cast<int>(mTransLoopCtlData.lArm_posFeedback_uniFtr.size()) != mTransLoopCtlData.lArm_Dof){
            std::cerr << "错误：左臂位置反馈卡尔曼滤波参数维度 "
                      << mTransLoopCtlData.lArm_posFeedback_uniFtr.size()
                      << " 与 LARMDOF=" << mTransLoopCtlData.lArm_Dof << " 不匹配" << std::endl;
            loaded_ = false;
            return false;
        }
        if(static_cast<int>(mTransLoopCtlData.rArm_posFeedback_uniFtr.size()) != mTransLoopCtlData.rArm_Dof){
            std::cerr << "错误：右臂位置反馈卡尔曼滤波参数维度 "
                      << mTransLoopCtlData.rArm_posFeedback_uniFtr.size()
                      << " 与 RARMDOF=" << mTransLoopCtlData.rArm_Dof << " 不匹配" << std::endl;
            loaded_ = false;
            return false;
        }

        loaded_ = true;
        std::cout << "===============总控流程配置加载完毕===============" << std::endl;
        printConfig();
        return true;
    }
    catch(const YAML::BadFile& e){
        std::cerr << "错误：无法打开总控流程配置文件 " << yaml_path_ << std::endl;
        std::cerr << "  " << e.what() << std::endl;
        loaded_ = false;
        return false;
    }
    catch(const YAML::Exception& e){
        std::cerr << "错误：解析总控流程配置文件失败 " << yaml_path_ << std::endl;
        std::cerr << "  " << e.what() << std::endl;
        loaded_ = false;
        return false;
    }
}

TransLoopCtl TransLoopCtlConfig::getTransLoopCtlData() const{
    return mTransLoopCtlData;
}

bool TransLoopCtlConfig::isLoaded() const{
    return loaded_;
}

void TransLoopCtlConfig::printConfig() const{
    std::cout << "========================================" << std::endl;
    std::cout << "  总控流程配置摘要" << std::endl;
    std::cout << "========================================" << std::endl;

    // ---- 中层节点控制参数 ----
    std::cout << "  [中层节点控制参数]" << std::endl;
    std::cout << "    TempTime: "       << mTransLoopCtlData.midNode_tempTime << " s" << std::endl;
    std::cout << "    lArm_DOF: "       << mTransLoopCtlData.lArm_Dof << std::endl;
    std::cout << "    rArm_DOF: "       << mTransLoopCtlData.rArm_Dof << std::endl;
    std::cout << "    urdf_path_option: " << mTransLoopCtlData.urdf_path_option << std::endl;
    std::cout << "    base_link_lArm: "   << mTransLoopCtlData.base_link_lArm << std::endl;
    std::cout << "    tip_link_lArm: "    << mTransLoopCtlData.tip_link_lArm << std::endl;
    std::cout << "    base_link_rArm: "   << mTransLoopCtlData.base_link_rArm << std::endl;
    std::cout << "    tip_link_rArm: "    << mTransLoopCtlData.tip_link_rArm << std::endl;

    // ---- 卡尔曼滤波参数 ----
    std::cout << "  位置反馈卡尔曼滤波器 (左臂) ["
              << mTransLoopCtlData.lArm_posFeedback_uniFtr.size() << "] :" << std::endl;
    std::cout << "    Q_gain:";
    for(const auto& f : mTransLoopCtlData.lArm_posFeedback_uniFtr){
        printf(" %.2f", f.Q_gain);
    }
    std::cout << std::endl;
    std::cout << "    R_gain:";
    for(const auto& f : mTransLoopCtlData.lArm_posFeedback_uniFtr){
        printf(" %.2f", f.R_gain);
    }
    std::cout << std::endl;
    std::cout << "  位置反馈卡尔曼滤波器 (右臂) ["
              << mTransLoopCtlData.rArm_posFeedback_uniFtr.size() << "] :" << std::endl;
    std::cout << "    Q_gain:";
    for(const auto& f : mTransLoopCtlData.rArm_posFeedback_uniFtr){
        printf(" %.2f", f.Q_gain);
    }
    std::cout << std::endl;
    std::cout << "    R_gain:";
    for(const auto& f : mTransLoopCtlData.rArm_posFeedback_uniFtr){
        printf(" %.2f", f.R_gain);
    }
    std::cout << std::endl;
    std::cout << "========================================" << std::endl;
}

//==============================================================================
// 接口函数封装
//==============================================================================

// ----- 关节层零位配置 -----
//将零位角度配置读取到全局变量中
bool loadZeroPosFromYaml(const std::string& yaml_path){
    ZeroPosConfig zeroCfg;
    if(!zeroCfg.loadFromYaml(yaml_path)){
        std::cerr << "警告：零位配置文件加载失败，使用默认零位 0.0" << std::endl;
        return false;
    }

    lArm_zeroPos = zeroCfg.getLeftArmZeroPos();
    rArm_zeroPos = zeroCfg.getRightArmZeroPos();

    std::cout << "零位配置已同步到全局控制变量" << std::endl;
    return true;
}



// ----- 笛卡尔零位配置 -----
//将零位笛卡尔空间配置读取到全局变量中
bool loadDescZeroPosFromYaml(const std::string& yaml_path){

    DescZeroPosConfig descCfg;
    if(!descCfg.loadFromYaml(yaml_path)){
        std::cerr << "警告：笛卡尔零位配置文件加载失败，使用默认零位 [0,0,0,0,0,0]" << std::endl;
        return false;
    }else{

        l_descZeroPos = descCfg.getLeftArmDescZeroPos();
        r_descZeroPos = descCfg.getRightArmDescZeroPos();

        std::cout << "笛卡尔零位配置已同步到全局控制变量" << std::endl;
        return true;
    }
}

// ----- 默认PD参数配置 -----
//加载 PD 配置文件 -- 从 initialPD.yaml 读取 PD 参数并填入全局变量
bool loadInitialPDFromYaml(const std::string& yaml_path){
    InitialPDConfig pdCfg_low;
    InitialPDConfig pdCfg_mid;
    InitialPDConfig pdCfg_high;
    InitialPDConfig pdCfg_shigh;
    InitialPDConfig pdCfg_tag;

    std::cout << "1. 读取低刚度PD参数配置" << std::endl;
    if(!pdCfg_low.loadFromYaml(yaml_path, "low")){
        std::cerr << "Warning: PD config load failed (level: low), using default Kp=0, Kd=0" << std::endl;
        return false;
    }
    std::cout << "低刚度PD参数配置成功" << std::endl;

    std::cout << "2. 读取中刚度PD参数配置" << std::endl;
    if(!pdCfg_mid.loadFromYaml(yaml_path, "mid")){
        std::cerr << "Warning: PD config load failed (level: mid), using default Kp=0, Kd=0" << std::endl;
        return false;
    }
    std::cout << "中刚度PD参数配置成功" << std::endl;

    std::cout << "3. 读取高刚度PD参数配置" << std::endl;
    if(!pdCfg_high.loadFromYaml(yaml_path, "high")){
        std::cerr << "Warning: PD config load failed (level: high), using default Kp=0, Kd=0" << std::endl;
        return false;
    }
    std::cout << "高刚度PD参数配置成功" << std::endl;

    std::cout << "4. 读取超高刚度PD参数配置" << std::endl;
    if(!pdCfg_shigh.loadFromYaml(yaml_path, "superhigh")){
        std::cerr << "Warning: PD config load failed (level: superhigh), using default Kp=0, Kd=0" << std::endl;
        return false;
    }
    std::cout << "超高刚度PD参数配置成功" << std::endl;

    std::cout << "5. 拖拽过程PD参数配置" << std::endl;
    if(!pdCfg_tag.loadFromYaml(yaml_path, "tag")){
        std::cerr << "Warning: PD config load failed (level: tag), using default Kp=0, Kd=0" << std::endl;
        return false;
    }
    std::cout << "拖拽过程PD参数配置成功" << std::endl;

    // 将各刚度等级 PD 参数同步到全局变量
    lArm_defaultKp_low  = pdCfg_low.getLeftArmKp();
    lArm_defaultKd_low  = pdCfg_low.getLeftArmKd();
    rArm_defaultKp_low  = pdCfg_low.getRightArmKp();
    rArm_defaultKd_low  = pdCfg_low.getRightArmKd();

    lArm_defaultKp_mid  = pdCfg_mid.getLeftArmKp();
    lArm_defaultKd_mid  = pdCfg_mid.getLeftArmKd();
    rArm_defaultKp_mid  = pdCfg_mid.getRightArmKp();
    rArm_defaultKd_mid  = pdCfg_mid.getRightArmKd();

    lArm_defaultKp_high = pdCfg_high.getLeftArmKp();
    lArm_defaultKd_high = pdCfg_high.getLeftArmKd();
    rArm_defaultKp_high = pdCfg_high.getRightArmKp();
    rArm_defaultKd_high = pdCfg_high.getRightArmKd();

    lArm_defaultKp_superhigh = pdCfg_shigh.getLeftArmKp();
    lArm_defaultKd_superhigh = pdCfg_shigh.getLeftArmKd();
    rArm_defaultKp_superhigh = pdCfg_shigh.getRightArmKp();
    rArm_defaultKd_superhigh = pdCfg_shigh.getRightArmKd();

    lArm_defaultKp_tag  = pdCfg_tag.getLeftArmKp();
    lArm_defaultKd_tag  = pdCfg_tag.getLeftArmKd();
    rArm_defaultKp_tag  = pdCfg_tag.getRightArmKp();
    rArm_defaultKd_tag  = pdCfg_tag.getRightArmKd();

    std::cout << "PD 配置已同步到全局控制变量 (low/mid/high/superhigh/tag)" << std::endl;
    return true;
}

// ----- 关节连杆结构配置 -----
//加载关节连杆结构配置文件 -- 从 jointLink_config.yaml 读取并填入全局变量
bool loadJointLinkFromYaml(const std::string& yaml_path){
    JointLinkConfig jointCfg;

    if(!jointCfg.loadFromYaml(yaml_path)){
        std::cerr << "Warning: 关节连杆结构配置文件加载失败" << std::endl;
        return false;
    }

    // 将读取到的配置写入全局变量 robotJointLinkConfig
    robotJointLinkConfig.lArm_frame_names      = jointCfg.getLeftArmFrameNames();
    robotJointLinkConfig.lArm_JntRotation_axis = jointCfg.getLeftArmRotationAxis();
    robotJointLinkConfig.lArm_JntNum           = jointCfg.getLeftArmJointNum();

    robotJointLinkConfig.rArm_frame_names      = jointCfg.getRightArmFrameNames();
    robotJointLinkConfig.rArm_JntRotation_axis = jointCfg.getRightArmRotationAxis();
    robotJointLinkConfig.rArm_JntNum           = jointCfg.getRightArmJointNum();

    std::cout << "关节连杆结构配置已同步到全局变量 robotJointLinkConfig" << std::endl;
    return true;
}

// ----- 机器人模型结构配置 -----
// 从 robot_config.yaml + .urdf 构建机器人结构配置与pinocchio模型
// 同时填充全局变量 robotModelStructConfig 与 robotProcessedModel
bool loadRobotConfigFromYaml(const std::string& yaml_filePath, const std::string& urdf_filepath){
    loadRobotCofig cfg;

    try{
        cfg.getModelFromYaml(yaml_filePath, urdf_filepath);
    }
    catch(const std::exception& e){
        std::cerr << "错误：机器人模型结构配置加载失败" << std::endl;
        std::cerr << "  " << e.what() << std::endl;
        return false;
    }

    // 提取结果到全局变量
    robotModelStructConfig = cfg.getRobotStructConfig();
    robotProcessedModel    = cfg.processed_model_outp();

    std::cout << "===============机器人模型结构配置已同步到全局变量===============" << std::endl;
    std::cout << "  robotModelStructConfig: base_name=" << robotModelStructConfig.base_name
              << ", lArm_JntNum=" << robotModelStructConfig.lArm_JntNum
              << ", rArm_JntNum=" << robotModelStructConfig.rArm_JntNum << std::endl;
    std::cout << "  robotProcessedModel: njoints=" << robotProcessedModel.njoints
              << ", nq=" << robotProcessedModel.nq
              << ", nv=" << robotProcessedModel.nv << std::endl;

    return true;
}

// ----- 重力补偿参数配置 -----
// 从 controllerConfig.yaml 读取重力补偿参数并填入全局变量 robotGravCompInte
bool loadGravCompFromYaml(const std::string& yaml_path){
    GravCompConfig cfg;
    if(!cfg.loadFromYaml(yaml_path)){
        std::cerr << "Warning: 重力补偿配置文件加载失败" << std::endl;
        return false;
    }
    robotGravCompInte = cfg.getGravCompData();
    std::cout << "重力补偿配置已同步到全局变量 robotGravCompInte" << std::endl;
    return true;
}

// ----- 求解器控制参数配置 -----
// 从 controllerConfig.yaml 读取求解器控制参数并填入全局变量
// optimizerControParam 与 msgPringControlParam
bool loadSolverControlFromYaml(const std::string& yaml_path){
    solverControlParam cfg;
    if(!cfg.loadFromYaml(yaml_path)){
        std::cerr << "Warning: 求解器控制配置文件加载失败" << std::endl;
        return false;
    }
    optimizerControParam  = cfg.getOptimizerCtl();
    msgPringControlParam  = cfg.getPrintMsgCtl();
    std::cout << "求解器控制参数已同步到全局变量 "
              << "optimizerControParam / msgPringControlParam" << std::endl;
    return true;
}

// ----- 总控流程参数配置 -----
// 从 controllerConfig.yaml 读取总控流程参数并填入全局变量 dataTransLoopCtl
bool loadTransLoopCtlFromYaml(const std::string& yaml_path){
    TransLoopCtlConfig cfg;
    if(!cfg.loadFromYaml(yaml_path)){
        std::cerr << "Warning: 总控流程配置文件加载失败" << std::endl;
        return false;
    }
    dataTransLoopCtl = cfg.getTransLoopCtlData();
    std::cout << "总控流程配置已同步到全局变量 dataTransLoopCtl" << std::endl;
    return true;
}
