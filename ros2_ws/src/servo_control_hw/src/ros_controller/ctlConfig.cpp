#include "ros_controller/ctlConfig.h"

ctlInteConfig ctl_inteMsg_config;

//yaml文件路径
std::string controllerConfig_yamlPath;

// ===== 构造函数 =====

ServoNodeCtlConfig::ServoNodeCtlConfig()
    : yaml_path_("")
    , loaded_(false)
{
    mData.TempTime          = 0.001;
    mData.LARMDOF           = 7;
    mData.RARMDOF           = 7;
    mData.ZeroPos_Threshold = 0.02;
    mData.NetCardName       = "enp0s31f6";

    // 夹爪力矩/温度限幅默认值
    mData.GripperLeft_Tor_Thr   = 10.0;
    mData.GripperLeft_Temp_Thr  = 70.0;
    mData.GripperRight_Tor_Thr  = 10.0;
    mData.GripperRight_Temp_Thr = 70.0;
}

ServoNodeCtlConfig::ServoNodeCtlConfig(const std::string& yaml_path)
    : yaml_path_(yaml_path)
    , loaded_(false)
{
    mData.TempTime          = 0.001;
    mData.LARMDOF           = 7;
    mData.RARMDOF           = 7;
    mData.ZeroPos_Threshold = 0.02;
    mData.NetCardName       = "enp0s31f6";

    // 夹爪力矩/温度限幅默认值
    mData.GripperLeft_Tor_Thr   = 10.0;
    mData.GripperLeft_Temp_Thr  = 70.0;
    mData.GripperRight_Tor_Thr  = 10.0;
    mData.GripperRight_Temp_Thr = 70.0;
    loadFromYaml(yaml_path);
}

// ===== 从 YAML 加载 =====

bool ServoNodeCtlConfig::loadFromYaml(const std::string& yaml_path){
    yaml_path_ = yaml_path;

    try{
        YAML::Node config = YAML::LoadFile(yaml_path_);

        if(config["TransLoopCtlInte"] && config["TransLoopCtlInte"]["servo_node_loop_ctl"]){
            const auto& sn = config["TransLoopCtlInte"]["servo_node_loop_ctl"];

            mData.TempTime          = sn["TempTime"].as<double>(0.001);
            mData.LARMDOF           = sn["lArm_DOF"].as<double>(7);
            mData.RARMDOF           = sn["rArm_DOF"].as<double>(7);
            mData.ZeroPos_Threshold = sn["ZeroPos_Threshold"].as<double>(0.02);
            mData.NetCardName       = sn["netCard_name"].as<std::string>("enp0s31f6");
        } else {
            std::cerr << "警告：未找到 TransLoopCtlInte/servo_node_loop_ctl 节点，"
                      << "使用默认底层节点参数" << std::endl;
        }

        // 夹爪力矩/温度限幅读取（顶层 gripperTolerance 节点）
        if(config["gripperTolerance"]){
            const auto& gt = config["gripperTolerance"];

            mData.GripperLeft_Tor_Thr   = gt["l_gripper_tol"]["torque"].as<double>(10.0);
            mData.GripperLeft_Temp_Thr  = gt["l_gripper_tol"]["temperature"].as<double>(70.0);
            mData.GripperLeft_Kt = gt["l_gripper_tol"]["Kt"].as<double>(1.4);
            mData.GripperLeft_IqRange = gt["l_gripper_tol"]["IqRange"].as<std::vector<double>>(std::vector<double>{-30.0, 30.0});

            mData.GripperRight_Tor_Thr  = gt["r_gripper_tol"]["torque"].as<double>(10.0);
            mData.GripperRight_Temp_Thr = gt["r_gripper_tol"]["temperature"].as<double>(70.0);
            mData.GripperRight_Kt = gt["r_gripper_tol"]["Kt"].as<double>(1.4);
            mData.GripperRight_IqRange = gt["r_gripper_tol"]["IqRange"].as<std::vector<double>>(std::vector<double>{-30.0, 30.0});

        } else {
            std::cerr << "警告：未找到 gripperTolerance 节点，"
                      << "夹爪限幅使用默认值" << std::endl;
        }

        loaded_ = true;
        std::cout << "===============底层节点控制配置加载完毕===============" << std::endl;
        printConfig();
        return true;
    }
    catch(const YAML::BadFile& e){
        std::cerr << "错误：无法打开配置文件 " << yaml_path_ << std::endl;
        std::cerr << "  " << e.what() << std::endl;
        loaded_ = false;
        return false;
    }
    catch(const YAML::Exception& e){
        std::cerr << "错误：解析配置文件失败 " << yaml_path_ << std::endl;
        std::cerr << "  " << e.what() << std::endl;
        loaded_ = false;
        return false;
    }
}

// ===== getter =====

ctlInteConfig ServoNodeCtlConfig::getData() const{
    return mData;
}

bool ServoNodeCtlConfig::isLoaded() const{
    return loaded_;
}

// ===== 打印 =====

void ServoNodeCtlConfig::printConfig() const{
    std::cout << "========================================" << std::endl;
    std::cout << "  底层节点控制配置" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "  TempTime:            " << mData.TempTime          << " s"   << std::endl;
    std::cout << "  lArm_DOF:            " << mData.LARMDOF           << std::endl;
    std::cout << "  rArm_DOF:            " << mData.RARMDOF           << std::endl;
    std::cout << "  ZeroPos_Threshold:   " << mData.ZeroPos_Threshold << " rad" << std::endl;
    std::cout << "  netCard_name:        " << mData.NetCardName       << std::endl;
    std::cout << "  ---------- 夹爪力矩/温度限幅 ----------" << std::endl;
    std::cout << "  左夹爪 力矩限幅: " << mData.GripperLeft_Tor_Thr  << " Nm"  << std::endl;
    std::cout << "  左夹爪 温度限幅: " << mData.GripperLeft_Temp_Thr  << " ℃"  << std::endl;
    std::cout << "  右夹爪 力矩限幅: " << mData.GripperRight_Tor_Thr << " Nm"  << std::endl;
    std::cout << "  右夹爪 温度限幅: " << mData.GripperRight_Temp_Thr << " ℃"  << std::endl;
    std::cout << "========================================" << std::endl;
}

// ===== 封装：同步到全局变量 =====

bool loadServoNodeCtlFromYaml(const std::string& yaml_path){
    ServoNodeCtlConfig cfg;
    if(!cfg.loadFromYaml(yaml_path)){
        std::cerr << "警告：底层节点控制配置加载失败" << std::endl;
        return false;
    }

    ctl_inteMsg_config = cfg.getData();
    std::cout << "底层节点控制配置已同步到 ctl_inteMsg_config" << std::endl;
    return true;
}
