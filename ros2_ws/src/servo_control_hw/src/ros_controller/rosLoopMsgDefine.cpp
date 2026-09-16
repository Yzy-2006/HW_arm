#include "ros_controller/rosLoopMsgDefine.h"

#include <cstdlib>
#include <filesystem>
#include <ament_index_cpp/get_package_share_directory.hpp>

std::string netCard_name;

// 夹爪控制指令全局变量
gripperCmd left_gripper_cmd;
gripperCmd right_gripper_cmd;
gripperFdb left_gripper_fdb;
gripperFdb right_gripper_fdb;

std::string get_yaml_path(){
    // 环境变量优先
    if(const char* env_p = std::getenv("SERVO_YAML_PATH")){
        std::filesystem::path p(env_p);
        if(std::filesystem::exists(p)) return p.string();
    }

    // 使用 ament index 查找包 share 目录
    try {
        std::string pkg_share = ament_index_cpp::get_package_share_directory("servo_control_hw");
        std::filesystem::path p = pkg_share;
        p /= "yamlpkg/can_config.yaml";
        if(std::filesystem::exists(p)) return p.string();
    } catch(const std::exception &e){
        // 忽略，继续回退
    }

    // 回退到工作区相对路径（在开发环境中常用）
    std::filesystem::path p2 = std::filesystem::current_path();
    p2 /= "src/servo_control_hw/yamlpkg/can_config.yaml";
    return p2.string();
}

servo_struct load_servo_config(const std::string& filepath){
    servo_struct servo_config;

    std::cout << "=======================================" << std::endl;
    std::cout << "开始读取从站配置文件，并基于此初始化从站" << std::endl;
    std::cout << "配置文件路径: " << filepath << std::endl;

    //加载YAML文件
    YAML::Node config = YAML::LoadFile(filepath);

    //读取基本配置
    if(config["ecat_8can"]) {
        YAML::Node ecat = config["ecat_8can"];

        servo_config.CAN_boards.clear();

        //读取每个CAN通道的配置
        if (ecat["channels"] && ecat["channels"].IsSequence()) {
            for (const auto& ch_node : ecat["channels"]) {
                can_slave_struct board;

                board.channel_idx = ch_node["channel"].as<int>();
                board.can_num_activate = 1;  // one channel per entry

                std::cout << "===============================" << std::endl;
                std::cout << "CAN通道: " << board.channel_idx << std::endl;

                int motor_count = 0;
                if (ch_node["motors"] && ch_node["motors"].IsSequence()) {
                    for (const auto& m_node : ch_node["motors"]) {
                        int slot = m_node["slot"].as<int>();
                        int motor_can_id = m_node["motor_can_id"].as<int>();
                        double kt = m_node["kt"].as<double>();
                        int dir = m_node["direction"].as<int>();

                        board.motor_slot.push_back(slot);
                        board.can_jnt_ID.push_back(motor_can_id);
                        board.can_jnt_Kt.push_back(kt);
                        board.can_jnt_direComp.push_back(dir);

                        motor_count++;
                        std::cout << "  slot=" << slot << " motor_id=" << motor_can_id
                                  << " kt=" << kt << " dir=" << dir << std::endl;
                    }
                }

                board.can_slave_num.push_back(motor_count);
                board.JntNum_onCan = motor_count;
                board.can_id.push_back(board.channel_idx);

                //初始化关节目标信息和反馈信息
                board.can_jnt_des_msg.resize(motor_count);
                board.can_jnt_ret_msg.resize(motor_count);
                for(int i = 0; i < motor_count; i++){
                    board.can_jnt_des_msg[i].transMod = 1;
                    board.can_jnt_des_msg[i].joint_pos = 0.0;
                    board.can_jnt_des_msg[i].joint_spd = 0.0;
                    board.can_jnt_des_msg[i].joint_tor = 0.0;
                    board.can_jnt_des_msg[i].joint_Kp = 0.0;
                    board.can_jnt_des_msg[i].joint_Kd = 0.0;

                    board.can_jnt_ret_msg[i].joint_posRet = 0.0;
                    board.can_jnt_ret_msg[i].joint_spdRet = 0.0;
                    board.can_jnt_ret_msg[i].joint_currentRet = 0.0;
                }

                servo_config.CAN_boards.push_back(board);
                std::cout << "===============================" << std::endl;
            }
        }

        servo_config.CAN_board_num = servo_config.CAN_boards.size();
        std::cout << "ECAT-8CAN 通道总数: " << servo_config.CAN_board_num << std::endl;
    }else{
        throw std::runtime_error("YAML文件中缺少 'ecat_8can' 根节点");
    }

    return servo_config;
}