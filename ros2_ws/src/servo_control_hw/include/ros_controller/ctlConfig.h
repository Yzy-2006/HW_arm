#ifndef SERVO_CONTROL_HW__ROS_CONTROLLER__CTLCONFIG_H_
#define SERVO_CONTROL_HW__ROS_CONTROLLER__CTLCONFIG_H_

#include <iostream>
#include <string>
#include <yaml-cpp/yaml.h>

struct GripperControlConfig {
    int mode = 0;
    double kp = 0.0;
    double kd = 0.0;
    double pos_min_deg = -180.0;
    double pos_max_deg = 180.0;
    double zero_pos_deg = 0.0;
};

typedef struct{
    double TempTime;
    double LARMDOF;
    double RARMDOF;
    double ZeroPos_Threshold;
    std::string NetCardName;

    //----------夹爪力矩/温度限幅----------
    // 力矩限幅 (Nm) 与温度限幅 (℃)，左右夹爪各一组
    // 从 controllerConfig.yaml 的顶层 gripperTolerance 节点读取
    double GripperLeft_Tor_Thr;
    double GripperLeft_Temp_Thr;
    double GripperLeft_Kt;
    std::vector<double> GripperLeft_IqRange;

    double GripperRight_Tor_Thr;
    double GripperRight_Temp_Thr;
    double GripperRight_Kt;
    std::vector<double> GripperRight_IqRange;
    GripperControlConfig GripperLeft_Control;
    GripperControlConfig GripperRight_Control;
    double Gripper_TempRecover_Hys = 5.0;
}ctlInteConfig;

//数据接受全局变量
extern ctlInteConfig ctl_inteMsg_config;

//yaml文件路径
extern std::string controllerConfig_yamlPath;

/*  底层节点控制配置读取
---------------------------
从 controllerConfig.yaml 的 servo_node_loop_ctl 节点读取底层节点参数，
从顶层 gripperTolerance 节点读取夹爪力矩/温度限幅，
同步到全局变量 ctl_inteMsg_config
*/
class ServoNodeCtlConfig{
    private:
        ctlInteConfig mData;
        std::string   yaml_path_;
        bool          loaded_;

    public:
        ServoNodeCtlConfig();
        ServoNodeCtlConfig(const std::string& yaml_path);

        bool loadFromYaml(const std::string& yaml_path);

        ctlInteConfig getData() const;
        void          printConfig() const;
        bool          isLoaded() const;
};

// 封装函数：从 YAML 读取并同步到全局变量
bool loadServoNodeCtlFromYaml(const std::string& yaml_path);

#endif  // SERVO_CONTROL_HW__ROS_CONTROLLER__CTLCONFIG_H_
