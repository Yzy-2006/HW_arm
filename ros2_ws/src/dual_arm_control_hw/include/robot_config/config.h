#ifndef __config__H__
#define __config__H__

#include <iostream>
#include <vector>
#include <string>
#include <yaml-cpp/yaml.h>
#include <iomanip>

#include <pinocchio/multibody/fwd.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <Eigen/Dense>

#include "robot_config/msgDefine.h"

/*
        by 霜云喵
================================
配置代码，配置零位和PD参数
============修改日志============

==============协议==============

================================
*/


extern std::string yaml_path_zeroPos;
extern std::string yaml_path_desc_zeroPos;
extern std::string yaml_path_initialPD;
extern std::string yaml_path_jointLink;
extern std::string yaml_path_robotStructConfig;
extern std::string yaml_path_gravCompConfig;
extern std::string yaml_path_impAdm;

//默认位置定义
extern std::vector<double> lArm_zeroPos;
extern std::vector<double> rArm_zeroPos;

extern std::vector<double> l_descZeroPos;
extern std::vector<double> r_descZeroPos;

//默认PD参数定义
extern std::vector<double> lArm_defaultKp_low;
extern std::vector<double> lArm_defaultKd_low;
extern std::vector<double> rArm_defaultKp_low;
extern std::vector<double> rArm_defaultKd_low;

extern std::vector<double> lArm_defaultKp_mid;
extern std::vector<double> lArm_defaultKd_mid;
extern std::vector<double> rArm_defaultKp_mid;
extern std::vector<double> rArm_defaultKd_mid;

extern std::vector<double> lArm_defaultKp_high;
extern std::vector<double> lArm_defaultKd_high;
extern std::vector<double> rArm_defaultKp_high;
extern std::vector<double> rArm_defaultKd_high;

extern std::vector<double> lArm_defaultKp_superhigh;
extern std::vector<double> lArm_defaultKd_superhigh;
extern std::vector<double> rArm_defaultKp_superhigh;
extern std::vector<double> rArm_defaultKd_superhigh;

extern std::vector<double> lArm_defaultKp_tag;
extern std::vector<double> lArm_defaultKd_tag;
extern std::vector<double> rArm_defaultKp_tag;
extern std::vector<double> rArm_defaultKd_tag;

//默认机器人关节连杆体系配置参数
extern RobotJntMsg robotJointLinkConfig;

// 机器人结构参数配置全局变量 —— 由 loadRobotConfigFromYaml 填充
extern RobotStructure robotModelStructConfig;
extern pinocchio::Model robotProcessedModel;

//默认机器人重力补偿配置参数
extern gravCompInte robotGravCompInte;

//基于优化器的逆解求解器参数
extern optimizerCtl optimizerControParam;
extern optiPrintMsgCtl msgPringControlParam;

//总控流程参数集合
extern TransLoopCtl dataTransLoopCtl;

//阻抗参数
extern ImpAdmPara imp_adm_ParaPak;


/*  机械臂零位配置读取类
---------------------------
功能：
    1. 从 zeropos.yaml 读取双臂零位关节角度
    2. 提供获取左/右臂零位向量的接口
    3. 打印默认位置到控制台

协议：
    零位定义为双臂竖直下垂时的关节位置，
    角度单位为弧度 (rad)，关节顺序与 Unitree_name_list 一致
*/
class ZeroPosConfig{
    private:
        // 零位位置数据
        std::vector<double> lArm_zeroPos;               // 左臂零位 (7 关节)
        std::vector<double> rArm_zeroPos;               // 右臂零位 (7 关节)

        // YAML 文件路径
        std::string yaml_path_;

        // 加载状态标志
        bool loaded_;

    public:
        // 构造函数
        ZeroPosConfig();
        ZeroPosConfig(const std::string& yaml_path);

        // 从 YAML 文件加载零位配置
        // 返回 true 表示加载成功
        bool loadFromYaml(const std::string& yaml_path);

        // 获取零位位置
        std::vector<double> getLeftArmZeroPos()  const;
        std::vector<double> getRightArmZeroPos() const;

        // 打印默认位置到控制台
        void printDefaultPositions() const;

        // 检查是否已成功加载
        bool isLoaded() const;
};

/*  笛卡尔空间零位配置读取类
--------------------------------
功能：
    1. 从 desc_zeropos.yaml 读取双臂笛卡尔空间零位
    2. 提供获取左/右臂末端位姿向量的接口
    3. 打印默认位置到控制台

协议：
    位姿格式：[x, y, z, roll, pitch, yaw]
    - x, y, z  — 末端位置 (m)
    - roll, pitch, yaw — RPY 欧拉角 (rad)
*/
class DescZeroPosConfig{
    private:
        // 笛卡尔零位数据 [x,y,z,roll,pitch,yaw]
        std::vector<double> lArm_descPos;           // 左臂末端零位 (6 维)
        std::vector<double> rArm_descPos;           // 右臂末端零位 (6 维)

        // YAML 文件路径
        std::string yaml_path_;

        // 加载状态标志
        bool loaded_;

    public:
        // 构造函数
        DescZeroPosConfig();
        DescZeroPosConfig(const std::string& yaml_path);

        // 从 YAML 文件加载笛卡尔零位配置
        bool loadFromYaml(const std::string& yaml_path);

        // 获取笛卡尔零位
        std::vector<double> getLeftArmDescZeroPos()  const;
        std::vector<double> getRightArmDescZeroPos() const;

        // 打印默认位置到控制台
        void printDefaultPositions() const;

        // 检查是否已成功加载
        bool isLoaded() const;
};


/*  机械臂 PD 参数读取类
---------------------------
功能：
    1. 从 initialPD.yaml 读取双臂 PD 控制器增益参数
    2. 通过刚度等级控制变量（low/mid/high/superhigh/tag）选择对应参数组
    3. 提供获取左/右臂 Kp、Kd 向量的接口
    4. 打印当前 PD 配置到控制台

协议：
    关节顺序与 Unitree_name_list 一致（7 关节），
    Kp 为比例增益，Kd 为微分增益
*/
class InitialPDConfig{
    private:
        // PD 参数数据
        std::vector<double> lArm_Kp;                // 左臂比例增益 (7 关节)
        std::vector<double> lArm_Kd;                // 左臂微分增益 (7 关节)
        std::vector<double> rArm_Kp;                // 右臂比例增益 (7 关节)
        std::vector<double> rArm_Kd;                // 右臂微分增益 (7 关节)

        // YAML 文件路径
        std::string yaml_path_;

        // 当前刚度等级
        std::string rigidity_level_;

        // 加载状态标志
        bool loaded_;

    public:
        // 构造函数
        InitialPDConfig();
        InitialPDConfig(const std::string& yaml_path,
                        const std::string& rigidity_level);

        // 从 YAML 文件加载指定刚度等级的 PD 配置
        // rigidity_level: "low" / "mid" / "high" / "superhigh" / "tag"
        // 返回 true 表示加载成功
        bool loadFromYaml(const std::string& yaml_path,
                          const std::string& rigidity_level);

        // 获取 PD 参数
        std::vector<double> getLeftArmKp()  const;
        std::vector<double> getLeftArmKd()  const;
        std::vector<double> getRightArmKp() const;
        std::vector<double> getRightArmKd() const;

        // 获取当前刚度等级
        std::string getRigidityLevel() const;

        // 打印当前 PD 配置到控制台
        void printPDConfig() const;

        // 检查是否已成功加载
        bool isLoaded() const;
};

/*  关节连杆作动参数
---------------------------
功能：
    1. 从 jointLink_config.yaml 读取左右臂关节-连杆结构信息
    2. 提供 RobotStructure 所需的 frame_names / rotation_axis / jointNum 数据
    3. 打印关节连杆结构到控制台

协议：
    连杆名、旋转轴信息由 URDF 提取，关节数目分别为 7
*/
class JointLinkConfig{
    private:
        // 左臂关节-连杆结构
        std::vector<std::string> lArm_frame_names;
        std::vector<std::string> lArm_JntRotation_axis;
        int lArm_jointNum;

        // 右臂关节-连杆结构
        std::vector<std::string> rArm_frame_names;
        std::vector<std::string> rArm_JntRotation_axis;
        int rArm_jointNum;

        // YAML 文件路径
        std::string yaml_path_;

        // 加载状态标志
        bool loaded_;

    public:
        // 构造函数
        JointLinkConfig();
        JointLinkConfig(const std::string& yaml_path);

        // 从 YAML 文件加载关节连杆配置
        bool loadFromYaml(const std::string& yaml_path);

        // 获取关节连杆结构数据
        std::vector<std::string> getLeftArmFrameNames()   const;
        std::vector<std::string> getLeftArmRotationAxis() const;
        int                     getLeftArmJointNum()      const;

        std::vector<std::string> getRightArmFrameNames()   const;
        std::vector<std::string> getRightArmRotationAxis() const;
        int                     getRightArmJointNum()      const;

        // 打印关节连杆结构到控制台
        void printJointLinkConfig() const;

        // 检查是否已成功加载
        bool isLoaded() const;
};

/*  机器人配置文件读取
---------------------------
功能：
    1. 融合.yaml文件信息，选取机器人
*/

class loadRobotCofig{
    //读取机器人配置文件的代码
    private:
        //机器人模型
        pinocchio::Model whole_model_;
        pinocchio::Model processed_model;

        //机器人结构体配置
        RobotStructure robotStructConfig;

        //读取.yaml文件，获取机器人配置
        void yamlRead(std::string yaml_filePath);

    public:
        //读取.yaml文件，并依据配置文件内容构建模型
        void getModelFromYaml(std::string yaml_filePath, std::string urdf_filepath);

        //输出处理后的pinocchio模型
        pinocchio::Model processed_model_outp(){
            return processed_model;
        }

        //输出机器人结构体配置
        RobotStructure getRobotStructConfig(){
            return robotStructConfig;
        }

};

/*  重力补偿配置文件读取
---------------------------
功能：
    1. 从 controllerConfig.yaml 读取双臂重力补偿相关参数
    2. 提供获取 gravCompInte 结构体的接口
    3. 打印配置摘要到控制台

协议：
    补偿修正系数顺序：左臂J0~J6, 右臂J0~J6 (14维)
    滤波器参数按关节顺序排列 (7维)
*/
class GravCompConfig{
    private:
        // 重力补偿参数数据
        gravCompInte mGravCompData;

        // YAML文件路径
        std::string yaml_path_;

        // 加载状态标志
        bool loaded_;

        // ---- 辅助读取函数 ----

        // 从 YAML node 读取 single-arm 滤波器组 (7个lowpassFtrInte)
        std::vector<lowpassFtrInte> readFilterGroup(
            const YAML::Node& node,
            const std::string& sampRate_key,
            const std::string& cutoffRate_key);

        // 读取双臂滤波器 (posInp / velInp / torInp / torOutp)
        void readGravCompFtr(const YAML::Node& ftr_root);

        // 读取加速度卡尔曼滤波器参数 (14维, Qgain/Rgain)
        void readAccFtr(const YAML::Node& accFtr_node);

    public:
        // 构造函数
        GravCompConfig();
        GravCompConfig(const std::string& yaml_path);

        // 从 YAML 文件加载重力补偿配置
        bool loadFromYaml(const std::string& yaml_path);

        // 获取重力补偿数据
        gravCompInte getGravCompData() const;

        // 打印配置摘要
        void printConfig() const;

        // 检查是否已成功加载
        bool isLoaded() const;
};

/*  求解器控制配置文件读取
---------------------------
功能：
    1. 从 controllerConfig.yaml 读取求解器控制相关参数
    2. 封装到 optimizerCtl 和 optiPrintMsgCtl 类型的变量中并输出
    3. 打印配置摘要到控制台

协议：
    优化器权重共 5 项：[位置, 旋转, 正则化, 平滑, 速度平滑]
    开关量采用 0/1 整数表示
*/
class solverControlParam{
    private:
        // 优化器控制参数
        optimizerCtl    mOptiCtlData;

        // 信息打印控制参数
        optiPrintMsgCtl mPrintMsgData;

        // YAML 文件路径
        std::string yaml_path_;

        // 加载状态标志
        bool loaded_;

    public:
        // 构造函数
        solverControlParam();
        solverControlParam(const std::string& yaml_path);

        // 从 YAML 文件加载求解器控制配置
        // 返回 true 表示加载成功
        bool loadFromYaml(const std::string& yaml_path);

        // 获取优化器控制参数
        optimizerCtl    getOptimizerCtl()    const;

        // 获取信息打印控制参数
        optiPrintMsgCtl getPrintMsgCtl()     const;

        // 打印配置摘要
        void printConfig() const;

        // 检查是否已成功加载
        bool isLoaded() const;
};

/*  数据传输流程控制配置文件读取
---------------------------
功能：
    1. 从 controllerConfig.yaml 读取总控流程相关参数
    2. 封装到 TransLoopCtl 类型的变量中并输出
    3. 打印配置摘要到控制台

协议：
    卡尔曼滤波器参数按关节顺序排列 (7维)
*/
class TransLoopCtlConfig{
    private:
        // 总控流程参数数据
        TransLoopCtl mTransLoopCtlData;

        // YAML 文件路径
        std::string yaml_path_;

        // 加载状态标志
        bool loaded_;

        // ---- 辅助读取函数 ----

        // 从 YAML 的 Qgain / Rgain 向量读取单臂卡尔曼滤波器组 (7个 kalmanFtrInte)
        std::vector<kalmanFtrInte> readKalmanFilterGroup(
            const YAML::Node& node,
            const std::string& Qgain_key,
            const std::string& Rgain_key);

    public:
        // 构造函数
        TransLoopCtlConfig();
        TransLoopCtlConfig(const std::string& yaml_path);

        // 从 YAML 文件加载总控流程配置
        // 返回 true 表示加载成功
        bool loadFromYaml(const std::string& yaml_path);

        // 获取总控流程数据
        TransLoopCtl getTransLoopCtlData() const;

        // 打印配置摘要
        void printConfig() const;

        // 检查是否已成功加载
        bool isLoaded() const;
};

/*  阻抗-导纳控制参数配置读取
---------------------------
功能：
    1. 从 admittanceImpPara.yaml 读取双臂末端笛卡尔空间阻抗/导纳控制参数
    2. 封装到 ImpAdmPara 结构体中并输出
    3. 打印配置摘要到控制台

协议：
    imp_adm_switch : 0 -- 不开启；1 -- 阻抗控制；2 -- 导纳控制
    阻抗/导纳参数 K、B、M 均为 6x6 矩阵，行/列顺序为 [x, y, z, roll, pitch, yaw]
    注意：笛卡尔空间阻抗/导纳控制尚未完成，本类仅负责参数的读取与存储，
    控制核心类 DualArmImpAdmCtl 尚未实例化接入控制回路
*/
class ImpAdmConfig{
    private:
        // 阻抗-导纳参数数据
        ImpAdmPara mImpAdmData;

        // YAML 文件路径
        std::string yaml_path_;

        // 加载状态标志
        bool loaded_;

        // ---- 辅助函数 ----

        // 将全部参数复位到默认值（开关关闭、矩阵全零）
        void initDefaults();

        // 从 YAML node 读取 6x6 矩阵（按行存储），非法/缺失时返回 false 并置零
        bool readMat6(const YAML::Node& node,
                      Eigen::Matrix<double, 6, 6>& mat);

        // 从 YAML node 读取 6 维 double 向量（对应六维力/力矩），非法/缺失时返回 false 并清空
        bool readVec6(const YAML::Node& node,
                      std::vector<double>& vec);

    public:
        // 构造函数
        ImpAdmConfig();
        ImpAdmConfig(const std::string& yaml_path);

        // 从 YAML 文件加载阻抗-导纳控制配置
        bool loadFromYaml(const std::string& yaml_path);

        // 获取阻抗-导纳控制数据
        ImpAdmPara getImpAdmData() const;

        // 打印配置摘要
        void printConfig() const;

        // 检查是否已成功加载
        bool isLoaded() const;
};

// --------------- 封装读取函数 ---------------

//注意关节空间与笛卡尔空间零位的匹配

// ----- 关节层零位配置 -----
//将零位角度配置读取到全局变量中
bool loadZeroPosFromYaml(const std::string& yaml_path);

// ----- 笛卡尔零位配置 -----
//将零位笛卡尔空间配置读取到全局变量中
bool loadDescZeroPosFromYaml(const std::string& yaml_path);

// ----- 默认PD参数配置 -----
//加载 PD 配置文件 -- 从 initialPD.yaml 读取 PD 参数并填入全局变量
bool loadInitialPDFromYaml(const std::string& yaml_path);

// ----- 关节连杆结构配置 -----
//加载关节连杆结构配置文件 -- 从 jointLink_config.yaml 读取并填入全局变量
bool loadJointLinkFromYaml(const std::string& yaml_path);

// ----- 机器人模型结构配置 -----
// 从 robot_config.yaml + .urdf 构建机器人结构配置与pinocchio模型
// 同时填充全局变量 robotModelStructConfig 与 robotProcessedModel
bool loadRobotConfigFromYaml(const std::string& yaml_filePath, const std::string& urdf_filepath);

// ----- 重力补偿参数配置 -----
// 从 controllerConfig.yaml 读取重力补偿参数并填入全局变量 robotGravCompInte
bool loadGravCompFromYaml(const std::string& yaml_path);

// ----- 求解器控制参数配置 -----
// 从 controllerConfig.yaml 读取求解器控制参数并填入全局变量
// optimizerControParam 与 msgPringControlParam
bool loadSolverControlFromYaml(const std::string& yaml_path);

// ----- 总控流程参数配置 -----
// 从 controllerConfig.yaml 读取总控流程参数并填入全局变量 dataTransLoopCtl
bool loadTransLoopCtlFromYaml(const std::string& yaml_path);

// ----- 阻抗-导纳控制参数配置 -----
// 从 admittanceImpPara.yaml 读取阻抗-导纳控制参数并填入全局变量 imp_adm_ParaPak
bool loadImpAdmFromYaml(const std::string& yaml_path);


#endif