#ifndef __ImpedanceAdmittanceController__H__
#define __ImpedanceAdmittanceController__H__

#include <vector>

#include <Eigen/Dense>
#include <kdl/frames.hpp>
#include <kdl/frames_io.hpp>

#include "robot_config/msgDefine.h"

/*
        by 霜云喵
================================
双臂末端笛卡尔空间阻抗-导纳控制核心类

本代码包为双臂末端六维笛卡尔空间的阻抗控制与导纳控制封装。
阻抗控制：由运动偏差（位姿/速度/加速度误差）计算末端期望力/力矩；
导纳控制：由末端外力/力矩计算柔顺的期望运动（位姿/速度/加速度）。
二者的阻抗/导纳参数（刚度 K、阻尼 B、惯性 M）均作为输入量，
由上层按需在线调节（例如实现变刚度、自适应柔顺等）。

============修改日志============

==============协议==============
末端位姿采用六维向量描述：[x, y, z, roll, pitch, yaw]（平移 + RPY 欧拉角）
末端力/力矩采用六维向量描述：[Fx, Fy, Fz, Mx, My, Mz]

臂杆顺序：
    0 -- 左臂
    1 -- 右臂
控制时间单位为秒(s)，与全局 TEMPTIME 保持一致。

旋转误差采用 RPY 小角度近似（与 Frame_to_vector / vector_to_Frame 约定一致），
适用于末端姿态偏离较小的柔顺/力控场景。
================================
*/

// 六维笛卡尔空间向量与矩阵类型别名（位姿 / 速度 / 力均适用）
using Vec6 = Eigen::Matrix<double, 6, 1>;
using Mat6 = Eigen::Matrix<double, 6, 6>;

//--------------------------------------------------------------------------------
//阻抗 / 导纳参数（刚度 K、阻尼 B、惯性 M 均为 6x6 矩阵）
struct CartImpAdmParams {
    Mat6 K;   // 刚度矩阵 (N/m, N·m/rad)
    Mat6 B;   // 阻尼矩阵 (N·s/m, N·m·s/rad)
    Mat6 M;   // 惯性矩阵 (kg, kg·m²)

    CartImpAdmParams() {
        K = Mat6::Zero();
        B = Mat6::Zero();
        M = Mat6::Zero();
    }

    // 便捷构造：以六维对角元素生成对角矩阵
    static CartImpAdmParams Diagonal(const Vec6& k, const Vec6& b, const Vec6& m) {
        CartImpAdmParams p;
        p.K = k.asDiagonal();
        p.B = b.asDiagonal();
        p.M = m.asDiagonal();
        return p;
    }

    // 便捷构造：以 std::vector<double>（六维）生成对角矩阵
    static CartImpAdmParams Diagonal(const std::vector<double>& k,
                                     const std::vector<double>& b,
                                     const std::vector<double>& m);
};

//--------------------------------------------------------------------------------
//笛卡尔运动状态（位姿 + 速度 + 加速度）
struct CartMotionState {
    Vec6 pos;   // 位姿 [x, y, z, roll, pitch, yaw]
    Vec6 vel;   // 速度（位姿一阶导）
    Vec6 acc;   // 加速度（位姿二阶导）

    CartMotionState() {
        pos = Vec6::Zero();
        vel = Vec6::Zero();
        acc = Vec6::Zero();
    }
};

//--------------------------------------------------------------------------------
//类型转换辅助函数

// KDL::Frame -> [x, y, z, roll, pitch, yaw]
Vec6 frameToPoseVec(const KDL::Frame& frame);
// [x, y, z, roll, pitch, yaw] -> KDL::Frame
KDL::Frame poseVecToFrame(const Vec6& pose);
// std::vector<double>（六维）-> Vec6（不足补零，超出截断）
Vec6 stdVecToVec6(const std::vector<double>& v);
// Vec6 -> std::vector<double>（六维）
std::vector<double> vec6ToStdVec(const Vec6& v);
// 将六维力/力矩向量 [Fx,Fy,Fz,Mx,My,Mz] 用旋转矩阵 R 旋转（力与力矩分量同乘 R）。
// 用于把传感器坐标系下测得的外力/力矩变换到导纳控制所采用的坐标系。
Vec6 rotateWrench(const Vec6& wrench, const KDL::Rotation& R);

//--------------------------------------------------------------------------------
//双臂末端笛卡尔空间阻抗-导纳控制核心类
class DualArmImpAdmCtl {
private:
    double control_T_;      // 控制周期 (s)

    // 导纳控制积分状态（左/右臂柔顺轨迹的位姿与速度）
    Vec6 adm_pos_l_, adm_vel_l_;
    Vec6 adm_pos_r_, adm_vel_r_;
    bool adm_initialized_;  // 导纳积分状态是否已初始化

    // 求解惯性矩阵的逆作用：a = M^{-1} * rhs（M 全零时返回零向量，即忽略惯性项）
    Vec6 solveInertia(const Mat6& M, const Vec6& rhs) const;
public:
    DualArmImpAdmCtl();

    // 初始化控制周期 (s)
    void init(double control_T);

    /*  接口函数 1：阻抗控制
    --------------------------------
    输入量：
        lDes / rDes -- 左右臂期望运动状态（位姿/速度/加速度）
        lAct / rAct -- 左右臂实际运动状态（位姿/速度/加速度）
        lParam / rParam -- 左右臂阻抗参数（K、B、M）
    输出量：
        左右臂末端期望六维力/力矩 [Fx,Fy,Fz,Mx,My,Mz]
    控制律：
        F = M*(xdd_d - xdd) + B*(xd_d - xd) + K*(x_d - x)
    说明：
        输出为末端力/力矩，需由上层调用 EndTor_To_Jnt 映射到关节力矩，
        再叠加重力/摩擦补偿后下发给底层关节控制。
        若不需要惯性项，可将阻抗参数 M 置零。
    */
    RobotFrameTor impedanceControl(
        const CartMotionState& lDes, const CartMotionState& rDes,
        const CartMotionState& lAct, const CartMotionState& rAct,
        const CartImpAdmParams& lParam, const CartImpAdmParams& rParam);

    /*  接口函数 2：导纳控制
    --------------------------------
    输入量：
        extWrench -- 外界作用于末端的外力/力矩（如六维力传感器测得）
        lRef / rRef -- 左右臂参考轨迹（无外力时的标称轨迹）
        lParam / rParam -- 左右臂导纳参数（K、B、M）
    输出量：
        lOut / rOut -- 左右臂柔顺后的期望运动状态（位姿/速度/加速度），
                       供上层位置环 / 逆解（FrameTrack）跟踪
    控制律：
        F_ext = M*(xdd - xdd_ref) + B*(xd - xd_ref) + K*(x - x_ref)
    说明：
        采用半隐式欧拉积分；旋转误差为 RPY 小角度近似。
        首次调用以参考轨迹初始化积分状态；切换模式或期望轨迹突变时，
        建议调用 resetAdmittance 重新以当前实际末端状态为起点。
    */
    void admittanceControl(
        const RobotFrameTor& extWrench,
        const CartMotionState& lRef, const CartMotionState& rRef,
        const CartImpAdmParams& lParam, const CartImpAdmParams& rParam,
        CartMotionState& lOut, CartMotionState& rOut);

    // 重置导纳积分状态（以当前实际末端状态为起点，避免刚切换时的输出突变）
    void resetAdmittance(const CartMotionState& lAct, const CartMotionState& rAct);
};

#endif
