#include "controller/ImpedanceAdmittanceController.h"

//--------------------------------------------------------------------------------
//类型转换辅助函数

Vec6 frameToPoseVec(const KDL::Frame& frame) {
    Vec6 pose = Vec6::Zero();
    for (int i = 0; i < 3; ++i) {
        pose[i] = frame.p[i];
    }
    double roll = 0.0, pitch = 0.0, yaw = 0.0;
    frame.M.GetRPY(roll, pitch, yaw);
    pose[3] = roll;
    pose[4] = pitch;
    pose[5] = yaw;
    return pose;
}

KDL::Frame poseVecToFrame(const Vec6& pose) {
    KDL::Frame frame;
    frame.p = KDL::Vector(pose[0], pose[1], pose[2]);
    frame.M = KDL::Rotation::RPY(pose[3], pose[4], pose[5]);
    return frame;
}

Vec6 stdVecToVec6(const std::vector<double>& v) {
    Vec6 r = Vec6::Zero();
    const int n = static_cast<int>(v.size());
    for (int i = 0; i < n && i < 6; ++i) {
        r[i] = v[i];
    }
    return r;
}

std::vector<double> vec6ToStdVec(const Vec6& v) {
    std::vector<double> r(6, 0.0);
    for (int i = 0; i < 6; ++i) {
        r[i] = v[i];
    }
    return r;
}

Vec6 rotateWrench(const Vec6& wrench, const KDL::Rotation& R) {
    // 力与力矩在坐标系旋转下同为矢量，均左乘旋转矩阵 R（传感器系 -> 目标系）。
    const KDL::Vector F(wrench[0], wrench[1], wrench[2]);
    const KDL::Vector M(wrench[3], wrench[4], wrench[5]);
    const KDL::Vector Fb = R * F;
    const KDL::Vector Mb = R * M;
    Vec6 out = Vec6::Zero();
    out[0] = Fb[0]; out[1] = Fb[1]; out[2] = Fb[2];
    out[3] = Mb[0]; out[4] = Mb[1]; out[5] = Mb[2];
    return out;
}

//--------------------------------------------------------------------------------
//参数便捷构造

CartImpAdmParams CartImpAdmParams::Diagonal(const std::vector<double>& k,
                                            const std::vector<double>& b,
                                            const std::vector<double>& m) {
    return Diagonal(stdVecToVec6(k), stdVecToVec6(b), stdVecToVec6(m));
}

//--------------------------------------------------------------------------------
//核心类实现

DualArmImpAdmCtl::DualArmImpAdmCtl()
    : control_T_(0.001),
      adm_pos_l_(Vec6::Zero()),
      adm_vel_l_(Vec6::Zero()),
      adm_pos_r_(Vec6::Zero()),
      adm_vel_r_(Vec6::Zero()),
      adm_initialized_(false) {}

void DualArmImpAdmCtl::init(double control_T) {
    control_T_ = control_T;
    adm_initialized_ = false;
}

Vec6 DualArmImpAdmCtl::solveInertia(const Mat6& M, const Vec6& rhs) const {
    // M 全零：忽略惯性项，直接返回零（即 a = 0）
    if (M.isZero(1e-12)) {
        return Vec6::Zero();
    }
    // 采用带列主元的 QR 分解求解，对半正定/奇异的对角惯性矩阵同样稳健
    return M.colPivHouseholderQr().solve(rhs);
}

RobotFrameTor DualArmImpAdmCtl::impedanceControl(
    const CartMotionState& lDes, const CartMotionState& rDes,
    const CartMotionState& lAct, const CartMotionState& rAct,
    const CartImpAdmParams& lParam, const CartImpAdmParams& rParam) {

    // 单臂阻抗控制律：F = M*e_acc + B*e_vel + K*e_pos
    auto singleArm = [](const CartMotionState& des, const CartMotionState& act,
                        const CartImpAdmParams& param) -> Vec6 {
        const Vec6 e_pos = des.pos - act.pos;
        const Vec6 e_vel = des.vel - act.vel;
        const Vec6 e_acc = des.acc - act.acc;
        return param.K * e_pos + param.B * e_vel + param.M * e_acc;
    };

    const Vec6 wrench_l = singleArm(lDes, lAct, lParam);
    const Vec6 wrench_r = singleArm(rDes, rAct, rParam);

    RobotFrameTor out;
    out.lArm_EndTor = vec6ToStdVec(wrench_l);
    out.rArm_EndTor = vec6ToStdVec(wrench_r);
    return out;
}

void DualArmImpAdmCtl::admittanceControl(
    const RobotFrameTor& extWrench,
    const CartMotionState& lRef, const CartMotionState& rRef,
    const CartImpAdmParams& lParam, const CartImpAdmParams& rParam,
    CartMotionState& lOut, CartMotionState& rOut) {

    const Vec6 F_l = stdVecToVec6(extWrench.lArm_EndTor);
    const Vec6 F_r = stdVecToVec6(extWrench.rArm_EndTor);

    // 首次调用：以参考轨迹初始化积分状态
    if (!adm_initialized_) {
        adm_pos_l_ = lRef.pos;
        adm_vel_l_ = lRef.vel;
        adm_pos_r_ = rRef.pos;
        adm_vel_r_ = rRef.vel;
        adm_initialized_ = true;
    }

    // 单臂导纳控制步：
    //   M*(xdd - xdd_ref) = -F_ext - B*(xd - xd_ref) - K*(x - x_ref)
    //   xdd = xdd_ref + M^{-1} * rhs
    //   随后半隐式欧拉积分更新 xd、x
    auto step = [this](const Vec6& F, const CartMotionState& ref,
                       const CartImpAdmParams& param,
                       Vec6& pos, Vec6& vel) -> CartMotionState {
        const Vec6 e_pos = pos - ref.pos;
        const Vec6 e_vel = vel - ref.vel;
        const Vec6 rhs = -F - param.B * e_vel - param.K * e_pos;
        const Vec6 acc = ref.acc + solveInertia(param.M, rhs);

        vel = vel + acc * control_T_;
        pos = pos + vel * control_T_;

        CartMotionState out;
        out.pos = pos;
        out.vel = vel;
        out.acc = acc;
        return out;
    };

    lOut = step(F_l, lRef, lParam, adm_pos_l_, adm_vel_l_);
    rOut = step(F_r, rRef, rParam, adm_pos_r_, adm_vel_r_);
}

void DualArmImpAdmCtl::resetAdmittance(const CartMotionState& lAct,
                                       const CartMotionState& rAct) {
    adm_pos_l_ = lAct.pos;
    adm_vel_l_ = lAct.vel;
    adm_pos_r_ = rAct.pos;
    adm_vel_r_ = rAct.vel;
    adm_initialized_ = true;
}
