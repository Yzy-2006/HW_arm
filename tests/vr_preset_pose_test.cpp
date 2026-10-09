#include "controller/xrTelepercationCom.h"
#include <cassert>
#include <Eigen/Geometry>

int LARMDOF = 7;
int RARMDOF = 7;
double TempTime = 0.02;
std::vector<double> lArm_zeroPos(7, 0.0), rArm_zeroPos(7, 0.0);
std::vector<double> l_descZeroPos(6, 0.0), r_descZeroPos(6, 0.0);

// Redirect only package discovery; exercise the real YAML loader and trajectory state machine.
std::string test_share;
namespace ament_index_cpp {
std::string get_package_share_directory(const std::string&) { return test_share; }
}

Eigen::Quaterniond pose_rotation(const std::vector<double>& pose){
    const auto q = eulerToQuaternionZYX({pose[5], pose[4], pose[3]});
    return Eigen::Quaterniond(q[0], q[1], q[2], q[3]).normalized();
}

void assert_same_rotation(const Eigen::Quaterniond& actual, const Eigen::Quaterniond& expected){
    assert(actual.angularDistance(expected) < 1e-9);
}

void check_tool_lock(bool enabled){
    for(const auto& rpy : std::vector<std::vector<double>>{
            {0.0, 0.0, 0.0}, {0.4, -0.7, 1.2}, {-0.5, 1.56, -0.8}}){
        l_descZeroPos = {0.2, 0.1, -0.3, rpy[0], rpy[1], rpy[2]};
        r_descZeroPos = {0.2, -0.1, -0.3, -rpy[0], -rpy[1], -rpy[2]};
        xr_telepercation_ctl ctl;
        ctl.init();
        auto cmd = ctl.trans_and_outp();
        for(int i = 0; i < 100; ++i){ cmd = ctl.trans_and_outp(); }
        Eigen::Quaterniond left = pose_rotation(cmd.ctlDesMsg.desPos_l_Descartes);
        Eigen::Quaterniond right = pose_rotation(cmd.ctlDesMsg.desPos_r_Descartes);
        const auto send = [&ctl](const Eigen::Quaterniond& q, bool left_side){
            const auto delta = quaternionToEulerZYX({q.w(), q.x(), q.y(), q.z()});
            if(left_side){ ctl.apply_left_orientation_delta(delta); }
            else{ ctl.apply_right_orientation_delta(delta); }
        };
        // Repeated pure spin stays blocked even with a tilted starting tool.
        for(int i = 0; i < 10; ++i){
            const Eigen::Quaterniond dl(Eigen::AngleAxisd(0.04, left * Eigen::Vector3d::UnitZ()));
            const Eigen::Quaterniond dr(Eigen::AngleAxisd(-0.04, right * Eigen::Vector3d::UnitZ()));
            send(dl, true);
            send(dr, false);
            if(!enabled){ left = dl * left; right = dr * right; }
            cmd = ctl.trans_and_outp();
            assert_same_rotation(pose_rotation(cmd.ctlDesMsg.desPos_l_Descartes), left);
            assert_same_rotation(pose_rotation(cmd.ctlDesMsg.desPos_r_Descartes), right);
        }
        // Mixed swing/twist keeps the intended pointing direction, then uses
        // the new tool axis for the next increment. Translation is unaffected.
        for(int i = 0; i < 10; ++i){
            const Eigen::Quaterniond sl(Eigen::AngleAxisd(0.03, left * Eigen::Vector3d::UnitX()));
            const Eigen::Quaterniond sr(Eigen::AngleAxisd(-0.03, right * Eigen::Vector3d::UnitY()));
            const Eigen::Quaterniond tl(Eigen::AngleAxisd(0.05, left * Eigen::Vector3d::UnitZ()));
            const Eigen::Quaterniond tr(Eigen::AngleAxisd(-0.05, right * Eigen::Vector3d::UnitZ()));
            send(sl * tl, true);
            send(sr * tr, false);
            left = (enabled ? sl : sl * tl) * left;
            right = (enabled ? sr : sr * tr) * right;
            ctl.l_arm_end_delta[0] += 0.001;
            cmd = ctl.trans_and_outp();
            assert_same_rotation(pose_rotation(cmd.ctlDesMsg.desPos_l_Descartes), left);
            assert_same_rotation(pose_rotation(cmd.ctlDesMsg.desPos_r_Descartes), right);
            assert(std::abs(cmd.ctlDesMsg.desPos_l_Descartes[0] - (0.2 + 0.001 * (i+1))) < 1e-9);
        }
        // A-key completion reanchors the axis from feedback, including tilt.
        ctl.trigger_joint_return();
        ctl.trans_and_outp();
        ctl.update_desc_ret(l_descZeroPos, r_descZeroPos);
        for(int i = 0; i < 400; ++i){ cmd = ctl.trans_and_outp(); }
        assert(cmd.ctl_transMod == 4);
        left = pose_rotation(cmd.ctlDesMsg.desPos_l_Descartes);
        const Eigen::Quaterniond twist(Eigen::AngleAxisd(0.04, left * Eigen::Vector3d::UnitZ()));
        send(twist, true);
        cmd = ctl.trans_and_outp();
        assert_same_rotation(pose_rotation(cmd.ctlDesMsg.desPos_l_Descartes), enabled ? left : twist * left);
    }
}

int main(int argc, char** argv){
    assert(argc == 3);
    test_share = argv[1];
    if(std::string(argv[2]) == "lock_enabled" || std::string(argv[2]) == "lock_disabled"){
        check_tool_lock(std::string(argv[2]) == "lock_enabled");
        return 0;
    }
    const bool no_feedback = std::string(argv[2]) == "valid_no_feedback";
    const bool valid = std::string(argv[2]) == "valid" || no_feedback;
    xr_telepercation_ctl ctl;
    ctl.init();
    ctl.trigger_preset_pose();
    if(valid){ assert(!ctl.accepts_vr_input()); }
    auto cmd = ctl.trans_and_outp();
    if(valid){
        assert(cmd.ctl_transMod == 1 && cmd.runinMod == 3 && cmd.ifUseRet == 1);
        for(int i = 0; i < 7; ++i){
            assert(std::abs(cmd.ctlDesMsg.desPos_l_JointSpace[i] - (0.1 + 0.01 * i)) < 1e-12);
            assert(std::abs(cmd.ctlDesMsg.desPos_r_JointSpace[i] + (0.1 + 0.01 * i)) < 1e-12);
        }
        const int cycles = static_cast<int>(std::ceil(cmd.TrajTime[0][0] / TempTime)) +
            cmd.m_stabMsg[0][0].start_stabTimes + cmd.m_stabMsg[0][0].des_stabTimes + 2;
        const std::vector<double> feedback = {0.2, 0.0, -0.3, 0.0, 0.0, 0.0};
        if(!no_feedback){ ctl.update_desc_ret(feedback, feedback); }
        for(int i = 0; i < cycles; ++i){
            ctl.l_arm_end_delta[0] += 0.01;
            auto held = ctl.trans_and_outp();
            assert(held.ctl_transMod == 1);
            assert(held.ctlDesMsg.desPos_l_JointSpace == cmd.ctlDesMsg.desPos_l_JointSpace);
        }
        // Simulate 20 seconds of VR traffic after completion, with either
        // stale feedback or none. The preset must never switch to Cartesian zero.
        for(int i = 0; i < 1000; ++i){
            ctl.l_arm_end_delta[0] += 0.01;
            ctl.r_arm_end_delta[1] -= 0.01;
            ctl.apply_left_orientation_delta({0.01, 0.02, 0.03});
            ctl.apply_right_orientation_delta({-0.01, -0.02, -0.03});
            const auto held = ctl.trans_and_outp();
            assert(!ctl.accepts_vr_input());
            assert(held.ctl_transMod == 1 && held.runinMod == 3);
            assert(held.ctlDesMsg.desPos_l_JointSpace == cmd.ctlDesMsg.desPos_l_JointSpace);
            assert(held.ctlDesMsg.desPos_r_JointSpace == cmd.ctlDesMsg.desPos_r_JointSpace);
            assert(ctl.l_arm_end_delta == std::vector<double>(3, 0.0));
            assert(ctl.r_arm_end_delta == std::vector<double>(3, 0.0));
        }
    }else{
        assert(cmd.ctl_transMod == 4);
    }
    ctl.trigger_joint_return();
    cmd = ctl.trans_and_outp();
    assert(cmd.ctl_transMod == 1);
    assert(cmd.ctlDesMsg.desPos_l_JointSpace == lArm_zeroPos);
    assert(cmd.ctlDesMsg.desPos_r_JointSpace == rArm_zeroPos);
    ctl.update_desc_ret(l_descZeroPos, r_descZeroPos);
    for(int i = 0; i < 400; ++i){ cmd = ctl.trans_and_outp(); }
    assert(cmd.ctl_transMod == 4 && ctl.accepts_vr_input());
    assert(cmd.ctlDesMsg.desPos_l_Descartes == l_descZeroPos);
    assert(cmd.ctlDesMsg.desPos_r_Descartes == r_descZeroPos);
}
