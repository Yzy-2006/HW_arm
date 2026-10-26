#include "controller/jointStreamGuard.h"
#include "controller/upper_Transmit.h"
#include <cassert>

int LARMDOF = 7, RARMDOF = 7;
double TempTime = 0.02;
std::vector<double> lArm_zeroPos(7, 0.0), rArm_zeroPos(7, 0.0);
std::vector<double> l_descZeroPos(6, 0.0), r_descZeroPos(6, 0.0);
std::vector<double> lArm_defaultKp_low(7, 1), rArm_defaultKp_low(7, 1);
std::vector<double> lArm_defaultKd_low(7, 1), rArm_defaultKd_low(7, 1);

int main() {
    JointStreamGuard guard;
    const std::vector<double> left(7, 0.5), right(7, -0.5), feedback(7, 0.1), drift(7, 0.2);
    guard.receive(left, right, false, false, 10.0);
    auto cmd = guard.sample(feedback, feedback, 10.01);
    assert(cmd.left == left && cmd.right == right && cmd.reanchor);
    assert(!guard.sample(feedback, feedback, 10.02).reanchor);
    // Timeout captures feedback once, never chases the previous policy target.
    cmd = guard.sample(feedback, feedback, 10.26);
    assert(cmd.left == feedback && cmd.right == feedback && cmd.reanchor);
    cmd = guard.sample(drift, drift, 10.5);
    assert(cmd.left == feedback && !cmd.reanchor);
    guard.receive(left, right, false, false, 11.0);
    cmd = guard.sample(drift, drift, 11.0);
    assert(cmd.left == left && cmd.reanchor);
    // Explicit holds use the same latch, including repeated hold packets.
    guard.receive(left, right, true, false, 11.01);
    cmd = guard.sample(feedback, feedback, 11.01);
    assert(cmd.left == feedback && cmd.reanchor);
    guard.receive(left, right, true, false, 11.02);
    cmd = guard.sample(drift, drift, 11.02);
    assert(cmd.left == feedback && !cmd.reanchor);

    arm_traj_ctl adapter;
    adapter.init();
    ctlDesMsg_trans des{};
    des.lArm_desMsg = left;
    des.rArm_desMsg = right;
    des.lArm_rigidity_mod.assign(7, 1);
    des.rArm_rigidity_mod.assign(7, 1);
    des.lArm_JntKp = des.rArm_JntKp = des.lArm_JntKd = des.rArm_JntKd = std::vector<double>(7, 1);
    JntLimMsg_trans limits{};
    // Real adapter must preserve absolute values across repeated stream packets.
    for (int i = 0; i < 100; ++i) {
        const auto result = adapter.trans_and_outp(limits, limits, 3, 0, des, false);
        assert(result.ctl_transMod == 3 && result.runinMod == 1);
        assert(result.ctlDesMsg.desPos_l_JointSpace == left);
        assert(result.ctlDesMsg.desPos_r_JointSpace == right);
    }
    assert(adapter.trans_and_outp(limits, limits, 1, 0, des, false).runinMod == 3);
}
