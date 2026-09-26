#include "controller/xrTelepercationCom.h"
#include <cassert>

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

int main(int argc, char** argv){
    assert(argc == 3);
    test_share = argv[1];
    const bool valid = std::string(argv[2]) == "valid";
    xr_telepercation_ctl ctl;
    ctl.init();
    ctl.trigger_preset_pose();
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
        ctl.update_desc_ret(feedback, feedback);
        for(int i = 0; i < cycles; ++i){
            ctl.l_arm_end_delta[0] += 0.01;
            auto held = ctl.trans_and_outp();
            assert(held.ctl_transMod == 1);
            assert(held.ctlDesMsg.desPos_l_JointSpace == cmd.ctlDesMsg.desPos_l_JointSpace);
        }
        auto resumed = ctl.trans_and_outp();
        assert(resumed.ctl_transMod == 4 && resumed.ifUseRet == 1);
        assert(resumed.ctlDesMsg.desPos_l_Descartes == feedback);
        resumed = ctl.trans_and_outp();
        assert(resumed.ifUseRet == 0);
        assert(resumed.ctlDesMsg.desPos_l_Descartes == feedback);
    }else{
        assert(cmd.ctl_transMod == 4);
    }
    ctl.trigger_joint_return();
    cmd = ctl.trans_and_outp();
    assert(cmd.ctl_transMod == 1);
    assert(cmd.ctlDesMsg.desPos_l_JointSpace == lArm_zeroPos);
    assert(cmd.ctlDesMsg.desPos_r_JointSpace == rArm_zeroPos);
}
