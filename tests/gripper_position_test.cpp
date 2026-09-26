#include <cassert>
#include <cmath>
#include <limits>

#include "JntController/ForceComp.h"

int main(int argc, char** argv){
    assert(argc == 2);
    ServoNodeCtlConfig loader;
    assert(loader.loadFromYaml(argv[1]));
    const auto config = loader.getData();
    assert(config.GripperLeft_Control.mode == 1);
    assert(config.GripperRight_Control.mode == 1);

    constexpr double rad = 0.017453292519943295;
    for(int direction : {-1, 1}){
        gripper_RangeLimit limit;
        limit.Init(3.0, 70.0, 5.0);
        GripperControlConfig control;
        control.mode = 1;
        control.kp = 8.0;
        control.kd = 2.0;
        control.pos_min_deg = -30.0;
        control.pos_max_deg = 45.0;
        auto command = limit.PositionOutp(20.0, control, direction, 69.0);
        assert(command.joint_mode == 1);
        assert(std::abs(direction * command.joint_pos - 20.0 * rad) < 1e-12);
        assert(command.joint_spd == 0.0 && command.joint_tor == 0.0);
        assert(command.joint_Kp == 8.0 && command.joint_Kd == 2.0);
        command = limit.PositionOutp(100.0, control, direction, 70.0);
        assert(std::abs(direction * command.joint_pos - 45.0 * rad) < 1e-12);
        assert(command.joint_Kp == 0.0 && command.joint_Kd == 0.0);
        assert(limit.PositionOutp(20.0, control, direction, 66.0).joint_Kp == 0.0);
        assert(limit.PositionOutp(20.0, control, direction, 65.0).joint_Kp == 8.0);
        command = limit.PositionOutp(-100.0, control, direction, 30.0);
        assert(std::abs(direction * command.joint_pos + 30.0 * rad) < 1e-12);
        assert(limit.PositionOutp(20.0, control, direction,
                                 std::numeric_limits<double>::quiet_NaN()).joint_Kp == 0.0);
        assert(limit.PositionOutp(20.0, control, 0, 30.0).joint_Kp == 0.0);
    }

    runin_core interpolator;
    for(int window : {1, 20, 100}){
        const auto points = interpolator.liner_runin(30.0, -45.0, window);
        assert(points.size() == static_cast<size_t>(window));
        assert(points.front() < 30.0);
        assert(points.back() == -45.0);
        for(double position : points){
            assert(position >= -45.0 && position <= 30.0);
        }
    }
}
