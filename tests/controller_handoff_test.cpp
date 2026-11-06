#include <cassert>
#include <cmath>
#include <limits>
#include "controller/controlHandoffEpoch.h"
#include "robot_control/startupPose.h"
#include "servo_control/homingHandoff.h"

int main() {
    const std::vector<double> home{.309, .087, -.389, -1.760, -1.533, -.874, -.031};
    assert(!outside_home_pose(home, home, .2));
    auto feedback = home;
    feedback[3] += .1;
    assert(!outside_home_pose(feedback, home, .2));
    feedback[3] += .15;
    assert(outside_home_pose(feedback, home, .2));
    feedback = home;
    feedback[5] -= .25;
    assert(outside_home_pose(feedback, home, .2));
    feedback[5] = std::numeric_limits<double>::quiet_NaN();
    bool rejected = false;
    try { outside_home_pose(feedback, home, .2); }
    catch (const std::invalid_argument&) { rejected = true; }
    assert(rejected);

    ControlHandoffEpoch tracker;
    ControlSource source{{1, 2, 3}, 0, 3, false, false,
                         std::vector<int32_t>(7, 1), std::vector<int32_t>(7, 1)};
    assert(tracker.value() == 0);

    ArmSafetyGuard guard;
    guard.limits.speed_only = true;
    for (auto& f : guard.feedback) f = {.5, 0, 10, 0};
    HomingHandoff handoff;
    std::vector<joint_desMsg> emitted(14, {1, .5, 0, 0, 15, 10});
    handoff.begin(emitted, guard);
    assert(!handoff.receive_epoch(0, emitted, guard));
    handoff.accept_command();
    for (int tick = 0; tick < 1100; ++tick) {
        auto output = emitted;
        handoff.apply(output, guard, .001);
    }
    assert(!handoff.transitioning());

    // The first robot packet arrives AFTER make arms has completed handoff.
    // Restart from the currently emitted command, not the old homing endpoint
    // or the lagging physical feedback. Repeated epochs must not reset the ramp.
    emitted[3].joint_pos = .8;
    assert(tracker.observe(source) && tracker.value() == 1);
    assert(handoff.receive_epoch(tracker.value(), emitted, guard));
    assert(handoff.position(3) == .8);
    assert(handoff.accept_command());
    double now = 10;
    for (int tick = 0; tick < 16000; ++tick) {
        assert(!tracker.observe(source));
        assert(!handoff.receive_epoch(tracker.value(), emitted, guard));
        assert(!handoff.accept_command());
        auto output = emitted;
        output[3] = {1, -.2, 0, 0, 40, 15};
        const auto previous = emitted[3];
        handoff.apply(output, guard, .001);
        const auto c = output[3];
        assert(std::abs(c.joint_pos - previous.joint_pos) <= .0003 + 1e-12);
        assert(std::abs(c.joint_spd - previous.joint_spd) <= .0005 + 1e-12);
        assert(std::abs(c.joint_Kp - previous.joint_Kp) <= .02 + 1e-12);
        assert(std::abs(c.joint_Kd - previous.joint_Kd) <= .01 + 1e-12);
        now += .001;
        // Feedback is deliberately lagging; command continuity must not depend
        // on assuming physical arrival. The existing overspeed guard stays active.
        if (tick == 0) guard.feedback[3].position = previous.joint_pos;
        for (size_t i = 0; i < 14; ++i) {
            const auto& command = output[i];
            guard.check_command(i, {command.joint_pos, command.joint_spd, command.joint_tor,
                                   command.joint_Kp, command.joint_Kd}, now);
        }
        assert(guard.fault.empty());
        emitted = output;
    }
    assert(!handoff.transitioning() && emitted[3].joint_pos == -.2);

    // Return-home -> VR transition; then a publisher restart during that ramp.
    source.control_mode = 4;
    source.interpolation_mode = 1;
    assert(tracker.observe(source) && tracker.value() == 2);
    assert(handoff.receive_epoch(tracker.value(), emitted, guard));
    handoff.accept_command();
    auto requested = emitted;
    requested[3].joint_pos = .6;
    handoff.apply(requested, guard, .001);
    source.publisher = {4, 5, 6};
    assert(tracker.observe(source) && tracker.value() == 3);
    assert(handoff.receive_epoch(tracker.value(), requested, guard));
    assert(handoff.position(3) == requested[3].joint_pos);
    handoff.accept_command();
    auto next = requested;
    next[3].joint_pos = .9;
    handoff.apply(next, guard, .001);
    assert(std::abs(next[3].joint_spd - requested[3].joint_spd) <= .0005 + 1e-12);

    source.bridge_stream = true;
    source.derivatives_valid = true;
    assert(tracker.observe(source));
    source.left_modes[0] = -1;
    assert(tracker.observe(source));
    guard.trip("operator_stop");
    assert(!handoff.receive_epoch(tracker.value(), next, guard));
    auto stopped = next;
    handoff.apply(stopped, guard, .001);
    assert(guard.fault == "operator_stop" && stopped[3].joint_pos == next[3].joint_pos);
}
