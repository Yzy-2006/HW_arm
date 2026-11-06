#include <cassert>
#include <limits>
#include "JntController/retZeroControl.h"
#include "servo_control/homingMonitor.h"
#include "servo_control/motorPositionOffsets.h"

double TEMPTIME = .001;
int LARMDOF = 7, RARMDOF = 7;
std::vector<double> lArm_zeroPos(7, .1), rArm_zeroPos(7, .1);
std::vector<double> lArm_defaultKp(7, 15), rArm_defaultKp(7, 15);
std::vector<double> lArm_defaultKd(7, 10), rArm_defaultKd(7, 10);

ArmSafetyGuard healthy(double t = 10) {
    ArmSafetyGuard g;
    g.limits.lower.fill(-3); g.limits.upper.fill(3);
    for (auto& f : g.feedback) f = {0, 0, t, 0};
    return g;
}

void refresh(ArmSafetyGuard& g, double t) {
    for (auto& f : g.feedback) f.received = t;
}

int main() {
    MotorPositionOffsets offsets;
    for (double degrees : {0., 180., -180.}) {
        YAML::Node config;
        config["motorPositionOffsets"]["left_shoulder_roll_deg"] = degrees;
        offsets.load(config);
        for (int direction : {-1, 1}) {
            const double raw = offsets.to_motor(0, 2, direction, .2);
            assert(std::abs(raw - (direction * .2 + degrees * M_PI / 180)) < 1e-12);
            assert(std::abs(offsets.from_motor(0, 2, direction, raw) - .2) < 1e-12);
        }
        assert(offsets.offset(1, 2) == 0 && offsets.offset(0, 8) == 0 && offsets.offset(0, 1) == 0);
    }
    offsets.load(YAML::Load("{}"));
    assert(offsets.offset(0, 2) == 0);
    for (const char* value : {".nan", ".inf", "361", "bad"}) {
        bool rejected = false;
        try { offsets.load(YAML::Load(std::string("motorPositionOffsets: {left_shoulder_roll_deg: ") + value + "}")); }
        catch (const std::exception&) { rejected = true; }
        assert(rejected);
    }
    std::vector<double> start(7, 0);
    JointPos_Init trajectory;
    auto g = healthy();
    assert(g.command_violation(0, {0, 0, 6, 15, 10}).find("field=feedforward") != std::string::npos);
    assert(g.command_violation(0, {0, 2, 0, 15, 10}).find("max_abs_v=1.5") != std::string::npos);
    assert(g.command_violation(0, {0, 0, 0, 501, 10}).find("nonfinite_or_gain") != std::string::npos);
    g.limits.lower[1] = 0;
    start[1] = -.001;
    g.feedback[1].position = start[1];
    assert(g.ready(10));  // Feedback tolerance does not authorize an out-of-range command.
    auto error = trajectory.retZero_runin(start, std::vector<double>(7, 0), g);
    assert(error.find("homing_start_rejected") != std::string::npos);
    assert(error.find("left_shoulder_roll_joint") != std::string::npos);
    assert(error.find("q=-0.001") != std::string::npos);
    assert(error.find("bounds=[0,3]") != std::string::npos);
    g.limits.check_command_position_limits = false;
    assert(trajectory.retZero_runin(start, std::vector<double>(7, 0), g).empty());
    g.check_command(1, {-.001, 0, 0, 15, 10}, 10);
    assert(g.fault.empty());
    assert(g.command_violation(1, {-.001, 2, 0, 15, 10}).find("field=velocity") != std::string::npos);
    assert(g.command_violation(1, {-.001, 0, 6, 15, 10}).find("field=feedforward") != std::string::npos);
    assert(g.command_violation(1, {std::numeric_limits<double>::quiet_NaN(), 0, 0, 15, 10})
           .find("nonfinite_or_gain") != std::string::npos);
    auto feedback_guard = g;
    feedback_guard.feedback[1].position = -.03;
    feedback_guard.check_feedback(10);
    assert(feedback_guard.fault.find("feedback_invalid_joint_1") != std::string::npos);
    auto no_position_bounds = g;
    no_position_bounds.limits.check_feedback_position_limits = false;
    no_position_bounds.feedback[1].position = -.03;
    no_position_bounds.feedback[2].position = 3.03;
    no_position_bounds.check_feedback(10);
    assert(no_position_bounds.ready(10));
    for (int failure = 0; failure < 5; ++failure) {
        auto guard = no_position_bounds;
        auto& f = guard.feedback[1];
        if (failure == 0) f.received = 9.8;
        if (failure == 1) f.velocity = 2;
        if (failure == 2) f.error = 1;
        if (failure == 3) f.position = std::numeric_limits<double>::quiet_NaN();
        if (failure == 4) f.velocity = std::numeric_limits<double>::infinity();
        guard.check_feedback(10);
        assert(!guard.ready(10));
        assert(guard.fault.find(failure == 1 ? "feedback_speed_joint_1" : "feedback_invalid_joint_1") != std::string::npos);
    }
    // Feedback bounds and target bounds are independently configurable.
    no_position_bounds.limits.check_command_position_limits = true;
    assert(no_position_bounds.command_violation(1, {-.03, 0, 0, 15, 10}).find("field=position") != std::string::npos);
    g.limits.check_command_position_limits = true;
    start[1] = 0;
    g.feedback[1].position = 0;
    lArm_zeroPos[1] = -.1;
    assert(trajectory.retZero_runin(start, start, g).find("homing_goal_rejected") != std::string::npos);
    lArm_zeroPos[1] = .1;
    lArm_zeroPos[0] = 1;
    g.limits.max_speed = .1;
    error = trajectory.retZero_runin(start, start, g);
    assert(error.find("homing_sample_") != std::string::npos);
    assert(error.find("field=velocity") != std::string::npos);
    lArm_zeroPos[0] = .1;
    g.limits.max_speed = 1.5;
    assert(trajectory.retZero_runin(start, start, g).empty());
    bool finished = false;
    for (int n = 0; n < 5000 && !finished; ++n) {
        auto sample = trajectory.retZero_trajOutp();
        for (int i = 0; i < 7; ++i) {
            assert(g.command_violation(i, {sample.lArm_desPos[i], sample.lArm_desVel[i], 0,
                                           sample.lArm_Kp[i], sample.lArm_Kd[i]}).empty());
        }
        finished = sample.ifOutpReachEnd;
    }
    assert(finished);
    lArm_zeroPos[0] = std::numeric_limits<double>::quiet_NaN();
    assert(trajectory.retZero_runin(start, start, g).find("nonfinite_or_gain") != std::string::npos);
    lArm_zeroPos[0] = .1;
    assert(trajectory.retZero_runin({}, start, g) == "homing_invalid_joint_dimensions");

    using Result = HomingMonitor::Result;
    std::array<double, 14> goal{};
    {
        auto guard = healthy();
        guard.limits.speed_only = true;
        guard.limits.lower.fill(1);
        guard.command_received(1);
        for (auto& f : guard.feedback) f.error = 7;
        // Cached feedback, motor errors and old commands do not trip speed-only mode.
        guard.check_feedback(20);
        assert(guard.ready(20));
        assert(guard.command_violation(0, {0, 0, 10, 15, 10}).empty());
        for (int n = 0; n < 600; ++n) {
            guard.check_command(0, {n * .001, 1, n % 2 ? 10. : -10., 15, 10}, 20 + n * .001);
        }
        assert(guard.fault.empty());  // Includes persistent >0.3rad tracking error.
        HomingMonitor monitor(goal, .02);
        guard.feedback[0].position = 1;
        assert(monitor.update(guard, 21, true) == Result::finished_unverified);
        auto command_speed = guard;
        command_speed.check_command(0, {1, 1.6, 0, 15, 10}, 21);
        assert(command_speed.fault.find("field=velocity") != std::string::npos);
        auto measured_speed = guard;
        measured_speed.feedback[5].velocity = -1.6;
        measured_speed.check_feedback(21);
        assert(measured_speed.fault.find("feedback_speed_joint_5") != std::string::npos);
        assert(measured_speed.fault.find("v=-1.6") != std::string::npos);
        auto jump = guard;
        jump.check_command(0, {2, 0, 0, 15, 10}, 20.6);
        assert(jump.fault.find("position_step") != std::string::npos);
        auto invalid = guard;
        invalid.feedback[0].velocity = std::numeric_limits<double>::quiet_NaN();
        invalid.check_feedback(21);
        assert(!invalid.fault.empty());
        guard.trip("operator_stop");
        assert(monitor.update(guard, 21, true) == Result::fault);
    }
    {
        auto guard = healthy();
        HomingMonitor monitor(goal, .02);
        assert(monitor.update(guard, 10, false) == Result::running);
        refresh(guard, 11);
        assert(monitor.update(guard, 11, true) == Result::running);
        guard.feedback[0].position = .03;
        refresh(guard, 11.1);
        assert(monitor.update(guard, 11.1, true) == Result::running);
        guard.feedback[0].position = 0;
        guard.feedback[0].velocity = .06;
        refresh(guard, 11.2);
        assert(monitor.update(guard, 11.2, true) == Result::running);
        guard.feedback[0].velocity = 0;
        refresh(guard, 11.3);
        assert(monitor.update(guard, 11.3, true) == Result::running);
        refresh(guard, 11.51);
        assert(monitor.update(guard, 11.51, true) == Result::reached);
    }
    {
        auto guard = healthy();
        HomingMonitor monitor(goal, .02);
        guard.feedback[0].position = .1;
        assert(monitor.update(guard, 10, true) == Result::running);
        refresh(guard, 12.01);
        assert(monitor.update(guard, 12.01, true) == Result::fault);
        assert(guard.fault.find("homing_settle_timeout") != std::string::npos);
    }
    {
        auto guard = healthy();
        HomingMonitor monitor(goal, .02);
        assert(monitor.update(guard, 10, true) == Result::running);
        assert(monitor.update(guard, 10.21, true) == Result::fault);  // Cached in-position feedback.
    }
    {
        auto guard = healthy();
        HomingMonitor monitor(goal, .02);
        guard.trip("operator_stop");
        assert(monitor.update(guard, 10, false) == Result::fault);
        refresh(guard, 11);
        assert(monitor.update(guard, 11, true) == Result::fault);
    }
    {
        auto guard = healthy();
        for (auto& f : guard.feedback) f.received = -1;
        HomingMonitor monitor(goal, .02);
        assert(monitor.update(guard, 10, true) == Result::fault);
    }
}
