#include <bits/stdc++.h>
#define private public
#include "controller/JntController.h"
#undef private
#include "servo_control/armSafetyGuard.h"

double TEMPTIME = 1.0 / 300.0;

int main() {
    for (double dt : {0.001, 1.0 / 300, 0.02}) {
        Kernel::Filter f;
        f.FilterInit(0, 1, .01, dt);
        double peak = 0;
        for (int n = 0; n < 180000; ++n) {
            const double y = f.FilteredResCal(.01 * std::sin(2 * M_PI * 2 * n * dt));
            assert(std::isfinite(y));
            peak = std::max(peak, std::abs(y));
            assert(f.Pmtrx[0][0] >= 0 && f.Pmtrx[1][1] >= 0);
            assert(std::abs(f.Pmtrx[0][1] - f.Pmtrx[1][0]) < 1e-12);
            assert(f.Pmtrx[0][0] * f.Pmtrx[1][1] - f.Pmtrx[0][1] * f.Pmtrx[1][0] >= -1e-12);
        }
        assert(peak < .03);
        assert(std::isnan(f.FilteredResCal(std::numeric_limits<double>::quiet_NaN())));
        assert(std::isfinite(f.FilteredResCal(0)));
        Kernel::lowpassFilter lowpass;
        lowpass.init(1 / dt, 2);
        const double first = lowpass.filter(1);
        assert(std::abs(first - (1 - std::exp(-2 * M_PI * 2 * dt))) < 1e-12);
    }
    auto healthy = [] {
        ArmSafetyGuard g;
        g.limits.lower.fill(-3); g.limits.upper.fill(3);
        for (auto& f : g.feedback) f = {0, 0, 10, 0};
        g.check_feedback(10);
        assert(g.ready(10));
        return g;
    };
    {
        auto g = healthy();
        g.command_received(10);
        // ROS commands continue, but the motor feedback is cached.
        g.command_received(10.11);
        g.check_feedback(10.11);
        assert(!g.fault.empty());
        for (auto& f : g.feedback) f.received = 10.12;
        assert(!g.ready(10.12));
        assert(!g.reset(10.12));
        for (auto& f : g.feedback) f.received = 11.2;
        assert(g.reset(11.2));
    }
    for (double ff : {-104., 642., -44114., std::numeric_limits<double>::quiet_NaN()}) {
        auto g = healthy();
        g.check_command(0, {0, 0, ff, 160, 50}, 10);
        assert(!g.fault.empty());
    }
    {
        auto g = healthy();
        g.check_command(0, {0, 0, 0, 160, 50}, 10);
        g.check_command(0, {0, 0, 1, 160, 50}, 10.001);
        assert(!g.fault.empty());
    }
    {
        auto g = healthy();
        g.feedback[0].velocity = 16.51;
        g.check_feedback(10);
        assert(!g.fault.empty());
    }
    {
        auto g = healthy();
        g.feedback[0].error = 1;
        g.check_feedback(10);
        assert(!g.fault.empty());
    }
    {
        auto g = healthy();
        g.feedback[0].position = .5;
        g.check_command(0, {.5, 0, 0, 160, 50}, 10);
        g.feedback[0].position = 0;
        for (int n = 1; n <= 111; ++n) {
            double t = 10 + n * .001;
            for (auto& f : g.feedback) f.received = t;
            g.check_command(0, {.5, 0, 0, 160, 50}, t);
        }
        assert(g.fault.find("tracking_error") != std::string::npos);
    }
    {
        auto g = healthy();
        g.feedback[0].position = .5;
        g.check_command(0, {0, 0, 0, 0, 12}, 10);
        g.check_command(0, {.5, 0, 0, 160, 50}, 10.001);
        assert(g.fault.empty());
        g.check_command(0, {0, 0, 0, 0, 12}, 10.002);
        g.check_command(0, {0, 0, 0, 160, 50}, 10.003);
        assert(g.fault.find("initial_position_step") != std::string::npos);
    }
    {
        auto g = healthy();
        g.command_received(10);
        for (auto& f : g.feedback) f.received = 10.3;
        g.check_feedback(10.3);
        assert(g.fault == "controller_command_timeout");
    }
    for (bool speed_only : {false, true}) {
        auto g = healthy();
        g.limits.speed_only = speed_only;
        g.limits.feedback_speed_trip_samples = 3;
        auto sample = [&](size_t joint, double velocity, double t) {
            g.feedback[joint] = {0, velocity, t, 0};
            g.check_feedback(t);
        };
        sample(5, 2, 10.001);
        for (int n = 0; n < 10; ++n) g.check_feedback(10.001 + n * .00001);
        assert(g.fault.empty() && g.ready(10.002)); // No cached-sample trip or heartbeat loss.
        sample(5, -2, 10.002);
        assert(g.fault.empty() && g.ready(10.002));
        sample(6, 2, 10.003); // Counts belong to individual joints.
        assert(g.fault.empty());
        sample(5, 1.5, 10.004); // Exactly at limit clears the streak.
        sample(5, 2, 10.005);
        sample(5, 2, 10.006);
        assert(g.fault.empty() && g.ready(10.006));
        sample(5, -2, 10.007);
        assert(g.fault.find("feedback_speed_joint_5") != std::string::npos);
        assert(g.fault.find("consecutive_samples=3 required=3") != std::string::npos);
        sample(5, 0, 10.008);
        assert(!g.fault.empty()); // Normal feedback never clears a latched fault.
        for (auto& f : g.feedback) f = {0, 0, 10.009, 0};
        assert(g.reset(10.009));
        sample(5, 2, 10.010);
        assert(g.fault.empty() && g.ready(10.010));
        g.feedback[5].position = std::numeric_limits<double>::quiet_NaN();
        g.check_feedback(10.010);
        assert(g.fault == "feedback_invalid_joint_5"); // Debounce only applies to finite speed.
    }
    {
        auto g = healthy();
        g.limits.speed_only = true;
        g.limits.feedback_speed_trip_samples = 3;
        for (double t : {10.001, 10.002, 10.2, 10.201}) {
            g.feedback[0] = {0, 2, t, 0};
            g.check_feedback(t);
            assert(g.fault.empty()); // A long feedback gap starts a new streak.
        }
        g.feedback[0] = {0, 2, 10.202, 0};
        g.check_feedback(10.202);
        assert(g.fault.find("feedback_speed_joint_0") != std::string::npos);
    }
    for (bool speed_only : {false, true}) {
        for (size_t joint = 0; joint < 14; ++joint) {
            auto g = healthy();
            g.limits.speed_only = speed_only;
            g.limits.speed_limit_enabled_per_arm = {true, true, true, true, false, false, false};
            g.limits.feedback_speed_trip_samples = 3;
            const bool limited = joint % 7 < 4;
            assert(g.speed_limited(joint) == limited);
            assert(g.command_violation(joint, {0, 6, 0, 100, 12}).empty() == !limited);
            g.check_command(joint, {0, 0, 0, 100, 12}, 10);
            g.check_command(joint, {.2, 0, 0, 100, 12}, 10.001);
            assert(g.fault.empty() == !limited);
            g.fault.clear();
            for (double t : {10.002, 10.003, 10.004}) {
                g.feedback[joint] = {0, 6, t, 0};
                g.check_feedback(t);
            }
            assert(g.fault.empty() == !limited);
            assert(g.ready(10.004) == !limited);
            if (!limited) {
                g.feedback[joint].velocity = std::numeric_limits<double>::quiet_NaN();
                g.check_feedback(10.004);
                assert(g.fault.find("feedback_invalid_joint_") != std::string::npos);
                assert(!g.command_violation(joint, {0, std::numeric_limits<double>::infinity(), 0, 100, 12}).empty());
            }
        }
    }
    for (size_t joint = 0; joint < 14; ++joint) {
        auto g = healthy();
        g.limits.speed_only = true;
        g.limits.max_speed = 5;
        g.limits.speed_limit_enabled_per_arm = {true, true, true, true, false, false, false};
        g.limits.feedback_speed_trip_samples = 3;
        g.policy_speed_override(true, .01, 10);
        assert(g.policy_speed_override_active() && !g.speed_limited(joint));
        g.check_command(joint, {1, -6.258, 0, 100, 30}, 10.001);
        for (double t : {10.002, 10.003, 10.004}) {
            g.feedback[joint] = {0, 8, t, 0};
            g.check_feedback(t);
        }
        assert(g.fault.empty() && g.ready(10.004));
        assert(!g.command_violation(joint, {0, NAN, 0, 100, 30}).empty());
        g.policy_speed_override(true, .21, 10.2); // Same original packet, forwarded again.
        g.feedback[joint] = {0, 0, 10.241, 0};
        g.check_feedback(10.241);
        assert(!g.policy_speed_override_active());
        assert(g.speed_limited(joint) == (joint % 7 < 4));
        assert(g.command_violation(joint, {0, 6.258, 0, 100, 30}).empty() == (joint % 7 >= 4));
        g.policy_speed_override(true, 0, 11);
        g.policy_speed_override(false, 0, 11.001); // Replay, VR or explicit hold restores immediately.
        assert(!g.policy_speed_override_active());
        for (double age : {-.01, .25, 1., std::numeric_limits<double>::quiet_NaN()}) {
            g.policy_speed_override(true, age, 11);
            assert(!g.policy_speed_override_active());
        }
        g.trip("operator_stop");
        g.policy_speed_override(true, 0, 12);
        assert(!g.policy_speed_override_active() && g.fault == "operator_stop");
    }
    std::cout << "Filter long-run PSD/boundedness and arm safety fault injection passed\n";
}
