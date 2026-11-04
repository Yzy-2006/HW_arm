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
    std::cout << "Filter long-run PSD/boundedness and arm safety fault injection passed\n";
}
