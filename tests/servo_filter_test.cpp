#include <bits/stdc++.h>
#include <yaml-cpp/yaml.h>
#define private public
#include "JntController/ForceComp.h"
#undef private
#include "servo_control/homingHandoff.h"

double TEMPTIME = .001;
int LARMDOF = 7, RARMDOF = 7;

int main() {
    for (double q : {1.0, 12000.0}) {
        kalman_filter f;
        f.FilterInit(0, q, q == 1 ? .01 : 300, TEMPTIME);
        for (int n = 0; n < 180000; ++n) {
            const double result = f.FilteredResCal(.01 * std::sin(2 * M_PI * 2 * n * TEMPTIME));
            assert(std::isfinite(result) && std::abs(result) < .03);
            assert(f.Pmtrx[0][0] >= 0 && f.Pmtrx[1][1] >= 0);
            assert(std::abs(f.Pmtrx[0][1] - f.Pmtrx[1][0]) < 1e-10);
            assert(f.Pmtrx[0][0] * f.Pmtrx[1][1] - f.Pmtrx[0][1] * f.Pmtrx[1][0] >= -1e-8);
        }
    }
    // Initializing away from zero must preserve a stationary trajectory.
    runin_control interpolator;
    std::vector<double> position(7, 1.0), zero(7, 0.0), kp(7, 100), kd(7, 12);
    interpolator.init(position, position, 3);
    interpolator.ModelRunin(position, zero, zero, zero, zero,
                            position, zero, zero, zero, zero, kp, kd, kp, kd, 1, 3);
    for (int i = 0; i < 5; ++i) {
        auto result = interpolator.dataOutp();
        for (double p : result.runinData.lArm_posDot) assert(std::abs(p - 1) < 1e-9);
    }

    // Homing ends at q=.5 while actual feedback still lags at .1. Waiting must
    // preserve the command endpoint, never silently initialize from feedback.
    ArmSafetyGuard safety;
    safety.limits.speed_only = true;
    safety.limits.lower.fill(-3);
    safety.limits.upper.fill(3);
    std::vector<joint_desMsg> emitted(14, {1, .5, 0, 0, 15, 10});
    for (size_t i = 0; i < 14; ++i) {
        safety.feedback[i] = {.5, 0, 10, 0};
    }
    for (size_t i = 0; i < 14; ++i) {
        safety.check_command(i, {.5, 0, 0, 15, 10}, 10);
        safety.feedback[i].position = .1;
    }
    HomingHandoff handoff;
    handoff.begin(emitted, safety);
    double now = 10;
    for (int tick = 0; tick < 300; ++tick) {
        now += TEMPTIME;
        std::vector<joint_desMsg> output(14, {1, 0, 0, 0, 0, 12});
        handoff.apply(output, safety, TEMPTIME);
        for (size_t i = 0; i < 14; ++i) {
            assert(output[i].joint_pos == .5 && output[i].joint_Kp == 15);
            safety.check_command(i, {output[i].joint_pos, output[i].joint_spd, 0, 15, 10}, now);
        }
        assert(safety.fault.empty());
    }
    // Seed the real legacy interpolator from the emitted endpoint as the node does.
    std::vector<double> seed(7, handoff.position(0));
    interpolator.init(seed, seed, 3);
    interpolator.ModelRunin(seed, zero, zero, zero, zero, seed, zero, zero, zero, zero, kp, kd, kp, kd, 2, 3);
    auto initial = interpolator.dataOutp();
    assert(std::abs(initial.runinData.lArm_posDot[1] - .5) < 1e-9);

    handoff.accept_command();
    double previous = .5;
    double previous_speed = 0;
    for (int tick = 0; tick < 15000; ++tick) {
        now += TEMPTIME;
        auto output = emitted;
        output[1].joint_pos = -.4;  // Also represents an immediate sampled-trajectory takeover.
        handoff.apply(output, safety, TEMPTIME);
        assert(std::abs(output[1].joint_pos - previous) <= handoff.limits.max_speed * TEMPTIME + 1e-12);
        assert(std::abs(output[1].joint_spd) <= handoff.limits.max_speed + 1e-12);
        assert(std::abs(output[1].joint_spd - previous_speed) <= handoff.limits.max_acceleration * TEMPTIME + 1e-12);
        assert(std::abs((output[1].joint_pos - previous) / TEMPTIME - output[1].joint_spd) <=
               handoff.limits.max_acceleration * TEMPTIME + 1e-12);
        previous = output[1].joint_pos;
        previous_speed = output[1].joint_spd;
        for (size_t i = 0; i < 14; ++i)
            safety.check_command(i, {output[i].joint_pos, output[i].joint_spd, 0, 15, 10}, now);
        assert(safety.fault.empty());
    }
    assert(std::abs(previous + .4) < 1e-12 && !handoff.transitioning());
    safety.check_command(1, {1, 0, 0, 15, 10}, now + TEMPTIME);
    assert(safety.fault.find("position_step_joint_1") != std::string::npos);
    assert(safety.fault.find("previous=") != std::string::npos);

    // Damping startup/reset anchors a later position command to feedback, not q=0.
    ArmSafetyGuard reset_guard;
    reset_guard.limits.speed_only = true;
    for (auto& f : reset_guard.feedback) f = {.3, 0, 10, 0};
    auto damping = std::vector<joint_desMsg>(14, {1, 0, 0, 0, 0, 12});
    handoff.begin(damping, reset_guard);
    handoff.apply(damping, reset_guard, TEMPTIME);
    assert(damping[1].joint_pos == .3 && damping[1].joint_Kp == 0);
    handoff.accept_command();
    for (auto& f : reset_guard.feedback) f.position = .6;
    auto resumed = emitted;
    handoff.apply(resumed, reset_guard, TEMPTIME);
    assert(std::abs(resumed[1].joint_pos - .6) <= reset_guard.limits.max_speed * TEMPTIME + 1e-12);
    for (size_t i = 0; i < 14; ++i)
        reset_guard.check_command(i, {resumed[i].joint_pos, resumed[i].joint_spd, 0, 15, 10}, 10.001);
    assert(reset_guard.fault.empty());
    auto bad = emitted;
    bad[1].joint_spd = 2;
    handoff.apply(bad, reset_guard, TEMPTIME);
    assert(reset_guard.fault.find("field=velocity") != std::string::npos);
    const auto unchanged = bad;
    handoff.apply(bad, reset_guard, TEMPTIME);
    assert(bad[1].joint_pos == unchanged[1].joint_pos);  // A fault never gets cleared.

    // Same target/gains on make arms startup must not introduce any motion,
    // even if measured position still lags. Release is not an arrival claim.
    ArmSafetyGuard stationary_guard;
    stationary_guard.limits.speed_only = true;
    for (auto& f : stationary_guard.feedback) f = {.1, 0, 10, 0};
    handoff.begin(emitted, stationary_guard);
    assert(handoff.accept_command());
    for (int tick = 0; tick < 1100; ++tick) {
        assert(!handoff.accept_command()); // Repeated packets do not restart the ramp.
        auto output = emitted;
        handoff.apply(output, stationary_guard, TEMPTIME);
        for (auto c : output) {
            assert(c.joint_pos == .5 && c.joint_spd == 0);
            assert(c.joint_Kp == 15 && c.joint_Kd == 10 && c.joint_tor == 0);
        }
        if (tick < 999) assert(handoff.transitioning());
    }
    assert(!handoff.transitioning());

    // Reversing a moving target and changing gains/feedforward must remain
    // bounded on both arms, including the release back to the upstream stream.
    for (double direction : {-1., 1.}) {
        HomingHandoff moving;
        moving.begin(emitted, stationary_guard);
        moving.accept_command();
        auto previous_output = emitted;
        double previous_step_speed = 0;
        for (int tick = 0; tick < 18000; ++tick) {
            auto output = emitted;
            output[11] = {1, .5 + direction * (tick < 1500 ? .9 : -.6), 0, 2, 35, 15};
            moving.apply(output, stationary_guard, TEMPTIME);
            const auto& c = output[11];
            const auto& old = previous_output[11];
            const double step_speed = (c.joint_pos - old.joint_pos) / TEMPTIME;
            assert(std::abs(c.joint_spd) <= .3 + 1e-12);
            assert(std::abs(c.joint_spd - old.joint_spd) <= .5 * TEMPTIME + 1e-12);
            assert(std::abs(step_speed - previous_step_speed) <= .5 * TEMPTIME + 1e-9);
            assert(std::abs(c.joint_Kp - old.joint_Kp) <= 20 * TEMPTIME + 1e-12);
            assert(std::abs(c.joint_Kd - old.joint_Kd) <= 10 * TEMPTIME + 1e-12);
            assert(std::abs(c.joint_tor - old.joint_tor) <= TEMPTIME + 1e-12);
            previous_step_speed = step_speed;
            previous_output = output;
        }
        assert(!moving.transitioning());
        assert(std::abs(previous_output[11].joint_pos - (.5 - direction * .6)) < 1e-12);
    }

    // Damping is immediate, and physical overspeed still latches in speed-only mode.
    handoff.begin(emitted, stationary_guard);
    handoff.accept_command();
    damping.assign(14, {1, 0, 0, 0, 0, 12});
    handoff.apply(damping, stationary_guard, TEMPTIME);
    assert(damping[11].joint_Kp == 0 && damping[11].joint_Kd == 12);
    stationary_guard.feedback[11].velocity = -1.929670334;
    stationary_guard.check_feedback(10);
    assert(stationary_guard.fault.find("feedback_speed_joint_11") != std::string::npos);

    // The same per-arm mask also controls takeover speed caps; acceleration remains bounded.
    {
        ArmSafetyGuard masked;
        masked.limits.speed_only = true;
        masked.limits.max_speed = .3;
        masked.limits.speed_limit_enabled_per_arm = {true, true, true, true, false, false, false};
        for (auto& f : masked.feedback) f = {0, 0, 10, 0};
        auto seed = std::vector<joint_desMsg>(14, {1, 0, 0, 0, 100, 12});
        HomingHandoff transition;
        transition.begin(seed, masked);
        transition.accept_command();
        std::array<double, 14> peak{}, previous_v{};
        for (int tick = 0; tick < 3000; ++tick) {
            auto target = seed;
            for (auto& c : target) c.joint_pos = 1;
            transition.apply(target, masked, TEMPTIME);
            assert(masked.fault.empty());
            for (size_t i = 0; i < 14; ++i) {
                peak[i] = std::max(peak[i], std::abs(target[i].joint_spd));
                assert(std::abs(target[i].joint_spd - previous_v[i]) <= .5 * TEMPTIME + 1e-12);
                previous_v[i] = target[i].joint_spd;
            }
        }
        for (size_t i = 0; i < 14; ++i) {
            if (i % 7 < 4) assert(peak[i] <= .3 + 1e-12);
            else assert(peak[i] > .3);
        }
    }

    HomingHandoff configured;
    configured.load(YAML::Load("homingHandoff: {max_speed_rad_s: 0.2, max_acceleration_rad_s2: 0.4}"));
    assert(configured.limits.max_speed == .2 && configured.limits.max_acceleration == .4);
    for (const char* value : {"0", "-1", ".nan", ".inf"}) {
        bool rejected = false;
        try { configured.load(YAML::Load(std::string("homingHandoff: {max_speed_rad_s: ") + value + "}")); }
        catch (const std::runtime_error&) { rejected = true; }
        assert(rejected);
    }
}
