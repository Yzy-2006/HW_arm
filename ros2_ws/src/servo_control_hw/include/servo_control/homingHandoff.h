#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <vector>
#include "ros_controller/rosLoopMsgDefine.h"
#include "servo_control/armSafetyGuard.h"

// Preserve the emitted endpoint and bridge explicit controller takeover events.
class HomingHandoff {
public:
    struct Limits {
        double max_speed = 0.3;
        double max_acceleration = 0.5;
        double kp_rate = 20;
        double kd_rate = 10;
        double feedforward_rate = 1;
    } limits;

    void load(const YAML::Node& config) {
        const auto node = config["homingHandoff"];
        auto read = [&](const char* key, double& value) {
            if (node && node[key]) value = node[key].as<double>();
            if (!std::isfinite(value) || value <= 0)
                throw std::runtime_error(std::string("Invalid homingHandoff.") + key);
        };
        read("max_speed_rad_s", limits.max_speed);
        read("max_acceleration_rad_s2", limits.max_acceleration);
        read("kp_rate_per_s", limits.kp_rate);
        read("kd_rate_per_s", limits.kd_rate);
        read("feedforward_rate_per_s", limits.feedforward_rate);
    }

    void begin(const std::vector<joint_desMsg>& emitted, const ArmSafetyGuard& safety) {
        if (emitted.size() < 14) throw std::invalid_argument("Incomplete handoff commands");
        for (size_t i = 0; i < 14; ++i) {
            last_[i] = emitted[i];
            if (last_[i].joint_Kp == 0) {
                last_[i].joint_pos = safety.feedback[i].position;
                last_[i].joint_spd = 0;
            }
        }
        initialized_ = true;
        waiting_ = true;
        transitioning_ = true;
        aligned_time_ = 0;
        takeover_time_ = 0;
    }
    bool accept_command() {
        const bool first = initialized_ && waiting_;
        waiting_ = false;
        return first;
    }
    bool transitioning() const { return initialized_ && transitioning_; }
    uint64_t epoch() const { return epoch_; }
    double position(size_t i) const { return last_.at(i).joint_pos; }

    bool receive_epoch(uint64_t epoch, const std::vector<joint_desMsg>& emitted,
                       const ArmSafetyGuard& safety) {
        if (!epoch_seen_) {
            epoch_seen_ = true;
            epoch_ = epoch;
            return false; // Initial reception already has the homing handoff.
        }
        if (epoch == epoch_ || !safety.fault.empty()) return false;
        epoch_ = epoch;
        begin(emitted, safety);
        return true;
    }

    void apply(std::vector<joint_desMsg>& output, ArmSafetyGuard& safety, double dt) {
        if (!initialized_ || !safety.fault.empty()) return;
        if (output.size() < 14 || !std::isfinite(dt) || dt <= 0 || dt > 0.02) {
            safety.trip("invalid_handoff_state"); return;
        }
        if (waiting_) {
            for (size_t i = 0; i < 14; ++i)
                if (last_[i].joint_Kp == 0) last_[i].joint_pos = safety.feedback[i].position;
            std::copy(last_.begin(), last_.end(), output.begin());
            return;
        }
        if (!transitioning_) return;
        // Reject invalid source data before changing any target velocity.
        for (size_t i = 0; i < 14; ++i) {
            const auto& c = output[i];
            const auto error = safety.command_violation(i,
                {c.joint_pos, c.joint_spd, c.joint_tor, c.joint_Kp, c.joint_Kd});
            if (!error.empty()) { safety.trip(error); return; }
        }
        bool aligned = true;
        std::array<joint_desMsg, 14> requested;
        std::copy_n(output.begin(), 14, requested.begin());
        const double dv = limits.max_acceleration * dt;
        takeover_time_ += dt;
        for (size_t i = 0; i < 14; ++i) {
            const double max_speed = safety.speed_limited(i)
                ? std::min(limits.max_speed, safety.limits.max_speed) : std::numeric_limits<double>::infinity();
            auto& c = output[i];
            auto& previous = last_[i];
            if (c.joint_Kp > 0 && c.transMod != 5) {
                if (previous.joint_Kp == 0 || previous.transMod == 5) {
                    previous.joint_pos = safety.feedback[i].position;
                    previous.joint_spd = 0;
                }
                // Critically damped target tracking, with bounded acceleration.
                // Derive q from the same velocity sent to the motor; independent
                // position/velocity filters can inject a derivative torque step.
                const double error = c.joint_pos - previous.joint_pos;
                const double acceleration = 16 * error + 8 * (c.joint_spd - previous.joint_spd);
                const double desired_speed = std::clamp(previous.joint_spd + acceleration * dt,
                                                        -max_speed, max_speed);
                c.joint_spd = approach(previous.joint_spd, desired_speed, dv);
                c.joint_pos = previous.joint_pos + c.joint_spd * dt;
                c.joint_Kp = approach(previous.joint_Kp, c.joint_Kp, limits.kp_rate * dt);
                c.joint_Kd = approach(previous.joint_Kd, c.joint_Kd, limits.kd_rate * dt);
                c.joint_tor = approach(previous.joint_tor, c.joint_tor, limits.feedforward_rate * dt);
                const auto& target = requested[i];
                // The final snap must also fit inside the discrete acceleration
                // budget, not merely a position tolerance independent of dt.
                aligned = aligned && std::abs(target.joint_pos - c.joint_pos) <= 0.25 * dv * dt &&
                    std::abs(target.joint_spd - c.joint_spd) <= 0.25 * dv &&
                    std::abs(target.joint_pos - previous.joint_pos) <= max_speed * dt &&
                    std::abs((target.joint_pos - previous.joint_pos) / dt - previous.joint_spd) <= dv &&
                    std::abs(target.joint_spd - previous.joint_spd) <= dv &&
                    c.joint_Kp == target.joint_Kp && c.joint_Kd == target.joint_Kd &&
                    c.joint_tor == target.joint_tor;
            }
            // Explicit damping/current commands must not be delayed by the ramp.
            previous = c;
        }
        aligned_time_ = aligned ? aligned_time_ + dt : 0;
        if (takeover_time_ >= 1.0 && aligned_time_ >= 0.2) {
            std::copy(requested.begin(), requested.end(), output.begin());
            last_ = requested;
            transitioning_ = false;
        }
    }
private:
    static double approach(double from, double to, double step) {
        if (std::abs(to - from) <= step) return to;
        return from + std::clamp(to - from, -step, step);
    }
    std::array<joint_desMsg, 14> last_{};
    bool initialized_ = false, waiting_ = true, transitioning_ = false;
    double aligned_time_ = 0, takeover_time_ = 0;
    uint64_t epoch_ = 0;
    bool epoch_seen_ = false;
};
