#pragma once

#include <array>
#include <algorithm>
#include <cmath>
#include <string>

// All times are steady-clock seconds. No ROS or transport dependency.
class ArmSafetyGuard {
public:
    static constexpr size_t joints = 14;
    struct Limits {
        double feedback_timeout = 0.1, command_timeout = 0.25;
        double max_speed = 1.5, max_feedforward = 5.0, max_feedforward_rate = 200.0;
        double tracking_error = 0.3, tracking_duration = 0.1, damping = 12.0;
        double feedback_position_tolerance = 0.02;
        std::array<double, joints> lower{}, upper{};
    } limits;
    struct Feedback {
        double position = 0, velocity = 0, received = -1;
        int error = 0;
    };
    struct Command { double position, velocity, feedforward, kp, kd; };
    std::array<Feedback, joints> feedback{};
    std::string fault;
    double command_period = 1.0 / 300;

    void trip(const std::string& reason) { if (fault.empty()) fault = reason; }
    bool ready(double now) const {
        if (!fault.empty()) return false;
        for (size_t i = 0; i < joints; ++i)
            if (!valid_feedback(i, now)) return false;
        return true;
    }
    void command_received(double now) { last_command_ = now; }
    void check_feedback(double now) {
        for (size_t i = 0; i < joints; ++i) {
            const auto& f = feedback[i];
            if (f.received < 0 && !armed_) continue;
            if (!valid_feedback(i, now)) trip("feedback_invalid_joint_" + std::to_string(i));
        }
        if (last_command_ >= 0 && now - last_command_ > limits.command_timeout)
            trip("controller_command_timeout");
        if (ready(now)) armed_ = true;
    }
    void check_command(size_t i, const Command& c, double now) {
        if (i >= joints) { trip("invalid_joint_mapping"); return; }
        if (!std::isfinite(c.position) || !std::isfinite(c.velocity) ||
            !std::isfinite(c.feedforward) || !std::isfinite(c.kp) || !std::isfinite(c.kd) ||
            c.kp < 0 || c.kp > 500 || c.kd < 0 || c.kd > 50) {
            trip("invalid_command_joint_" + std::to_string(i)); return;
        }
        if (c.position < limits.lower[i] || c.position > limits.upper[i] ||
            std::abs(c.velocity) > limits.max_speed ||
            std::abs(c.feedforward) > limits.max_feedforward)
            trip("command_limit_joint_" + std::to_string(i));
        if (last_output_time_[i] >= 0) {
            const double dt = now - last_output_time_[i];
            if (dt <= 0 || dt > limits.command_timeout ||
                std::abs(c.feedforward - last_feedforward_[i]) > limits.max_feedforward_rate * dt)
                trip("feedforward_rate_joint_" + std::to_string(i));
        }
        const bool position_active = ready(now) && c.kp > 0;
        if (position_active) {
            // Damping has no position target. Reanchor to feedback when enabling Kp.
            const double previous = position_active_[i] ? last_position_[i] : feedback[i].position;
            const double dt = position_active_[i] ? now - last_output_time_[i] : command_period;
            if (std::abs(c.position - previous) > limits.max_speed * std::max(command_period, dt) + 0.002)
                trip(position_active_[i] ? "position_step_joint_" + std::to_string(i)
                                        : "initial_position_step_joint_" + std::to_string(i));
        }
        position_active_[i] = position_active;
        last_output_time_[i] = now;
        last_feedforward_[i] = c.feedforward;
        last_position_[i] = c.position;
        if (ready(now) && c.kp > 0 && std::abs(c.position - feedback[i].position) > limits.tracking_error) {
            if (tracking_since_[i] < 0) tracking_since_[i] = now;
            if (now - tracking_since_[i] >= limits.tracking_duration)
                trip("tracking_error_joint_" + std::to_string(i));
        } else tracking_since_[i] = -1;
    }
    bool reset(double now) {
        // Stop upstream controllers first; never resume a queued trajectory on reset.
        if (last_command_ >= 0 && now - last_command_ < 1.0) return false;
        for (size_t i = 0; i < joints; ++i)
            if (!valid_feedback(i, now) || std::abs(feedback[i].velocity) > 0.05) return false;
        fault.clear();
        last_command_ = -1;
        last_output_time_.fill(-1);
        tracking_since_.fill(-1);
        last_feedforward_.fill(0);
        position_active_.fill(false);
        armed_ = false;
        return true;
    }
    ArmSafetyGuard() {
        last_output_time_.fill(-1);
        tracking_since_.fill(-1);
    }
private:
    bool valid_feedback(size_t i, double now) const {
        const auto& f = feedback[i];
        return f.received >= 0 && now >= f.received && now - f.received <= limits.feedback_timeout &&
            !f.error && std::isfinite(f.position) && std::isfinite(f.velocity) &&
            f.position >= limits.lower[i] - limits.feedback_position_tolerance &&
            f.position <= limits.upper[i] + limits.feedback_position_tolerance &&
            std::abs(f.velocity) <= limits.max_speed;
    }
    bool armed_ = false;
    double last_command_ = -1;
    std::array<double, joints> last_output_time_{}, last_feedforward_{}, tracking_since_{};
    std::array<double, joints> last_position_{};
    std::array<bool, joints> position_active_{};
};
