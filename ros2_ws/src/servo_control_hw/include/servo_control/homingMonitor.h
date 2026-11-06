#pragma once

#include "servo_control/armSafetyGuard.h"

// Trajectory exhaustion alone is never evidence that the mechanism reached home.
class HomingMonitor {
public:
    enum class Result { running, reached, finished_unverified, fault };
    explicit HomingMonitor(const std::array<double, 14>& target, double tolerance)
        : target_(target), tolerance_(tolerance) {}

    Result update(ArmSafetyGuard& safety, double now, bool trajectory_finished) {
        safety.check_feedback(now);
        if (!safety.fault.empty()) return Result::fault;
        if (!safety.ready(now)) {
            safety.trip("homing_feedback_not_ready");
            return Result::fault;
        }
        if (!trajectory_finished) return Result::running;
        if (safety.limits.speed_only) return Result::finished_unverified;
        if (finished_at_ < 0) finished_at_ = now;
        bool at_target = true;
        size_t worst = 0;
        double max_error = -1;
        for (size_t i = 0; i < 14; ++i) {
            const auto& f = safety.feedback[i];
            const double error = std::abs(f.position - target_[i]);
            if (error > max_error) { max_error = error; worst = i; }
            if (error > tolerance_ || std::abs(f.velocity) > 0.05) at_target = false;
        }
        if (at_target) {
            if (settled_at_ < 0) settled_at_ = now;
            if (now - settled_at_ >= 0.2) return Result::reached;
        } else settled_at_ = -1;
        if (now - finished_at_ >= 2.0) {
            safety.trip("homing_settle_timeout joint_" + std::to_string(worst) +
                " error_rad=" + std::to_string(max_error) + " tolerance_rad=" + std::to_string(tolerance_) +
                " required_all_abs_velocity<=0.05_rad/s_for_0.2s");
            return Result::fault;
        }
        return Result::running;
    }
private:
    std::array<double, 14> target_;
    double tolerance_, finished_at_ = -1, settled_at_ = -1;
};
