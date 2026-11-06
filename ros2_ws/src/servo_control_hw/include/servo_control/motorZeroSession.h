#pragma once

#include <string>
#include <algorithm>
#include "servo_control/motorFeedbackQuery.h"

struct MotorZeroTarget {
    int channel, motor_id;
    static MotorZeroTarget from_name(const std::string& name) {
        static const char* joints[] = {"shoulder_pitch", "shoulder_roll", "shoulder_yaw",
            "elbow", "wrist_roll", "wrist_pitch", "wrist_yaw"};
        for (int side = 0; side < 2; ++side)
            for (int j = 0; j < 7; ++j)
                if (name == std::string(side == 0 ? "left_" : "right_") + joints[j] + "_joint")
                    return {side, j + 1};
        throw std::invalid_argument("motor_zero_joint must name exactly one left/right arm joint");
    }
};

// Single-shot internal-zero operation using raw ACK5 angles, independent of offsets.
class MotorZeroSession {
public:
    enum class Phase { measuring, send_zero, awaiting_ack, verifying, done, failed };
    explicit MotorZeroSession(unsigned motor_id, double now) : id_(motor_id), phase_start_(now) {
        if (motor_id < 1 || motor_id > 7) throw std::invalid_argument("Arm motor ID must be 1..7");
    }
    Phase phase = Phase::measuring;
    std::string error;
    bool sent = false, acknowledged = false;
    double before_deg = 0, after_deg = 0;

    MotorBlock next(double now) {
        if (phase == Phase::done || phase == Phase::failed) return {};
        if (now - phase_start_ > 2.0) {
            fail(sent ? "Zero may have changed; acknowledgement/readback timed out. Do not retry blindly."
                      : "No stable raw position feedback before zero; nothing was written.");
            return {};
        }
        if (phase == Phase::send_zero) {
            MotorBlock block{};
            MotorZeroSetting(&block, id_);
            sent = true;
            phase = Phase::awaiting_ack;
            phase_start_ = now;
            return block;
        }
        if (phase == Phase::awaiting_ack) return {};  // Never resend a zero request.
        return MotorFeedbackQuery::request(id_, param_get_pos);
    }

    void receive(const MotorBlock& block, bool valid_wkc, double now) {
        if (!valid_wkc) return;
        if (phase == Phase::awaiting_ack) {
            if (block.id != 0x7ff || block.dlc != 4 ||
                ((unsigned(block.data[0]) << 8) | block.data[1]) != id_ ||
                block.data[2] != 1 || block.data[3] != 3) return;
            acknowledged = true;
            phase = Phase::verifying;
            phase_start_ = now;
            count_ = 0;
            return;
        }
        if (phase != Phase::measuring && phase != Phase::verifying) return;
        MotorFeedbackQuery sample;
        if (!sample.accept(block, id_, true, now) || !sample.fresh(0, now)) return;
        if (sample.error[0]) {
            fail("Motor error in raw position response: " + std::to_string(sample.error[0]));
            return;
        }
        const double q = sample.value[0];
        if (phase == Phase::measuring) {
            if (!count_) { first_sample_ = now; min_ = max_ = q; }
            min_ = std::min(min_, q); max_ = std::max(max_, q);
            if (max_ - min_ > 0.2) { fail("Motor position is moving (>0.2 deg); nothing was written."); return; }
            before_deg = q;
            if (++count_ >= 10 && now - first_sample_ >= 0.2) phase = Phase::send_zero;
        } else {
            after_deg = q;
            if (std::abs(q) > 0.5) { count_ = 0; return; }
            if (!count_) first_sample_ = now;
            if (++count_ >= 5 && now - first_sample_ >= 0.1) phase = Phase::done;
        }
    }
private:
    void fail(const std::string& reason) { error = reason; phase = Phase::failed; }
    unsigned id_, count_ = 0;
    double phase_start_, first_sample_ = 0, min_ = 0, max_ = 0;
};
