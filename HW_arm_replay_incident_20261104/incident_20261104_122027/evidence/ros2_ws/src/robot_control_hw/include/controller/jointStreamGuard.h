#pragma once

#include <vector>

// State for the continuous absolute-joint bridge only. Times are monotonic seconds.
class JointStreamGuard {
public:
    struct Command {
        std::vector<double> left, right;
        bool reanchor = false;
    };

    void receive(const std::vector<double>& left, const std::vector<double>& right,
                 bool hold, bool reanchor, double now) {
        last_received_ = now;
        hold_requested_ = hold;
        if (!hold) {
            command_.left = left;
            command_.right = right;
            command_.reanchor = command_.reanchor || reanchor || holding_ || !received_;
            holding_ = false;
        }
        received_ = true;
    }

    Command sample(const std::vector<double>& left_feedback,
                   const std::vector<double>& right_feedback, double now) {
        if (hold_requested_ || now - last_received_ > timeout_s) {
            if (!holding_) {
                command_ = {left_feedback, right_feedback, true};
                holding_ = true;
            }
        }
        const auto result = command_;
        command_.reanchor = false;
        return result;
    }

    static constexpr double timeout_s = 0.25;

private:
    Command command_;
    double last_received_ = 0.0;
    bool received_ = false;
    bool hold_requested_ = false;
    bool holding_ = false;
};
