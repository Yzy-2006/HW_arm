#pragma once

#include <array>
#include <cmath>
#include <cstring>
#include <stdexcept>
#include "servo_control/motor_control.h"

// ACK5 query caches are separate for every CAN channel and motor slot.
struct MotorFeedbackQuery {
    std::array<float, 3> value{};
    std::array<double, 3> received{{-1, -1, -1}};
    std::array<unsigned, 3> error{};

    static MotorBlock request(unsigned id, unsigned parameter) {
        if (id < 1 || id > 0x7fe || parameter < 1 || parameter > 3)
            throw std::invalid_argument("Invalid feedback query");
        MotorBlock block{};
        get_motor_parameter(&block, id, parameter);
        return block;
    }

    bool accept(const MotorBlock& block, unsigned expected_id, bool valid_wkc, double now) {
        if (!valid_wkc || block.id != expected_id || block.dlc != 6 || (block.data[0] >> 5) != 5 ||
            block.data[1] < 1 || block.data[1] > 3) return false;
        const uint32_t bits = (uint32_t(block.data[2]) << 24) | (uint32_t(block.data[3]) << 16) |
                              (uint32_t(block.data[4]) << 8) | uint32_t(block.data[5]);
        float decoded;
        static_assert(sizeof(decoded) == sizeof(bits));
        std::memcpy(&decoded, &bits, sizeof(decoded));
        if (!std::isfinite(decoded)) return false;
        const size_t index = block.data[1] - 1;
        value[index] = decoded;
        received[index] = now;
        error[index] = block.data[0] & 0x1f;
        return true;
    }

    bool fresh(size_t field, double now) const {
        return received[field] >= 0 && now >= received[field] && now - received[field] <= 0.5;
    }
};
