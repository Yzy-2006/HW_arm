#pragma once

#include <stdexcept>
#include "rclcpp/rclcpp.hpp"

inline void sleep_while_running(rclcpp::Rate& rate) {
    if (!rclcpp::ok()) return;
    try {
        rate.sleep();
    } catch (const std::runtime_error&) {
        // SIGINT can invalidate the context between ok() and Rate::sleep().
        if (rclcpp::ok()) throw;
    }
}
