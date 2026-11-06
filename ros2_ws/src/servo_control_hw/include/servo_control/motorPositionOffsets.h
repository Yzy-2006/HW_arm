#pragma once

#include <cmath>
#include <stdexcept>
#include <yaml-cpp/yaml.h>

// Offset is in motor coordinates, after direction compensation. No angle wrapping.
class MotorPositionOffsets {
public:
    void load(const YAML::Node& config) {
        const auto section = config["motorPositionOffsets"];
        if (section && !section.IsMap())
            throw std::runtime_error("motorPositionOffsets must be a mapping");
        const double degrees = section && section["left_shoulder_roll_deg"]
            ? section["left_shoulder_roll_deg"].as<double>() : 0.0;
        if (!std::isfinite(degrees) || std::abs(degrees) > 360.0)
            throw std::runtime_error("motorPositionOffsets.left_shoulder_roll_deg must be finite within [-360,360]");
        left_shoulder_roll_rad_ = degrees * pi / 180.0;
    }
    double offset(int channel, int motor_id) const {
        return channel == 0 && motor_id == 2 ? left_shoulder_roll_rad_ : 0.0;
    }
    double to_motor(int channel, int motor_id, int direction, double joint_position) const {
        return direction * joint_position + offset(channel, motor_id);
    }
    double from_motor(int channel, int motor_id, int direction, double motor_position) const {
        return direction * (motor_position - offset(channel, motor_id));
    }
    static constexpr double pi = 3.14159265358979323846;
private:
    double left_shoulder_roll_rad_ = 0;
};
