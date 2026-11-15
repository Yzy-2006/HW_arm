#pragma once

#include <cmath>
#include <stdexcept>
#include <string>
#include <yaml-cpp/yaml.h>

class PolicyHomeGuard {
public:
    double resume_delay_s = 3.0;
    bool active = false;

    void load(const YAML::Node& config) {
        const auto node = config["policyHome"];
        auto read = [&](const char* key, double& value) {
            if (node && node[key]) value = node[key].as<double>();
            if (!std::isfinite(value) || value <= 0)
                throw std::runtime_error(std::string("Invalid policyHome.") + key);
        };
        read("resume_delay_s", resume_delay_s);
    }

    void begin() { active = true; finished_since_ = -1; }
    void finish() { active = false; finished_since_ = -1; }

    bool ready(double now, bool trajectory_active) {
        const bool trajectory_finished = active && std::isfinite(now) && !trajectory_active;
        if (!trajectory_finished) finished_since_ = -1;
        else if (finished_since_ < 0 || now < finished_since_) finished_since_ = now;
        return trajectory_finished && now - finished_since_ >= resume_delay_s;
    }

private:
    double finished_since_ = -1;
};
