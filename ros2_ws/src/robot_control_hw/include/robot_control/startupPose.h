#pragma once

#include <cmath>
#include <stdexcept>
#include <vector>

inline bool outside_home_pose(const std::vector<double>& feedback,
                              const std::vector<double>& home, double tolerance) {
    if (feedback.size() != 7 || home.size() != 7 || !std::isfinite(tolerance) || tolerance <= 0)
        throw std::invalid_argument("Invalid startup home-pose inputs");
    bool outside = false;
    for (size_t i = 0; i < home.size(); ++i) {
        if (!std::isfinite(feedback[i]) || !std::isfinite(home[i]))
            throw std::invalid_argument("Nonfinite startup home pose");
        outside = outside || std::abs(feedback[i] - home[i]) >= tolerance;
    }
    return outside;
}
