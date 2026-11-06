#pragma once

#include <cstdint>
#include <optional>
#include <tuple>
#include <vector>

// Identity changes trigger a handoff; ordinary trajectory samples do not.
struct ControlSource {
    std::vector<uint8_t> publisher;
    int control_mode, interpolation_mode;
    bool bridge_stream, derivatives_valid;
    std::vector<int32_t> left_modes, right_modes;

    bool operator==(const ControlSource& other) const {
        return std::tie(publisher, control_mode, interpolation_mode, bridge_stream,
                        derivatives_valid, left_modes, right_modes) ==
               std::tie(other.publisher, other.control_mode, other.interpolation_mode,
                        other.bridge_stream, other.derivatives_valid, other.left_modes, other.right_modes);
    }
};

class ControlHandoffEpoch {
public:
    bool changed(const ControlSource& source) const { return !source_ || !(*source_ == source); }
    bool observe(const ControlSource& source) {
        if (!changed(source)) return false;
        source_ = source;
        ++epoch_;
        return true;
    }
    uint64_t value() const { return epoch_; }
private:
    std::optional<ControlSource> source_;
    uint64_t epoch_ = 0;
};
