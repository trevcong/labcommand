// MemoryConfig from key=value words, shared by room_memory_replay and room_memory_bag. Keys are
// the field names; sensors=a,b; max_range=none and state_dir=none clear the optional.
#pragma once

#include <cstdint>
#include <sstream>
#include <stdexcept>
#include <string>

#include "room_memory/memory.hpp"

namespace room_memory::tools {

// The whole of `value` as a number; throws std::invalid_argument otherwise.
inline std::int64_t integer(const std::string& value) {
    std::size_t used = 0;
    const std::int64_t out = std::stoll(value, &used);
    if (used != value.size()) throw std::invalid_argument("not an integer: " + value);
    return out;
}

inline double real(const std::string& value) {
    std::size_t used = 0;
    const double out = std::stod(value, &used);
    if (used != value.size()) throw std::invalid_argument("not a number: " + value);
    return out;
}

inline void configure(MemoryConfig& config, const std::string& key, const std::string& value) {
    if (key == "room_id") {
        config.room_id = value;
    } else if (key == "sensors") {
        config.sensors.clear();
        std::stringstream parts(value);
        for (std::string name; std::getline(parts, name, ',');) {
            if (!name.empty()) config.sensors.push_back(name);
        }
    } else if (key == "voxel_size") {
        config.voxel_size = real(value);
    } else if (key == "max_range") {
        config.max_range = value == "none" ? std::nullopt : std::optional<double>(real(value));
    } else if (key == "tolerance") {
        config.tolerance = integer(value);
    } else if (key == "min_voxels") {
        config.min_voxels = integer(value);
    } else if (key == "min_extent") {
        config.min_extent = integer(value);
    } else if (key == "window_s") {
        config.window_s = real(value);
    } else if (key == "reference_windows") {
        config.reference_windows = integer(value);
    } else if (key == "recent_windows") {
        config.recent_windows = integer(value);
    } else if (key == "state_dir") {
        config.state_dir = value == "none" ? std::nullopt : std::optional<std::string>(value);
    } else if (key == "occupied_min_hits") {
        config.occupied_min_hits = integer(value);
    } else if (key == "occupied_min_ratio") {
        config.occupied_min_ratio = real(value);
    } else if (key == "free_min_seen") {
        config.free_min_seen = integer(value);
    } else if (key == "free_max_ratio") {
        config.free_max_ratio = real(value);
    } else if (key == "candidate_windows") {
        config.candidate_windows = integer(value);
    } else if (key == "expire_windows") {
        config.expire_windows = integer(value);
    } else if (key == "learn_max_offset_m") {
        config.learn_max_offset_m = real(value);
    } else if (key == "fresh_min_voxels") {
        config.fresh_min_voxels = integer(value);
    } else if (key == "fresh_min_fraction") {
        config.fresh_min_fraction = real(value);
    } else if (key == "stable_min_fraction") {
        config.stable_min_fraction = real(value);
    } else if (key == "activity_radius_m") {
        config.activity_radius_m = real(value);
    } else if (key == "learn_windows") {
        config.learn_windows = integer(value);
    } else if (key == "map_sensor") {
        config.map_sensor = value;
    } else if (key == "delta_period_s") {
        config.delta_period_s = real(value);
    } else if (key == "keyframe_period_s") {
        config.keyframe_period_s = real(value);
    } else if (key == "keyframe_delta_ratio") {
        config.keyframe_delta_ratio = real(value);
    } else if (key == "suppress_clear_windows") {
        config.suppress_clear_windows = integer(value);
    } else {
        throw std::runtime_error("unknown config key " + key);
    }
}

}  // namespace room_memory::tools
