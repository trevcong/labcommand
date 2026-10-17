// MemoryConfig from key=value words, shared by room_memory_replay and room_memory_bag. Keys are
// the field names; sensors=a,b; max_range=none and state_dir=none clear the optional; booleans are
// true or false.
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

inline bool boolean(const std::string& value) {
    if (value == "true") return true;
    if (value == "false") return false;
    throw std::invalid_argument("not true or false: " + value);
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
    } else if (key == "rhythm_enable") {
        config.rhythm_enable = boolean(value);
    } else if (key == "rhythm_publish") {
        config.rhythm_publish = boolean(value);
    } else if (key == "rhythm_cell_m") {
        config.rhythm_cell_m = real(value);
    } else if (key == "rhythm_dims") {
        config.rhythm_dims = integer(value);
    } else if (key == "rhythm_period_s") {
        config.rhythm_period_s = integer(value);
    } else if (key == "rhythm_dir") {
        config.rhythm_dir = value;
    } else if (key == "rhythm_keep_days") {
        config.rhythm_keep_days = integer(value);
    } else if (key == "rhythm_max_bytes") {
        config.rhythm_max_bytes = integer(value);
    } else if (key == "rhythm_max_file_bytes") {
        config.rhythm_max_file_bytes = integer(value);
    } else if (key == "rhythm_fsync_s") {
        config.rhythm_fsync_s = integer(value);
    } else if (key == "rig_guard") {
        config.rig_guard = boolean(value);
    } else if (key == "auto_relearn") {
        config.auto_relearn = boolean(value);
    } else if (key == "imu_topic") {
        config.imu_topic = value;
    } else if (key == "imu_shock_frac") {
        config.imu_shock_frac = real(value);
    } else if (key == "imu_gyro_dps") {
        config.imu_gyro_dps = real(value);
    } else if (key == "imu_quiet_s") {
        config.imu_quiet_s = real(value);
    } else if (key == "tilt_trigger_deg") {
        config.tilt_trigger_deg = real(value);
    } else if (key == "pose_trigger_m") {
        config.pose_trigger_m = real(value);
    } else if (key == "guard_check_period_s") {
        config.guard_check_period_s = real(value);
    } else if (key == "check_max_s") {
        config.check_max_s = real(value);
    } else if (key == "move_min_m") {
        config.move_min_m = real(value);
    } else if (key == "move_min_deg") {
        config.move_min_deg = real(value);
    } else if (key == "align_max_m") {
        config.align_max_m = real(value);
    } else if (key == "align_max_deg") {
        config.align_max_deg = real(value);
    } else if (key == "align_inlier_ratio") {
        config.align_inlier_ratio = real(value);
    } else if (key == "align_max_rms_m") {
        config.align_max_rms_m = real(value);
    } else if (key == "observe_min_eig") {
        config.observe_min_eig = real(value);
    } else if (key == "gross_inlier_ratio") {
        config.gross_inlier_ratio = real(value);
    } else if (key == "relearn_after_s") {
        config.relearn_after_s = real(value);
    } else if (key == "reg_voxel_m") {
        config.reg_voxel_m = real(value);
    } else if (key == "src_voxel_m") {
        config.src_voxel_m = real(value);
    } else if (key == "icp_max_range_m") {
        config.icp_max_range_m = real(value);
    } else if (key == "align_stable_windows") {
        config.align_stable_windows = integer(value);
    } else if (key == "reg_max_points") {
        config.reg_max_points = integer(value);
    } else if (key == "src_max_points") {
        config.src_max_points = integer(value);
    } else {
        throw std::runtime_error("unknown config key " + key);
    }
}

}  // namespace room_memory::tools
