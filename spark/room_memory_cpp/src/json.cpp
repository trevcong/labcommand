// Canonical v2 JSON, written by hand so key order and number formatting match the Swift
// encoder (and the Python port) byte for byte.
#include <bit>
#include <cstdio>
#include <stdexcept>
#include <string>

#include "room_memory/change_detection.hpp"

namespace room_memory {

std::uint32_t float_bits(float value) { return std::bit_cast<std::uint32_t>(value); }

float bits_float(std::uint32_t bits) { return std::bit_cast<float>(bits); }

namespace {

std::string hex_bits(float value) {
    char buffer[9];
    std::snprintf(buffer, sizeof buffer, "%08x", float_bits(value));
    return buffer;
}

std::string point(const Vec3& values) {
    return "[\"" + hex_bits(values[0]) + "\",\"" + hex_bits(values[1]) + "\",\"" + hex_bits(values[2]) + "\"]";
}

const char* boolean(bool value) { return value ? "true" : "false"; }

std::string map_record(const OccupancyMap& map) {
    return "{\"points\":" + std::to_string(map.accepted_points + map.dropped_points) +
           ",\"accepted\":" + std::to_string(map.accepted_points) + ",\"dropped\":" + std::to_string(map.dropped_points) +
           ",\"voxels\":" + std::to_string(map.occupied.size()) + "}";
}

std::string region_record(const Region& region) {
    std::string text = std::string("{\"kind\":\"") + kind_name(region.kind) +
                       "\",\"voxel_count\":" + std::to_string(region.voxel_count) + ",\"min_key\":\"" +
                       std::to_string(region.min_key) + "\"" + ",\"centroid_bits\":" + point(region.centroid) +
                       ",\"bounds_min_bits\":" + point(region.bounds_min) + ",\"bounds_max_bits\":" +
                       point(region.bounds_max) + ",\"volume_bits\":\"" + hex_bits(region.volume) + "\"";
    if (region.kind == Region::Kind::moved) {
        text += ",\"from_voxel_count\":" + std::to_string(region.from_voxel_count) + ",\"from_min_key\":\"" +
                std::to_string(region.from_min_key) + "\"" + ",\"from_centroid_bits\":" + point(region.from_centroid) +
                ",\"from_bounds_min_bits\":" + point(region.from_bounds_min) +
                ",\"from_bounds_max_bits\":" + point(region.from_bounds_max);
    }
    return text + "}";
}

}  // namespace

std::string to_json(const Report& report, const OccupancyMap& before, const OccupancyMap& after, const Config& config) {
    if (!report.unknown_voxels) throw std::invalid_argument("to_json needs a report of DiffScope::full");
    const std::string max_range = config.max_range ? "\"" + hex_bits(*config.max_range) + "\"" : "null";
    std::string text = "{\"schema\":\"labcommand.change-detection.v2\"";
    text += std::string(",\"evidence\":") + boolean(config.evidence_of_absence);
    text += std::string(",\"report_unobserved\":") + boolean(config.report_unobserved);
    text += ",\"origin_bits\":" + point(config.origin);
    text += ",\"ray_stride\":" + std::to_string(config.ray_stride);
    text += ",\"config\":{";
    text += "\"voxel_size_bits\":\"" + hex_bits(config.voxel_size) + "\"";
    text += ",\"max_range_bits\":" + max_range;
    text += ",\"min_points\":" + std::to_string(config.min_points);
    text += ",\"tolerance\":" + std::to_string(config.tolerance);
    text += ",\"min_voxels\":" + std::to_string(config.min_voxels);
    text += ",\"min_extent\":" + std::to_string(config.min_extent);
    text += std::string(",\"pair_moves\":") + boolean(config.pair_moves);
    text += ",\"max_move_bits\":\"" + hex_bits(config.max_move) + "\"";
    text += ",\"move_count_ratio_bits\":\"" + hex_bits(config.move_count_ratio) + "\"";
    text += "}";
    text += ",\"before\":" + map_record(before);
    text += ",\"after\":" + map_record(after);
    text += ",\"appeared_voxels\":" + std::to_string(report.appeared_voxels);
    text += ",\"disappeared_voxels\":" + std::to_string(report.disappeared_voxels);
    text += ",\"unknown_voxels\":" + std::to_string(*report.unknown_voxels);
    text += ",\"regions\":[";
    for (std::size_t n = 0; n < report.regions.size(); ++n) {
        if (n) text += ",";
        text += region_record(report.regions[n]);
    }
    text += "]}";
    return text;
}

}  // namespace room_memory
