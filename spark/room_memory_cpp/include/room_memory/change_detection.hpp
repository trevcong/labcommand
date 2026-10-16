// Occupancy diff between two clouds: the C++ port of room_memory/core/change_detection.py
// (itself the port of ChangeDetection.swift, v2). docs/CHANGE-DETECTION.md is the contract.
//
// Every value the spec calls binary32 is a `float` here, computed in the spec's operation
// order. The core must be compiled with -ffp-contract=off and without fast-math (the CMake
// target sets both PUBLIC), so no multiply-add is fused and to_json reproduces the Swift
// and Python bytes.
#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <vector>

#include "room_memory/keys.hpp"

namespace room_memory {

class RayBackend;

class ChangeDetectionError : public std::invalid_argument {
public:
    using std::invalid_argument::invalid_argument;
};

// A config field is out of range, or evidence is on without ray endpoints.
class InvalidConfig : public ChangeDetectionError {
public:
    using ChangeDetectionError::ChangeDetectionError;
};

// The two maps, or a map and config.voxel_size, differ in edge length (meters).
class VoxelSizeMismatch : public ChangeDetectionError {
public:
    using ChangeDetectionError::ChangeDetectionError;
};

using Vec3 = std::array<float, 3>;

// Knobs for occupancy, dilation, clustering, move pairing and free-space evidence. Distances
// are binary32 meters; tolerance, min_voxels and min_extent are voxel counts. Defaults match
// the Swift Config() and the Python Config.
struct Config {
    float voxel_size = 0.05f;
    std::optional<float> max_range;
    std::int64_t min_points = 1;
    std::int64_t tolerance = 1;
    std::int64_t min_voxels = 6;
    std::int64_t min_extent = 2;
    bool pair_moves = true;
    float max_move = 1.5f;
    float move_count_ratio = 2.0f;
    bool evidence_of_absence = false;
    Vec3 origin{0.0f, 0.0f, 0.0f};
    std::int64_t ray_stride = 1;
    bool report_unobserved = false;

    // f32(1) / voxel_size: one binary32 division, in voxels per meter.
    float inverse() const { return 1.0f / voxel_size; }
};

// Throws InvalidConfig when `config` is out of range (the Swift and Python checks).
void validate(const Config& config);

// Occupied voxels of one frame. `occupied` is sorted unique; `endpoints` holds x, y, z of the
// representative return of each occupied voxel when the map was built with evidence on.
struct OccupancyMap {
    float voxel_size = 0.05f;
    Keys occupied;
    std::int64_t accepted_points = 0;
    std::int64_t dropped_points = 0;
    std::optional<std::vector<float>> endpoints;
};

// Per-voxel aggregate of a point set before the min_points cut: point count and the
// representative return (the lexicographic minimum of the per-axis binary32 order keys).
// Summaries of disjoint point sets merge exactly into the summary of their concatenation,
// which is how the memory engine keeps recent windows without their points.
struct CellSummary {
    Keys keys;
    std::vector<std::uint32_t> counts;
    std::vector<std::array<std::uint32_t, 3>> order_keys;  // empty unless built with representatives
    std::int64_t accepted_points = 0;
    std::int64_t dropped_points = 0;
    bool representatives = false;
};

// `xyz` holds 3 floats per point, meters. Representatives are kept when the config has
// evidence on. Throws InvalidConfig.
CellSummary summarize(std::span<const float> xyz, const Config& config);
CellSummary merge(std::span<const CellSummary* const> parts);
OccupancyMap occupancy_from(const CellSummary& summary, const Config& config);

// Builds an occupancy map from 3 floats per point (meters). Throws InvalidConfig.
OccupancyMap occupancy_map(std::span<const float> xyz, const Config& config);

// One changed object after clustering and optional move pairing. Geometry is binary32
// meters; for moved regions the plain fields describe the appeared side and from_* the
// disappeared side. `keys` and `from_keys` hold the component's voxels, sorted.
struct Region {
    enum class Kind { moved = 0, appeared = 1, disappeared = 2, unobserved = 3 };
    Kind kind = Kind::appeared;
    std::int64_t voxel_count = 0;
    Vec3 centroid{};
    Vec3 bounds_min{};
    Vec3 bounds_max{};
    float volume = 0.0f;
    Key min_key = 0;
    std::int64_t from_voxel_count = 0;
    Key from_min_key = 0;
    Vec3 from_centroid{};
    Vec3 from_bounds_min{};
    Vec3 from_bounds_max{};
    Keys keys;
    Keys from_keys;
};

const char* kind_name(Region::Kind kind);

// Diff of two occupancy maps. Voxel counts are after dilation and before the size cuts.
// `unknown_voxels` is absent when the diff ran in the regions-only scope.
struct Report {
    std::vector<Region> regions;
    std::int64_t appeared_voxels = 0;
    std::int64_t disappeared_voxels = 0;
    std::optional<std::int64_t> unknown_voxels;
    std::int64_t before_voxels = 0;
    std::int64_t after_voxels = 0;
};

// What a diff must compute. `full` is the v2 record. `regions` serves callers that read only
// regions and the appeared/disappeared counts (the memory engine, report_unobserved off):
// the before-only candidates are dilated before the rays are cast, which gives the same
// disappeared set (dilation tests each voxel independently and the rays only filter) while
// most rays miss the much smaller keep set; unknown_voxels is then not computed.
enum class DiffScope { full, regions };

struct DiffOptions {
    // Sorted keys the before side observed (occupied or free). When set, an after-only voxel
    // is an appeared candidate only if the before side observed it (memory engine extension,
    // outside the v2 contract).
    const Keys* before_known = nullptr;
    DiffScope scope = DiffScope::full;
    const RayBackend* backend = nullptr;  // null: the CPU backend
};

// Throws InvalidConfig or VoxelSizeMismatch.
Report diff(const OccupancyMap& before, const OccupancyMap& after, const Config& config,
            const DiffOptions& options = {});

struct Comparison {
    OccupancyMap before;
    OccupancyMap after;
    Report report;
};

// Builds both maps with `config`, then diffs them.
Comparison compare(std::span<const float> before, std::span<const float> after, const Config& config);

// Kept rays: occupied after voxels passing the stride filter, and their endpoints.
struct RayEndpoints {
    Keys keys;
    std::vector<float> points;
};
RayEndpoints ray_endpoints(const OccupancyMap& after, const Config& config);

// Every cell some ray marks (the spec DDA below), occupied after voxels included; with `keep`
// only marks inside it. Sorted unique.
Keys ray_marks(const OccupancyMap& after, const Config& config, const Keys* keep = nullptr,
               const RayBackend* backend = nullptr);

// Walks the spec's 3D DDA from config.origin to every kept ray endpoint. With `keep` (sorted)
// only marks inside it are returned, otherwise every mark. Occupied after voxels are removed
// either way. Returns sorted unique keys.
Keys cast_rays(const OccupancyMap& after, const Config& config, const Keys* keep = nullptr,
               const RayBackend* backend = nullptr);

// 26-connected components with at least min_voxels voxels whose index box spans at least
// min_extent cells on every axis, ascending by min key.
struct Cluster {
    std::int64_t voxel_count = 0;
    Vec3 centroid{};
    Vec3 bounds_min{};
    Vec3 bounds_max{};
    float volume = 0.0f;
    Key min_key = 0;
    Keys keys;
};
std::vector<Cluster> clusters(std::span<const Key> keys, const Config& config);

// Canonical v2 record (labcommand.change-detection.v2), no trailing newline.
std::string to_json(const Report& report, const OccupancyMap& before, const OccupancyMap& after, const Config& config);

// Binary32 helpers shared by the JSON writer and the CLIs.
std::uint32_t float_bits(float value);
float bits_float(std::uint32_t bits);

}  // namespace room_memory
