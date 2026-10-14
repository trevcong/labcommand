// Three-state voxel memory and the window engine the ROS node drives: the C++ port of
// room_memory/memory.py. No ROS dependency.
//
// A VoxelLayer holds two sorted key sets, occupied within known: free is known minus
// occupied, and every other voxel is UNKNOWN. A window's free space comes from the core's
// spec DDA from the sensor origin. Each sensor keeps its own reference and recent layers, in
// its own frame, until the JT128 <-> Mid-360 extrinsic exists (brief 5 R1).
#pragma once

#include <algorithm>
#include <cstdint>
#include <deque>
#include <filesystem>
#include <optional>
#include <span>
#include <string>
#include <utility>
#include <vector>

#include "room_memory/change_detection.hpp"
#include "room_memory/messages.hpp"

namespace room_memory {

// One lidar: its topic, its ChangeEvent.sensors bit and its per_sensor_hz slot.
struct Sensor {
    std::string name;
    std::string topic;
    std::int64_t bit = 0;
    int hz_index = 0;
};

// mid360 (/livox/lidar, bit 1, slot 0) and jt128 (/lidar_points, bit 2, slot 1).
const std::vector<Sensor>& known_sensors();

struct VoxelLayer {
    Keys occupied;
    Keys known;

    // One window: the occupied voxels plus every voxel an after-ray marks as free. The map
    // must be built with evidence on.
    static VoxelLayer observe(const OccupancyMap& occupancy, const Config& config, const RayBackend* backend = nullptr);
    // Union of evidence; a voxel occupied in either layer stays occupied.
    VoxelLayer merge(const VoxelLayer& other) const;
    // The occupied set as the before side of a diff.
    OccupancyMap as_occupancy(float voxel_size) const;
    // Bricks with at least one known voxel (the n_bricks a snapshot would carry).
    std::int64_t brick_count() const;
    // Canonical bricks: strictly increasing (i, j, k), all-free bricks kept, all-unknown
    // omitted. Throws std::invalid_argument when a brick index does not fit int16.
    std::vector<msg::Brick> to_bricks() const;
    static VoxelLayer from_bricks(std::span<const msg::Brick> bricks);

private:
    mutable std::optional<std::int64_t> brick_count_;
};

// <state_dir>/<room_id>/reference-<sensor>.cdr, with a leading ~ expanded. Throws
// std::invalid_argument unless room_id is a plain name ([A-Za-z0-9._-], no leading dot).
std::filesystem::path reference_path(const std::string& state_dir, const std::string& room_id, const std::string& sensor);
// Writes `layer` as one zlib MapBricks snapshot, atomically (temp file, fsync, rename).
void save_reference(const std::filesystem::path& path, const VoxelLayer& layer, const msg::Header& header,
                    float voxel_size);
// Reads a saved reference. Throws msg::CDRError or std::runtime_error.
std::pair<msg::MapBricks, VoxelLayer> load_reference(const std::filesystem::path& path);

// One open change: CANDIDATE sent, no terminal transition yet.
struct Candidate {
    std::uint64_t event_id = 0;
    msg::EventKind kind = msg::EventKind::appeared;
    Keys keys;
    Region region;
    msg::Time first_seen;
    msg::Time last_seen;
};

// Every voxel of a region, both poses for moved, sorted.
Keys region_keys(const Region& region);

// Transition-only bookkeeping for one sensor's change regions. A region without an open
// candidate of the same kind sharing a voxel starts a CANDIDATE; a candidate no region of
// the next window matches goes EXPIRED; a matched candidate sends nothing.
class CandidateTracker {
public:
    // (state, candidate) in a fixed order: new candidates, then expiries by event id.
    template <typename NextId>
    std::vector<std::pair<msg::EventState, Candidate>> update(const std::vector<Region>& regions, const msg::Time& stamp,
                                                              NextId&& next_id);
    const std::vector<Candidate>& open() const { return open_; }

private:
    std::vector<Candidate> open_;
};

// Node parameters. Distances in meters, window_s in seconds.
struct MemoryConfig {
    std::string room_id = "lab";
    std::vector<std::string> sensors{"mid360", "jt128"};
    double voxel_size = 0.05;
    std::optional<double> max_range = 15.0;
    std::int64_t tolerance = 1;
    std::int64_t min_voxels = 6;
    std::int64_t min_extent = 2;
    double window_s = 1.0;
    std::int64_t reference_windows = 5;
    std::int64_t recent_windows = 1;
    std::optional<std::string> state_dir = std::string("~/room-memory");

    // The core config: evidence on, sensor origin (0, 0, 0) in each sensor's frame.
    Config change_config() const;
};

// Accumulation windows, reference/recent layers, candidates and status for one room. The
// caller feeds frames with add_cdr or add_points, calls close_window every window_s and
// status at 1 Hz, and publishes the returned messages.
class RoomMemory {
public:
    struct Channel {
        Sensor sensor;
        std::size_t frames = 0;
        std::vector<float> points;  // the current window's frames, 3 floats per point
        std::int64_t received = 0;
        std::int64_t failed = 0;
        VoxelLayer reference;
        std::int64_t reference_windows = 0;  // windows with usable points merged into the reference
        std::deque<CellSummary> recent;      // one entry per closed window, empty when it had none
        CandidateTracker tracker;
        std::optional<std::int64_t> hz;
    };

    // Throws std::invalid_argument on a bad config or inconsistent saved state. `scope` is
    // the diff scope of tracking windows: regions (the default) or full, which gives the same
    // events and exists to measure and prove that.
    RoomMemory(MemoryConfig config, const msg::Time& start, const RayBackend* backend = nullptr,
               DiffScope scope = DiffScope::regions);

    // Queues positions (3 floats per point, meters, sensor frame) for the current window.
    void add_points(const std::string& sensor, std::span<const float> xyz);
    // Decodes one CDR PointCloud2 and queues it. Returns false when the frame is rejected:
    // any exception from the decoder counts as a failed frame, never escapes.
    bool add_cdr(const std::string& sensor, std::span<const std::uint8_t> data);
    // Integrates the queued frames. Returns the ChangeEvent messages to publish, in order.
    std::vector<msg::ChangeEvent> close_window(const msg::Time& stamp);
    // MemoryStatus for the last period_s seconds; resets the per-sensor frame counts.
    msg::MemoryStatus status(const msg::Time& stamp, double period_s);
    // Writes every sensor's reference with the current map_epoch. No-op without state_dir.
    void save(const msg::Time& stamp);

    const MemoryConfig& config() const { return config_; }
    msg::MemoryState state() const { return state_; }
    std::uint64_t map_epoch() const { return map_epoch_; }
    std::int64_t dropped_windows() const { return dropped_windows_; }
    const Channel& channel(const std::string& sensor) const;

private:
    Channel& find(const std::string& sensor);
    std::vector<std::filesystem::path> paths() const;
    void load();
    std::uint64_t next_id();
    msg::ChangeEvent event(const Channel& channel, msg::EventState state, const Candidate& candidate,
                           const msg::Time& stamp);

    MemoryConfig config_;
    Config change_;
    const RayBackend* backend_;
    DiffScope scope_;
    std::vector<Channel> channels_;
    msg::MemoryState state_ = msg::MemoryState::building;
    std::uint64_t map_epoch_ = 0;
    std::uint64_t event_seq_ = 0;
    std::uint64_t status_seq_ = 0;
    std::int64_t dropped_windows_ = 0;
    bool last_window_dropped_ = false;
    std::uint64_t next_event_id_ = 0;
};

template <typename NextId>
std::vector<std::pair<msg::EventState, Candidate>> CandidateTracker::update(const std::vector<Region>& regions,
                                                                            const msg::Time& stamp, NextId&& next_id) {
    std::vector<std::pair<msg::EventState, Candidate>> transitions;
    std::vector<std::uint64_t> matched;
    const auto is_matched = [&](std::uint64_t id) {
        return std::find(matched.begin(), matched.end(), id) != matched.end();
    };
    for (const Region& region : regions) {
        msg::EventKind kind;
        switch (region.kind) {
            case Region::Kind::appeared: kind = msg::EventKind::appeared; break;
            case Region::Kind::disappeared: kind = msg::EventKind::disappeared; break;
            case Region::Kind::moved: kind = msg::EventKind::moved; break;
            default: continue;
        }
        Keys keys = region_keys(region);
        Candidate* match = nullptr;
        for (Candidate& c : open_) {
            if (!is_matched(c.event_id) && c.kind == kind && intersects(c.keys, keys)) {
                match = &c;
                break;
            }
        }
        if (!match) {
            open_.push_back(Candidate{next_id(), kind, std::move(keys), region, stamp, stamp});
            match = &open_.back();
            transitions.emplace_back(msg::EventState::candidate, *match);
        } else {
            match->keys = std::move(keys);
            match->region = region;
            match->last_seen = stamp;
        }
        matched.push_back(match->event_id);
    }
    std::vector<const Candidate*> by_id;
    for (const Candidate& c : open_) by_id.push_back(&c);
    std::sort(by_id.begin(), by_id.end(), [](const Candidate* a, const Candidate* b) { return a->event_id < b->event_id; });
    for (const Candidate* c : by_id) {
        if (!is_matched(c->event_id)) transitions.emplace_back(msg::EventState::expired, *c);
    }
    std::erase_if(open_, [&](const Candidate& c) { return !is_matched(c.event_id); });
    return transitions;
}

}  // namespace room_memory
