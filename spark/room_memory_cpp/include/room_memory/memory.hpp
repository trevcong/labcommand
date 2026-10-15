// Three-state voxel memory and the window engine the ROS node drives: the C++ port of
// room_memory/memory.py. No ROS dependency.
//
// A VoxelLayer holds two sorted key sets, occupied within known: free is known minus
// occupied, and every other voxel is UNKNOWN. A window's free space comes from the core's
// spec DDA from the sensor origin. Each sensor keeps its own reference and recent layers, in
// its own frame, until the JT128 <-> Mid-360 extrinsic exists (brief 5 R1). The reference is
// the settled part of per-voxel window counts (VoxelStats); one sensor's reference is the map
// the node publishes as MapBricks snapshots and deltas.
#pragma once

#include <algorithm>
#include <cstdint>
#include <deque>
#include <filesystem>
#include <map>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <utility>
#include <vector>

#include "room_memory/change_detection.hpp"
#include "room_memory/key_set.hpp"
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

// Thresholds that turn per-voxel window counts into reference evidence.
struct SettleRule {
    std::int64_t occupied_min_hits = 1;
    double occupied_min_ratio = 0.0;
    std::int64_t free_min_seen = 1;
    double free_max_ratio = 0.0;
};

// Per-voxel window counts: `seen` counts the windows that observed the voxel (occupied or
// ray-free), `hits` the windows in which it was occupied. Sorted unique keys, parallel counts.
struct VoxelStats {
    Keys keys;
    std::vector<std::uint32_t> hits;
    std::vector<std::uint32_t> seen;

    // Counts one window: every voxel of `known` once in seen, those of `occupied` (a subset)
    // also in hits.
    void add(const Keys& occupied, const Keys& known);
    // The voxels the rule settles: occupied when hits >= occupied_min_hits and
    // hits >= occupied_min_ratio * seen; else free when seen >= free_min_seen and
    // hits <= free_max_ratio * seen. Every other counted voxel stays UNKNOWN.
    VoxelLayer settled(const SettleRule& rule) const;
    // Drops the counts of the sorted keys `drop`.
    void erase(const Keys& drop);
};

// What turns the canonical brick list `before` into `after`: the bricks to send (new or
// changed) and the indices into `before` of the bricks to remove (absent from `after`).
struct MapDelta {
    std::vector<msg::Brick> bricks;
    std::vector<std::int64_t> removed;
};
MapDelta map_delta(std::span<const msg::Brick> before, std::span<const msg::Brick> after);

// <state_dir>/<room_id>/reference-<sensor>.cdr, with a leading ~ expanded. Throws
// std::invalid_argument unless room_id is a plain name ([A-Za-z0-9._-], no leading dot).
std::filesystem::path reference_path(const std::string& state_dir, const std::string& room_id, const std::string& sensor);
// Writes `layer` as one zlib MapBricks snapshot, atomically (temp file, fsync, rename).
void save_reference(const std::filesystem::path& path, const VoxelLayer& layer, const msg::Header& header,
                    float voxel_size);
// Reads a saved reference. Throws msg::CDRError or std::runtime_error.
std::pair<msg::MapBricks, VoxelLayer> load_reference(const std::filesystem::path& path);

// One change region followed across windows: pending until it has persisted, then an open
// candidate (CANDIDATE sent, no terminal transition yet).
struct Candidate {
    std::uint64_t event_id = 0;  // 0 while pending
    msg::EventKind kind = msg::EventKind::appeared;
    Keys keys;
    Region region;
    msg::Time first_seen;
    msg::Time last_seen;
    std::int64_t windows = 0;  // windows with fresh support in its unbroken run of matched windows
    std::int64_t missed = 0;   // consecutive windows without a match
};

// Every voxel of a region, both poses for moved, sorted.
Keys region_keys(const Region& region);

// Transition-only bookkeeping for one sensor's change regions. A region matches the first
// open, else the first pending, region of the same kind sharing a voxel that no other region
// of this window matched; otherwise it starts a pending one. A region is fresh when this
// window itself supports it (update's `fresh`); a stale one keeps a match alive but neither
// counts toward promotion nor starts a pending region. An appeared or disappeared region that
// matches no open entry of its kind but shares a voxel with an open moved candidate (either
// pose) is that move seen as its two halves: it keeps the move alive and starts nothing. A
// pending region becomes a CANDIDATE (and gets its event id) once `candidate_windows` fresh
// windows have matched it without a break, and is dropped silently the first window it is not
// matched. An open candidate goes
// EXPIRED after `expire_windows` consecutive windows without a match. A matched candidate
// sends nothing.
class CandidateTracker {
public:
    CandidateTracker(std::int64_t candidate_windows = 1, std::int64_t expire_windows = 1)
        : candidate_windows_(candidate_windows), expire_windows_(expire_windows) {}
    // (state, candidate) in a fixed order: new candidates in region order, then expiries by
    // event id. `fresh` holds one flag per region, empty when every region is fresh.
    template <typename NextId>
    std::vector<std::pair<msg::EventState, Candidate>> update(const std::vector<Region>& regions, const msg::Time& stamp,
                                                              NextId&& next_id, const std::vector<bool>& fresh = {});
    const std::vector<Candidate>& open() const { return open_; }
    const std::vector<Candidate>& pending() const { return pending_; }

private:
    std::int64_t candidate_windows_;
    std::int64_t expire_windows_;
    std::vector<Candidate> open_;
    std::vector<Candidate> pending_;
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
    // Defaults measured on the static recording (PLAN.md, "False change events").
    std::int64_t reference_windows = 30;
    std::int64_t recent_windows = 3;
    std::optional<std::string> state_dir = std::string("~/room-memory");
    // Reference evidence from per-voxel window counts (VoxelStats::settled).
    std::int64_t occupied_min_hits = 2;
    double occupied_min_ratio = 0.3;
    std::int64_t free_min_seen = 2;
    double free_max_ratio = 0.0;
    // Change gate: windows a region must persist before its CANDIDATE, and windows an open
    // candidate may go unmatched before it EXPIRES.
    std::int64_t candidate_windows = 3;
    std::int64_t expire_windows = 3;
    // A tracking window adds to the counts of unsettled voxels only while its pose offset is
    // at most this (meters), so a moved rig never teaches the reference its new view.
    double learn_max_offset_m = 0.02;
    // Map stream: the sensor whose reference is published (empty: jt128 when configured,
    // else the first configured sensor),
    // the minimum time between map messages, the maximum time between snapshots, and the
    // share of the last snapshot's size the deltas since it may reach before a snapshot
    // replaces the next delta.
    std::string map_sensor;
    double delta_period_s = 5.0;
    double keyframe_period_s = 60.0;
    double keyframe_delta_ratio = 0.5;

    // The core config: evidence on, sensor origin (0, 0, 0) in each sensor's frame.
    Config change_config() const;
    SettleRule settle_rule() const;
};

// One published message kept for RequestSync replays.
struct Logged {
    std::uint64_t seq = 0;
    std::uint64_t map_epoch = 0;
    msg::Bytes bytes;
};

// The protocol's replay depths (section 5).
inline constexpr std::size_t kMapLogDepth = 10;
inline constexpr std::size_t kEventLogDepth = 100;
// pose_residual_m: the registration search reaches this many voxels along each axis.
inline constexpr std::int64_t kPoseSearchVoxels = 4;

// The translation (meters) that best registers `recent` (occupied keys) onto the reference
// occupancy, as one number: for each axis, the shift of -kPoseSearchVoxels .. kPoseSearchVoxels
// voxels that maximizes the count of recent voxels landing on reference-occupied ones, refined
// by the parabola through that count and its two neighbours (not at the ends of the range);
// the result is the norm of the three per-axis shifts. Empty when no shift overlaps at all.
// Binary64 like the Python oracle, so the learning gate decides alike; status rounds to float.
std::optional<double> pose_offset(const Keys& recent, const Membership& reference, double voxel_size);

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
        VoxelStats stats;  // every voxel while BUILDING; the not yet settled ones while TRACKING
        std::int64_t reference_windows = 0;  // windows with usable points counted into the reference
        std::deque<CellSummary> recent;      // one entry per closed window, empty when it had none
        CandidateTracker tracker;
        std::optional<std::int64_t> hz;
        std::optional<double> residual;      // pose_offset of the last tracking window with points
        std::shared_ptr<const Membership> reference_index;  // over reference.occupied
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
    // events_seq is the seq of the newest event message, empty before the first.
    msg::MemoryStatus status(const msg::Time& stamp, double period_s);
    // The live MapBricks message due at `stamp`, if any (call once per window, after
    // close_window). Nothing while BUILDING. The first message is a snapshot. Afterwards,
    // once delta_period_s has passed since the last message, voxels that settled since are
    // added to the references (map_epoch + 1, references saved) and sent as a delta of the
    // changed map bricks; a snapshot instead when keyframe_period_s has passed since the last
    // one, or when the deltas since it would exceed keyframe_delta_ratio of its size.
    std::optional<msg::Bytes> map_update(const msg::Time& stamp);
    // The answer to one RequestSync (protocol section 5), to publish in order on the sync
    // topic of request.topic. Empty for another room_id, and for a map request before the
    // first live map message.
    std::vector<msg::Bytes> sync(const msg::RequestSync& request, const msg::Time& stamp);
    // Writes every sensor's reference with the current map_epoch. No-op without state_dir.
    void save(const msg::Time& stamp);

    const MemoryConfig& config() const { return config_; }
    msg::MemoryState state() const { return state_; }
    std::uint64_t map_epoch() const { return map_epoch_; }
    std::int64_t dropped_windows() const { return dropped_windows_; }
    const Channel& channel(const std::string& sensor) const;
    const Channel& map_channel() const { return channels_[map_channel_]; }

private:
    Channel& find(const std::string& sensor);
    // A built reference: its quota of windows and some settled occupancy, which tracking needs
    // (pose_offset registers on it, and without it no window may teach the reference).
    bool reference_ready(const Channel& channel) const;
    std::vector<std::filesystem::path> paths() const;
    void load();
    std::uint64_t next_id();
    msg::ChangeEvent event(const Channel& channel, msg::EventState state, const Candidate& candidate,
                           const msg::Time& stamp);
    void set_reference(Channel& channel, VoxelLayer reference);
    msg::MapBricks map_message(msg::MapKind kind, const msg::Time& stamp) const;
    msg::Bytes publish_map(const msg::MapBricks& message);
    void log_event(const msg::ChangeEvent& event);

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
    std::size_t map_channel_ = 0;
    // Map stream: the bricks and epoch of the last live map message, its seq counter and
    // timing, and the bytes sent since the last snapshot.
    bool map_started_ = false;
    std::vector<msg::Brick> map_bricks_;
    std::uint64_t map_epoch_sent_ = 0;
    std::uint64_t map_seq_ = 0;
    std::int64_t last_map_ns_ = 0;
    std::int64_t last_keyframe_ns_ = 0;
    std::size_t keyframe_bytes_ = 0;
    std::size_t delta_bytes_ = 0;
    std::deque<Logged> map_log_;
    std::deque<Logged> event_log_;
    std::map<std::uint64_t, Logged> open_latest_;  // event_id -> latest message, open events only
    std::optional<float> coverage_;
};

template <typename NextId>
std::vector<std::pair<msg::EventState, Candidate>> CandidateTracker::update(const std::vector<Region>& regions,
                                                                            const msg::Time& stamp, NextId&& next_id,
                                                                            const std::vector<bool>& fresh) {
    std::vector<std::pair<msg::EventState, Candidate>> transitions;
    std::vector<bool> open_matched(open_.size(), false);
    std::vector<bool> pending_matched(pending_.size(), false);
    std::vector<bool> pending_promoted(pending_.size(), false);
    std::vector<bool> absorbed(open_.size(), false);
    std::vector<Candidate> promoted;
    std::vector<Candidate> started;
    // Index of the first unmatched entry of `list` of this kind sharing a voxel, marked matched.
    const auto first = [](const std::vector<Candidate>& list, std::vector<bool>& matched, msg::EventKind kind,
                          const Keys& keys) -> std::optional<std::size_t> {
        for (std::size_t n = 0; n < list.size(); ++n) {
            if (!matched[n] && list[n].kind == kind && intersects(list[n].keys, keys)) {
                matched[n] = true;
                return n;
            }
        }
        return std::nullopt;
    };
    // Index of the first open moved candidate sharing a voxel with `keys`.
    const auto whole_move = [&](const Keys& keys) -> std::optional<std::size_t> {
        for (std::size_t n = 0; n < open_.size(); ++n) {
            if (open_[n].kind == msg::EventKind::moved && intersects(open_[n].keys, keys)) return n;
        }
        return std::nullopt;
    };
    const auto promote = [&](Candidate c) {
        c.event_id = next_id();
        transitions.emplace_back(msg::EventState::candidate, c);
        promoted.push_back(std::move(c));
    };
    for (std::size_t r = 0; r < regions.size(); ++r) {
        const Region& region = regions[r];
        msg::EventKind kind;
        switch (region.kind) {
            case Region::Kind::appeared: kind = msg::EventKind::appeared; break;
            case Region::Kind::disappeared: kind = msg::EventKind::disappeared; break;
            case Region::Kind::moved: kind = msg::EventKind::moved; break;
            default: continue;
        }
        const bool supported = fresh.empty() || fresh[r];
        Keys keys = region_keys(region);
        if (const auto n = first(open_, open_matched, kind, keys)) {
            Candidate& match = open_[*n];
            match.keys = std::move(keys);
            match.region = region;
            match.last_seen = stamp;
            match.missed = 0;
        } else if (const auto w = kind != msg::EventKind::moved ? whole_move(keys) : std::nullopt) {
            // A move reported as its two halves (the diff's pairing gates flicker) is still that move.
            absorbed[*w] = true;
            open_[*w].last_seen = stamp;
            open_[*w].missed = 0;
        } else if (const auto m = first(pending_, pending_matched, kind, keys)) {
            Candidate& waiting = pending_[*m];
            waiting.keys = std::move(keys);
            waiting.region = region;
            waiting.last_seen = stamp;
            if (supported) ++waiting.windows;
            if (waiting.windows >= candidate_windows_) {
                pending_promoted[*m] = true;
                promote(waiting);
            }
        } else if (!supported) {
            continue;
        } else if (candidate_windows_ <= 1) {
            promote(Candidate{0, kind, std::move(keys), region, stamp, stamp, 1, 0});
        } else {
            started.push_back(Candidate{0, kind, std::move(keys), region, stamp, stamp, 1, 0});
        }
    }
    std::vector<std::size_t> by_id(open_.size());
    for (std::size_t n = 0; n < by_id.size(); ++n) by_id[n] = n;
    std::sort(by_id.begin(), by_id.end(), [&](std::size_t a, std::size_t b) { return open_[a].event_id < open_[b].event_id; });
    std::vector<bool> expired(open_.size(), false);
    for (const std::size_t n : by_id) {
        if (!open_matched[n] && !absorbed[n] && ++open_[n].missed >= expire_windows_) {
            expired[n] = true;
            transitions.emplace_back(msg::EventState::expired, open_[n]);
        }
    }
    std::vector<Candidate> still_open;
    for (std::size_t n = 0; n < open_.size(); ++n) {
        if (!expired[n]) still_open.push_back(std::move(open_[n]));
    }
    for (Candidate& c : promoted) still_open.push_back(std::move(c));
    open_ = std::move(still_open);
    std::vector<Candidate> still_pending;
    for (std::size_t n = 0; n < pending_.size(); ++n) {
        if (pending_matched[n] && !pending_promoted[n]) still_pending.push_back(std::move(pending_[n]));
    }
    for (Candidate& c : started) still_pending.push_back(std::move(c));
    pending_ = std::move(still_pending);
    return transitions;
}

}  // namespace room_memory
