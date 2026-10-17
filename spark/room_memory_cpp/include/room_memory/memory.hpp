// Three-state voxel memory and the window engine the ROS node drives: the C++ port of
// room_memory/memory.py. No ROS dependency.
//
// A VoxelLayer holds two sorted key sets, occupied within known: free is known minus
// occupied, and every other voxel is UNKNOWN. A window's free space comes from the core's
// spec DDA from the sensor origin. Each sensor keeps its own reference and recent layers, in
// its own frame, until the JT128 <-> Mid-360 extrinsic exists (brief 5 R1). The reference is
// the settled part of per-voxel window counts (VoxelStats); one sensor's reference is the map
// the node publishes as MapBricks snapshots and deltas. With rig_guard on, the rig-pose guard
// (guard.hpp, imu.hpp, state.hpp) checks each sensor's pose against its registration cloud and
// re-aligns it.
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
#include "room_memory/guard.hpp"
#include "room_memory/imu.hpp"
#include "room_memory/key_set.hpp"
#include "room_memory/messages.hpp"
#include "room_memory/state.hpp"

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

// Python's os.path.expanduser for "~" and "~user" prefixes.
std::filesystem::path expand_user(const std::string& path);
// <state_dir>/<room_id>, with a leading ~ expanded. Throws std::invalid_argument unless room_id is
// a plain name ([A-Za-z0-9._-], no leading dot).
std::filesystem::path room_dir(const std::string& state_dir, const std::string& room_id);
// <state_dir>/<room_id>/reference-<sensor>.cdr. Same room_id rule as room_dir.
std::filesystem::path reference_path(const std::string& state_dir, const std::string& room_id, const std::string& sensor);
// Writes `layer` as one zlib MapBricks snapshot, atomically (write_atomic), in the legacy layout:
// a publisher generation on disk means nothing, and an older image must still load the file
// (protocol 1.4). load_reference reads both layouts.
void save_reference(const std::filesystem::path& path, const VoxelLayer& layer, const msg::Header& header,
                    float voxel_size);
// Reads a saved reference. Throws msg::CDRError or std::runtime_error.
std::pair<msg::MapBricks, VoxelLayer> load_reference(const std::filesystem::path& path);
// <state_dir>/<room_id>/map_epoch: the room's last map_epoch, whichever sensors the node ran,
// as decimal ASCII and a newline. Same room_id rule as reference_path.
std::filesystem::path epoch_path(const std::string& state_dir, const std::string& room_id);
// Writes the epoch file, atomically (write_atomic).
void save_epoch(const std::filesystem::path& path, std::uint64_t epoch);
// Reads the epoch file: nullopt if it does not exist. Throws std::invalid_argument if it does not
// hold one uint64 as digits and a newline.
std::optional<std::uint64_t> load_epoch(const std::filesystem::path& path);

// Inclusive seq ranges sorted, with overlapping or touching ones merged.
std::vector<msg::SeqRange> canonical_ranges(std::vector<msg::SeqRange> ranges);
// Canonical `ranges` minus canonical `removed`.
std::vector<msg::SeqRange> subtract_ranges(const std::vector<msg::SeqRange>& ranges,
                                           const std::vector<msg::SeqRange>& removed);

// One change region followed across windows: pending until it has persisted, then an open
// candidate (CANDIDATE sent, no terminal transition yet).
struct Candidate {
    std::uint64_t event_id = 0;  // 0 while pending
    msg::EventKind kind = msg::EventKind::appeared;
    Keys keys;
    Region region;
    msg::Time first_seen;
    msg::Time last_seen;
    std::int64_t windows = 0;  // pending: windows in its current run (fresh, same shape, no activity nearby)
    std::int64_t missed = 0;   // consecutive windows without a match
    Keys anchor;               // pending: its (new-pose) voxels when the run started
    Keys anchor_from;          // pending: its old-pose voxels then (empty unless moved)
};

// Every voxel of a region, both poses for moved, sorted.
Keys region_keys(const Region& region);
// How many voxels of `keys` have a voxel of `other` within `tolerance` voxels along each axis
// (Chebyshev).
std::size_t count_within(const Keys& keys, const Keys& other, std::int64_t tolerance);
// At least `fraction` of each set lies within `tolerance` voxels of the other.
bool same_shape(const Keys& keys, const Keys& anchor, std::int64_t tolerance, double fraction);
// Whether one window's own evidence supports `region`: at least `min_voxels` and `min_fraction`
// of its voxels occupied by the window (appeared, the new pose of a move) or traversed by its rays
// (`free`: disappeared, the old pose of a move), each pose of a move.
bool region_fresh(const Region& region, const Keys& occupied, const Keys& free, std::int64_t min_voxels,
                  double min_fraction);
// A region of `kind` over exactly these voxels (the old pose in `from_keys`, moved only), with the
// geometry the diff's clusters give.
Region region_from_poses(Region::Kind kind, const Keys& keys, const Keys& from_keys, double voxel_size);
// Voxels between two boxes along the axis that separates them most (0 when they touch).
std::int64_t box_gap(const Box& a, const Box& b);

// Transition-only bookkeeping for one sensor's change regions. A region first matches the first
// open candidate of its kind sharing a voxel that no other region of this window matched. An
// appeared or disappeared region that matches none but shares a voxel with the same side of an
// open moved candidate (its new pose, its old pose) is a half of that move and starts nothing;
// when a window has both halves, they are the move itself and count as its match. An open
// candidate goes EXPIRED after `expire_windows` consecutive windows without a match. A matched
// open candidate sends nothing.
//
// The rest are pending, in this order: a moved region matches the first pending move sharing a
// voxel; a pending move with no such region matches an appeared region on its new-pose anchor plus
// a disappeared region on its old-pose anchor (its two halves); a moved region that matched
// nothing takes over a pending appeared on its new pose plus a pending disappeared on its old pose
// as one pending move, whose run is the shorter of theirs; an appeared or disappeared region
// matches the first pending entry of its kind sharing a voxel; a fresh appeared or disappeared
// region left over whose voxel box lies within `tolerance` voxels of the same side of a pending
// entry matched this window joins that entry (a fragment of its edge); a fresh region left over
// after that starts a pending entry. A pending entry not matched in a window is dropped, and
// interrupt (a window without usable points) drops them all.
//
// A pending entry's run counts the windows since it last settled. A window whose own evidence
// does not support it (`fresh`) sets the run to 0. A fresh window whose voxels keep the run's
// shape (each pose against its own anchor: at least `stable_fraction` of the voxels of each within
// `tolerance` voxels of the other) adds 1 (from 0, it becomes the anchor of the new run); one that
// does not (or a new entry) starts a new run at 1 with itself as anchor, and its regions are activity. Then, with `radius`, an entry that has an activity region other than its own within
// `radius` voxels (box gap, either pose) sets its run to 0: a change next to motion only counts
// once the motion has settled. No `tolerance` skips the shape check, no `radius` the activity
// check. Last, every entry whose run reached `candidate_windows` becomes a CANDIDATE (and gets its
// event id), in region order, unless `hold` is set (a rig guard check): then nothing promotes, and an
// entry whose run reached the gate promotes on the first window without the hold.
class CandidateTracker {
public:
    CandidateTracker(std::int64_t candidate_windows = 1, std::int64_t expire_windows = 1,
                     std::optional<std::int64_t> tolerance = std::nullopt, std::optional<std::int64_t> radius = std::nullopt,
                     double stable_fraction = 1.0, std::optional<double> voxel_size = std::nullopt)
        : candidate_windows_(candidate_windows),
          expire_windows_(expire_windows),
          tolerance_(tolerance),
          radius_(radius),
          stable_fraction_(stable_fraction),
          voxel_size_(voxel_size) {}
    // (state, candidate) in a fixed order: new candidates in region order, then expiries by
    // event id. `fresh` holds one flag per region, empty when every region is fresh.
    template <typename NextId>
    std::vector<std::pair<msg::EventState, Candidate>> update(const std::vector<Region>& regions, const msg::Time& stamp,
                                                              NextId&& next_id, const std::vector<bool>& fresh = {},
                                                              bool hold = false);
    // A window without usable points: no pending run bridges it.
    void interrupt() {
        pending_.clear();
        activity_.clear();
    }
    // A moved channel: every pending entry goes, and the open candidates are returned (to expire)
    // by event id.
    std::vector<Candidate> drop_all() {
        std::vector<Candidate> open = std::move(open_);
        open_.clear();
        pending_.clear();
        activity_.clear();
        std::sort(open.begin(), open.end(), [](const Candidate& a, const Candidate& b) { return a.event_id < b.event_id; });
        return open;
    }
    const std::vector<Candidate>& open() const { return open_; }
    const std::vector<Candidate>& pending() const { return pending_; }
    // Indices into the last update's regions that were activity, ascending.
    const std::vector<std::size_t>& activity() const { return activity_; }

private:
    std::int64_t candidate_windows_;
    std::int64_t expire_windows_;
    std::optional<std::int64_t> tolerance_;
    std::optional<std::int64_t> radius_;
    double stable_fraction_;
    std::optional<double> voxel_size_;  // rebuilds the region of halves and fragments (empty: keeps the last one)
    std::vector<Candidate> open_;
    std::vector<Candidate> pending_;
    std::vector<std::size_t> activity_;
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
    // Change gate: windows a region must stay settled (fresh, the same voxels within tolerance,
    // no activity within activity_radius_m) before its CANDIDATE, and windows an open candidate
    // may go unmatched before it EXPIRES. 40 windows outlast the longest settled run in the
    // static recordings, 24 windows of a person's BUILDING-time pose next to the rig (PLAN.md,
    // "Activity").
    std::int64_t candidate_windows = 40;
    std::int64_t expire_windows = 3;
    // A region is fresh in a window whose own evidence supports at least fresh_min_voxels and
    // fresh_min_fraction of its voxels (each pose of a move): occupied by the window where it
    // appeared, traversed by the window's rays where it disappeared.
    std::int64_t fresh_min_voxels = 3;
    double fresh_min_fraction = 0.5;
    // A region keeps its shape while at least stable_min_fraction of its voxels lie within tolerance
    // voxels of the voxels it had when its count started, and the other way round.
    double stable_min_fraction = 0.9;
    // Activity (new or reshaping regions) within this distance (meters, voxel box gap) of a pending
    // region restarts its count (0: never).
    double activity_radius_m = 1.0;
    // A tracking window adds to the counts of unsettled voxels only while its pose offset is
    // at most this (meters), so a moved rig never teaches the reference its new view.
    double learn_max_offset_m = 0.02;
    // Closed windows that enter TRACKING and may still add to unsettled voxels. An empty window
    // and one whose pose exceeds learn_max_offset_m count. The budget is about learn_windows *
    // window_s seconds. After it, map_update settles once more on its usual schedule, then drops
    // the counts: a voxel outside the frozen reference is not an event and never becomes one.
    // 0: tracking teaches the reference nothing. Learning while a person works next to the rig
    // writes their surroundings into the reference (PLAN.md, "Activity").
    std::int64_t learn_windows = 0;
    // Map stream: the sensor whose reference is published (empty: jt128 when configured,
    // else the first configured sensor),
    // the minimum time between map messages, the maximum time between snapshots, and the
    // share of the last snapshot's size the deltas since it may reach before a snapshot
    // replaces the next delta.
    std::string map_sensor;
    double delta_period_s = 5.0;
    double keyframe_period_s = 60.0;
    double keyframe_delta_ratio = 0.5;
    // The rig-pose guard (PLAN.md). Off: no check, transform, IMU or extra file, so outputs and
    // files are today's exactly. On: triggers (an IMU shock or tilt past tilt_trigger_deg,
    // pose_offset above pose_trigger_m on 2 windows, every guard_check_period_s, a saved-state
    // start, a sensor back after 5 s) start a check that holds promotion and learning for up to
    // check_max_s and aligns each sensor against its registration cloud (reg_voxel_m points of the
    // BUILDING windows, at most reg_max_points; the window's first point per src_voxel_m, at most
    // src_max_points, within max_range or icp_max_range_m). An estimate is fit with inlier_frac >=
    // align_inlier_ratio x its baseline, RMS <= align_max_rms_m and lambda_min / sum w >=
    // observe_min_eig; it moved when its increment reaches move_min_m or move_min_deg. Move evidence
    // (a fit move; IMU tilt; every channel below gross_inlier_ratio x baseline on 2 windows
    // unwatched by a quiet IMU) gives RIG_MOVED; align_stable_windows agreeing estimates within
    // align_max_m / align_max_deg of the frame re-align; none for relearn_after_s relearns
    // (auto_relearn). imu_topic is the node's subscription ("": none).
    bool rig_guard = false;
    std::string imu_topic = "/livox/imu";
    double imu_shock_frac = 0.05;
    double imu_gyro_dps = 2.0;
    double imu_quiet_s = 2.0;
    double tilt_trigger_deg = 0.2;
    double pose_trigger_m = 0.015;
    double guard_check_period_s = 30.0;
    double check_max_s = 10.0;
    double move_min_m = 0.015;
    double move_min_deg = 0.25;
    double align_max_m = 0.3;
    double align_max_deg = 10.0;
    double align_inlier_ratio = 0.9;
    double align_max_rms_m = 0.02;
    std::int64_t align_stable_windows = 3;
    double observe_min_eig = 0.02;
    double gross_inlier_ratio = 0.5;
    double relearn_after_s = 60.0;
    bool auto_relearn = true;
    double reg_voxel_m = 0.02;
    std::int64_t reg_max_points = 300000;
    double src_voxel_m = 0.1;
    std::int64_t src_max_points = 50000;
    double icp_max_range_m = 30.0;

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
// Rig guard: a sensor silent this long and back triggers a check; RigPose goes out at least this
// often; an ICP source needs this many points; re-alignment needs consecutive estimates this close.
inline constexpr std::int64_t kResumeSilenceNs = 5'000'000'000;
inline constexpr std::int64_t kRigPosePeriodNs = 10'000'000'000;
inline constexpr std::size_t kMinSourcePoints = 30;
inline constexpr double kStableM = 0.005;
inline constexpr double kStableDeg = 0.1;

// The translation (meters) that best registers `recent` (occupied keys) onto the reference
// occupancy, as one number: for each axis, the shift of -kPoseSearchVoxels .. kPoseSearchVoxels
// voxels that maximizes the count of recent voxels landing on reference-occupied ones, refined
// by the parabola through that count and its two neighbours (not at the ends of the range);
// the result is the norm of the three per-axis shifts. Empty when no shift overlaps at all.
// Binary64 like the Python oracle, so the learning gate decides alike; status rounds to float.
std::optional<double> pose_offset(const Keys& recent, const Membership& reference, double voxel_size);
// The same search per axis (meters): adding it to the recent points moves them onto the reference.
std::optional<rig::Vec3d> pose_shift(const Keys& recent, const Membership& reference, double voxel_size);
// The norm of a pose_shift in binary64, summed in axis order (pose_residual_m).
double shift_norm(const rig::Vec3d& shift);

// Why add_cdr rejected a frame: the error's type name and message.
struct RejectedFrame {
    std::string type_name;
    std::string message;
    friend bool operator==(const RejectedFrame&, const RejectedFrame&) = default;
};

// Accumulation windows, reference/recent layers, candidates and status for one room. The
// caller feeds frames with add_cdr or add_points, calls close_window every window_s and
// status at 1 Hz, and publishes the returned messages. publisher_generation (protocol 1.4)
// stamps every status, event and map message; the ROS node allocates it (allocate_generation),
// offline drivers and tests pass a fixed value. Seq counters start at 0 here and are never
// reset within the process.
class RoomMemory {
public:
    // The answer to one RequestSync: the CDR messages to publish in order on the sync topic of
    // the request's topic, then `end` on sync/end. No end (and no messages) for another room_id.
    struct SyncAnswer {
        std::vector<msg::Bytes> messages;
        std::optional<msg::SyncEnd> end;
    };

    struct Channel {
        Sensor sensor;
        std::size_t frames = 0;
        std::vector<float> points;  // the current window's frames, 3 floats per point
        std::int64_t received = 0;
        std::int64_t failed = 0;
        // The last rejected frame's error, cleared by an accepted one. Never the frame.
        std::optional<RejectedFrame> last_error;
        VoxelLayer reference;
        VoxelStats stats;  // every voxel while BUILDING; the not yet settled ones while TRACKING
        std::int64_t reference_windows = 0;  // windows with usable points counted into the reference
        std::deque<CellSummary> recent;      // one entry per closed window, empty when it had none
        CandidateTracker tracker;
        std::optional<std::int64_t> hz;
        std::optional<double> residual;      // pose_offset of the last tracking window with points
        std::shared_ptr<const Membership> reference_index;  // over reference.occupied
        // Rig guard. pose maps this window's points into the reference frame; config is the change
        // config for them (origin at the pose, no range clip: the transform pass clipped).
        enum class Mode { normal, moved, stale };  // moved: no diff until re-aligned; stale: moved while silent
        rig::Pose pose;
        Config config;
        std::vector<float> window;            // this window's points, sensor frame
        std::optional<rig::Vec3d> shift;      // pose_shift of the last window with points
        std::optional<rig::RegistrationBuilder> builder;
        std::shared_ptr<const rig::RegistrationCloud> cloud;  // once the reference is built
        std::optional<RegistrationFile> registration;         // as last written or loaded
        Mode mode = Mode::normal;
        std::optional<double> baseline;        // inlier_frac at the last accepted decision
        std::vector<msg::RigChannel> results;  // of the current check, in order
        std::vector<rig::Estimate> estimates;  // fit estimates while moved, the newest last
        std::optional<rig::Estimate> last_estimate;
        bool last_fit = false;
        std::optional<std::pair<double, double>> delta;  // (m, deg) of the last applied increment
        std::int64_t pose_over = 0;  // consecutive windows with pose_offset above pose_trigger_m
        std::optional<std::int64_t> silent_since;
        bool learn_off = false;  // rejected by an inconclusive check, until a check finds it in place
        bool available = false;
    };

    // Throws std::invalid_argument on a bad config, a publisher_generation of 0, or inconsistent
    // saved state. `scope` is the diff scope of tracking windows: regions (the default) or full,
    // which gives the same events and exists to measure and prove that.
    RoomMemory(MemoryConfig config, const msg::Time& start, std::uint64_t publisher_generation,
               const RayBackend* backend = nullptr, DiffScope scope = DiffScope::regions);

    // Queues positions (3 floats per point, meters, sensor frame) for the current window.
    void add_points(const std::string& sensor, std::span<const float> xyz);
    // Decodes one CDR PointCloud2 and queues it. Returns false when the frame is rejected:
    // any exception from the decoder counts as a failed frame, never escapes, and is kept as
    // the channel's last_error.
    bool add_cdr(const std::string& sensor, std::span<const std::uint8_t> data);
    // Integrates the queued frames. Returns the ChangeEvent messages to publish, in order.
    std::vector<msg::ChangeEvent> close_window(const msg::Time& stamp);
    // The Activity message at `stamp` (call at 1 Hz): the newest tracking window with activity
    // since the previous call, if any, and the time since the newest one ever.
    msg::Activity activity(const msg::Time& stamp);
    // MemoryStatus for the last period_s seconds; resets the per-sensor frame counts.
    // events_seq is the seq of the newest event message of this generation, empty before the first.
    msg::MemoryStatus status(const msg::Time& stamp, double period_s);
    // The live MapBricks message due at `stamp`, if any (call once per window, after
    // close_window). Nothing while BUILDING. The first message is a snapshot. Afterwards,
    // once delta_period_s has passed since the last message, voxels that settled since are
    // added to the references (map_epoch + 1, references saved) and sent as a delta of the
    // changed map bricks; a snapshot instead when keyframe_period_s has passed since the last
    // one, or when the deltas since it would exceed keyframe_delta_ratio of its size.
    // After learn_windows tracking windows, that settle pass drops every sensor's counts.
    // Later ticks do not settle, and a keyframe of the unchanged map does not bump map_epoch.
    std::optional<msg::Bytes> map_update(const msg::Time& stamp);
    // The answer to one RequestSync (protocol section 5) and its SyncEnd; nothing for another
    // room_id. `since` counts only when expected_generation is this generation; otherwise (a
    // legacy request included, as generation 0 with an all-zero token) the request is answered
    // as SINCE_NONE.
    SyncAnswer sync(const msg::RequestSync& request, const msg::Time& stamp);
    // Writes every sensor's reference and the room's epoch file with the current map_epoch, and
    // with the rig guard its registration clouds and rig_state (save_rig: one ordered commit, the
    // references inside it; the epoch file is already written, by record_epoch). No-op without
    // state_dir.
    void save(const msg::Time& stamp);
    // Feeds one CDR sensor_msgs/Imu received at `received` to the rig guard. False when the guard is
    // off or the frame does not decode (counted in imu_failed, never thrown).
    bool add_imu(std::span<const std::uint8_t> data, const msg::Time& received);
    // The RigPose due at `stamp` (call once per window, after close_window): on every change of the
    // outcome or decision and every 10 s. Empty with the guard off or when not due.
    std::optional<msg::RigPose> rig_pose(const msg::Time& stamp);
    // MOVED while a channel is moved, CHECKING during a check, else the last decision.
    msg::RigOutcome rig_outcome() const;
    std::int64_t imu_failed() const { return imu_failed_; }
    // Measurement only, in no output: the guard's ICP estimates in the last close_window and their
    // wall time (source cloud, seed with its yaw sweep, alignment). close_window runs on the node's
    // single-threaded executor, so this time holds every other callback.
    std::int64_t window_estimates() const { return window_estimates_; }
    double window_estimate_ms() const { return window_estimate_ms_; }
    // The rig files on disk lag the guard: a write or the relearn's archive failed (disk full,
    // permissions). The guard runs on from memory and close_window retries every window; until
    // the archive goes through, the old frame's files stay as they are. rig_save_failed counts the
    // failed attempts, a held epoch write (record_epoch) included, and rig_save_error is the last
    // one's message. epoch_held: the last epoch write failed, so the build's end or a settle waits
    // for it (no save is pending for that).
    bool rig_save_pending() const { return rig_save_pending_; }
    std::int64_t rig_save_failed() const { return rig_save_failed_; }
    const std::string& rig_save_error() const { return rig_save_error_; }
    bool epoch_held() const { return epoch_held_; }

    const MemoryConfig& config() const { return config_; }
    msg::MemoryState state() const { return state_; }
    std::uint64_t map_epoch() const { return map_epoch_; }
    std::uint64_t publisher_generation() const { return publisher_generation_; }
    std::uint64_t event_seq() const { return event_seq_; }
    std::uint64_t sync_end_seq() const { return sync_end_seq_; }
    std::int64_t dropped_windows() const { return dropped_windows_; }
    const Channel& channel(const std::string& sensor) const;
    const Channel& map_channel() const { return channels_[map_channel_]; }

private:
    Channel& find(const std::string& sensor);
    // A built reference: its quota of windows and some settled occupancy, which tracking needs
    // (pose_offset registers on it, and without it no window may teach the reference).
    bool reference_ready(const Channel& channel) const;
    void build(Channel& channel, std::size_t frames, const std::vector<float>& points);
    bool built(std::int64_t now) const;
    // A sensor silent through the build has not built its reference yet (guard only).
    bool building() const;
    std::vector<std::filesystem::path> paths() const;
    void load(const msg::Time& start);
    std::uint64_t next_id();
    msg::ChangeEvent event(const Channel& channel, msg::EventState state, const Candidate& candidate,
                           const msg::Time& stamp);
    void set_reference(Channel& channel, VoxelLayer reference);
    msg::MapBricks map_message(msg::MapKind kind, const msg::Time& stamp) const;
    msg::Bytes publish_map(const msg::MapBricks& message);
    void log_event(const msg::ChangeEvent& event);
    // Rig guard.
    struct Check {
        std::int64_t start = 0;
        std::int64_t causes = 0;
        std::int64_t decisions = 0;
    };
    void load_rig(const std::filesystem::path& room, const std::filesystem::path& path);
    // With the rig guard, writes `epoch` to the room's epoch file before anything carries it, so the
    // file never falls behind an epoch in use, whatever else of a save fails. False when the write
    // fails (disk full, permissions; counted like a failed save): the caller holds what would take
    // the epoch (the build's end, a settle) and the next window tries again. Without the guard or
    // state_dir it writes nothing and returns true: save writes the epoch file first and a failure
    // throws, as before the guard.
    bool record_epoch(std::uint64_t epoch);
    void save_rig();
    void set_pose(Channel& channel, const rig::Pose& pose);
    bool imu_ok(std::int64_t now) const;
    bool any_moved() const;
    bool move_unresolved() const;
    void start_check(std::int64_t now, std::int64_t causes);
    void end_check(std::int64_t now, msg::RigOutcome outcome);
    void guard_triggers(std::int64_t now);
    void guard_building(std::int64_t now);
    void guard_built(std::int64_t now);
    void finish_cloud(Channel& channel);
    void joined(Channel& channel, const msg::Time& stamp);
    void restart_build(Channel& channel);
    void guard_step(std::int64_t now, const msg::Time& stamp, std::vector<msg::ChangeEvent>& events);
    std::optional<rig::Mat3> rotation_prior(const Channel& channel) const;
    msg::RigChannel estimate(Channel& channel, const std::optional<rig::Pose>& seed_from_last = std::nullopt);
    msg::RigChannel align_channel(Channel& channel, const std::optional<rig::Pose>& seed_from_last);
    void decide(std::int64_t now, const msg::Time& stamp, std::vector<msg::ChangeEvent>& events);
    void enter_moved(std::int64_t now, const msg::Time& stamp, std::vector<msg::ChangeEvent>& events);
    void channel_moved(Channel& channel, const msg::Time& stamp, std::vector<msg::ChangeEvent>& events);
    void track_moved(std::int64_t now, const msg::Time& stamp, std::vector<msg::ChangeEvent>& events);
    void relearn(const msg::Time& stamp, std::vector<msg::ChangeEvent>& events);

    MemoryConfig config_;
    std::uint64_t publisher_generation_;
    Config change_;
    const RayBackend* backend_;
    DiffScope scope_;
    std::vector<Channel> channels_;
    msg::MemoryState state_ = msg::MemoryState::building;
    // Closed windows that entered TRACKING, and whether a later settle pass still has counts.
    std::int64_t tracking_windows_ = 0;
    bool settle_pending_ = false;
    std::uint64_t map_epoch_ = 0;
    std::uint64_t event_seq_ = 0;
    std::uint64_t status_seq_ = 0;
    std::uint64_t activity_seq_ = 0;
    std::uint64_t sync_end_seq_ = 0;
    // The newest window with activity since the last activity message, and that window's stamp
    // (the newest window with activity ever).
    std::optional<msg::Activity> activity_;
    std::optional<std::int64_t> last_activity_ns_;
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
    // Rig guard, room level.
    std::optional<ImuDetector> detector_;
    std::int64_t imu_failed_ = 0;
    std::int64_t window_estimates_ = 0;
    double window_estimate_ms_ = 0.0;
    bool rig_save_pending_ = false;
    std::int64_t rig_save_failed_ = 0;
    std::string rig_save_error_;
    bool epoch_held_ = false;
    std::optional<msg::Header> references_due_;  // a save's references, not yet written (save_rig)
    std::optional<std::string> archive_due_;     // the relearned frame's prev/ name, not yet archived
    std::optional<rig::Vec3d> gravity_;  // the newest window's gravity
    std::optional<rig::Vec3d> g_base_;   // gravity at the last accepted decision
    std::optional<rig::Mat3> gyro_;      // the gyro rotation of the last shock, until the check or move uses it
    bool watched_ = false;               // a quiet IMU streamed since the last accepted decision
    std::optional<Check> check_;
    msg::RigOutcome outcome_ = msg::RigOutcome::in_place;
    std::int64_t causes_ = 0;
    std::optional<std::int64_t> next_check_;
    std::optional<std::int64_t> moved_since_;
    msg::Time moved_at_{0, 0};
    std::int64_t n_realigned_ = 0;
    std::uint64_t frame_epoch_ = 0;
    std::int64_t tilt_run_ = 0;
    std::int64_t gross_run_ = 0;
    std::map<std::string, msg::RigChannel> decision_;  // sensor -> result in the last decision window
    std::uint64_t rig_seq_ = 0;
    std::optional<std::pair<msg::RigOutcome, std::int64_t>> rig_sent_;  // (outcome, ns) of the last RigPose
    bool rig_dirty_ = true;
};

template <typename NextId>
std::vector<std::pair<msg::EventState, Candidate>> CandidateTracker::update(const std::vector<Region>& regions,
                                                                            const msg::Time& stamp, NextId&& next_id,
                                                                            const std::vector<bool>& fresh, bool hold) {
    std::vector<std::pair<msg::EventState, Candidate>> transitions;
    std::vector<bool> open_matched(open_.size(), false);
    std::vector<bool> pending_matched(pending_.size(), false);
    std::vector<bool> taken_over(pending_.size(), false);
    std::vector<bool> absorbed(open_.size(), false);
    // Index of the first unmatched entry of `list` of this kind sharing a voxel.
    const auto first_index = [](const std::vector<Candidate>& list, const std::vector<bool>& matched, msg::EventKind kind,
                                const Keys& keys) -> std::optional<std::size_t> {
        for (std::size_t n = 0; n < list.size(); ++n) {
            if (!matched[n] && list[n].kind == kind && intersects(list[n].keys, keys)) return n;
        }
        return std::nullopt;
    };
    // The same, marked matched.
    const auto first = [&](const std::vector<Candidate>& list, std::vector<bool>& matched, msg::EventKind kind,
                           const Keys& keys) -> std::optional<std::size_t> {
        const auto n = first_index(list, matched, kind, keys);
        if (n) matched[*n] = true;
        return n;
    };
    // Index of the first open moved candidate whose pose on this side (the new one for an
    // appeared region, the old one for a disappeared) shares a voxel with `keys`.
    const auto half_of = [&](msg::EventKind kind, const Keys& keys) -> std::optional<std::size_t> {
        for (std::size_t n = 0; n < open_.size(); ++n) {
            if (open_[n].kind != msg::EventKind::moved) continue;
            const Region& pose = open_[n].region;
            if (intersects(kind == msg::EventKind::appeared ? pose.keys : pose.from_keys, keys)) return n;
        }
        return std::nullopt;
    };
    const auto kind_of = [](Region::Kind kind) -> std::optional<msg::EventKind> {
        switch (kind) {
            case Region::Kind::appeared: return msg::EventKind::appeared;
            case Region::Kind::disappeared: return msg::EventKind::disappeared;
            case Region::Kind::moved: return msg::EventKind::moved;
            default: return std::nullopt;
        }
    };
    const auto is_fresh = [&](std::size_t r) { return fresh.empty() || fresh[r]; };

    // Open candidates and the halves of open moves.
    struct Loose {
        std::size_t index;
        msg::EventKind kind;
        Keys keys;
    };
    std::vector<Loose> unmatched;
    std::vector<bool> new_half(open_.size(), false);
    std::vector<bool> old_half(open_.size(), false);
    for (std::size_t r = 0; r < regions.size(); ++r) {
        const Region& region = regions[r];
        const auto kind = kind_of(region.kind);
        if (!kind) continue;
        Keys keys = region_keys(region);
        if (const auto n = first(open_, open_matched, *kind, keys)) {
            Candidate& match = open_[*n];
            match.keys = std::move(keys);
            match.region = region;
            match.last_seen = stamp;
            match.missed = 0;
            continue;
        }
        if (*kind != msg::EventKind::moved) {
            if (const auto h = half_of(*kind, keys)) {
                (*kind == msg::EventKind::appeared ? new_half : old_half)[*h] = true;
                continue;
            }
        }
        unmatched.push_back(Loose{r, *kind, std::move(keys)});
    }
    // A move reported as its two halves (the diff's pairing gates flicker) is still that move.
    // One half alone is not: it keeps nothing alive, so a move whose other half has gone
    // (removed from its new pose, or its old pose filled again) expires and frees the half.
    for (std::size_t n = 0; n < open_.size(); ++n) {
        if (new_half[n] && old_half[n]) {
            absorbed[n] = true;
            open_[n].last_seen = stamp;
            open_[n].missed = 0;
        }
    }

    // Pending. A step is one entry and the region(s) that matched it this window.
    struct Step {
        Candidate* entry;
        std::size_t a;                 // its region, or the appeared half
        std::optional<std::size_t> b;  // the disappeared half
        const Region* region;          // empty for halves: the entry keeps its last moved region
        Keys pose;
        Keys from_pose;
        bool started;
        std::vector<std::size_t> fragments{};  // regions that joined it as fragments of its edge
        bool promoted = false;
    };
    std::vector<Step> steps;
    std::vector<bool> used(regions.size(), false);
    std::vector<Candidate> started;
    started.reserve(regions.size());  // steps point into it
    for (const Loose& loose : unmatched) {
        if (loose.kind != msg::EventKind::moved) continue;
        if (const auto n = first(pending_, pending_matched, loose.kind, loose.keys)) {
            used[loose.index] = true;
            const Region& region = regions[loose.index];
            steps.push_back(Step{&pending_[*n], loose.index, std::nullopt, &region, region.keys, region.from_keys, false});
        }
    }
    for (std::size_t n = 0; n < pending_.size(); ++n) {
        if (pending_matched[n] || pending_[n].kind != msg::EventKind::moved) continue;
        const auto side = [&](msg::EventKind kind, const Keys& anchor) -> std::optional<std::size_t> {
            for (const Loose& loose : unmatched) {
                if (!used[loose.index] && loose.kind == kind && intersects(regions[loose.index].keys, anchor)) {
                    return loose.index;
                }
            }
            return std::nullopt;
        };
        const auto appeared = side(msg::EventKind::appeared, pending_[n].anchor);
        const auto disappeared = side(msg::EventKind::disappeared, pending_[n].anchor_from);
        if (!appeared || !disappeared) continue;
        pending_matched[n] = true;
        used[*appeared] = used[*disappeared] = true;
        steps.push_back(Step{&pending_[n], *appeared, *disappeared, nullptr, regions[*appeared].keys,
                             regions[*disappeared].keys, false});
    }
    for (const Loose& loose : unmatched) {
        if (used[loose.index] || loose.kind != msg::EventKind::moved) continue;
        const Region& region = regions[loose.index];
        const auto new_pose = first_index(pending_, pending_matched, msg::EventKind::appeared, region.keys);
        const auto old_pose = first_index(pending_, pending_matched, msg::EventKind::disappeared, region.from_keys);
        if (!new_pose || !old_pose) continue;
        pending_matched[*new_pose] = pending_matched[*old_pose] = true;
        taken_over[*new_pose] = taken_over[*old_pose] = true;
        used[loose.index] = true;
        const Candidate& a = pending_[*new_pose];
        const Candidate& d = pending_[*old_pose];
        const bool a_first = a.first_seen.sec < d.first_seen.sec ||
                             (a.first_seen.sec == d.first_seen.sec && a.first_seen.nanosec <= d.first_seen.nanosec);
        started.push_back(Candidate{0, loose.kind, loose.keys, region, a_first ? a.first_seen : d.first_seen, stamp,
                                    std::min(a.windows, d.windows), 0, a.anchor, d.anchor});
        steps.push_back(Step{&started.back(), loose.index, std::nullopt, &region, region.keys, region.from_keys, false});
    }
    for (const Loose& loose : unmatched) {
        if (used[loose.index] || loose.kind == msg::EventKind::moved) continue;
        if (const auto n = first(pending_, pending_matched, loose.kind, loose.keys)) {
            used[loose.index] = true;
            const Region& region = regions[loose.index];
            steps.push_back(Step{&pending_[*n], loose.index, std::nullopt, &region, region.keys, region.from_keys, false});
        }
    }
    // A fresh fragment next to the same side of a matched entry is part of it.
    if (tolerance_) {
        const std::size_t matched_steps = steps.size();
        for (const Loose& loose : unmatched) {
            if (used[loose.index] || !is_fresh(loose.index) || loose.kind == msg::EventKind::moved) continue;
            const Box box = box_of(regions[loose.index].keys);
            for (std::size_t n = 0; n < matched_steps; ++n) {
                Step& step = steps[n];
                const msg::EventKind kind = step.entry->kind;
                if (kind != loose.kind && kind != msg::EventKind::moved) continue;
                Keys& side = kind == msg::EventKind::moved && loose.kind == msg::EventKind::disappeared ? step.from_pose : step.pose;
                if (side.empty() || box_gap(box, box_of(side)) > *tolerance_) continue;
                side = set_union(side, regions[loose.index].keys);
                step.fragments.push_back(loose.index);
                used[loose.index] = true;
                break;
            }
        }
    }
    for (const Loose& loose : unmatched) {
        if (used[loose.index] || !is_fresh(loose.index)) continue;
        const Region& region = regions[loose.index];
        started.push_back(Candidate{0, loose.kind, loose.keys, region, stamp, stamp, 0, 0, {}, {}});
        steps.push_back(Step{&started.back(), loose.index, std::nullopt, &region, region.keys, region.from_keys, true});
    }

    // Runs: stale, same shape, or a new run (activity).
    std::vector<bool> active(regions.size(), false);
    bool any_active = false;
    for (Step& step : steps) {
        Candidate& entry = *step.entry;
        entry.keys = step.from_pose.empty() ? step.pose : set_union(step.pose, step.from_pose);
        if ((!step.region || !step.fragments.empty()) && voxel_size_) {
            const Region::Kind kind = entry.kind == msg::EventKind::appeared      ? Region::Kind::appeared
                                      : entry.kind == msg::EventKind::disappeared ? Region::Kind::disappeared
                                                                                   : Region::Kind::moved;
            entry.region = region_from_poses(kind, step.pose, step.from_pose, *voxel_size_);
        } else if (step.region) {
            entry.region = *step.region;
        }
        entry.last_seen = stamp;
        if (!is_fresh(step.a) || (step.b && !is_fresh(*step.b))) {
            entry.windows = 0;
        } else if (!step.started && (!tolerance_ || (same_shape(step.pose, entry.anchor, *tolerance_, stable_fraction_) &&
                                                      same_shape(step.from_pose, entry.anchor_from, *tolerance_,
                                                                 stable_fraction_)))) {
            if (entry.windows == 0) {  // the run restarts here: anchor it to this window, not the last run
                entry.anchor = step.pose;
                entry.anchor_from = step.from_pose;
            }
            ++entry.windows;
        } else {
            entry.windows = 1;
            entry.anchor = step.pose;
            entry.anchor_from = step.from_pose;
            active[step.a] = true;
            if (step.b) active[*step.b] = true;
            for (const std::size_t r : step.fragments) active[r] = true;
            any_active = true;
        }
    }
    // Motion nearby restarts the run.
    activity_.clear();
    for (std::size_t r = 0; r < regions.size(); ++r) {
        if (active[r]) activity_.push_back(r);
    }
    if (radius_ && any_active) {
        const auto boxes = [](const Keys& pose, const Keys& from_pose) {
            std::vector<Box> out;
            if (!pose.empty()) out.push_back(box_of(pose));
            if (!from_pose.empty()) out.push_back(box_of(from_pose));
            return out;
        };
        std::vector<std::vector<Box>> moving(activity_.size());
        for (std::size_t n = 0; n < activity_.size(); ++n) {
            moving[n] = boxes(regions[activity_[n]].keys, regions[activity_[n]].from_keys);
        }
        for (Step& step : steps) {
            Candidate& entry = *step.entry;
            if (entry.windows == 0) continue;
            const std::vector<Box> mine = boxes(step.pose, step.from_pose);
            bool near = false;
            for (std::size_t n = 0; n < activity_.size() && !near; ++n) {
                if (activity_[n] == step.a || (step.b && activity_[n] == *step.b) ||
                    std::find(step.fragments.begin(), step.fragments.end(), activity_[n]) != step.fragments.end()) {
                    continue;
                }
                for (const Box& theirs : moving[n]) {
                    for (const Box& box : mine) near = near || box_gap(box, theirs) <= *radius_;
                }
            }
            if (near) {
                entry.windows = 0;
                entry.anchor = step.pose;
                entry.anchor_from = step.from_pose;
            }
        }
    }

    // Promotion, in region order.
    std::vector<std::size_t> order(steps.size());
    for (std::size_t n = 0; n < order.size(); ++n) order[n] = n;
    const auto lead = [&](const Step& step) { return step.b ? std::min(step.a, *step.b) : step.a; };
    std::sort(order.begin(), order.end(), [&](std::size_t x, std::size_t y) { return lead(steps[x]) < lead(steps[y]); });
    std::vector<Candidate> promoted;
    for (const std::size_t n : order) {
        Step& step = steps[n];
        if (hold || step.entry->windows < candidate_windows_) continue;
        step.promoted = true;
        Candidate c = *step.entry;
        c.event_id = next_id();
        transitions.emplace_back(msg::EventState::candidate, c);
        promoted.push_back(std::move(c));
    }
    std::vector<const Candidate*> gone;
    for (const Step& step : steps) {
        if (step.promoted) gone.push_back(step.entry);
    }
    const auto promoted_from = [&](const Candidate* entry) {
        return std::find(gone.begin(), gone.end(), entry) != gone.end();
    };

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
        if (pending_matched[n] && !taken_over[n] && !promoted_from(&pending_[n])) still_pending.push_back(std::move(pending_[n]));
    }
    for (Candidate& c : started) {
        if (!promoted_from(&c)) still_pending.push_back(std::move(c));
    }
    pending_ = std::move(still_pending);
    return transitions;
}

}  // namespace room_memory
