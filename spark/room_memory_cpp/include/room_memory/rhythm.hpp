// Per-lidar occupancy rhythm: fixed grid, minute records, one writer thread.
//
// The accumulator only counts occupied voxel keys close_window already built. Disk and the
// optional /room_memory/rhythm publish run on one writer thread. The window callback copies a
// finished record into a fixed queue and returns. pose_ok is 1 only when the caller passes a
// tracking residual at or below learn_max_offset_m.
#pragma once

#include <cstdint>
#include <filesystem>
#include <functional>
#include <memory>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "room_memory/keys.hpp"

namespace room_memory {

inline constexpr int kRegionSlots = 4096;
inline constexpr int kQueueSlots = 8;
inline constexpr int kRegionBytes = 10;
inline constexpr int kEnvelopeBytes = 40;
inline constexpr int kCrcBytes = 4;
inline constexpr int kMaxRecordBytes = kEnvelopeBytes + kRegionSlots * kRegionBytes + kCrcBytes;  // 41004
inline constexpr int kRecoveryBytes = 32 * 1024 * 1024;
inline constexpr int kGraceSeconds = 2;
inline constexpr std::uint32_t kFlagPartial = 1u << 0;
inline constexpr std::uint32_t kFlagDiscontinuity = 1u << 1;
inline constexpr std::uint32_t kFlagUnavailable = 1u << 2;
inline constexpr std::uint32_t kFlagOverflow = 1u << 3;
inline constexpr std::uint32_t kFlagLost = 1u << 4;

// What the logger needs from MemoryConfig. RoomMemory copies its fields into this.
struct RhythmSettings {
    std::string room_id = "lab";
    std::vector<std::string> sensors{"mid360", "jt128"};
    double voxel_size = 0.05;
    double window_s = 1.0;
    std::optional<std::string> state_dir = std::string("~/room-memory");
    bool enable = false;
    double cell_m = 0.5;
    int dims = 2;
    std::int64_t period_s = 60;
    bool publish = false;
    std::string dir;
    std::int64_t keep_days = 90;
    std::int64_t max_bytes = 536870912;
    std::int64_t max_file_bytes = 33554432;
    std::int64_t fsync_s = 600;
    // Stamped on RhythmMinute when set. Empty keeps the legacy layout (pinned goldens). Never 0.
    std::optional<std::uint64_t> publisher_generation;
};

struct RhythmRegion {
    std::int16_t ix = 0;
    std::int16_t iy = 0;
    std::int16_t iz = 0;
    std::uint16_t hits = 0;
    std::uint16_t mean = 0;
    friend bool operator==(const RhythmRegion&, const RhythmRegion&) = default;
};

struct RhythmIdentity {
    int header_bytes = 0;
    int sensor = 0;
    int dims = 0;
    int voxels_per_cell = 0;
    std::uint32_t voxel_bits = 0;
    std::uint32_t cell_bits = 0;
    std::uint32_t period_s = 0;
    std::string room_id;
};

struct RhythmRecord {
    std::int64_t minute = 0;
    std::uint64_t map_epoch = 0;
    std::uint16_t closed = 0;
    std::uint16_t with_points = 0;
    std::uint16_t unavailable = 0;
    std::uint16_t pad = 0;
    std::uint32_t clipped = 0;
    std::uint32_t regions_dropped = 0;
    std::uint8_t pose_ok = 0;
    std::uint8_t flags = 0;
    std::vector<RhythmRegion> regions;
};

struct RhythmFragment {
    std::string sensor;
    std::int64_t minute = 0;
    std::uint64_t map_epoch = 0;
    std::uint32_t flags = 0;
    std::vector<std::uint8_t> data;
    bool lost() const { return (flags & kFlagLost) != 0; }
};

// Open bin the tests can see without reading the table.
struct RhythmSensorView {
    bool open = false;
    int with_points = 0;
    bool loss = false;
};

// Filesystem calls the writer thread is allowed to block on. Empty functions use POSIX.
struct RhythmStore {
    std::function<std::ptrdiff_t(int, const void*, std::size_t)> write;
    std::function<int(int)> fsync;
    std::function<int(int, std::int64_t)> truncate;
};

// Binary32 cell_m / voxel_size. An integer in 1…64, or std::invalid_argument.
int voxels_per_cell(double cell_m, double voxel_size);
// period_s / window_s as an integer in 1…3600, or std::invalid_argument.
int windows_per_period(std::int64_t period_s, double window_s);
void validate_rhythm(const RhythmSettings& settings);
// {dir or state_dir}/{room_id}/rhythm, with ~ expanded once. Empty when unset.
std::optional<std::filesystem::path> rhythm_directory(const RhythmSettings& settings);
// False when rhythm is off, or when there is nowhere to put a bin.
bool rhythm_active(const RhythmSettings& settings);
// Keep-last depth of /room_memory/rhythm: one stored minute per configured sensor.
// Depth 1 keeps only the newest message, so a late subscriber misses the other lidar.
inline std::size_t history_depth(std::size_t sensor_count) {
    if (sensor_count < 1) throw std::invalid_argument("rhythm history depth needs a sensor count >= 1");
    return sensor_count;
}

std::uint32_t rhythm_crc32(std::span<const std::uint8_t> data);
std::vector<std::uint8_t> encode_header(int sensor, int dims, int voxels, double voxel_size, double cell_m,
                                        std::uint32_t period_s, const std::string& room_id);
std::optional<RhythmIdentity> header_identity(std::span<const std::uint8_t> data);
bool identity_matches(const RhythmIdentity& parsed, const RhythmIdentity& expected);
std::vector<std::uint8_t> encode_record(std::int64_t minute, std::uint64_t map_epoch, int closed, int with_points,
                                        int unavailable, std::uint32_t clipped, std::uint32_t dropped, int pose_ok,
                                        int flags, std::span<const RhythmRegion> regions);
std::vector<std::uint8_t> encode_marker(std::int64_t minute, std::uint64_t map_epoch);
std::optional<RhythmRecord> parse_record(std::span<const std::uint8_t> data);
// End of the last good record, and that record's minute (empty when the file has none).
std::optional<std::pair<std::size_t, std::optional<std::int64_t>>> scan_records(std::span<const std::uint8_t> data);

// splitmix64 of the three uint16 lanes, masked into a power-of-two table.
int rhythm_hash_slot(std::int16_t ix, std::int16_t iy, std::int16_t iz, int slots = kRegionSlots);

// Per-sensor minute bins. No files. The region tables are allocated in the constructor.
class RhythmAccumulator {
public:
    RhythmAccumulator(std::vector<std::string> sensors, int voxels, int dims, int period_s, int windows_per,
                      int slots = kRegionSlots);
    ~RhythmAccumulator();
    RhythmAccumulator(RhythmAccumulator&&) noexcept;
    RhythmAccumulator& operator=(RhythmAccumulator&&) noexcept;
    RhythmAccumulator(const RhythmAccumulator&) = delete;
    RhythmAccumulator& operator=(const RhythmAccumulator&) = delete;

    void add(const std::string& sensor, std::int64_t sec, std::span<const Key> keys, std::uint64_t map_epoch,
             bool tracking = false, std::optional<double> residual = std::nullopt,
             std::optional<double> learn_max_offset_m = std::nullopt);
    void add_empty(const std::string& sensor, std::int64_t sec, std::uint64_t map_epoch);
    void note_unavailable(const std::string& sensor, std::int64_t sec, std::uint64_t map_epoch);
    void discard(const std::string& sensor, std::int64_t sec, std::uint64_t map_epoch);
    void note_epoch(std::uint64_t map_epoch);
    // The next add() throws after the bin counters move and before the insert.
    void fail_next_insert(const std::string& sensor);
    void set_force_disc(const std::string& sensor);
    RhythmSensorView view(const std::string& sensor) const;
    bool table_holds(const std::string& sensor, int slot, std::int16_t ix, std::int16_t iy, std::int16_t iz) const;
    std::vector<RhythmFragment> take();
    std::vector<RhythmFragment> flush();
    int hook_errors() const;
    std::optional<std::pair<std::int64_t, std::uint64_t>> loss_of(const std::string& sensor) const;
    void remember_loss(const std::string& sensor, std::int64_t minute, std::uint64_t epoch);
    void clear_loss(const std::string& sensor);

private:
    struct State;
    std::unique_ptr<State> state_;
};

// Accumulator plus writer. enqueue() is the only callback-side output.
class Rhythm {
public:
    using Publish = std::function<void(std::span<const std::uint8_t>)>;

    Rhythm(RhythmSettings settings, RhythmStore store = {}, Publish publish = {},
           std::function<bool()> publish_would_block = {});
    ~Rhythm();
    Rhythm(const Rhythm&) = delete;
    Rhythm& operator=(const Rhythm&) = delete;

    void add(const std::string& sensor, std::int64_t sec, std::span<const Key> keys, std::uint64_t map_epoch,
             bool tracking = false, std::optional<double> residual = std::nullopt,
             std::optional<double> learn_max_offset_m = std::nullopt);
    void add_empty(const std::string& sensor, std::int64_t sec, std::uint64_t map_epoch);
    void note_unavailable(const std::string& sensor, std::int64_t sec, std::uint64_t map_epoch);
    void discard(const std::string& sensor, std::int64_t sec, std::uint64_t map_epoch);
    void note_epoch(std::uint64_t map_epoch);
    void fail_next_insert(const std::string& sensor);
    // Copy finished records into the writer queue. No filesystem I/O. Swallows errors.
    void enqueue();
    // Flush open bins, drain the writer, fsync once, join at most 5 s. No prune.
    void close();
    void set_publish(Publish publish, std::function<bool()> publish_would_block = {});
    RhythmSensorView view(const std::string& sensor) const;
    bool table_holds(const std::string& sensor, int slot, std::int16_t ix, std::int16_t iy, std::int16_t iz) const;
    int budget_dropped() const;

private:
    struct Impl;
    void push_fragment(const RhythmFragment& fragment);
    void drain_loss_markers();
    std::unique_ptr<Impl> impl_;
};

}  // namespace room_memory
