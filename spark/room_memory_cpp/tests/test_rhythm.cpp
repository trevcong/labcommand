// Rhythm v1 pins shared with spark/room_memory/tests/test_rhythm.py: the header + 60-window
// hex, the minute/overflow/hash rules, and a one-sensor failure that leaves the other sensor's
// detection bytes unchanged.
#include "harness.hpp"
#include "scenes.hpp"

#include <unistd.h>

#include <array>
#include <cstdint>
#include <filesystem>
#include <optional>
#include <span>
#include <string>
#include <utility>
#include <vector>

#include "room_memory/memory.hpp"
#include "room_memory/rhythm.hpp"

namespace {

using room_memory::Key;
using room_memory::Keys;
using room_memory::MemoryConfig;
using room_memory::Rhythm;
using room_memory::RhythmAccumulator;
using room_memory::RhythmRecord;
using room_memory::RhythmRegion;
using room_memory::RhythmSettings;
using room_memory::RoomMemory;
using room_memory::pack;
using room_memory::splitmix64;
namespace msg = room_memory::msg;

const char* FILE_HEX =
    "524859310100240001020a00cdcc4c3d0000003f3c00000003006c6162000000f427ca96"
    "3c0000003c0000000000000004000000000000003c003c000000000000000000000000000000"
    "02000000000000000f0003000100000000002d000100dcdc9597";
const char* CDR_LE =
    "00010000040000006c616200010000000000000004000000000000000100000000000000"
    "3c000000000000003c003c00000000000000000000000000000002000200000000000100"
    "02000000000000000200000000000000020000000f002d000200000003000100";
const char* CDR_BE =
    "00000000000000046c616200010000000000000000000000000000040000000000000001"
    "0000003c00000000003c003c000000000000000000000000000000020000000200000001"
    "0000000200000000000000020000000000000002000f002d0000000200030001";

std::string hex_of(std::span<const std::uint8_t> bytes) {
    static const char digits[] = "0123456789abcdef";
    std::string out;
    out.reserve(bytes.size() * 2);
    for (const std::uint8_t byte : bytes) {
        out.push_back(digits[byte >> 4]);
        out.push_back(digits[byte & 15]);
    }
    return out;
}

std::vector<std::uint8_t> unhex(const char* text) {
    std::vector<std::uint8_t> out;
    for (std::size_t n = 0; text[n] && text[n + 1]; n += 2) {
        out.push_back(static_cast<std::uint8_t>(std::stoul(std::string(text + n, 2), nullptr, 16)));
    }
    return out;
}

struct TempDir {
    std::filesystem::path path;
    TempDir() {
        std::string pattern = (std::filesystem::temp_directory_path() / "room-memory-rhythm-XXXXXX").string();
        if (!mkdtemp(pattern.data())) throw harness::Failure("mkdtemp failed");
        path = pattern;
    }
    ~TempDir() { std::filesystem::remove_all(path); }
};

RhythmRecord record_of(const std::vector<room_memory::RhythmFragment>& fragments) {
    CHECK_EQ(fragments.size(), std::size_t{1});
    const auto record = room_memory::parse_record(fragments[0].data);
    CHECK(record);
    return *record;
}

const RhythmRegion* region_ix(const RhythmRecord& record, int ix) {
    for (const RhythmRegion& region : record.regions) {
        if (region.ix == ix) return &region;
    }
    return nullptr;
}

void add_window(RhythmAccumulator& accumulator, int sec, const Keys& keys) {
    accumulator.add("mid360", sec, keys, 4);
}

Keys line(std::initializer_list<std::array<std::int64_t, 3>> voxels) {
    Keys keys;
    for (const auto& voxel : voxels) keys.push_back(pack(voxel[0], voxel[1], voxel[2]));
    return keys;
}

MemoryConfig loose(std::optional<std::string> rhythm_dir, bool enable) {
    MemoryConfig config;
    config.sensors = {"mid360", "jt128"};
    config.max_range = std::nullopt;
    config.min_voxels = 1;
    config.min_extent = 1;
    config.reference_windows = 1;
    config.recent_windows = 1;
    config.occupied_min_hits = 1;
    config.occupied_min_ratio = 0;
    config.free_min_seen = 1;
    config.candidate_windows = 1;
    config.expire_windows = 1;
    config.fresh_min_voxels = 1;
    config.fresh_min_fraction = 0;
    config.activity_radius_m = 0;
    config.learn_windows = 0;
    config.state_dir = std::nullopt;
    config.rhythm_enable = enable;
    config.rhythm_publish = false;
    config.rhythm_dir = rhythm_dir.value_or("");
    config.rhythm_keep_days = 0;
    config.rhythm_max_bytes = 0;
    config.rhythm_max_file_bytes = 0;
    config.rhythm_fsync_s = 0;
    return config;
}

struct Snapshot {
    std::string events;
    std::string maps;
    std::string status;
};

Snapshot drive(RoomMemory& memory, bool fail_mid360) {
    const scenes::Cloud room = scenes::room_shell();
    const scenes::Cloud box = scenes::box_shell({0.3, 1.2, 0.3});
    const scenes::Cloud both = scenes::concat({&room, &box});
    Snapshot out;
    const auto take = [&](std::int64_t sec, const scenes::Cloud& cloud, bool fail) {
        memory.add_points("mid360", cloud);
        memory.add_points("jt128", cloud);
        if (fail) memory.fail_next_rhythm_insert("mid360");
        for (const msg::ChangeEvent& event : memory.close_window(msg::Time{sec, 0})) out.events += hex_of(event.to_cdr()) + "\n";
        const std::optional<msg::Bytes> map = memory.map_update(msg::Time{sec, 0});
        out.maps += map ? hex_of(*map) : "none";
        out.maps += "\n";
        memory.enqueue_rhythm();
    };
    take(60, room, false);
    take(61, both, fail_mid360);
    out.status = hex_of(memory.status(msg::Time{62, 0}, 1.0).to_cdr());
    return out;
}

}  // namespace

TEST(crc_of_rhy1_is_the_shared_constant) {
    const std::uint8_t magic[] = {'R', 'H', 'Y', '1'};
    CHECK_EQ(room_memory::rhythm_crc32(magic), std::uint32_t{0x046E226A});
    CHECK_EQ(room_memory::voxels_per_cell(0.5, 0.05), 10);
    CHECK_EQ(room_memory::windows_per_period(60, 1.0), 60);
}

TEST(floor_pins_and_a_repeated_coarse_cell_are_one_row) {
    RhythmAccumulator floor_cells({"mid360"}, 10, 2, 60, 60);
    floor_cells.add("mid360", 60, line({{-11, 0, 0}, {-10, 0, 0}, {-1, 0, 0}, {9, 0, 0}, {10, 0, 0}}), 1);
    const RhythmRecord record = record_of(floor_cells.flush());
    CHECK_EQ(record.regions.size(), std::size_t{4});
    CHECK(record.regions[0] == (RhythmRegion{-2, 0, 0, 1, 1}));
    CHECK(record.regions[1] == (RhythmRegion{-1, 0, 0, 1, 2}));
    CHECK(record.regions[2] == (RhythmRegion{0, 0, 0, 1, 1}));
    CHECK(record.regions[3] == (RhythmRegion{1, 0, 0, 1, 1}));

    // Voxels (0,0,0), (0,10,0), (1,0,0) are cells A, B, A. One row each; A sums both voxels.
    RhythmAccumulator coarse({"mid360"}, 10, 2, 60, 60);
    coarse.add("mid360", 60, line({{0, 0, 0}, {0, 10, 0}, {1, 0, 0}}), 1);
    const RhythmRecord once = record_of(coarse.flush());
    CHECK_EQ(once.regions.size(), std::size_t{2});
    CHECK(once.regions[0] == (RhythmRegion{0, 0, 0, 1, 2}));
    CHECK(once.regions[1] == (RhythmRegion{0, 1, 0, 1, 1}));
}

TEST(empty_is_not_unavailable) {
    RhythmAccumulator empty({"mid360"}, 10, 2, 60, 60);
    empty.add_empty("mid360", 60, 1);
    empty.add_empty("mid360", 61, 1);
    const RhythmRecord blank = record_of(empty.flush());
    CHECK_EQ(static_cast<int>(blank.closed), 2);
    CHECK_EQ(static_cast<int>(blank.with_points), 0);
    CHECK_EQ(static_cast<int>(blank.unavailable), 0);
    CHECK(blank.regions.empty());
    CHECK_EQ(static_cast<int>(blank.pose_ok), 0);
    CHECK((blank.flags & room_memory::kFlagPartial) != 0);
    CHECK((blank.flags & room_memory::kFlagUnavailable) == 0);

    RhythmAccumulator skipped({"mid360"}, 10, 2, 60, 60);
    skipped.note_unavailable("mid360", 60, 1);
    const RhythmRecord missing = record_of(skipped.flush());
    CHECK_EQ(static_cast<int>(missing.closed), 0);
    CHECK_EQ(static_cast<int>(missing.with_points), 0);
    CHECK_EQ(static_cast<int>(missing.unavailable), 1);
    CHECK(missing.regions.empty());
    CHECK((missing.flags & room_memory::kFlagUnavailable) != 0);
    CHECK_EQ(static_cast<int>(missing.pose_ok), 0);
}

TEST(pose_ok_requires_every_point_window) {
    const auto pose = [](const std::vector<std::pair<int, std::optional<double>>>& samples) {
        RhythmAccumulator accumulator({"mid360"}, 10, 2, 60, 60);
        for (const auto& [sec, residual] : samples) {
            if (!residual) accumulator.add_empty("mid360", sec, 1);
            else accumulator.add("mid360", sec, Keys{pack(0, 0, 0)}, 1, true, residual, 0.2);
        }
        return static_cast<int>(record_of(accumulator.flush()).pose_ok);
    };
    std::vector<std::pair<int, std::optional<double>>> all;
    for (int sec = 0; sec < 60; ++sec) all.push_back({sec, 0.1});
    CHECK_EQ(pose(all), 1);
    CHECK_EQ(pose({{60, 0.1}, {61, 0.5}}), 0);
    CHECK_EQ(pose({{60, 0.0}, {61, std::nullopt}, {62, 0.0}}), 1);
    CHECK_EQ(pose({{60, std::nullopt}}), 0);
}

TEST(grace_keeps_one_fragment_and_a_gap_writes_no_filler) {
    RhythmAccumulator same({"mid360"}, 10, 2, 60, 60);
    for (const int sec : {60, 61, 59}) same.add("mid360", sec, Keys{pack(0, 0, 0)}, 1);
    const RhythmRecord stayed = record_of(same.flush());
    CHECK_EQ(stayed.minute, std::int64_t{60});
    CHECK_EQ(static_cast<int>(stayed.closed), 3);
    CHECK((stayed.flags & room_memory::kFlagDiscontinuity) == 0);

    RhythmAccumulator edge({"mid360"}, 10, 2, 60, 60);
    edge.add("mid360", 60, Keys{pack(0, 0, 0)}, 1);
    edge.add("mid360", 58, Keys{pack(0, 0, 0)}, 1);
    CHECK(edge.take().empty());
    edge.add("mid360", 57, Keys{pack(0, 0, 0)}, 1);
    const auto sealed = edge.take();
    CHECK_EQ(sealed.size(), std::size_t{1});
    CHECK_EQ(record_of(sealed).minute, std::int64_t{60});
    const auto earlier = edge.flush();
    CHECK_EQ(earlier.size(), std::size_t{1});
    const RhythmRecord opened = record_of(earlier);
    CHECK_EQ(opened.minute, std::int64_t{0});
    CHECK((opened.flags & room_memory::kFlagDiscontinuity) != 0);

    RhythmAccumulator gap({"mid360"}, 10, 2, 60, 60);
    gap.add("mid360", 60, Keys{pack(0, 0, 0)}, 1);
    gap.add("mid360", 240, Keys{pack(0, 0, 0)}, 1);
    const auto jumped = gap.flush();
    CHECK_EQ(jumped.size(), std::size_t{2});
    CHECK_EQ(record_of({jumped[0]}).minute, std::int64_t{60});
    CHECK_EQ(record_of({jumped[1]}).minute, std::int64_t{240});
    CHECK((record_of({jumped[1]}).flags & room_memory::kFlagDiscontinuity) != 0);

    RhythmAccumulator back({"mid360"}, 10, 2, 60, 60);
    back.add("mid360", 120, Keys{pack(0, 0, 0)}, 1);
    back.add("mid360", 0, Keys{pack(0, 0, 0)}, 1);
    const auto stepped = back.flush();
    CHECK_EQ(record_of({stepped[0]}).minute, std::int64_t{120});
    CHECK_EQ(record_of({stepped[1]}).minute, std::int64_t{0});
    CHECK((record_of({stepped[1]}).flags & room_memory::kFlagDiscontinuity) != 0);

    RhythmAccumulator negative({"mid360"}, 10, 2, 60, 60);
    negative.add("mid360", -1, Keys{pack(0, 0, 0)}, 1);
    CHECK(negative.flush().empty());
}

TEST(the_first_miss_stops_the_window) {
    Keys full;
    full.reserve(4096);
    for (int i = 1; i <= 4096; ++i) full.push_back(pack(i, 0, 0));
    RhythmAccumulator table({"mid360"}, 1, 2, 60, 60);
    table.add("mid360", 10, full, 1);
    table.add("mid360", 10, Keys{pack(0, 0, 0), pack(1, 0, 0)}, 1);
    const RhythmRecord overflow = record_of(table.flush());
    CHECK((overflow.flags & room_memory::kFlagOverflow) != 0);
    CHECK((overflow.flags & room_memory::kFlagLost) == 0);
    CHECK_EQ(overflow.regions_dropped, std::uint32_t{2});
    CHECK(region_ix(overflow, 0) == nullptr);
    CHECK(region_ix(overflow, 1) && region_ix(overflow, 1)->hits == 1);
    CHECK_EQ(overflow.regions.size(), std::size_t{4096});

    Keys wide;
    wide.reserve(4100);
    for (int i = 0; i < 4100; ++i) wide.push_back(pack(i, 0, 0));
    RhythmAccumulator broad({"mid360"}, 1, 2, 60, 60);
    broad.add("mid360", 10, wide, 1);
    const RhythmRecord cut = record_of(broad.flush());
    CHECK_EQ(cut.regions.size(), std::size_t{4096});
    CHECK_EQ(cut.regions_dropped, std::uint32_t{4});
    CHECK((cut.flags & room_memory::kFlagOverflow) != 0);
    CHECK_EQ(static_cast<int>(cut.regions.front().ix), 0);
    CHECK_EQ(static_cast<int>(cut.regions.back().ix), 4095);
}

TEST(hash_slot_uses_the_uint16_lanes) {
    const std::uint64_t packed = (std::uint64_t{std::uint16_t(-1)} << 32) | (std::uint64_t{2} << 16) | 3;
    const int slot = room_memory::rhythm_hash_slot(-1, 2, 3);
    CHECK_EQ(slot, static_cast<int>(splitmix64(packed) & 4095));
    RhythmAccumulator accumulator({"mid360"}, 1, 3, 60, 60);
    accumulator.add("mid360", 60, Keys{pack(-1, 2, 3)}, 1);
    CHECK(accumulator.table_holds("mid360", slot, -1, 2, 3));
}

TEST(shared_header_and_sixty_window_hex) {
    RhythmAccumulator accumulator({"mid360"}, 10, 2, 60, 60);
    for (int n = 0; n < 60; ++n) {
        Keys keys;
        if (n < 15) {
            for (int i = 0; i < 10; ++i) keys.push_back(pack(i, 0, 99));
        } else {
            keys.push_back(pack(10, 0, 5));
        }
        add_window(accumulator, 60 + n, keys);
    }
    const auto fragments = accumulator.flush();
    const RhythmRecord record = record_of(fragments);
    CHECK_EQ(record.minute, std::int64_t{60});
    CHECK_EQ(record.map_epoch, std::uint64_t{4});
    CHECK_EQ(static_cast<int>(record.closed), 60);
    CHECK_EQ(static_cast<int>(record.with_points), 60);
    CHECK_EQ(static_cast<int>(record.unavailable), 0);
    CHECK_EQ(static_cast<int>(record.pose_ok), 0);
    CHECK_EQ(static_cast<int>(record.flags), 0);
    CHECK(record.regions[0] == (RhythmRegion{0, 0, 0, 15, 3}));
    CHECK(record.regions[1] == (RhythmRegion{1, 0, 0, 45, 1}));
    auto bytes = room_memory::encode_header(1, 2, 10, 0.05, 0.5, 60, "lab");
    bytes.insert(bytes.end(), fragments[0].data.begin(), fragments[0].data.end());
    CHECK_EQ(hex_of(bytes), std::string(FILE_HEX));

    TempDir dir;
    RhythmSettings settings;
    settings.room_id = "lab";
    settings.sensors = {"mid360"};
    settings.voxel_size = 0.05;
    settings.window_s = 1.0;
    settings.state_dir = std::nullopt;
    settings.dir = dir.path.string();
    settings.keep_days = 0;
    settings.max_bytes = 0;
    settings.max_file_bytes = 0;
    settings.fsync_s = 0;
    {
        Rhythm rhythm(settings);
        for (int n = 0; n < 60; ++n) {
            Keys keys;
            if (n < 15) {
                for (int i = 0; i < 10; ++i) keys.push_back(pack(i, 0, 99));
            } else {
                keys.push_back(pack(10, 0, 5));
            }
            rhythm.add("mid360", 60 + n, keys, 4);
        }
        rhythm.enqueue();
        rhythm.close();
    }
    const auto written = harness::read_bytes(dir.path / "lab" / "rhythm" / "rhythm-mid360-19700101.rhy");
    CHECK_EQ(hex_of(written), std::string(FILE_HEX));
}

TEST(rhythm_minute_golden_round_trips_both_endians) {
    msg::RhythmMinute message;
    message.room_id = "lab";
    message.sensor = 1;
    message.map_epoch = 4;
    message.seq = 1;
    message.stamp = msg::Time{60, 0};
    message.windows_closed = 60;
    message.windows_with_points = 60;
    message.ix = {0, 1};
    message.iy = {0, 0};
    message.iz = {0, 0};
    message.hit_windows = {15, 45};
    message.mean_voxels = {3, 1};
    const msg::Bytes encoded = message.to_cdr();
    CHECK_EQ(hex_of(encoded), std::string(CDR_LE));
    CHECK(msg::RhythmMinute::from_cdr(encoded) == message);
    CHECK(msg::RhythmMinute::from_cdr(unhex(CDR_BE)) == message);
    CHECK_EQ(static_cast<int>(encoded[12]), 1);
    message.sensor = 2;
    const msg::Bytes twin = message.to_cdr();
    CHECK(twin != encoded);
    CHECK_EQ(static_cast<int>(twin[12]), 2);
    CHECK(msg::RhythmMinute::from_cdr(twin) == message);
    message.sensor = 3;
    CHECK_THROWS(std::invalid_argument, message.to_cdr());
    message.sensor = 1;
    const msg::Bytes other = unhex(CDR_LE);
    msg::Bytes rejected = other;
    rejected[12] = 9;
    CHECK_THROWS(std::invalid_argument, msg::RhythmMinute::from_cdr(rejected));
}

TEST(bad_rhythm_config_is_rejected_before_a_bin_exists) {
    RhythmSettings settings;
    settings.voxel_size = 0.05;
    settings.window_s = 1.0;
    settings.room_id = "lab";
    settings.state_dir = std::nullopt;
    validate_rhythm(settings);
    settings.cell_m = 0.03;
    CHECK_THROWS(std::invalid_argument, validate_rhythm(settings));
    settings.cell_m = 0.5;
    settings.max_bytes = 100000;
    settings.max_file_bytes = 200000;
    CHECK_THROWS(std::invalid_argument, validate_rhythm(settings));
    settings.max_file_bytes = 0;
    validate_rhythm(settings);
    settings.max_bytes = 0;
    settings.period_s = 0;
    CHECK_THROWS(std::invalid_argument, validate_rhythm(settings));
}

TEST(rhythm_on_matches_rhythm_off_and_a_throw_drops_only_that_bin) {
    TempDir dir;
    const msg::Time start{0, 0};
    RoomMemory enabled(loose(dir.path.string(), true), start);
    RoomMemory disabled(loose(std::nullopt, false), start);
    const Snapshot on = drive(enabled, false);
    const Snapshot off = drive(disabled, false);
    CHECK(!on.events.empty());
    CHECK_EQ(on.events, off.events);
    CHECK_EQ(on.maps, off.maps);
    CHECK_EQ(on.status, off.status);
    enabled.close();
    const auto path = dir.path / "lab" / "rhythm" / "rhythm-mid360-19700101.rhy";
    const auto bytes = harness::read_bytes(path);
    const auto identity = room_memory::header_identity(bytes);
    CHECK(identity && identity->sensor == 1 && identity->room_id == "lab");
    const auto scanned = room_memory::scan_records(bytes);
    CHECK(scanned && scanned->second && *scanned->second == 60);
    const auto stored = room_memory::parse_record(std::span<const std::uint8_t>(
        bytes.data() + static_cast<std::size_t>(identity->header_bytes), scanned->first - static_cast<std::size_t>(identity->header_bytes)));
    CHECK(stored && stored->with_points == 2 && stored->pose_ok == 0);

    TempDir other;
    RoomMemory throwing(loose(other.path.string(), true), start);
    RoomMemory quiet(loose(std::nullopt, false), start);
    const Snapshot boom = drive(throwing, true);
    const auto mid = throwing.rhythm_sensor("mid360");
    const auto jt = throwing.rhythm_sensor("jt128");
    CHECK(!mid.open && mid.loss);
    CHECK(jt.open && jt.with_points == 2);
    const Snapshot still = drive(quiet, false);
    CHECK_EQ(boom.events, still.events);
    CHECK_EQ(boom.maps, still.maps);
    CHECK_EQ(boom.status, still.status);
}

TEST_MAIN
