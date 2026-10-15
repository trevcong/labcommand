// Voxel layers, bricks, reference persistence and the window engine, ported from
// spark/room_memory/tests/test_memory.py (including the three Codex regressions of 289afb6),
// and the rejected-frame throttle of test_rejectlog.py.
#include <unistd.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <fstream>
#include <functional>
#include <map>
#include <random>

#include "harness.hpp"
#include "room_memory/memory.hpp"
#include "room_memory/pointcloud2.hpp"
#include "room_memory/rejectlog.hpp"
#include "scenes.hpp"

using namespace room_memory;
using msg::EventKind;
using msg::EventState;
using msg::MemoryState;
using scenes::Cloud;

namespace {

const msg::Time kStart{1760400000, 0};

msg::Time stamp(std::int64_t second) { return msg::Time{kStart.sec + second, 0}; }

msg::Bytes unhex(const std::string& text) {
    msg::Bytes out;
    for (std::size_t n = 0; n + 1 < text.size(); n += 2) {
        out.push_back(static_cast<std::uint8_t>(std::stoul(text.substr(n, 2), nullptr, 16)));
    }
    return out;
}

Keys keys(const std::vector<std::array<std::int64_t, 3>>& cells) {
    Keys out;
    for (const auto& c : cells) out.push_back(pack(c[0], c[1], c[2]));
    sort_unique(out);
    return out;
}

VoxelLayer layer(const std::vector<std::array<std::int64_t, 3>>& occupied,
                 const std::vector<std::array<std::int64_t, 3>>& known) {
    VoxelLayer out;
    out.occupied = keys(occupied);
    out.known = keys(known);
    return out;
}

std::array<std::uint8_t, msg::kMaskBytes> mask(std::uint8_t first, std::uint8_t last = 0) {
    std::array<std::uint8_t, msg::kMaskBytes> m{};
    m[0] = first;
    m[63] = last;
    return m;
}

// A fresh directory under the system temp dir, removed when the test ends.
struct TempDir {
    std::filesystem::path path;
    TempDir() {
        std::string pattern = (std::filesystem::temp_directory_path() / "room-memory-test-XXXXXX").string();
        if (!mkdtemp(pattern.data())) throw harness::Failure("mkdtemp failed");
        path = pattern;
    }
    ~TempDir() { std::filesystem::remove_all(path); }
};

// Single-window mechanics: every voxel a window sees settles, the recent layer is the last
// window, one supported voxel makes a region fresh, no activity holds a region back, and every
// region is sent and expired at once. The node's defaults are measured on the static recordings
// (tests/recording).
MemoryConfig mid360_config(const std::optional<std::string>& state_dir) {
    MemoryConfig config;
    config.sensors = {"mid360"};
    config.max_range = std::nullopt;
    config.reference_windows = 3;
    config.recent_windows = 1;
    config.occupied_min_hits = 1;
    config.occupied_min_ratio = 0.0;
    config.free_min_seen = 1;
    config.candidate_windows = 1;
    config.expire_windows = 1;
    config.fresh_min_voxels = 1;
    config.fresh_min_fraction = 0.0;
    config.activity_radius_m = 0.0;
    config.state_dir = state_dir;
    return config;
}

std::int64_t build_reference(RoomMemory& memory, const Cloud& cloud, int windows = 3) {
    for (int second = 0; second < windows; ++second) {
        memory.add_points("mid360", cloud);
        CHECK(memory.close_window(stamp(second)).empty());
    }
    return windows;
}

std::vector<std::pair<EventKind, EventState>> transitions(const std::vector<msg::ChangeEvent>& events) {
    std::vector<std::pair<EventKind, EventState>> out;
    for (const auto& e : events) out.emplace_back(e.kind, e.state);
    return out;
}

Keys occupied_keys(const Cloud& cloud) { return occupancy_map(cloud, Config()).occupied; }

Cloud partial_shell(const Cloud& room) {
    Cloud partial;
    for (std::size_t n = 0; n < room.size(); n += 3) {
        if (room[n] < 1.5f) {
            partial.insert(partial.end(), room.begin() + static_cast<std::ptrdiff_t>(n),
                           room.begin() + static_cast<std::ptrdiff_t>(n) + 3);
        }
    }
    return partial;
}

// One reference window, then one tracking window of budget. The due tick is 3 s later, so the
// window that spends the budget is not itself a map tick.
MemoryConfig budget_config(std::int64_t learn_windows = 1) {
    MemoryConfig config = mid360_config(std::nullopt);
    config.reference_windows = 1;
    config.learn_windows = learn_windows;
    config.delta_period_s = 3.0;
    config.keyframe_period_s = 4.0;
    config.keyframe_delta_ratio = 100.0;
    return config;
}

std::optional<msg::MapBricks> feed(RoomMemory& memory, const Cloud* cloud, std::int64_t second) {
    if (cloud != nullptr) memory.add_points("mid360", *cloud);
    memory.close_window(stamp(second));
    const auto bytes = memory.map_update(stamp(second));
    if (!bytes) return std::nullopt;
    return msg::MapBricks::from_cdr(*bytes);
}

}  // namespace

TEST(bricks_follow_the_protocol_bit_layout) {
    const auto bricks = layer({{-8, 16, -24}, {-1, 23, -17}}, {{-8, 16, -24}, {-1, 23, -17}, {-7, 16, -24}}).to_bricks();
    CHECK(bricks.size() == 1);
    CHECK((bricks[0].coordinate == std::array<std::int64_t, 3>{-1, 2, -3}));
    CHECK(bricks[0].occupied == mask(0x01, 0x80) && bricks[0].known == mask(0x03, 0x80));
}

TEST(bricks_keep_all_free_and_omit_unknown) {
    std::vector<std::array<std::int64_t, 3>> known = {{0, 0, 0}};
    for (int x = 0; x < 8; ++x) {
        for (int y = 0; y < 8; ++y) {
            for (int z = 8; z < 16; ++z) known.push_back({x, y, z});
        }
    }
    const auto bricks = layer({{0, 0, 0}}, known).to_bricks();
    CHECK(bricks.size() == 2);
    CHECK((bricks[1].coordinate == std::array<std::int64_t, 3>{0, 0, 1}));
    std::array<std::uint8_t, msg::kMaskBytes> full;
    full.fill(0xFF);
    const std::array<std::uint8_t, msg::kMaskBytes> empty{};
    CHECK(bricks[1].occupied == empty && bricks[1].known == full);
}

TEST(bricks_round_trip_random_layers) {
    std::mt19937_64 rng(3);
    std::uniform_int_distribution<std::int64_t> lane(-300, 299);
    std::bernoulli_distribution occupied_draw(0.3);
    VoxelLayer original;
    for (int n = 0; n < 5000; ++n) original.known.push_back(pack(lane(rng), lane(rng), lane(rng)));
    sort_unique(original.known);
    for (const Key key : original.known) {
        if (occupied_draw(rng)) original.occupied.push_back(key);
    }
    const auto bricks = original.to_bricks();
    for (std::size_t n = 1; n < bricks.size(); ++n) CHECK(bricks[n - 1].coordinate < bricks[n].coordinate);
    const VoxelLayer restored = VoxelLayer::from_bricks(bricks);
    CHECK(restored.occupied == original.occupied && restored.known == original.known);
    CHECK_EQ(original.brick_count(), static_cast<std::int64_t>(bricks.size()));
}

TEST(bricks_reject_out_of_range) {
    CHECK_THROWS(std::invalid_argument, layer({{8 * 40000, 0, 0}}, {{8 * 40000, 0, 0}}).to_bricks());
}

TEST(observe_marks_the_ray_and_keeps_the_endpoint_occupied) {
    Config config;
    config.evidence_of_absence = true;
    const VoxelLayer window = VoxelLayer::observe(occupancy_map(scenes::column({40}), config), config);
    CHECK(window.occupied == Keys{pack(40, 0, 0)});
    Keys expected;
    for (int i = 0; i <= 40; ++i) expected.push_back(pack(i, 0, 0));
    CHECK(window.known == expected);
}

TEST(save_and_load_reference) {
    TempDir dir;
    std::mt19937_64 rng(5);
    std::uniform_int_distribution<std::int64_t> lane(-100, 99);
    VoxelLayer original;
    for (int n = 0; n < 2000; ++n) original.known.push_back(pack(lane(rng), lane(rng), lane(rng)));
    sort_unique(original.known);
    for (std::size_t n = 0; n < original.known.size(); n += 3) original.occupied.push_back(original.known[n]);
    const auto path = reference_path(dir.path.string(), "lab", "mid360");
    save_reference(path, original, msg::Header{"lab", 7, 0, kStart}, 0.05f);
    const auto [message, restored] = load_reference(path);
    CHECK(message.header.map_epoch == 7 && message.header.room_id == "lab");
    CHECK(message.bricks == original.to_bricks());
    CHECK(restored.known == original.known && restored.occupied == original.occupied);
    CHECK(!std::filesystem::exists(path.string() + ".tmp"));
    CHECK_THROWS(std::invalid_argument, reference_path(dir.path.string(), "../etc", "mid360"));
}

TEST(reference_build_then_tracking_with_unknown_back_face) {
    TempDir dir;
    const scenes::Board board = scenes::board_scene();
    MemoryConfig config = mid360_config(dir.path.string());
    config.min_extent = 1;  // the board's appeared faces are one cell thick
    RoomMemory memory(config, kStart);
    msg::MemoryStatus status = memory.status(stamp(0), 1.0);
    CHECK(status.state == MemoryState::degraded && status.header.map_epoch == 0 && status.mid360_hz == 0);
    CHECK(!status.jt128_hz && !status.pose_residual_m && !status.coverage_known_frac && !status.events_seq);
    memory.add_points("mid360", board.before);
    CHECK(memory.status(stamp(0), 1.0).state == MemoryState::building);
    const std::int64_t second = build_reference(memory, board.before);
    memory.status(stamp(second), 1.0);
    CHECK(memory.state() == MemoryState::tracking && memory.map_epoch() == 1);
    CHECK(std::filesystem::exists(reference_path(dir.path.string(), "lab", "mid360")));

    memory.add_points("mid360", board.after);
    const auto events = memory.close_window(stamp(second));
    CHECK((transitions(events) == std::vector<std::pair<EventKind, EventState>>{
                                      {EventKind::appeared, EventState::candidate},
                                      {EventKind::appeared, EventState::candidate},
                                      {EventKind::disappeared, EventState::candidate}}));
    const Keys back = occupied_keys(board.back), front = occupied_keys(board.front);
    const msg::ChangeEvent& gone = events[2];
    const VoxelLayer mask_layer = VoxelLayer::from_bricks(gone.mask->bricks);
    CHECK(!intersects(mask_layer.occupied, back));
    CHECK(static_cast<double>(count_present(mask_layer.occupied, front)) / static_cast<double>(front.size()) > 0.95);
    CHECK(gone.evidence == 1.0f && !gone.alpha && gone.sensors == msg::kSensorMid360);
    CHECK(gone.header.map_epoch == 1 && gone.first_seen == stamp(second) && gone.last_seen == stamp(second));
    for (const auto& e : events) CHECK(msg::ChangeEvent::from_cdr(e.to_cdr()) == e);
    CHECK(events[0].event_id != events[1].event_id && events[1].event_id != events[2].event_id);
    for (std::size_t n = 0; n < events.size(); ++n) CHECK_EQ(events[n].header.seq, std::uint64_t{n});

    memory.add_points("mid360", board.after);
    CHECK(memory.close_window(stamp(second + 1)).empty());
    status = memory.status(stamp(second + 1), 2.0);
    CHECK(status.state == MemoryState::tracking && status.n_open_events == 3 && status.mid360_hz == 1);
    CHECK(status.events_seq == events.back().header.seq && status.events_seq == std::uint64_t{2});
    CHECK_EQ(status.n_bricks, static_cast<std::int64_t>(memory.channel("mid360").reference.to_bricks().size()));
    CHECK(msg::MemoryStatus::from_cdr(status.to_cdr()) == status);

    memory.add_points("mid360", board.before);
    const auto expired = memory.close_window(stamp(second + 2));
    CHECK(expired.size() == 3);
    for (const auto& e : expired) CHECK(e.state == EventState::expired && e.last_seen == stamp(second + 1));
    std::vector<std::uint64_t> a, b;
    for (const auto& e : expired) a.push_back(e.event_id);
    for (const auto& e : events) b.push_back(e.event_id);
    std::sort(a.begin(), a.end());
    std::sort(b.begin(), b.end());
    CHECK(a == b);
    status = memory.status(stamp(second + 2), 1.0);
    CHECK_EQ(status.n_open_events, std::int64_t{0});
    CHECK(status.events_seq == expired.back().header.seq && status.events_seq == std::uint64_t{5});
}

TEST(new_coverage_is_not_an_appearance) {
    const Cloud room = scenes::room_shell();
    Cloud missing_wall, wall;
    for (std::size_t n = 0; n < room.size(); n += 3) {
        Cloud& into = room[n] < 2.4f ? missing_wall : wall;
        into.insert(into.end(), room.begin() + static_cast<std::ptrdiff_t>(n), room.begin() + static_cast<std::ptrdiff_t>(n) + 3);
    }
    RoomMemory memory(mid360_config(std::nullopt), kStart);
    const std::int64_t second = build_reference(memory, missing_wall);
    const Keys wall_keys = occupied_keys(wall);
    CHECK(count_present(memory.channel("mid360").reference.known, wall_keys) < wall_keys.size());
    memory.add_points("mid360", room);
    CHECK(memory.close_window(stamp(second)).empty());
    const Cloud box = scenes::box_shell({0.3, 1.2, 0.3});
    memory.add_points("mid360", scenes::concat({&room, &box}));
    const auto events = memory.close_window(stamp(second + 1));
    CHECK((transitions(events) == std::vector<std::pair<EventKind, EventState>>{{EventKind::appeared, EventState::candidate}}));
    const std::array<float, 3> center = {0.3f, 1.2f, 0.3f};
    for (int a = 0; a < 3; ++a) CHECK(std::fabs(events[0].centroid[a] - center[a]) <= 0.025f);
    CHECK(events[0].evidence == 1.0f);
}

TEST(moved_box_is_one_moved_candidate) {
    const Cloud room = scenes::room_shell();
    const Cloud box = scenes::box_shell({0.0, 0.5, 0.30});
    const Cloud moved = scenes::shifted(box, 1);
    for (const DiffScope scope : {DiffScope::regions, DiffScope::full}) {
        RoomMemory memory(mid360_config(std::nullopt), kStart, nullptr, scope);
        const std::int64_t second = build_reference(memory, scenes::concat({&room, &box}));
        memory.add_points("mid360", scenes::concat({&room, &moved}));
        const auto events = memory.close_window(stamp(second));
        CHECK((transitions(events) == std::vector<std::pair<EventKind, EventState>>{{EventKind::moved, EventState::candidate}}));
        CHECK(events[0].from_centroid.has_value() && msg::ChangeEvent::from_cdr(events[0].to_cdr()) == events[0]);
    }
}

TEST(reload_restores_epoch_and_bricks) {
    TempDir dir;
    RoomMemory memory(mid360_config(dir.path.string()), kStart);
    const Cloud room = scenes::room_shell();
    const Cloud box = scenes::box_shell({1.0, 0.5, 0.3});
    build_reference(memory, scenes::concat({&room, &box}));
    const auto bricks = memory.channel("mid360").reference.to_bricks();
    RoomMemory restarted(mid360_config(dir.path.string()), kStart);
    CHECK(restarted.state() == MemoryState::tracking && memory.map_epoch() == 1 && restarted.map_epoch() == 2);
    CHECK(restarted.channel("mid360").reference.to_bricks() == bricks);
    CHECK_EQ(restarted.status(stamp(9), 1.0).n_bricks, static_cast<std::int64_t>(bricks.size()));
}

// Codex P2: seq restarts at 0, so a restart that kept map_epoch reissued (map_epoch, seq)
// cursors, and an old client's cursor matched a new, unrelated event: an empty answer.
TEST(sync_cursor_from_before_a_restart_names_nothing_after_it) {
    TempDir dir;
    const Cloud room = scenes::room_shell();
    const Cloud box = scenes::box_shell({0.3, 1.2, 0.3});
    const Cloud boxed = scenes::concat({&room, &box});
    const MemoryConfig config = mid360_config(dir.path.string());
    RoomMemory memory(config, kStart);
    const std::int64_t second = build_reference(memory, room);
    memory.add_points("mid360", boxed);
    const auto old = memory.close_window(stamp(second));
    CHECK(old.size() == 1 && old[0].header.map_epoch == 1 && old[0].header.seq == 0);
    CHECK(RoomMemory(config, stamp(100)).map_epoch() == 2 && RoomMemory(config, stamp(200)).map_epoch() == 3);
    RoomMemory restarted(config, stamp(300));
    restarted.add_points("mid360", boxed);
    const auto fresh = restarted.close_window(stamp(301));
    CHECK(fresh.size() == 1 && fresh[0].event_id != old[0].event_id);
    CHECK(fresh[0].header.map_epoch == 4 && fresh[0].header.seq == 0);
    msg::RequestSync since;
    since.header = msg::Header{"lab", old[0].header.map_epoch, 0, kStart};
    since.since = msg::RequestSync::Since{old[0].header.map_epoch, old[0].header.seq};
    since.topic = msg::SyncTopic::events;
    CHECK((restarted.sync(since, stamp(302)) == std::vector<msg::Bytes>{fresh[0].to_cdr()}));
}

// Codex P2: a restart with sensors=['mid360'] saved only reference-mid360.cdr at the next
// epoch, so the saved references disagreed and a restart with both sensors refused the room.
TEST(a_sensor_subset_run_keeps_the_room_loadable) {
    TempDir dir;
    const Cloud room = scenes::room_shell();
    Cloud partial;
    for (std::size_t n = 0; n < room.size(); n += 3) {
        if (room[n] < 1.5f) partial.insert(partial.end(), room.begin() + static_cast<std::ptrdiff_t>(n), room.begin() + static_cast<std::ptrdiff_t>(n) + 3);
    }
    MemoryConfig both = mid360_config(dir.path.string());
    both.sensors = {"mid360", "jt128"};
    RoomMemory memory(both, kStart);
    for (std::int64_t second = 0; second < 3; ++second) {
        memory.add_points("mid360", partial);
        memory.add_points("jt128", partial);
        memory.close_window(stamp(second));
    }
    CHECK(memory.state() == MemoryState::tracking && memory.map_epoch() == 1);
    MemoryConfig mid360 = mid360_config(dir.path.string());
    mid360.delta_period_s = 1.0;
    mid360.learn_windows = 30;  // the subset run learns new coverage
    RoomMemory subset(mid360, kStart);
    CHECK(subset.state() == MemoryState::tracking && subset.map_epoch() == 2);
    CHECK_EQ(msg::MapBricks::from_cdr(*subset.map_update(stamp(10))).header.map_epoch, std::uint64_t{2});
    // The subset run learns the far wall, so reference-mid360.cdr moves ahead of reference-jt128.cdr.
    subset.add_points("mid360", room);
    subset.close_window(stamp(11));
    CHECK_EQ(msg::MapBricks::from_cdr(*subset.map_update(stamp(11))).header.map_epoch, std::uint64_t{3});
    CHECK_EQ(subset.map_epoch(), std::uint64_t{3});
    CHECK_EQ(load_reference(reference_path(dir.path.string(), "lab", "mid360")).first.header.map_epoch, std::uint64_t{3});
    CHECK_EQ(load_reference(reference_path(dir.path.string(), "lab", "jt128")).first.header.map_epoch, std::uint64_t{1});
    RoomMemory restarted(both, stamp(20));
    CHECK(restarted.state() == MemoryState::tracking && restarted.map_epoch() == 4);
    CHECK(restarted.channel("mid360").reference.to_bricks() == subset.channel("mid360").reference.to_bricks());
    CHECK_EQ(RoomMemory(both, stamp(30)).map_epoch(), std::uint64_t{5});
    CHECK(load_epoch(epoch_path(dir.path.string(), "lab")) == std::optional<std::uint64_t>{5});
    std::ifstream in(epoch_path(dir.path.string(), "lab"), std::ios::binary);
    CHECK_EQ(std::string((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>()), std::string("5\n"));
}

// A sensor the room has no reference for builds one from the room's epoch, not from 0.
TEST(a_sensor_subset_build_continues_the_room_epoch) {
    TempDir dir;
    MemoryConfig jt128_only = mid360_config(dir.path.string());
    jt128_only.sensors = {"jt128"};
    RoomMemory jt128(jt128_only, kStart);
    for (std::int64_t second = 0; second < 3; ++second) {
        jt128.add_points("jt128", scenes::room_shell());
        jt128.close_window(stamp(second));
    }
    CHECK(jt128.state() == MemoryState::tracking && jt128.map_epoch() == 1);
    CHECK_EQ(RoomMemory(jt128_only, stamp(10)).map_epoch(), std::uint64_t{2});
    RoomMemory mid360(mid360_config(dir.path.string()), kStart);
    CHECK(mid360.state() == MemoryState::building && mid360.map_epoch() == 2);
    build_reference(mid360, scenes::room_shell());
    CHECK(mid360.state() == MemoryState::tracking && mid360.map_epoch() == 3);
    MemoryConfig both = mid360_config(dir.path.string());
    both.sensors = {"mid360", "jt128"};
    CHECK_EQ(RoomMemory(both, stamp(20)).map_epoch(), std::uint64_t{4});
}

TEST(inconsistent_saved_state_is_refused) {
    TempDir dir;
    RoomMemory memory(mid360_config(dir.path.string()), kStart);
    build_reference(memory, scenes::room_shell());
    MemoryConfig both;
    both.state_dir = dir.path.string();
    CHECK_THROWS(std::invalid_argument, RoomMemory(both, kStart));
    MemoryConfig coarse = mid360_config(dir.path.string());
    coarse.voxel_size = 0.1;
    CHECK_THROWS(std::invalid_argument, RoomMemory(coarse, kStart));
    for (const std::string data : {"", "3", "-3\n", " 3\n", "18446744073709551616\n"}) {
        std::ofstream(epoch_path(dir.path.string(), "lab"), std::ios::binary | std::ios::trunc) << data;
        CHECK_THROWS(std::invalid_argument, RoomMemory(mid360_config(dir.path.string()), kStart));
    }
}

// Codex P2: row_step 0 falls back to point_step x width = 2^42, and 2^42 x height (2^22)
// overflows 64 bits. Python rejects it at the data-size check, with this exact message; the
// decoder must too, and add_cdr must count it as a failed frame instead of throwing.
TEST(overflowing_cloud_size_is_rejected) {
    const msg::Bytes cdr = unhex(
        "000100000000000000000000010000000000000000004000000040000300000002000000780000000000000007000000010000000200"
        "000079000000040000000700000001000000020000007a00000008000000070000000100000000000000000010000000000000000000"
        "01");
    std::string message;
    try {
        decode_pointcloud2(cdr);
    } catch (const PointCloudError& error) {
        message = error.what();
    }
    CHECK_EQ(message, std::string("PointCloud2 CDR: data holds 0 bytes but row_step (4398046511104) x height "
                                  "(4194304) needs more"));
    MemoryConfig config;
    config.state_dir = std::nullopt;
    config.reference_windows = 1;
    RoomMemory memory(config, kStart);
    memory.add_points("mid360", scenes::room_shell());
    CHECK(!memory.add_cdr("jt128", cdr));
    memory.close_window(stamp(0));
    CHECK(memory.state() == MemoryState::building && memory.dropped_windows() == 1);
}

TEST(degraded_and_dropped_windows) {
    MemoryConfig config;
    config.state_dir = std::nullopt;
    config.reference_windows = 1;
    config.occupied_min_hits = 1;  // one window settles occupancy: each reference is ready after one window
    RoomMemory memory(config, kStart);
    memory.add_points("mid360", scenes::room_shell());
    const msg::Bytes header_only = {0x00, 0x01, 0x00, 0x00};
    CHECK(!memory.add_cdr("jt128", header_only));
    memory.close_window(stamp(0));
    msg::MemoryStatus status = memory.status(stamp(1), 1.0);
    CHECK(memory.state() == MemoryState::building && memory.dropped_windows() == 1);
    CHECK(status.state == MemoryState::degraded && status.dropped_windows == 1);
    CHECK(status.mid360_hz == 1 && status.jt128_hz == 1);
    CHECK(memory.add_cdr("mid360", encode_pointcloud2(scenes::room_shell())));
    CHECK(memory.add_cdr("jt128", encode_pointcloud2(scenes::room_shell())));
    memory.close_window(stamp(1));
    status = memory.status(stamp(2), 1.0);
    CHECK(memory.state() == MemoryState::tracking && status.state == MemoryState::tracking && memory.map_epoch() == 1);
    status = memory.status(stamp(3), 1.0);
    CHECK(status.state == MemoryState::degraded && status.mid360_hz == 0 && status.jt128_hz == 0);
}

// Codex P2 (round 6): the node warns with the decoder's error, so add_cdr keeps it. Ported
// from spark/room_memory/tests/test_rejectlog.py.
TEST(rejected_cdr_keeps_type_name_and_message) {
    MemoryConfig config;
    config.state_dir = std::nullopt;
    RoomMemory memory(config, kStart);
    CHECK(!memory.channel("mid360").last_error);
    CHECK(!memory.add_cdr("mid360", msg::Bytes{0x00, 0x01, 0x00, 0x00}));
    const RejectedFrame truncated{"PointCloudError", "PointCloud2 CDR: truncated header.stamp.sec at byte 4 (need 4, have 0)"};
    CHECK(memory.channel("mid360").last_error == truncated && !memory.channel("jt128").last_error);
    CHECK(memory.channel("mid360").received == 1 && memory.channel("mid360").failed == 1);
    RejectionLog log(kRejectionIntervalS, [] { return 0.0; });
    CHECK(log.observe("mid360", truncated.type_name, truncated.message) ==
          "mid360 rejected frame: PointCloudError: " + truncated.message);
    memory.close_window(stamp(0));
    CHECK(memory.channel("mid360").last_error == truncated && memory.channel("mid360").failed == 0);
    CHECK(!memory.add_cdr("mid360", msg::Bytes{0x00, 0x01, 0x00, 0x00}));
    CHECK(memory.add_cdr("mid360", encode_pointcloud2(scenes::room_shell())));
    // An accepted frame clears last_error; failed is close_window's to clear.
    CHECK(!memory.channel("mid360").last_error && memory.channel("mid360").failed == 1);
}

TEST(rejection_log_throttles_per_sensor) {
    double t = 0;
    RejectionLog log(kRejectionIntervalS, [&] { return t; });
    const auto line = [&](const std::string& sensor, const std::string& message) {
        return log.observe(sensor, "PointCloudError", message);
    };
    using Line = std::optional<std::string>;
    CHECK(kRejectionIntervalS == 10.0);
    CHECK(line("mid360", "bad mid") == Line("mid360 rejected frame: PointCloudError: bad mid"));
    CHECK(line("jt128", "bad jt") == Line("jt128 rejected frame: PointCloudError: bad jt"));
    t = 1.0;
    CHECK(!line("mid360", "bad mid"));
    t = 9.999;
    CHECK(!line("mid360", "bad mid"));
    t = 4.0;  // a clock step back stays inside the window
    CHECK(!line("jt128", "bad jt"));
    t = 10.0;
    CHECK(line("mid360", "bad mid") == Line("mid360 rejected frame: PointCloudError: bad mid (2 suppressed)"));
    CHECK(line("jt128", "bad jt") == Line("jt128 rejected frame: PointCloudError: bad jt (1 suppressed)"));
    t = 11.0;
    CHECK(!line("mid360", "bad mid"));
    t = 20.0;
    // jt128 emits first; that must not clear mid360's suppressed call from t = 11, and the
    // count restarts after each emission.
    CHECK(line("jt128", "bad jt") == Line("jt128 rejected frame: PointCloudError: bad jt"));
    CHECK(line("mid360", "still bad") == Line("mid360 rejected frame: PointCloudError: still bad (1 suppressed)"));
    // The interval restarts at the last emission, not on a grid from t = 0.
    t = 29.0;
    CHECK(!line("mid360", "still bad"));
    t = 30.0;
    CHECK(line("mid360", "still bad") == Line("mid360 rejected frame: PointCloudError: still bad (1 suppressed)"));
}

// Codex fix 1: a reference window needs at least one occupied voxel.
TEST(windows_without_usable_points_do_not_build_the_reference) {
    TempDir dir;
    const Cloud room = scenes::room_shell();
    MemoryConfig config = mid360_config(dir.path.string());
    config.max_range = 15.0;
    RoomMemory memory(config, kStart);
    Cloud far = room;
    for (float& v : far) v += 50.0f;
    const std::vector<Cloud> unusable = {Cloud{}, Cloud(150, std::nanf("")), far};
    for (std::size_t second = 0; second < unusable.size(); ++second) {
        memory.add_points("mid360", unusable[second]);
        CHECK(memory.close_window(stamp(static_cast<std::int64_t>(second))).empty());
    }
    CHECK(memory.state() == MemoryState::building && memory.map_epoch() == 0);
    CHECK_EQ(memory.channel("mid360").reference_windows, std::int64_t{0});
    CHECK(!std::filesystem::exists(reference_path(dir.path.string(), "lab", "mid360")));
    for (int second = 3; second < 6; ++second) {
        memory.add_points("mid360", room);
        memory.close_window(stamp(second));
    }
    CHECK(memory.state() == MemoryState::tracking && memory.map_epoch() == 1);
    const Cloud box = scenes::box_shell({0.3, 1.2, 0.3});
    memory.add_points("mid360", scenes::concat({&room, &box}));
    const auto events = memory.close_window(stamp(6));
    CHECK((transitions(events) == std::vector<std::pair<EventKind, EventState>>{{EventKind::appeared, EventState::candidate}}));
}

// Codex fix 2: each sensor's reference freezes at its own window count.
TEST(each_sensor_reference_freezes_at_its_own_window_count) {
    const Cloud room = scenes::room_shell();
    const Cloud box = scenes::box_shell({0.3, 1.2, 0.3});
    const Cloud with_box = scenes::concat({&room, &box});
    MemoryConfig config = mid360_config(std::nullopt);
    config.sensors = MemoryConfig().sensors;
    config.reference_windows = 1;
    RoomMemory memory(config, kStart);
    memory.add_points("mid360", room);
    memory.close_window(stamp(0));
    CHECK(memory.state() == MemoryState::building);
    memory.add_points("mid360", with_box);  // the JT128 arrives late; the box is not in the Mid-360 reference
    memory.add_points("jt128", room);
    CHECK(memory.close_window(stamp(1)).empty());
    CHECK(memory.state() == MemoryState::tracking);
    CHECK_EQ(memory.channel("mid360").reference_windows, std::int64_t{1});
    memory.add_points("mid360", with_box);
    const auto events = memory.close_window(stamp(2));
    CHECK(events.size() == 1 && events[0].kind == EventKind::appeared && events[0].state == EventState::candidate &&
          events[0].sensors == msg::kSensorMid360);
}

// Codex P2: reference_windows=1 under occupied_min_hits=2 settled no occupancy in its one
// window, so TRACKING began on an empty reference that pose_offset could never register on and
// no window could ever teach. The reference keeps building until it has occupancy.
TEST(tracking_waits_for_settled_reference_occupancy) {
    const Cloud room = scenes::room_shell();
    MemoryConfig config;
    config.sensors = {"mid360"};
    config.state_dir = std::nullopt;
    config.max_range = std::nullopt;
    config.reference_windows = 1;
    config.candidate_windows = 3;
    RoomMemory memory(config, kStart);
    memory.add_points("mid360", room);
    memory.close_window(stamp(0));
    CHECK(memory.state() == MemoryState::building && memory.map_epoch() == 0);
    CHECK(memory.channel("mid360").reference_windows == 1 && memory.channel("mid360").reference.occupied.empty());
    memory.add_points("mid360", room);
    memory.close_window(stamp(1));
    CHECK(memory.state() == MemoryState::tracking && memory.map_epoch() == 1);
    CHECK(memory.status(stamp(1), 1.0).n_bricks > 0);
    const Cloud box = scenes::box_shell({0.3, 1.2, 0.3});
    const Cloud boxed = scenes::concat({&room, &box});
    std::vector<msg::ChangeEvent> events;
    for (int second = 2; second < 5; ++second) {
        memory.add_points("mid360", boxed);
        for (auto& e : memory.close_window(stamp(second))) events.push_back(std::move(e));
    }
    CHECK((transitions(events) == std::vector<std::pair<EventKind, EventState>>{{EventKind::appeared, EventState::candidate}}));
    CHECK(events[0].first_seen == stamp(2));
}

// Codex P2: with recent_windows=3 a box seen in one window stayed in three consecutive reports
// and passed the 3-window persistence gate alone. Only windows that observe it count now.
TEST(a_return_kept_in_the_recent_layer_is_not_fresh_evidence) {
    const Cloud room = scenes::room_shell();
    const Cloud box = scenes::box_shell({0.3, 1.2, 0.3});
    const Cloud boxed = scenes::concat({&room, &box});
    MemoryConfig config = mid360_config(std::nullopt);
    config.recent_windows = 3;
    config.candidate_windows = 3;
    config.expire_windows = 3;
    {
        RoomMemory memory(config, kStart);
        const std::int64_t second = build_reference(memory, room);
        memory.add_points("mid360", boxed);
        CHECK(memory.close_window(stamp(second)).empty());
        for (std::int64_t later = second + 1; later < second + 6; ++later) {
            memory.add_points("mid360", room);
            CHECK(memory.close_window(stamp(later)).empty());
        }
    }
    // The same box seen in three windows in a row is a candidate on the third.
    RoomMemory memory(config, kStart);
    const std::int64_t second = build_reference(memory, room);
    std::vector<std::vector<msg::ChangeEvent>> events;
    for (int n = 0; n < 3; ++n) {
        memory.add_points("mid360", boxed);
        events.push_back(memory.close_window(stamp(second + n)));
    }
    CHECK(events[0].empty() && events[1].empty());
    CHECK((transitions(events[2]) == std::vector<std::pair<EventKind, EventState>>{{EventKind::appeared, EventState::candidate}}));
    CHECK(events[2][0].first_seen == stamp(second));
}

// Codex P2: with recent_windows=3 one window's rays through a removed box kept it disappeared
// in three consecutive reports, each counted fresh, so it passed the 3-window gate alone. Only
// windows whose own rays traverse it count now.
TEST(free_space_kept_in_the_recent_layer_is_not_fresh_evidence) {
    const Cloud room = scenes::room_shell();
    const Cloud box = scenes::box_shell({0.3, 1.2, 0.3});
    const Cloud boxed = scenes::concat({&room, &box});
    // The other half of the room: no ray from the origin to it comes near the box.
    Cloud other_half;
    for (std::size_t n = 0; n < room.size(); n += 3) {
        if (room[n + 1] < 0) other_half.insert(other_half.end(), room.begin() + n, room.begin() + n + 3);
    }
    MemoryConfig config = mid360_config(std::nullopt);
    config.recent_windows = 3;
    config.candidate_windows = 3;
    config.expire_windows = 3;
    {
        RoomMemory memory(config, kStart);
        std::int64_t second = build_reference(memory, boxed);
        for (const Cloud* points : std::initializer_list<const Cloud*>{&room, &other_half, &other_half, &other_half}) {
            memory.add_points("mid360", *points);
            CHECK(memory.close_window(stamp(second++)).empty());
        }
    }
    // The box missing from three windows in a row is a candidate on the third.
    RoomMemory memory(config, kStart);
    const std::int64_t second = build_reference(memory, boxed);
    std::vector<std::vector<msg::ChangeEvent>> events;
    for (int n = 0; n < 3; ++n) {
        memory.add_points("mid360", room);
        events.push_back(memory.close_window(stamp(second + n)));
    }
    CHECK(events[0].empty() && events[1].empty());
    CHECK((transitions(events[2]) == std::vector<std::pair<EventKind, EventState>>{{EventKind::disappeared, EventState::candidate}}));
    CHECK(events[2][0].first_seen == stamp(second));
}

// Astra: a pending region kept its fresh-window count across stale windows, so fresh, stale,
// fresh, stale, fresh promoted under a 3-window gate. A stale window keeps the match but sets the
// run to 0, and a stale region alone starts nothing.
TEST(a_stale_window_resets_the_run_and_starts_nothing) {
    CandidateTracker tracker(3, 1);
    std::uint64_t ids = 100;
    const auto next = [&] { return ids++; };
    Region box;
    box.kind = Region::Kind::appeared;
    box.keys = keys({{0, 0, 0}, {1, 0, 0}});
    CHECK(tracker.update({box}, stamp(0), next, {false}).empty() && tracker.pending().empty());
    const bool flags[] = {true, false, true, false, true};
    for (int n = 0; n < 5; ++n) {
        CHECK(tracker.update({box}, stamp(n + 1), next, {flags[n]}).empty());
        CHECK(tracker.pending().size() == 1 && tracker.pending()[0].windows == (flags[n] ? 1 : 0));
    }
    CHECK(tracker.update({box}, stamp(6), next, {true}).empty());
    const auto started = tracker.update({box}, stamp(7), next, {true});
    CHECK(started.size() == 1 && started[0].first == EventState::candidate && started[0].second.first_seen == stamp(1));
}

// With 1 cm injection noise the diff's move-count gate flickered on the static recording: the
// open MOVED expired and reopened as an APPEARED plus a DISAPPEARED every few seconds.
TEST(a_move_reported_as_its_halves_stays_one_open_move) {
    CandidateTracker tracker(1, 1);
    std::uint64_t ids = 100;
    const auto next = [&] { return ids++; };
    Region move;
    move.kind = Region::Kind::moved;
    move.keys = keys({{10, 0, 0}, {11, 0, 0}});
    move.from_keys = keys({{0, 0, 0}, {1, 0, 0}});
    Region appeared;
    appeared.kind = Region::Kind::appeared;
    appeared.keys = keys({{11, 0, 0}, {12, 0, 0}});
    Region disappeared;
    disappeared.kind = Region::Kind::disappeared;
    disappeared.keys = keys({{0, 0, 0}});
    Region speck = appeared;
    speck.keys = keys({{30, 0, 0}});
    const auto opened = tracker.update({move}, stamp(0), next);
    CHECK(opened.size() == 1 && opened[0].first == EventState::candidate && opened[0].second.kind == EventKind::moved);
    const auto halves = tracker.update({appeared, disappeared, speck}, stamp(1), next);
    CHECK(halves.size() == 1 && halves[0].first == EventState::candidate && halves[0].second.event_id == 101);
    CHECK(tracker.pending().empty() && tracker.open().size() == 2);
    CHECK(tracker.open()[0].event_id == 100 && tracker.open()[0].kind == EventKind::moved);
    CHECK(tracker.open()[0].last_seen == stamp(1) && tracker.open()[0].missed == 0);
    const auto gone = tracker.update({}, stamp(2), next);
    CHECK(gone.size() == 2 && gone[0].first == EventState::expired && gone[0].second.event_id == 100 &&
          gone[0].second.last_seen == stamp(1) && gone[1].second.event_id == 101);
}

// Codex P2: an object moved from A to B and then removed from B left only the DISAPPEARED at A,
// which kept resetting the open MOVED: the move never expired and the removal never started a
// candidate. Only both halves together keep a move alive now; a lone one starts nothing until
// the move has expired.
TEST(a_lone_half_keeps_nothing_alive) {
    CandidateTracker tracker(3, 3);
    std::uint64_t ids = 100;
    const auto next = [&] { return ids++; };
    Region move;
    move.kind = Region::Kind::moved;
    move.keys = keys({{10, 0, 0}, {11, 0, 0}});
    move.from_keys = keys({{0, 0, 0}, {1, 0, 0}});
    Region removed;
    removed.kind = Region::Kind::disappeared;
    removed.keys = move.from_keys;
    std::vector<std::vector<std::pair<EventState, Candidate>>> windows;
    for (std::int64_t n = 0; n < 13; ++n) windows.push_back(tracker.update({n < 3 ? move : removed}, stamp(n), next));
    std::vector<std::size_t> busy;
    for (std::size_t n = 0; n < windows.size(); ++n) {
        if (!windows[n].empty()) busy.push_back(n);
    }
    CHECK((busy == std::vector<std::size_t>{2, 5, 8}));
    CHECK(windows[2].size() == 1 && windows[2][0].first == EventState::candidate &&
          windows[2][0].second.event_id == 100 && windows[2][0].second.kind == EventKind::moved);
    CHECK(windows[5].size() == 1 && windows[5][0].first == EventState::expired &&
          windows[5][0].second.event_id == 100 && windows[5][0].second.last_seen == stamp(2));
    CHECK(windows[8].size() == 1 && windows[8][0].first == EventState::candidate &&
          windows[8][0].second.event_id == 101 && windows[8][0].second.kind == EventKind::disappeared &&
          windows[8][0].second.first_seen == stamp(6));
    // The other half alone, without persistence: the move expires that window, the half starts the next.
    CandidateTracker single(1, 1);
    single.update({move}, stamp(0), next);
    Region appeared;
    appeared.kind = Region::Kind::appeared;
    appeared.keys = keys({{11, 0, 0}, {12, 0, 0}});
    const auto gone = single.update({appeared}, stamp(1), next);
    CHECK(gone.size() == 1 && gone[0].first == EventState::expired && gone[0].second.kind == EventKind::moved &&
          single.open().empty());
    const auto opened = single.update({appeared}, stamp(2), next);
    CHECK(opened.size() == 1 && opened[0].first == EventState::candidate && opened[0].second.kind == EventKind::appeared);
}

// Codex fix 3: windows without usable data still age the recent layer.
TEST(recent_observations_age_through_missing_windows) {
    const Cloud room = scenes::room_shell();
    MemoryConfig config = mid360_config(std::nullopt);
    config.recent_windows = 3;
    RoomMemory memory(config, kStart);
    const std::int64_t second = build_reference(memory, room);
    const Cloud box = scenes::box_shell({0.3, 1.2, 0.3});
    memory.add_points("mid360", scenes::concat({&room, &box}));
    const auto appeared = memory.close_window(stamp(second));
    CHECK((transitions(appeared) == std::vector<std::pair<EventKind, EventState>>{{EventKind::appeared, EventState::candidate}}));
    CHECK(memory.close_window(stamp(second + 1)).empty());
    const msg::Bytes header_only = {0x00, 0x01, 0x00, 0x00};
    CHECK(!memory.add_cdr("mid360", header_only));
    CHECK(memory.close_window(stamp(second + 2)).empty());
    memory.add_points("mid360", room);
    const auto expired = memory.close_window(stamp(second + 3));
    CHECK(expired.size() == 1 && expired[0].event_id == appeared[0].event_id && expired[0].state == EventState::expired);
    CHECK(expired[0].last_seen == stamp(second));
}

TEST(config_rejections) {
    MemoryConfig velodyne;
    velodyne.sensors = {"velodyne"};
    CHECK_THROWS(std::invalid_argument, RoomMemory(velodyne, kStart));
    MemoryConfig none;
    none.sensors = {};
    CHECK_THROWS(std::invalid_argument, RoomMemory(none, kStart));
    MemoryConfig instant;
    instant.window_s = 0;
    instant.state_dir = std::nullopt;
    CHECK_THROWS(std::invalid_argument, RoomMemory(instant, kStart));
    for (const double bad : {-1.0, std::numeric_limits<double>::infinity(), std::numeric_limits<double>::quiet_NaN()}) {
        instant.window_s = bad;
        CHECK_THROWS(std::invalid_argument, RoomMemory(instant, kStart));
    }
}

TEST(voxel_stats_count_windows_and_settle_by_rule) {
    VoxelStats stats;
    stats.add(keys({{0, 0, 0}}), keys({{0, 0, 0}, {1, 0, 0}, {2, 0, 0}}));
    stats.add(keys({{0, 0, 0}, {2, 0, 0}}), keys({{0, 0, 0}, {2, 0, 0}, {3, 0, 0}}));
    stats.add(keys({}), keys({{0, 0, 0}, {1, 0, 0}}));
    CHECK(stats.keys == keys({{0, 0, 0}, {1, 0, 0}, {2, 0, 0}, {3, 0, 0}}));
    CHECK((stats.hits == std::vector<std::uint32_t>{2, 0, 1, 0}));
    CHECK((stats.seen == std::vector<std::uint32_t>{3, 2, 2, 1}));
    // Union rule (the defaults): ever hit is occupied, ever seen is known.
    VoxelLayer all = stats.settled(SettleRule{});
    CHECK(all.occupied == keys({{0, 0, 0}, {2, 0, 0}}) && all.known == stats.keys);
    // Two hits in at least 60 % of the windows, free after two clean windows: (2,0,0) flickers
    // (1 of 2) and (3,0,0) was seen once, so both stay UNKNOWN.
    VoxelLayer strict = stats.settled(SettleRule{2, 0.6, 2, 0.0});
    CHECK(strict.occupied == keys({{0, 0, 0}}) && strict.known == keys({{0, 0, 0}, {1, 0, 0}}));
    // A tolerated hit share makes the flicker free.
    CHECK(stats.settled(SettleRule{2, 0.6, 2, 0.5}).known == keys({{0, 0, 0}, {1, 0, 0}, {2, 0, 0}}));
    stats.erase(strict.known);
    CHECK(stats.keys == keys({{2, 0, 0}, {3, 0, 0}}) && (stats.hits == std::vector<std::uint32_t>{1, 0}));
}

TEST(map_delta_sends_changed_bricks_and_removes_absent_ones) {
    const auto brick = [](std::int64_t i, std::uint8_t bits) {
        msg::Brick b;
        b.coordinate = {i, 0, 0};
        b.known = mask(bits);
        return b;
    };
    const std::vector<msg::Brick> before = {brick(0, 1), brick(1, 1), brick(2, 1), brick(4, 1)};
    const std::vector<msg::Brick> after = {brick(0, 1), brick(2, 3), brick(3, 1), brick(4, 1)};
    const MapDelta delta = map_delta(before, after);
    CHECK((delta.removed == std::vector<std::int64_t>{1}));
    CHECK(delta.bricks.size() == 2 && delta.bricks[0] == after[1] && delta.bricks[1] == after[2]);
    CHECK(map_delta(after, after).bricks.empty() && map_delta(after, after).removed.empty());
}

TEST(candidate_gate_waits_and_expiry_has_hysteresis) {
    CandidateTracker tracker(3, 2);
    std::uint64_t ids = 100;
    const auto next = [&] { return ids++; };
    Region box;
    box.kind = Region::Kind::appeared;
    box.keys = keys({{0, 0, 0}, {1, 0, 0}});
    Region speck = box;
    speck.keys = keys({{9, 9, 9}});
    CHECK(tracker.update({box, speck}, stamp(0), next).empty());
    CHECK(tracker.update({box}, stamp(1), next).empty());  // the speck's pending entry is dropped
    CHECK(tracker.pending().size() == 1 && tracker.open().empty());
    const auto started = tracker.update({box, speck}, stamp(2), next);
    CHECK(started.size() == 1 && started[0].first == EventState::candidate && started[0].second.event_id == 100);
    CHECK(started[0].second.first_seen == stamp(0) && started[0].second.last_seen == stamp(2));
    CHECK(tracker.update({}, stamp(3), next).empty());  // one miss is tolerated
    CHECK(tracker.update({box}, stamp(4), next).empty());
    CHECK(tracker.update({}, stamp(5), next).empty());
    const auto gone = tracker.update({}, stamp(6), next);
    CHECK(gone.size() == 1 && gone[0].first == EventState::expired && gone[0].second.last_seen == stamp(4));
    CHECK(tracker.open().empty() && ids == 101);
}

TEST(map_stream_snapshot_then_deltas_and_keyframes) {
    const Cloud room = scenes::room_shell();
    Cloud partial;
    for (std::size_t n = 0; n < room.size(); n += 3) {
        if (room[n] < 1.5f) partial.insert(partial.end(), room.begin() + static_cast<std::ptrdiff_t>(n), room.begin() + static_cast<std::ptrdiff_t>(n) + 3);
    }
    MemoryConfig config = mid360_config(std::nullopt);
    config.learn_windows = 30;
    config.delta_period_s = 2.0;
    config.keyframe_period_s = 6.0;
    config.keyframe_delta_ratio = 100.0;
    RoomMemory memory(config, kStart);
    CHECK(!memory.map_update(stamp(0)));
    const std::int64_t second = build_reference(memory, partial);
    std::vector<msg::MapBricks> sent;
    std::vector<std::int64_t> at;
    for (std::int64_t s = second; s < second + 9; ++s) {
        memory.add_points("mid360", room);
        memory.close_window(stamp(s));
        if (const auto bytes = memory.map_update(stamp(s))) {
            sent.push_back(msg::MapBricks::from_cdr(*bytes));
            at.push_back(s - second);
        }
    }
    // Snapshot at once; the new wall settles and goes out as a delta after delta_period_s;
    // nothing while the map is unchanged; a keyframe once keyframe_period_s has passed.
    CHECK((at == std::vector<std::int64_t>{0, 2, 6}));
    CHECK(sent[0].kind == msg::MapKind::snapshot && sent[0].header.map_epoch == 1);
    CHECK(sent[1].kind == msg::MapKind::delta && sent[1].base_epoch == 1 && sent[1].header.map_epoch == 2);
    CHECK(sent[2].kind == msg::MapKind::snapshot && sent[2].header.map_epoch == 2);
    for (std::size_t n = 0; n < sent.size(); ++n) CHECK_EQ(sent[n].header.seq, std::uint64_t{n});
    // The mirror: snapshot + delta reproduces the keyframe and the reference.
    std::map<std::array<std::int64_t, 3>, msg::Brick> mirror;
    for (const msg::Brick& b : sent[0].bricks) mirror[b.coordinate] = b;
    for (const msg::Brick& b : sent[1].bricks) mirror[b.coordinate] = b;
    std::vector<msg::Brick> mirrored;
    for (const auto& [c, b] : mirror) mirrored.push_back(b);
    CHECK(mirrored == sent[2].bricks && mirrored == memory.channel("mid360").reference.to_bricks());
    CHECK(sent[1].bricks.size() < sent[2].bricks.size());
    CHECK_EQ(memory.status(stamp(second + 9), 1.0).n_bricks, static_cast<std::int64_t>(mirrored.size()));
}

TEST(map_stream_sends_a_snapshot_when_deltas_outgrow_the_keyframe) {
    const Cloud room = scenes::room_shell();
    Cloud partial;
    for (std::size_t n = 0; n < room.size(); n += 3) {
        if (room[n] < 1.5f) partial.insert(partial.end(), room.begin() + static_cast<std::ptrdiff_t>(n), room.begin() + static_cast<std::ptrdiff_t>(n) + 3);
    }
    MemoryConfig config = mid360_config(std::nullopt);
    config.learn_windows = 30;
    config.delta_period_s = 1.0;
    config.keyframe_delta_ratio = 0.01;
    RoomMemory memory(config, kStart);
    const std::int64_t second = build_reference(memory, partial);
    CHECK(msg::MapBricks::from_cdr(*memory.map_update(stamp(second))).kind == msg::MapKind::snapshot);
    memory.add_points("mid360", room);
    memory.close_window(stamp(second + 1));
    const msg::MapBricks next = msg::MapBricks::from_cdr(*memory.map_update(stamp(second + 1)));
    CHECK(next.kind == msg::MapKind::snapshot && next.header.map_epoch == 2 && next.header.seq == 1);
}

TEST(sync_replays_or_snapshots_maps_and_replays_or_falls_back_for_events) {
    const Cloud room = scenes::room_shell();
    const Cloud box = scenes::box_shell({0.3, 1.2, 0.3});
    const Cloud boxed = scenes::concat({&room, &box});
    MemoryConfig config = mid360_config(std::nullopt);
    config.delta_period_s = 1.0;
    config.keyframe_period_s = 1.0;
    RoomMemory memory(config, kStart);
    const auto request = [](std::optional<msg::RequestSync::Since> since, msg::SyncTopic topic, const std::string& room_id = "lab") {
        msg::RequestSync r;
        r.header = msg::Header{room_id, since ? since->epoch : msg::kSinceNone, 0, kStart};
        r.since = since;
        r.topic = topic;
        return r;
    };
    CHECK(memory.sync(request(std::nullopt, msg::SyncTopic::map_bricks), stamp(0)).empty());  // no map yet
    std::int64_t second = build_reference(memory, room);
    std::vector<msg::Bytes> live;
    for (int n = 0; n < 3; ++n, ++second) {
        memory.add_points("mid360", n == 1 ? boxed : room);
        memory.close_window(stamp(second));
        live.push_back(*memory.map_update(stamp(second)));  // a keyframe every window
    }
    const auto map_answer = [&](std::optional<msg::RequestSync::Since> since) {
        return memory.sync(request(since, msg::SyncTopic::map_bricks), stamp(second));
    };
    const msg::MapBricks first = msg::MapBricks::from_cdr(live[0]);
    CHECK((map_answer(msg::RequestSync::Since{first.header.map_epoch, 0}) == std::vector<msg::Bytes>{live[1], live[2]}));
    for (const auto since : {std::optional<msg::RequestSync::Since>{}, std::optional<msg::RequestSync::Since>{{first.header.map_epoch, 2}},
                             std::optional<msg::RequestSync::Since>{{first.header.map_epoch + 7, 0}},
                             std::optional<msg::RequestSync::Since>{{first.header.map_epoch, 40}}}) {
        const std::vector<msg::Bytes> answer = map_answer(since);
        CHECK(answer.size() == 1);
        const msg::MapBricks snapshot = msg::MapBricks::from_cdr(answer[0]);
        CHECK(snapshot.kind == msg::MapKind::snapshot && snapshot.header.seq == 2);
        CHECK(snapshot.bricks == msg::MapBricks::from_cdr(live[2]).bricks);
    }
    CHECK(memory.sync(request(std::nullopt, msg::SyncTopic::map_bricks, "other"), stamp(second)).empty());
    // Events: a candidate (window 1) and its expiry (window 2).
    const std::vector<msg::Bytes> all = memory.sync(request(std::nullopt, msg::SyncTopic::events), stamp(second));
    CHECK(all.size() == 2);
    const msg::ChangeEvent opened = msg::ChangeEvent::from_cdr(all[0]);
    CHECK(opened.state == EventState::candidate && msg::ChangeEvent::from_cdr(all[1]).state == EventState::expired);
    const auto since_open = memory.sync(request(msg::RequestSync::Since{opened.header.map_epoch, opened.header.seq}, msg::SyncTopic::events), stamp(second));
    CHECK((since_open == std::vector<msg::Bytes>{all[1]}));
    const auto since_last = memory.sync(request(msg::RequestSync::Since{opened.header.map_epoch, 1}, msg::SyncTopic::events), stamp(second));
    CHECK(since_last.empty());
}

TEST(event_fallback_resends_open_events_older_than_the_log) {
    RoomMemory memory(mid360_config(std::nullopt), kStart);
    const Cloud room = scenes::room_shell();
    std::int64_t second = build_reference(memory, room);
    const Cloud box = scenes::box_shell({0.3, 1.2, 0.3});
    const Cloud speck = scenes::box_shell({-0.8, -0.8, 0.3});
    const Cloud boxed = scenes::concat({&room, &box});
    const Cloud both = scenes::concat({&room, &box, &speck});
    memory.add_points("mid360", boxed);
    const auto opened = memory.close_window(stamp(second++));
    CHECK(opened.size() == 1 && opened[0].state == EventState::candidate);
    // The box stays (no transition) while the speck comes and goes: 100 more transitions push
    // the box's only message out of the log.
    std::size_t transitions = 0;
    while (transitions < kEventLogDepth) {
        memory.add_points("mid360", transitions % 2 ? boxed : both);
        transitions += memory.close_window(stamp(second++)).size();
    }
    CHECK(transitions == kEventLogDepth);
    msg::RequestSync r;
    r.header = msg::Header{"lab", msg::kSinceNone, 0, kStart};
    r.topic = msg::SyncTopic::events;
    const std::vector<msg::Bytes> answer = memory.sync(r, stamp(second));
    CHECK(answer.size() == kEventLogDepth + 1 && answer[0] == opened[0].to_cdr());
    CHECK(msg::ChangeEvent::from_cdr(answer[1]).header.seq == 1);
}

TEST(coverage_and_pose_residual) {
    const Cloud room = scenes::room_shell();
    RoomMemory memory(mid360_config(std::nullopt), kStart);
    const std::int64_t second = build_reference(memory, room);
    const msg::MemoryStatus building = memory.status(stamp(second), 1.0);
    CHECK(building.coverage_known_frac && *building.coverage_known_frac > 0.2f && *building.coverage_known_frac <= 1.0f);
    CHECK(!building.pose_residual_m);  // the guard has not run
    memory.add_points("mid360", room);
    memory.close_window(stamp(second));
    CHECK(memory.status(stamp(second), 1.0).pose_residual_m == 0.0f);
    // The rig shifted by (0.1, 0.05, 0.1) m: each wall registers on its own axis.
    Cloud shifted = room;
    for (std::size_t n = 0; n < shifted.size(); n += 3) {
        shifted[n] += 0.1f;
        shifted[n + 1] += 0.05f;
        shifted[n + 2] += 0.1f;
    }
    memory.add_points("mid360", shifted);
    memory.close_window(stamp(second + 1));
    const float residual = *memory.status(stamp(second + 1), 1.0).pose_residual_m;
    CHECK(std::fabs(residual - 0.15f) <= 0.01f);
    // A shifted window teaches the reference nothing.
    CHECK(memory.channel("mid360").stats.keys.empty());
}

// Overlap counts 9, 10, 1 along x refine to -0.4 voxels: 0.020000000000000004 m at 5 cm, just
// above learn_max_offset_m (0.02), where float rounding would land just below it and learn. The
// Python oracle pins the same value.
TEST(learning_budget_flushes_on_the_next_due_tick_then_freezes) {
    const Cloud room = scenes::room_shell();
    const Cloud partial = partial_shell(room);
    RoomMemory memory(budget_config(), kStart);
    const auto opening = feed(memory, &partial, 0);
    CHECK(opening && opening->kind == msg::MapKind::snapshot && opening->header.map_epoch == 1);
    CHECK(memory.channel("mid360").stats.keys.empty());
    // The learning window is not due, so the new wall stays in the counts.
    CHECK(!feed(memory, &room, 1) && !memory.channel("mid360").stats.keys.empty());
    CHECK(!feed(memory, &room, 2) && !memory.channel("mid360").stats.keys.empty());
    const auto flushed = feed(memory, &room, 3);
    CHECK(flushed && flushed->kind == msg::MapKind::delta && flushed->header.map_epoch == 2);
    CHECK(flushed->base_epoch == 1);
    const auto learned = memory.channel("mid360").reference.to_bricks();
    CHECK(learned.size() > opening->bricks.size() && memory.channel("mid360").stats.keys.empty());
    const auto key = feed(memory, &room, 4);
    CHECK(key && key->kind == msg::MapKind::snapshot && key->header.map_epoch == 2 && key->bricks == learned);
    CHECK(!feed(memory, &room, 5));
    const auto again = feed(memory, &room, 8);
    CHECK(again && again->kind == msg::MapKind::snapshot && again->header.map_epoch == 2 && again->bricks == learned);
    const Cloud box = scenes::box_shell({0.3, 1.2, 0.3});
    const Cloud boxed = scenes::concat({&room, &box});
    memory.add_points("mid360", boxed);
    const auto events = memory.close_window(stamp(9));
    CHECK(std::any_of(events.begin(), events.end(), [](const msg::ChangeEvent& event) {
        return event.kind == EventKind::appeared && std::fabs(event.centroid[0] - 0.3) <= 0.05 &&
               std::fabs(event.centroid[1] - 1.2) <= 0.05 && std::fabs(event.centroid[2] - 0.3) <= 0.05;
    }));
    CHECK(!memory.map_update(stamp(9)));
    const Cloud far_box = scenes::box_shell({8.0, 8.0, 0.5});
    const Cloud far = scenes::concat({&room, &far_box});
    memory.add_points("mid360", far);
    // The open box expires. The outside cluster is not a candidate and is not learned.
    const auto far_events = memory.close_window(stamp(10));
    CHECK(std::none_of(far_events.begin(), far_events.end(),
                       [](const msg::ChangeEvent& event) { return event.state == EventState::candidate; }));
    const auto frozen = feed(memory, &room, 12);
    CHECK(frozen && frozen->kind == msg::MapKind::snapshot && frozen->header.map_epoch == 2 &&
          frozen->bricks == learned);
}

TEST(an_empty_tracking_window_consumes_the_learn_budget) {
    const Cloud room = scenes::room_shell();
    const Cloud partial = partial_shell(room);
    RoomMemory memory(budget_config(), kStart);
    const auto opening = feed(memory, &partial, 0);
    CHECK(opening && !feed(memory, nullptr, 1) && memory.channel("mid360").stats.keys.empty());
    memory.add_points("mid360", room);
    CHECK(memory.close_window(stamp(2)).empty() && memory.channel("mid360").stats.keys.empty());
    CHECK(!memory.map_update(stamp(3)) && memory.map_epoch() == 1);
    CHECK(memory.channel("mid360").reference.to_bricks() == opening->bricks);
    const auto key = feed(memory, &room, 4);
    CHECK(key && key->kind == msg::MapKind::snapshot && key->header.map_epoch == 1 && key->bricks == opening->bricks);
}

TEST(a_pose_rejected_window_consumes_the_learn_budget) {
    const Cloud room = scenes::room_shell();
    const Cloud partial = partial_shell(room);
    RoomMemory memory(budget_config(), kStart);
    const auto opening = feed(memory, &partial, 0);
    Cloud shifted = partial;
    for (std::size_t n = 0; n < shifted.size(); n += 3) {
        shifted[n] += 0.1f;
        shifted[n + 1] += 0.05f;
        shifted[n + 2] += 0.1f;
    }
    CHECK(opening && !feed(memory, &shifted, 1));
    const auto residual = memory.channel("mid360").residual;
    CHECK(residual && *residual > memory.config().learn_max_offset_m);
    CHECK(memory.channel("mid360").stats.keys.empty());
    memory.add_points("mid360", room);
    // The shifted window's candidate expires. The aligned room teaches nothing.
    const auto settled = memory.close_window(stamp(2));
    CHECK(std::none_of(settled.begin(), settled.end(),
                       [](const msg::ChangeEvent& event) { return event.state == EventState::candidate; }));
    CHECK(memory.channel("mid360").stats.keys.empty());
    CHECK(!memory.map_update(stamp(3)) && memory.map_epoch() == 1);
    const auto key = feed(memory, &room, 4);
    CHECK(key && key->kind == msg::MapKind::snapshot && key->header.map_epoch == 1 && key->bricks == opening->bricks);
}

TEST(learn_windows_zero_drops_leftovers_and_keeps_known_space_events) {
    const Cloud room = scenes::room_shell();
    const Cloud partial = partial_shell(room);
    RoomMemory memory(budget_config(0), kStart);
    const auto opening = feed(memory, &partial, 0);
    CHECK(opening && memory.channel("mid360").stats.keys.empty());
    for (std::int64_t second = 1; second < 4; ++second) {
        memory.add_points("mid360", room);
        CHECK(memory.close_window(stamp(second)).empty());
        CHECK(!memory.map_update(stamp(second)) && memory.map_epoch() == 1);
    }
    CHECK(memory.channel("mid360").reference.to_bricks() == opening->bricks);
    const Cloud box = scenes::box_shell({0.3, 1.2, 0.3});
    const Cloud boxed = scenes::concat({&room, &box});
    memory.add_points("mid360", boxed);
    const auto events = memory.close_window(stamp(4));
    CHECK(std::any_of(events.begin(), events.end(), [](const msg::ChangeEvent& event) {
        return event.kind == EventKind::appeared && std::fabs(event.centroid[0] - 0.3) <= 0.05 &&
               std::fabs(event.centroid[1] - 1.2) <= 0.05 && std::fabs(event.centroid[2] - 0.3) <= 0.05;
    }));
    const msg::MapBricks sent = msg::MapBricks::from_cdr(*memory.map_update(stamp(4)));
    CHECK(sent.kind == msg::MapKind::snapshot && sent.header.map_epoch == 1 && sent.bricks == opening->bricks);
    const Cloud far_box = scenes::box_shell({8.0, 8.0, 0.5});
    const Cloud far = scenes::concat({&room, &far_box});
    memory.add_points("mid360", far);
    const auto far_events = memory.close_window(stamp(5));
    CHECK(std::none_of(far_events.begin(), far_events.end(),
                       [](const msg::ChangeEvent& event) { return event.state == EventState::candidate; }));
    CHECK(memory.channel("mid360").reference.to_bricks() == opening->bricks);
}

TEST(a_reload_with_learn_windows_zero_does_not_learn) {
    TempDir dir;
    MemoryConfig config = mid360_config(dir.path.string());
    config.reference_windows = 1;
    config.learn_windows = 0;
    config.delta_period_s = 1.0;
    config.keyframe_period_s = 100.0;
    const Cloud room = scenes::room_shell();
    const Cloud partial = partial_shell(room);
    RoomMemory memory(config, kStart);
    memory.add_points("mid360", partial);
    CHECK(memory.close_window(stamp(0)).empty());
    CHECK(memory.state() == MemoryState::tracking);
    CHECK_EQ(memory.map_epoch(), std::uint64_t{1});
    const auto bricks = memory.channel("mid360").reference.to_bricks();
    RoomMemory restarted(config, stamp(10));
    CHECK(restarted.state() == MemoryState::tracking);
    CHECK_EQ(restarted.map_epoch(), std::uint64_t{2});
    const msg::MapBricks opening = msg::MapBricks::from_cdr(*restarted.map_update(stamp(10)));
    CHECK(opening.header.map_epoch == 2 && opening.bricks == bricks);
    restarted.add_points("mid360", room);
    CHECK(restarted.close_window(stamp(11)).empty());
    CHECK(!restarted.map_update(stamp(11)));
    CHECK_EQ(restarted.map_epoch(), std::uint64_t{2});
    CHECK(restarted.channel("mid360").reference.to_bricks() == bricks);
}

TEST(pose_offset_is_binary64) {
    Keys recent, reference;
    for (std::int64_t m = 0; m < 10; ++m) {
        // 10 voxels apart, beyond the +-4 search, so y and z refine to 0.
        recent.push_back(pack(0, 0, 10 * m));
        reference.push_back(pack(0, 0, 10 * m));
        if (m < 9) reference.push_back(pack(-1, 0, 10 * m));
    }
    reference.push_back(pack(1, 0, 0));
    std::sort(recent.begin(), recent.end());
    std::sort(reference.begin(), reference.end());
    const std::optional<double> offset = pose_offset(recent, Membership(reference), 0.05);
    CHECK(offset && *offset == 0.020000000000000004 && *offset > MemoryConfig{}.learn_max_offset_m);
}

TEST(stream_config_rejections) {
    for (const auto& edit : std::vector<std::function<void(MemoryConfig&)>>{
             [](MemoryConfig& c) { c.occupied_min_hits = 0; }, [](MemoryConfig& c) { c.free_min_seen = 0; },
             [](MemoryConfig& c) { c.occupied_min_ratio = 1.5; }, [](MemoryConfig& c) { c.free_max_ratio = -0.1; },
             [](MemoryConfig& c) { c.candidate_windows = 0; }, [](MemoryConfig& c) { c.expire_windows = 0; },
             [](MemoryConfig& c) { c.delta_period_s = 0; }, [](MemoryConfig& c) { c.keyframe_period_s = NAN; },
             [](MemoryConfig& c) { c.keyframe_delta_ratio = -1; }, [](MemoryConfig& c) { c.map_sensor = "jt128"; },
             [](MemoryConfig& c) { c.learn_max_offset_m = -0.1; },
             [](MemoryConfig& c) { c.learn_windows = -1; }}) {
        MemoryConfig config = mid360_config(std::nullopt);
        edit(config);
        CHECK_THROWS(std::invalid_argument, RoomMemory(config, kStart));
    }
}

// MARK: Activity and the settle gate (the same cases as test_memory.py)

namespace {

Region region_of(Region::Kind kind, const std::vector<std::array<std::int64_t, 3>>& cells,
                 const std::vector<std::array<std::int64_t, 3>>& from_cells = {}) {
    Region r;
    r.kind = kind;
    r.keys = keys(cells);
    r.from_keys = keys(from_cells);
    r.voxel_count = static_cast<std::int64_t>(r.keys.size());
    return r;
}

std::vector<std::array<std::int64_t, 3>> block(std::int64_t x, std::int64_t y = 0, std::int64_t z = 0, std::int64_t side = 3) {
    std::vector<std::array<std::int64_t, 3>> cells;
    for (std::int64_t i = 0; i < side; ++i) {
        for (std::int64_t j = 0; j < side; ++j) {
            for (std::int64_t k = 0; k < side; ++k) cells.push_back({x + i, y + j, z + k});
        }
    }
    return cells;
}

std::vector<std::array<std::int64_t, 3>> join_cells(std::vector<std::array<std::int64_t, 3>> a,
                                                    const std::vector<std::array<std::int64_t, 3>>& b) {
    a.insert(a.end(), b.begin(), b.end());
    return a;
}

// Feeds one region list per window (nullopt: a window without points); returns the windows with a CANDIDATE.
std::vector<int> run_tracker(CandidateTracker& tracker, const std::vector<std::optional<std::vector<Region>>>& windows,
                             const std::vector<std::vector<bool>>& fresh = {}) {
    std::uint64_t ids = 100;
    std::vector<int> out;
    for (std::size_t n = 0; n < windows.size(); ++n) {
        if (!windows[n]) {
            tracker.interrupt();
            continue;
        }
        const std::vector<bool> flags = fresh.empty() ? std::vector<bool>{} : fresh[n];
        for (const auto& [state, c] : tracker.update(*windows[n], stamp(static_cast<std::int64_t>(n)), [&] { return ids++; }, flags)) {
            if (state == EventState::candidate) {
                out.push_back(static_cast<int>(n));
                break;
            }
        }
    }
    return out;
}

using Windows = std::vector<std::optional<std::vector<Region>>>;

}  // namespace

// Astra: a window without usable points must not be bridged.
TEST(a_gap_breaks_the_run) {
    const Region box = region_of(Region::Kind::appeared, block(0));
    CandidateTracker tracker(5, 1, 1, 20, 0.9);
    Windows windows(4, std::vector<Region>{box});
    windows.push_back(std::nullopt);
    for (int n = 0; n < 5; ++n) windows.push_back(std::vector<Region>{box});
    CHECK((run_tracker(tracker, windows) == std::vector<int>{9}));
}

TEST(a_gap_window_in_the_engine_interrupts_pending_regions) {
    const Cloud room = scenes::room_shell();
    const Cloud box = scenes::box_shell({0.3, 1.2, 0.3});
    const Cloud boxed = scenes::concat({&room, &box});
    MemoryConfig config = mid360_config(std::nullopt);
    config.candidate_windows = 3;
    config.expire_windows = 3;
    RoomMemory memory(config, kStart);
    const std::int64_t second = build_reference(memory, room);
    std::vector<int> with_events;
    for (int n = 0; n < 6; ++n) {
        if (n != 2) memory.add_points("mid360", boxed);
        if (!memory.close_window(stamp(second + n)).empty()) with_events.push_back(n);
    }
    CHECK((with_events == std::vector<int>{5}));
}

TEST(a_reshaping_region_is_activity_and_never_promotes) {
    CandidateTracker tracker(3, 1, 1, 20, 0.9);
    Windows windows;
    for (int n = 0; n < 8; ++n) {
        windows.push_back(std::vector<Region>{region_of(Region::Kind::appeared, join_cells(block(0), block(3 + 3 * (n % 2))))});
    }
    CHECK(run_tracker(tracker, windows).empty());
    CHECK((tracker.activity() == std::vector<std::size_t>{0}) && tracker.pending()[0].windows == 1);
}

TEST(a_change_next_to_motion_waits_until_the_motion_settles) {
    CandidateTracker tracker(3, 1, 1, 5, 0.9);
    const Region box = region_of(Region::Kind::appeared, block(0));
    Windows windows;
    for (int n = 0; n < 6; ++n) windows.push_back(std::vector<Region>{box, region_of(Region::Kind::appeared, block(7, 0, 3 * (n % 2)))});
    for (int n = 0; n < 4; ++n) windows.push_back(std::vector<Region>{box});
    CHECK((run_tracker(tracker, windows) == std::vector<int>{8}));
}

TEST(motion_beyond_the_radius_does_not_hold_a_change) {
    CandidateTracker tracker(3, 1, 1, 5, 0.9);
    const Region box = region_of(Region::Kind::appeared, block(0));
    Windows windows;
    for (int n = 0; n < 6; ++n) windows.push_back(std::vector<Region>{box, region_of(Region::Kind::appeared, block(20, 0, 3 * (n % 2)))});
    CHECK((run_tracker(tracker, windows) == std::vector<int>{2}));
}

// The documented limit: a region that stays still, fresh and alone for candidate_windows is a change.
TEST(a_still_residue_beyond_the_gate_is_reported) {
    CandidateTracker tracker(40, 1, 1, 20, 0.9);
    const Windows windows(41, std::vector<Region>{region_of(Region::Kind::disappeared, block(0))});
    CHECK((run_tracker(tracker, windows) == std::vector<int>{39}));
}

// Sparse returns at range: a voxel or two of the boundary comes and goes, and a small fragment off
// its edge joins it instead of being motion.
TEST(an_edge_that_flickers_keeps_the_shape) {
    CandidateTracker tracker(4, 1, 1, 20, 0.9);
    const auto base = block(0, 0, 0, 5);
    auto trimmed = base;
    trimmed.resize(trimmed.size() - 2);
    const Windows windows = {
        std::vector<Region>{region_of(Region::Kind::appeared, base)},
        std::vector<Region>{region_of(Region::Kind::appeared, join_cells(base, {{6, 0, 0}}))},
        std::vector<Region>{region_of(Region::Kind::appeared, base),
                            region_of(Region::Kind::appeared, {{5, 3, 3}, {6, 3, 3}, {5, 4, 3}})},
        std::vector<Region>{region_of(Region::Kind::appeared, trimmed)},
    };
    CHECK((run_tracker(tracker, windows) == std::vector<int>{3}));
    CHECK(tracker.activity().empty());
}

TEST(fresh_needs_a_count_and_a_share_of_the_voxels) {
    const auto cells = block(0);
    const Region appeared = region_of(Region::Kind::appeared, cells);
    const auto seen = [&](std::size_t n) { return keys(std::vector<std::array<std::int64_t, 3>>(cells.begin(), cells.begin() + static_cast<std::ptrdiff_t>(n))); };
    CHECK(!region_fresh(appeared, seen(2), {}, 3, 0.5));
    CHECK(!region_fresh(appeared, seen(13), {}, 3, 0.5));
    CHECK(region_fresh(appeared, seen(14), {}, 3, 0.5));
    const Region moved = region_of(Region::Kind::moved, cells, block(10));
    CHECK(!region_fresh(moved, seen(27), {}, 3, 0.5));
    const auto old_cells = block(10);
    CHECK(region_fresh(moved, seen(27), keys(std::vector<std::array<std::int64_t, 3>>(old_cells.begin(), old_cells.begin() + 14)), 3, 0.5));
    CHECK(region_fresh(region_of(Region::Kind::disappeared, cells), {}, seen(14), 3, 0.5));
}

TEST(a_move_seen_as_its_halves_keeps_its_run) {
    const Region whole = region_of(Region::Kind::moved, block(10), block(0));
    const Region appeared = region_of(Region::Kind::appeared, block(10));
    const Region disappeared = region_of(Region::Kind::disappeared, block(0));
    for (const std::vector<Region>& order : {std::vector<Region>{appeared, disappeared}, std::vector<Region>{disappeared, appeared}}) {
        CandidateTracker tracker(4, 1, 1, 20, 0.9);
        CHECK((run_tracker(tracker, {std::vector<Region>{whole}, order, std::vector<Region>{whole}, order}) == std::vector<int>{3}));
        CHECK(tracker.open().size() == 1 && tracker.open()[0].kind == EventKind::moved);
    }
    CandidateTracker tracker(4, 1, 1, 20, 0.9);
    const std::vector<Region> halves{appeared, disappeared};
    CHECK(run_tracker(tracker, {std::vector<Region>{whole}, halves, halves, halves}, {{}, {true, false}, {}, {}}).empty());
    CHECK(tracker.pending()[0].windows == 2);
}

// Codex P2: a pending move matched by its halves kept the last whole move's geometry, so a half
// that reshaped and then settled was reported at its old place next to its new mask.
TEST(a_move_promoted_through_its_halves_reports_their_geometry) {
    CandidateTracker tracker(3, 1, 1, std::nullopt, 0.9, 0.05);
    const Region whole = region_of(Region::Kind::moved, block(10), block(0));
    const std::vector<Region> halves{region_of(Region::Kind::appeared, block(12)), region_of(Region::Kind::disappeared, block(0))};
    std::uint64_t ids = 100;
    const auto next = [&] { return ids++; };
    CHECK(tracker.update({whole}, stamp(0), next).empty());
    CHECK(tracker.update(halves, stamp(1), next).empty());
    CHECK(tracker.update(halves, stamp(2), next).empty());
    const auto out = tracker.update(halves, stamp(3), next);
    CHECK(out.size() == 1 && out[0].first == EventState::candidate && out[0].second.kind == EventKind::moved);
    const Region& r = out[0].second.region;
    CHECK(r.centroid == (Vec3{static_cast<float>(13.5 * static_cast<double>(0.05f)), static_cast<float>(1.5 * static_cast<double>(0.05f)),
                              static_cast<float>(1.5 * static_cast<double>(0.05f))}));
    CHECK(r.voxel_count == 27 && r.keys == keys(block(12)) && r.bounds_min[0] == 12.0f * 0.05f);
    CHECK(r.from_centroid[0] == static_cast<float>(1.5 * static_cast<double>(0.05f)));
}

// Halves first (2 and 1 settled windows), then the whole move: one pending move whose run is the
// shorter of theirs plus this window, both halves consumed. (No activity check: the new
// disappeared half would hold the appeared one back.)
TEST(a_move_takes_over_its_pending_halves) {
    CandidateTracker tracker(4, 1, 1, std::nullopt, 0.9);
    const Region appeared = region_of(Region::Kind::appeared, block(10));
    const Region disappeared = region_of(Region::Kind::disappeared, block(0));
    const Region whole = region_of(Region::Kind::moved, block(10), block(0));
    run_tracker(tracker, {std::vector<Region>{appeared}, std::vector<Region>{appeared, disappeared}});
    CHECK(tracker.pending().size() == 2 && tracker.pending()[0].windows == 2 && tracker.pending()[1].windows == 1);
    std::uint64_t ids = 100;
    const auto next = [&] { return ids++; };
    CHECK(tracker.update({whole}, stamp(2), next).empty());
    CHECK(tracker.pending().size() == 1 && tracker.pending()[0].kind == EventKind::moved && tracker.pending()[0].windows == 2 &&
          tracker.pending()[0].first_seen == stamp(0));
    CHECK(tracker.update({whole}, stamp(3), next).empty());
    const auto started = tracker.update({whole}, stamp(4), next);
    CHECK(started.size() == 1 && started[0].first == EventState::candidate && started[0].second.kind == EventKind::moved);
}

TEST(activity_reports_the_newest_window_with_motion) {
    const Cloud room = scenes::room_shell();
    MemoryConfig config = mid360_config(std::nullopt);
    config.candidate_windows = 5;
    config.activity_radius_m = 1.0;
    RoomMemory memory(config, kStart);
    const msg::Activity idle = memory.activity(stamp(0));
    CHECK(!idle.active && !idle.since_s && idle.header.seq == 0);
    const std::int64_t second = build_reference(memory, room);
    for (int n = 0; n < 4; ++n) {
        const Cloud box = scenes::box_shell({0.3 + 0.3 * (n % 2), 1.2, 0.3});
        memory.add_points("mid360", scenes::concat({&room, &box}));
        CHECK(memory.close_window(stamp(second + n)).empty());
    }
    const msg::Activity moving = memory.activity(stamp(second + 4));
    CHECK(moving.active && moving.sensors == msg::kSensorMid360 && moving.largest_sensor == msg::kSensorMid360);
    CHECK(moving.n_regions >= 1 && moving.voxel_count > 0 && moving.since_s == 1.0f && moving.header.seq == 1);
    CHECK(msg::Activity::from_cdr(moving.to_cdr()) == moving);
    for (int n = 4; n < 6; ++n) {
        memory.add_points("mid360", room);
        memory.close_window(stamp(second + n));
    }
    const msg::Activity later = memory.activity(stamp(second + 9));
    CHECK(!later.active && later.n_regions == 0 && !later.centroid && later.since_s == 6.0f);
}

TEST(gate_config_rejections) {
    for (const auto& edit : std::initializer_list<void (*)(MemoryConfig&)>{
             [](MemoryConfig& c) { c.fresh_min_voxels = 0; }, [](MemoryConfig& c) { c.fresh_min_fraction = 1.5; },
             [](MemoryConfig& c) { c.stable_min_fraction = -0.1; }, [](MemoryConfig& c) { c.stable_min_fraction = NAN; },
             [](MemoryConfig& c) { c.activity_radius_m = -1.0; }, [](MemoryConfig& c) { c.activity_radius_m = INFINITY; }}) {
        MemoryConfig config;
        config.state_dir = std::nullopt;
        edit(config);
        CHECK_THROWS(std::invalid_argument, RoomMemory(config, kStart));
    }
}

TEST_MAIN
