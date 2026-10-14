// Voxel layers, bricks, reference persistence and the window engine, ported from
// spark/room_memory/tests/test_memory.py (including the three Codex regressions of 289afb6).
#include <unistd.h>

#include <cmath>
#include <random>

#include "harness.hpp"
#include "room_memory/memory.hpp"
#include "room_memory/pointcloud2.hpp"
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

MemoryConfig mid360_config(const std::optional<std::string>& state_dir) {
    MemoryConfig config;
    config.sensors = {"mid360"};
    config.max_range = std::nullopt;
    config.reference_windows = 3;
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
    CHECK(!status.jt128_hz && !status.pose_residual_m && !status.coverage_known_frac);
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
    CHECK_EQ(memory.status(stamp(second + 2), 1.0).n_open_events, std::int64_t{0});
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
    CHECK(restarted.state() == MemoryState::tracking && restarted.map_epoch() == 1 && memory.map_epoch() == 1);
    CHECK(restarted.channel("mid360").reference.to_bricks() == bricks);
    CHECK_EQ(restarted.status(stamp(9), 1.0).n_bricks, static_cast<std::int64_t>(bricks.size()));
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
    MemoryConfig config;
    config.state_dir = std::nullopt;
    config.max_range = std::nullopt;
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
}

TEST_MAIN
