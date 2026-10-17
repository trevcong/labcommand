// Operator actions (protocol 3.6, D46): confirm, dismiss and its suppression, accept and its held
// ACCEPTED, the confirmed lifecycle, the durable writes and their faults, and every refusal. The
// Python twin is spark/room_memory/tests/test_operator.py; the differential suite compares both.
#include <sys/stat.h>
#include <unistd.h>

#include <algorithm>
#include <cstdio>
#include <fstream>

#include "harness.hpp"
#include "room_memory/memory.hpp"
#include "room_memory/operator.hpp"
#include "scenes.hpp"

using namespace room_memory;
using msg::EventKind;
using msg::EventState;
using msg::OperatorActionKind;
using Outcome = msg::OperatorOutcome;
using scenes::Cloud;

namespace {

const msg::Time kStart{1760400000, 0};
constexpr std::uint64_t kGeneration = 1760400000000ULL;

msg::Time stamp(std::int64_t second) { return msg::Time{kStart.sec + second, 0}; }

struct TempDir {
    std::filesystem::path path;
    TempDir() {
        std::string pattern = (std::filesystem::temp_directory_path() / "room-memory-operator-XXXXXX").string();
        if (!mkdtemp(pattern.data())) throw harness::Failure("mkdtemp failed");
        path = pattern;
    }
    ~TempDir() {
        std::filesystem::permissions(path / "lab", std::filesystem::perms::owner_all, std::filesystem::perm_options::add,
                                     ignored);
        std::filesystem::remove_all(path, ignored);
    }
    std::error_code ignored;
};

// No injected fault survives a test.
struct NoFaults {
    ~NoFaults() { set_write_fault({}); }
};

// Single-window mechanics (every voxel a window sees settles, one supported voxel is fresh, no
// activity gate, a CANDIDATE at once), as in test_memory.cpp.
MemoryConfig config(const std::optional<std::string>& state_dir, std::vector<std::string> sensors = {"mid360"}) {
    MemoryConfig c;
    c.sensors = std::move(sensors);
    c.max_range = std::nullopt;
    c.reference_windows = 3;
    c.recent_windows = 1;
    c.occupied_min_hits = 1;
    c.occupied_min_ratio = 0.0;
    c.free_min_seen = 1;
    c.candidate_windows = 1;
    c.expire_windows = 1;
    c.fresh_min_voxels = 1;
    c.fresh_min_fraction = 0.0;
    c.activity_radius_m = 0.0;
    c.suppress_clear_windows = 3;
    c.state_dir = state_dir;
    return c;
}

const Cloud& room() {
    static const Cloud cloud = scenes::room_shell();
    return cloud;
}

const Cloud& box() {
    static const Cloud cloud = scenes::box_shell({1.0, 0.5, 0.3});
    return cloud;
}

Cloud with(const Cloud& extra) { return scenes::concat({&room(), &extra}); }

// The room's points behind the sensor only: no ray reaches the box.
Cloud behind() {
    Cloud out;
    const Cloud& all = room();
    for (std::size_t n = 0; n < all.size(); n += 3) {
        if (all[n] < -0.5f) out.insert(out.end(), all.begin() + static_cast<std::ptrdiff_t>(n), all.begin() + static_cast<std::ptrdiff_t>(n) + 3);
    }
    return out;
}

std::vector<msg::ChangeEvent> window(RoomMemory& memory, const Cloud& cloud, std::int64_t second,
                                     const std::string& sensor = "mid360") {
    memory.add_points(sensor, cloud);
    return memory.close_window(stamp(second));
}

// The reference from the room at seconds 0..2, then the first map snapshot at 3 (epoch 1).
void track(RoomMemory& memory, const Cloud& reference) {
    for (int second = 0; second < 3; ++second) {
        for (const std::string& sensor : memory.config().sensors) memory.add_points(sensor, reference);
        CHECK(memory.close_window(stamp(second)).empty());
    }
    CHECK(memory.state() == msg::MemoryState::tracking);
    CHECK(memory.map_update(stamp(3)).has_value());
}

std::string uuid(int n) {
    char text[37];
    std::snprintf(text, sizeof text, "00000000-0000-4000-8000-%012d", n);
    return text;
}

msg::OperatorAction act(OperatorActionKind kind, std::uint64_t event, EventState expected, std::uint64_t epoch, int id,
                        std::uint64_t generation = kGeneration) {
    msg::OperatorAction a;
    a.header = msg::Header{"lab", epoch, static_cast<std::uint64_t>(id), stamp(100 + id)};
    a.action = kind;
    a.event_id = event;
    a.expected_state = expected;
    a.expected_generation = generation;
    a.operator_id = "trevor";
    a.action_id = uuid(id);
    return a;
}

std::vector<std::pair<EventKind, EventState>> transitions(const std::vector<msg::ChangeEvent>& events) {
    std::vector<std::pair<EventKind, EventState>> out;
    for (const auto& e : events) out.emplace_back(e.kind, e.state);
    return out;
}

std::string read(const std::filesystem::path& path) {
    std::ifstream in(path, std::ios::binary);
    return std::string((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
}

// A tracking memory with one open candidate of the box (appeared) at second 4.
std::uint64_t candidate(RoomMemory& memory) {
    track(memory, room());
    const auto events = window(memory, with(box()), 4);
    CHECK((transitions(events) == std::vector<std::pair<EventKind, EventState>>{{EventKind::appeared, EventState::candidate}}));
    return events[0].event_id;
}

// What changes when an action is refused: none of these.
struct Snapshot {
    std::uint64_t event_seq;
    std::int64_t open;
    std::size_t suppressed;
    Keys reference;
    std::string files;
};

Snapshot snapshot(RoomMemory& memory, const std::optional<std::filesystem::path>& dir) {
    std::string files;
    if (dir && std::filesystem::exists(*dir / "lab")) {
        for (const auto& entry : std::filesystem::directory_iterator(*dir / "lab")) {
            files += entry.path().filename().string() + "=" + read(entry.path()) + ";";
        }
    }
    return Snapshot{memory.event_seq(), memory.status(stamp(50), 1.0).n_open_events, memory.suppressions("mid360").size(),
                    memory.channel("mid360").reference.occupied, files};
}

bool same(const Snapshot& a, const Snapshot& b) {
    return a.event_seq == b.event_seq && a.open == b.open && a.suppressed == b.suppressed && a.reference == b.reference &&
           a.files == b.files;
}

}  // namespace

// MARK: Confirm and the confirmed lifecycle

TEST(confirm_sends_confirmed_and_the_event_stays_open) {
    RoomMemory memory(config(std::nullopt), kStart, kGeneration);
    const std::uint64_t id = candidate(memory);
    const std::uint64_t seq = memory.event_seq();
    const auto result = memory.operate(act(OperatorActionKind::confirm, id, EventState::candidate, 1, 1), stamp(5));
    CHECK(result.outcome == Outcome::applied && result.events.size() == 1);
    const msg::ChangeEvent& confirmed = result.events[0];
    CHECK(confirmed.state == EventState::confirmed && confirmed.event_id == id && confirmed.header.seq == seq);
    CHECK(msg::ChangeEvent::from_cdr(confirmed.to_cdr()) == confirmed);
    CHECK_EQ(memory.status(stamp(5), 1.0).n_open_events, std::int64_t{1});
    // A confirmed event is matched as before and sends nothing.
    CHECK(window(memory, with(box()), 6).empty());
}

TEST(a_confirmed_event_goes_reverted_only_when_seen_back_at_the_reference) {
    RoomMemory memory(config(std::nullopt), kStart, kGeneration);
    const std::uint64_t id = candidate(memory);
    CHECK(memory.operate(act(OperatorActionKind::confirm, id, EventState::candidate, 1, 1), stamp(5)).outcome == Outcome::applied);
    // Occluded: unmatched, but no ray reaches its voxels. It neither expires nor reverts.
    for (int second = 6; second < 10; ++second) CHECK(window(memory, behind(), second).empty());
    CHECK(memory.channel("mid360").tracker.open().size() == 1 && memory.channel("mid360").tracker.open()[0].missed == 4);
    // Seen: the rays pass through where the box was.
    const auto back = window(memory, room(), 10);
    CHECK((transitions(back) == std::vector<std::pair<EventKind, EventState>>{{EventKind::appeared, EventState::reverted}}));
    CHECK(back[0].event_id == id && memory.status(stamp(10), 1.0).n_open_events == 0);
}

TEST(a_partly_undone_move_stays_confirmed) {
    const Cloud a = scenes::box_shell({0.0, 0.5, 0.30});
    const Cloud b = scenes::shifted(a, 1);
    RoomMemory memory(config(std::nullopt), kStart, kGeneration);
    track(memory, with(a));
    const auto moved = window(memory, with(b), 4);
    CHECK((transitions(moved) == std::vector<std::pair<EventKind, EventState>>{{EventKind::moved, EventState::candidate}}));
    CHECK(memory.operate(act(OperatorActionKind::confirm, moved[0].event_id, EventState::candidate, 1, 1), stamp(5)).outcome ==
          Outcome::applied);
    // Removed from its new pose but not put back: the new pose is free, the old one still empty.
    for (int second = 6; second < 10; ++second) CHECK(window(memory, room(), second).empty());
    CHECK(memory.channel("mid360").tracker.open().size() == 1);
    // Put back where it was: both poses agree with the reference.
    const auto back = window(memory, with(a), 10);
    CHECK((transitions(back) == std::vector<std::pair<EventKind, EventState>>{{EventKind::moved, EventState::reverted}}));
}

TEST(an_unconfirmed_candidate_still_expires) {
    RoomMemory memory(config(std::nullopt), kStart, kGeneration);
    candidate(memory);
    const auto gone = window(memory, behind(), 5);
    CHECK((transitions(gone) == std::vector<std::pair<EventKind, EventState>>{{EventKind::appeared, EventState::expired}}));
}

// MARK: Dismiss and suppression

TEST(dismiss_reverts_and_suppresses_the_same_change) {
    TempDir dir;
    RoomMemory memory(config(dir.path.string()), kStart, kGeneration);
    const std::uint64_t id = candidate(memory);
    const auto result = memory.operate(act(OperatorActionKind::dismiss, id, EventState::candidate, 1, 1), stamp(5));
    CHECK(result.outcome == Outcome::applied);
    CHECK((transitions(result.events) == std::vector<std::pair<EventKind, EventState>>{{EventKind::appeared, EventState::reverted}}));
    CHECK(memory.suppressions("mid360").size() == 1 && memory.status(stamp(5), 1.0).n_open_events == 0);
    CHECK(memory.activity(stamp(5)).active);  // the candidate's own window
    // The same box: no candidate and no activity, window after window.
    for (int second = 6; second < 66; ++second) {
        CHECK(window(memory, with(box()), second).empty());
        CHECK(!memory.activity(stamp(second)).active);
    }
    // The file pins the suppression's bytes.
    const Suppression& s = memory.suppressions("mid360")[0];
    std::string expected = "room_memory suppressed 1\n" + std::to_string(id) + " appeared " + std::to_string(s.keys.size());
    for (const Key key : s.keys) expected += " " + std::to_string(key);
    expected += " 0\n";
    CHECK(read(suppressions_path(dir.path.string(), "lab", "mid360")) == expected);
    CHECK(parse_suppressions(expected).size() == 1 && parse_suppressions(expected)[0].keys == s.keys);
    // It survives a restart.
    RoomMemory restarted(config(dir.path.string()), stamp(70), kGeneration + 1);
    CHECK(restarted.suppressions("mid360").size() == 1);
    CHECK(window(restarted, with(box()), 71).empty());
}

TEST(a_different_change_at_a_dismissed_place_still_alerts) {
    RoomMemory memory(config(std::nullopt), kStart, kGeneration);
    const std::uint64_t id = candidate(memory);
    CHECK(memory.operate(act(OperatorActionKind::dismiss, id, EventState::candidate, 1, 1), stamp(5)).outcome == Outcome::applied);
    // A second box against the first: one bigger region, another shape.
    const Cloud next = scenes::box_shell({1.0, 0.9, 0.3});
    const Cloud both = scenes::concat({&room(), &box(), &next});
    const auto events = window(memory, both, 6);
    CHECK((transitions(events) == std::vector<std::pair<EventKind, EventState>>{{EventKind::appeared, EventState::candidate}}));
    // It keeps the suppression alive: the dismissed box was there.
    CHECK(memory.suppressions("mid360").size() == 1 && memory.suppressions("mid360")[0].cleared == 0);
}

TEST(a_suppression_lifts_only_after_the_place_is_seen_clear) {
    RoomMemory memory(config(std::nullopt), kStart, kGeneration);
    const std::uint64_t id = candidate(memory);
    CHECK(memory.operate(act(OperatorActionKind::dismiss, id, EventState::candidate, 1, 1), stamp(5)).outcome == Outcome::applied);
    // Points that never reach the place do not count.
    for (int second = 6; second < 16; ++second) CHECK(window(memory, behind(), second).empty());
    CHECK(memory.suppressions("mid360").size() == 1 && memory.suppressions("mid360")[0].cleared == 0);
    // Seen clear twice, then the box again: the count restarts.
    CHECK(window(memory, room(), 16).empty() && window(memory, room(), 17).empty());
    CHECK(memory.suppressions("mid360")[0].cleared == 2);
    CHECK(window(memory, with(box()), 18).empty());
    CHECK(memory.suppressions("mid360")[0].cleared == 0);
    // Three clear windows lift it; the box put back alerts again.
    for (int second = 19; second < 22; ++second) CHECK(window(memory, room(), second).empty());
    CHECK(memory.suppressions("mid360").empty());
    const auto again = window(memory, with(box()), 22);
    CHECK((transitions(again) == std::vector<std::pair<EventKind, EventState>>{{EventKind::appeared, EventState::candidate}}));
    CHECK(again[0].event_id != id);
}

// A dismissed move whose diff comes back as its two halves (the pairing gate flickers) stays
// silent: each half keeps the shape of its pose.
TEST(a_dismissed_move_also_screens_its_halves) {
    TempDir dir;
    const Cloud a = scenes::box_shell({0.0, 0.5, 0.30});
    const Cloud far = scenes::box_shell({0.0, -1.0, 0.30});
    // Two runs without suppression give the halves' voxels: the box far from its place is an
    // appearance and a disappearance, not a move.
    std::vector<msg::ChangeEvent> halves;
    {
        RoomMemory probe(config(std::nullopt), kStart, kGeneration);
        track(probe, with(a));
        halves = window(probe, with(far), 4);
    }
    CHECK(halves.size() == 2);
    Suppression move;
    move.event_id = 7;
    move.kind = EventKind::moved;
    for (const msg::ChangeEvent& half : halves) {
        const Keys keys = VoxelLayer::from_bricks(half.mask->bricks).occupied;
        (half.kind == EventKind::appeared ? move.keys : move.from_keys) = keys;
    }
    CHECK(!move.keys.empty() && !move.from_keys.empty());
    {
        RoomMemory building(config(dir.path.string()), kStart, kGeneration);
        track(building, with(a));
    }
    const std::string text = format_suppressions({move});
    const msg::Bytes bytes(text.begin(), text.end());
    CHECK(write_staged(suppressions_path(dir.path.string(), "lab", "mid360"), bytes));
    RoomMemory memory(config(dir.path.string()), stamp(10), kGeneration + 1);
    CHECK(memory.suppressions("mid360").size() == 1);
    for (int second = 11; second < 20; ++second) CHECK(window(memory, with(far), second).empty());
}

TEST(a_lift_whose_write_fails_is_retried_every_window) {
    TempDir dir;
    NoFaults guard;
    RoomMemory memory(config(dir.path.string()), kStart, kGeneration);
    const std::uint64_t id = candidate(memory);
    CHECK(memory.operate(act(OperatorActionKind::dismiss, id, EventState::candidate, 1, 1), stamp(5)).outcome == Outcome::applied);
    const std::filesystem::path file = suppressions_path(dir.path.string(), "lab", "mid360");
    const std::string dismissed = read(file);
    set_write_fault([](WriteStage stage) { return stage == WriteStage::rename; });
    for (int second = 6; second < 9; ++second) CHECK(window(memory, room(), second).empty());
    // Lifted in memory, not on disk.
    CHECK(memory.suppressions("mid360").empty() && read(file) == dismissed && memory.storage_error());
    // The object comes back: it alerts in this process.
    const auto back = window(memory, with(box()), 9);
    CHECK((transitions(back) == std::vector<std::pair<EventKind, EventState>>{{EventKind::appeared, EventState::candidate}}));
    // The disk heals: the next window writes the lift.
    set_write_fault({});
    window(memory, with(box()), 10);
    CHECK(read(file) == "room_memory suppressed 1\n" && !memory.storage_error());
    RoomMemory restarted(config(dir.path.string()), stamp(20), kGeneration + 1);
    CHECK(restarted.suppressions("mid360").empty());
    const auto fresh = window(restarted, with(box()), 21);
    CHECK((transitions(fresh) == std::vector<std::pair<EventKind, EventState>>{{EventKind::appeared, EventState::candidate}}));
}

TEST(suppressions_file_is_strict_and_a_new_reference_drops_it) {
    CHECK(parse_suppressions("room_memory suppressed 1\n").empty());
    const std::vector<std::string> bad = {
        "",
        "room_memory suppressed 2\n",
        "room_memory suppressed 1",
        "room_memory suppressed 1\n5 appeared 1 10 0",
        "room_memory suppressed 1\n5 appeared 0 0\n",
        "room_memory suppressed 1\n5 appeared 2 10 9 0\n",
        "room_memory suppressed 1\n5 appeared 1 10 1 11\n",
        "room_memory suppressed 1\n5 moved 1 10 0\n",
        "room_memory suppressed 1\n5 sideways 1 10 0\n",
        "room_memory suppressed 1\n5 appeared 1 10 0 7\n",
        "room_memory suppressed 1\n5  appeared 1 10 0\n",
        "room_memory suppressed 1\n05 appeared 1 10 0\n",
        "room_memory suppressed 1\n5 appeared 1 10 0\n5 appeared 1 11 0\n",
        "room_memory suppressed 1\n5 appeared 9 10 0\n",
    };
    for (const std::string& text : bad) CHECK_THROWS(std::invalid_argument, parse_suppressions(text));
    Suppression s;
    s.event_id = 5;
    s.kind = EventKind::moved;
    s.keys = {-3, 10};
    s.from_keys = {20};
    CHECK(format_suppressions({s}) == "room_memory suppressed 1\n5 moved 2 -3 10 1 20\n");
    CHECK(parse_suppressions(format_suppressions({s}))[0].from_keys == s.from_keys);
    // A malformed file refuses the start, like a malformed map_epoch.
    TempDir dir;
    {
        RoomMemory memory(config(dir.path.string()), kStart, kGeneration);
        track(memory, room());
    }
    const std::filesystem::path file = suppressions_path(dir.path.string(), "lab", "mid360");
    {
        std::ofstream out(file);
        out << "room_memory suppressed 1\nnonsense\n";
    }
    CHECK_THROWS(std::invalid_argument, RoomMemory(config(dir.path.string()), kStart, kGeneration));
    // Without references (BUILDING) a stale file is removed: a new reference has no dismissals.
    std::filesystem::remove(reference_path(dir.path.string(), "lab", "mid360"));
    RoomMemory building(config(dir.path.string()), kStart, kGeneration);
    CHECK(building.state() == msg::MemoryState::building && !std::filesystem::exists(file));
}

// MARK: Accept

TEST(accept_absorbs_the_change_and_holds_accepted_for_its_map) {
    TempDir dir;
    RoomMemory memory(config(dir.path.string()), kStart, kGeneration);
    const std::uint64_t id = candidate(memory);
    const auto confirmed = memory.operate(act(OperatorActionKind::confirm, id, EventState::candidate, 1, 1), stamp(5));
    const Keys published = VoxelLayer::from_bricks(confirmed.events[0].mask->bricks).occupied;
    // The scene's pose residual is within the accept gate.
    CHECK(memory.channel("mid360").residual && *memory.channel("mid360").residual <= memory.config().learn_max_offset_m);
    const auto accepted = memory.operate(act(OperatorActionKind::accept, id, EventState::confirmed, 1, 2), stamp(5));
    CHECK(accepted.outcome == Outcome::applied && accepted.events.empty());
    // On disk at once; still open (and CONFIRMED on the wire) until its map message.
    const auto [message, layer] = load_reference(reference_path(dir.path.string(), "lab", "mid360"));
    CHECK(count_present(layer.occupied, published) == published.size());
    CHECK_EQ(memory.status(stamp(5), 1.0).n_open_events, std::int64_t{1});
    CHECK(memory.accepted_after_map(stamp(5)).empty());
    memory.activity(stamp(5));
    // The absorbed reference: the box is no change any more.
    CHECK(window(memory, with(box()), 6).empty() && !memory.activity(stamp(6)).active);
    CHECK(!memory.map_update(stamp(6)));
    const auto map = memory.map_update(stamp(8));
    CHECK(map.has_value());
    const msg::MapBricks delta = msg::MapBricks::from_cdr(*map);
    CHECK(delta.kind == msg::MapKind::delta && delta.header.map_epoch == 2 && delta.base_epoch == 1 && !delta.bricks.empty());
    const auto released = memory.accepted_after_map(stamp(8));
    CHECK(released.size() == 1 && released[0].state == EventState::accepted && released[0].event_id == id &&
          released[0].header.map_epoch == 2);
    CHECK(VoxelLayer::from_bricks(released[0].mask->bricks).occupied == published);
    CHECK(memory.accepted_after_map(stamp(8)).empty() && memory.status(stamp(8), 1.0).n_open_events == 0);
    // After a restart the region still matches the reference.
    RoomMemory restarted(config(dir.path.string()), stamp(20), kGeneration + 1);
    CHECK(window(restarted, with(box()), 21).empty());
}

TEST(accept_absorbs_what_was_published_not_the_current_region) {
    RoomMemory memory(config(std::nullopt), kStart, kGeneration);
    const std::uint64_t id = candidate(memory);
    const auto confirmed = memory.operate(act(OperatorActionKind::confirm, id, EventState::candidate, 1, 1), stamp(5));
    const Keys published = region_keys(memory.channel("mid360").tracker.open()[0].published);
    CHECK(published == VoxelLayer::from_bricks(confirmed.events[0].mask->bricks).occupied);
    CHECK(memory.operate(act(OperatorActionKind::accept, id, EventState::confirmed, 1, 2), stamp(5)).outcome == Outcome::applied);
    const Keys& reference = memory.channel("mid360").reference.occupied;
    CHECK(count_present(reference, published) == published.size());
}

TEST(an_accept_on_a_keyframe_tick_goes_out_in_the_snapshot) {
    MemoryConfig c = config(std::nullopt);
    c.keyframe_period_s = 6.0;
    RoomMemory memory(c, kStart, kGeneration);
    const std::uint64_t id = candidate(memory);
    memory.operate(act(OperatorActionKind::confirm, id, EventState::candidate, 1, 1), stamp(5));
    CHECK(memory.operate(act(OperatorActionKind::accept, id, EventState::confirmed, 1, 2), stamp(5)).outcome == Outcome::applied);
    const auto map = memory.map_update(stamp(9));
    const msg::MapBricks snapshot = msg::MapBricks::from_cdr(*map);
    CHECK(snapshot.kind == msg::MapKind::snapshot && snapshot.header.map_epoch == 2);
    const auto released = memory.accepted_after_map(stamp(9));
    CHECK(released.size() == 1 && released[0].header.map_epoch == 2);
}

TEST(an_accept_of_the_other_sensor_sends_an_empty_delta) {
    RoomMemory memory(config(std::nullopt, {"mid360", "jt128"}), kStart, kGeneration);
    track(memory, room());
    memory.add_points("jt128", room());
    const auto events = window(memory, with(box()), 4);
    CHECK(events.size() == 1 && events[0].sensors == msg::kSensorMid360);
    const std::uint64_t id = events[0].event_id;
    memory.operate(act(OperatorActionKind::confirm, id, EventState::candidate, 1, 1), stamp(5));
    CHECK(memory.operate(act(OperatorActionKind::accept, id, EventState::confirmed, 1, 2), stamp(5)).outcome == Outcome::applied);
    const msg::MapBricks delta = msg::MapBricks::from_cdr(*memory.map_update(stamp(8)));
    CHECK(delta.kind == msg::MapKind::delta && delta.header.map_epoch == 2 && delta.bricks.empty() && delta.removed.empty());
    CHECK(memory.accepted_after_map(stamp(8)).size() == 1);
}

TEST(a_map_save_that_throws_keeps_the_accept_held_until_a_map_carries_it) {
    TempDir dir;
    RoomMemory memory(config(dir.path.string()), kStart, kGeneration);
    const std::uint64_t id = candidate(memory);
    memory.operate(act(OperatorActionKind::confirm, id, EventState::candidate, 1, 1), stamp(5));
    CHECK(memory.operate(act(OperatorActionKind::accept, id, EventState::confirmed, 1, 2), stamp(5)).outcome == Outcome::applied);
    const std::filesystem::path room_path = dir.path / "lab";
    std::filesystem::permissions(room_path, std::filesystem::perms::owner_write, std::filesystem::perm_options::remove);
    bool threw = false;
    try {
        memory.map_update(stamp(8));
    } catch (const std::exception&) {
        threw = true;
    }
    std::filesystem::permissions(room_path, std::filesystem::perms::owner_write, std::filesystem::perm_options::add);
    if (getuid() == 0) throw harness::Skip("root ignores directory permissions");
    CHECK(threw && memory.accepted_after_map(stamp(8)).empty() && memory.status(stamp(8), 1.0).n_open_events == 1);
    const auto map = memory.map_update(stamp(9));
    CHECK(map.has_value());
    const msg::MapBricks message = msg::MapBricks::from_cdr(*map);
    const auto released = memory.accepted_after_map(stamp(9));
    CHECK(released.size() == 1 && released[0].header.map_epoch == message.header.map_epoch);
    CHECK(memory.accepted_after_map(stamp(10)).empty());
}

// MARK: Storage faults

TEST(a_write_that_fails_before_the_rename_changes_nothing) {
    for (const WriteStage stage : {WriteStage::write, WriteStage::sync, WriteStage::rename}) {
        TempDir dir;
        NoFaults guard;
        RoomMemory memory(config(dir.path.string()), kStart, kGeneration);
        const std::uint64_t id = candidate(memory);
        memory.operate(act(OperatorActionKind::confirm, id, EventState::candidate, 1, 1), stamp(5));
        const Snapshot before = snapshot(memory, dir.path);
        set_write_fault([stage](WriteStage at) { return at == stage; });
        for (const auto& [kind, n] : {std::pair{OperatorActionKind::accept, 2}, std::pair{OperatorActionKind::dismiss, 3}}) {
            const auto result = memory.operate(act(kind, id, EventState::confirmed, 1, n), stamp(5));
            CHECK(result.outcome == Outcome::storage_failed && result.events.empty());
            CHECK(same(snapshot(memory, dir.path), before) && memory.storage_error());
            CHECK(!std::filesystem::exists(reference_path(dir.path.string(), "lab", "mid360").string() + ".tmp"));
        }
        // Not remembered: the same action_id may act once the disk heals.
        set_write_fault({});
        CHECK(memory.operate(act(OperatorActionKind::dismiss, id, EventState::confirmed, 1, 3), stamp(5)).outcome == Outcome::applied);
    }
}

TEST(a_write_installed_without_its_directory_sync_is_applied) {
    TempDir dir;
    NoFaults guard;
    RoomMemory memory(config(dir.path.string()), kStart, kGeneration);
    const std::uint64_t id = candidate(memory);
    memory.operate(act(OperatorActionKind::confirm, id, EventState::candidate, 1, 1), stamp(5));
    set_write_fault([](WriteStage at) { return at == WriteStage::directory; });
    const auto result = memory.operate(act(OperatorActionKind::accept, id, EventState::confirmed, 1, 2), stamp(5));
    CHECK(result.outcome == Outcome::applied && memory.storage_error());
    set_write_fault({});
    // What a restart reads is what memory holds.
    const auto [message, layer] = load_reference(reference_path(dir.path.string(), "lab", "mid360"));
    CHECK(layer.occupied == memory.channel("mid360").reference.occupied);
}

// MARK: Refusals

TEST(every_refusal_changes_nothing) {
    TempDir dir;
    RoomMemory memory(config(dir.path.string()), kStart, kGeneration);
    const std::uint64_t id = candidate(memory);
    const Snapshot before = snapshot(memory, dir.path);
    int n = 10;
    const auto refused = [&](msg::OperatorAction action, Outcome outcome) {
        const auto result = memory.operate(action, stamp(5));
        if (result.outcome != outcome) {
            throw harness::Failure("action " + std::string(msg::name(action.action)) + ": " + msg::name(result.outcome) +
                                   ", expected " + msg::name(outcome));
        }
        CHECK(result.events.empty() && same(snapshot(memory, dir.path), before));
    };
    msg::OperatorAction other_room = act(OperatorActionKind::confirm, id, EventState::candidate, 1, ++n);
    other_room.header.room_id = "lab-b";
    refused(other_room, Outcome::wrong_room);
    refused(act(OperatorActionKind::confirm, id, EventState::candidate, 1, ++n, kGeneration + 1), Outcome::stale_generation);
    refused(act(OperatorActionKind::confirm, id + 1, EventState::candidate, 1, ++n), Outcome::unknown_event);
    refused(act(OperatorActionKind::dismiss, id, EventState::confirmed, 1, ++n), Outcome::state_mismatch);
    refused(act(OperatorActionKind::accept, id, EventState::confirmed, 1, ++n), Outcome::state_mismatch);
    msg::OperatorAction pause = act(OperatorActionKind::pause, id, EventState::candidate, 1, ++n);
    pause.event_id.reset();
    pause.expected_state.reset();
    refused(pause, Outcome::unsupported);
    // Confirmed, then the accept-only gates.
    CHECK(memory.operate(act(OperatorActionKind::confirm, id, EventState::candidate, 1, ++n), stamp(5)).outcome == Outcome::applied);
    const Snapshot confirmed = snapshot(memory, dir.path);
    const auto refused_confirmed = [&](msg::OperatorAction action, Outcome outcome) {
        const auto result = memory.operate(action, stamp(5));
        CHECK(result.outcome == outcome && result.events.empty() && same(snapshot(memory, dir.path), confirmed));
    };
    refused_confirmed(act(OperatorActionKind::confirm, id, EventState::candidate, 1, ++n), Outcome::state_mismatch);
    refused_confirmed(act(OperatorActionKind::accept, id, EventState::confirmed, 0, ++n), Outcome::stale_epoch);
    // A window without points: what the operator saw is not supported now.
    CHECK(memory.close_window(stamp(6)).empty());
    refused_confirmed(act(OperatorActionKind::accept, id, EventState::confirmed, 1, ++n), Outcome::target_changed);
    refused_confirmed(act(OperatorActionKind::acknowledge, id + 1, EventState::confirmed, 1, ++n), Outcome::unknown_event);
    CHECK(memory.operate(act(OperatorActionKind::acknowledge, id, EventState::candidate, 1, ++n), stamp(5)).outcome ==
          Outcome::applied);
}

TEST(a_moved_rig_refuses_accept) {
    RoomMemory memory(config(std::nullopt), kStart, kGeneration);
    const std::uint64_t id = candidate(memory);
    memory.operate(act(OperatorActionKind::confirm, id, EventState::candidate, 1, 1), stamp(5));
    // The whole view one voxel over: the box still matches its shape, the pose does not.
    const Cloud all = with(box());
    const Cloud moved = scenes::shifted(all, 0.05);
    window(memory, moved, 6);
    const auto residual = memory.channel("mid360").residual;
    CHECK(residual && *residual > memory.config().learn_max_offset_m);
    CHECK(memory.channel("mid360").tracker.open().size() == 1 && memory.channel("mid360").tracker.open()[0].missed == 0);
    const auto result = memory.operate(act(OperatorActionKind::accept, id, EventState::confirmed, 1, 2), stamp(6));
    CHECK(result.outcome == Outcome::rig_unsettled && result.events.empty());
}

// Review round 2: the recent layer keeps a region matched (missed 0) for recent_windows windows
// after the object left or went out of view. Only the last window's own evidence counts.
TEST(confirm_and_accept_need_the_target_in_the_last_window_itself) {
    const auto three = [](const std::optional<std::string>& dir) {
        MemoryConfig c = config(dir);
        c.recent_windows = 3;
        return c;
    };
    const auto confirmed = [](RoomMemory& memory) {
        const std::uint64_t id = candidate(memory);
        CHECK(memory.operate(act(OperatorActionKind::confirm, id, EventState::candidate, 1, 1), stamp(5)).outcome ==
              Outcome::applied);
        return id;
    };
    {
        TempDir dir;
        RoomMemory still(three(dir.path.string()), kStart, kGeneration);
        const std::uint64_t id = confirmed(still);
        CHECK(window(still, with(box()), 5).empty());
        CHECK(still.operate(act(OperatorActionKind::accept, id, EventState::confirmed, 1, 2), stamp(5)).outcome ==
              Outcome::applied);
    }
    const Cloud hidden = behind();
    for (const Cloud* cloud : {&room(), &hidden}) {
        TempDir dir;
        RoomMemory memory(three(dir.path.string()), kStart, kGeneration);
        const std::uint64_t id = confirmed(memory);
        CHECK(window(memory, *cloud, 5).empty());
        CHECK(memory.channel("mid360").tracker.open()[0].missed == 0);
        const Snapshot before = snapshot(memory, dir.path);
        const auto result = memory.operate(act(OperatorActionKind::accept, id, EventState::confirmed, 1, 2), stamp(5));
        CHECK(result.outcome == Outcome::target_changed && result.events.empty());
        CHECK(same(before, snapshot(memory, dir.path)));
        // Seen again: a new action applies.
        CHECK(window(memory, with(box()), 6).empty());
        CHECK(memory.operate(act(OperatorActionKind::accept, id, EventState::confirmed, 1, 3), stamp(6)).outcome ==
              Outcome::applied);
    }
    RoomMemory gone(three(std::nullopt), kStart, kGeneration);
    const std::uint64_t id = candidate(gone);
    CHECK(window(gone, room(), 5).empty() && gone.channel("mid360").tracker.open()[0].missed == 0);
    CHECK(gone.operate(act(OperatorActionKind::confirm, id, EventState::candidate, 1, 1), stamp(5)).outcome ==
          Outcome::target_changed);
    CHECK(!gone.channel("mid360").tracker.open()[0].confirmed);
}

TEST(a_disappearance_needs_its_voxels_seen_free_in_the_last_window) {
    MemoryConfig c = config(std::nullopt);
    c.recent_windows = 3;
    RoomMemory memory(c, kStart, kGeneration);
    track(memory, with(box()));
    const auto events = window(memory, room(), 4);
    CHECK((transitions(events) == std::vector<std::pair<EventKind, EventState>>{{EventKind::disappeared, EventState::candidate}}));
    const std::uint64_t id = events[0].event_id;
    CHECK(window(memory, behind(), 5).empty() && memory.channel("mid360").tracker.open()[0].missed == 0);
    CHECK(memory.operate(act(OperatorActionKind::confirm, id, EventState::candidate, 1, 1), stamp(5)).outcome ==
          Outcome::target_changed);
    CHECK(window(memory, room(), 6).empty());
    CHECK(memory.operate(act(OperatorActionKind::confirm, id, EventState::candidate, 1, 2), stamp(6)).outcome ==
          Outcome::applied);
}

TEST(a_reshaped_candidate_is_not_what_the_operator_saw) {
    RoomMemory memory(config(std::nullopt), kStart, kGeneration);
    const std::uint64_t id = candidate(memory);
    const Cloud next = scenes::box_shell({1.0, 0.9, 0.3});
    CHECK(window(memory, scenes::concat({&room(), &box(), &next}), 5).empty());
    CHECK(memory.channel("mid360").tracker.open()[0].missed == 0);
    const auto result = memory.operate(act(OperatorActionKind::confirm, id, EventState::candidate, 1, 1), stamp(5));
    CHECK(result.outcome == Outcome::target_changed && result.events.empty());
}

TEST(nothing_is_tracked_while_building) {
    RoomMemory memory(config(std::nullopt), kStart, kGeneration);
    const auto result = memory.operate(act(OperatorActionKind::confirm, 5, EventState::candidate, 0, 1), stamp(1));
    CHECK(result.outcome == Outcome::not_tracking);
}

TEST(a_second_accept_waits_for_the_first_map) {
    RoomMemory memory(config(std::nullopt), kStart, kGeneration);
    const std::uint64_t id = candidate(memory);
    memory.operate(act(OperatorActionKind::confirm, id, EventState::candidate, 1, 1), stamp(5));
    CHECK(memory.operate(act(OperatorActionKind::accept, id, EventState::confirmed, 1, 2), stamp(5)).outcome == Outcome::applied);
    // The held accept is ACCEPTED here: neither dismissed nor confirmed again.
    CHECK(memory.operate(act(OperatorActionKind::dismiss, id, EventState::confirmed, 1, 3), stamp(5)).outcome ==
          Outcome::state_mismatch);
    CHECK(memory.operate(act(OperatorActionKind::accept, id, EventState::confirmed, 1, 4), stamp(5)).outcome ==
          Outcome::state_mismatch);
}

// MARK: Idempotence

TEST(a_repeated_action_id_acts_once) {
    RoomMemory memory(config(std::nullopt), kStart, kGeneration);
    const std::uint64_t id = candidate(memory);
    const msg::OperatorAction confirm = act(OperatorActionKind::confirm, id, EventState::candidate, 1, 1);
    CHECK(memory.operate(confirm, stamp(5)).events.size() == 1);
    const std::uint64_t seq = memory.event_seq();
    const auto again = memory.operate(confirm, stamp(6));
    CHECK(again.outcome == Outcome::applied && again.repeat && !again.repeat_differs && again.events.empty());
    msg::OperatorAction changed = confirm;
    changed.text = "another note";
    const auto differs = memory.operate(changed, stamp(6));
    CHECK(differs.outcome == Outcome::applied && differs.repeat && differs.repeat_differs && memory.event_seq() == seq);
    // A refusal is remembered too.
    const msg::OperatorAction unknown = act(OperatorActionKind::confirm, id + 9, EventState::candidate, 1, 2);
    CHECK(memory.operate(unknown, stamp(6)).outcome == Outcome::unknown_event);
    CHECK(memory.operate(unknown, stamp(6)).repeat);
    // The ring forgets the oldest: a retry is then judged afresh (and compare-and-set refuses it).
    for (int n = 0; n < static_cast<int>(kStoredActions); ++n) {
        memory.operate(act(OperatorActionKind::confirm, id + 100, EventState::candidate, 1, 1000 + n), stamp(7));
    }
    const auto late = memory.operate(confirm, stamp(8));
    CHECK(!late.repeat && late.outcome == Outcome::state_mismatch);
}

TEST_MAIN
