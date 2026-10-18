// The name registry: durability, linking, and the frame-change receipt (protocol §3.15, §3.16).
#include <unistd.h>

#include <cstring>
#include <fstream>
#include <string>

#include "harness.hpp"
#include "room_memory/objects.hpp"

using namespace room_memory;

namespace {

constexpr std::uint8_t kMid = static_cast<std::uint8_t>(msg::kSensorMid360);
constexpr std::uint8_t kJt = static_cast<std::uint8_t>(msg::kSensorJT128);
constexpr std::uint64_t kGen = 50;
constexpr std::uint64_t kPublisher = 1760400000000ULL;

int dir_seq = 0;

struct TempDir {
    std::filesystem::path path;
    TempDir() {
        path = std::filesystem::temp_directory_path() /
               ("lc-objects-" + std::to_string(::getpid()) + "-" + std::to_string(++dir_seq));
        std::filesystem::create_directories(path);
    }
    ~TempDir() { std::filesystem::remove_all(path); }
};

struct Disk {
    int writes = 0;
    bool fail = false;
    bool unsynced = false;
    WriteStaged writer() {
        return [this](const std::filesystem::path& path, std::span<const std::uint8_t> bytes) {
            if (fail) throw StorageError("disk");
            ++writes;
            const bool synced = install_registry_file(path, bytes);
            return synced && !unsynced;
        };
    }
};

Keys line(std::int64_t first, std::int64_t last, std::int64_t y = 0, std::int64_t z = 0) {
    Keys keys;
    for (std::int64_t x = first; x <= last; ++x) keys.push_back(pack(x, y, z));
    sort_unique(keys);
    return keys;
}

Keys column(std::int64_t x, std::int64_t n) {
    Keys keys;
    for (std::int64_t y = 0; y < n; ++y) keys.push_back(pack(x, y, 0));
    sort_unique(keys);
    return keys;
}

msg::NameObjectRequest ask(std::uint64_t object, std::uint64_t revision, std::uint64_t generation, std::uint64_t event,
                           std::string name, std::string action, std::string room = "lab") {
    msg::NameObjectRequest request;
    request.header.room_id = std::move(room);
    request.header.map_epoch = 1;
    request.object_id = object;
    request.expected_revision = revision;
    request.event_generation = generation;
    request.event_id = event;
    request.name = std::move(name);
    request.operator_id = "trevor";
    request.action_id = std::move(action);
    return request;
}

msg::Header head() {
    msg::Header header;
    header.room_id = "lab";
    header.map_epoch = 1;
    header.seq = 1;
    return header;
}

Registry open_at(Disk& disk, const std::filesystem::path& dir, std::uint64_t frame, Absorb absorb = {}, Occupied occupied = {},
                 std::uint8_t sensors = kMid, double voxel = 0.05, const std::string& room = "lab") {
    return Registry::open(dir, room, voxel, sensors, frame, disk.writer(), std::move(absorb), std::move(occupied));
}

const Object& only(const Registry& reg) {
    CHECK_EQ(reg.objects().size(), std::size_t{1});
    return reg.objects().front();
}

bool has_receipt(const Registry& reg, std::uint64_t generation, std::uint64_t event) {
    for (const Receipt& receipt : reg.receipts()) {
        if (receipt.event_generation == generation && receipt.event_id == event) return true;
    }
    return false;
}

const ObjectLinkRecord* link_of(const Object& obj, std::uint64_t generation, std::uint64_t event) {
    for (const ObjectLinkRecord& link : obj.links) {
        if (link.event_generation == generation && link.event_id == event) return &link;
    }
    return nullptr;
}

Registry named(Disk& disk, const std::filesystem::path& dir, Keys keys, std::uint64_t generation = kGen,
               std::uint64_t event = 10, std::uint64_t frame = 1) {
    Registry reg = open_at(disk, dir, frame);
    CHECK(reg.add_seed(generation, event, kMid, keys));
    const msg::NameObjectResponse response = reg.name(ask(0, 0, generation, event, "Scope cart", "create"), 1);
    CHECK(response.outcome == msg::NameOutcome::applied);
    CHECK_EQ(response.object_id, std::uint64_t{1});
    CHECK_EQ(response.revision, std::uint64_t{1});
    return reg;
}

std::array<std::uint8_t, 32> parse_hash(const char* hex) {
    std::array<std::uint8_t, 32> out{};
    for (std::size_t n = 0; n < 32; ++n) out[n] = static_cast<std::uint8_t>(std::strtoul(std::string(hex + n * 2, 2).c_str(), nullptr, 16));
    return out;
}

}  // namespace

TEST(sha256_empty_and_abc_and_the_request_digest_ignores_action_id) {
    CHECK(sha256({}) == parse_hash("e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855"));
    const std::string abc = "abc";
    CHECK(sha256(std::span<const std::uint8_t>(reinterpret_cast<const std::uint8_t*>(abc.data()), abc.size())) ==
          parse_hash("ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad"));
    msg::NameObjectRequest request = ask(0, 0, kGen, 10, "Scope cart", "act-1");
    const auto digest = request_digest(request);
    request.action_id = "other-action";
    CHECK(request_digest(request) == digest);
    request.name = "Other";
    CHECK(request_digest(request) != digest);
}

TEST(accept_matches_needs_the_shape_and_seen_now) {
    const Keys keys = line(0, 9);
    CHECK(accept_matches(keys, keys, 0, 0.9, true));
    CHECK(!accept_matches(keys, line(100, 109), 0, 0.9, true));
    CHECK(!accept_matches(keys, keys, 0, 0.9, false));
}

TEST(create_rename_and_attach_follow_the_outcome_table) {
    TempDir dir;
    Disk disk;
    Registry reg = named(disk, dir.path, line(0, 9));
    CHECK(reg.seeds().empty());
    CHECK(only(reg).baseline == msg::Baseline::anchored);
    CHECK_EQ(only(reg).name, std::string("Scope cart"));
    CHECK_EQ(reg.next_id(), std::uint64_t{2});

    const auto same = reg.name(ask(1, 1, 0, 0, "Scope cart", "rename-same"), 2);
    CHECK(same.outcome == msg::NameOutcome::applied);
    CHECK_EQ(same.revision, std::uint64_t{2});
    CHECK_EQ(only(reg).name, std::string("Scope cart"));

    const auto stale = reg.name(ask(1, 1, 0, 0, "Nope", "rename-stale"), 3);
    CHECK(stale.outcome == msg::NameOutcome::stale_revision);
    CHECK_EQ(stale.object_id, std::uint64_t{1});
    CHECK_EQ(stale.revision, std::uint64_t{2});
    CHECK_EQ(only(reg).revision, std::uint64_t{2});

    const auto missing = reg.name(ask(9, 1, 0, 0, "Nope", "rename-missing"), 4);
    CHECK(missing.outcome == msg::NameOutcome::unknown_object);
    CHECK_EQ(missing.object_id, std::uint64_t{9});
    CHECK_EQ(missing.revision, std::uint64_t{0});

    const auto wrong = reg.name(ask(1, 2, 0, 0, "Nope", "rename-room", "other"), 5);
    CHECK(wrong.outcome == msg::NameOutcome::wrong_room);
    CHECK_EQ(wrong.object_id, std::uint64_t{0});

    const auto bad = reg.name(ask(1, 2, kGen, 0, "Nope", "bad-shape"), 6);
    CHECK(bad.outcome == msg::NameOutcome::invalid);
    CHECK(reg.name(ask(1, 2, kGen, 0, "Nope", "bad-shape"), 7).outcome == msg::NameOutcome::invalid);
    CHECK(!reg.name(ask(1, 2, kGen, 0, "Different", "bad-shape"), 8).repeat_differs == false);
    CHECK(reg.name(ask(1, 2, kGen, 0, "Different", "bad-shape"), 8).repeat_differs);

    const std::uint64_t before = reg.snapshot_seq();
    CHECK(reg.name(ask(1, 2, 0, 0, "\n", "control"), 9).outcome == msg::NameOutcome::invalid);
    CHECK_EQ(reg.snapshot_seq(), before);
    const auto named_again = reg.name(ask(1, 2, 0, 0, "Scope cart", "applied-again"), 10);
    CHECK(named_again.outcome == msg::NameOutcome::applied);
    CHECK(reg.snapshot_seq() == before + 1);

    msg::ObjectRegistry message = reg.to_message(head(), kPublisher);
    bool saw_applied = false;
    bool saw_rejected = false;
    for (const std::string& action : message.recent_action_ids) {
        if (action == "applied-again" || action == "create" || action == "rename-same") saw_applied = true;
        if (action == "bad-shape" || action == "control") saw_rejected = true;
    }
    CHECK(saw_applied);
    CHECK(!saw_rejected);
    CHECK(msg::ObjectRegistry::from_cdr(message.to_cdr()) == message);

    const auto wide = reg.name(ask(1, only(reg).revision, 0, 0, "\u0100", "utf8"), 11);
    CHECK(wide.outcome == msg::NameOutcome::applied);
    CHECK_EQ(only(reg).name, std::string("\u0100"));
}

TEST(create_and_attach_reject_a_bad_seed_and_keep_it) {
    TempDir dir;
    Disk disk;
    Registry reg = open_at(disk, dir.path, 1);
    CHECK(reg.add_seed(kGen, 10, kMid, line(0, 9)));
    reg.set_occupied([](Key) { return false; });
    const auto low = reg.name(ask(0, 0, kGen, 10, "Scope cart", "low"), 1);
    CHECK(low.outcome == msg::NameOutcome::state_mismatch);
    CHECK_EQ(low.object_id, std::uint64_t{0});
    CHECK_EQ(reg.seeds().size(), std::size_t{1});
    CHECK(reg.objects().empty());

    reg.set_occupied({});
    CHECK(reg.name(ask(0, 0, kGen, 10, "Scope cart", "ok"), 2).outcome == msg::NameOutcome::applied);
    CHECK(reg.seeds().empty());
    CHECK(reg.name(ask(0, 0, kGen, 10, "Scope cart", "consumed"), 3).outcome == msg::NameOutcome::unknown_event);

    CHECK(reg.add_seed(kGen, 11, kMid, line(0, 4)));
    const auto overlap = reg.name(ask(0, 0, kGen, 11, "Other", "overlap"), 4);
    CHECK(overlap.outcome == msg::NameOutcome::state_mismatch);
    CHECK_EQ(reg.seeds().size(), std::size_t{1});
    CHECK_EQ(reg.objects().size(), std::size_t{1});

    CHECK(reg.add_seed(kGen, 12, kMid, line(20, 29)));
    const auto anchored = reg.name(ask(1, only(reg).revision, kGen, 12, "Attach", "onto-anchored"), 5);
    CHECK(anchored.outcome == msg::NameOutcome::state_mismatch);
    CHECK_EQ(anchored.object_id, std::uint64_t{1});
    CHECK_EQ(anchored.revision, only(reg).revision);
    CHECK(only(reg).baseline == msg::Baseline::anchored);
    CHECK_EQ(reg.seeds().size(), std::size_t{2});
}

TEST(nine_of_ten_voxels_still_names) {
    TempDir dir;
    Disk disk;
    Registry reg = open_at(disk, dir.path, 1);
    CHECK(reg.add_seed(kGen, 10, kMid, line(0, 9)));
    reg.set_occupied([](Key key) { return unpack(key).i != 0; });
    const auto response = reg.name(ask(0, 0, kGen, 10, "Scope cart", "nine"), 1);
    CHECK(response.outcome == msg::NameOutcome::applied);
    CHECK(reg.seeds().empty());
}

TEST(seed_fifo_evicts_the_oldest_and_an_old_frame_is_unknown) {
    TempDir dir;
    Disk disk;
    Registry reg = open_at(disk, dir.path, 1);
    for (std::uint64_t n = 1; n <= 64; ++n) CHECK(reg.add_seed(kGen, n, kMid, line(static_cast<std::int64_t>(n), static_cast<std::int64_t>(n))));
    CHECK_EQ(reg.seeds().size(), std::size_t{64});
    CHECK_EQ(reg.seeds().front().event_id, std::uint64_t{1});
    CHECK(reg.add_seed(kGen, 65, kMid, line(65, 65)));
    CHECK_EQ(reg.seeds().size(), std::size_t{64});
    CHECK_EQ(reg.seeds().front().event_id, std::uint64_t{2});
    CHECK(reg.name(ask(0, 0, kGen, 1, "Gone", "evicted"), 1).outcome == msg::NameOutcome::unknown_event);
    const std::uint64_t before = reg.snapshot_seq();
    CHECK(reg.add_seed(kGen, 65, kMid, line(65, 65)));
    CHECK_EQ(reg.snapshot_seq(), before);

    CHECK(reg.add_seed(kGen, 80, kMid, line(80, 89)));
    reg.commit_frame_change(2);
    CHECK(reg.seeds().empty());
    CHECK(reg.name(ask(0, 0, kGen, 80, "Old", "old-frame"), 2).outcome == msg::NameOutcome::unknown_event);
}

TEST(dedupe_replays_until_120s_and_then_the_same_request_does_not_apply_twice) {
    TempDir dir;
    Disk disk;
    Registry reg = named(disk, dir.path, line(0, 9));
    const auto again = reg.name(ask(0, 0, kGen, 10, "Scope cart", "create"), 1);
    CHECK(again.outcome == msg::NameOutcome::applied);
    CHECK(!again.repeat_differs);
    CHECK_EQ(reg.objects().size(), std::size_t{1});
    auto different = ask(0, 0, kGen, 10, "Other name", "create");
    const auto replay = reg.name(different, 1);
    CHECK(replay.outcome == msg::NameOutcome::applied);
    CHECK(replay.repeat_differs);
    CHECK_EQ(only(reg).name, std::string("Scope cart"));

    const auto held = reg.name(ask(0, 0, kGen, 10, "Scope cart", "create"), kRegistryDedupeTtlNs);
    CHECK(held.outcome == msg::NameOutcome::applied);
    CHECK(!held.repeat_differs);

    const auto expired = reg.name(ask(0, 0, kGen, 10, "Scope cart", "create"), kRegistryDedupeTtlNs + 2);
    CHECK(expired.outcome == msg::NameOutcome::unknown_event);
    CHECK_EQ(reg.objects().size(), std::size_t{1});

    const auto renamed = reg.name(ask(1, 1, 0, 0, "Cart", "rename"), 10);
    CHECK(renamed.outcome == msg::NameOutcome::applied);
    CHECK_EQ(renamed.revision, std::uint64_t{2});
    const auto retry = reg.name(ask(1, 1, 0, 0, "Cart", "rename"), 10 + kRegistryDedupeTtlNs + 1);
    CHECK(retry.outcome == msg::NameOutcome::stale_revision);
    CHECK_EQ(retry.revision, std::uint64_t{2});
    CHECK_EQ(only(reg).revision, std::uint64_t{2});

    const auto future = reg.name(ask(1, 2, 0, 0, "Later", "future"), 100);
    CHECK(future.outcome == msg::NameOutcome::applied);
    const auto backward = reg.name(ask(1, 2, 0, 0, "Later", "future"), 50);
    CHECK(backward.outcome == msg::NameOutcome::applied);
    CHECK(!backward.repeat_differs);
    CHECK_EQ(only(reg).revision, std::uint64_t{3});
}

TEST(dedupe_cap_answers_storage_failed_without_storing_the_request) {
    TempDir dir;
    Disk disk;
    Registry reg = open_at(disk, dir.path, 1);
    reg.set_writer([](const std::filesystem::path&, std::span<const std::uint8_t>) { return true; });
    for (std::uint64_t n = 0; n < kRegistryDedupeCap; ++n) {
        const auto response = reg.name(ask(0, 1, 0, 0, "", "a" + std::to_string(n)), n);
        CHECK(response.outcome == msg::NameOutcome::invalid);
    }
    CHECK_EQ(reg.dedupe().size(), kRegistryDedupeCap);
    const auto overflow = reg.name(ask(0, 1, 0, 0, "", "overflow"), kRegistryDedupeCap);
    CHECK(overflow.outcome == msg::NameOutcome::storage_failed);
    CHECK_EQ(reg.dedupe().size(), kRegistryDedupeCap);
    CHECK(reg.objects().empty());
}

TEST(save_throw_rolls_back_and_save_false_applies_with_storage_error) {
    TempDir dir;
    Disk disk;
    Registry reg = open_at(disk, dir.path, 1);
    CHECK(reg.add_seed(kGen, 10, kMid, line(0, 9)));
    disk.fail = true;
    const auto failed = reg.name(ask(0, 0, kGen, 10, "Scope cart", "fail"), 1);
    CHECK(failed.outcome == msg::NameOutcome::storage_failed);
    CHECK(reg.objects().empty());
    CHECK_EQ(reg.seeds().size(), std::size_t{1});
    CHECK(!reg.persist_blocked());
    CHECK(reg.dedupe().empty());

    disk.fail = false;
    disk.unsynced = true;
    const auto applied = reg.name(ask(0, 0, kGen, 10, "Scope cart", "unsynced"), 2);
    CHECK(applied.outcome == msg::NameOutcome::applied);
    CHECK(reg.storage_error());
    CHECK(!reg.persist_blocked());
    CHECK_EQ(only(reg).name, std::string("Scope cart"));
}

TEST(a_required_write_blocks_names_until_it_is_installed) {
    TempDir dir;
    Disk disk;
    Registry reg = named(disk, dir.path, line(0, 9));
    reg.link_window(kMid, {LinkEvent{kGen, 11, msg::EventKind::disappeared, 0, line(0, 9), {}}});
    CHECK(only(reg).association == msg::Association::pending);
    PublishedChange published{kMid, 1, msg::EventKind::disappeared, line(100, 109), {}};
    Finalize finalize{kGen, 11, msg::EventKind::disappeared, 1};
    CHECK(reg.stage_pending_accept(published, finalize, true));
    CHECK(reg.record()->effect.kind == SpatialEffect::Kind::gone);
    const std::size_t dedupe = reg.dedupe().size();
    disk.fail = true;
    CHECK_THROWS(StorageError, reg.commit_frame_change(2));
    CHECK(reg.persist_blocked());
    CHECK_EQ(reg.frame_epoch(), std::uint64_t{1});
    CHECK(reg.record().has_value());
    CHECK(only(reg).baseline == msg::Baseline::anchored);
    const auto blocked = reg.name(ask(1, only(reg).revision, 0, 0, "Nope", "while-blocked"), 5);
    CHECK(blocked.outcome == msg::NameOutcome::storage_failed);
    CHECK_EQ(reg.dedupe().size(), dedupe);

    disk.fail = false;
    reg.commit_frame_change(2);
    CHECK(!reg.persist_blocked());
    CHECK(!reg.record());
    CHECK_EQ(reg.frame_epoch(), std::uint64_t{2});
    CHECK(reg.name(ask(1, only(reg).revision, 0, 0, "After", "after-block"), 6).outcome == msg::NameOutcome::applied);
}

TEST(load_into_a_new_frame_keeps_a_receipt_and_does_not_absorb) {
    TempDir dir;
    Disk disk;
    Registry reg = named(disk, dir.path, line(0, 9));
    reg.link_window(kMid, {LinkEvent{kGen, 11, msg::EventKind::disappeared, 0, line(0, 9), {}}});
    CHECK(reg.stage_pending_accept(PublishedChange{kMid, 1, msg::EventKind::disappeared, line(100, 109), {}},
                                   Finalize{kGen, 11, msg::EventKind::disappeared, 1}, true));
    CHECK(reg.record()->effect.kind == SpatialEffect::Kind::gone);
    CHECK(reg.record()->redo.has_value());
    int absorbed = 0;
    const int writes = disk.writes;
    Registry loaded = open_at(disk, dir.path, 2, [&](const PublishedChange&) { ++absorbed; });
    CHECK_EQ(absorbed, 0);
    CHECK_EQ(disk.writes, writes + 1);
    CHECK(!loaded.record());
    CHECK(only(loaded).baseline == msg::Baseline::retired);
    CHECK(only(loaded).anchor.has_value());
    CHECK_EQ(only(loaded).anchor->keys.size(), std::size_t{10});
    CHECK_EQ(only(loaded).anchor->frame_epoch, std::uint64_t{1});
    const ObjectLinkRecord* link = link_of(only(loaded), kGen, 11);
    CHECK(link && link->recovered);
    CHECK(has_receipt(loaded, kGen, 11));
    CHECK_EQ(loaded.receipts().size(), std::size_t{1});
    const std::uint64_t revision = only(loaded).revision;

    Registry again = open_at(disk, dir.path, 2, [&](const PublishedChange&) { ++absorbed; });
    CHECK_EQ(absorbed, 0);
    CHECK_EQ(disk.writes, writes + 1);
    CHECK(!again.record());
    CHECK(only(again).baseline == msg::Baseline::retired);
    CHECK_EQ(only(again).revision, revision);
    CHECK_EQ(again.receipts().size(), std::size_t{1});
    CHECK(link_of(only(again), kGen, 11)->recovered);
}

TEST(live_relearn_emits_one_receipt_with_or_without_the_redo) {
    TempDir dir;
    Disk disk;
    Registry reg = named(disk, dir.path, line(0, 9));
    reg.link_window(kMid, {LinkEvent{kGen, 11, msg::EventKind::disappeared, 0, line(0, 9), {}}});
    CHECK(reg.stage_pending_accept(PublishedChange{kMid, 1, msg::EventKind::disappeared, line(100, 109), {}},
                                   Finalize{kGen, 11, msg::EventKind::disappeared, 1}, true));
    reg.commit_frame_change(2);
    CHECK(!reg.record());
    CHECK(only(reg).baseline == msg::Baseline::retired);
    CHECK(only(reg).anchor.has_value());
    CHECK(link_of(only(reg), kGen, 11)->recovered);
    CHECK_EQ(reg.receipts().size(), std::size_t{1});
    const std::uint64_t revision = only(reg).revision;
    reg.commit_frame_change(2);
    CHECK_EQ(only(reg).revision, revision);
    CHECK_EQ(reg.receipts().size(), std::size_t{1});

    TempDir other;
    Disk disk2;
    Registry cleared = named(disk2, other.path, line(0, 9));
    cleared.link_window(kMid, {LinkEvent{kGen, 11, msg::EventKind::disappeared, 0, line(0, 9), {}}});
    CHECK(cleared.stage_pending_accept(PublishedChange{kMid, 1, msg::EventKind::disappeared, line(100, 109), {}},
                                       Finalize{kGen, 11, msg::EventKind::disappeared, 1}, false));
    CHECK(!cleared.record()->redo.has_value());
    CHECK(cleared.record()->finalize.has_value());
    cleared.commit_frame_change(2);
    CHECK(only(cleared).baseline == msg::Baseline::retired);
    CHECK(only(cleared).anchor.has_value());
    CHECK(has_receipt(cleared, kGen, 11));
    CHECK(link_of(only(cleared), kGen, 11)->recovered);
    const std::uint64_t once = only(cleared).revision;
    cleared.commit_frame_change(2);
    CHECK_EQ(only(cleared).revision, once);
    CHECK_EQ(cleared.receipts().size(), std::size_t{1});
}

TEST(an_unlinked_finalize_still_leaves_a_receipt_on_a_frame_change) {
    TempDir dir;
    Disk disk;
    Registry reg = named(disk, dir.path, line(0, 9));
    CHECK(reg.stage_pending_accept(PublishedChange{kMid, 1, msg::EventKind::appeared, {}, {}},
                                   Finalize{kGen, 12, msg::EventKind::appeared, 0}, true));
    reg.commit_frame_change(2);
    CHECK(has_receipt(reg, kGen, 12));
    CHECK(link_of(only(reg), kGen, 12) == nullptr);
    CHECK_EQ(reg.receipts().size(), std::size_t{1});
}

TEST(same_frame_replay_applies_gone_and_detach_and_then_attach_works) {
    TempDir dir;
    Disk disk;
    Registry reg = named(disk, dir.path, line(0, 9));
    reg.link_window(kMid, {LinkEvent{kGen, 11, msg::EventKind::disappeared, 0, line(0, 9), {}}});
    CHECK(reg.stage_pending_accept(PublishedChange{kMid, 1, msg::EventKind::disappeared, {}, {}},
                                   Finalize{kGen, 11, msg::EventKind::disappeared, 1}, true));
    int absorbed = 0;
    Registry loaded = open_at(disk, dir.path, 1, [&](const PublishedChange&) { ++absorbed; });
    CHECK_EQ(absorbed, 1);
    CHECK(!loaded.record());
    CHECK(only(loaded).baseline == msg::Baseline::gone);
    CHECK(!only(loaded).anchor);
    CHECK(has_receipt(loaded, kGen, 11));
    CHECK(link_of(only(loaded), kGen, 11)->recovered);
    const std::string name = only(loaded).name;
    CHECK(loaded.add_seed(kGen, 20, kMid, line(20, 29)));
    const auto attached = loaded.name(ask(1, only(loaded).revision, kGen, 20, "Different", "attach"), 5);
    CHECK(attached.outcome == msg::NameOutcome::applied);
    CHECK(only(loaded).baseline == msg::Baseline::anchored);
    CHECK_EQ(only(loaded).name, name);
    CHECK(loaded.seeds().empty());

    TempDir detached_dir;
    Disk detached_disk;
    Registry half = named(detached_disk, detached_dir.path, line(0, 9));
    CHECK(half.add_seed(kGen, 30, kMid, line(0, 3)));
    half.set_occupied([](Key key) { return unpack(key).i >= 5; });
    CHECK(half.stage_pending_accept(PublishedChange{kMid, 1, msg::EventKind::disappeared, line(0, 3), {}},
                                    Finalize{kGen, 40, msg::EventKind::disappeared, 0}, true));
    CHECK(half.record()->effect.kind == SpatialEffect::Kind::none);
    CHECK_EQ(half.record()->effect.detach_ids.size(), std::size_t{1});
    CHECK_EQ(half.record()->effect.detach_ids.front(), std::uint64_t{1});
    CHECK(only(half).baseline == msg::Baseline::anchored);
    CHECK(half.seeds().empty());
    int half_absorb = 0;
    Occupied still = [](Key key) { return unpack(key).i >= 5; };
    Registry after = open_at(detached_disk, detached_dir.path, 1, [&](const PublishedChange&) { ++half_absorb; }, still);
    CHECK_EQ(half_absorb, 1);
    CHECK(!after.record());
    CHECK(only(after).baseline == msg::Baseline::detached);
    CHECK(has_receipt(after, kGen, 40));
    CHECK(after.add_seed(kGen, 41, kMid, line(20, 29)));
    const auto renamed = after.name(ask(1, only(after).revision, kGen, 41, "Different", "reattach"), 8);
    CHECK(renamed.outcome == msg::NameOutcome::applied);
    CHECK(only(after).baseline == msg::Baseline::anchored);
    CHECK_EQ(only(after).name, std::string("Scope cart"));
}

TEST(a_second_accept_is_refused_while_a_record_is_outstanding) {
    TempDir dir;
    Disk disk;
    Registry reg = named(disk, dir.path, line(0, 9));
    CHECK(reg.stage_pending_accept(PublishedChange{kMid, 1, msg::EventKind::disappeared, {}, {}},
                                   Finalize{kGen, 11, msg::EventKind::disappeared, 0}, true));
    CHECK_THROWS(RegistryError, reg.stage_pending_accept(PublishedChange{kMid, 1, msg::EventKind::disappeared, {}, {}},
                                                         Finalize{kGen, 12, msg::EventKind::disappeared, 0}, true));
    CHECK(reg.record()->finalize->event_id == 11);
    CHECK_THROWS(RegistryError, reg.settle());
    CHECK(only(reg).baseline == msg::Baseline::anchored);
}

TEST(realignment_keeps_anchors_and_a_mismatch_fails_closed) {
    TempDir dir;
    Disk disk;
    Registry reg = named(disk, dir.path, line(0, 9));
    reg.link_window(kMid, {LinkEvent{kGen, 11, msg::EventKind::disappeared, 0, line(0, 9), {}}});
    CHECK(only(reg).association == msg::Association::pending);
    const std::uint64_t seq = reg.snapshot_seq();
    CHECK(seq > 0);
    int absorbed = 0;
    const int writes = disk.writes;
    Registry same = open_at(disk, dir.path, 1, [&](const PublishedChange&) { ++absorbed; });
    CHECK_EQ(absorbed, 0);
    CHECK_EQ(disk.writes, writes);
    CHECK(only(same).baseline == msg::Baseline::anchored);
    CHECK(only(same).association == msg::Association::none);
    CHECK_EQ(same.snapshot_seq(), std::uint64_t{0});

    CHECK_THROWS(RegistryError, open_at(disk, dir.path, 1, {}, {}, kJt));
    CHECK_EQ(disk.writes, writes);
    CHECK_THROWS(RegistryError, open_at(disk, dir.path, 1, {}, {}, kMid, 0.1));
    CHECK_EQ(disk.writes, writes);
    std::filesystem::create_directories(dir.path / "other");
    std::filesystem::copy_file(dir.path / "lab" / "objects", dir.path / "other" / "objects");
    CHECK_THROWS(RegistryError, open_at(disk, dir.path, 1, {}, {}, kMid, 0.05, "other"));
    CHECK_EQ(disk.writes, writes);
    CHECK_EQ(absorbed, 0);
}

TEST(linking_bands) {
    TempDir dir;
    Disk disk;
    Registry reg = open_at(disk, dir.path, 1);
    CHECK(reg.add_seed(kGen, 1, kMid, line(0, 9)));
    CHECK(reg.name(ask(0, 0, kGen, 1, "A", "a"), 1).outcome == msg::NameOutcome::applied);
    CHECK(reg.add_seed(kGen, 2, kMid, line(10, 19)));
    CHECK(reg.name(ask(0, 0, kGen, 2, "B", "b"), 2).outcome == msg::NameOutcome::applied);

    auto association = [&](std::uint64_t id) {
        for (const Object& obj : reg.objects()) {
            if (obj.id == id) return obj.association;
        }
        throw harness::Failure("missing object");
    };
    const std::uint64_t before = reg.snapshot_seq();
    reg.link_window(kMid, {LinkEvent{kGen, 11, msg::EventKind::disappeared, 0, line(0, 9), {}}});
    CHECK(association(1) == msg::Association::pending);
    CHECK(association(2) == msg::Association::none);
    CHECK_EQ(reg.snapshot_seq(), before + 1);
    reg.link_window(kMid, {LinkEvent{kGen, 11, msg::EventKind::disappeared, 0, line(0, 9), {}}});
    CHECK_EQ(reg.snapshot_seq(), before + 1);

    reg.link_window(kMid, {LinkEvent{kGen, 11, msg::EventKind::disappeared, 0, line(0, 8), {}}});
    CHECK(association(1) == msg::Association::pending);
    reg.link_window(kMid, {LinkEvent{kGen, 11, msg::EventKind::disappeared, 0, line(0, 4), {}}});
    CHECK(association(1) == msg::Association::unresolved);
    CHECK(association(2) == msg::Association::none);

    Keys both = line(0, 9);
    Keys rest = line(10, 19);
    both.insert(both.end(), rest.begin(), rest.end());
    sort_unique(both);
    reg.link_window(kMid, {LinkEvent{kGen, 12, msg::EventKind::disappeared, 0, both, {}}});
    CHECK(association(1) == msg::Association::unresolved);
    CHECK(association(2) == msg::Association::unresolved);

    reg.link_window(kMid, {LinkEvent{kGen, 13, msg::EventKind::moved, 0, line(0, 13), line(0, 9)}});
    CHECK(association(1) == msg::Association::unresolved);

    reg.link_window(kMid, {LinkEvent{kGen, 14, msg::EventKind::moved, 0, line(5, 14), line(0, 9)}});
    CHECK(association(1) == msg::Association::pending);
    reg.link_window(kMid, {LinkEvent{kGen, 15, msg::EventKind::moved, 0, line(1000, 1009), line(0, 9)}});
    CHECK(association(1) == msg::Association::pending);
    reg.link_window(kMid, {LinkEvent{kGen, 16, msg::EventKind::moved, 0, column(100, 10), line(0, 9)}});
    CHECK(association(1) == msg::Association::unresolved);

    reg.link_window(kMid, {LinkEvent{kGen, 11, msg::EventKind::disappeared, 4, line(0, 9), {}}});
    CHECK(association(1) == msg::Association::none);
    reg.link_window(kMid, {LinkEvent{kGen, 11, msg::EventKind::disappeared, 0, line(0, 9), {}}});
    CHECK(association(1) == msg::Association::pending);
    reg.link_window(kMid, {});
    CHECK(association(1) == msg::Association::none);
    CHECK(association(2) == msg::Association::none);
}

TEST(settle_detaches_a_thin_anchor_and_a_restart_drops_association) {
    TempDir dir;
    Disk disk;
    Registry reg = named(disk, dir.path, line(0, 9));
    reg.set_occupied([](Key) { return false; });
    const std::uint64_t revision = only(reg).revision;
    reg.settle();
    CHECK(only(reg).baseline == msg::Baseline::detached);
    CHECK_EQ(only(reg).revision, revision + 1);
    const std::uint64_t settled = only(reg).revision;
    reg.settle();
    CHECK_EQ(only(reg).revision, settled);

    TempDir fresh;
    Registry linked = named(disk, fresh.path, line(30, 39));
    linked.link_window(kMid, {LinkEvent{kGen, 11, msg::EventKind::disappeared, 0, line(30, 39), {}}});
    CHECK(only(linked).association == msg::Association::pending);
    Registry restarted = open_at(disk, fresh.path, linked.frame_epoch());
    for (const Object& obj : restarted.objects()) CHECK(obj.association == msg::Association::none);
    CHECK_EQ(restarted.snapshot_seq(), std::uint64_t{0});
    CHECK(restarted.objects().size() == linked.objects().size());
}

TEST(corrupt_crc_is_rejected) {
    TempDir dir;
    Disk disk;
    Registry reg = open_at(disk, dir.path, 1);
    (void)reg;
    const std::filesystem::path file = dir.path / "lab" / "objects";
    std::fstream io(file, std::ios::in | std::ios::out | std::ios::binary);
    CHECK(io);
    io.seekp(-1, std::ios::end);
    char byte = 0;
    io.seekg(-1, std::ios::end);
    io.read(&byte, 1);
    byte = static_cast<char>(byte ^ 0xFF);
    io.seekp(-1, std::ios::end);
    io.write(&byte, 1);
    io.close();
    CHECK_THROWS(RegistryError, open_at(disk, dir.path, 1));
}

TEST_MAIN
