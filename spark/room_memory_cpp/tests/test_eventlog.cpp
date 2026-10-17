// Durable event log (D45): crash recovery, prefix retention, and RequestSync past the 100-deep cache.
#include <unistd.h>
#include <zlib.h>

#include <algorithm>
#include <cstring>
#include <fstream>
#include <map>
#include <memory>
#include <optional>
#include <set>

#include "harness.hpp"
#include "room_memory/eventlog.hpp"
#include "room_memory/generation.hpp"
#include "room_memory/memory.hpp"
#include "scenes.hpp"

using namespace room_memory;
using msg::EventState;

namespace {

const msg::Time kStart{1760400000, 0};
constexpr std::uint64_t kGeneration = 1760400000000ULL;
constexpr std::int64_t kDay = 24LL * 60 * 60 * 1'000'000'000LL;

msg::Time stamp(std::int64_t second) { return msg::Time{kStart.sec + second, 0}; }
msg::Time day_stamp(int day) { return msg::Time{kStart.sec + static_cast<std::int64_t>(day) * 86400, 0}; }
std::int64_t day_ns(int day) { return day_stamp(day).sec * 1'000'000'000LL; }

const msg::Token kTokenA = {0x5a, 0x1e, 0x7c, 0x3d, 0x9b, 0x02, 0x4f, 0x61,
                            0x8e, 0x33, 0xc4, 0xd5, 0xe6, 0xf7, 0x08, 0x19};

msg::RequestSync sync_request(std::optional<msg::RequestSync::Since> since, msg::SyncTopic topic,
                              std::optional<std::uint64_t> expected = kGeneration, const std::string& room_id = "lab") {
    msg::RequestSync request;
    request.header = msg::Header{room_id, since ? since->epoch : msg::kSinceNone, 0, kStart};
    request.since = since;
    request.topic = topic;
    request.expected_generation = expected;
    if (expected) request.token = kTokenA;
    return request;
}

using Ranges = std::vector<msg::SeqRange>;

struct TempDir {
    std::filesystem::path path;
    TempDir() {
        std::string pattern = (std::filesystem::temp_directory_path() / "room-memory-test-XXXXXX").string();
        if (!mkdtemp(pattern.data())) throw harness::Failure("mkdtemp failed");
        path = pattern;
    }
    ~TempDir() { std::filesystem::remove_all(path); }
};

struct Room {
    TempDir dir;
    std::filesystem::path path;
    Room() : path(dir.path / "lab") { std::filesystem::create_directory(path); }
};

MemoryConfig config_with(const std::optional<std::string>& state_dir) {
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

std::int64_t build_reference(RoomMemory& memory, const scenes::Cloud& cloud, int windows = 3) {
    for (int second = 0; second < windows; ++second) {
        memory.add_points("mid360", cloud);
        CHECK(memory.close_window(stamp(second)).empty());
    }
    return windows;
}

msg::Bytes bytes_of(std::uint8_t mark, std::size_t size = 4) { return msg::Bytes(size, mark); }

void commit_one(EventLog& log, std::uint64_t generation, std::uint64_t seq, std::int64_t stamp_ns, msg::Bytes cdr) {
    log.append(generation, seq, 7, stamp_ns, std::move(cdr));
    log.commit();
}

std::filesystem::path only_segment(const std::filesystem::path& room) {
    const std::filesystem::path dir = room / "events";
    std::filesystem::path found;
    for (const auto& entry : std::filesystem::directory_iterator(dir)) {
        const std::string name = entry.path().filename().string();
        if (name.rfind("seg-", 0) == 0) {
            CHECK(found.empty());
            found = entry.path();
        }
    }
    CHECK(!found.empty());
    return found;
}

std::vector<std::uint8_t> read_file(const std::filesystem::path& path) {
    std::ifstream in(path, std::ios::binary);
    CHECK(in);
    return std::vector<std::uint8_t>((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
}

void write_file(const std::filesystem::path& path, const std::vector<std::uint8_t>& data) {
    std::ofstream out(path, std::ios::binary | std::ios::trunc);
    CHECK(out);
    out.write(reinterpret_cast<const char*>(data.data()), static_cast<std::streamsize>(data.size()));
    CHECK(out);
}

std::uint32_t read_u32(const std::uint8_t* p) {
    return std::uint32_t{p[0]} | (std::uint32_t{p[1]} << 8) | (std::uint32_t{p[2]} << 16) | (std::uint32_t{p[3]} << 24);
}

// Offsets of every well-formed header. Used to damage a record the scanner must not skip.
std::vector<std::uint64_t> record_offsets(const std::vector<std::uint8_t>& data) {
    std::vector<std::uint64_t> offsets;
    std::uint64_t offset = 0;
    while (offset + 12 <= data.size()) {
        const std::uint32_t len = read_u32(data.data() + offset + 4);
        CHECK(offset + 12 + len <= data.size());
        offsets.push_back(offset);
        offset += 12 + len;
    }
    CHECK(offset == data.size());
    return offsets;
}

void put_u32(std::uint8_t* p, std::uint32_t value) {
    p[0] = static_cast<std::uint8_t>(value);
    p[1] = static_cast<std::uint8_t>(value >> 8);
    p[2] = static_cast<std::uint8_t>(value >> 16);
    p[3] = static_cast<std::uint8_t>(value >> 24);
}

struct IoFault : std::runtime_error {
    using std::runtime_error::runtime_error;
};

struct TracingIo : EventLogIo {
    PosixEventLogIo real;
    int appends = 0, file_syncs = 0, dir_syncs = 0, truncates = 0, unlinks = 0, mkdirs = 0;
    std::vector<std::filesystem::path> synced_dirs;
    std::string fail;
    std::string fail_name;
    int fail_after = 0;
    int fail_seen = 0;

    void guard(const std::string& op, const std::filesystem::path& path = {}) {
        if (fail != op) return;
        if (!fail_name.empty() && path.filename() != fail_name) return;
        if (fail_seen >= fail_after) throw IoFault(op);
        ++fail_seen;
    }

    bool exists(const std::filesystem::path& path) override { return real.exists(path); }
    void create_directory(const std::filesystem::path& path) override {
        guard("mkdir", path);
        ++mkdirs;
        real.create_directory(path);
    }
    std::vector<std::filesystem::path> list(const std::filesystem::path& dir) override { return real.list(dir); }
    std::uint64_t file_size(const std::filesystem::path& path) override { return real.file_size(path); }
    void read_at(const std::filesystem::path& path, std::uint64_t offset, std::span<std::uint8_t> dest) override {
        real.read_at(path, offset, dest);
    }
    void append(const std::filesystem::path& path, std::span<const std::uint8_t> data) override {
        guard("append", path);
        ++appends;
        real.append(path, data);
    }
    void fsync_file(const std::filesystem::path& path) override {
        guard("fsync_file", path);
        ++file_syncs;
        real.fsync_file(path);
    }
    void fsync_dir(const std::filesystem::path& path) override {
        guard("fsync_dir", path);
        ++dir_syncs;
        synced_dirs.push_back(path);
        real.fsync_dir(path);
    }
    void truncate(const std::filesystem::path& path, std::uint64_t size) override {
        guard("truncate", path);
        ++truncates;
        real.truncate(path, size);
    }
    void remove_file(const std::filesystem::path& path) override {
        guard("unlink", path);
        ++unlinks;
        real.remove_file(path);
    }
    void write_atomic(const std::filesystem::path& path, std::span<const std::uint8_t> data) override {
        guard("write_atomic", path);
        real.write_atomic(path, data);
    }
};

// Page cache versus disk. Writes change only the visible bytes. fsync_file makes those
// bytes durable; fsync_dir makes the directory entries durable. power_loss() drops
// whatever is not yet both. crash_next_file_sync dies inside fsync_file and sets dead,
// so EventLog's failure rollback cannot truncate the visible tail. That is a process
// crash with the page cache still warm, which is the state recovery then has to harden
// before a power loss.
struct Crash : std::runtime_error {
    using std::runtime_error::runtime_error;
};

struct CrashIo : EventLogIo {
    struct Node {
        bool is_dir = false;
        std::vector<std::uint8_t> visible_data;
        std::vector<std::uint8_t> durable_data;
        std::map<std::string, std::unique_ptr<Node>> children;
        std::set<std::string> visible_names;
        std::set<std::string> durable_names;
    };
    struct Note {
        bool dir = false;
        std::string name;
    };

    std::filesystem::path root_path;
    Node root;
    bool dead = false;
    bool crash_next_file_sync = false;
    std::vector<Note> notes;

    explicit CrashIo(std::filesystem::path room) : root_path(std::move(room)) { root.is_dir = true; }

    void power_loss() { restore(root); }

    std::optional<std::uint64_t> visible_bytes(const std::filesystem::path& path) const {
        const Node* node = walk(path, false);
        if (!node || node->is_dir) return std::nullopt;
        return node->visible_data.size();
    }
    std::optional<std::uint64_t> durable_bytes(const std::filesystem::path& path) const {
        const Node* node = walk(path, true);
        if (!node || node->is_dir) return std::nullopt;
        return node->durable_data.size();
    }

    bool exists(const std::filesystem::path& path) override { return walk(path, false) != nullptr; }
    void create_directory(const std::filesystem::path& path) override {
        poke("mkdir");
        if (walk(path, false)) return;
        Node& parent = require_dir(path.parent_path());
        auto created = std::make_unique<Node>();
        created->is_dir = true;
        parent.visible_names.insert(path.filename().string());
        parent.children[path.filename().string()] = std::move(created);
    }
    std::vector<std::filesystem::path> list(const std::filesystem::path& dir) override {
        const Node* node = walk(dir, false);
        if (!node || !node->is_dir) throw std::runtime_error("cannot list " + dir.string());
        std::vector<std::filesystem::path> out;
        for (const std::string& name : node->visible_names) out.push_back(dir / name);
        return out;
    }
    std::uint64_t file_size(const std::filesystem::path& path) override {
        const Node* node = walk(path, false);
        if (!node || node->is_dir) throw std::runtime_error("cannot stat " + path.string());
        return node->visible_data.size();
    }
    void read_at(const std::filesystem::path& path, std::uint64_t offset, std::span<std::uint8_t> dest) override {
        const Node* node = walk(path, false);
        if (!node || node->is_dir) throw std::runtime_error("cannot read " + path.string());
        if (offset > node->visible_data.size() || dest.size() > node->visible_data.size() - static_cast<std::size_t>(offset)) {
            throw std::runtime_error("short read " + path.string());
        }
        std::copy_n(node->visible_data.begin() + static_cast<std::ptrdiff_t>(offset), dest.size(), dest.begin());
    }
    void append(const std::filesystem::path& path, std::span<const std::uint8_t> data) override {
        poke("append");
        Node& parent = require_dir(path.parent_path());
        const std::string name = path.filename().string();
        Node* node = nullptr;
        if (parent.visible_names.count(name)) {
            node = parent.children[name].get();
            if (!node || node->is_dir) throw std::runtime_error("cannot write " + path.string());
        } else {
            auto created = std::make_unique<Node>();
            node = created.get();
            parent.children[name] = std::move(created);
            parent.visible_names.insert(name);
        }
        node->visible_data.insert(node->visible_data.end(), data.begin(), data.end());
    }
    void fsync_file(const std::filesystem::path& path) override {
        poke("fsync_file");
        if (crash_next_file_sync) {
            crash_next_file_sync = false;
            dead = true;
            throw Crash("fsync_file");
        }
        Node* node = walk_mut(path, false);
        if (!node || node->is_dir) throw std::runtime_error("cannot sync " + path.string());
        node->durable_data = node->visible_data;
        notes.push_back(Note{false, path.filename().string()});
    }
    void fsync_dir(const std::filesystem::path& path) override {
        poke("fsync_dir");
        Node* node = walk_mut(path, false);
        if (!node || !node->is_dir) throw std::runtime_error("cannot sync " + path.string());
        node->durable_names = node->visible_names;
        notes.push_back(Note{true, path.filename().string()});
    }
    void truncate(const std::filesystem::path& path, std::uint64_t size) override {
        poke("truncate");
        Node* node = walk_mut(path, false);
        if (!node || node->is_dir) throw std::runtime_error("cannot truncate " + path.string());
        node->visible_data.resize(static_cast<std::size_t>(size));
    }
    void remove_file(const std::filesystem::path& path) override {
        poke("unlink");
        Node* parent = walk_mut(path.parent_path(), false);
        if (!parent || !parent->is_dir) return;
        parent->visible_names.erase(path.filename().string());
    }
    void write_atomic(const std::filesystem::path& path, std::span<const std::uint8_t> data) override {
        poke("write_atomic");
        Node& parent = require_dir(path.parent_path());
        auto created = std::make_unique<Node>();
        created->visible_data.assign(data.begin(), data.end());
        created->durable_data = created->visible_data;
        const std::string name = path.filename().string();
        parent.children[name] = std::move(created);
        parent.visible_names.insert(name);
        parent.durable_names = parent.visible_names;
        notes.push_back(Note{false, name});
        notes.push_back(Note{true, path.parent_path().filename().string()});
    }

    std::optional<std::size_t> note_at(bool dir, const std::string& name) const {
        for (std::size_t n = 0; n < notes.size(); ++n) {
            if (notes[n].dir == dir && notes[n].name == name) return n;
        }
        return std::nullopt;
    }

private:
    void poke(const char* what) {
        if (dead) throw Crash(what);
    }
    void restore(Node& node) {
        node.visible_data = node.durable_data;
        node.visible_names = node.durable_names;
        std::vector<std::string> drop;
        for (const auto& [name, child] : node.children) {
            if (!node.durable_names.count(name)) drop.push_back(name);
        }
        for (const std::string& name : drop) node.children.erase(name);
        for (auto& [name, child] : node.children) restore(*child);
    }
    const Node* walk(const std::filesystem::path& path, bool durable) const {
        return const_cast<CrashIo*>(this)->walk_mut(path, durable);
    }
    Node* walk_mut(const std::filesystem::path& path, bool durable) {
        const auto normal = path.lexically_normal();
        const auto base = root_path.lexically_normal();
        if (normal == base) return &root;
        const auto rel = normal.lexically_relative(base);
        if (rel.empty() || rel.is_absolute() || *rel.begin() == "..") return nullptr;
        Node* node = &root;
        for (const std::filesystem::path& part : rel) {
            const std::string name = part.string();
            const std::set<std::string>& names = durable ? node->durable_names : node->visible_names;
            if (!names.count(name)) return nullptr;
            const auto it = node->children.find(name);
            if (it == node->children.end() || !it->second) return nullptr;
            node = it->second.get();
        }
        return node;
    }
    Node& require_dir(const std::filesystem::path& path) {
        Node* node = walk_mut(path, false);
        if (!node || !node->is_dir) throw std::runtime_error("missing directory " + path.string());
        return *node;
    }
};

void require_durable_before_manifest(const CrashIo& io, const std::string& segment) {
    const std::optional<std::size_t> seg = io.note_at(false, segment);
    const std::optional<std::size_t> manifest = io.note_at(false, "manifest.log");
    const std::optional<std::size_t> events = io.note_at(true, "events");
    const std::optional<std::size_t> room = io.note_at(true, "lab");
    CHECK(seg && manifest && events && room);
    CHECK(*seg < *manifest && *events < *manifest && *room < *manifest);
}

EventReplay live(const EventLog& log, std::uint64_t generation, std::uint64_t next_seq,
                 std::vector<msg::Bytes>& cache_bytes, std::vector<EventPiece> open = {}) {
    std::vector<EventPiece> cache;
    for (std::size_t n = 0; n < cache_bytes.size(); ++n) {
        cache.push_back(EventPiece{static_cast<std::uint64_t>(n), 7, &cache_bytes[n]});
    }
    return log.replay_current(generation, next_seq, std::nullopt, cache, open);
}

}  // namespace

TEST(limits_and_a_missing_room_do_not_create_directories) {
    Room room;
    CHECK_THROWS(std::invalid_argument, EventLog::open(room.path, EventLogLimits{0, 0, 1}));
    CHECK_THROWS(std::invalid_argument, EventLog::open(room.path, EventLogLimits{10, 32, 1}));
    CHECK_THROWS(std::invalid_argument, EventLog::open(room.path, EventLogLimits{32, 16, 0}));
    EventLog log = EventLog::open(room.dir.path / "absent", EventLogLimits{});
    log.append(kGeneration, 0, 1, 0, bytes_of(1));
    CHECK_THROWS(std::runtime_error, log.commit());
    CHECK(!std::filesystem::exists(room.dir.path / "absent"));
}

TEST(empty_commit_does_no_io_and_the_first_commit_fsyncs_the_room_directory) {
    Room room;
    TracingIo io;
    EventLog log = EventLog::open(room.path, EventLogLimits{}, &io);
    log.commit();
    CHECK(io.appends == 0 && io.file_syncs == 0 && io.dir_syncs == 0 && io.mkdirs == 0);
    CHECK(!std::filesystem::exists(room.path / "events"));
    commit_one(log, kGeneration, 0, 10, bytes_of(3));
    CHECK(io.appends >= 1 && io.file_syncs >= 1 && io.mkdirs == 1);
    CHECK(std::find_if(io.synced_dirs.begin(), io.synced_dirs.end(),
                       [&](const std::filesystem::path& path) { return path == room.path; }) != io.synced_dirs.end());
    CHECK(std::find_if(io.synced_dirs.begin(), io.synced_dirs.end(), [&](const std::filesystem::path& path) {
              return path.filename() == "events";
          }) != io.synced_dirs.end());
    const int appends = io.appends, syncs = io.file_syncs, dirs = io.dir_syncs;
    log.commit();
    CHECK(io.appends == appends && io.file_syncs == syncs && io.dir_syncs == dirs);
}

TEST(torn_tail_and_a_full_length_bad_crc_are_truncated) {
    // The bad bytes are an uncommitted tail: the manifest names only the committed record.
    // A CRC-valid record the manifest already names is not a tear (see the middle-corruption test).
    Room room;
    {
        EventLog log = EventLog::open(room.path, EventLogLimits{});
        commit_one(log, kGeneration, 0, 1, bytes_of(1));
    }
    const std::filesystem::path segment = only_segment(room.path);
    const std::vector<std::uint8_t> good = read_file(segment);
    const std::vector<std::uint8_t> committed = good;

    auto restored = [&](std::vector<std::uint8_t> damaged) {
        write_file(segment, damaged);
        EventLog log = EventLog::open(room.path, EventLogLimits{});
        const EventReplay replay = log.replay_archive(kGeneration, std::nullopt).value();
        CHECK((replay.messages == std::vector<msg::Bytes>{bytes_of(1)}));
        CHECK(replay.bound == 0 && replay.unavailable.empty());
        CHECK(read_file(segment) == committed);
    };

    std::vector<std::uint8_t> torn = good;
    torn.insert(torn.end(), 8, 0xAB);
    restored(torn);

    std::vector<std::uint8_t> past_eof = good;
    past_eof.insert(past_eof.end(), {0x45, 0x56, 0x4C, 0x31, 0xE8, 0x03, 0x00, 0x00, 0, 0, 0, 0, 1, 2, 3});
    restored(past_eof);

    std::vector<std::uint8_t> bad_crc = good;
    bad_crc.insert(bad_crc.end(), good.begin(), good.end());
    bad_crc[good.size() + 12 + 10] ^= 0xFF;
    restored(bad_crc);

    std::vector<std::uint8_t> bad_magic = good;
    bad_magic.insert(bad_magic.end(), 12, 0);
    restored(bad_magic);
}

TEST(truncate_fsync_failure_refuses_open_and_a_later_open_keeps_the_prefix) {
    Room room;
    {
        EventLog log = EventLog::open(room.path, EventLogLimits{});
        commit_one(log, kGeneration, 0, 1, bytes_of(1));
    }
    const std::filesystem::path segment = only_segment(room.path);
    std::vector<std::uint8_t> torn = read_file(segment);
    const auto good_size = torn.size();
    torn.insert(torn.end(), 6, 0x11);
    write_file(segment, torn);
    {
        TracingIo io;
        io.fail = "fsync_file";
        CHECK_THROWS(IoFault, EventLog::open(room.path, EventLogLimits{}, &io));
    }
    EventLog log = EventLog::open(room.path, EventLogLimits{});
    const EventReplay replay = log.replay_archive(kGeneration, std::nullopt).value();
    CHECK(replay.messages == std::vector<msg::Bytes>{bytes_of(1)});
    CHECK(read_file(segment).size() == good_size);
    commit_one(log, kGeneration, 1, 2, bytes_of(9));
    EventLog again = EventLog::open(room.path, EventLogLimits{});
    CHECK(again.replay_archive(kGeneration, std::nullopt)->messages.size() == 2);
}

TEST(corruption_before_a_later_valid_record_refuses_startup) {
    Room room;
    {
        EventLog log = EventLog::open(room.path, EventLogLimits{});
        commit_one(log, kGeneration, 0, 1, bytes_of(1));
        commit_one(log, kGeneration, 1, 2, bytes_of(2));
    }
    const std::filesystem::path segment = only_segment(room.path);
    std::vector<std::uint8_t> data = read_file(segment);
    const std::vector<std::uint64_t> offsets = record_offsets(data);
    data[static_cast<std::size_t>(offsets[0]) + 12 + 4] ^= 0x5A;
    write_file(segment, data);
    CHECK_THROWS(std::runtime_error, EventLog::open(room.path, EventLogLimits{}));
    data = read_file(segment);
    std::fill_n(data.begin(), 4, 0);
    write_file(segment, data);
    CHECK_THROWS(std::runtime_error, EventLog::open(room.path, EventLogLimits{}));

    EventLogLimits tiny;
    tiny.segment_bytes = 1;
    tiny.total_bytes = 1;
    Room split;
    {
        EventLog log = EventLog::open(split.path, tiny);
        commit_one(log, kGeneration, 0, 1, bytes_of(1));
        commit_one(log, kGeneration, 1, 2, bytes_of(2));
    }
    std::ofstream extra(split.path / "events" / "seg-000001.log", std::ios::binary | std::ios::app);
    extra << "torn";
    extra.close();
    CHECK_THROWS(std::runtime_error, EventLog::open(split.path, tiny));
}

TEST(a_crc_valid_retain_that_goes_backwards_is_not_truncated) {
    Room room;
    {
        EventLog log = EventLog::open(room.path, EventLogLimits{});
        commit_one(log, kGeneration, 0, 50, bytes_of(1));
        commit_one(log, kGeneration, 1, 60, bytes_of(2));
    }
    const std::filesystem::path segment = only_segment(room.path);
    std::vector<std::uint8_t> data = read_file(segment);
    const std::uint64_t second = record_offsets(data)[1];
    std::fill_n(data.begin() + static_cast<std::ptrdiff_t>(second + 12 + 32), 8, 0);
    const std::uint32_t len = read_u32(data.data() + second + 4);
    const auto crc = ::crc32(0L, data.data() + second + 12, len);
    put_u32(data.data() + second + 8, static_cast<std::uint32_t>(crc));
    write_file(segment, data);
    CHECK_THROWS(std::runtime_error, EventLog::open(room.path, EventLogLimits{}));
}

TEST(backward_stamps_do_not_open_a_hole_and_a_later_age_drops_the_prefix) {
    EventLogLimits limits;
    limits.age_ns = 7 * kDay;
    Room room;
    msg::Bytes cdrs[4] = {bytes_of(10), bytes_of(1), bytes_of(11), bytes_of(18)};
    {
        EventLog log = EventLog::open(room.path, limits);
        commit_one(log, kGeneration, 0, day_ns(10), cdrs[0]);
        commit_one(log, kGeneration, 1, day_ns(1), cdrs[1]);
        commit_one(log, kGeneration, 2, day_ns(10), cdrs[2]);
    }
    {
        EventLog log = EventLog::open(room.path, limits);
        const EventReplay replay = log.replay_archive(kGeneration, std::nullopt).value();
        CHECK((replay.messages == std::vector<msg::Bytes>{cdrs[0], cdrs[1], cdrs[2]}));
        CHECK(replay.unavailable.empty() && (replay.delivered == Ranges{{0, 2}}) && replay.bound == 2);
        commit_one(log, kGeneration, 3, day_ns(18), cdrs[3]);
        std::vector<msg::Bytes> cache = {cdrs[0], cdrs[1], cdrs[2], cdrs[3]};
        EventPiece open_box{0, 7, &cdrs[0]};
        const EventReplay held = log.replay_current(kGeneration, 4, std::nullopt, {}, std::span<const EventPiece>(&open_box, 1));
        // The cache is empty here: file bytes only, plus the still-open seq 0.
        CHECK((held.messages == std::vector<msg::Bytes>{cdrs[0], cdrs[3]}));
        CHECK((held.unavailable == Ranges{{1, 2}}) && (held.delivered == Ranges{{0, 0}, {3, 3}}));
        const EventReplay cached = live(log, kGeneration, 4, cache, {open_box});
        CHECK(cached.messages == held.messages && cached.unavailable == held.unavailable);
        const EventReplay closed = live(log, kGeneration, 4, cache);
        CHECK((closed.messages == std::vector<msg::Bytes>{cdrs[3]}));
        CHECK((closed.unavailable == Ranges{{0, 2}}));
        const EventReplay again = log.replay_current(kGeneration, 4, std::nullopt, {}, {});
        CHECK(again.unavailable == closed.unavailable && again.messages == closed.messages);
    }
    EventLog reopened = EventLog::open(room.path, limits);
    const EventReplay replay = reopened.replay_archive(kGeneration, std::nullopt).value();
    CHECK((replay.messages == std::vector<msg::Bytes>{cdrs[3]}));
    CHECK((replay.unavailable == Ranges{{0, 2}}) && replay.bound == 3);
}

TEST(size_rotation_drops_a_prefix_and_the_cache_cannot_bring_it_back) {
    EventLogLimits limits;
    limits.segment_bytes = 1;
    limits.total_bytes = 1;
    Room room;
    EventLog log = EventLog::open(room.path, limits);
    const msg::Bytes first = bytes_of(1), second = bytes_of(2), third = bytes_of(3);
    commit_one(log, kGeneration, 0, 1, first);
    commit_one(log, kGeneration, 1, 2, second);
    commit_one(log, kGeneration, 2, 3, third);
    std::vector<msg::Bytes> cache = {first, second, third};
    const EventReplay replay = live(log, kGeneration, 3, cache);
    CHECK((replay.messages == std::vector<msg::Bytes>{third}));
    CHECK((replay.unavailable == Ranges{{0, 1}}) && (replay.delivered == Ranges{{2, 2}}));
    const EventReplay second_look = live(log, kGeneration, 3, cache);
    CHECK(subtract_ranges(replay.unavailable, second_look.unavailable).empty());
    CHECK(std::filesystem::exists(room.path / "events" / "seg-000003.log"));
    CHECK(!std::filesystem::exists(room.path / "events" / "seg-000001.log"));
}

TEST(an_evicted_generation_stays_addressable_and_is_not_relabelled) {
    EventLogLimits limits;
    limits.segment_bytes = 1;
    limits.total_bytes = 1;
    Room room;
    EventLog log = EventLog::open(room.path, limits);
    const msg::Bytes old0 = bytes_of(4), old1 = bytes_of(5), fresh = bytes_of(6);
    commit_one(log, kGeneration, 0, 1, old0);
    commit_one(log, kGeneration, 1, 2, old1);
    commit_one(log,  kGeneration + 1, 0, 3, fresh);
    CHECK(log.knows(kGeneration));
    const EventReplay archived = log.replay_archive(kGeneration, std::nullopt).value();
    CHECK(archived.messages.empty() && archived.bound == 1);
    CHECK((archived.unavailable == Ranges{{0, 1}}));
    CHECK(!log.replay_archive(kGeneration + 5, std::nullopt));
    CHECK(!log.replay_archive(0, std::nullopt));
    const EventReplay live_now = log.replay_current(kGeneration + 1, 1, std::nullopt, {}, {});
    CHECK((live_now.messages == std::vector<msg::Bytes>{fresh}) && live_now.bound == 0 && live_now.unavailable.empty());
    const EventReplay none = log.replay_current(kGeneration + 1, 0, std::nullopt, {}, {});
    CHECK(none.messages.empty() && !none.bound);
}

TEST(failed_write_fsync_and_unlink_retry_without_duplicating) {
    Room room;
    TracingIo io;
    EventLog log = EventLog::open(room.path, EventLogLimits{}, &io);
    log.append(kGeneration, 0, 7, 1, bytes_of(1));
    io.fail = "append";
    CHECK_THROWS(IoFault, log.commit());
    io.fail.clear();
    io.fail_seen = 0;
    log.commit();
    CHECK(log.replay_archive(kGeneration, std::nullopt)->messages.size() == 1);

    log.append(kGeneration, 1, 7, 2, bytes_of(2));
    io.fail = "fsync_file";
    CHECK_THROWS(IoFault, log.commit());
    io.fail.clear();
    io.fail_seen = 0;
    log.commit();
    const EventReplay once = log.replay_archive(kGeneration, std::nullopt).value();
    CHECK(once.messages.size() == 2 && once.messages[1] == bytes_of(2));

    Room created;
    TracingIo create_io;
    EventLog creating = EventLog::open(created.path, EventLogLimits{}, &create_io);
    creating.append(kGeneration, 0, 7, 1, bytes_of(3));
    create_io.fail = "fsync_dir";
    create_io.fail_name = "events";
    CHECK_THROWS(IoFault, creating.commit());
    CHECK(std::find(create_io.synced_dirs.begin(), create_io.synced_dirs.end(), created.path) != create_io.synced_dirs.end());
    create_io.fail.clear();
    create_io.fail_name.clear();
    creating.commit();
    CHECK(creating.replay_archive(kGeneration, std::nullopt)->messages.size() == 1);

    EventLogLimits tiny;
    tiny.segment_bytes = 1;
    tiny.total_bytes = 1;
    Room rotate_room;
    TracingIo rotate_io;
    EventLog rotating = EventLog::open(rotate_room.path, tiny, &rotate_io);
    commit_one(rotating, kGeneration, 0, 1, bytes_of(8, 64));
    rotating.append(kGeneration, 1, 7, 2, bytes_of(9, 64));
    rotate_io.fail = "unlink";
    CHECK_THROWS(IoFault, rotating.commit());
    rotate_io.fail.clear();
    rotating.commit();
    const EventReplay kept = rotating.replay_archive(kGeneration, std::nullopt).value();
    CHECK((kept.messages == std::vector<msg::Bytes>{bytes_of(9, 64)}));
    CHECK((kept.unavailable == Ranges{{0, 0}}) && kept.bound == 1);
}

TEST(state_dir_replays_past_the_hundred_event_cache) {
    TempDir dir;
    RoomMemory memory(config_with(dir.path.string()), kStart, kGeneration);
    const scenes::Cloud room = scenes::room_shell();
    std::int64_t second = build_reference(memory, room);
    CHECK(!std::filesystem::exists(dir.path / "lab" / "events"));
    const scenes::Cloud box = scenes::box_shell({0.3, 1.2, 0.3});
    const scenes::Cloud speck = scenes::box_shell({-0.8, -0.8, 0.3});
    const scenes::Cloud boxed = scenes::concat({&room, &box});
    const scenes::Cloud both = scenes::concat({&room, &box, &speck});
    memory.add_points("mid360", boxed);
    std::vector<msg::Bytes> published;
    for (const msg::ChangeEvent& event : memory.close_window(stamp(second))) published.push_back(event.to_cdr());
    CHECK(published.size() == 1);
    std::size_t transitions = 0;
    while (published.size() < kEventLogDepth + 5) {
        ++second;
        memory.add_points("mid360", transitions % 2 ? boxed : both);
        const std::vector<msg::ChangeEvent> events = memory.close_window(stamp(second));
        CHECK(!events.empty());
        transitions += events.size();
        for (const msg::ChangeEvent& event : events) published.push_back(event.to_cdr());
    }
    const RoomMemory::SyncAnswer answer = memory.sync(sync_request(std::nullopt, msg::SyncTopic::events), stamp(second));
    CHECK(answer.messages == published);
    CHECK(answer.end->unavailable.empty());
    CHECK((answer.end->delivered == Ranges{{0, published.size() - 1}}));
    CHECK(answer.end->bound == published.size() - 1);
    CHECK(answer.end->publisher_generation == kGeneration && answer.end->topic == msg::SyncTopic::events);
    CHECK(memory.event_seq() == published.size());
    CHECK(memory.status(stamp(second), 1.0).events_seq == answer.end->bound);
    const msg::ChangeEvent first = msg::ChangeEvent::from_cdr(published[0]);
    const RoomMemory::SyncAnswer since = memory.sync(
        sync_request(msg::RequestSync::Since{first.header.map_epoch, 0}, msg::SyncTopic::events), stamp(second));
    CHECK(since.messages == std::vector<msg::Bytes>(published.begin() + 1, published.end()));
    CHECK((since.end->delivered == Ranges{{1, *answer.end->bound}}) && since.end->unavailable.empty());
    const RoomMemory::SyncAnswer caught_up = memory.sync(
        sync_request(msg::RequestSync::Since{first.header.map_epoch, *answer.end->bound}, msg::SyncTopic::events), stamp(second));
    CHECK(caught_up.messages.empty() && caught_up.end->delivered.empty() && caught_up.end->unavailable.empty());
    CHECK(msg::SyncEnd::from_cdr(answer.end->to_cdr()) == *answer.end);
}

TEST(under_one_hundred_matches_the_memory_only_answer) {
    TempDir dir;
    const scenes::Cloud room = scenes::room_shell();
    const scenes::Cloud box = scenes::box_shell({0.3, 1.2, 0.3});
    const scenes::Cloud boxed = scenes::concat({&room, &box});
    RoomMemory plain(config_with(std::nullopt), kStart, kGeneration);
    RoomMemory durable(config_with(dir.path.string()), kStart, kGeneration);
    const std::int64_t second = build_reference(plain, room);
    build_reference(durable, room);
    plain.add_points("mid360", boxed);
    durable.add_points("mid360", boxed);
    CHECK(plain.close_window(stamp(second)).size() == 1);
    CHECK(durable.close_window(stamp(second)).size() == 1);
    plain.add_points("mid360", room);
    durable.add_points("mid360", room);
    plain.close_window(stamp(second + 1));
    durable.close_window(stamp(second + 1));
    const auto left = plain.sync(sync_request(std::nullopt, msg::SyncTopic::events), stamp(second + 1));
    const auto right = durable.sync(sync_request(std::nullopt, msg::SyncTopic::events), stamp(second + 1));
    CHECK(left.messages == right.messages);
    CHECK(left.end == right.end);
    CHECK((right.end->delivered == Ranges{{0, 1}}) && right.end->unavailable.empty());
}

TEST(restart_serves_the_old_generation_only_through_archive) {
    TempDir dir;
    const auto config = config_with(dir.path.string());
    std::vector<msg::Bytes> published;
    std::uint64_t epoch = 0;
    {
        RoomMemory memory(config, kStart, kGeneration);
        const scenes::Cloud room = scenes::room_shell();
        const std::int64_t second = build_reference(memory, room);
        const scenes::Cloud box = scenes::box_shell({0.3, 1.2, 0.3});
        memory.add_points("mid360", scenes::concat({&room, &box}));
        const auto events = memory.close_window(stamp(second));
        CHECK(events.size() == 1);
        published.push_back(events[0].to_cdr());
        epoch = events[0].header.map_epoch;
        memory.add_points("mid360", room);
        const auto expired = memory.close_window(stamp(second + 1));
        CHECK(expired.size() == 1);
        published.push_back(expired[0].to_cdr());
    }
    RoomMemory restarted(config, stamp(100), kGeneration + 1);
    CHECK(restarted.event_seq() == 0);
    CHECK(!restarted.status(stamp(100), 1.0).events_seq);
    const auto live_old = restarted.sync(
        sync_request(msg::RequestSync::Since{epoch, 0}, msg::SyncTopic::events, kGeneration), stamp(100));
    CHECK(live_old.messages.empty() && live_old.end && !live_old.end->bound);
    CHECK(live_old.end->publisher_generation == kGeneration + 1);
    CHECK(live_old.end->delivered.empty() && live_old.end->unavailable.empty());

    const auto archived = restarted.sync(sync_request(std::nullopt, msg::SyncTopic::event_archive, kGeneration), stamp(100));
    CHECK(archived.messages == published);
    CHECK(archived.end->publisher_generation == kGeneration);
    CHECK(archived.end->topic == msg::SyncTopic::event_archive);
    CHECK(archived.end->bound == 1 && archived.end->unavailable.empty());
    CHECK((archived.end->delivered == Ranges{{0, 1}}));
    CHECK(msg::ChangeEvent::from_cdr(archived.messages[0]).publisher_generation == kGeneration);
    CHECK(msg::ChangeEvent::from_cdr(archived.messages[0]).header.seq == 0);
    const std::uint64_t seq_before = restarted.sync_end_seq();
    const auto unknown = restarted.sync(sync_request(std::nullopt, msg::SyncTopic::event_archive, kGeneration + 9), stamp(100));
    const auto legacy = restarted.sync(sync_request(std::nullopt, msg::SyncTopic::event_archive, std::nullopt), stamp(100));
    const auto zero = restarted.sync(sync_request(std::nullopt, msg::SyncTopic::event_archive, 0), stamp(100));
    CHECK(!unknown.end && !legacy.end && !zero.end);
    CHECK(restarted.sync_end_seq() == seq_before);

    const scenes::Cloud room = scenes::room_shell();
    const scenes::Cloud neu_box = scenes::box_shell({-0.4, 0.4, 0.3});
    restarted.add_points("mid360", scenes::concat({&room, &neu_box}));
    const auto neu = restarted.close_window(stamp(101));
    CHECK(neu.size() == 1 && neu[0].header.seq == 0);
    const auto live_new = restarted.sync(sync_request(std::nullopt, msg::SyncTopic::events, kGeneration + 1), stamp(101));
    CHECK(live_new.messages.size() == 1 && live_new.messages[0] == neu[0].to_cdr());
    CHECK(live_new.end->publisher_generation == kGeneration + 1 && live_new.end->bound == 0);
    const auto still_old =
        restarted.sync(sync_request(std::nullopt, msg::SyncTopic::event_archive, kGeneration), stamp(101));
    CHECK(still_old.messages == published && still_old.end->publisher_generation == kGeneration);
}

TEST(age_keeps_an_open_event_then_a_closed_seq_stays_unavailable) {
    TempDir dir;
    MemoryConfig config = config_with(dir.path.string());
    config.event_segment_bytes = 1;
    config.event_log_bytes = std::uint64_t{8} << 20;
    config.event_log_age_ns = 7 * kDay;
    RoomMemory memory(config, kStart, kGeneration);
    const scenes::Cloud room = scenes::room_shell();
    build_reference(memory, room);
    const scenes::Cloud box = scenes::box_shell({0.3, 1.2, 0.3});
    const scenes::Cloud speck = scenes::box_shell({-0.8, -0.8, 0.3});
    const scenes::Cloud boxed = scenes::concat({&room, &box});
    const scenes::Cloud both = scenes::concat({&room, &box, &speck});
    memory.add_points("mid360", boxed);
    const auto opened = memory.close_window(day_stamp(10));
    CHECK(opened.size() == 1 && opened[0].state == EventState::candidate && opened[0].header.seq == 0);
    memory.add_points("mid360", both);
    const auto speck_open = memory.close_window(day_stamp(1));
    CHECK(speck_open.size() == 1 && speck_open[0].header.seq == 1);
    memory.add_points("mid360", boxed);
    const auto speck_gone = memory.close_window(day_stamp(10));
    CHECK(speck_gone.size() == 1 && speck_gone[0].state == EventState::expired && speck_gone[0].header.seq == 2);
    const auto before = memory.sync(sync_request(std::nullopt, msg::SyncTopic::events), day_stamp(10));
    CHECK(before.messages.size() == 3 && before.end->unavailable.empty());

    const scenes::Cloud other = scenes::box_shell({1.2, -0.6, 0.3});
    memory.add_points("mid360", scenes::concat({&room, &box, &other}));
    const auto jumped = memory.close_window(day_stamp(18));
    CHECK(jumped.size() == 1 && jumped[0].header.seq == 3);
    const auto aged = memory.sync(sync_request(std::nullopt, msg::SyncTopic::events), day_stamp(18));
    CHECK(aged.messages.size() == 2);
    CHECK(msg::ChangeEvent::from_cdr(aged.messages[0]).header.seq == 0);
    CHECK(msg::ChangeEvent::from_cdr(aged.messages[1]).header.seq == 3);
    CHECK((aged.end->unavailable == Ranges{{1, 2}}));
    CHECK(msg::ChangeEvent::from_cdr(aged.messages[0]) == opened[0]);

    memory.add_points("mid360", scenes::concat({&room, &other}));
    const auto box_gone = memory.close_window(day_stamp(19));
    CHECK(!box_gone.empty() && box_gone.back().state == EventState::expired);
    const auto closed = memory.sync(sync_request(std::nullopt, msg::SyncTopic::events), day_stamp(19));
    CHECK(subtract_ranges(Ranges{{0, 0}}, closed.end->unavailable).empty());
    bool served_zero = false;
    for (const msg::Bytes& message : closed.messages) {
        served_zero = served_zero || msg::ChangeEvent::from_cdr(message).header.seq == 0;
    }
    CHECK(!served_zero);
    for (int extra = 0; extra < 4; ++extra) {
        memory.add_points("mid360", extra % 2 ? scenes::concat({&room, &other}) : room);
        memory.close_window(day_stamp(20 + extra));
    }
    const auto rotated = memory.sync(sync_request(std::nullopt, msg::SyncTopic::events), day_stamp(24));
    CHECK(subtract_ranges(closed.end->unavailable, rotated.end->unavailable).empty());
    for (const msg::Bytes& message : rotated.messages) CHECK(msg::ChangeEvent::from_cdr(message).header.seq != 0);
}

TEST(a_failed_commit_is_not_in_the_archive_until_it_lands) {
    TempDir dir;
    TracingIo io;
    MemoryConfig config = config_with(dir.path.string());
    config.event_log_io = &io;
    RoomMemory memory(config, kStart, kGeneration);
    const scenes::Cloud room = scenes::room_shell();
    const std::int64_t second = build_reference(memory, room);
    io.fail = "fsync_file";
    const scenes::Cloud failed_box = scenes::box_shell({0.3, 1.2, 0.3});
    memory.add_points("mid360", scenes::concat({&room, &failed_box}));
    CHECK_THROWS(IoFault, memory.close_window(stamp(second)));
    const auto archived = memory.sync(sync_request(std::nullopt, msg::SyncTopic::event_archive, kGeneration), stamp(second));
    CHECK(!archived.end);
    const auto live = memory.sync(sync_request(std::nullopt, msg::SyncTopic::events), stamp(second));
    CHECK(live.messages.size() == 1 && live.end->bound == 0);
    io.fail.clear();
    memory.close_window(stamp(second + 1));
    const auto landed = memory.sync(sync_request(std::nullopt, msg::SyncTopic::event_archive, kGeneration), stamp(second + 1));
    CHECK(landed.end && landed.messages.size() == 1 && landed.end->publisher_generation == kGeneration);
}

TEST(corrupt_history_refuses_the_room) {
    TempDir dir;
    const auto config = config_with(dir.path.string());
    {
        RoomMemory memory(config, kStart, kGeneration);
        const scenes::Cloud room = scenes::room_shell();
        const std::int64_t second = build_reference(memory, room);
        const scenes::Cloud box = scenes::box_shell({0.3, 1.2, 0.3});
        memory.add_points("mid360", scenes::concat({&room, &box}));
        CHECK(memory.close_window(stamp(second)).size() == 1);
        memory.add_points("mid360", room);
        CHECK(memory.close_window(stamp(second + 1)).size() == 1);
    }
    std::filesystem::path segment;
    for (const auto& entry : std::filesystem::directory_iterator(dir.path / "lab" / "events")) {
        if (entry.path().filename().string().rfind("seg-", 0) == 0) segment = entry.path();
    }
    std::vector<std::uint8_t> data = read_file(segment);
    const auto offsets = record_offsets(data);
    CHECK(offsets.size() >= 2);
    data[static_cast<std::size_t>(offsets[0]) + 12 + 3] ^= 0x11;
    write_file(segment, data);
    CHECK_THROWS(std::runtime_error, RoomMemory(config, stamp(50), kGeneration + 1));
}

TEST(a_fitting_batch_is_one_segment_write) {
    Room room;
    TracingIo io;
    EventLog log = EventLog::open(room.path, EventLogLimits{}, &io);
    log.append(kGeneration, 0, 7, 1, bytes_of(1));
    log.append(kGeneration, 1, 7, 2, bytes_of(2));
    log.commit();
    CHECK(io.appends == 2 && io.file_syncs == 2);
    CHECK(std::filesystem::exists(room.path / "events" / "seg-000001.log"));
    CHECK(!std::filesystem::exists(room.path / "events" / "seg-000002.log"));
    const EventReplay replay = log.replay_archive(kGeneration, std::nullopt).value();
    CHECK((replay.messages == std::vector<msg::Bytes>{bytes_of(1), bytes_of(2)}));
}

TEST(a_batch_over_the_total_cap_drops_a_prefix_and_reopens) {
    std::uint64_t one = 0;
    {
        Room probe;
        EventLog log = EventLog::open(probe.path, EventLogLimits{});
        commit_one(log, kGeneration, 0, 1, bytes_of(1));
        one = std::filesystem::file_size(only_segment(probe.path));
    }
    EventLogLimits limits;
    limits.segment_bytes = 1;
    limits.total_bytes = one * 2;
    Room room;
    EventLog log = EventLog::open(room.path, limits);
    const msg::Bytes cdrs[4] = {bytes_of(1), bytes_of(2), bytes_of(3), bytes_of(4)};
    for (int n = 0; n < 4; ++n) log.append(kGeneration, static_cast<std::uint64_t>(n), 7, n + 1, cdrs[n]);
    log.commit();
    CHECK(!std::filesystem::exists(room.path / "events" / "seg-000001.log"));
    CHECK(!std::filesystem::exists(room.path / "events" / "seg-000002.log"));
    CHECK(std::filesystem::exists(room.path / "events" / "seg-000003.log"));
    CHECK(std::filesystem::exists(room.path / "events" / "seg-000004.log"));
    const EventReplay replay = log.replay_archive(kGeneration, std::nullopt).value();
    CHECK((replay.messages == std::vector<msg::Bytes>{cdrs[2], cdrs[3]}));
    CHECK((replay.unavailable == Ranges{{0, 1}}) && replay.bound == 3);
    EventLog again = EventLog::open(room.path, limits);
    CHECK(again.replay_archive(kGeneration, std::nullopt)->messages == replay.messages);
}

TEST(maximum_records_in_one_commit_stay_under_the_recovery_ceiling) {
    // Default limits make 34 maximum records one 35,651,992-byte segment, over the
    // 34,607,116-byte recovery ceiling, and retention cannot unlink the only segment.
    // The same inequality with total_bytes == 1 is two maximum records.
    EventLogLimits limits;
    limits.segment_bytes = 1;
    limits.total_bytes = 1;
    const std::uint64_t ceiling =
        limits.total_bytes + static_cast<std::uint64_t>(kEventMaxPayload) + kEventHeaderBytes + 4096;
    const std::uint64_t record = static_cast<std::uint64_t>(kEventHeaderBytes) + kEventMaxPayload;
    CHECK(record * 2 > ceiling);
    const std::size_t payload = static_cast<std::size_t>(kEventMaxPayload) - kEventFixedBytes;
    const msg::Bytes first = bytes_of(1, payload);
    const msg::Bytes second = bytes_of(2, payload);
    Room room;
    {
        EventLog log = EventLog::open(room.path, limits);
        log.append(kGeneration, 0, 7, 1, first);
        log.append(kGeneration, 1, 7, 2, second);
        log.commit();
    }
    const std::filesystem::path kept = room.path / "events" / "seg-000002.log";
    CHECK(!std::filesystem::exists(room.path / "events" / "seg-000001.log"));
    CHECK(std::filesystem::exists(kept));
    CHECK(std::filesystem::file_size(kept) <= ceiling);
    CHECK(std::filesystem::file_size(kept) == record);
    EventLog again = EventLog::open(room.path, limits);
    const EventReplay replay = again.replay_archive(kGeneration, std::nullopt).value();
    CHECK((replay.messages == std::vector<msg::Bytes>{second}));
    CHECK(replay.bound == 1 && (replay.unavailable == Ranges{{0, 0}}));
}

TEST(a_split_batch_retries_the_unfinished_suffix_without_duplicating) {
    EventLogLimits limits;
    limits.segment_bytes = 1;
    Room room;
    TracingIo io;
    EventLog log = EventLog::open(room.path, limits, &io);
    log.append(kGeneration, 0, 7, 1, bytes_of(1));
    log.append(kGeneration, 1, 7, 2, bytes_of(2));
    io.fail = "fsync_file";
    CHECK_THROWS(IoFault, log.commit());
    io.fail.clear();
    io.fail_seen = 0;
    log.commit();
    const EventReplay replay = log.replay_archive(kGeneration, std::nullopt).value();
    CHECK((replay.messages == std::vector<msg::Bytes>{bytes_of(1), bytes_of(2)}));
    CHECK(replay.bound == 1 && replay.unavailable.empty());
    EventLog again = EventLog::open(room.path, limits);
    CHECK(again.replay_archive(kGeneration, std::nullopt)->messages == replay.messages);
}

TEST(recovery_fsyncs_unsynced_records_before_the_manifest) {
    Room room;
    CrashIo io(room.path);
    const std::filesystem::path segment = room.path / "events" / "seg-000001.log";
    const std::filesystem::path manifest = room.path / "events" / "manifest.log";
    {
        EventLog log = EventLog::open(room.path, EventLogLimits{}, &io);
        log.append(kGeneration, 0, 7, 1, bytes_of(1));
        log.append(kGeneration, 1, 7, 2, bytes_of(2));
        io.crash_next_file_sync = true;
        CHECK_THROWS(Crash, log.commit());
    }
    CHECK(!io.durable_bytes(segment));
    CHECK(io.visible_bytes(segment).value_or(0) > kEventHeaderBytes);
    io.dead = false;
    io.crash_next_file_sync = false;
    io.notes.clear();
    {
        EventLog log = EventLog::open(room.path, EventLogLimits{}, &io);
        require_durable_before_manifest(io, "seg-000001.log");
        const EventReplay replay = log.replay_archive(kGeneration, std::nullopt).value();
        CHECK((replay.messages == std::vector<msg::Bytes>{bytes_of(1), bytes_of(2)}));
    }
    const auto durable = io.durable_bytes(segment);
    const auto visible = io.visible_bytes(segment);
    CHECK(durable && visible && *durable == *visible && *durable > 0);
    CHECK(io.durable_bytes(manifest).value_or(0) > 0);
    io.power_loss();
    EventLog again = EventLog::open(room.path, EventLogLimits{}, &io);
    const EventReplay replay = again.replay_archive(kGeneration, std::nullopt).value();
    CHECK((replay.messages == std::vector<msg::Bytes>{bytes_of(1), bytes_of(2)}));
    CHECK(replay.bound == 1 && replay.unavailable.empty());
}

TEST(recovery_fsyncs_a_new_segment_before_an_existing_manifest) {
    EventLogLimits limits;
    limits.segment_bytes = 1;
    Room room;
    CrashIo io(room.path);
    {
        EventLog log = EventLog::open(room.path, limits, &io);
        commit_one(log, kGeneration, 0, 1, bytes_of(1));
    }
    io.power_loss();
    {
        EventLog log = EventLog::open(room.path, limits, &io);
        CHECK(log.replay_archive(kGeneration, std::nullopt)->messages == std::vector<msg::Bytes>{bytes_of(1)});
        log.append(kGeneration, 1, 7, 2, bytes_of(2));
        io.crash_next_file_sync = true;
        CHECK_THROWS(Crash, log.commit());
    }
    const std::filesystem::path segment = room.path / "events" / "seg-000002.log";
    CHECK(!io.durable_bytes(segment));
    CHECK(io.visible_bytes(segment).value_or(0) > kEventHeaderBytes);
    io.dead = false;
    io.crash_next_file_sync = false;
    io.notes.clear();
    {
        EventLog log = EventLog::open(room.path, limits, &io);
        require_durable_before_manifest(io, "seg-000002.log");
        CHECK(log.replay_archive(kGeneration, std::nullopt)->messages.size() == 2);
    }
    const auto rolled = io.durable_bytes(segment);
    const auto rolled_visible = io.visible_bytes(segment);
    CHECK(rolled && rolled_visible && *rolled == *rolled_visible && *rolled > 0);
    io.power_loss();
    EventLog again = EventLog::open(room.path, limits, &io);
    const EventReplay replay = again.replay_archive(kGeneration, std::nullopt).value();
    CHECK((replay.messages == std::vector<msg::Bytes>{bytes_of(1), bytes_of(2)}));
    CHECK(replay.bound == 1 && replay.unavailable.empty());
}

TEST(a_reused_generation_is_refused_before_it_is_written) {
    Room room;
    EventLog log = EventLog::open(room.path, EventLogLimits{});
    commit_one(log, kGeneration, 4, 10, bytes_of(1));
    log.append(kGeneration, 0, 7, 11, bytes_of(2));
    CHECK_THROWS(std::runtime_error, log.commit());
    EventLog again = EventLog::open(room.path, EventLogLimits{});
    const EventReplay kept = again.replay_archive(kGeneration, std::nullopt).value();
    CHECK(kept.messages.size() == 1 && kept.bound == 4);
    again.append(kGeneration + 1, 0, 7, 12, bytes_of(3));
    again.commit();
    CHECK(again.replay_archive(kGeneration, std::nullopt)->bound == 4);
    CHECK(again.replay_archive(kGeneration + 1, std::nullopt)->messages.size() == 1);
}

TEST(a_second_process_allocates_instead_of_reusing_the_generation) {
    TempDir dir;
    const auto config = config_with(dir.path.string());
    const std::int64_t now_ms = static_cast<std::int64_t>(kGeneration);
    std::vector<msg::Bytes> published;
    std::uint64_t first_generation = 0;
    {
        const Allocation held = allocate_generation(config.state_dir, "lab", now_ms);
        first_generation = held.generation();
        CHECK_EQ(first_generation, kGeneration);
        RoomMemory memory(config, kStart, first_generation);
        const scenes::Cloud room = scenes::room_shell();
        const std::int64_t second = build_reference(memory, room);
        const scenes::Cloud box = scenes::box_shell({0.3, 1.2, 0.3});
        memory.add_points("mid360", scenes::concat({&room, &box}));
        const auto events = memory.close_window(stamp(second));
        CHECK(events.size() == 1 && events[0].header.seq == 0);
        published.push_back(events[0].to_cdr());
        memory.add_points("mid360", room);
        const auto expired = memory.close_window(stamp(second + 1));
        CHECK(expired.size() == 1 && expired[0].header.seq == 1);
        published.push_back(expired[0].to_cdr());
    }
    {
        RoomMemory reused(config, stamp(50), first_generation);
        const scenes::Cloud room = scenes::room_shell();
        const scenes::Cloud box = scenes::box_shell({0.3, 1.2, 0.3});
        reused.add_points("mid360", scenes::concat({&room, &box}));
        bool refused = false;
        try {
            reused.close_window(stamp(50));
        } catch (const std::runtime_error& error) {
            refused = std::string(error.what()).find("manifest high water") != std::string::npos;
        }
        CHECK(refused);
    }
    const Allocation again = allocate_generation(config.state_dir, "lab", now_ms);
    CHECK_EQ(again.generation(), first_generation + 1);
    RoomMemory restarted(config, stamp(60), again.generation());
    const auto archived =
        restarted.sync(sync_request(std::nullopt, msg::SyncTopic::event_archive, first_generation), stamp(60));
    CHECK(archived.messages == published);
    CHECK(archived.end && archived.end->publisher_generation == first_generation && archived.end->bound == 1);
    const scenes::Cloud room = scenes::room_shell();
    const scenes::Cloud other = scenes::box_shell({-0.4, 0.4, 0.3});
    restarted.add_points("mid360", scenes::concat({&room, &other}));
    const auto neu = restarted.close_window(stamp(60));
    CHECK(neu.size() == 1 && neu[0].header.seq == 0 && neu[0].publisher_generation == again.generation());
    const auto live = restarted.sync(sync_request(std::nullopt, msg::SyncTopic::events, again.generation()), stamp(60));
    CHECK(live.messages.size() == 1 && live.end->bound == 0 && live.end->publisher_generation == again.generation());
    const auto still =
        restarted.sync(sync_request(std::nullopt, msg::SyncTopic::event_archive, first_generation), stamp(60));
    CHECK(still.messages == published && still.end->bound == 1);
}

TEST_MAIN
