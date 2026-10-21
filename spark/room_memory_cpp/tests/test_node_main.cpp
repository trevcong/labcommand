// The node's main (room_memory::run_node) without rclcpp: each way out returns the exit code the
// node binary would, logs the FATAL line it would, and shuts down once (#94, #108).
#include <unistd.h>

#include <filesystem>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "harness.hpp"
#include "room_memory/generation.hpp"
#include "room_memory/memory.hpp"
#include "room_memory/node_main.hpp"
#include "scenes.hpp"

namespace {

using room_memory::MemoryConfig;
using room_memory::RoomMemory;
namespace msg = room_memory::msg;

const msg::Time kStart{1760400000, 0};
constexpr std::int64_t kNowMs = 1760400000000LL;

struct TempDir {
    std::filesystem::path path;
    TempDir() {
        std::string pattern = (std::filesystem::temp_directory_path() / "room-memory-test-XXXXXX").string();
        if (!mkdtemp(pattern.data())) throw harness::Failure("mkdtemp failed");
        path = pattern;
    }
    ~TempDir() { std::filesystem::remove_all(path); }
};

MemoryConfig config_in(const std::filesystem::path& state_dir) {
    MemoryConfig config;
    config.sensors = {"mid360"};
    config.max_range = std::nullopt;
    config.reference_windows = 3;
    config.state_dir = state_dir.string();
    return config;
}

// RoomMemoryNode's construction order: the generation (and its lock) first, then RoomMemory.
struct Node {
    explicit Node(const MemoryConfig& config)
        : generation(room_memory::allocate_generation(config.state_dir, config.room_id, kNowMs)),
          memory(config, kStart, generation.generation()) {}
    room_memory::Allocation generation;
    RoomMemory memory;
};

// What main's hooks saw, in order: "spin", "fatal: <line>", "shutdown".
struct Calls {
    std::vector<std::string> seen;
    // Whether a new generation could be allocated at shutdown, i.e. the node's lock was gone.
    bool lock_free_at_shutdown = false;
};

int run(const MemoryConfig& config, Calls& calls, bool throw_in_spin = false) {
    return room_memory::run_node(
        [&] { return std::make_unique<Node>(config); },
        [&](const std::unique_ptr<Node>&) {
            calls.seen.push_back("spin");
            if (throw_in_spin) throw std::runtime_error("window failed");
        },
        [&] {
            calls.seen.push_back("shutdown");
            try {
                room_memory::allocate_generation(config.state_dir, config.room_id, kNowMs);
                calls.lock_free_at_shutdown = true;
            } catch (const room_memory::GenerationError&) {
            }
        },
        [&](const std::string& line) { calls.seen.push_back("fatal: " + line); });
}

bool starts_with(const std::string& text, const std::string& prefix) { return text.rfind(prefix, 0) == 0; }

}  // namespace

// #94: a construction failure that is not a GenerationError (a truncated state file) exits 1
// with one FATAL line naming the file, not std::terminate.
TEST(truncated_state_exits_1_with_a_fatal_line) {
    for (const std::string file : {"map_epoch", "reference-mid360.cdr"}) {
        TempDir dir;
        const MemoryConfig config = config_in(dir.path);
        {  // builds and saves a reference, then releases the lock
            Node first(config);
            for (std::int64_t second = 0; second < config.reference_windows; ++second) {
                first.memory.add_points("mid360", scenes::room_shell());
                first.memory.close_window(msg::Time{kStart.sec + second, 0});
            }
        }
        const auto bad = dir.path / "lab" / file;
        CHECK(std::filesystem::exists(bad));
        std::filesystem::resize_file(bad, 1);  // missing epoch newline / incomplete CDR header
        CHECK_THROWS(std::invalid_argument, Node(config));
        Calls calls;
        CHECK_EQ(run(config, calls), 1);
        CHECK_EQ(calls.seen.size(), std::size_t{2});
        CHECK(starts_with(calls.seen[0], std::string("fatal: ") + room_memory::kNodeStoppedPrefix));
        CHECK(calls.seen[0].find(bad.string()) != std::string::npos);
        CHECK_EQ(calls.seen[1], std::string("shutdown"));
        CHECK(calls.lock_free_at_shutdown);
    }
}

// A throw out of spin (any callback) after a good start takes the same way out, and the node,
// with its generation lock, is gone before shutdown.
TEST(throw_during_spin_exits_1_and_releases_the_node_first) {
    TempDir dir;
    const MemoryConfig config = config_in(dir.path);
    Calls calls;
    CHECK_EQ(run(config, calls, true), 1);
    CHECK_EQ(calls.seen.size(), std::size_t{3});
    CHECK_EQ(calls.seen[0], std::string("spin"));
    CHECK_EQ(calls.seen[1], std::string("fatal: ") + room_memory::kNodeStoppedPrefix + "window failed");
    CHECK_EQ(calls.seen[2], std::string("shutdown"));
    CHECK(calls.lock_free_at_shutdown);
}

// A GenerationError is logged fatal by the constructor: exit 1, no second FATAL line.
TEST(generation_error_exits_1_without_a_second_fatal_line) {
    TempDir dir;
    const MemoryConfig config = config_in(dir.path / "missing");
    Calls calls;
    CHECK_EQ(run(config, calls), 1);
    CHECK_EQ(calls.seen.size(), std::size_t{1});
    CHECK_EQ(calls.seen[0], std::string("shutdown"));
}

TEST(spin_returning_exits_0) {
    TempDir dir;
    const MemoryConfig config = config_in(dir.path);
    Calls calls;
    CHECK_EQ(run(config, calls), 0);
    CHECK_EQ(calls.seen.size(), std::size_t{2});
    CHECK_EQ(calls.seen[0], std::string("spin"));
    CHECK_EQ(calls.seen[1], std::string("shutdown"));
}

TEST_MAIN
