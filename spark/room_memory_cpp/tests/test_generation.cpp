// Durable publisher generation allocation (protocol 1.4), ported from
// spark/room_memory/tests/test_generation.py.
#include <sys/stat.h>
#include <unistd.h>

#include <cstdlib>
#include <fstream>
#include <vector>

#include "harness.hpp"
#include "room_memory/generation.hpp"

using namespace room_memory;

namespace {

constexpr std::int64_t kNow = 1760400000000;

// A fresh directory under the system temp dir, removed when the test ends.
struct TempDir {
    std::filesystem::path path;
    TempDir() {
        std::string pattern = (std::filesystem::temp_directory_path() / "room-memory-generation-XXXXXX").string();
        if (!mkdtemp(pattern.data())) throw harness::Failure("mkdtemp failed");
        path = pattern;
    }
    ~TempDir() {
        std::error_code ignored;
        std::filesystem::permissions(path / "lab", std::filesystem::perms::owner_all, ignored);
        std::filesystem::remove_all(path, ignored);
    }
    std::string str() const { return path.string(); }
};

void write_file(const std::filesystem::path& path, const std::string& text) {
    std::ofstream out(path, std::ios::binary);
    out << text;
}

// Like CHECK_THROWS(GenerationError, ...), and the message contains `text`.
#define CHECK_REFUSED(text, expression)                                                                    \
    do {                                                                                                   \
        bool thrown = false;                                                                               \
        try {                                                                                              \
            (void)(expression);                                                                            \
        } catch (const GenerationError& error) {                                                           \
            thrown = std::string(error.what()).find(text) != std::string::npos;                            \
        }                                                                                                  \
        if (!thrown) {                                                                                     \
            throw harness::Failure(harness::location(__FILE__, __LINE__) + ": no GenerationError matching '" \
                                   text "' from " #expression);                                            \
        }                                                                                                  \
    } while (0)

}  // namespace

TEST(first_start_takes_the_wall_clock) {
    TempDir dir;
    const Allocation allocation = allocate_generation(dir.str(), "lab", kNow);
    CHECK_EQ(allocation.generation(), std::uint64_t{kNow});
    CHECK(allocation.held() && allocation.path() == generation_path(dir.str(), "lab"));
    CHECK_EQ(harness::read_text(generation_path(dir.str(), "lab")), std::to_string(kNow) + "\n");
    CHECK(!std::filesystem::exists(generation_path(dir.str(), "lab").string() + ".tmp"));
}

TEST(every_start_rises_even_if_the_clock_goes_back) {
    TempDir dir;
    std::vector<std::uint64_t> generations;
    for (const std::int64_t now : {kNow, kNow, kNow - 5000, kNow + 10}) {
        generations.push_back(allocate_generation(dir.str(), "lab", now).generation());
    }
    const std::uint64_t now = kNow;
    CHECK((generations == std::vector<std::uint64_t>{now, now + 1, now + 2, now + 10}));
}

TEST(wall_clock_floor_survives_a_lost_file) {
    TempDir dir;
    allocate_generation(dir.str(), "lab", kNow).release();
    std::filesystem::remove(generation_path(dir.str(), "lab"));
    CHECK_EQ(allocate_generation(dir.str(), "lab", kNow + 60'000).generation(), std::uint64_t{kNow + 60'000});
}

TEST(state_dir_must_exist_and_is_never_created) {
    TempDir dir;
    CHECK_REFUSED("empty", allocate_generation(std::string(), "lab", kNow));
    CHECK_REFUSED("empty", allocate_generation(std::nullopt, "lab", kNow));
    const std::filesystem::path missing = dir.path / "room-memory";
    CHECK_REFUSED("does not exist", allocate_generation(missing.string(), "lab", kNow));
    CHECK(!std::filesystem::exists(missing));
    CHECK_REFUSED("plain file name", allocate_generation(dir.str(), "../lab", kNow));
}

TEST(malformed_file_is_never_reset) {
    for (const std::string data : {"", "12", "-3\n", " 3\n", "3\n\n", "18446744073709551616\n", "abc\n"}) {
        TempDir dir;
        const std::filesystem::path path = generation_path(dir.str(), "lab");
        std::filesystem::create_directory(path.parent_path());
        write_file(path, data);
        CHECK_REFUSED("refusing to reset", allocate_generation(dir.str(), "lab", kNow));
        CHECK(harness::read_text(path) == data);
    }
}

// Python's now_ms of 2^64 has no std::int64_t value.
TEST(overflow_is_refused) {
    TempDir dir;
    const std::filesystem::path path = generation_path(dir.str(), "lab");
    std::filesystem::create_directory(path.parent_path());
    write_file(path, "18446744073709551615\n");
    CHECK_EQ(load_generation(path), ~std::uint64_t{0});
    CHECK_REFUSED("overflow", allocate_generation(dir.str(), "lab", kNow));
    CHECK(harness::read_text(path) == "18446744073709551615\n");
}

TEST(a_live_lock_holder_blocks_a_second_publisher) {
    TempDir dir;
    {
        Allocation first = allocate_generation(dir.str(), "lab", kNow);
        CHECK_REFUSED("another process", allocate_generation(dir.str(), "lab", kNow));
        CHECK_EQ(load_generation(generation_path(dir.str(), "lab")), std::uint64_t{kNow});  // the refused start wrote nothing
        allocate_generation(dir.str(), "garage", kNow);  // one lock per room
        // A moved allocation keeps the lock; the moved-from one holds nothing.
        Allocation moved = std::move(first);
        CHECK(moved.held() && !first.held());
        first.release();
        CHECK_REFUSED("another process", allocate_generation(dir.str(), "lab", kNow));
    }
    CHECK_EQ(allocate_generation(dir.str(), "lab", kNow).generation(), std::uint64_t{kNow + 1});
}

TEST(unwritable_room_dir_fails_closed) {
    TempDir dir;
    const std::filesystem::path room = dir.path / "lab";
    std::filesystem::create_directory(room);
    write_file(room / kGenerationLockFile, "");
    ::chmod(room.c_str(), 0500);
    const bool writable = ::access(room.c_str(), W_OK) == 0;
    if (!writable) CHECK_REFUSED("durably", allocate_generation(dir.str(), "lab", kNow));
    ::chmod(room.c_str(), 0700);
    if (writable) throw harness::Skip("running as a user that ignores directory permissions");
    CHECK(!std::filesystem::exists(generation_path(dir.str(), "lab")));
}

TEST(write_failure_releases_the_lock) {
    TempDir dir;
    const std::filesystem::path scratch = generation_path(dir.str(), "lab").string() + ".tmp";
    std::filesystem::create_directories(scratch);  // the temp file cannot be opened
    CHECK_REFUSED("durably", allocate_generation(dir.str(), "lab", kNow));
    std::filesystem::remove(scratch);
    CHECK_EQ(allocate_generation(dir.str(), "lab", kNow).generation(), std::uint64_t{kNow});  // not blocked by a leaked lock
}

TEST_MAIN
