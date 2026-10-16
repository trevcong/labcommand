// Durable publisher generation (protocol 1.4): the C++ port of room_memory/generation.py. No ROS
// dependency.
//
// Every node start takes g = max(stored + 1, now_ms), writes it to
// <state_dir>/<room_id>/publisher_generation (decimal ASCII and a newline, next to map_epoch)
// with a temp file, fsync, rename and a directory fsync, and only then publishes. The wall-clock
// floor keeps generations rising if the file is lost; the residual is a lost file *and* a clock
// that went back past the old value. The allocator fails closed: it never creates state_dir,
// never resets a malformed file, and holds publisher.lock (flock) for the life of the process,
// so one process publishes per room.
#pragma once

#include <cstdint>
#include <filesystem>
#include <optional>
#include <stdexcept>
#include <string>

namespace room_memory {

inline constexpr const char* kGenerationFile = "publisher_generation";
inline constexpr const char* kGenerationLockFile = "publisher.lock";

// The node must not start: no durable, exclusive generation could be allocated.
class GenerationError : public std::runtime_error {
public:
    using std::runtime_error::runtime_error;
};

// An allocated generation and the open lock file that keeps it exclusive. Keep it for the life of
// the process: destroying it (or release()) closes the lock file, which releases the flock.
class Allocation {
public:
    Allocation(std::uint64_t generation, std::filesystem::path path, int lock_fd);
    Allocation(Allocation&& other) noexcept;
    Allocation& operator=(Allocation&& other) noexcept;
    Allocation(const Allocation&) = delete;
    Allocation& operator=(const Allocation&) = delete;
    ~Allocation();

    std::uint64_t generation() const { return generation_; }
    const std::filesystem::path& path() const { return path_; }
    // Whether the lock is still held (not released or moved from).
    bool held() const { return lock_ >= 0; }
    void release();

private:
    std::uint64_t generation_ = 0;
    std::filesystem::path path_;
    int lock_ = -1;
};

// <state_dir>/<room_id>/publisher_generation. Same room_id rule as reference_path.
std::filesystem::path generation_path(const std::string& state_dir, const std::string& room_id);

// The stored generation, 0 if the file does not exist. Throws GenerationError unless it holds one
// uint64 as digits and a newline.
std::uint64_t load_generation(const std::filesystem::path& path);

// Allocates this process's generation: max(stored + 1, now_ms), durable before it returns. now_ms
// is the wall clock in ms since the Unix epoch (a parameter, for tests). Throws GenerationError
// when state_dir is empty or missing, the room name is not a plain file name, another process
// holds the lock, the file is malformed, the value would overflow, or the write or fsync fails.
Allocation allocate_generation(const std::optional<std::string>& state_dir, const std::string& room_id,
                               std::int64_t now_ms);

}  // namespace room_memory
