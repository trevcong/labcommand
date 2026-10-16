#include "room_memory/generation.hpp"

#include <fcntl.h>
#include <sys/file.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cstring>
#include <fstream>
#include <iterator>
#include <span>
#include <utility>

#include "room_memory/memory.hpp"
#include "room_memory/messages.hpp"

namespace room_memory {

namespace {

// An open lock file descriptor, closed unless taken.
class LockFd {
public:
    explicit LockFd(int fd) : fd_(fd) {}
    LockFd(const LockFd&) = delete;
    LockFd& operator=(const LockFd&) = delete;
    ~LockFd() {
        if (fd_ >= 0) ::close(fd_);
    }
    int get() const { return fd_; }
    int take() { return std::exchange(fd_, -1); }

private:
    int fd_;
};

}  // namespace

Allocation::Allocation(std::uint64_t generation, std::filesystem::path path, int lock_fd)
    : generation_(generation), path_(std::move(path)), lock_(lock_fd) {}

Allocation::Allocation(Allocation&& other) noexcept
    : generation_(other.generation_), path_(std::move(other.path_)), lock_(std::exchange(other.lock_, -1)) {}

Allocation& Allocation::operator=(Allocation&& other) noexcept {
    if (this != &other) {
        release();
        generation_ = other.generation_;
        path_ = std::move(other.path_);
        lock_ = std::exchange(other.lock_, -1);
    }
    return *this;
}

Allocation::~Allocation() { release(); }

void Allocation::release() {
    if (lock_ >= 0) ::close(std::exchange(lock_, -1));
}

std::filesystem::path generation_path(const std::string& state_dir, const std::string& room_id) {
    return room_dir(state_dir, room_id) / kGenerationFile;
}

std::uint64_t load_generation(const std::filesystem::path& path) {
    if (!std::filesystem::exists(path)) return 0;
    std::ifstream in(path, std::ios::binary);
    if (!in) throw GenerationError("cannot read " + path.string());
    const std::string text((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    const auto malformed = [&] {
        return GenerationError(path.string() + " does not hold one publisher_generation; refusing to reset it");
    };
    if (text.size() < 2 || text.size() > 21 || text.back() != '\n') throw malformed();
    std::uint64_t value = 0;
    for (std::size_t n = 0; n + 1 < text.size(); ++n) {
        const char c = text[n];
        if (c < '0' || c > '9') throw malformed();
        const auto digit = static_cast<std::uint64_t>(c - '0');
        if (value > (msg::kUint64Max - digit) / 10) throw malformed();
        value = value * 10 + digit;
    }
    return value;
}

Allocation allocate_generation(const std::optional<std::string>& state_dir, const std::string& room_id,
                               std::int64_t now_ms) {
    if (!state_dir || state_dir->empty()) {
        throw GenerationError("state_dir is empty: a publisher generation needs durable storage");
    }
    const std::filesystem::path base = expand_user(*state_dir);
    std::error_code error;
    if (!std::filesystem::is_directory(base, error)) {
        throw GenerationError("state_dir " + base.string() +
                              " does not exist (is the volume mounted?); it is never created here");
    }
    std::filesystem::path room;
    try {
        room = room_dir(*state_dir, room_id);
    } catch (const std::invalid_argument& invalid) {
        throw GenerationError(invalid.what());
    }
    const std::filesystem::path lock_path = room / kGenerationLockFile;
    // Only the room directory, never state_dir itself.
    std::filesystem::create_directory(room, error);
    const int fd = ::open(lock_path.c_str(), O_WRONLY | O_CREAT | O_APPEND | O_CLOEXEC, 0666);
    if (fd < 0) throw GenerationError("cannot open " + lock_path.string() + ": " + std::strerror(errno));
    // Owns the lock from here on: every failure below closes it, the result takes it over.
    LockFd lock(fd);
    if (::flock(lock.get(), LOCK_EX | LOCK_NB) != 0) {
        throw GenerationError("another process holds " + lock_path.string() + " (" + std::strerror(errno) + ")");
    }
    const std::filesystem::path path = room / kGenerationFile;
    const std::uint64_t stored = load_generation(path);
    if (stored >= msg::kUint64Max) {
        throw GenerationError(path.string() + " holds " + std::to_string(stored) +
                              ": the next generation would overflow uint64");
    }
    const std::uint64_t generation = std::max(stored + 1, now_ms > 0 ? static_cast<std::uint64_t>(now_ms) : 0);
    const std::string text = std::to_string(generation) + "\n";
    try {
        write_atomic(path, std::span(reinterpret_cast<const std::uint8_t*>(text.data()), text.size()), false);
    } catch (const std::runtime_error& failure) {
        throw GenerationError("cannot write " + path.string() + " durably: " + failure.what());
    }
    return Allocation(generation, path, lock.take());
}

}  // namespace room_memory
