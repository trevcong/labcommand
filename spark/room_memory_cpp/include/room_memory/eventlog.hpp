#pragma once

// Durable ChangeEvent log (protocol D45). Append-only segments under
// <state_dir>/<room_id>/events, fsynced before close_window returns. The in-memory
// deque stays a cache of the tail. RequestSync for the live generation reads past
// that cache; TOPIC_EVENT_ARCHIVE reads one generation's original bytes after a restart.

#include <cstdint>
#include <filesystem>
#include <map>
#include <memory>
#include <optional>
#include <span>
#include <vector>

#include "room_memory/messages.hpp"

namespace room_memory {

inline constexpr std::uint32_t kEventRecordMagic = 0x314C5645;    // 'EVL1' little-endian
inline constexpr std::uint32_t kEventManifestMagic = 0x314E414D;  // 'MAN1'
inline constexpr std::uint32_t kEventHeaderBytes = 12;
inline constexpr std::uint32_t kEventFixedBytes = 44;  // generation, seq, epoch, stamp, retain, cdr_len
inline constexpr std::uint32_t kEventMaxPayload = 1u << 20;

// POSIX file operations behind the log, so tests can count and fail them.
// A null EventLogIo on open uses the real filesystem. The pointer must outlive the log.
class EventLogIo {
public:
    virtual ~EventLogIo() = default;
    virtual bool exists(const std::filesystem::path& path) = 0;
    virtual void create_directory(const std::filesystem::path& path) = 0;
    virtual std::vector<std::filesystem::path> list(const std::filesystem::path& dir) = 0;
    virtual std::uint64_t file_size(const std::filesystem::path& path) = 0;
    virtual void read_at(const std::filesystem::path& path, std::uint64_t offset, std::span<std::uint8_t> dest) = 0;
    virtual void append(const std::filesystem::path& path, std::span<const std::uint8_t> data) = 0;
    virtual void fsync_file(const std::filesystem::path& path) = 0;
    virtual void fsync_dir(const std::filesystem::path& path) = 0;
    virtual void truncate(const std::filesystem::path& path, std::uint64_t size) = 0;
    virtual void remove_file(const std::filesystem::path& path) = 0;
    virtual void write_atomic(const std::filesystem::path& path, std::span<const std::uint8_t> data) = 0;
};

class PosixEventLogIo : public EventLogIo {
public:
    bool exists(const std::filesystem::path& path) override;
    void create_directory(const std::filesystem::path& path) override;
    std::vector<std::filesystem::path> list(const std::filesystem::path& dir) override;
    std::uint64_t file_size(const std::filesystem::path& path) override;
    void read_at(const std::filesystem::path& path, std::uint64_t offset, std::span<std::uint8_t> dest) override;
    void append(const std::filesystem::path& path, std::span<const std::uint8_t> data) override;
    void fsync_file(const std::filesystem::path& path) override;
    void fsync_dir(const std::filesystem::path& path) override;
    void truncate(const std::filesystem::path& path, std::uint64_t size) override;
    void remove_file(const std::filesystem::path& path) override;
    void write_atomic(const std::filesystem::path& path, std::span<const std::uint8_t> data) override;
};

struct EventLogLimits {
    std::uint64_t total_bytes = std::uint64_t{32} << 20;
    std::uint64_t segment_bytes = std::uint64_t{4} << 20;
    std::int64_t age_ns = 7LL * 24 * 60 * 60 * 1'000'000'000LL;
};

// A seq dropped from the retained index. The deque must forget it; open_latest_ may keep it.
struct EvictedEvent {
    std::uint64_t generation = 0;
    std::uint64_t seq = 0;
};

// One cached message. `bytes` is not copied and must outlive the replay call.
struct EventPiece {
    std::uint64_t seq = 0;
    std::uint64_t map_epoch = 0;
    const msg::Bytes* bytes = nullptr;
};

struct EventReplay {
    std::vector<msg::Bytes> messages;
    std::optional<std::uint64_t> bound;
    std::vector<msg::SeqRange> delivered;
    std::vector<msg::SeqRange> unavailable;
};

class EventLog {
public:
    EventLog(EventLog&&) noexcept = default;
    EventLog& operator=(EventLog&&) noexcept = default;
    EventLog(const EventLog&) = delete;
    EventLog& operator=(const EventLog&) = delete;

    // Scans segments. A torn tail is truncated and fsynced; a bad record with a valid
    // record after it throws std::runtime_error. Absent events/ is an empty log.
    // `io` null uses POSIX and is owned here. Throws std::invalid_argument on bad limits.
    static EventLog open(std::filesystem::path room, EventLogLimits limits, EventLogIo* io = nullptr);

    // Copies one original CDR into the commit buffer. Throws if the record would pass 1 MiB.
    void append(std::uint64_t generation, std::uint64_t seq, std::uint64_t map_epoch, std::int64_t stamp_ns,
                msg::Bytes cdr);

    // Writes, fsyncs, then drops a prefix for age and size. No I/O when nothing is buffered
    // and nothing was left half-rotated. A seq that is not the next seq of a generation the
    // log already knows throws before the write, so a second process cannot append seq 0
    // onto a generation it does not own. Throws on I/O failure; a failed write leaves the
    // buffer in place. Returns seqs removed from the retained index.
    std::vector<EvictedEvent> commit();

    bool knows(std::uint64_t generation) const;

    // Live generation. `next_seq` is the process counter (0: nothing published, known empty).
    // `cache` is the deque, newest at the back. `open_latest` may resend a prefix the file dropped.
    EventReplay replay_current(std::uint64_t generation, std::uint64_t next_seq,
                               const std::optional<msg::RequestSync::Since>& since, std::span<const EventPiece> cache,
                               std::span<const EventPiece> open_latest) const;

    // Durable records of one generation. Empty when this log has never committed that generation.
    std::optional<EventReplay> replay_archive(std::uint64_t generation,
                                              const std::optional<msg::RequestSync::Since>& since) const;

private:
    struct Pending {
        std::uint64_t generation = 0;
        std::uint64_t seq = 0;
        std::uint64_t map_epoch = 0;
        std::int64_t stamp_ns = 0;
        msg::Bytes cdr;
    };
    struct Stored {
        std::uint64_t generation = 0;
        std::uint64_t seq = 0;
        std::uint64_t map_epoch = 0;
        std::int64_t stamp_ns = 0;
        std::int64_t retain_ns = 0;
        int segment = 0;
        std::uint64_t offset = 0;
        std::uint32_t payload_len = 0;
    };
    struct Segment {
        int number = 0;
        std::uint64_t bytes = 0;
    };

    EventLog(std::filesystem::path room, EventLogLimits limits, EventLogIo* io);

    std::filesystem::path events_dir() const;
    std::filesystem::path segment_path(int number) const;
    std::filesystem::path manifest_path() const;
    void ensure_events();
    void roll_if_needed();
    std::vector<std::uint8_t> encode_pending(std::int64_t retain_base, bool have_base) const;
    void write_pending();
    void persist_manifest();
    std::vector<EvictedEvent> retain();
    void unlink_segment(int number);
    msg::Bytes read_cdr(const Stored& stored) const;
    std::vector<Stored> retained_of(std::uint64_t generation) const;
    void recover();
    void require_next_seq() const;
    void require_suffix() const;
    bool expired(std::int64_t retain) const;
    std::uint64_t occupied_bytes() const;

    std::filesystem::path room_;
    EventLogLimits limits_;
    std::unique_ptr<EventLogIo> owned_;
    EventLogIo* io_ = nullptr;
    std::vector<Segment> segments_;
    std::vector<Stored> index_;
    std::vector<Pending> pending_;
    // Last published seq of each generation, including ones whose records have been dropped.
    std::map<std::uint64_t, std::uint64_t> generations_;
    std::map<std::uint64_t, std::uint64_t> manifest_at_;
    std::size_t manifest_records_ = 0;
    bool manifest_dirty_ = false;
    // Size of manifest.log before an attempt that may have written a partial tail.
    std::optional<std::uint64_t> manifest_mark_;
    bool events_ready_ = false;
    bool parent_synced_ = false;
    bool active_dir_synced_ = true;
    bool have_retain_ = false;
    std::int64_t retain_high_ = 0;
    // Size of the active segment before an attempt that may have written a partial tail.
    std::optional<std::uint64_t> write_mark_;
};

EventReplay replay_from_pieces(std::optional<std::uint64_t> bound, const std::optional<msg::RequestSync::Since>& since,
                               std::vector<EventPiece> log, std::vector<EventPiece> older);

}  // namespace room_memory
