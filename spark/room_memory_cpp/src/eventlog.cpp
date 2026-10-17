#include "room_memory/eventlog.hpp"

#include "room_memory/memory.hpp"

#include <zlib.h>

#include <fcntl.h>
#include <unistd.h>

#include <algorithm>
#include <cstring>
#include <deque>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string_view>
#include <utility>

namespace room_memory {
namespace {

constexpr std::uint32_t kManifestPayload = 16;
constexpr std::uint64_t kManifestCompactBytes = std::uint64_t{1} << 20;

[[noreturn]] void io_error(const std::string& what, const std::filesystem::path& path) {
    throw std::runtime_error(what + " " + path.string() + ": " + std::strerror(errno));
}

void put_u32(std::vector<std::uint8_t>& out, std::uint32_t value) {
    out.push_back(static_cast<std::uint8_t>(value));
    out.push_back(static_cast<std::uint8_t>(value >> 8));
    out.push_back(static_cast<std::uint8_t>(value >> 16));
    out.push_back(static_cast<std::uint8_t>(value >> 24));
}

void put_u64(std::vector<std::uint8_t>& out, std::uint64_t value) {
    for (int shift = 0; shift < 64; shift += 8) out.push_back(static_cast<std::uint8_t>(value >> shift));
}

void put_i64(std::vector<std::uint8_t>& out, std::int64_t value) {
    std::uint64_t bits = 0;
    std::memcpy(&bits, &value, sizeof(bits));
    put_u64(out, bits);
}

std::uint32_t get_u32(const std::uint8_t* p) {
    return std::uint32_t{p[0]} | (std::uint32_t{p[1]} << 8) | (std::uint32_t{p[2]} << 16) | (std::uint32_t{p[3]} << 24);
}

std::uint64_t get_u64(const std::uint8_t* p) {
    std::uint64_t value = 0;
    for (int n = 0; n < 8; ++n) value |= std::uint64_t{p[n]} << (8 * n);
    return value;
}

std::int64_t get_i64(const std::uint8_t* p) {
    const std::uint64_t bits = get_u64(p);
    std::int64_t value = 0;
    std::memcpy(&value, &bits, sizeof(value));
    return value;
}

std::uint32_t checksum(std::span<const std::uint8_t> payload) {
    const auto crc = ::crc32(0L, payload.empty() ? Z_NULL : payload.data(), static_cast<uInt>(payload.size()));
    return static_cast<std::uint32_t>(crc);
}

std::vector<std::uint8_t> frame(std::uint32_t magic, const std::vector<std::uint8_t>& payload) {
    std::vector<std::uint8_t> out;
    out.reserve(kEventHeaderBytes + payload.size());
    put_u32(out, magic);
    put_u32(out, static_cast<std::uint32_t>(payload.size()));
    put_u32(out, checksum(payload));
    out.insert(out.end(), payload.begin(), payload.end());
    return out;
}

bool valid_frame(const std::vector<std::uint8_t>& data, std::uint64_t offset, std::uint32_t magic, std::uint32_t min_len,
                 std::uint32_t max_len) {
    if (offset > data.size() || data.size() - offset < kEventHeaderBytes) return false;
    const std::uint8_t* p = data.data() + offset;
    if (get_u32(p) != magic) return false;
    const std::uint32_t len = get_u32(p + 4);
    if (len < min_len || len > max_len) return false;
    if (data.size() - offset < static_cast<std::uint64_t>(kEventHeaderBytes) + len) return false;
    const auto payload = std::span<const std::uint8_t>(p + kEventHeaderBytes, len);
    return checksum(payload) == get_u32(p + 8);
}

bool any_valid_after(const std::vector<std::uint8_t>& data, std::uint64_t start, std::uint32_t magic, std::uint32_t min_len,
                     std::uint32_t max_len) {
    for (std::uint64_t n = start; n < data.size(); ++n) {
        if (valid_frame(data, n, magic, min_len, max_len)) return true;
    }
    return false;
}

bool parse_segment_name(const std::string& name, int& number) {
    constexpr std::string_view kPrefix = "seg-";
    constexpr std::string_view kSuffix = ".log";
    if (name.size() != kPrefix.size() + 6 + kSuffix.size()) return false;
    if (name.compare(0, kPrefix.size(), kPrefix) != 0) return false;
    if (name.compare(name.size() - kSuffix.size(), kSuffix.size(), kSuffix) != 0) return false;
    int value = 0;
    for (std::size_t n = 0; n < 6; ++n) {
        const char c = name[kPrefix.size() + n];
        if (c < '0' || c > '9') return false;
        value = value * 10 + (c - '0');
    }
    number = value;
    return true;
}

// Writes `bytes` so the file's durable contents end with them at `*mark`. A torn tail from an
// earlier attempt is truncated first. A matching tail is only fsynced. On failure the file is
// rolled back to `*mark` when that fsync succeeds; otherwise `mark` stays so the next call adopts.
void durable_append(EventLogIo& io, const std::filesystem::path& path, std::span<const std::uint8_t> bytes,
                    std::optional<std::uint64_t>& mark, bool& dir_synced, const std::filesystem::path& dir) {
    if (!mark) {
        mark = io.exists(path) ? io.file_size(path) : 0;
    }
    const std::uint64_t mark_at = *mark;
    bool durable = false;
    try {
        const std::uint64_t size = io.exists(path) ? io.file_size(path) : 0;
        if (size < mark_at) throw std::runtime_error("event log shrank under a commit: " + path.string());
        bool already = false;
        if (size == mark_at + bytes.size()) {
            if (bytes.empty()) {
                already = true;
            } else {
                std::vector<std::uint8_t> tail(bytes.size());
                io.read_at(path, mark_at, tail);
                already = std::equal(tail.begin(), tail.end(), bytes.begin());
            }
        }
        if (!already) {
            if (size != mark_at) {
                io.truncate(path, mark_at);
                io.fsync_file(path);
            }
            if (!bytes.empty()) io.append(path, bytes);
        }
        io.fsync_file(path);
        durable = true;
        if (!dir_synced) {
            io.fsync_dir(dir);
            dir_synced = true;
        }
        mark.reset();
    } catch (const std::exception&) {
        if (!durable) {
            try {
                if (io.exists(path)) {
                    const std::uint64_t size = io.file_size(path);
                    if (size != mark_at) {
                        io.truncate(path, mark_at);
                        io.fsync_file(path);
                    }
                }
            } catch (const std::exception&) {
                throw;
            }
        }
        throw;
    }
}

std::vector<std::uint8_t> manifest_frame(std::uint64_t generation, std::uint64_t seq) {
    std::vector<std::uint8_t> payload;
    payload.reserve(kManifestPayload);
    put_u64(payload, generation);
    put_u64(payload, seq);
    return frame(kEventManifestMagic, payload);
}

}  // namespace

bool PosixEventLogIo::exists(const std::filesystem::path& path) {
    std::error_code error;
    return std::filesystem::exists(path, error);
}

void PosixEventLogIo::create_directory(const std::filesystem::path& path) {
    std::error_code error;
    std::filesystem::create_directory(path, error);
    if (error) throw std::runtime_error("cannot create " + path.string() + ": " + error.message());
}

std::vector<std::filesystem::path> PosixEventLogIo::list(const std::filesystem::path& dir) {
    std::vector<std::filesystem::path> out;
    std::error_code error;
    for (const std::filesystem::directory_entry& entry : std::filesystem::directory_iterator(dir, error)) {
        if (error) break;
        out.push_back(entry.path());
    }
    if (error) throw std::runtime_error("cannot list " + dir.string() + ": " + error.message());
    return out;
}

std::uint64_t PosixEventLogIo::file_size(const std::filesystem::path& path) {
    std::error_code error;
    const auto size = std::filesystem::file_size(path, error);
    if (error) throw std::runtime_error("cannot stat " + path.string() + ": " + error.message());
    return static_cast<std::uint64_t>(size);
}

void PosixEventLogIo::read_at(const std::filesystem::path& path, std::uint64_t offset, std::span<std::uint8_t> dest) {
    const int fd = ::open(path.c_str(), O_RDONLY | O_CLOEXEC);
    if (fd < 0) io_error("cannot read", path);
    std::size_t got = 0;
    while (got < dest.size()) {
        const ssize_t n = ::pread(fd, dest.data() + got, dest.size() - got, static_cast<off_t>(offset + got));
        if (n < 0) {
            ::close(fd);
            io_error("cannot read", path);
        }
        if (n == 0) {
            ::close(fd);
            throw std::runtime_error("short read " + path.string());
        }
        got += static_cast<std::size_t>(n);
    }
    ::close(fd);
}

void PosixEventLogIo::append(const std::filesystem::path& path, std::span<const std::uint8_t> data) {
    const int fd = ::open(path.c_str(), O_WRONLY | O_CREAT | O_APPEND | O_CLOEXEC, 0644);
    if (fd < 0) io_error("cannot write", path);
    std::size_t wrote = 0;
    while (wrote < data.size()) {
        const ssize_t n = ::write(fd, data.data() + wrote, data.size() - wrote);
        if (n < 0) {
            ::close(fd);
            io_error("cannot write", path);
        }
        if (n == 0) {
            ::close(fd);
            throw std::runtime_error("short write " + path.string());
        }
        wrote += static_cast<std::size_t>(n);
    }
    ::close(fd);
}

void PosixEventLogIo::fsync_file(const std::filesystem::path& path) {
    const int fd = ::open(path.c_str(), O_RDWR | O_CLOEXEC);
    if (fd < 0) io_error("cannot sync", path);
    const bool ok = ::fsync(fd) == 0;
    ::close(fd);
    if (!ok) io_error("cannot sync", path);
}

void PosixEventLogIo::fsync_dir(const std::filesystem::path& path) {
    const int fd = ::open(path.c_str(), O_RDONLY | O_CLOEXEC);
    if (fd < 0) io_error("cannot sync", path);
    const bool ok = ::fsync(fd) == 0;
    ::close(fd);
    if (!ok) io_error("cannot sync", path);
}

void PosixEventLogIo::truncate(const std::filesystem::path& path, std::uint64_t size) {
    if (::truncate(path.c_str(), static_cast<off_t>(size)) != 0) io_error("cannot truncate", path);
}

void PosixEventLogIo::remove_file(const std::filesystem::path& path) {
    std::error_code error;
    std::filesystem::remove(path, error);
    if (error && error != std::errc::no_such_file_or_directory) {
        throw std::runtime_error("cannot remove " + path.string() + ": " + error.message());
    }
}

void PosixEventLogIo::write_atomic(const std::filesystem::path& path, std::span<const std::uint8_t> data) {
    const std::filesystem::path parent = path.has_parent_path() ? path.parent_path() : std::filesystem::path(".");
    const std::filesystem::path scratch = path.string() + ".tmp";
    const int fd = ::open(scratch.c_str(), O_WRONLY | O_CREAT | O_TRUNC | O_CLOEXEC, 0644);
    if (fd < 0) io_error("cannot write", scratch);
    std::size_t wrote = 0;
    while (wrote < data.size()) {
        const ssize_t n = ::write(fd, data.data() + wrote, data.size() - wrote);
        if (n < 0) {
            ::close(fd);
            io_error("cannot write", scratch);
        }
        wrote += static_cast<std::size_t>(n);
    }
    const bool synced = ::fsync(fd) == 0;
    ::close(fd);
    if (!synced) io_error("cannot sync", scratch);
    std::error_code error;
    std::filesystem::rename(scratch, path, error);
    if (error) throw std::runtime_error("cannot replace " + path.string() + ": " + error.message());
    fsync_dir(parent);
}

EventLog::EventLog(std::filesystem::path room, EventLogLimits limits, EventLogIo* io)
    : room_(std::move(room)), limits_(limits), io_(io) {}

EventLog EventLog::open(std::filesystem::path room, EventLogLimits limits, EventLogIo* io) {
    if (limits.segment_bytes == 0 || limits.total_bytes < limits.segment_bytes || limits.age_ns <= 0) {
        throw std::invalid_argument("event log limits must have segment_bytes > 0, total_bytes >= segment_bytes, age_ns > 0");
    }
    std::unique_ptr<EventLogIo> owned;
    if (!io) {
        owned = std::make_unique<PosixEventLogIo>();
        io = owned.get();
    }
    EventLog log(std::move(room), limits, io);
    log.owned_ = std::move(owned);
    log.recover();
    return log;
}

std::filesystem::path EventLog::events_dir() const { return room_ / "events"; }

std::filesystem::path EventLog::segment_path(int number) const {
    std::ostringstream name;
    name << "seg-" << std::setw(6) << std::setfill('0') << number << ".log";
    return events_dir() / name.str();
}

std::filesystem::path EventLog::manifest_path() const { return events_dir() / "manifest.log"; }

void EventLog::ensure_events() {
    if (events_ready_ && parent_synced_) return;
    const std::filesystem::path dir = events_dir();
    if (!io_->exists(dir)) {
        if (!io_->exists(room_)) {
            throw std::runtime_error("event log room directory does not exist: " + room_.string());
        }
        io_->create_directory(dir);
        parent_synced_ = false;
    }
    if (!parent_synced_) {
        io_->fsync_dir(room_);
        parent_synced_ = true;
    }
    events_ready_ = true;
}

std::uint64_t EventLog::segment_ceiling() const {
    return limits_.total_bytes + static_cast<std::uint64_t>(kEventMaxPayload) + kEventHeaderBytes + 4096;
}

std::uint64_t EventLog::record_bytes(const Pending& pending) const {
    return static_cast<std::uint64_t>(kEventHeaderBytes) + kEventFixedBytes + pending.cdr.size();
}

bool EventLog::segment_accepts(std::uint64_t used, std::uint64_t record) const {
    // A record of its own may pass the segment target. It still fits under the ceiling:
    // the payload cap is 1 MiB and the ceiling adds that on top of total_bytes.
    if (used == 0) return true;
    if (used >= limits_.segment_bytes) return false;
    return used + record <= segment_ceiling();
}

void EventLog::roll_if_needed() {
    const std::uint64_t next = pending_.empty() ? 0 : record_bytes(pending_.front());
    if (!segments_.empty() && segment_accepts(segments_.back().bytes, next)) return;
    const int number = segments_.empty() ? 1 : segments_.back().number + 1;
    segments_.push_back(Segment{number, 0});
    active_dir_synced_ = false;
}

std::size_t EventLog::fit_in_segment(std::uint64_t used) const {
    std::size_t count = 0;
    std::uint64_t at = used;
    for (const Pending& pending : pending_) {
        const std::uint64_t record = record_bytes(pending);
        if (count > 0 && !segment_accepts(at, record)) break;
        at += record;
        ++count;
    }
    return count;
}

std::vector<std::uint8_t> EventLog::encode_pending(std::size_t count, std::int64_t retain_base, bool have_base) const {
    std::vector<std::uint8_t> out;
    std::int64_t retain = retain_base;
    bool have = have_base;
    const std::size_t end = std::min(count, pending_.size());
    for (std::size_t n = 0; n < end; ++n) {
        const Pending& pending = pending_[n];
        retain = have ? std::max(retain, pending.stamp_ns) : pending.stamp_ns;
        have = true;
        std::vector<std::uint8_t> payload;
        payload.reserve(kEventFixedBytes + pending.cdr.size());
        put_u64(payload, pending.generation);
        put_u64(payload, pending.seq);
        put_u64(payload, pending.map_epoch);
        put_i64(payload, pending.stamp_ns);
        put_i64(payload, retain);
        put_u32(payload, static_cast<std::uint32_t>(pending.cdr.size()));
        payload.insert(payload.end(), pending.cdr.begin(), pending.cdr.end());
        const std::vector<std::uint8_t> record = frame(kEventRecordMagic, payload);
        out.insert(out.end(), record.begin(), record.end());
    }
    return out;
}

void EventLog::append(std::uint64_t generation, std::uint64_t seq, std::uint64_t map_epoch, std::int64_t stamp_ns,
                      msg::Bytes cdr) {
    if (cdr.size() > static_cast<std::size_t>(kEventMaxPayload) - kEventFixedBytes) {
        throw std::invalid_argument("event record exceeds 1 MiB");
    }
    pending_.push_back(Pending{generation, seq, map_epoch, stamp_ns, std::move(cdr)});
}

void EventLog::require_next_seq() const {
    // A known generation's next record is its high water plus one. A new generation may
    // start at whatever seq this process publishes (a restart starts at 0). Checked
    // before the write: appending seq 0 after a higher high water must not land on disk.
    std::map<std::uint64_t, std::uint64_t> next;
    for (const auto& [generation, seq] : generations_) {
        if (seq == std::numeric_limits<std::uint64_t>::max()) {
            throw std::runtime_error("event log does not reach its manifest high water");
        }
        next.emplace(generation, seq + 1);
    }
    for (const Pending& record : pending_) {
        const auto known = next.find(record.generation);
        if (known == next.end()) {
            if (record.seq == std::numeric_limits<std::uint64_t>::max()) {
                throw std::runtime_error("event log does not reach its manifest high water");
            }
            next.emplace(record.generation, record.seq + 1);
            continue;
        }
        if (record.seq != known->second || known->second == std::numeric_limits<std::uint64_t>::max()) {
            throw std::runtime_error("event log does not reach its manifest high water");
        }
        known->second += 1;
    }
}

void EventLog::write_pending() {
    if (pending_.empty() && !write_mark_) return;
    if (!pending_.empty()) require_next_seq();
    ensure_events();
    // One chunk fills the active segment and is one write plus one fsync. The next chunk
    // rolls. A retry (write_mark_ set) rewrites the same segment instead of rolling.
    while (!pending_.empty() || write_mark_) {
        if (!write_mark_) {
            if (pending_.empty()) break;
            roll_if_needed();
        }
        const std::uint64_t used = write_mark_ ? *write_mark_ : segments_.back().bytes;
        const std::size_t count = fit_in_segment(used);
        if (count == 0) {
            if (!pending_.empty()) throw std::logic_error("event log could not place a record");
            break;
        }
        const std::vector<std::uint8_t> encoded = encode_pending(count, retain_high_, have_retain_);
        const std::filesystem::path path = segment_path(segments_.back().number);
        if (!write_mark_) write_mark_ = io_->exists(path) ? io_->file_size(path) : std::uint64_t{0};
        const std::uint64_t start = *write_mark_;
        durable_append(*io_, path, encoded, write_mark_, active_dir_synced_, events_dir());
        std::int64_t retain = retain_high_;
        bool have = have_retain_;
        std::uint64_t offset = start;
        for (std::size_t n = 0; n < count; ++n) {
            const Pending& pending = pending_[n];
            retain = have ? std::max(retain, pending.stamp_ns) : pending.stamp_ns;
            have = true;
            const auto payload_len = static_cast<std::uint32_t>(kEventFixedBytes + pending.cdr.size());
            index_.push_back(Stored{pending.generation, pending.seq, pending.map_epoch, pending.stamp_ns, retain,
                                    segments_.back().number, offset, payload_len});
            offset += kEventHeaderBytes + payload_len;
            const auto known = generations_.find(pending.generation);
            if (known == generations_.end() || known->second < pending.seq) generations_[pending.generation] = pending.seq;
            const auto filed = manifest_at_.find(pending.generation);
            if (filed == manifest_at_.end() || filed->second < generations_[pending.generation]) manifest_dirty_ = true;
        }
        retain_high_ = retain;
        have_retain_ = have;
        segments_.back().bytes = offset;
        pending_.erase(pending_.begin(), pending_.begin() + static_cast<std::ptrdiff_t>(count));
    }
}

void EventLog::persist_manifest() {
    if (!manifest_dirty_) return;
    ensure_events();
    std::vector<std::uint8_t> blob;
    std::size_t added = 0;
    for (const auto& [generation, seq] : generations_) {
        const auto filed = manifest_at_.find(generation);
        if (filed != manifest_at_.end() && filed->second >= seq) continue;
        const std::vector<std::uint8_t> record = manifest_frame(generation, seq);
        blob.insert(blob.end(), record.begin(), record.end());
        ++added;
    }
    if (blob.empty()) {
        manifest_dirty_ = false;
        return;
    }
    const std::filesystem::path path = manifest_path();
    const bool exists = io_->exists(path);
    const std::uint64_t size = exists ? io_->file_size(path) : 0;
    const bool compact = exists && (size > kManifestCompactBytes || manifest_records_ + added > generations_.size() * 4);
    if (compact) {
        std::vector<std::uint8_t> whole;
        for (const auto& [generation, seq] : generations_) {
            const std::vector<std::uint8_t> record = manifest_frame(generation, seq);
            whole.insert(whole.end(), record.begin(), record.end());
        }
        io_->write_atomic(path, whole);
        active_dir_synced_ = true;
        manifest_at_ = generations_;
        manifest_records_ = generations_.size();
        manifest_dirty_ = false;
        manifest_mark_.reset();
        return;
    }
    // A new manifest file needs its directory entry fsynced; the segment's earlier sync did not cover it.
    if (!exists) active_dir_synced_ = false;
    if (!manifest_mark_) manifest_mark_ = size;
    durable_append(*io_, path, blob, manifest_mark_, active_dir_synced_, events_dir());
    manifest_at_ = generations_;
    manifest_records_ += added;
    manifest_dirty_ = false;
}

bool EventLog::expired(std::int64_t retain) const {
    if (!have_retain_ || retain >= retain_high_) return false;
    if (retain > std::numeric_limits<std::int64_t>::max() - limits_.age_ns) return false;
    return retain_high_ > retain + limits_.age_ns;
}

std::uint64_t EventLog::occupied_bytes() const {
    std::uint64_t total = 0;
    for (const Segment& segment : segments_) total += segment.bytes;
    return total;
}

void EventLog::unlink_segment(int number) { io_->remove_file(segment_path(number)); }

std::vector<EvictedEvent> EventLog::retain() {
    std::vector<EvictedEvent> dropped;
    if (index_.empty() && segments_.size() <= 1) return dropped;
    std::size_t cut = 0;
    while (cut < index_.size() && expired(index_[cut].retain_ns)) ++cut;
    if (cut > 0) {
        const int keep = cut < index_.size() ? index_[cut].segment : segments_.back().number + 1;
        std::vector<int> unlink;
        for (const Segment& segment : segments_) {
            if (segment.number < keep && segment.number != segments_.back().number) unlink.push_back(segment.number);
        }
        for (const int number : unlink) unlink_segment(number);
        if (!unlink.empty()) io_->fsync_dir(events_dir());
        for (std::size_t n = 0; n < cut; ++n) dropped.push_back(EvictedEvent{index_[n].generation, index_[n].seq});
        index_.erase(index_.begin(), index_.begin() + static_cast<std::ptrdiff_t>(cut));
        segments_.erase(std::remove_if(segments_.begin(), segments_.end(),
                                       [&](const Segment& segment) {
                                           return std::find(unlink.begin(), unlink.end(), segment.number) != unlink.end();
                                       }),
                        segments_.end());
    }
    while (segments_.size() > 1 && occupied_bytes() > limits_.total_bytes) {
        const int oldest = segments_.front().number;
        unlink_segment(oldest);
        io_->fsync_dir(events_dir());
        for (const Stored& stored : index_) {
            if (stored.segment == oldest) dropped.push_back(EvictedEvent{stored.generation, stored.seq});
        }
        index_.erase(std::remove_if(index_.begin(), index_.end(),
                                    [&](const Stored& stored) { return stored.segment == oldest; }),
                     index_.end());
        segments_.erase(segments_.begin());
    }
    return dropped;
}

std::vector<EvictedEvent> EventLog::commit() {
    if (!pending_.empty() || write_mark_) write_pending();
    if (manifest_dirty_) persist_manifest();
    std::vector<EvictedEvent> dropped = retain();
    require_suffix();
    return dropped;
}

bool EventLog::knows(std::uint64_t generation) const { return generations_.find(generation) != generations_.end(); }

msg::Bytes EventLog::read_cdr(const Stored& stored) const {
    std::vector<std::uint8_t> payload(stored.payload_len);
    io_->read_at(segment_path(stored.segment), stored.offset + kEventHeaderBytes, payload);
    if (payload.size() < kEventFixedBytes) throw std::runtime_error("event record is shorter than its prefix");
    const std::uint32_t cdr_len = get_u32(payload.data() + 40);
    if (static_cast<std::uint64_t>(kEventFixedBytes) + cdr_len != payload.size()) {
        throw std::runtime_error("event record length does not match its payload");
    }
    return msg::Bytes(payload.begin() + kEventFixedBytes, payload.end());
}

std::vector<EventLog::Stored> EventLog::retained_of(std::uint64_t generation) const {
    std::vector<Stored> out;
    for (const Stored& stored : index_) {
        if (stored.generation == generation) out.push_back(stored);
    }
    return out;
}

void EventLog::recover() {
    const std::filesystem::path dir = events_dir();
    if (!io_->exists(room_) || !io_->exists(dir)) return;
    events_ready_ = true;
    parent_synced_ = true;
    std::vector<int> numbers;
    for (const std::filesystem::path& path : io_->list(dir)) {
        int number = 0;
        if (!parse_segment_name(path.filename().string(), number)) continue;
        numbers.push_back(number);
    }
    std::sort(numbers.begin(), numbers.end());
    for (std::size_t n = 1; n < numbers.size(); ++n) {
        if (numbers[n] == numbers[n - 1]) throw std::runtime_error("duplicate event segment in " + dir.string());
    }
    const std::uint64_t ceiling = segment_ceiling();
    std::optional<std::int64_t> previous_retain;
    for (std::size_t n = 0; n < numbers.size(); ++n) {
        const bool allow_tear = n + 1 == numbers.size();
        const std::filesystem::path path = segment_path(numbers[n]);
        const std::uint64_t size = io_->file_size(path);
        if (size > ceiling) throw std::runtime_error("event segment is too large: " + path.string());
        std::vector<std::uint8_t> data(static_cast<std::size_t>(size));
        if (size) io_->read_at(path, 0, data);
        std::uint64_t offset = 0;
        while (offset < data.size()) {
            if (!valid_frame(data, offset, kEventRecordMagic, kEventFixedBytes, kEventMaxPayload)) {
                if (allow_tear && !any_valid_after(data, offset + 1, kEventRecordMagic, kEventFixedBytes, kEventMaxPayload)) {
                    io_->truncate(path, offset);
                    io_->fsync_file(path);
                    data.resize(static_cast<std::size_t>(offset));
                    break;
                }
                throw std::runtime_error("corrupt event segment " + path.string());
            }
            const std::uint8_t* payload = data.data() + offset + kEventHeaderBytes;
            const std::uint32_t payload_len = get_u32(data.data() + offset + 4);
            const std::uint32_t cdr_len = get_u32(payload + 40);
            if (static_cast<std::uint64_t>(kEventFixedBytes) + cdr_len != payload_len) {
                throw std::runtime_error("corrupt event segment " + path.string());
            }
            Stored stored;
            stored.generation = get_u64(payload);
            stored.seq = get_u64(payload + 8);
            stored.map_epoch = get_u64(payload + 16);
            stored.stamp_ns = get_i64(payload + 24);
            stored.retain_ns = get_i64(payload + 32);
            stored.segment = numbers[n];
            stored.offset = offset;
            stored.payload_len = payload_len;
            if (previous_retain && stored.retain_ns < *previous_retain) {
                throw std::runtime_error("event retain went backwards in " + path.string());
            }
            previous_retain = stored.retain_ns;
            const auto known = generations_.find(stored.generation);
            if (known == generations_.end() || known->second < stored.seq) generations_[stored.generation] = stored.seq;
            index_.push_back(stored);
            offset += kEventHeaderBytes + payload_len;
        }
        segments_.push_back(Segment{numbers[n], offset});
        have_retain_ = previous_retain.has_value();
        if (previous_retain) retain_high_ = *previous_retain;
    }
    if (io_->exists(manifest_path())) {
        const std::filesystem::path path = manifest_path();
        const std::uint64_t size = io_->file_size(path);
        if (size > kManifestCompactBytes + static_cast<std::uint64_t>(kEventMaxPayload)) {
            throw std::runtime_error("event manifest is too large: " + path.string());
        }
        std::vector<std::uint8_t> data(static_cast<std::size_t>(size));
        if (size) io_->read_at(path, 0, data);
        std::uint64_t offset = 0;
        while (offset < data.size()) {
            if (!valid_frame(data, offset, kEventManifestMagic, kManifestPayload, kManifestPayload)) {
                if (!any_valid_after(data, offset + 1, kEventManifestMagic, kManifestPayload, kManifestPayload)) {
                    io_->truncate(path, offset);
                    io_->fsync_file(path);
                    break;
                }
                throw std::runtime_error("corrupt event manifest " + path.string());
            }
            const std::uint8_t* payload = data.data() + offset + kEventHeaderBytes;
            const std::uint64_t generation = get_u64(payload);
            const std::uint64_t seq = get_u64(payload + 8);
            const auto filed = manifest_at_.find(generation);
            if (filed == manifest_at_.end() || filed->second < seq) manifest_at_[generation] = seq;
            ++manifest_records_;
            offset += kEventHeaderBytes + kManifestPayload;
        }
    }
    for (const auto& [generation, seq] : generations_) {
        const auto filed = manifest_at_.find(generation);
        if (filed == manifest_at_.end() || filed->second < seq) manifest_dirty_ = true;
    }
    for (const auto& [generation, seq] : manifest_at_) {
        const auto known = generations_.find(generation);
        if (known == generations_.end() || known->second < seq) generations_[generation] = seq;
    }
    // A crash can leave a CRC-valid tail in the page cache and no fsync. The manifest
    // must not name those bytes until the segment and its directory entry are durable.
    sync_recovered();
    if (manifest_dirty_) persist_manifest();
    retain();
    require_suffix();
}

void EventLog::sync_recovered() {
    if (segments_.empty()) return;
    for (const Segment& segment : segments_) io_->fsync_file(segment_path(segment.number));
    io_->fsync_dir(events_dir());
    io_->fsync_dir(room_);
    parent_synced_ = true;
    active_dir_synced_ = true;
}

void EventLog::require_suffix() const {
    for (const auto& [generation, seq] : generations_) {
        std::optional<std::uint64_t> last;
        for (const Stored& stored : index_) {
            if (stored.generation == generation) last = stored.seq;
        }
        // A fully evicted generation has a manifest high and no records. A retained suffix
        // has to end on that high: a torn tail is only the uncommitted write, which the
        // manifest does not name yet.
        if (last && *last != seq) throw std::runtime_error("event log does not reach its manifest high water");
    }
}

EventReplay replay_from_pieces(std::optional<std::uint64_t> bound, const std::optional<msg::RequestSync::Since>& since,
                               std::vector<EventPiece> log, std::vector<EventPiece> older) {
    EventReplay out;
    out.bound = bound;
    if (!bound) return out;
    if (!log.empty()) {
        for (std::size_t n = 1; n < log.size(); ++n) {
            if (log[n].seq != log[n - 1].seq + 1 || log[n].bytes == nullptr) {
                throw std::logic_error("event log replay is not a contiguous suffix");
            }
        }
        if (log.front().bytes == nullptr || log.back().seq != *bound) {
            throw std::logic_error("event log replay does not end at the published bound");
        }
    }
    std::optional<std::size_t> at;
    if (since) {
        for (std::size_t n = 0; n < log.size(); ++n) {
            if (log[n].seq == since->seq && log[n].map_epoch == since->epoch) {
                at = n;
                break;
            }
        }
    }
    if (at) {
        for (std::size_t n = *at + 1; n < log.size(); ++n) out.messages.push_back(*log[n].bytes);
        if (!out.messages.empty()) out.delivered.push_back(msg::SeqRange{since->seq + 1, *bound});
        return out;
    }
    std::sort(older.begin(), older.end(), [](const EventPiece& a, const EventPiece& b) { return a.seq < b.seq; });
    std::vector<msg::SeqRange> sent;
    for (const EventPiece& piece : older) {
        out.messages.push_back(*piece.bytes);
        sent.push_back(msg::SeqRange{piece.seq, piece.seq});
    }
    if (!log.empty()) {
        for (const EventPiece& piece : log) out.messages.push_back(*piece.bytes);
        sent.push_back(msg::SeqRange{log.front().seq, log.back().seq});
    }
    out.delivered = canonical_ranges(std::move(sent));
    const std::uint64_t first = since ? since->seq + 1 : 0;
    if (first <= *bound) out.unavailable = subtract_ranges({{first, *bound}}, out.delivered);
    return out;
}

EventReplay EventLog::replay_current(std::uint64_t generation, std::uint64_t next_seq,
                                     const std::optional<msg::RequestSync::Since>& since, std::span<const EventPiece> cache,
                                     std::span<const EventPiece> open_latest) const {
    if (next_seq == 0) return EventReplay{};
    const std::uint64_t bound = next_seq - 1;
    std::optional<std::uint64_t> committed;
    if (const auto known = generations_.find(generation); known != generations_.end()) committed = known->second;
    std::deque<msg::Bytes> owned;
    std::vector<EventPiece> log;
    std::vector<std::uint64_t> taken;
    const auto take = [&](std::uint64_t seq, std::uint64_t epoch, const msg::Bytes* bytes) {
        log.push_back(EventPiece{seq, epoch, bytes});
        taken.push_back(seq);
    };
    for (const Stored& stored : retained_of(generation)) {
        const msg::Bytes* bytes = nullptr;
        for (const EventPiece& piece : cache) {
            if (piece.seq == stored.seq && piece.bytes) {
                bytes = piece.bytes;
                break;
            }
        }
        if (!bytes) {
            owned.push_back(read_cdr(stored));
            bytes = &owned.back();
        }
        take(stored.seq, stored.map_epoch, bytes);
    }
    for (const Pending& pending : pending_) {
        if (pending.generation != generation) continue;
        if (committed && pending.seq <= *committed) continue;
        take(pending.seq, pending.map_epoch, &pending.cdr);
    }
    for (const EventPiece& piece : cache) {
        if (!piece.bytes) continue;
        if (committed && piece.seq <= *committed) continue;
        if (std::find(taken.begin(), taken.end(), piece.seq) != taken.end()) continue;
        if (piece.seq > bound) continue;
        take(piece.seq, piece.map_epoch, piece.bytes);
    }
    std::vector<std::size_t> order(log.size());
    for (std::size_t n = 0; n < order.size(); ++n) order[n] = n;
    std::sort(order.begin(), order.end(), [&](std::size_t a, std::size_t b) { return log[a].seq < log[b].seq; });
    std::vector<EventPiece> sorted;
    sorted.reserve(log.size());
    for (const std::size_t n : order) sorted.push_back(log[n]);
    std::vector<EventPiece> older;
    const std::optional<std::uint64_t> oldest = sorted.empty() ? std::nullopt : std::optional<std::uint64_t>{sorted.front().seq};
    for (const EventPiece& piece : open_latest) {
        if (!piece.bytes || piece.seq > bound) continue;
        if (oldest && piece.seq >= *oldest) continue;
        if (std::find(taken.begin(), taken.end(), piece.seq) != taken.end()) continue;
        older.push_back(piece);
    }
    return replay_from_pieces(bound, since, std::move(sorted), std::move(older));
}

std::optional<EventReplay> EventLog::replay_archive(std::uint64_t generation,
                                                   const std::optional<msg::RequestSync::Since>& since) const {
    if (generation == 0 || !knows(generation)) return std::nullopt;
    const std::uint64_t bound = generations_.at(generation);
    std::deque<msg::Bytes> owned;
    std::vector<EventPiece> log;
    for (const Stored& stored : retained_of(generation)) {
        owned.push_back(read_cdr(stored));
        log.push_back(EventPiece{stored.seq, stored.map_epoch, &owned.back()});
    }
    return replay_from_pieces(bound, since, std::move(log), {});
}

}  // namespace room_memory
