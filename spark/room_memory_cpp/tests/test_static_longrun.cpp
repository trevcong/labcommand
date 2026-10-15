// static_longrun: loop a static lidar recording through one RoomMemory for at least 30 minutes
// of simulated time and require zero change events.
//
// static5m contains a real motion from 86 s (PLAN.md), so the default slice is [t0, t0+80s).
// Each loop adds an integer number of window_s to every stamp, and the copy starts on a window
// boundary. The recent layer still crosses that seam. close_window is followed by map_update,
// matching room_memory_bag, then deadlines are flushed through the final boundary (the bag CLI
// leaves the tail open). state_dir is nullopt so nothing is read from or written to ~/room-memory.
//
// MCAP chunk checksums are zlib CRC-32. Zero means the writer did not set one.

#include "room_memory/memory.hpp"
#include "room_memory/pointcloud2.hpp"

#include <algorithm>
#include <chrono>
#include <cinttypes>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cerrno>
#include <filesystem>
#include <map>
#include <span>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <sys/resource.h>
#include <zlib.h>

#if defined(_OPENMP)
#include <omp.h>
#endif

#ifdef ROOM_MEMORY_HAS_ZSTD
#include <zstd.h>
#endif

namespace {

using room_memory::MemoryConfig;
using room_memory::PointCloud;
using room_memory::RoomMemory;
using room_memory::decode_pointcloud2;
namespace msg = room_memory::msg;

constexpr std::int64_t kNs = 1'000'000'000;
constexpr std::int64_t kDefaultUntilS = 80;
constexpr std::int64_t kDefaultSimS = 1800;
constexpr std::int64_t kPositiveUntilS = 110;
constexpr std::int64_t kChangeFromS = 86;
constexpr std::uint64_t kChunkCap = 128ull << 20;
constexpr int kSkip = 77;

const char* kNames[] = {"mid360", "jt128"};
const char* kKinds[] = {"APPEARED", "DISAPPEARED", "MOVED"};
const char* kStates[] = {"CANDIDATE", "CONFIRMED", "ACCEPTED", "REVERTED", "EXPIRED"};

struct Skip : std::runtime_error {
    using std::runtime_error::runtime_error;
};

struct Frame {
    int sensor = 0;
    std::int64_t log_ns = 0;
    std::uint64_t ordinal = 0;
    std::vector<float> xyz;
};

struct Seen {
    std::int64_t t_s = 0;
    int kind = 0;
    int state = 0;
    std::int64_t sensors = 0;
    std::int64_t voxels = 0;
    float c[3] = {};
};

struct Replay {
    std::vector<Seen> events;
    std::int64_t closed_s = 0;
    std::int64_t tracking_from_s = -1;
    bool tracking = false;
    std::uint64_t map_epoch = 0;
    std::uint64_t map_messages = 0;
};

[[noreturn]] void fail(const std::string& text) { throw std::runtime_error(text); }

std::int64_t env_i64(const char* name) {
    const char* value = std::getenv(name);
    if (!value || !*value) return -1;
    errno = 0;
    char* end = nullptr;
    const long long parsed = std::strtoll(value, &end, 10);
    if (errno || end == value || *end) fail(std::string(name) + " is not an integer");
    return static_cast<std::int64_t>(parsed);
}

std::uint32_t crc32_bytes(const std::uint8_t* data, std::size_t size) {
    uLong crc = ::crc32(0L, nullptr, 0);
    while (size) {
        const uInt take = size > (1u << 20) ? (1u << 20) : static_cast<uInt>(size);
        crc = ::crc32(crc, data, take);
        data += take;
        size -= take;
    }
    return static_cast<std::uint32_t>(crc);
}

class View {
public:
    View(const std::uint8_t* data, std::size_t size) : data_(data), size_(size) {}

    std::size_t remaining() const { return size_ - off_; }
    std::size_t tell() const { return off_; }
    const std::uint8_t* cursor() const { return data_ + off_; }

    void need(std::size_t n, const char* what) const {
        if (n > size_ - off_) fail(std::string("truncated mcap ") + what);
    }

    std::uint8_t u8(const char* what) {
        need(1, what);
        return data_[off_++];
    }

    std::uint16_t u16(const char* what) {
        need(2, what);
        const std::uint16_t value = static_cast<std::uint16_t>(data_[off_] | (data_[off_ + 1] << 8));
        off_ += 2;
        return value;
    }

    std::uint32_t u32(const char* what) {
        need(4, what);
        std::uint32_t value = 0;
        for (int byte = 0; byte < 4; ++byte) value |= static_cast<std::uint32_t>(data_[off_ + byte]) << (8 * byte);
        off_ += 4;
        return value;
    }

    std::uint64_t u64(const char* what) {
        need(8, what);
        std::uint64_t value = 0;
        for (int byte = 0; byte < 8; ++byte) value |= static_cast<std::uint64_t>(data_[off_ + byte]) << (8 * byte);
        off_ += 8;
        return value;
    }

    std::string str(const char* what) {
        const std::uint32_t n = u32(what);
        need(n, what);
        std::string text(reinterpret_cast<const char*>(data_ + off_), n);
        off_ += n;
        return text;
    }

    std::span<const std::uint8_t> take(std::size_t n, const char* what) {
        need(n, what);
        std::span<const std::uint8_t> out(data_ + off_, n);
        off_ += n;
        return out;
    }

private:
    const std::uint8_t* data_;
    std::size_t size_;
    std::size_t off_ = 0;
};

class File {
public:
    explicit File(const std::filesystem::path& path) {
        file_ = std::fopen(path.c_str(), "rb");
        if (!file_) fail("cannot open " + path.string());
        if (fseeko(file_, 0, SEEK_END) != 0) fail("seek end " + path.string());
        const off_t end = ftello(file_);
        if (end < 0) fail("size " + path.string());
        size_ = static_cast<std::uint64_t>(end);
        path_ = path;
    }

    File(const File&) = delete;
    File& operator=(const File&) = delete;

    ~File() {
        if (file_) std::fclose(file_);
    }

    std::uint64_t size() const { return size_; }
    const std::filesystem::path& path() const { return path_; }

    void read(std::uint64_t offset, void* dest, std::size_t n) const {
        if (n == 0) return;
        if (offset > size_ || n > size_ - offset) fail("short read in " + path_.string());
        if (fseeko(file_, static_cast<off_t>(offset), SEEK_SET) != 0) fail("seek in " + path_.string());
        if (std::fread(dest, 1, n, file_) != n) fail("read in " + path_.string());
    }

    std::vector<std::uint8_t> bytes(std::uint64_t offset, std::size_t n) const {
        std::vector<std::uint8_t> out(n);
        read(offset, out.data(), n);
        return out;
    }

private:
    FILE* file_ = nullptr;
    std::uint64_t size_ = 0;
    std::filesystem::path path_;
};

int bag_index(const std::filesystem::path& path) {
    const std::string name = path.filename().string();
    const auto dot = name.rfind('.');
    const auto underscore = name.rfind('_');
    if (dot == std::string::npos || underscore == std::string::npos || underscore + 1 >= dot) return -1;
    int value = 0;
    for (std::size_t i = underscore + 1; i < dot; ++i) {
        if (name[i] < '0' || name[i] > '9') return -1;
        value = value * 10 + (name[i] - '0');
    }
    return value;
}

std::vector<std::filesystem::path> mcap_files(const std::filesystem::path& path) {
    namespace fs = std::filesystem;
    std::vector<fs::path> files;
    std::error_code error;
    if (fs::is_regular_file(path, error) && path.extension() == ".mcap") {
        files.push_back(path);
        return files;
    }
    if (!fs::is_directory(path, error)) return files;
    for (const fs::directory_entry& entry : fs::directory_iterator(path, error)) {
        if (error) break;
        if (entry.path().extension() == ".mcap") files.push_back(entry.path());
    }
    std::sort(files.begin(), files.end(), [](const fs::path& a, const fs::path& b) {
        const int ia = bag_index(a);
        const int ib = bag_index(b);
        if (ia != ib) return ia < ib;
        return a.filename().string() < b.filename().string();
    });
    return files;
}

bool known_static5m(const std::filesystem::path& path) {
    return path.filename() == "static5m" || path.parent_path().filename() == "static5m";
}

int topic_sensor(const std::string& topic) {
    if (topic == "/livox/lidar") return 0;
    if (topic == "/lidar_points") return 1;
    return -1;
}

struct ChunkLoc {
    std::uint64_t offset = 0;
    std::uint64_t length = 0;
    std::uint64_t start_ns = 0;
    std::uint64_t end_ns = 0;
};

struct BagIndex {
    std::map<std::uint16_t, int> channels;
    std::vector<ChunkLoc> chunks;
    std::uint64_t file_min = 0;
    std::uint64_t file_max = 0;
    bool have_time = false;
};

void note_time(BagIndex& index, std::uint64_t start, std::uint64_t end) {
    if (!index.have_time) {
        index.file_min = start;
        index.file_max = end;
        index.have_time = true;
        return;
    }
    index.file_min = std::min(index.file_min, start);
    index.file_max = std::max(index.file_max, end);
}

void parse_channel(std::span<const std::uint8_t> body, std::map<std::uint16_t, int>& channels) {
    View view(body.data(), body.size());
    const std::uint16_t id = view.u16("channel id");
    view.u16("schema id");
    const std::string topic = view.str("channel topic");
    channels[id] = topic_sensor(topic);
}

void parse_chunk_index(std::span<const std::uint8_t> body, BagIndex& index) {
    View view(body.data(), body.size());
    ChunkLoc chunk;
    chunk.start_ns = view.u64("chunk start");
    chunk.end_ns = view.u64("chunk end");
    chunk.offset = view.u64("chunk offset");
    chunk.length = view.u64("chunk length");
    const std::uint32_t offsets = view.u32("message index offsets");
    view.need(offsets, "message index offsets");
    view.take(offsets, "message index offsets");
    view.u64("message index length");
    view.str("chunk compression");
    view.u64("compressed size");
    view.u64("uncompressed size");
    if (chunk.length < 9 || chunk.length > kChunkCap) fail("chunk length out of range");
    note_time(index, chunk.start_ns, chunk.end_ns);
    index.chunks.push_back(chunk);
}

void absorb_records(const std::uint8_t* data, std::size_t size, BagIndex& index) {
    View view(data, size);
    while (view.remaining() >= 9) {
        const std::uint8_t opcode = view.u8("opcode");
        const std::uint64_t length = view.u64("record length");
        if (length > view.remaining()) fail("summary record exceeds the section");
        const auto body = view.take(static_cast<std::size_t>(length), "record");
        if (opcode == 0x04) parse_channel(body, index.channels);
        else if (opcode == 0x08) parse_chunk_index(body, index);
    }
    if (view.remaining() != 0) fail("truncated record header in the mcap summary");
}

BagIndex read_index(const File& file) {
    if (file.size() < 8 + 29 + 8) fail("mcap too small: " + file.path().string());
    std::uint8_t magic[8];
    file.read(0, magic, 8);
    file.read(file.size() - 8, magic + 0, 8);
    static const std::uint8_t kMagic[8] = {0x89, 'M', 'C', 'A', 'P', '0', '\r', '\n'};
    std::uint8_t start[8];
    file.read(0, start, 8);
    if (std::memcmp(start, kMagic, 8) != 0 || std::memcmp(magic, kMagic, 8) != 0) {
        fail("not an mcap file: " + file.path().string());
    }
    std::uint8_t footer[29];
    file.read(file.size() - 8 - 29, footer, 29);
    if (footer[0] != 0x02) fail("mcap footer missing in " + file.path().string());
    View foot(footer + 1, 28);
    const std::uint64_t length = foot.u64("footer length");
    if (length != 20) fail("unexpected mcap footer length");
    const std::uint64_t summary_start = foot.u64("summary start");
    foot.u64("summary offset start");
    foot.u32("summary crc");
    if (summary_start == 0 || summary_start >= file.size() - 37) fail("mcap has no summary section");
    const auto summary = file.bytes(summary_start, static_cast<std::size_t>(file.size() - 37 - summary_start));
    BagIndex index;
    absorb_records(summary.data(), summary.size(), index);
    if (index.channels.empty()) {
        const std::uint64_t first = index.chunks.empty()
                                        ? file.size()
                                        : std::min_element(index.chunks.begin(), index.chunks.end(),
                                                           [](const ChunkLoc& a, const ChunkLoc& b) {
                                                               return a.offset < b.offset;
                                                           })
                                              ->offset;
        if (first > 8 && first < file.size() && first - 8 < (64ull << 20)) {
            const auto preamble = file.bytes(8, static_cast<std::size_t>(first - 8));
            absorb_records(preamble.data(), preamble.size(), index);
        }
    }
    if (index.chunks.empty() || !index.have_time) fail("mcap summary has no chunks");
    return index;
}

std::vector<Frame> decode_chunk(const std::uint8_t* record, std::size_t size, std::map<std::uint16_t, int> channels) {
    if (size < 9 || record[0] != 0x06) fail("chunk record is not opcode 0x06");
    View header(record + 1, 8);
    const std::uint64_t body_len = header.u64("chunk body length");
    if (body_len + 9 != size) fail("chunk_length does not match the record body");
    View body(record + 9, static_cast<std::size_t>(body_len));
    body.u64("message start");
    body.u64("message end");
    const std::uint64_t uncompressed = body.u64("uncompressed size");
    const std::uint32_t crc = body.u32("uncompressed crc");
    const std::string compression = body.str("compression");
    const std::uint64_t compressed = body.u64("compressed length");
    if (compressed != body.remaining()) fail("compressed length does not fill the chunk");
    if (uncompressed > kChunkCap || compressed > kChunkCap) fail("chunk exceeds 128 MB");
    const std::uint8_t* payload = body.cursor();
    std::vector<std::uint8_t> raw(static_cast<std::size_t>(uncompressed));
    if (compression.empty()) {
        if (compressed != uncompressed) fail("uncompressed chunk size mismatch");
        std::memcpy(raw.data(), payload, static_cast<std::size_t>(uncompressed));
    } else if (compression == "zstd") {
#ifdef ROOM_MEMORY_HAS_ZSTD
        const std::size_t got = ZSTD_decompress(raw.data(), raw.size(), payload, static_cast<std::size_t>(compressed));
        if (ZSTD_isError(got) || got != uncompressed) fail(std::string("zstd decompress failed: ") + ZSTD_getErrorName(got));
#else
        fail("zstd chunk but libzstd was not linked");
#endif
    } else {
        fail("unsupported mcap compression " + compression);
    }
    if (crc != 0 && crc32_bytes(raw.data(), raw.size()) != crc) fail("mcap chunk CRC-32 mismatch");

    std::vector<Frame> frames;
    View inner(raw.data(), raw.size());
    while (inner.remaining() >= 9) {
        const std::uint8_t opcode = inner.u8("chunk opcode");
        const std::uint64_t length = inner.u64("chunk record length");
        if (length > inner.remaining()) fail("record exceeds decompressed chunk");
        const auto rec = inner.take(static_cast<std::size_t>(length), "chunk record");
        if (opcode == 0x04) {
            parse_channel(rec, channels);
            continue;
        }
        if (opcode != 0x05) continue;
        if (rec.size() < 22) fail("message record shorter than its header");
        View message(rec.data(), rec.size());
        const std::uint16_t id = message.u16("message channel");
        message.u32("message sequence");
        const std::uint64_t log_ns = message.u64("log time");
        message.u64("publish time");
        const auto found = channels.find(id);
        if (found == channels.end() || found->second < 0) continue;
        if (log_ns > static_cast<std::uint64_t>(INT64_MAX)) fail("log time does not fit int64");
        const auto cdr = message.take(message.remaining(), "point cloud");
        PointCloud cloud = decode_pointcloud2(cdr);
        Frame frame;
        frame.sensor = found->second;
        frame.log_ns = static_cast<std::int64_t>(log_ns);
        frame.xyz = std::move(cloud.positions);
        frames.push_back(std::move(frame));
    }
    if (inner.remaining() != 0) fail("truncated record header in a decompressed chunk");
    return frames;
}

struct Loaded {
    std::vector<Frame> frames;
    std::uint64_t file_min = 0;
    std::uint64_t file_max = 0;
};

Loaded load_recording(const std::vector<std::filesystem::path>& files, std::int64_t load_s) {
    Loaded loaded;
    bool have_time = false;
    std::vector<Frame> frames;
    for (const std::filesystem::path& path : files) {
        File file(path);
        BagIndex index = read_index(file);
        if (!have_time) {
            loaded.file_min = index.file_min;
            loaded.file_max = index.file_max;
            have_time = true;
        } else {
            loaded.file_min = std::min(loaded.file_min, index.file_min);
            loaded.file_max = std::max(loaded.file_max, index.file_max);
        }
        std::vector<ChunkLoc> selected;
        for (const ChunkLoc& chunk : index.chunks) {
            if (load_s < 0 || chunk.start_ns < loaded.file_min + static_cast<std::uint64_t>(load_s) * static_cast<std::uint64_t>(kNs)) {
                selected.push_back(chunk);
            }
        }
        std::sort(selected.begin(), selected.end(), [](const ChunkLoc& a, const ChunkLoc& b) { return a.offset < b.offset; });
        constexpr int kBatch = 32;
        for (std::size_t batch = 0; batch < selected.size(); batch += kBatch) {
            const int count = static_cast<int>(std::min(selected.size() - batch, static_cast<std::size_t>(kBatch)));
            std::vector<std::vector<std::uint8_t>> blobs(static_cast<std::size_t>(count));
            for (int n = 0; n < count; ++n) {
                const ChunkLoc& chunk = selected[batch + static_cast<std::size_t>(n)];
                blobs[static_cast<std::size_t>(n)] = file.bytes(chunk.offset, static_cast<std::size_t>(chunk.length));
            }
            std::vector<std::vector<Frame>> parts(static_cast<std::size_t>(count));
            std::vector<std::string> errors(static_cast<std::size_t>(count));
#pragma omp parallel for schedule(dynamic) if (count > 1)
            for (int n = 0; n < count; ++n) {
                try {
                    parts[static_cast<std::size_t>(n)] = decode_chunk(blobs[static_cast<std::size_t>(n)].data(),
                                                                      blobs[static_cast<std::size_t>(n)].size(), index.channels);
                } catch (const std::exception& error) {
                    errors[static_cast<std::size_t>(n)] = error.what();
                }
            }
            for (int n = 0; n < count; ++n) {
                if (!errors[static_cast<std::size_t>(n)].empty()) fail(errors[static_cast<std::size_t>(n)]);
                for (Frame& frame : parts[static_cast<std::size_t>(n)]) frames.push_back(std::move(frame));
            }
        }
    }
    if (!have_time) fail("no mcap chunks");
    std::uint64_t ordinal = 0;
    for (Frame& frame : frames) frame.ordinal = ordinal++;
    std::sort(frames.begin(), frames.end(), [](const Frame& a, const Frame& b) {
        if (a.log_ns != b.log_ns) return a.log_ns < b.log_ns;
        return a.ordinal < b.ordinal;
    });
    loaded.frames = std::move(frames);
    return loaded;
}

msg::Time time_of(std::int64_t ns) { return msg::Time{ns / kNs, ns % kNs}; }

Replay replay(const std::vector<Frame>& frames, std::int64_t t0, std::int64_t period_ns, int loops) {
    if (loops < 1 || period_ns < kNs || period_ns % kNs != 0) fail("replay period is not a positive whole number of seconds");
    if (period_ns > INT64_MAX / loops) fail("replay span overflows");
    MemoryConfig config;
    config.state_dir = std::nullopt;
    RoomMemory memory(config, time_of(t0));
    Replay out;
    std::int64_t window_end = t0 + kNs;
    std::int64_t next_status = t0 + kNs;
    const auto close_window = [&] {
        const std::vector<msg::ChangeEvent> events = memory.close_window(time_of(window_end));
        if (memory.map_update(time_of(window_end))) ++out.map_messages;
        const std::int64_t rel_s = (window_end - t0) / kNs;
        if (memory.state() == msg::MemoryState::tracking && out.tracking_from_s < 0) out.tracking_from_s = rel_s;
        for (const msg::ChangeEvent& event : events) {
            Seen seen;
            seen.t_s = rel_s;
            seen.kind = static_cast<int>(event.kind);
            seen.state = static_cast<int>(event.state);
            seen.sensors = event.sensors;
            seen.voxels = event.voxel_count;
            seen.c[0] = event.centroid[0];
            seen.c[1] = event.centroid[1];
            seen.c[2] = event.centroid[2];
            out.events.push_back(seen);
        }
        out.closed_s = rel_s;
        window_end += kNs;
    };
    const auto advance_to = [&](std::int64_t ns) {
        while (std::min(window_end, next_status) <= ns) {
            if (window_end <= next_status) {
                close_window();
            } else {
                (void)memory.status(time_of(next_status), 1.0);
                next_status += kNs;
            }
        }
    };
    for (int loop = 0; loop < loops; ++loop) {
        const std::int64_t shift = static_cast<std::int64_t>(loop) * period_ns;
        for (const Frame& frame : frames) {
            const std::int64_t rel = frame.log_ns - t0;
            if (rel < 0 || rel >= period_ns) continue;
            advance_to(t0 + shift + rel);
            memory.add_points(kNames[frame.sensor], frame.xyz);
        }
    }
    advance_to(t0 + static_cast<std::int64_t>(loops) * period_ns);
    out.tracking = memory.state() == msg::MemoryState::tracking;
    out.map_epoch = memory.map_epoch();
    return out;
}

void print_events(const std::vector<Seen>& events, int limit) {
    const int shown = std::min(limit, static_cast<int>(events.size()));
    for (int n = 0; n < shown; ++n) {
        const Seen& event = events[static_cast<std::size_t>(n)];
        const char* kind = event.kind >= 0 && event.kind < 3 ? kKinds[event.kind] : "?";
        const char* state = event.state >= 0 && event.state < 5 ? kStates[event.state] : "?";
        std::printf("static_longrun event t=%" PRId64 " %s %s sensors=%" PRId64 " voxels=%" PRId64
                    " centroid=%.3f %.3f %.3f\n",
                    event.t_s, kind, state, event.sensors, event.voxels, static_cast<double>(event.c[0]),
                    static_cast<double>(event.c[1]), static_cast<double>(event.c[2]));
    }
    if (static_cast<int>(events.size()) > shown) {
        std::printf("static_longrun event ... %zu more\n", events.size() - static_cast<std::size_t>(shown));
    }
}

void print_buckets(const Replay& run) {
    if (run.closed_s < 1) return;
    const int buckets = static_cast<int>((run.closed_s - 1) / 300) + 1;
    std::vector<int> counts(static_cast<std::size_t>(buckets), 0);
    for (const Seen& event : run.events) {
        if (event.t_s < 1 || event.t_s > run.closed_s) continue;
        ++counts[static_cast<std::size_t>((event.t_s - 1) / 300)];
    }
    for (int bucket = 0; bucket < buckets; ++bucket) {
        const std::int64_t start = static_cast<std::int64_t>(bucket) * 300;
        const std::int64_t end = std::min(run.closed_s, start + 300);
        const std::int64_t windows = end - start;
        const double per_min = windows ? static_cast<double>(counts[static_cast<std::size_t>(bucket)]) * 60.0 /
                                             static_cast<double>(windows)
                                       : 0.0;
        std::printf("static_longrun bucket %d %" PRId64 "-%" PRId64 " s windows=%" PRId64 " events=%d per_min=%.4f\n",
                    bucket, start, end, windows, counts[static_cast<std::size_t>(bucket)], per_min);
    }
}

int threads() {
#if defined(_OPENMP)
    return omp_get_max_threads();
#else
    return 1;
#endif
}

double rss_mb() {
    rusage usage{};
    if (getrusage(RUSAGE_SELF, &usage) != 0) return 0;
#ifdef __APPLE__
    return static_cast<double>(usage.ru_maxrss) / (1024.0 * 1024.0);
#else
    return static_cast<double>(usage.ru_maxrss) / 1024.0;
#endif
}

}  // namespace

#ifndef ROOM_MEMORY_HAS_ZSTD
int main() {
    std::printf("static_longrun: skip: built without libzstd, cannot read zstd mcap chunks\n");
    return kSkip;
}
#else
int main() {
    try {
        const auto started = std::chrono::steady_clock::now();
        namespace fs = std::filesystem;
        fs::path recording;
        if (const char* env = std::getenv("ROOM_MEMORY_RECORDING"); env && *env) {
            recording = env;
        } else {
            const char* home = std::getenv("HOME");
            if (!home || !*home) throw Skip("HOME is not set and ROOM_MEMORY_RECORDING is empty");
            recording = fs::path(home) / "Developer/lc-tasks/recordings/static5m";
        }
        const std::vector<fs::path> files = mcap_files(recording);
        if (files.empty()) {
            throw Skip("static recording not found at " + recording.string() + " (set ROOM_MEMORY_RECORDING)");
        }

        const std::int64_t until_env = env_i64("ROOM_MEMORY_LONGRUN_UNTIL_S");
        const std::int64_t sim_env = env_i64("ROOM_MEMORY_LONGRUN_SIM_S");
        const std::int64_t until_s = until_env < 0 ? kDefaultUntilS : until_env;
        const std::int64_t sim_s = sim_env < 0 ? kDefaultSimS : sim_env;
        if (until_s < 0) fail("ROOM_MEMORY_LONGRUN_UNTIL_S must be >= 0");
        if (sim_s < kDefaultSimS) fail("ROOM_MEMORY_LONGRUN_SIM_S must be >= 1800");
        const bool known = known_static5m(recording);
        if (known && (until_s == 0 || until_s > kDefaultUntilS)) {
            fail("static5m after 80 s includes the documented motion at 86 s; refusing a 0-event run");
        }
        bool positive = known && until_s == kDefaultUntilS;
        if (const char* flag = std::getenv("ROOM_MEMORY_LONGRUN_POSITIVE")) {
            if (std::strcmp(flag, "0") == 0) positive = false;
            else if (std::strcmp(flag, "1") == 0) positive = true;
            else fail("ROOM_MEMORY_LONGRUN_POSITIVE must be 0 or 1");
        }

        const std::int64_t load_s = until_s == 0 ? -1 : (positive ? std::max(until_s, kPositiveUntilS) : until_s);
        const auto loaded_at = std::chrono::steady_clock::now();
        const Loaded loaded = load_recording(files, load_s);
        const double load_seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - loaded_at).count();
        if (loaded.frames.empty()) fail("recording has no lidar frames");
        const std::int64_t t0 = loaded.frames.front().log_ns;
        if (static_cast<std::uint64_t>(t0) < loaded.file_min ||
            static_cast<std::uint64_t>(t0) - loaded.file_min > static_cast<std::uint64_t>(kNs)) {
            fail("first lidar frame is more than 1 s after the first chunk time");
        }

        std::int64_t span_s = until_s;
        if (until_s == 0) {
            if (loaded.file_max < static_cast<std::uint64_t>(t0)) fail("recording ends before it starts");
            span_s = static_cast<std::int64_t>((loaded.file_max - static_cast<std::uint64_t>(t0)) / static_cast<std::uint64_t>(kNs));
        }
        if (span_s < 1) fail("recording span is under 1 s");
        const std::int64_t span_ns = span_s * kNs;
        if (loaded.file_max < static_cast<std::uint64_t>(t0) + static_cast<std::uint64_t>(span_ns)) {
            fail("recording ends before the requested " + std::to_string(span_s) + " s interval");
        }
        if (positive && loaded.file_max < static_cast<std::uint64_t>(t0) + static_cast<std::uint64_t>(kPositiveUntilS) * static_cast<std::uint64_t>(kNs)) {
            fail("recording does not cover the 110 s known-positive interval");
        }

        std::int64_t slice_frames[2] = {0, 0};
        std::int64_t slice_points[2] = {0, 0};
        std::vector<char> second_hit[2] = {std::vector<char>(static_cast<std::size_t>(span_s), 0),
                                           std::vector<char>(static_cast<std::size_t>(span_s), 0)};
        for (const Frame& frame : loaded.frames) {
            if (frame.sensor < 0 || frame.sensor > 1) fail("bad sensor index");
            const std::int64_t rel = frame.log_ns - t0;
            if (rel < 0 || rel >= span_ns) continue;
            const std::int64_t sec = rel / kNs;
            second_hit[frame.sensor][static_cast<std::size_t>(sec)] = 1;
            ++slice_frames[frame.sensor];
            slice_points[frame.sensor] += static_cast<std::int64_t>(frame.xyz.size() / 3);
        }
        int missing[2] = {0, 0};
        for (int sensor = 0; sensor < 2; ++sensor) {
            for (char hit : second_hit[sensor]) {
                if (!hit) ++missing[sensor];
            }
            if (slice_frames[sensor] < span_s * 5) {
                fail(std::string(kNames[sensor]) + " has " + std::to_string(slice_frames[sensor]) +
                     " frames in the slice, want at least 5 Hz");
            }
            if (slice_points[sensor] / slice_frames[sensor] < 1000) {
                fail(std::string(kNames[sensor]) + " mean points per frame are under 1000");
            }
            if (missing[sensor] > 5) {
                fail(std::string(kNames[sensor]) + " is missing " + std::to_string(missing[sensor]) +
                     " seconds inside the slice");
            }
        }

        const int loops = static_cast<int>((sim_s + span_s - 1) / span_s);
        std::printf("static_longrun recording=%s interval_s=%" PRId64 " loops=%d sim_floor_s=%" PRId64
                    " threads=%d positive=%d\n",
                    recording.string().c_str(), span_s, loops, sim_s, threads(), positive ? 1 : 0);
        std::printf("static_longrun slice mid360 frames=%" PRId64 " points=%" PRId64 " missing_s=%d jt128 frames=%" PRId64
                    " points=%" PRId64 " missing_s=%d\n",
                    slice_frames[0], slice_points[0], missing[0], slice_frames[1], slice_points[1], missing[1]);
        std::fflush(stdout);

        if (positive) {
            const Replay check = replay(loaded.frames, t0, kPositiveUntilS * kNs, 1);
            int before = 0;
            int from_change = 0;
            std::vector<Seen> early;
            for (const Seen& event : check.events) {
                if (event.t_s < kChangeFromS) {
                    ++before;
                    early.push_back(event);
                } else {
                    ++from_change;
                }
            }
            std::printf("static_longrun positive 0-%" PRId64 " s events_before_%" PRId64 "=%d events_from_%" PRId64
                        "=%d tracking_from_s=%" PRId64 "\n",
                        kPositiveUntilS, kChangeFromS, before, kChangeFromS, from_change, check.tracking_from_s);
            std::fflush(stdout);
            if (before != 0 || from_change < 1 || !check.tracking) {
                print_events(early.empty() ? check.events : early, 20);
                std::fflush(stdout);
                fail("known-positive pass on static5m did not see the 86 s motion with a clean prefix");
            }
        }

        const auto replay_at = std::chrono::steady_clock::now();
        const Replay run = replay(loaded.frames, t0, span_ns, loops);
        const double replay_seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - replay_at).count();
        const double wall = std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();
        const std::int64_t tracked = run.tracking_from_s < 0 ? 0 : run.closed_s - run.tracking_from_s;
        std::printf("static_longrun tracking_from_s=%" PRId64 " tracked_s=%" PRId64 " state=%s map_epoch=%" PRIu64
                    " map_messages=%" PRIu64 "\n",
                    run.tracking_from_s, tracked, run.tracking ? "TRACKING" : "BUILDING", run.map_epoch, run.map_messages);
        print_buckets(run);
        std::printf("static_longrun events=%zu closed_s=%" PRId64 " load_s=%.2f replay_s=%.2f wall_s=%.2f maxrss_mb=%.0f\n",
                    run.events.size(), run.closed_s, load_seconds, replay_seconds, wall, rss_mb());
        std::fflush(stdout);

        if (run.closed_s < sim_s) fail("closed simulated time is under the requested floor");
        if (run.closed_s != static_cast<std::int64_t>(loops) * span_s) fail("flush did not close the final window");
        if (!run.tracking || run.tracking_from_s < 1 || run.tracking_from_s > 45) {
            fail("tracking did not start within the reference window");
        }
        if (tracked + 45 < run.closed_s) fail("tracking exposure is shorter than the closed run minus 45 s");
        if (!run.events.empty()) {
            print_events(run.events, 20);
            std::fflush(stdout);
            fail("expected 0 events, got " + std::to_string(run.events.size()));
        }
        return 0;
    } catch (const Skip& skip) {
        std::printf("static_longrun: skip: %s\n", skip.what());
        return kSkip;
    } catch (const std::exception& error) {
        std::printf("static_longrun: FAIL: %s\n", error.what());
        std::fflush(stdout);
        return 1;
    }
}
#endif
