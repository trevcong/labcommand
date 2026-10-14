// room_memory_bag [key=value ...] < records: replays a recording through RoomMemory at its own
// timestamps, the way the node runs it, and reports every output message and its size.
//
// Records on stdin, little endian: u8 kind (0 CDR sensor_msgs/PointCloud2, 1 float32 x, y, z),
// u8 sensor (0 mid360, 1 jt128), u64 receive time (ns since the Unix epoch), u32 length, then
// the bytes. tools/bag_stream.py writes them from a rosbag2 mcap. Windows are window_s long
// from the first record. When a record falls past the current window, the window closes with
// its end time as the stamp: close_window, map_update, then status for every whole second
// boundary passed, as the node's timers do. The window still open when the records end is
// not closed: it holds only part of a window's frames.
//
// Output, one line each: window, event, map and status lines, then one JSON summary line.
#include <algorithm>
#include <chrono>
#include <cinttypes>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "config_keys.hpp"
#include "room_memory/memory.hpp"

namespace {

using room_memory::MemoryConfig;
using room_memory::RoomMemory;
namespace msg = room_memory::msg;

constexpr std::int64_t kSecond = 1'000'000'000;

msg::Time time_of(std::int64_t ns) { return msg::Time{ns / kSecond, ns % kSecond}; }

bool read_exact(void* out, std::size_t size) { return std::fread(out, 1, size, stdin) == size; }

template <typename T>
bool read_le(T& value) {
    std::uint8_t bytes[sizeof(T)];
    if (!read_exact(bytes, sizeof bytes)) return false;
    std::uint64_t v = 0;
    for (std::size_t n = 0; n < sizeof(T); ++n) v |= static_cast<std::uint64_t>(bytes[n]) << (8 * n);
    value = static_cast<T>(v);
    return true;
}

std::string optional_float(const std::optional<float>& value) {
    if (!value) return "nan";
    char text[32];
    std::snprintf(text, sizeof text, "%.4f", static_cast<double>(*value));
    return text;
}

struct Totals {
    std::uint64_t count = 0;
    std::uint64_t bytes = 0;
    void add(std::size_t size) {
        ++count;
        bytes += size;
    }
};

}  // namespace

int main(int argc, char** argv) {
    MemoryConfig config;
    config.state_dir = std::nullopt;
    try {
        for (int n = 1; n < argc; ++n) {
            const std::string pair = argv[n];
            const std::size_t eq = pair.find('=');
            if (eq == std::string::npos) throw std::runtime_error("expected key=value, got " + pair);
            room_memory::tools::configure(config, pair.substr(0, eq), pair.substr(eq + 1));
        }
    } catch (const std::exception& error) {
        std::cerr << "room_memory_bag: " << error.what() << "\n";
        return 2;
    }
    const char* names[] = {"mid360", "jt128"};
    const std::int64_t window_ns = static_cast<std::int64_t>(config.window_s * 1e9);
    std::unique_ptr<RoomMemory> memory;
    std::int64_t window_end = 0;
    std::int64_t next_status = 0;
    std::int64_t first_ns = 0;
    std::int64_t tracking_since = -1;
    std::int64_t windows = 0;
    std::int64_t frames[2] = {0, 0};
    std::uint64_t events_total = 0;
    Totals snapshots, deltas, statuses, events;
    std::vector<double> tracking_ms;
    std::vector<double> building_ms;

    const auto close_until = [&](std::int64_t ns) {
        while (ns >= window_end) {
            const msg::Time stamp = time_of(window_end);
            const auto start = std::chrono::steady_clock::now();
            const std::vector<msg::ChangeEvent> out = memory->close_window(stamp);
            const std::optional<msg::Bytes> map = memory->map_update(stamp);
            const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
            const bool tracking = memory->state() != msg::MemoryState::building;
            (tracking ? tracking_ms : building_ms).push_back(ms);
            if (tracking && tracking_since < 0) tracking_since = window_end;
            std::printf("window %" PRId64 " %.3f %s %.2f ms frames %" PRId64 " %" PRId64 "\n", windows,
                        static_cast<double>(window_end - first_ns) / 1e9, msg::name(memory->state()), ms, frames[0],
                        frames[1]);
            frames[0] = frames[1] = 0;
            for (const msg::ChangeEvent& e : out) {
                const std::size_t size = e.to_cdr().size();
                events.add(size);
                ++events_total;
                static const char* kinds[] = {"APPEARED", "DISAPPEARED", "MOVED"};
                static const char* states[] = {"CANDIDATE", "CONFIRMED", "ACCEPTED", "REVERTED", "EXPIRED"};
                std::printf("event %.3f %s %s %" PRIu64 " sensors %" PRId64 " voxels %" PRId64
                            " centroid %.3f %.3f %.3f %zu B\n",
                            static_cast<double>(window_end - first_ns) / 1e9, kinds[static_cast<int>(e.kind)],
                            states[static_cast<int>(e.state)], e.event_id & 0xFFFFF, e.sensors, e.voxel_count,
                            static_cast<double>(e.centroid[0]), static_cast<double>(e.centroid[1]),
                            static_cast<double>(e.centroid[2]), size);
            }
            if (map) {
                const msg::MapBricks decoded = msg::MapBricks::from_cdr(*map);
                (decoded.kind == msg::MapKind::snapshot ? snapshots : deltas).add(map->size());
                std::printf("map %.3f %s epoch %" PRIu64 " bricks %zu removed %zu %zu B\n",
                            static_cast<double>(window_end - first_ns) / 1e9,
                            decoded.kind == msg::MapKind::snapshot ? "SNAPSHOT" : "DELTA", decoded.header.map_epoch,
                            decoded.bricks.size(), decoded.removed.size(), map->size());
            }
            while (next_status <= window_end) {
                const msg::MemoryStatus status = memory->status(time_of(next_status), 1.0);
                const std::size_t size = status.to_cdr().size();
                statuses.add(size);
                std::printf("status %.3f %s bricks %" PRId64 " open %" PRId64 " coverage %s residual %s %zu B\n",
                            static_cast<double>(next_status - first_ns) / 1e9, msg::name(status.state), status.n_bricks,
                            status.n_open_events, optional_float(status.coverage_known_frac).c_str(),
                            optional_float(status.pose_residual_m).c_str(), size);
                next_status += kSecond;
            }
            window_end += window_ns;
            ++windows;
        }
    };

    std::vector<std::uint8_t> data;
    while (true) {
        std::uint8_t kind = 0, sensor = 0;
        std::uint64_t t = 0;
        std::uint32_t length = 0;
        if (!read_le(kind)) break;
        if (!read_le(sensor) || !read_le(t) || !read_le(length) || sensor > 1 || kind > 1) {
            std::cerr << "room_memory_bag: malformed record\n";
            return 2;
        }
        data.resize(length);
        if (!read_exact(data.data(), length)) {
            std::cerr << "room_memory_bag: truncated record\n";
            return 2;
        }
        const auto ns = static_cast<std::int64_t>(t);
        if (!memory) {
            try {
                memory = std::make_unique<RoomMemory>(config, time_of(ns));
            } catch (const std::exception& error) {
                std::cerr << "room_memory_bag: " << error.what() << "\n";
                return 2;
            }
            first_ns = ns;
            window_end = ns + window_ns;
            next_status = ns + kSecond;
        }
        close_until(ns);
        const std::string name = names[sensor];
        if (std::find(config.sensors.begin(), config.sensors.end(), name) == config.sensors.end()) continue;
        ++frames[sensor];
        if (kind == 0) {
            memory->add_cdr(name, data);
        } else {
            std::vector<float> xyz(length / 4);
            std::memcpy(xyz.data(), data.data(), xyz.size() * 4);
            memory->add_points(name, xyz);
        }
    }
    if (!memory) {
        std::cerr << "room_memory_bag: no records\n";
        return 2;
    }
    const auto median = [](std::vector<double> values) {
        if (values.empty()) return 0.0;
        std::sort(values.begin(), values.end());
        return values[values.size() / 2];
    };
    const auto maximum = [](const std::vector<double>& values) {
        return values.empty() ? 0.0 : *std::max_element(values.begin(), values.end());
    };
    const double duration = static_cast<double>(window_end - window_ns - first_ns) / 1e9;
    const double tracked = tracking_since < 0 ? 0.0 : static_cast<double>(window_end - window_ns - tracking_since) / 1e9;
    std::printf(
        "summary {\"duration_s\": %.3f, \"windows\": %" PRId64 ", \"tracking_from_s\": %.3f, \"tracked_s\": %.3f, "
        "\"events\": %" PRIu64 ", "
        "\"bytes\": {\"snapshot\": %" PRIu64 ", \"delta\": %" PRIu64 ", \"status\": %" PRIu64 ", \"events\": %" PRIu64
        "}, \"messages\": {\"snapshot\": %" PRIu64 ", \"delta\": %" PRIu64 ", \"status\": %" PRIu64
        ", \"events\": %" PRIu64 "}, \"window_ms\": {\"building_median\": %.2f, \"building_max\": %.2f, "
        "\"tracking_median\": %.2f, \"tracking_max\": %.2f}}\n",
        duration, windows, tracking_since < 0 ? -1.0 : static_cast<double>(tracking_since - first_ns) / 1e9, tracked,
        events_total, snapshots.bytes, deltas.bytes, statuses.bytes, events.bytes, snapshots.count,
        deltas.count, statuses.count, events.count, median(building_ms), maximum(building_ms), median(tracking_ms),
        maximum(tracking_ms));
    return 0;
}
