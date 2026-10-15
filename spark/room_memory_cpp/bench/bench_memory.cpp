// The realistic room_memory workload of spark/room_memory/bench/bench_memory.py, on the same
// frames: two sensors, 80,000 points per frame, 10 Hz, 1 s windows, ray evidence on.
//
//   bench_memory <frames_dir> [windows] [--full-diff]
//
// <frames_dir> comes from bench/dump_frames.py. When it holds python_events.hex, the event
// bytes of this run are compared with the Python engine's. Threads follow OMP_NUM_THREADS.
// --full-diff runs tracking windows with DiffScope::full instead of the engine's regions scope
// (same events; the rays then walk against every before-only voxel).
#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <stdexcept>
#include <cstdio>
#include <fstream>
#include <iterator>
#include <sstream>
#include <string>
#include <vector>

#if defined(_OPENMP)
#include <omp.h>
#endif

#include "room_memory/memory.hpp"

namespace {

using namespace room_memory;

constexpr std::size_t kHz = 10;
constexpr std::size_t kPoints = 80'000;

std::vector<float> read_floats(const std::string& path) {
    std::ifstream in(path, std::ios::binary);
    if (!in) throw std::runtime_error("cannot read " + path + " (run bench/dump_frames.py first)");
    std::vector<float> out;
    in.seekg(0, std::ios::end);
    out.resize(static_cast<std::size_t>(in.tellg()) / sizeof(float));
    in.seekg(0);
    in.read(reinterpret_cast<char*>(out.data()), static_cast<std::streamsize>(out.size() * sizeof(float)));
    return out;
}

std::string hex(const msg::Bytes& bytes) {
    static const char digits[] = "0123456789abcdef";
    std::string out;
    for (const std::uint8_t b : bytes) {
        out.push_back(digits[b >> 4]);
        out.push_back(digits[b & 15]);
    }
    return out;
}

double median(std::vector<double> values) {
    std::sort(values.begin(), values.end());
    const std::size_t n = values.size();
    return n % 2 ? values[n / 2] : (values[n / 2 - 1] + values[n / 2]) / 2;
}

}  // namespace

int main(int argc, char** argv) {
    std::vector<std::string> args(argv + 1, argv + argc);
    const auto flag = std::find(args.begin(), args.end(), "--full-diff");
    const bool full = flag != args.end();
    if (full) args.erase(flag);
    if (args.empty()) {
        std::fprintf(stderr, "bench_memory <frames_dir> [windows] [--full-diff]\n");
        return 2;
    }
    const std::string dir = args[0];
    const int windows = args.size() > 1 ? std::atoi(args[1].c_str()) : 6;
    const std::vector<float> pool[2] = {read_floats(dir + "/still.f32"), read_floats(dir + "/moved.f32")};
    for (const auto& frames : pool) {
        if (frames.size() != kHz * kPoints * 3) throw std::runtime_error("unexpected frame file size");
    }

    MemoryConfig config;
    // One reference window settles everything it saw, and every region is sent at once, so
    // tracking starts at window 1 with the benchmark's two events; the recent layer and the
    // per-window learning are the node's.
    config.reference_windows = 1;
    config.occupied_min_hits = 1;
    config.occupied_min_ratio = 0.0;
    config.free_min_seen = 1;
    config.candidate_windows = 1;
    config.expire_windows = 1;
    config.state_dir = std::nullopt;
    // The indoor move lies within the default near field.
    config.tolerance = 1;
    config.min_voxels = 6;
    config.near_range_m = 0.0;
    RoomMemory memory(config, msg::Time{0, 0}, nullptr, full ? DiffScope::full : DiffScope::regions);
    std::vector<double> add_ms, close_ms;
    double reference_ms = 0;
    std::vector<std::string> events;
    using clock = std::chrono::steady_clock;
    for (int window = 0; window < windows; ++window) {
        const std::vector<float>& scene = pool[window < 2 ? 0 : 1];
        const auto start = clock::now();
        for (std::size_t frame = 0; frame < kHz; ++frame) {
            const std::span<const float> points(scene.data() + frame * kPoints * 3, kPoints * 3);
            memory.add_points("mid360", points);
            memory.add_points("jt128", points);
        }
        const auto mid = clock::now();
        const std::vector<msg::ChangeEvent> out = memory.close_window(msg::Time{window + 1, 0});
        const auto end = clock::now();
        for (const msg::ChangeEvent& event : out) events.push_back(hex(event.to_cdr()));
        if (window >= 1) {  // skip the reference-building window
            add_ms.push_back(std::chrono::duration<double, std::milli>(mid - start).count());
            close_ms.push_back(std::chrono::duration<double, std::milli>(end - mid).count());
        } else {
            reference_ms = std::chrono::duration<double, std::milli>(end - start).count();
        }
    }
    std::vector<double> per_window(add_ms.size());
    for (std::size_t n = 0; n < add_ms.size(); ++n) per_window[n] = add_ms[n] + close_ms[n];
    const double frames = 2.0 * kHz;
    int threads = 1;
#if defined(_OPENMP)
    threads = omp_get_max_threads();
#endif
    std::printf("threads: %d  diff scope: %s\n", threads, full ? "full" : "regions");
    std::printf("windows timed: %zu  frames/window: %d x %zu pts  events: %zu\n", per_window.size(),
                static_cast<int>(frames), kPoints, events.size());
    std::printf("close_window ms: median %.1f  max %.1f\n", median(close_ms),
                *std::max_element(close_ms.begin(), close_ms.end()));
    std::printf("per-window ms (1 s of data): median %.1f  max %.1f\n", median(per_window),
                *std::max_element(per_window.begin(), per_window.end()));
    std::printf("ms per frame: %.2f\n", median(per_window) / frames);
    std::printf("ms per 10 Hz tick (both sensors): %.2f\n", 2 * median(per_window) / frames);
    std::printf("reference-building window ms (both sensors, rays from every voxel): %.1f\n", reference_ms);

    std::ifstream reference(dir + "/python_events.hex");
    if (reference && windows == 6) {
        std::vector<std::string> expected;
        for (std::string line; std::getline(reference, line);) {
            if (!line.empty()) expected.push_back(line);
        }
        const bool same = expected == events;
        std::printf("event bytes vs Python: %s (%zu events)\n", same ? "identical" : "DIFFERENT", expected.size());
        return same ? 0 : 1;
    }
    return 0;
}
