#include "scenes.hpp"

#include <set>
#include <tuple>
#include <utility>

namespace scenes {

namespace {

// numpy builds every constant as float32(<binary64 literal>); so do these.
constexpr float f32(double value) { return static_cast<float>(value); }
constexpr float kVoxel = f32(0.05);
constexpr float kGridStep = f32(0.02);
constexpr float kGridPhase = f32(0.007);

void add(Cloud& cloud, float x, float y, float z) {
    cloud.push_back(x);
    cloud.push_back(y);
    cloud.push_back(z);
}

std::vector<float> samples(float low, float high) {
    std::vector<float> values;
    for (int i = 0;; ++i) {
        const float value = (low + kGridPhase) + static_cast<float>(i) * kGridStep;
        if (value >= high) return values;
        values.push_back(value);
    }
}

Pair box_pair(bool before_box, bool after_box) {
    const Cloud room = room_shell();
    const Cloud box = box_shell({1.0, 0.5, 0.30});
    return {before_box ? concat({&room, &box}) : room, after_box ? concat({&room, &box}) : room};
}

Pair moved_pair() {
    const Cloud room = room_shell();
    const Cloud box = box_shell({0.0, 0.5, 0.30});
    const Cloud moved = shifted(box, 1);
    return {concat({&room, &box}), concat({&room, &moved})};
}

Pair jitter_pair() {
    const Cloud room = room_shell();
    const Cloud box = box_shell({1.0, 0.5, 0.30});
    const Cloud cloud = concat({&room, &box});
    return {jittered(cloud, 0x9E3779B97F4A7C15ULL), jittered(cloud, 0xD1B54A32D192ED03ULL)};
}

Pair clear_line_doubled() {
    const Cloud before = column({20, 21, 22, 23, 24, 25, 26, 27, 40});
    const Cloud after = column({40});
    return {concat({&before, &before}), concat({&after, &after})};
}

Pair board_pair() {
    Board board = board_scene();
    return {std::move(board.before), std::move(board.after)};
}

std::array<float, 3> unit_center(std::int64_t i, std::int64_t j, std::int64_t k) {
    return {static_cast<float>(i) + 0.5f, static_cast<float>(j) + 0.5f, static_cast<float>(k) + 0.5f};
}

Pair min_key_pair() {
    Cloud after;
    for (std::int64_t i = 0; i < 6; ++i) {
        const auto p = cell_point(i, 0, 0);
        add(after, p[0], p[1], p[2]);
    }
    for (std::int64_t j = 20; j < 26; ++j) {
        const auto p = cell_point(1, j, 0);
        add(after, p[0], p[1], p[2]);
    }
    return {{}, after};
}

const std::vector<std::string> kEvidence = {"--evidence", "--json"};

std::vector<std::string> with(std::vector<std::string> flags, const std::vector<std::string>& more) {
    flags.insert(flags.end(), more.begin(), more.end());
    return flags;
}

const std::vector<std::string> kThin = {"--min-extent", "1"};
const std::vector<std::string> kUnit = with({"--voxel-size", "1", "--origin", "0.5,0.5,0.5", "--evidence", "--tolerance",
                                             "0", "--min-voxels", "1", "--no-pair", "--unobserved", "--json"},
                                            kThin);

}  // namespace

Cloud concat(std::initializer_list<const Cloud*> parts) {
    Cloud out;
    for (const Cloud* part : parts) out.insert(out.end(), part->begin(), part->end());
    return out;
}

Cloud shuffled(const Cloud& points, std::uint64_t seed) {
    const std::size_t count = points.size() / 3;
    std::vector<std::size_t> order(count);
    for (std::size_t n = 0; n < count; ++n) order[n] = n;
    LCG rng{seed};
    std::size_t index = count;
    while (index > 1) {
        --index;
        const std::size_t swap = static_cast<std::size_t>(rng.next() % (index + 1));
        std::swap(order[index], order[swap]);
    }
    Cloud out;
    out.reserve(points.size());
    for (const std::size_t n : order) out.insert(out.end(), points.begin() + 3 * n, points.begin() + 3 * n + 3);
    return out;
}

Cloud room_shell() {
    const float x0 = f32(-1.5), x1 = f32(2.5), y0 = f32(-1.5), y1 = f32(2.5), z0 = f32(-0.7), z1 = f32(1.3);
    const auto xs = samples(x0, x1), ys = samples(y0, y1), zs = samples(z0, z1);
    Cloud points;
    const float floor_z = z0 + kGridPhase;
    for (const float x : xs) {
        for (const float y : ys) add(points, x, y, floor_z);
    }
    const float x_lo = x0 + kGridPhase, x_hi = x1 - kGridPhase;
    for (const float y : ys) {
        for (const float z : zs) {
            add(points, x_lo, y, z);
            add(points, x_hi, y, z);
        }
    }
    const float y_lo = y0 + kGridPhase, y_hi = y1 - kGridPhase;
    for (const float x : xs) {
        for (const float z : zs) {
            add(points, x, y_lo, z);
            add(points, x, y_hi, z);
        }
    }
    return points;
}

Cloud box_shell(std::array<double, 3> center, double side) {
    const float half = f32(side) * f32(0.5);
    const float cx = static_cast<float>(center[0]), cy = static_cast<float>(center[1]), cz = static_cast<float>(center[2]);
    const float x0 = cx - half, x1 = cx + half, y0 = cy - half, y1 = cy + half, z0 = cz - half, z1 = cz + half;
    const auto xs = samples(x0, x1), ys = samples(y0, y1), zs = samples(z0, z1);
    Cloud points;
    for (const float y : ys) {
        for (const float z : zs) {
            add(points, x0 + kGridPhase, y, z);
            add(points, x1 - kGridPhase, y, z);
        }
    }
    for (const float x : xs) {
        for (const float z : zs) {
            add(points, x, y0 + kGridPhase, z);
            add(points, x, y1 - kGridPhase, z);
        }
    }
    for (const float x : xs) {
        for (const float y : ys) {
            add(points, x, y, z0 + kGridPhase);
            add(points, x, y, z1 - kGridPhase);
        }
    }
    return points;
}

Cloud shifted(const Cloud& points, double dx) {
    Cloud out = points;
    for (std::size_t n = 0; n < out.size(); n += 3) out[n] = out[n] + static_cast<float>(dx);
    return out;
}

Cloud jittered(const Cloud& points, std::uint64_t seed) {
    LCG rng{seed};
    Cloud out;
    for (std::size_t n = 0; n < points.size(); n += 3) {
        const float draw = rng.unit();
        const float dx = (rng.unit() * f32(2) - f32(1)) * f32(0.01);
        const float dy = (rng.unit() * f32(2) - f32(1)) * f32(0.01);
        const float dz = (rng.unit() * f32(2) - f32(1)) * f32(0.01);
        if (draw < f32(0.20)) continue;
        add(out, points[n] + dx, points[n + 1] + dy, points[n + 2] + dz);
    }
    return out;
}

std::array<float, 3> cell_point(std::int64_t i, std::int64_t j, std::int64_t k) {
    const auto axis = [](std::int64_t index) { return (static_cast<float>(index) + 0.5f) * kVoxel; };
    return {axis(i), axis(j), axis(k)};
}

Cloud cells(std::initializer_list<std::array<std::int64_t, 3>> list) {
    Cloud out;
    for (const auto& c : list) {
        const auto p = cell_point(c[0], c[1], c[2]);
        add(out, p[0], p[1], p[2]);
    }
    return out;
}

Cloud column(const std::vector<std::int64_t>& indices) {
    Cloud out;
    for (const std::int64_t i : indices) {
        const auto p = cell_point(i, 0, 0);
        add(out, p[0], p[1], p[2]);
    }
    return out;
}

Cloud indoor(bool move_box_a) {
    std::vector<std::array<std::int64_t, 3>> list;
    for (std::int64_t i = -80; i < 80; ++i) {
        for (std::int64_t j = -80; j < 80; ++j) {
            for (std::int64_t k = -30; k < 30; ++k) {
                if (i == -80 || i == 79 || j == -80 || j == 79 || k == -30 || k == 29) list.push_back({i, j, k});
            }
        }
    }
    const auto box = [&](std::int64_t i0, std::int64_t i1, std::int64_t j0, std::int64_t j1, std::int64_t k0,
                         std::int64_t k1, std::int64_t shift) {
        for (std::int64_t i = i0; i <= i1; ++i) {
            for (std::int64_t j = j0; j <= j1; ++j) {
                for (std::int64_t k = k0; k <= k1; ++k) {
                    if (i == i0 || i == i1 || j == j0 || j == j1 || k == k0 || k == k1) list.push_back({i + shift, j, k});
                }
            }
        }
    };
    box(20, 27, -10, -3, -8, -1, move_box_a ? 20 : 0);
    box(-30, -23, 16, 23, -4, 3, 0);
    box(-6, 5, -28, -17, 4, 13, 0);
    Cloud out;
    for (const auto& c : list) {
        const auto p = cell_point(c[0], c[1], c[2]);
        add(out, p[0], p[1], p[2]);
    }
    return out;
}

Pair long_ray() {
    std::uint64_t state = 0x15A6;
    std::set<std::tuple<std::int64_t, std::int64_t, std::int64_t>> seen;
    Cloud before;
    while (before.size() < 3 * 80'000) {
        state = state * 6364136223846793005ULL + 1442695040888963407ULL;
        const auto i = static_cast<std::int64_t>(state % 599) - 299;
        const auto j = static_cast<std::int64_t>((state >> 21) % 599) - 299;
        const auto k = static_cast<std::int64_t>((state >> 42) % 599) - 299;
        if (i * i + j * j + k * k > 299 * 299 || !seen.insert({i, j, k}).second) continue;
        add(before, (static_cast<float>(i) + 0.5f) * kVoxel, (static_cast<float>(j) + 0.5f) * kVoxel,
            (static_cast<float>(k) + 0.5f) * kVoxel);
    }
    Cloud after = before;
    const float limit = 15.0f * 15.0f;
    for (std::size_t index = 0; index < 8'000; ++index) {
        const float x = after[3 * index] + 1.0f;
        const float y = after[3 * index + 1], z = after[3 * index + 2];
        if ((x * x + y * y) + z * z <= limit) after[3 * index] = x;
    }
    return {before, after};
}

Board board_scene() {
    const Cloud room = room_shell();
    const Cloud box = box_shell({1.0, 0.5, 0.30});
    const float board_x = f32(1.003);
    const auto ys = samples(f32(0.2), f32(0.8));
    const auto zs = samples(f32(0.0), f32(0.6));
    Cloud board;
    for (const float y : ys) {
        for (const float z : zs) add(board, board_x, y, z);
    }
    // The shadow test is binary64, as numpy promotes it there.
    const double y_lo = ys.front(), y_hi = ys.back(), z_lo = zs.front(), z_hi = zs.back();
    Cloud kept;
    for (std::size_t n = 0; n < room.size(); n += 3) {
        const double x = room[n];
        const double t = static_cast<double>(board_x) / x;
        const double y_hit = static_cast<double>(room[n + 1]) * t;
        const double z_hit = static_cast<double>(room[n + 2]) * t;
        const bool shadowed = x > static_cast<double>(board_x) && y_hit >= y_lo && y_hit <= y_hi && z_hit >= z_lo &&
                              z_hit <= z_hi;
        if (!shadowed) add(kept, room[n], room[n + 1], room[n + 2]);
    }
    Board out;
    out.before = concat({&room, &box});
    out.after = concat({&kept, &board});
    for (std::size_t n = 0; n < box.size(); n += 3) {
        if (box[n] > f32(1.15)) add(out.back, box[n], box[n + 1], box[n + 2]);
        if (box[n] < f32(0.85)) add(out.front, box[n], box[n + 1], box[n + 2]);
    }
    return out;
}

Pair boundary_pair(bool far) {
    const std::array<float, 3> endpoint = {16.0f, -15.0f, 0.5f};
    Cloud before, after;
    const auto put = [](Cloud& cloud, const std::array<float, 3>& p) { add(cloud, p[0], p[1], p[2]); };
    for (std::int64_t i = 0; i < 16; ++i) put(before, unit_center(i, -i, 0));
    put(before, unit_center(15, 0, 0));
    put(before, unit_center(16, -1, 0));
    put(before, unit_center(16, -16, 0));
    put(before, unit_center(21, -21, 0));
    put(before, endpoint);
    put(after, endpoint);
    if (far) {
        put(before, unit_center(700, 700, 700));
        put(after, unit_center(700, 700, 700));
    }
    return {before, after};
}

Pair speck_pair() {
    const Cloud room = room_shell();
    const Cloud box = box_shell({1.0, 0.5, 0.30});
    const Cloud speck = {f32(-0.8), f32(1.8), f32(0.413), f32(-0.8), f32(1.8), f32(0.463)};
    return {room, concat({&room, &box, &speck})};
}

Pair sheet_pair() { return {{}, cells({{0, 0, 0}, {1, 0, 0}, {2, 0, 0}, {3, 0, 0}, {4, 0, 0}, {1, 1, 0}, {2, 2, 0}})}; }

Pair small_box_pair(bool before_box, bool after_box) {
    const Cloud room = room_shell();
    const Cloud box = box_shell({1.0, 0.5, 0.30}, 0.10);
    return {before_box ? concat({&room, &box}) : room, after_box ? concat({&room, &box}) : room};
}

Pair small_moved_pair() {
    const Cloud room = room_shell();
    const Cloud box = box_shell({0.0, 0.5, 0.30}, 0.10);
    const Cloud moved = shifted(box, 1);
    return {concat({&room, &box}), concat({&room, &moved})};
}

const std::vector<GoldenCase>& golden_cases() {
    static const std::vector<GoldenCase> cases = {
        {"indoor-moved", [] { return Pair{indoor(false), indoor(true)}; }, kEvidence},
        {"box-added", [] { return box_pair(false, true); }, kEvidence},
        {"box-removed", [] { return box_pair(true, false); }, kEvidence},
        {"box-moved", moved_pair, kEvidence},
        {"box-moved-v1", moved_pair, {"--json"}},
        {"box-moved-unpaired-v1", moved_pair, {"--no-pair", "--json"}},
        {"jitter", jitter_pair, kEvidence},
        {"speck-min-voxels-1", speck_pair, with({"--min-voxels", "1", "--evidence", "--json"}, kThin)},
        {"speck-min-voxels-1-extent-2", speck_pair, {"--min-voxels", "1", "--evidence", "--json"}},
        {"sheet", sheet_pair, kEvidence},
        {"sheet-min-extent-1", sheet_pair, with({"--evidence", "--json"}, kThin)},
        {"shell-10cm-added", [] { return small_box_pair(false, true); }, kEvidence},
        {"shell-10cm-removed", [] { return small_box_pair(true, false); }, kEvidence},
        {"shell-10cm-moved", small_moved_pair, kEvidence},
        {"boundary-endpoint", [] { return boundary_pair(false); }, kUnit},
        {"boundary-endpoint-far", [] { return boundary_pair(true); }, kUnit},
        {"board-occluded", board_pair, {"--evidence", "--unobserved", "--json"}},
        {"occluded-column",
         [] { return Pair{column({8, 9, 10, 11, 12, 13, 14, 15, 16, 20, 21, 22, 23, 24, 25, 26, 27}), column({16})}; },
         with({"--evidence", "--tolerance", "0", "--unobserved", "--json"}, kThin)},
        {"clear-line-min-points-2", clear_line_doubled,
         with({"--evidence", "--tolerance", "0", "--min-points", "2", "--json"}, kThin)},
        {"diagonal-tie",
         [] { return Pair{cells({{0, 0, 0}, {1, 1, 1}, {2, 2, 2}, {1, 0, 0}, {3, 3, 3}}), cells({{3, 3, 3}})}; },
         with({"--evidence", "--tolerance", "0", "--min-voxels", "1", "--unobserved", "--origin", "0.025,0.025,0.025",
               "--json"},
              kThin)},
        {"slide-toward", [] { return Pair{column({20}), column({19})}; }, {"--evidence", "--min-voxels", "1", "--json"}},
        {"slide-away", [] { return Pair{column({20}), column({21})}; }, {"--evidence", "--min-voxels", "1", "--json"}},
        {"wide-grid", [] { return Pair{cells({{10, 10, 10}, {700, 700, 700}}), cells({{700, 700, 700}})}; },
         with({"--evidence", "--tolerance", "0", "--min-voxels", "1", "--json"}, kThin)},
        {"distance-tie", [] { return Pair{cells({{-20, 0, 0}, {20, 0, 0}}), cells({{0, 0, 0}})}; },
         with({"--tolerance", "0", "--min-voxels", "1", "--json"}, kThin)},
        {"min-key-order", min_key_pair, with({"--tolerance", "0", "--min-voxels", "1", "--no-pair", "--json"}, kThin)},
        {"long-ray-stride-16", long_ray, {"--evidence", "--max-range", "15", "--ray-stride", "16", "--json"}},
        {"long-ray-stride-64", long_ray, {"--evidence", "--max-range", "15", "--ray-stride", "64", "--json"}},
    };
    return cases;
}

const std::vector<Phase0Case>& phase0_cases() {
    static const std::vector<Phase0Case> cases = {
        {"phase0-static", "base_s1.bin", "base_s2.bin"},
        {"phase0-box_added", "base_s1.bin", "box_a.bin"},
        {"phase0-box_removed", "box_a.bin", "base_s2.bin"},
        {"phase0-box_moved", "box_a.bin", "box_b.bin"},
        {"phase0-person_walkthrough-0", "base_s1.bin", "person_0.bin"},
        {"phase0-person_walkthrough-1", "base_s1.bin", "person_1.bin"},
        {"phase0-person_walkthrough-2", "base_s1.bin", "person_2.bin"},
        {"phase0-person_walkthrough-3", "base_s1.bin", "person_3.bin"},
        {"phase0-person_walkthrough-4", "base_s1.bin", "person_4.bin"},
        {"phase0-rig_shifted", "base_s1.bin", "rig_shifted.bin"},
    };
    return cases;
}

}  // namespace scenes
