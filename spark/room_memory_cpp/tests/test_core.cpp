// Core behavior, ported from spark/room_memory/tests/test_change_detection.py, plus the C++
// specifics: thread-count determinism, the regions-only diff scope, summary merging, and the
// key containers.
#include <algorithm>
#include <cmath>
#include <map>
#include <random>

#if defined(_OPENMP)
#include <omp.h>
#endif

#include "harness.hpp"
#include "offline.hpp"
#include "room_memory/change_detection.hpp"
#include "room_memory/key_set.hpp"
#include "room_memory/pointcloud2.hpp"
#include "scenes.hpp"

using namespace room_memory;
using scenes::Cloud;

namespace {

Config evidence() {
    Config c;
    c.evidence_of_absence = true;
    c.tolerance = 0;
    c.min_extent = 1;
    return c;
}

Config config_for(std::vector<std::string> flags) {
    flags.push_back("before.bin");
    flags.push_back("after.bin");
    return offline::parse_args(flags, false, false)->config;
}

std::string record(const Cloud& before, const Cloud& after, const Config& config) {
    const Comparison c = compare(before, after, config);
    return to_json(c.report, c.before, c.after, config);
}

Keys keys_of(const Cloud& points, const Config& config = Config()) { return occupancy_map(points, config).occupied; }

std::vector<Region::Kind> kinds(const Report& report) {
    std::vector<Region::Kind> out;
    for (const Region& r : report.regions) out.push_back(r.kind);
    return out;
}

Keys concat_keys(const Report& report, Region::Kind kind) {
    Keys out;
    for (const Region& r : report.regions) {
        if (r.kind == kind) out.insert(out.end(), r.keys.begin(), r.keys.end());
    }
    std::sort(out.begin(), out.end());
    return out;
}

void expect_box(const Region& region, Region::Kind kind, std::array<float, 3> center) {
    CHECK(region.kind == kind && region.voxel_count > 6);
    for (int a = 0; a < 3; ++a) CHECK(std::fabs(region.centroid[a] - center[a]) <= 0.025f);
    const float v = 0.05f;
    CHECK(region.volume == static_cast<float>(region.voxel_count) * v * v * v);
    for (int a = 0; a < 3; ++a) {
        const float span = region.bounds_max[a] - region.bounds_min[a];
        CHECK(span >= 0.399f && span < 0.70f);
    }
}

void set_threads(int threads) {
#if defined(_OPENMP)
    omp_set_num_threads(threads);
#else
    (void)threads;
#endif
}

int max_threads() {
#if defined(_OPENMP)
    return omp_get_num_procs();
#else
    return 1;
#endif
}

}  // namespace

// MARK: Keys and occupancy

TEST(key_packing) {
    const std::vector<Index3> samples = {
        {0, 0, 0}, {1, -2, 3}, {kLimit - 1, 0, -kLimit}, {-kLimit, kLimit - 1, 4}, {-kLimit, -kLimit, -kLimit}};
    Keys keys;
    for (const Index3& s : samples) {
        const Key key = pack(s.i, s.j, s.k);
        CHECK(key >= 0);
        CHECK(unpack(key) == s);
        keys.push_back(key);
    }
    std::vector<std::size_t> by_key(samples.size()), by_lex(samples.size());
    for (std::size_t n = 0; n < samples.size(); ++n) by_key[n] = by_lex[n] = n;
    std::sort(by_key.begin(), by_key.end(), [&](auto a, auto b) { return keys[a] < keys[b]; });
    std::sort(by_lex.begin(), by_lex.end(), [&](auto a, auto b) {
        return std::tie(samples[a].i, samples[a].j, samples[a].k) < std::tie(samples[b].i, samples[b].j, samples[b].k);
    });
    CHECK(by_key == by_lex);
    CHECK_EQ(pack(20, 0, 0), Key{4611776178381914112});
}

TEST(floor_quantizer) {
    const float values[] = {-0.01f, 0.06f, -0.0f, -0.05f, 0.05f};
    const std::int64_t expected[] = {-1, 1, 0, -1, 1};
    for (int n = 0; n < 5; ++n) {
        CHECK_EQ(unpack(keys_of({values[n], 0, 0})[0]).i, expected[n]);
    }
    Config coarse;
    coarse.voxel_size = static_cast<float>(0.1);
    CHECK_EQ(unpack(keys_of({0.06f, 0, 0}, coarse)[0]).i, std::int64_t{0});
}

TEST(drops_non_finite_far_and_out_of_limit) {
    const float nan = std::nanf(""), inf = INFINITY;
    const OccupancyMap map = occupancy_map(Cloud{nan, 0, 0, 0.06f, -0.01f, 0, 0, inf, 1, 60000, 0, 0}, Config());
    CHECK_EQ(map.accepted_points, std::int64_t{1});
    CHECK_EQ(map.dropped_points, std::int64_t{3});
    CHECK(map.occupied == Keys{pack(1, -1, 0)});
    const Cloud near = {14.999f, 0, 0, 15.001f, 0, 0};
    Config clip;
    clip.max_range = 15.0f;
    const OccupancyMap clipped = occupancy_map(near, clip);
    CHECK(clipped.accepted_points == 1 && clipped.dropped_points == 1 && clipped.occupied.size() == 1);
    CHECK_EQ(occupancy_map(near, Config()).occupied.size(), std::size_t{2});
}

TEST(quantizer_lanes_match_the_scalar_tail) {
    // 4-wide SIMD lanes and the scalar tail must quantize identically, including drops.
    std::mt19937 rng(5);
    std::uniform_real_distribution<float> coordinate(-20.0f, 20.0f);
    Cloud points;
    for (int n = 0; n < 4003; ++n) {
        for (int a = 0; a < 3; ++a) points.push_back(coordinate(rng));
    }
    points[7] = std::nanf("");
    points[20] = INFINITY;
    Config config;
    config.max_range = 15.0f;
    const OccupancyMap all = occupancy_map(points, config);
    Keys one_by_one;
    std::int64_t accepted = 0;
    for (std::size_t n = 0; n < points.size(); n += 3) {
        const OccupancyMap single = occupancy_map(std::span<const float>(points).subspan(n, 3), config);
        accepted += single.accepted_points;
        one_by_one.insert(one_by_one.end(), single.occupied.begin(), single.occupied.end());
    }
    sort_unique(one_by_one);
    CHECK(all.occupied == one_by_one);
    CHECK_EQ(all.accepted_points, accepted);
}

TEST(representative_return_is_the_binary32_order_minimum) {
    const Cloud points = {0.02f, 0.0f,  0.0f,  0.01f, 0.03f, 0.04f,  0.01f, 0.02f, 0.045f, 0.01f, 0.02f,
                          0.041f, 0.0f, 0.26f, 0.26f, -0.0f, 0.26f, 0.26f, -0.01f, 0.26f, 0.26f};
    const auto pick = [&](std::vector<int> order) {
        Cloud out;
        for (int n : order) out.insert(out.end(), points.begin() + 3 * n, points.begin() + 3 * n + 3);
        return out;
    };
    Config config;
    config.evidence_of_absence = true;
    for (const auto& order : {std::vector<int>{0, 1, 2, 3, 4, 5, 6}, std::vector<int>{5, 3, 6, 1, 4, 0, 2}}) {
        const OccupancyMap map = occupancy_map(pick(order), config);
        CHECK(map.occupied == (Keys{pack(-1, 5, 5), pack(0, 0, 0), pack(0, 5, 5)}));
        const Cloud expected = pick({6, 3, 5});
        for (std::size_t n = 0; n < expected.size(); ++n) CHECK(float_bits((*map.endpoints)[n]) == float_bits(expected[n]));
    }
}

TEST(min_points) {
    const Cloud piled = {0.01f, 0.01f, 0.01f, 0.02f, 0.01f, 0.01f, 0.01f, 0.02f, 0.01f,
                         0.01f, 0.01f, 0.02f, 0.06f, 0.01f, 0.01f};
    Config config;
    config.min_points = 2;
    const OccupancyMap kept = occupancy_map(piled, config);
    CHECK(kept.occupied.size() == 1 && kept.accepted_points == 5);
}

TEST(invalid_config) {
    std::vector<Config> bad(11);
    bad[0].voxel_size = 0;
    bad[1].max_range = -1.0f;
    bad[2].min_points = 0;
    bad[3].min_voxels = 0;
    bad[4].min_extent = 0;
    bad[5].tolerance = 5;
    bad[6].max_move = std::nanf("");
    bad[7].move_count_ratio = 0.5f;
    bad[8].ray_stride = 3;
    bad[9].origin = {std::nanf(""), 0, 0};
    bad[10].evidence_of_absence = true;
    bad[10].origin = {80000, 0, 0};
    for (const Config& config : bad) CHECK_THROWS(InvalidConfig, occupancy_map(Cloud{}, config));
}

TEST(voxel_size_mismatch_and_missing_endpoints) {
    const Cloud room = scenes::room_shell();
    Config coarse_config;
    coarse_config.voxel_size = static_cast<float>(0.10);
    const OccupancyMap fine = occupancy_map(room, Config());
    const OccupancyMap coarse = occupancy_map(room, coarse_config);
    CHECK_THROWS(VoxelSizeMismatch, diff(fine, coarse, Config()));
    Config with_evidence;
    with_evidence.evidence_of_absence = true;
    CHECK_THROWS(InvalidConfig, diff(fine, fine, with_evidence));
}

// MARK: Synthetic scenes

TEST(box_added_removed_moved) {
    for (const bool on : {false, true}) {
        Config config;
        config.evidence_of_absence = on;
        const Cloud room = scenes::room_shell();
        const Cloud box = scenes::box_shell({1.0, 0.5, 0.30});
        const Cloud with_box = scenes::concat({&room, &box});
        const Report added = compare(room, with_box, config).report;
        CHECK(added.regions.size() == 1 && added.disappeared_voxels == 0);
        expect_box(added.regions[0], Region::Kind::appeared, {1.0f, 0.5f, 0.30f});
        const Report removed = compare(with_box, room, config).report;
        CHECK(removed.regions.size() == 1 && removed.appeared_voxels == 0);
        expect_box(removed.regions[0], Region::Kind::disappeared, {1.0f, 0.5f, 0.30f});
        const Cloud start = scenes::box_shell({0.0, 0.5, 0.30});
        const Cloud end = scenes::shifted(start, 1);
        const Report moved = compare(scenes::concat({&room, &start}), scenes::concat({&room, &end}), config).report;
        CHECK(kinds(moved) == std::vector<Region::Kind>{Region::Kind::moved});
        expect_box(moved.regions[0], Region::Kind::moved, {1.0f, 0.5f, 0.30f});
        if (!on) {
            CHECK_EQ(moved.regions[0].from_voxel_count, moved.regions[0].voxel_count);
            const std::array<float, 3> from = {0.0f, 0.5f, 0.3f};
            for (int a = 0; a < 3; ++a) CHECK(std::fabs(moved.regions[0].from_centroid[a] - from[a]) <= 0.025f);
        }
    }
}

TEST(box_removed_without_line_of_sight_is_unknown) {
    const scenes::Board board = scenes::board_scene();
    Config config;
    config.evidence_of_absence = true;
    config.report_unobserved = true;
    const Report report = compare(board.before, board.after, config).report;
    const Keys back = keys_of(board.back), front = keys_of(board.front);
    const Keys disappeared = concat_keys(report, Region::Kind::disappeared);
    const Keys unobserved = concat_keys(report, Region::Kind::unobserved);
    CHECK(!intersects(disappeared, back));
    CHECK_EQ(count_present(unobserved, back), back.size());
    CHECK(*report.unknown_voxels >= static_cast<std::int64_t>(back.size()));
    CHECK(static_cast<double>(count_present(disappeared, front)) / static_cast<double>(front.size()) > 0.95);
    for (const Region& r : report.regions) CHECK(r.kind != Region::Kind::moved);
}

TEST(occluded_column) {
    const Cloud before = scenes::column({8, 9, 10, 11, 12, 13, 14, 15, 16, 20, 21, 22, 23, 24, 25, 26, 27});
    const Report report = compare(before, scenes::column({16}), evidence()).report;
    CHECK(report.appeared_voxels == 0 && report.disappeared_voxels == 8 && *report.unknown_voxels == 8);
    CHECK(report.regions.size() == 1 && report.regions[0].kind == Region::Kind::disappeared &&
          report.regions[0].voxel_count == 8);
    CHECK(report.regions[0].bounds_min[0] == 8.0f * 0.05f);
    Config full_config = evidence();
    full_config.report_unobserved = true;
    const Report full = compare(before, scenes::column({16}), full_config).report;
    CHECK(kinds(full) == (std::vector<Region::Kind>{Region::Kind::disappeared, Region::Kind::unobserved}));
    CHECK(full.regions[1].voxel_count == 8 && full.regions[1].bounds_min[0] == 20.0f * 0.05f);
}

TEST(diagonal_tie_steps_all_axes) {
    const Cloud before = scenes::cells({{0, 0, 0}, {1, 1, 1}, {2, 2, 2}, {1, 0, 0}, {3, 3, 3}});
    Config config = evidence();
    config.min_voxels = 1;
    config.report_unobserved = true;
    config.origin = scenes::cell_point(0, 0, 0);
    const Report report = compare(before, scenes::cells({{3, 3, 3}}), config).report;
    CHECK(report.disappeared_voxels == 3 && *report.unknown_voxels == 1);
    CHECK(kinds(report) == (std::vector<Region::Kind>{Region::Kind::disappeared, Region::Kind::unobserved}));
    CHECK(report.regions[0].voxel_count == 3 && report.regions[1].voxel_count == 1);
    CHECK_EQ(report.regions[1].min_key, pack(1, 0, 0));
}

TEST(slides) {
    Config config = evidence();
    config.tolerance = 1;
    config.min_voxels = 1;
    const Report toward = compare(scenes::column({20}), scenes::column({19}), config).report;
    CHECK(*toward.unknown_voxels == 1 && toward.disappeared_voxels == 0 && toward.appeared_voxels == 0);
    const Report away = compare(scenes::column({20}), scenes::column({21}), config).report;
    CHECK(*away.unknown_voxels == 0 && away.disappeared_voxels == 0 && away.appeared_voxels == 0);
}

TEST(order_and_tie_rules) {
    Cloud after;
    for (int i = 0; i < 6; ++i) {
        const auto p = scenes::cell_point(i, 0, 0);
        after.insert(after.end(), p.begin(), p.end());
    }
    for (int j = 20; j < 26; ++j) {
        const auto p = scenes::cell_point(1, j, 0);
        after.insert(after.end(), p.begin(), p.end());
    }
    Config config;
    config.tolerance = 0;
    config.min_voxels = 1;
    config.min_extent = 1;
    config.pair_moves = false;
    const Report report = compare(Cloud{}, after, config).report;
    CHECK(report.regions[0].min_key < report.regions[1].min_key);
    CHECK(report.regions[0].centroid[0] > report.regions[1].centroid[0]);
    config.pair_moves = true;
    const Report tied = compare(scenes::cells({{-20, 0, 0}, {20, 0, 0}}), scenes::cells({{0, 0, 0}}), config).report;
    std::vector<std::string> names;
    for (const Region& r : tied.regions) names.push_back(kind_name(r.kind));
    std::sort(names.begin(), names.end());
    CHECK(names == (std::vector<std::string>{"appeared", "disappeared", "disappeared"}));
}

TEST(threshold_counts_the_dropped_speck) {
    const scenes::Pair pair = scenes::speck_pair();
    Config loose_config;
    loose_config.min_voxels = 1;
    loose_config.min_extent = 1;
    Config thin_config;
    thin_config.min_voxels = 1;
    const Report tight = compare(pair.before, pair.after, Config()).report;
    const Report loose = compare(pair.before, pair.after, loose_config).report;
    const Report thin = compare(pair.before, pair.after, thin_config).report;
    CHECK(kinds(tight) == std::vector<Region::Kind>{Region::Kind::appeared});
    CHECK_EQ(loose.regions[1].voxel_count, std::int64_t{2});
    CHECK(thin.regions.size() == 1 && thin.regions[0].voxel_count == tight.regions[0].voxel_count);
    CHECK(tight.appeared_voxels == loose.appeared_voxels && loose.appeared_voxels == thin.appeared_voxels);
    CHECK_EQ(tight.appeared_voxels, tight.regions[0].voxel_count + 2);
}

TEST(extent_gate) {
    const scenes::Pair sheet = scenes::sheet_pair();
    const Report streak = compare(sheet.before, sheet.after, Config()).report;
    CHECK(streak.appeared_voxels == 7 && streak.regions.empty());
    Config thin;
    thin.min_extent = 1;
    const Report kept = compare(sheet.before, sheet.after, thin).report;
    CHECK(kept.regions.size() == 1 && kept.regions[0].kind == Region::Kind::appeared && kept.regions[0].voxel_count == 7);
    const std::pair<scenes::Pair, Region::Kind> cases[] = {
        {scenes::small_box_pair(false, true), Region::Kind::appeared},
        {scenes::small_box_pair(true, false), Region::Kind::disappeared},
        {scenes::small_moved_pair(), Region::Kind::moved},
    };
    for (const auto& [pair, kind] : cases) {
        const Report report = compare(pair.before, pair.after, Config()).report;
        CHECK(report.regions.size() == 1 && report.regions[0].kind == kind && report.regions[0].voxel_count == 8);
        for (int a = 0; a < 3; ++a) {
            CHECK(std::fabs(report.regions[0].bounds_max[a] - report.regions[0].bounds_min[a] - 0.10f) <= 0.001f);
        }
        if (kind == Region::Kind::moved) CHECK_EQ(report.regions[0].from_voxel_count, std::int64_t{8});
    }
}

TEST(long_ray_stride_table) {
    const scenes::Pair pair = scenes::long_ray();
    Config config;
    config.evidence_of_absence = true;
    config.max_range = 15.0f;
    const OccupancyMap occupancy = occupancy_map(pair.before, config);
    const std::map<std::int64_t, std::pair<std::size_t, double>> expected = {
        {16, {5014, 1.68e6}}, {32, {2586, 8.74e5}}, {64, {1275, 4.32e5}}};
    for (const auto& [stride, values] : expected) {
        config.ray_stride = stride;
        const RayEndpoints rays = ray_endpoints(occupancy, config);
        CHECK_EQ(rays.keys.size(), values.first);
        double manhattan = 0;
        for (const Key key : rays.keys) {
            const Index3 v = unpack(key);
            manhattan += static_cast<double>(std::llabs(v.i) + std::llabs(v.j) + std::llabs(v.k));
        }
        CHECK(std::fabs(manhattan - values.second) <= 0.005 * values.second);
    }
}

TEST(boundary_endpoint_stops_on_the_end_cell) {
    const Config config = config_for({"--voxel-size", "1", "--origin", "0.5,0.5,0.5", "--evidence", "--tolerance", "0",
                                      "--min-voxels", "1", "--no-pair", "--unobserved", "--json", "--min-extent", "1"});
    for (const bool far : {false, true}) {
        const scenes::Pair pair = scenes::boundary_pair(far);
        const Report report = compare(pair.before, pair.after, config).report;
        CHECK(report.appeared_voxels == 0 && report.disappeared_voxels == 16 && *report.unknown_voxels == 4);
        const Region& region = report.regions[0];
        CHECK(region.kind == Region::Kind::disappeared && region.voxel_count == 16);
        CHECK_EQ(region.min_key, pack(0, 0, 0));
        CHECK((region.bounds_min == Vec3{0, -15, 0}) && (region.bounds_max == Vec3{16, 1, 1}));
    }
}

TEST(boundary_octants_stay_inside_the_origin_end_box) {
    const Config config = config_for({"--voxel-size", "1", "--origin", "0.5,0.5,0.5", "--evidence", "--tolerance", "0",
                                      "--min-voxels", "1", "--no-pair", "--unobserved", "--json", "--min-extent", "1"});
    scenes::LCG rng{0xB0A1DDA5};
    int samples = 0;
    for (const int sx : {-1, 1}) {
        for (const int sy : {-1, 1}) {
            for (const int sz : {-1, 1}) {
                for (int mask = 1; mask < 8; ++mask) {
                    std::int64_t end[3];
                    const int signs[3] = {sx, sy, sz};
                    for (int a = 0; a < 3; ++a) end[a] = signs[a] * (static_cast<std::int64_t>(rng.next() % 8) + 1);
                    Cloud point;
                    for (int a = 0; a < 3; ++a) {
                        point.push_back(mask & (1 << a) ? static_cast<float>(end[a]) : static_cast<float>(end[a]) + 0.5f);
                    }
                    std::int64_t low[3], high[3];
                    for (int a = 0; a < 3; ++a) {
                        low[a] = std::min<std::int64_t>(0, end[a]);
                        high[a] = std::max<std::int64_t>(0, end[a]);
                    }
                    Cloud before;
                    for (std::int64_t i = low[0] - 1; i <= high[0] + 1; ++i) {
                        for (std::int64_t j = low[1] - 1; j <= high[1] + 1; ++j) {
                            for (std::int64_t k = low[2] - 1; k <= high[2] + 1; ++k) {
                                before.insert(before.end(), {static_cast<float>(i) + 0.5f, static_cast<float>(j) + 0.5f,
                                                             static_cast<float>(k) + 0.5f});
                            }
                        }
                    }
                    const Report report = compare(before, point, config).report;
                    const std::int64_t cells = (high[0] - low[0] + 1) * (high[1] - low[1] + 1) * (high[2] - low[2] + 1);
                    const std::int64_t longest = std::max({std::llabs(end[0]), std::llabs(end[1]), std::llabs(end[2])});
                    CHECK(longest <= report.disappeared_voxels && report.disappeared_voxels <= cells - 1);
                    std::int64_t nearest = -1;
                    for (const Region& region : report.regions) {
                        if (region.kind != Region::Kind::disappeared) continue;
                        std::int64_t gap = 0;
                        for (int a = 0; a < 3; ++a) {
                            const auto lo = static_cast<std::int64_t>(region.bounds_min[a]);
                            const auto hi = static_cast<std::int64_t>(region.bounds_max[a]) - 1;
                            CHECK(low[a] <= lo && hi <= high[a]);
                            gap = std::max({gap, lo - end[a], end[a] - hi, std::int64_t{0}});
                        }
                        nearest = nearest < 0 ? gap : std::min(nearest, gap);
                    }
                    CHECK(nearest >= 0 && nearest <= 1);
                    ++samples;
                }
            }
        }
    }
    CHECK_EQ(samples, 56);
}

// MARK: Determinism

TEST(shuffled_and_duplicated_input_keeps_the_bytes) {
    const Cloud column = scenes::column({20, 21, 22, 23, 24, 25, 26, 27, 40});
    const Cloud end = scenes::column({40});
    const Cloud before = scenes::concat({&column, &column});
    const Cloud after = scenes::concat({&end, &end});
    const std::string original = record(before, after, evidence());
    CHECK(record(scenes::shuffled(before, 0xC0FFEE), scenes::shuffled(after, 0xC0FFEE), evidence()) == original);
    Config doubled = evidence();
    doubled.min_points = 2;
    std::string expected = original;
    expected.replace(expected.find("\"min_points\":1"), 14, "\"min_points\":2");
    CHECK(record(before, after, doubled) == expected);

    const Cloud room = scenes::room_shell();
    const Cloud box = scenes::box_shell({0.0, 0.5, 0.30});
    const Cloud moved_box = scenes::shifted(box, 1);
    const Cloud moved_before = scenes::concat({&room, &box});
    const Cloud moved_after = scenes::concat({&room, &moved_box});
    Config config;
    config.evidence_of_absence = true;
    CHECK(record(scenes::shuffled(moved_before, 0xC0FFEE), scenes::shuffled(moved_after, 0xC0FFEE), config) ==
          record(moved_before, moved_after, config));
}

TEST(results_do_not_depend_on_the_thread_count) {
    const int most = max_threads();
    const std::vector<std::string> names = {"indoor-moved", "jitter", "board-occluded", "long-ray-stride-16"};
    for (const scenes::GoldenCase& c : scenes::golden_cases()) {
        if (std::find(names.begin(), names.end(), c.name) == names.end()) continue;
        const scenes::Pair pair = c.build();
        const Config config = config_for(c.flags);
        std::string first;
        Keys first_marks;
        for (const int threads : {1, 2, 5, most}) {
            set_threads(threads);
            const std::string text = record(pair.before, pair.after, config);
            const Keys marks = cast_rays(occupancy_map(pair.after, config), config);
            if (first.empty()) {
                first = text;
                first_marks = marks;
            }
            CHECK(text == first);
            CHECK(marks == first_marks);
        }
    }
    set_threads(most);
}

TEST(regions_scope_matches_the_full_diff) {
    // Dilating the before-only candidates ahead of the rays is exact for regions and the
    // appeared/disappeared counts; only unknown_voxels is left out.
    for (const scenes::GoldenCase& c : scenes::golden_cases()) {
        const scenes::Pair pair = c.build();
        Config config = config_for(c.flags);
        config.report_unobserved = false;
        const OccupancyMap before = occupancy_map(pair.before, config);
        const OccupancyMap after = occupancy_map(pair.after, config);
        const Report full = diff(before, after, config);
        DiffOptions options;
        options.scope = DiffScope::regions;
        const Report regions = diff(before, after, config, options);
        CHECK(!regions.unknown_voxels);
        CHECK_EQ(regions.appeared_voxels, full.appeared_voxels);
        CHECK_EQ(regions.disappeared_voxels, full.disappeared_voxels);
        CHECK_EQ(regions.regions.size(), full.regions.size());
        for (std::size_t n = 0; n < full.regions.size(); ++n) {
            const Region& a = regions.regions[n];
            const Region& b = full.regions[n];
            CHECK(a.kind == b.kind && a.keys == b.keys && a.from_keys == b.from_keys && a.centroid == b.centroid &&
                  a.from_centroid == b.from_centroid && a.volume == b.volume);
        }
    }
}

// MARK: Containers

TEST(summaries_merge_into_the_summary_of_the_union) {
    std::mt19937 rng(11);
    std::normal_distribution<float> noise(0.0f, 0.4f);
    Config config;
    config.evidence_of_absence = true;
    Cloud parts[3];
    for (Cloud& part : parts) {
        for (int n = 0; n < 30000; ++n) {
            for (int a = 0; a < 3; ++a) part.push_back(noise(rng));
        }
    }
    const Cloud all = scenes::concat({&parts[0], &parts[1], &parts[2]});
    const CellSummary whole = summarize(all, config);
    const CellSummary a = summarize(parts[0], config), b = summarize(parts[1], config), c = summarize(parts[2], config);
    const CellSummary empty;
    const CellSummary* list[] = {&a, &empty, &b, &c};
    const CellSummary merged = merge(list);
    CHECK(merged.keys == whole.keys && merged.counts == whole.counts && merged.order_keys == whole.order_keys);
    CHECK_EQ(merged.accepted_points, whole.accepted_points);
    for (const std::int64_t min_points : {1, 2, 3}) {
        config.min_points = min_points;
        const OccupancyMap x = occupancy_from(merged, config), y = occupancy_map(all, config);
        CHECK(x.occupied == y.occupied && *x.endpoints == *y.endpoints);
    }
}

TEST(key_sort_and_membership) {
    std::mt19937_64 rng(3);
    std::uniform_int_distribution<std::int64_t> lane(-600, 600);
    Keys keys;
    for (int n = 0; n < 200000; ++n) keys.push_back(pack(lane(rng), lane(rng) / 7, lane(rng) / 50));
    Keys expected = keys;
    std::sort(expected.begin(), expected.end());
    Keys sorted = keys;
    sort_keys(sorted);
    CHECK(sorted == expected);
    const std::vector<std::uint32_t> order = sort_order(keys);
    for (std::size_t n = 1; n < order.size(); ++n) {
        CHECK(keys[order[n - 1]] < keys[order[n]] || (keys[order[n - 1]] == keys[order[n]] && order[n - 1] < order[n]));
    }
    sort_unique(expected);
    const Membership dense(expected);
    const std::uint64_t saved = tuning().dense_cell_limit;
    tuning().dense_cell_limit = 0;
    const Membership hashed(expected);
    tuning().dense_cell_limit = saved;
    CHECK(dense.dense() && !hashed.dense());
    for (int n = 0; n < 100000; ++n) {
        const Key probe = pack(lane(rng), lane(rng) / 7, lane(rng) / 50);
        const bool present = std::binary_search(expected.begin(), expected.end(), probe);
        CHECK(dense.contains(probe) == present && hashed.contains(probe) == present);
    }
}

TEST_MAIN
