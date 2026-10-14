// Deterministic test scenes, bit for bit the binary32 builders of spark/room_memory/tests/
// scenes.py (and so of ChangeDetectionTests.swift), plus the golden cases of goldens.py.
#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace scenes {

using Cloud = std::vector<float>;  // 3 floats per point

// The test runner's 64-bit LCG.
struct LCG {
    std::uint64_t state;
    std::uint64_t next() { return state = state * 6364136223846793005ULL + 1442695040888963407ULL; }
    float unit() { return static_cast<float>(next() >> 40) / static_cast<float>(1 << 24); }
};

Cloud concat(std::initializer_list<const Cloud*> parts);
Cloud shuffled(const Cloud& points, std::uint64_t seed);
Cloud room_shell();
Cloud box_shell(std::array<double, 3> center, double side = 0.40);
Cloud shifted(const Cloud& points, double dx);
Cloud jittered(const Cloud& points, std::uint64_t seed);
std::array<float, 3> cell_point(std::int64_t i, std::int64_t j, std::int64_t k);
Cloud cells(std::initializer_list<std::array<std::int64_t, 3>> list);
Cloud column(const std::vector<std::int64_t>& indices);
Cloud indoor(bool move_box_a);
struct Pair {
    Cloud before;
    Cloud after;
};
Pair long_ray();
struct Board {
    Cloud before, after, front, back;
};
Board board_scene();
Pair boundary_pair(bool far);

// One goldens.SCENES entry: the expected bytes live in fixtures/swift/<name>.json.
struct GoldenCase {
    std::string name;
    Pair (*build)();
    std::vector<std::string> flags;
};
const std::vector<GoldenCase>& golden_cases();

// Phase 0 pairs: (golden name, before file, after file) in the fixture directory.
struct Phase0Case {
    std::string name;
    std::string before;
    std::string after;
};
const std::vector<Phase0Case>& phase0_cases();

// The individual pairs some tests reuse.
Pair speck_pair();
Pair sheet_pair();
Pair small_box_pair(bool before_box, bool after_box);
Pair small_moved_pair();

}  // namespace scenes
