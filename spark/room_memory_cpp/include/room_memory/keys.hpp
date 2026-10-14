// Packed voxel keys and sorted key-set algebra.
//
// A key packs a voxel index triple, each lane in -LIMIT .. LIMIT-1, as
// ((i + LIMIT) << 42) | ((j + LIMIT) << 21) | (k + LIMIT). Keys are non-negative and their
// integer order is the lexicographic (i, j, k) order, so a sorted unique key vector is the
// canonical set type everywhere: no hash order ever reaches an output.
#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

namespace room_memory {

using Key = std::int64_t;
using Keys = std::vector<Key>;

inline constexpr std::int64_t kLimit = std::int64_t{1} << 20;
inline constexpr std::int64_t kLane = (std::int64_t{1} << 21) - 1;

struct Index3 {
    std::int64_t i, j, k;
    friend bool operator==(const Index3&, const Index3&) = default;
};

constexpr Key pack(std::int64_t i, std::int64_t j, std::int64_t k) {
    return ((i + kLimit) << 42) | ((j + kLimit) << 21) | (k + kLimit);
}

constexpr Index3 unpack(Key key) {
    return {(key >> 42) - kLimit, ((key >> 21) & kLane) - kLimit, (key & kLane) - kLimit};
}

constexpr bool in_range(std::int64_t index) { return index >= -kLimit && index < kLimit; }

constexpr bool in_range(std::int64_t i, std::int64_t j, std::int64_t k) {
    return in_range(i) && in_range(j) && in_range(k);
}

// SplitMix64 finalizer, mod 2^64 (the ray-stride filter and hash mixing).
constexpr std::uint64_t splitmix64(std::uint64_t z) {
    z += 0x9E3779B97F4A7C15ULL;
    z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ULL;
    z = (z ^ (z >> 27)) * 0x94D049BB133111EBULL;
    return z ^ (z >> 31);
}

// Inclusive index box of a key set.
struct Box {
    std::array<std::int64_t, 3> low{0, 0, 0};
    std::array<std::int64_t, 3> high{-1, -1, -1};

    bool empty() const { return high[0] < low[0]; }
    std::int64_t extent(int axis) const { return high[axis] - low[axis] + 1; }
    // Cell count (at most 2^63: three lanes of at most 2^21 cells).
    std::uint64_t cells() const;
    bool contains(std::int64_t i, std::int64_t j, std::int64_t k) const {
        return i >= low[0] && i <= high[0] && j >= low[1] && j <= high[1] && k >= low[2] && k <= high[2];
    }
    void include(std::int64_t i, std::int64_t j, std::int64_t k);
};

Box box_of(std::span<const Key> keys);

// Set algebra on sorted unique vectors.
Keys set_union(std::span<const Key> a, std::span<const Key> b);
Keys set_difference(std::span<const Key> a, std::span<const Key> b);
Keys set_intersection(std::span<const Key> a, std::span<const Key> b);
bool intersects(std::span<const Key> a, std::span<const Key> b);
// Number of elements of sorted unique `keys` present in sorted unique `set`.
std::size_t count_present(std::span<const Key> set, std::span<const Key> keys);

// Sorts keys ascending (LSD radix on the box-relative code; equal output to std::sort).
void sort_keys(Keys& keys);
// Sorts and removes duplicates.
void sort_unique(Keys& keys);
// Stable permutation that sorts `keys` ascending.
std::vector<std::uint32_t> sort_order(std::span<const Key> keys);

}  // namespace room_memory
