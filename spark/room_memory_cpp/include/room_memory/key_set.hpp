// Membership structures for packed keys: an open-addressing hash and a dense bit grid.
#pragma once

#include <atomic>
#include <cstdint>
#include <span>
#include <vector>

#include "room_memory/keys.hpp"

namespace room_memory {

// Process-wide knobs. Results never depend on them; tests force the sparse paths with them.
struct Tuning {
    // A dense bit grid is used over a box of at most this many cells (32 MiB of bits).
    std::uint64_t dense_cell_limit = std::uint64_t{1} << 28;
    // Point aggregation direct-addresses a partition of at most 2^this voxel codes (256 KiB of
    // per-thread tables at 13) and hashes larger ones; -1 always hashes.
    int direct_partition_bits = 13;
};
Tuning& tuning();

// Open-addressing (linear probing) map from a key to its position in the key span it was
// built from. Capacity is a power of two at least twice the key count; -1 marks a free slot,
// which no packed key can be.
class KeyIndex {
public:
    KeyIndex() = default;
    explicit KeyIndex(std::span<const Key> keys);

    static constexpr std::uint32_t kMissing = 0xFFFFFFFFu;

    // Position of `key` in the original span, or kMissing.
    std::uint32_t find(Key key) const {
        if (mask_ == 0) return kMissing;
        std::uint64_t slot = splitmix64(static_cast<std::uint64_t>(key)) & mask_;
        while (true) {
            const Key stored = slots_[slot];
            if (stored == key) return values_[slot];
            if (stored < 0) return kMissing;
            slot = (slot + 1) & mask_;
        }
    }
    bool contains(Key key) const { return find(key) != kMissing; }

private:
    std::vector<Key> slots_;
    std::vector<std::uint32_t> values_;
    std::uint64_t mask_ = 0;
};

// Bits over an index box, i-major then j then k, so increasing linear index is increasing key.
class BitGrid {
public:
    BitGrid() = default;
    explicit BitGrid(const Box& box);

    const Box& box() const { return box_; }
    std::int64_t stride(int axis) const { return axis == 0 ? extent_j_ * extent_k_ : axis == 1 ? extent_k_ : 1; }
    std::uint64_t linear(std::int64_t i, std::int64_t j, std::int64_t k) const {
        return static_cast<std::uint64_t>(((i - box_.low[0]) * extent_j_ + (j - box_.low[1])) * extent_k_ +
                                          (k - box_.low[2]));
    }
    bool test(std::uint64_t index) const { return (words_[index >> 6] >> (index & 63)) & 1u; }
    void set(std::uint64_t index) { words_[index >> 6] |= std::uint64_t{1} << (index & 63); }
    // Thread-safe set; skips the read-modify-write when the bit is already on, so the cells
    // every ray shares near the sensor stay read-only after their first mark.
    void set_shared(std::uint64_t index) {
        std::uint64_t* word = &words_[index >> 6];
        const std::uint64_t bit = std::uint64_t{1} << (index & 63);
        if ((__atomic_load_n(word, __ATOMIC_RELAXED) & bit) == 0) __atomic_fetch_or(word, bit, __ATOMIC_RELAXED);
    }
    // Every set bit as a key, ascending.
    Keys keys() const;
    // ORs every grid of `others` (same box) into this one, in parallel over words.
    void absorb(std::span<const BitGrid> others);
    std::size_t bytes() const { return words_.size() * sizeof(std::uint64_t); }

private:
    Box box_;
    std::int64_t extent_j_ = 0;
    std::int64_t extent_k_ = 0;
    std::vector<std::uint64_t> words_;
};

// Exact membership in a sorted key set: a BitGrid when its box is within the dense limit,
// otherwise a KeyIndex.
class Membership {
public:
    explicit Membership(std::span<const Key> keys);

    bool dense() const { return dense_; }
    const Box& box() const { return box_; }
    bool contains(std::int64_t i, std::int64_t j, std::int64_t k) const {
        if (!box_.contains(i, j, k)) return false;
        if (dense_) return grid_.test(grid_.linear(i, j, k));
        return index_.contains(pack(i, j, k));
    }
    bool contains(Key key) const {
        const Index3 v = unpack(key);
        return contains(v.i, v.j, v.k);
    }
    const BitGrid& grid() const { return grid_; }
    const KeyIndex& index() const { return index_; }

private:
    Box box_;
    bool dense_ = false;
    BitGrid grid_;
    KeyIndex index_;
};

}  // namespace room_memory
