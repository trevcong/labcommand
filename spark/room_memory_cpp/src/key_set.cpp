#include "room_memory/key_set.hpp"

#include <bit>

#include "parallel.hpp"

namespace room_memory {

Tuning& tuning() {
    static Tuning value;
    return value;
}

KeyIndex::KeyIndex(std::span<const Key> keys) {
    if (keys.empty()) return;
    const std::uint64_t capacity = std::bit_ceil(static_cast<std::uint64_t>(keys.size()) * 2);
    mask_ = capacity - 1;
    slots_.assign(capacity, Key{-1});
    values_.resize(capacity);
    for (std::size_t n = 0; n < keys.size(); ++n) {
        std::uint64_t slot = splitmix64(static_cast<std::uint64_t>(keys[n])) & mask_;
        while (slots_[slot] >= 0 && slots_[slot] != keys[n]) slot = (slot + 1) & mask_;
        if (slots_[slot] < 0) {
            slots_[slot] = keys[n];
            values_[slot] = static_cast<std::uint32_t>(n);
        }
    }
}

BitGrid::BitGrid(const Box& box) : box_(box) {
    if (box.empty()) return;
    extent_j_ = box.extent(1);
    extent_k_ = box.extent(2);
    words_.assign((box.cells() + 63) / 64, 0);
}

Keys BitGrid::keys() const {
    if (box_.empty()) return {};
    const std::int64_t rows = box_.extent(0) * extent_j_;
    const std::int64_t row_cells = extent_k_;
    // Ranges of rows (fixed i, j) are counted, then each fills its own slice of the output,
    // in row order, so the keys come out ascending.
    const std::int64_t parts =
        words_.size() < detail::kParallelGrain ? 1 : std::min<std::int64_t>(rows, 8 * detail::max_threads());
    const auto span_of = [&](std::int64_t part) {
        return std::pair<std::uint64_t, std::uint64_t>{
            static_cast<std::uint64_t>(static_cast<std::int64_t>(detail::chunk_begin(rows, parts, part)) * row_cells),
            static_cast<std::uint64_t>(static_cast<std::int64_t>(detail::chunk_begin(rows, parts, part + 1)) * row_cells)};
    };
    // Calls emit(cell) for every set cell in [begin, end), ascending.
    const auto scan = [&](std::uint64_t begin, std::uint64_t end, auto&& emit) {
        std::uint64_t index = begin;
        while (index < end) {
            const std::uint64_t word_index = index >> 6;
            std::uint64_t bits = words_[word_index] >> (index & 63);
            const std::uint64_t word_end = std::min(end, (word_index + 1) << 6);
            const std::uint64_t span = word_end - index;
            if (span < 64) bits &= (std::uint64_t{1} << span) - 1;
            while (bits) {
                emit(index + static_cast<std::uint64_t>(std::countr_zero(bits)));
                bits &= bits - 1;
            }
            index = word_end;
        }
    };
    std::vector<std::size_t> start(static_cast<std::size_t>(parts) + 1, 0);
#pragma omp parallel for schedule(static) if (parts > 1)
    for (std::int64_t part = 0; part < parts; ++part) {
        const auto [begin, end] = span_of(part);
        std::size_t count = 0;
        scan(begin, end, [&](std::uint64_t) { ++count; });
        start[static_cast<std::size_t>(part) + 1] = count;
    }
    for (std::size_t part = 0; part < static_cast<std::size_t>(parts); ++part) start[part + 1] += start[part];
    Keys out(start.back());
#pragma omp parallel for schedule(static) if (parts > 1)
    for (std::int64_t part = 0; part < parts; ++part) {
        const auto [begin, end] = span_of(part);
        std::size_t at = start[static_cast<std::size_t>(part)];
        scan(begin, end, [&](std::uint64_t cell) {
            const std::int64_t row = static_cast<std::int64_t>(cell) / row_cells;
            const std::int64_t k = static_cast<std::int64_t>(cell) - row * row_cells + box_.low[2];
            out[at++] = pack(row / extent_j_ + box_.low[0], row % extent_j_ + box_.low[1], k);
        });
    }
    return out;
}

void BitGrid::absorb(std::span<const BitGrid> others) {
    const auto count = static_cast<std::int64_t>(words_.size());
#pragma omp parallel for schedule(static) if (words_.size() >= detail::kParallelGrain)
    for (std::int64_t w = 0; w < count; ++w) {
        std::uint64_t word = words_[static_cast<std::size_t>(w)];
        for (const BitGrid& other : others) word |= other.words_[static_cast<std::size_t>(w)];
        words_[static_cast<std::size_t>(w)] = word;
    }
}

Membership::Membership(std::span<const Key> keys) : box_(box_of(keys)) {
    if (keys.empty()) return;
    dense_ = box_.cells() <= tuning().dense_cell_limit;
    if (dense_) {
        grid_ = BitGrid(box_);
        for (const Key key : keys) {
            const Index3 v = unpack(key);
            grid_.set(grid_.linear(v.i, v.j, v.k));
        }
    } else {
        index_ = KeyIndex(keys);
    }
}

}  // namespace room_memory
