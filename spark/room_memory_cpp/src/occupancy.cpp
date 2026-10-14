// Config validation, the spec quantizer, and per-voxel aggregation of point sets.
//
// Aggregation is exact and order-free: a voxel's count is a sum and its representative is
// the lexicographic minimum of the per-axis binary32 order keys, so the result does not
// depend on point order, partitioning, or thread count. Points are quantized (NEON on aarch64),
// then range-partitioned on the box-relative (i, j, k) code, which keeps partitions in key
// order. Each partition's points are scattered contiguously and aggregated in a per-thread
// table that yields its voxels in code order, so the concatenation is the sorted unique key
// set without a global sort.
#include <algorithm>
#include <bit>
#include <cmath>
#include <limits>
#include <memory>

#include "parallel.hpp"
#include "room_memory/change_detection.hpp"
#include "room_memory/key_set.hpp"

#if ROOM_MEMORY_NEON
#include <arm_neon.h>
#endif

namespace room_memory {

namespace {

constexpr float kLimitF32 = static_cast<float>(kLimit);
constexpr std::uint32_t kSign = 0x80000000u;

std::uint32_t order_key(float value) {
    const std::uint32_t bits = std::bit_cast<std::uint32_t>(value);
    return (bits & kSign) ? ~bits : (bits | kSign);
}

float from_order_key(std::uint32_t key) { return std::bit_cast<float>((key & kSign) ? (key & ~kSign) : ~key); }

using OrderKey = std::array<std::uint32_t, 3>;

bool order_less(const OrderKey& a, const OrderKey& b) {
    if (a[0] != b[0]) return a[0] < b[0];
    if (a[1] != b[1]) return a[1] < b[1];
    return a[2] < b[2];
}

constexpr Key kInvalid = -1;

// Spec quantizer on binary32 values: drops non-finite points, points beyond max_range
// ((x*x + y*y) + z*z > max_range^2) and points whose |q| reaches 2^20; floor of q otherwise.
struct Quantizer {
    float inverse;
    bool clip;
    float max_range_sq;

    Key scalar(float x, float y, float z) const {
        if (!(std::isfinite(x) && std::isfinite(y) && std::isfinite(z))) return kInvalid;
        if (clip && ((x * x + y * y) + z * z) > max_range_sq) return kInvalid;
        const float qx = x * inverse;
        const float qy = y * inverse;
        const float qz = z * inverse;
        if (!(std::fabs(qx) < kLimitF32 && std::fabs(qy) < kLimitF32 && std::fabs(qz) < kLimitF32)) return kInvalid;
        return pack(static_cast<std::int64_t>(std::floor(qx)), static_cast<std::int64_t>(std::floor(qy)),
                    static_cast<std::int64_t>(std::floor(qz)));
    }

    // Keys of points [begin, end); returns the number accepted and grows `box`.
    std::int64_t run(const float* xyz, std::size_t begin, std::size_t end, Key* keys, Box& box) const {
        std::int64_t accepted = 0;
        std::int64_t lo[3] = {kLimit, kLimit, kLimit};
        std::int64_t hi[3] = {-kLimit - 1, -kLimit - 1, -kLimit - 1};
        std::size_t n = begin;
#if ROOM_MEMORY_NEON
        const float32x4_t inv = vdupq_n_f32(inverse);
        const float32x4_t limit = vdupq_n_f32(kLimitF32);
        const float32x4_t infinity = vdupq_n_f32(std::numeric_limits<float>::infinity());
        const float32x4_t range_sq = vdupq_n_f32(max_range_sq);
        const int64x2_t bias = vdupq_n_s64(kLimit);
        const int32x4_t largest = vdupq_n_s32(std::numeric_limits<std::int32_t>::max());
        const int32x4_t smallest = vdupq_n_s32(std::numeric_limits<std::int32_t>::min());
        int32x4_t low[3] = {largest, largest, largest};
        int32x4_t high[3] = {smallest, smallest, smallest};
        uint32x4_t taken = vdupq_n_u32(0);
        for (; n + 4 <= end; n += 4) {
            const float32x4x3_t p = vld3q_f32(xyz + 3 * n);
            uint32x4_t ok = vandq_u32(vandq_u32(vcltq_f32(vabsq_f32(p.val[0]), infinity),
                                                vcltq_f32(vabsq_f32(p.val[1]), infinity)),
                                      vcltq_f32(vabsq_f32(p.val[2]), infinity));
            if (clip) {
                // Separate multiplies and adds: the spec's (x*x + y*y) + z*z, never fused.
                const float32x4_t xx = vmulq_f32(p.val[0], p.val[0]);
                const float32x4_t yy = vmulq_f32(p.val[1], p.val[1]);
                const float32x4_t zz = vmulq_f32(p.val[2], p.val[2]);
                const float32x4_t r2 = vaddq_f32(vaddq_f32(xx, yy), zz);
                ok = vandq_u32(ok, vmvnq_u32(vcgtq_f32(r2, range_sq)));
            }
            int32x4_t index[3];
            for (int a = 0; a < 3; ++a) {
                const float32x4_t q = vmulq_f32(p.val[a], inv);
                ok = vandq_u32(ok, vcltq_f32(vabsq_f32(q), limit));
                // Round toward minus infinity: the spec floor, exact inside |q| < 2^20.
                index[a] = vcvtmq_s32_f32(q);
            }
            taken = vsubq_u32(taken, ok);
            // key = ((i + L) << 42) | ((j + L) << 21) | (k + L), two lanes per 64-bit vector;
            // rejected lanes get -1.
            const int32x4_t valid = vreinterpretq_s32_u32(ok);
            for (int half = 0; half < 2; ++half) {
                const auto widen = [&](int32x4_t v) {
                    return vaddq_s64(half ? vmovl_high_s32(v) : vmovl_s32(vget_low_s32(v)), bias);
                };
                const int64x2_t key = vorrq_s64(vorrq_s64(vshlq_n_s64(widen(index[0]), 42), vshlq_n_s64(widen(index[1]), 21)),
                                                widen(index[2]));
                const int64x2_t mask = half ? vmovl_high_s32(valid) : vmovl_s32(vget_low_s32(valid));
                vst1q_s64(keys + n + 2 * half, vbslq_s64(vreinterpretq_u64_s64(mask), key, vdupq_n_s64(kInvalid)));
            }
            for (int a = 0; a < 3; ++a) {
                low[a] = vminq_s32(low[a], vbslq_s32(ok, index[a], largest));
                high[a] = vmaxq_s32(high[a], vbslq_s32(ok, index[a], smallest));
            }
        }
        accepted += vaddvq_u32(taken);
        for (int a = 0; a < 3; ++a) {
            lo[a] = std::min<std::int64_t>(lo[a], vminvq_s32(low[a]));
            hi[a] = std::max<std::int64_t>(hi[a], vmaxvq_s32(high[a]));
        }
#endif
        for (; n < end; ++n) {
            const Key key = scalar(xyz[3 * n], xyz[3 * n + 1], xyz[3 * n + 2]);
            keys[n] = key;
            if (key == kInvalid) continue;
            ++accepted;
            const Index3 v = unpack(key);
            const std::int64_t lanes[3] = {v.i, v.j, v.k};
            for (int a = 0; a < 3; ++a) {
                lo[a] = std::min(lo[a], lanes[a]);
                hi[a] = std::max(hi[a], lanes[a]);
            }
        }
        if (accepted) {
            box.include(lo[0], lo[1], lo[2]);
            box.include(hi[0], hi[1], hi[2]);
        }
        return accepted;
    }
};

// The box-relative code of a voxel: i, j and k offsets in adjacent bit fields, i on top, so
// code order is key order. Partitions are ranges of `drop` low bits.
struct Partitioning {
    int shift_i = 0, shift_j = 0;  // bit offsets of the i and j lanes in the code
    int drop = 0;                  // low code bits below the partition id
    std::size_t parts = 1;
    Box box;

    std::uint64_t code(Key key) const {
        const Index3 v = unpack(key);
        return (static_cast<std::uint64_t>(v.i - box.low[0]) << shift_i) |
               (static_cast<std::uint64_t>(v.j - box.low[1]) << shift_j) | static_cast<std::uint64_t>(v.k - box.low[2]);
    }

    Key key(std::uint64_t code) const {
        const std::uint64_t k_mask = (std::uint64_t{1} << shift_j) - 1;
        const std::uint64_t j_mask = (std::uint64_t{1} << (shift_i - shift_j)) - 1;
        return pack(static_cast<std::int64_t>(code >> shift_i) + box.low[0],
                    static_cast<std::int64_t>((code >> shift_j) & j_mask) + box.low[1],
                    static_cast<std::int64_t>(code & k_mask) + box.low[2]);
    }
};

Partitioning partitioning(const Box& box, std::size_t accepted) {
    Partitioning p;
    p.box = box;
    const int wk = std::bit_width(static_cast<std::uint64_t>(box.high[2] - box.low[2]));
    const int wj = std::bit_width(static_cast<std::uint64_t>(box.high[1] - box.low[1]));
    const int wi = std::bit_width(static_cast<std::uint64_t>(box.high[0] - box.low[0]));
    p.shift_j = wk;
    p.shift_i = wk + wj;
    const int bits = wi + wj + wk;
    // About 512 points per partition, at most 4096 partitions.
    const int wanted =
        accepted < detail::kParallelGrain ? 0 : std::min(12, static_cast<int>(std::bit_width(accepted / 512)) - 1);
    const int part_bits = std::min(bits, std::max(0, wanted));
    p.drop = bits - part_bits;
    p.parts = std::size_t{1} << part_bits;
    return p;
}

// One accepted point, scattered into its partition: its voxel code and the per-axis order
// keys of its coordinates. After aggregation the same slots hold one voxel each.
struct Entry {
    std::uint64_t code;
    std::uint32_t count;
    OrderKey best;
};

constexpr std::uint64_t kNoCode = ~std::uint64_t{0};  // codes use at most 63 bits

// Per-thread aggregation of one partition's entries into voxels, reused across partitions.
// A partition spans 2^drop codes. When that is at most 2^Tuning::direct_partition_bits the
// voxels are direct-addressed by their low code bits and come out in code order by scanning
// the touched-bit words; otherwise an open-addressing table (Fibonacci hash, linear probing)
// collects them and they are sorted.
struct Aggregator {
    std::vector<std::uint32_t> counts;
    std::vector<OrderKey> best;
    std::vector<std::uint64_t> touched;  // direct mode: one bit per code, all zero between partitions
    std::vector<std::uint64_t> codes;    // hash mode: slot keys, kNoCode when free
    std::vector<std::uint32_t> used;     // hash mode: occupied slots
    std::uint64_t mask = 0;
    int shift = 64;
    bool direct = false;

    void reset(int drop, std::size_t entries) {
        direct = drop <= tuning().direct_partition_bits;
        used.clear();
        if (direct) {
            const std::size_t slots = std::size_t{1} << drop;
            if (counts.size() < slots) {
                counts.resize(slots);
                best.resize(slots);
            }
            if (touched.size() < (slots + 63) / 64) touched.resize((slots + 63) / 64, 0);
            mask = slots - 1;
            return;
        }
        const std::uint64_t capacity = std::bit_ceil(std::max<std::uint64_t>(16, entries * 2));
        if (codes.size() < capacity) codes.resize(capacity, kNoCode);
        if (counts.size() < capacity) {
            counts.resize(capacity);
            best.resize(capacity);
        }
        mask = capacity - 1;
        shift = 64 - std::countr_zero(capacity);
    }

    void add(const Entry& entry) {
        std::uint64_t slot;
        if (direct) {
            slot = entry.code & mask;
            std::uint64_t& word = touched[slot >> 6];
            const std::uint64_t bit = std::uint64_t{1} << (slot & 63);
            if (!(word & bit)) {
                word |= bit;
                counts[slot] = entry.count;
                best[slot] = entry.best;
                return;
            }
        } else {
            slot = (entry.code * 0x9E3779B97F4A7C15ULL) >> shift;
            while (codes[slot] != entry.code) {
                if (codes[slot] == kNoCode) {
                    codes[slot] = entry.code;
                    counts[slot] = entry.count;
                    best[slot] = entry.best;
                    used.push_back(static_cast<std::uint32_t>(slot));
                    return;
                }
                slot = (slot + 1) & mask;
            }
        }
        counts[slot] += entry.count;
        if (order_less(entry.best, best[slot])) best[slot] = entry.best;
    }

    // Writes the voxels in code order to `out` (codes relative to `base`), empties the
    // table, and returns the voxel count.
    std::size_t drain(std::uint64_t base, Entry* out) {
        std::size_t n = 0;
        if (direct) {
            for (std::size_t w = 0; w <= (mask >> 6); ++w) {
                std::uint64_t bits = touched[w];
                touched[w] = 0;
                while (bits) {
                    const std::uint64_t slot = (w << 6) + static_cast<std::uint64_t>(std::countr_zero(bits));
                    bits &= bits - 1;
                    out[n++] = Entry{base | slot, counts[slot], best[slot]};
                }
            }
            return n;
        }
        for (const std::uint32_t slot : used) {
            out[n++] = Entry{codes[slot], counts[slot], best[slot]};
            codes[slot] = kNoCode;
        }
        std::sort(out, out + n, [](const Entry& a, const Entry& b) { return a.code < b.code; });
        return n;
    }
};

// Grow-only buffer without value initialization.
template <typename T>
struct Buffer {
    std::unique_ptr<T[]> data;
    std::size_t capacity = 0;
    T* ensure(std::size_t count) {
        if (count > capacity) {
            data.reset(new T[count]);
            capacity = count;
        }
        return data.get();
    }
};

// Scratch of the calling thread, reused across calls, so a window's tens of megabytes of
// intermediate arrays are not page-faulted in again every time.
struct Scratch {
    Buffer<Key> keys;
    Buffer<Entry> entries;
    std::vector<Aggregator> tables;
};

Scratch& scratch() {
    thread_local Scratch value;
    return value;
}

}  // namespace

void validate(const Config& config) {
    const float voxel = config.voxel_size;
    if (!std::isfinite(voxel) || voxel <= 0) throw InvalidConfig("voxelSize must be finite and > 0 m");
    if (config.max_range && (!std::isfinite(*config.max_range) || *config.max_range <= 0)) {
        throw InvalidConfig("maxRange must be finite and > 0 m");
    }
    if (config.min_points < 1) throw InvalidConfig("minPoints must be >= 1");
    if (config.min_voxels < 1) throw InvalidConfig("minVoxels must be >= 1");
    if (config.min_extent < 1) throw InvalidConfig("minExtent must be >= 1 voxel");
    if (config.tolerance < 0 || config.tolerance > 4) throw InvalidConfig("tolerance must be in 0...4 voxels");
    if (!std::isfinite(config.max_move) || config.max_move < 0) throw InvalidConfig("maxMove must be finite and >= 0 m");
    if (!std::isfinite(config.move_count_ratio) || config.move_count_ratio < 1) {
        throw InvalidConfig("moveCountRatio must be finite and >= 1");
    }
    const std::int64_t stride = config.ray_stride;
    if (stride < 1 || stride > 64 || (stride & (stride - 1)) != 0) {
        throw InvalidConfig("rayStride must be a power of two from 1 through 64");
    }
    for (const float value : config.origin) {
        if (!std::isfinite(value)) throw InvalidConfig("origin must be finite");
    }
    if (config.evidence_of_absence) {
        const float inverse = config.inverse();
        for (const float value : config.origin) {
            const float q = value * inverse;
            if (!std::isfinite(q) || std::fabs(q) >= kLimitF32) throw InvalidConfig("origin is outside the voxel grid");
        }
    }
}

CellSummary summarize(std::span<const float> xyz, const Config& config) {
    validate(config);
    const std::size_t count = xyz.size() / 3;
    const bool reps = config.evidence_of_absence;
    const float range = config.max_range.value_or(0.0f);
    const Quantizer quantizer{config.inverse(), config.max_range.has_value(), range * range};
    Scratch& work = scratch();

    CellSummary summary;
    summary.representatives = reps;
    Key* keys = work.keys.ensure(count);
    const std::size_t chunks =
        count < detail::kParallelGrain ? 1 : std::min<std::size_t>(count / 4096, 4 * static_cast<std::size_t>(detail::max_threads()));
    std::vector<std::int64_t> chunk_accepted(chunks, 0);
    std::vector<Box> chunk_box(chunks);
#pragma omp parallel for schedule(static) if (chunks > 1)
    for (std::size_t c = 0; c < chunks; ++c) {
        chunk_accepted[c] = quantizer.run(xyz.data(), detail::chunk_begin(count, chunks, c),
                                          detail::chunk_begin(count, chunks, c + 1), keys, chunk_box[c]);
    }
    Box box;
    std::int64_t accepted = 0;
    for (std::size_t c = 0; c < chunks; ++c) {
        accepted += chunk_accepted[c];
        if (!chunk_box[c].empty()) {
            box.include(chunk_box[c].low[0], chunk_box[c].low[1], chunk_box[c].low[2]);
            box.include(chunk_box[c].high[0], chunk_box[c].high[1], chunk_box[c].high[2]);
        }
    }
    summary.accepted_points = accepted;
    summary.dropped_points = static_cast<std::int64_t>(count) - accepted;
    if (accepted == 0) return summary;

    // Keys become box codes in place; per-chunk partition histograms, then offsets in
    // (partition, chunk) order, so every partition lists its points in input order.
    const Partitioning parts = partitioning(box, static_cast<std::size_t>(accepted));
    std::vector<std::uint32_t> histogram(chunks * parts.parts, 0);
#pragma omp parallel for schedule(static) if (chunks > 1)
    for (std::size_t c = 0; c < chunks; ++c) {
        std::uint32_t* h = &histogram[c * parts.parts];
        for (std::size_t n = detail::chunk_begin(count, chunks, c); n < detail::chunk_begin(count, chunks, c + 1); ++n) {
            if (keys[n] == kInvalid) continue;
            const std::uint64_t code = parts.code(keys[n]);
            keys[n] = static_cast<Key>(code);
            ++h[code >> parts.drop];
        }
    }
    std::vector<std::size_t> part_start(parts.parts + 1, 0);
    std::vector<std::size_t> offset(chunks * parts.parts);
    const bool many = chunks * parts.parts >= detail::kParallelGrain;
#pragma omp parallel for schedule(static) if (many)
    for (std::size_t p = 0; p < parts.parts; ++p) {
        std::size_t sum = 0;
        for (std::size_t c = 0; c < chunks; ++c) sum += histogram[c * parts.parts + p];
        part_start[p + 1] = sum;
    }
    for (std::size_t p = 0; p < parts.parts; ++p) part_start[p + 1] += part_start[p];
    const std::size_t total = part_start[parts.parts];
#pragma omp parallel for schedule(static) if (many)
    for (std::size_t p = 0; p < parts.parts; ++p) {
        std::size_t run = part_start[p];
        for (std::size_t c = 0; c < chunks; ++c) {
            offset[c * parts.parts + p] = run;
            run += histogram[c * parts.parts + p];
        }
    }
    Entry* entries = work.entries.ensure(total);
    const OrderKey none{0xFFFFFFFFu, 0xFFFFFFFFu, 0xFFFFFFFFu};
#pragma omp parallel for schedule(static) if (chunks > 1)
    for (std::size_t c = 0; c < chunks; ++c) {
        std::size_t* o = &offset[c * parts.parts];
        for (std::size_t n = detail::chunk_begin(count, chunks, c); n < detail::chunk_begin(count, chunks, c + 1); ++n) {
            if (keys[n] == kInvalid) continue;
            const auto code = static_cast<std::uint64_t>(keys[n]);
            const OrderKey best = reps ? OrderKey{order_key(xyz[3 * n]), order_key(xyz[3 * n + 1]),
                                                  order_key(xyz[3 * n + 2])}
                                       : none;
            entries[o[code >> parts.drop]++] = Entry{code, 1, best};
        }
    }

    // Each partition aggregates its contiguous entries and writes its voxels, in code order,
    // back over the front of its own range.
    std::vector<std::size_t> found(parts.parts, 0);
    const int threads = parts.parts > 1 ? detail::max_threads() : 1;
    if (work.tables.size() < static_cast<std::size_t>(threads)) work.tables.resize(static_cast<std::size_t>(threads));
#pragma omp parallel for schedule(dynamic, 8) if (parts.parts > 1)
    for (std::size_t p = 0; p < parts.parts; ++p) {
        const std::size_t first = part_start[p];
        const std::size_t last = part_start[p + 1];
        if (first == last) continue;
        Aggregator& table = work.tables[static_cast<std::size_t>(detail::thread_index())];
        table.reset(parts.drop, last - first);
        for (std::size_t e = first; e < last; ++e) table.add(entries[e]);
        found[p] = table.drain(static_cast<std::uint64_t>(p) << parts.drop, entries + first);
    }
    std::vector<std::size_t> out_start(parts.parts + 1, 0);
    for (std::size_t p = 0; p < parts.parts; ++p) out_start[p + 1] = out_start[p] + found[p];
    const std::size_t unique = out_start[parts.parts];
    summary.keys.resize(unique);
    summary.counts.resize(unique);
    if (reps) summary.order_keys.resize(unique);
#pragma omp parallel for schedule(static) if (parts.parts > 1 && unique >= detail::kParallelGrain)
    for (std::size_t p = 0; p < parts.parts; ++p) {
        for (std::size_t n = 0; n < found[p]; ++n) {
            const Entry& entry = entries[part_start[p] + n];
            const std::size_t at = out_start[p] + n;
            summary.keys[at] = parts.key(entry.code);
            summary.counts[at] = entry.count;
            if (reps) summary.order_keys[at] = entry.best;
        }
    }
    return summary;
}

CellSummary merge(std::span<const CellSummary* const> parts) {
    if (parts.size() == 1) return *parts.front();
    // Representatives survive when every part that holds a voxel carries them.
    CellSummary out;
    out.representatives = true;
    Keys keys;
    std::vector<std::uint32_t> counts;
    std::vector<OrderKey> best;
    for (const CellSummary* part : parts) {
        out.accepted_points += part->accepted_points;
        out.dropped_points += part->dropped_points;
        if (!part->keys.empty()) out.representatives = out.representatives && part->representatives;
        keys.insert(keys.end(), part->keys.begin(), part->keys.end());
        counts.insert(counts.end(), part->counts.begin(), part->counts.end());
        if (part->representatives) best.insert(best.end(), part->order_keys.begin(), part->order_keys.end());
    }
    const std::vector<std::uint32_t> order = sort_order(keys);
    for (std::size_t e = 0; e < order.size();) {
        const Key key = keys[order[e]];
        std::uint64_t count = 0;
        OrderKey minimum{0xFFFFFFFFu, 0xFFFFFFFFu, 0xFFFFFFFFu};
        for (; e < order.size() && keys[order[e]] == key; ++e) {
            count += counts[order[e]];
            if (out.representatives && order_less(best[order[e]], minimum)) minimum = best[order[e]];
        }
        out.keys.push_back(key);
        out.counts.push_back(static_cast<std::uint32_t>(std::min<std::uint64_t>(count, 0xFFFFFFFFu)));
        if (out.representatives) out.order_keys.push_back(minimum);
    }
    return out;
}

OccupancyMap occupancy_from(const CellSummary& summary, const Config& config) {
    OccupancyMap map;
    map.voxel_size = config.voxel_size;
    map.accepted_points = summary.accepted_points;
    map.dropped_points = summary.dropped_points;
    const bool reps = config.evidence_of_absence;
    if (reps && !summary.representatives) {
        throw InvalidConfig("evidence requires a summary built with evidenceOfAbsence enabled");
    }
    const std::uint64_t threshold = static_cast<std::uint64_t>(config.min_points);
    const std::size_t count = summary.keys.size();
    // Dense voxels per chunk, then each chunk writes its own output range.
    const std::size_t chunks = count < 4 * detail::kParallelGrain ? 1 : static_cast<std::size_t>(detail::max_threads());
    std::vector<std::size_t> start(chunks + 1, 0);
#pragma omp parallel for schedule(static) if (chunks > 1)
    for (std::size_t c = 0; c < chunks; ++c) {
        std::size_t dense = 0;
        for (std::size_t n = detail::chunk_begin(count, chunks, c); n < detail::chunk_begin(count, chunks, c + 1); ++n) {
            dense += summary.counts[n] >= threshold;
        }
        start[c + 1] = dense;
    }
    for (std::size_t c = 0; c < chunks; ++c) start[c + 1] += start[c];
    map.occupied.resize(start[chunks]);
    std::vector<float> endpoints(reps ? 3 * start[chunks] : 0);
#pragma omp parallel for schedule(static) if (chunks > 1)
    for (std::size_t c = 0; c < chunks; ++c) {
        std::size_t at = start[c];
        for (std::size_t n = detail::chunk_begin(count, chunks, c); n < detail::chunk_begin(count, chunks, c + 1); ++n) {
            if (summary.counts[n] < threshold) continue;
            map.occupied[at] = summary.keys[n];
            if (reps) {
                for (int a = 0; a < 3; ++a) endpoints[3 * at + static_cast<std::size_t>(a)] = from_order_key(summary.order_keys[n][a]);
            }
            ++at;
        }
    }
    if (reps) map.endpoints = std::move(endpoints);
    return map;
}

OccupancyMap occupancy_map(std::span<const float> xyz, const Config& config) {
    return occupancy_from(summarize(xyz, config), config);
}

}  // namespace room_memory
