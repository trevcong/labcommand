#include "room_memory/keys.hpp"

#include "parallel.hpp"

#include <bit>
#include <utility>

namespace room_memory {

std::uint64_t Box::cells() const {
    if (empty()) return 0;
    // Each extent is at most 2^21, so the product is at most 2^63 and fits.
    return static_cast<std::uint64_t>(extent(0)) * static_cast<std::uint64_t>(extent(1)) *
           static_cast<std::uint64_t>(extent(2));
}

void Box::include(std::int64_t i, std::int64_t j, std::int64_t k) {
    if (empty()) {
        low = {i, j, k};
        high = {i, j, k};
        return;
    }
    low = {std::min(low[0], i), std::min(low[1], j), std::min(low[2], k)};
    high = {std::max(high[0], i), std::max(high[1], j), std::max(high[2], k)};
}

Box box_of(std::span<const Key> keys) {
    Box box;
    if (keys.empty()) return box;
    std::int64_t lo[3] = {kLimit, kLimit, kLimit};
    std::int64_t hi[3] = {-kLimit - 1, -kLimit - 1, -kLimit - 1};
    for (const Key key : keys) {
        const Index3 v = unpack(key);
        lo[0] = std::min(lo[0], v.i);
        hi[0] = std::max(hi[0], v.i);
        lo[1] = std::min(lo[1], v.j);
        hi[1] = std::max(hi[1], v.j);
        lo[2] = std::min(lo[2], v.k);
        hi[2] = std::max(hi[2], v.k);
    }
    box.low = {lo[0], lo[1], lo[2]};
    box.high = {hi[0], hi[1], hi[2]};
    return box;
}

namespace {

// Runs a sorted-set merge `op(a_part, b_part, out)` on value ranges cut at elements of `a`, in
// parallel for large inputs, and concatenates the parts in order. Every b element lands in
// the range of the a elements around it (the first and last ranges are open-ended), which is
// all a union, difference or intersection of that range needs.
template <typename Op>
Keys merge_parts(std::span<const Key> a, std::span<const Key> b, std::size_t reserve, Op&& op) {
    const std::size_t parts = a.size() + b.size() < 4 * detail::kParallelGrain || a.size() < b.size() / 64
                                  ? 1
                                  : static_cast<std::size_t>(detail::max_threads());
    if (parts == 1) {
        Keys out;
        out.reserve(reserve);
        op(a, b, out);
        return out;
    }
    std::vector<Keys> found(parts);
#pragma omp parallel for schedule(static)
    for (std::size_t p = 0; p < parts; ++p) {
        const std::size_t a_first = detail::chunk_begin(a.size(), parts, p);
        const std::size_t a_last = detail::chunk_begin(a.size(), parts, p + 1);
        const auto cut = [&](std::size_t at) {
            return at == 0 ? b.begin() : at == a.size() ? b.end() : std::lower_bound(b.begin(), b.end(), a[at]);
        };
        const auto b_first = cut(a_first);
        const auto b_last = cut(a_last);
        op(a.subspan(a_first, a_last - a_first), std::span<const Key>(b_first, b_last), found[p]);
    }
    std::vector<std::size_t> start(parts + 1, 0);
    for (std::size_t p = 0; p < parts; ++p) start[p + 1] = start[p] + found[p].size();
    Keys out(start[parts]);
#pragma omp parallel for schedule(static)
    for (std::size_t p = 0; p < parts; ++p) std::copy(found[p].begin(), found[p].end(), out.begin() + static_cast<std::ptrdiff_t>(start[p]));
    return out;
}

}  // namespace

Keys set_union(std::span<const Key> a, std::span<const Key> b) {
    return merge_parts(a, b, a.size() + b.size(), [](std::span<const Key> x, std::span<const Key> y, Keys& out) {
        std::set_union(x.begin(), x.end(), y.begin(), y.end(), std::back_inserter(out));
    });
}

Keys set_difference(std::span<const Key> a, std::span<const Key> b) {
    return merge_parts(a, b, a.size(), [](std::span<const Key> x, std::span<const Key> y, Keys& out) {
        std::set_difference(x.begin(), x.end(), y.begin(), y.end(), std::back_inserter(out));
    });
}

Keys set_intersection(std::span<const Key> a, std::span<const Key> b) {
    return merge_parts(a, b, std::min(a.size(), b.size()), [](std::span<const Key> x, std::span<const Key> y, Keys& out) {
        std::set_intersection(x.begin(), x.end(), y.begin(), y.end(), std::back_inserter(out));
    });
}

bool intersects(std::span<const Key> a, std::span<const Key> b) {
    auto x = a.begin();
    auto y = b.begin();
    while (x != a.end() && y != b.end()) {
        if (*x < *y) {
            ++x;
        } else if (*y < *x) {
            ++y;
        } else {
            return true;
        }
    }
    return false;
}

std::size_t count_present(std::span<const Key> set, std::span<const Key> keys) {
    std::size_t count = 0;
    auto x = set.begin();
    for (const Key key : keys) {
        x = std::lower_bound(x, set.end(), key);
        if (x == set.end()) break;
        if (*x == key) ++count;
    }
    return count;
}

namespace {

constexpr std::size_t kSmallSort = 256;
constexpr int kDigitBits = 11;

// One LSD pass per digit of each lane's box-relative value, k lane first. Lanes are
// independent fields of the key, so sorting by (i, j, k) relative values is key order.
template <bool WithPayload>
void radix_sort(Key* keys, std::uint32_t* payload, std::size_t count) {
    const Box box = box_of(std::span<const Key>(keys, count));
    Keys key_scratch(count);
    std::vector<std::uint32_t> payload_scratch(WithPayload ? count : 0);
    Key* src = keys;
    Key* dst = key_scratch.data();
    std::uint32_t* psrc = payload;
    std::uint32_t* pdst = payload_scratch.data();
    std::vector<std::size_t> histogram(std::size_t{1} << kDigitBits);
    for (int lane = 2; lane >= 0; --lane) {
        const int shift = lane == 0 ? 42 : lane == 1 ? 21 : 0;
        const std::uint64_t base = static_cast<std::uint64_t>(box.low[lane] + kLimit);
        const int width = std::bit_width(static_cast<std::uint64_t>(box.high[lane] - box.low[lane]));
        for (int digit = 0; digit < width; digit += kDigitBits) {
            const int bits = std::min(kDigitBits, width - digit);
            const std::uint64_t mask = (std::uint64_t{1} << bits) - 1;
            const auto digit_of = [&](Key key) {
                return static_cast<std::size_t>(
                    (((static_cast<std::uint64_t>(key) >> shift) & static_cast<std::uint64_t>(kLane)) - base) >>
                        digit & mask);
            };
            std::fill(histogram.begin(), histogram.begin() + (std::ptrdiff_t{1} << bits), 0);
            for (std::size_t n = 0; n < count; ++n) ++histogram[digit_of(src[n])];
            std::size_t total = 0;
            for (std::size_t d = 0; d <= mask; ++d) total += std::exchange(histogram[d], total);
            for (std::size_t n = 0; n < count; ++n) {
                const std::size_t to = histogram[digit_of(src[n])]++;
                dst[to] = src[n];
                if constexpr (WithPayload) pdst[to] = psrc[n];
            }
            std::swap(src, dst);
            if constexpr (WithPayload) std::swap(psrc, pdst);
        }
    }
    if (src != keys) {
        std::copy(src, src + count, keys);
        if constexpr (WithPayload) std::copy(psrc, psrc + count, payload);
    }
}

}  // namespace

void sort_keys(Keys& keys) {
    if (keys.size() <= kSmallSort) {
        std::sort(keys.begin(), keys.end());
        return;
    }
    radix_sort<false>(keys.data(), nullptr, keys.size());
}

void sort_unique(Keys& keys) {
    sort_keys(keys);
    keys.erase(std::unique(keys.begin(), keys.end()), keys.end());
}

std::vector<std::uint32_t> sort_order(std::span<const Key> keys) {
    std::vector<std::uint32_t> order(keys.size());
    for (std::size_t n = 0; n < order.size(); ++n) order[n] = static_cast<std::uint32_t>(n);
    if (keys.size() <= kSmallSort) {
        std::stable_sort(order.begin(), order.end(), [&](std::uint32_t a, std::uint32_t b) { return keys[a] < keys[b]; });
        return order;
    }
    Keys copy(keys.begin(), keys.end());
    radix_sort<true>(copy.data(), order.data(), copy.size());
    return order;
}

}  // namespace room_memory
