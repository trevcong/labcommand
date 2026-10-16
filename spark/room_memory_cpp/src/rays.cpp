// Free-space evidence: the spec's 3D DDA from the sensor origin to every kept ray endpoint.
//
// Rays walk independently, several per thread in SIMD lanes (OpenMP over chunks of rays).
// Marks land in bit grids (per-thread grids ORed together, or one grid taking relaxed atomic
// ORs) or per-thread lists, so the result is a set union and does not depend on scheduling or
// thread count. The walk follows the Python lockstep loop exactly: tMax < 0 -> 0 and the NaN
// drop at setup, the at-end test before marking, the per-axis clamp (an axis on its end index
// gets tMax = +inf), the strict-< minimum, and every tied axis still short of its end
// stepping. tMax accumulates by repeated binary32 additions, so no ray can be skipped ahead;
// rays are only rejected whole, where that is exact.
#include <atomic>
#include <cmath>
#include <limits>
#include <memory>

#include "parallel.hpp"
#include "room_memory/change_detection.hpp"
#include "room_memory/key_set.hpp"
#include "room_memory/ray_backend.hpp"

#if ROOM_MEMORY_NEON
#include <arm_neon.h>
#endif

namespace room_memory {

namespace {

constexpr float kInf = std::numeric_limits<float>::infinity();

struct Ray {
    std::int64_t cur[3];
    std::int64_t end[3];
    std::int64_t step[3];
    std::int64_t remaining[3];
    float t_max[3];
    float t_delta[3];
};

// Spec setup of one ray. Returns false for a ray the spec drops before its first step:
// endpoint in the start cell (cap 0) or a NaN tDelta / tMax.
bool setup(const RayBatch& batch, std::size_t r, Ray& ray) {
    const Index3 end = unpack(batch.end_keys[r]);
    const std::int64_t ends[3] = {end.i, end.j, end.k};
    std::int64_t cap = 0;
    for (int a = 0; a < 3; ++a) {
        const std::int64_t begin = batch.start[a];
        const std::int64_t step = (ends[a] > begin) - (ends[a] < begin);
        // dir = p * inverse - q_origin, two binary32 roundings.
        const float scaled = batch.endpoints[3 * r + a] * batch.inverse;
        const float d = scaled - batch.q_origin[a];
        const float plane = (step > 0 ? static_cast<float>(begin + 1) : static_cast<float>(begin)) - batch.q_origin[a];
        float t_max = step == 0 ? kInf : plane / d;
        if (t_max < 0) t_max = 0;
        const float t_delta = step == 0 ? kInf : 1.0f / std::fabs(d);
        if (std::isnan(t_max) || std::isnan(t_delta)) return false;
        ray.cur[a] = begin;
        ray.end[a] = ends[a];
        ray.step[a] = step;
        ray.remaining[a] = step > 0 ? ends[a] - begin : begin - ends[a];
        ray.t_max[a] = t_max;
        ray.t_delta[a] = t_delta;
        cap += ray.remaining[a];
    }
    return cap > 0;
}

// Every cell of the box spanned by the start and end cells, i.e. every cell the walk can
// visit, misses `box`. Needs only integers, so it runs before the float setup.
bool misses(const std::array<std::int64_t, 3>& start, Key end_key, const Box& box) {
    const Index3 end = unpack(end_key);
    const std::int64_t ends[3] = {end.i, end.j, end.k};
    for (int a = 0; a < 3; ++a) {
        const std::int64_t lo = std::min(start[a], ends[a]);
        const std::int64_t hi = std::max(start[a], ends[a]);
        if (hi < box.low[a] || lo > box.high[a]) return true;
    }
    return false;
}

constexpr int kLanes = 4;
constexpr std::size_t kPrivateGridBytes = std::size_t{64} << 20;
constexpr std::size_t kChunk = 64;

// Up to kLanes rays in flight per thread, one array per field. Each lane performs the scalar
// spec operations in the scalar order, so the walk is exact; on aarch64 the lanes are one
// NEON register per field. Indices fit int32: lanes lie in -2^20 .. 2^20 - 1.
//
// With a keep box, `entered` counts per axis the cells travelled past the box's entry face in
// the step direction (cur - low when stepping up or not at all, high - cur when stepping
// down): the cell is inside when every count is in 0..width, and the ray has left for good
// once a count exceeds width. That is the Python exit rule (left the box on an axis in its
// step direction; sitting outside on an axis that does not step is dropped up front by
// misses()). Every axis moves monotonically toward the end, so a ray that has left can mark
// no keep key again.
struct Pack {
    alignas(16) float t_max[3][kLanes];
    alignas(16) float t_delta[3][kLanes];
    alignas(16) std::int32_t cur[3][kLanes];
    alignas(16) std::int32_t step[3][kLanes];
    alignas(16) std::int32_t remaining[3][kLanes];
    alignas(16) std::int32_t entered[3][kLanes];
    alignas(16) std::int32_t live[kLanes];  // -1 live, 0 retired
    std::int32_t width[3];

    void put(int lane, const Ray& ray, const Box* keep) {
        for (int a = 0; a < 3; ++a) {
            t_max[a][lane] = ray.t_max[a];
            t_delta[a][lane] = ray.t_delta[a];
            cur[a][lane] = static_cast<std::int32_t>(ray.cur[a]);
            step[a][lane] = static_cast<std::int32_t>(ray.step[a]);
            remaining[a][lane] = static_cast<std::int32_t>(ray.remaining[a]);
            entered[a][lane] = keep ? static_cast<std::int32_t>(ray.step[a] < 0 ? keep->high[a] - ray.cur[a]
                                                                                 : ray.cur[a] - keep->low[a])
                                    : 0;
        }
        live[lane] = -1;
    }

    void retire(int lane) {
        for (int a = 0; a < 3; ++a) {
            remaining[a][lane] = 0;
            t_max[a][lane] = kInf;
        }
        live[lane] = 0;
    }
};

// Steps every live lane, marking cells, until some live lane is done: at its end (the spec's
// at_end: all remaining counts 0) or past the keep box. Sets done[l] to nonzero for those.
//
// One iteration is one spec DDA step out of the current cell: mark the cell (with a keep box
// only inside it), take the minimum tMax, step every axis that attains it and is short of its
// end, accumulate tMax += tDelta there, and clamp an axis that reached its end to +inf. Every
// iteration steps at least one axis of a live lane (an axis still short has a finite or +inf
// tMax, never NaN, so it attains the minimum when nothing smaller does), which is why the
// Manhattan cap |di| + |dj| + |dk| can never bind first. The minimum may differ from the
// strict-< chain only in the sign of a zero, which compares equal, so the same axes step.
// Retired lanes have no remaining steps and stay put.
template <typename Visit>
void run_until_done(Pack& pack, bool keep, int thread, Visit& visit, std::int32_t (&done)[kLanes]) {
#if ROOM_MEMORY_NEON
    float32x4_t t[3], d[3];
    int32x4_t cur[3], step[3], rem[3], ent[3], width[3];
    for (int a = 0; a < 3; ++a) {
        t[a] = vld1q_f32(pack.t_max[a]);
        d[a] = vld1q_f32(pack.t_delta[a]);
        cur[a] = vld1q_s32(pack.cur[a]);
        step[a] = vld1q_s32(pack.step[a]);
        rem[a] = vld1q_s32(pack.remaining[a]);
        ent[a] = vld1q_s32(pack.entered[a]);
        width[a] = vdupq_n_s32(pack.width[a]);
    }
    const uint32x4_t alive = vreinterpretq_u32_s32(vld1q_s32(pack.live));
    const float32x4_t infinity = vdupq_n_f32(kInf);
    uint32x4_t finished;
    while (true) {
        uint32x4_t gone = vceqzq_s32(vorrq_s32(vorrq_s32(rem[0], rem[1]), rem[2]));
        if (keep) {
            gone = vorrq_u32(gone, vorrq_u32(vorrq_u32(vcgtq_s32(ent[0], width[0]), vcgtq_s32(ent[1], width[1])),
                                             vcgtq_s32(ent[2], width[2])));
        }
        finished = vandq_u32(gone, alive);
        if (vmaxvq_u32(finished)) break;
        uint32x4_t marks = alive;
        if (keep) marks = vandq_u32(marks, vcgezq_s32(vorrq_s32(vorrq_s32(ent[0], ent[1]), ent[2])));
        alignas(16) std::int32_t ci[kLanes], cj[kLanes], ck[kLanes];
        alignas(16) std::uint32_t marked[kLanes];
        vst1q_s32(ci, cur[0]);
        vst1q_s32(cj, cur[1]);
        vst1q_s32(ck, cur[2]);
        vst1q_u32(marked, marks);
        for (int l = 0; l < kLanes; ++l) {
            if (marked[l]) visit(ci[l], cj[l], ck[l], thread);
        }
        const float32x4_t next = vminq_f32(vminq_f32(t[0], t[1]), t[2]);
        for (int a = 0; a < 3; ++a) {
            const uint32x4_t moves = vandq_u32(vceqq_f32(t[a], next), vcgtzq_s32(rem[a]));
            const int32x4_t minus = vreinterpretq_s32_u32(moves);  // -1 where the axis steps
            cur[a] = vaddq_s32(cur[a], vandq_s32(step[a], minus));
            rem[a] = vaddq_s32(rem[a], minus);
            ent[a] = vsubq_s32(ent[a], minus);
            t[a] = vbslq_f32(moves, vaddq_f32(t[a], d[a]), t[a]);
            t[a] = vbslq_f32(vceqzq_s32(rem[a]), infinity, t[a]);
        }
    }
    for (int a = 0; a < 3; ++a) {
        vst1q_f32(pack.t_max[a], t[a]);
        vst1q_s32(pack.cur[a], cur[a]);
        vst1q_s32(pack.remaining[a], rem[a]);
        vst1q_s32(pack.entered[a], ent[a]);
    }
    vst1q_s32(done, vreinterpretq_s32_u32(finished));
#else
    while (true) {
        bool any = false;
        for (int l = 0; l < kLanes; ++l) {
            const bool at_end = (pack.remaining[0][l] | pack.remaining[1][l] | pack.remaining[2][l]) == 0;
            const bool left = keep && (pack.entered[0][l] > pack.width[0] || pack.entered[1][l] > pack.width[1] ||
                                       pack.entered[2][l] > pack.width[2]);
            done[l] = (at_end || left) && pack.live[l];
            any = any || done[l];
        }
        if (any) return;
        for (int l = 0; l < kLanes; ++l) {
            const bool inside = (pack.entered[0][l] | pack.entered[1][l] | pack.entered[2][l]) >= 0;
            if (pack.live[l] && (!keep || inside)) visit(pack.cur[0][l], pack.cur[1][l], pack.cur[2][l], thread);
        }
        for (int l = 0; l < kLanes; ++l) {
            float next = pack.t_max[0][l];
            if (pack.t_max[1][l] < next) next = pack.t_max[1][l];
            if (pack.t_max[2][l] < next) next = pack.t_max[2][l];
            for (int a = 0; a < 3; ++a) {
                const bool moves = pack.t_max[a][l] == next && pack.remaining[a][l] > 0;
                pack.cur[a][l] += moves ? pack.step[a][l] : 0;
                pack.remaining[a][l] -= moves;
                pack.entered[a][l] += moves;
                const float advanced = pack.t_max[a][l] + pack.t_delta[a][l];
                pack.t_max[a][l] = pack.remaining[a][l] == 0 ? kInf : (moves ? advanced : pack.t_max[a][l]);
            }
        }
    }
#endif
}

// Walks every ray of the batch with kLanes rays in flight per thread; rays come in chunks from
// a shared counter. With a keep box, rays whose start-end box misses it are dropped before
// their walk. `visit(i, j, k, thread)` gets every marked cell: each cell before the step out
// of it, except the end cell, and with a keep box only cells inside it. Marks are set unions,
// so neither the lane interleaving nor the scheduling can change the result.
template <typename Visit>
void walk_all(const RayBatch& batch, const Box* keep, Visit&& visit) {
    const std::size_t count = batch.end_keys.size();
    std::atomic<std::size_t> next_chunk{0};
#pragma omp parallel if (count * 16 >= detail::kParallelGrain)
    {
        const int thread = detail::thread_index();
        std::size_t position = 0, end = 0;
        // Value-initialized: a lane that retires before its first put() (fewer usable rays
        // than lanes) still holds defined cur / step / entered / t_delta for the lane loop.
        Pack pack{};
        for (int a = 0; a < 3; ++a) pack.width[a] = keep ? static_cast<std::int32_t>(keep->high[a] - keep->low[a]) : 0;
        const auto load = [&](int lane) {
            Ray ray;
            while (true) {
                if (position == end) {
                    position = next_chunk.fetch_add(kChunk, std::memory_order_relaxed);
                    if (position >= count) {
                        position = end;
                        pack.retire(lane);
                        return false;
                    }
                    end = std::min(position + kChunk, count);
                }
                const std::size_t r = position++;
                if ((!keep || !misses(batch.start, batch.end_keys[r], *keep)) && setup(batch, r, ray)) {
                    pack.put(lane, ray, keep);
                    return true;
                }
            }
        };
        int live = 0;
        for (int l = 0; l < kLanes; ++l) live += load(l);
        alignas(16) std::int32_t done[kLanes];
        // A fresh ray is never done: setup leaves it short of its end, and a kept ray's box
        // meets the keep box, so it starts before the box's exit faces.
        while (live > 0) {
            run_until_done(pack, keep != nullptr, thread, visit, done);
            for (int l = 0; l < kLanes; ++l) {
                if (done[l]) live -= !load(l);
            }
        }
    }
}

std::int64_t linear(const Box& box, const BitGrid& grid, std::int64_t i, std::int64_t j, std::int64_t k) {
    return (i - box.low[0]) * grid.stride(0) + (j - box.low[1]) * grid.stride(1) + (k - box.low[2]);
}

class CpuRayBackend final : public RayBackend {
public:
    Keys marks(const RayBatch& batch, const Keys* keep) const override {
        return keep ? keep_marks(batch, *keep) : all_marks(batch);
    }

private:
    static Keys keep_marks(const RayBatch& batch, const Keys& keep) {
        if (keep.empty() || batch.end_keys.empty()) return {};
        const Membership member(keep);
        const Box& box = member.box();
        if (member.dense()) {
            BitGrid hit(box);
            const BitGrid& grid = member.grid();
            walk_all(batch, &box, [&](std::int64_t i, std::int64_t j, std::int64_t k, int) {
                const auto bit = static_cast<std::uint64_t>(linear(box, grid, i, j, k));
                if (grid.test(bit)) hit.set_shared(bit);
            });
            Keys out;
            for (const Key key : keep) {
                const Index3 v = unpack(key);
                if (hit.test(hit.linear(v.i, v.j, v.k))) out.push_back(key);
            }
            return out;
        }
        const KeyIndex& index = member.index();
        std::unique_ptr<std::atomic<std::uint8_t>[]> hit(new std::atomic<std::uint8_t>[keep.size()]);
        for (std::size_t n = 0; n < keep.size(); ++n) hit[n].store(0, std::memory_order_relaxed);
        walk_all(batch, &box, [&](std::int64_t i, std::int64_t j, std::int64_t k, int) {
            const std::uint32_t slot = index.find(pack(i, j, k));
            if (slot != KeyIndex::kMissing) hit[slot].store(1, std::memory_order_relaxed);
        });
        Keys out;
        for (std::size_t n = 0; n < keep.size(); ++n) {
            if (hit[n].load(std::memory_order_relaxed)) out.push_back(keep[n]);
        }
        return out;
    }

    static Keys all_marks(const RayBatch& batch) {
        if (batch.end_keys.empty()) return {};
        // Every visited cell lies in the box of the start cell and all endpoints.
        Box box;
        box.include(batch.start[0], batch.start[1], batch.start[2]);
        for (const Key key : batch.end_keys) {
            const Index3 v = unpack(key);
            box.include(v.i, v.j, v.k);
        }
        if (box.cells() <= tuning().dense_cell_limit) {
            BitGrid grid(box);
            // Every ray marks the cells near the sensor, so one shared grid makes the cores
            // fight over those cache lines. Each thread gets a private grid (plain stores) when
            // they fit the budget, ORed together at the end; otherwise the shared grid takes
            // relaxed atomic ORs.
            const auto threads = static_cast<std::size_t>(detail::max_threads());
            if (grid.bytes() * threads <= kPrivateGridBytes) {
                std::vector<BitGrid> grids(threads - 1);
#pragma omp parallel for schedule(static)
                for (std::size_t t = 0; t < grids.size(); ++t) grids[t] = BitGrid(box);
                walk_all(batch, nullptr, [&](std::int64_t i, std::int64_t j, std::int64_t k, int thread) {
                    BitGrid& mine = thread == 0 ? grid : grids[static_cast<std::size_t>(thread) - 1];
                    mine.set(static_cast<std::uint64_t>(linear(box, mine, i, j, k)));
                });
                grid.absorb(grids);
                return grid.keys();
            }
            walk_all(batch, nullptr, [&](std::int64_t i, std::int64_t j, std::int64_t k, int) {
                grid.set_shared(static_cast<std::uint64_t>(linear(box, grid, i, j, k)));
            });
            return grid.keys();
        }
        std::vector<Keys> found(static_cast<std::size_t>(detail::max_threads()));
        walk_all(batch, nullptr, [&](std::int64_t i, std::int64_t j, std::int64_t k, int thread) {
            found[static_cast<std::size_t>(thread)].push_back(pack(i, j, k));
        });
        Keys all;
        for (Keys& part : found) {
            sort_unique(part);
            all = set_union(all, part);
        }
        return all;
    }
};

}  // namespace

const RayBackend& cpu_ray_backend() {
    static const CpuRayBackend backend;
    return backend;
}

namespace {

void require_endpoints(const OccupancyMap& after) {
    if (!after.endpoints || after.endpoints->size() != after.occupied.size() * 3) {
        throw InvalidConfig("evidence requires the after map to be built with evidenceOfAbsence enabled");
    }
}

}  // namespace

RayEndpoints ray_endpoints(const OccupancyMap& after, const Config& config) {
    require_endpoints(after);
    RayEndpoints out;
    const std::vector<float>& points = *after.endpoints;
    const std::uint64_t mask = config.ray_stride > 1 ? static_cast<std::uint64_t>(config.ray_stride - 1) : 0;
    for (std::size_t n = 0; n < after.occupied.size(); ++n) {
        if ((splitmix64(static_cast<std::uint64_t>(after.occupied[n])) & mask) != 0) continue;
        out.keys.push_back(after.occupied[n]);
        out.points.insert(out.points.end(), points.begin() + 3 * n, points.begin() + 3 * n + 3);
    }
    return out;
}

Keys ray_marks(const OccupancyMap& after, const Config& config, const Keys* keep, const RayBackend* backend) {
    if (keep && keep->empty()) return {};
    require_endpoints(after);
    // Stride 1 keeps every ray: the batch reads the map in place.
    RayEndpoints strided;
    RayBatch batch;
    if (config.ray_stride > 1) {
        strided = ray_endpoints(after, config);
        batch.end_keys = strided.keys;
        batch.endpoints = strided.points;
    } else {
        batch.end_keys = after.occupied;
        batch.endpoints = *after.endpoints;
    }
    batch.inverse = config.inverse();
    for (int a = 0; a < 3; ++a) {
        batch.q_origin[a] = config.origin[a] * batch.inverse;
        batch.start[a] = static_cast<std::int64_t>(std::floor(batch.q_origin[a]));
    }
    return (backend ? *backend : cpu_ray_backend()).marks(batch, keep);
}

Keys cast_rays(const OccupancyMap& after, const Config& config, const Keys* keep, const RayBackend* backend) {
    if (keep && keep->empty()) return {};
    return set_difference(ray_marks(after, config, keep, backend), after.occupied);
}

}  // namespace room_memory
