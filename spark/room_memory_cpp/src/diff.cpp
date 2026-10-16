// Diff, tolerance dilation, 26-connected clustering, move pairing and the v2 order.
#include <algorithm>
#include <charconv>
#include <cmath>
#include <string>

#include "parallel.hpp"
#include "room_memory/change_detection.hpp"
#include "room_memory/key_set.hpp"

namespace room_memory {

namespace {

std::string short_float(float value) {
    char buffer[32];
    const auto result = std::to_chars(buffer, buffer + sizeof buffer, value);
    return std::string(buffer, result.ptr);
}

std::vector<Index3> offsets(std::int64_t radius) {
    std::vector<Index3> out;
    for (std::int64_t di = -radius; di <= radius; ++di) {
        for (std::int64_t dj = -radius; dj <= radius; ++dj) {
            for (std::int64_t dk = -radius; dk <= radius; ++dk) {
                if (di != 0 || dj != 0 || dk != 0) out.push_back({di, dj, dk});
            }
        }
    }
    return out;
}

// Candidates with no voxel of `other` within Chebyshev `tolerance`, in input order.
Keys surviving(const Keys& candidates, const Keys& other, std::int64_t tolerance) {
    if (tolerance == 0 || candidates.empty()) return candidates;
    const Membership member(other);
    const std::vector<Index3> steps = offsets(tolerance);
    std::vector<std::uint8_t> kept(candidates.size());
#pragma omp parallel for schedule(static) if (candidates.size() >= detail::kParallelGrain / 4)
    for (std::size_t n = 0; n < candidates.size(); ++n) {
        const Index3 v = unpack(candidates[n]);
        bool suppressed = false;
        for (const Index3& d : steps) {
            if (member.contains(v.i + d.i, v.j + d.j, v.k + d.k)) {
                suppressed = true;
                break;
            }
        }
        kept[n] = !suppressed;
    }
    Keys out;
    for (std::size_t n = 0; n < candidates.size(); ++n) {
        if (kept[n]) out.push_back(candidates[n]);
    }
    return out;
}

using Kind = Region::Kind;

Region region_of(const Cluster& cluster, Kind kind) {
    Region region;
    region.kind = kind;
    region.voxel_count = cluster.voxel_count;
    region.centroid = cluster.centroid;
    region.bounds_min = cluster.bounds_min;
    region.bounds_max = cluster.bounds_max;
    region.volume = cluster.volume;
    region.min_key = cluster.min_key;
    region.keys = cluster.keys;
    return region;
}

Region moved_of(const Cluster& appeared, const Cluster& disappeared) {
    Region region = region_of(appeared, Kind::moved);
    region.from_voxel_count = disappeared.voxel_count;
    region.from_min_key = disappeared.min_key;
    region.from_centroid = disappeared.centroid;
    region.from_bounds_min = disappeared.bounds_min;
    region.from_bounds_max = disappeared.bounds_max;
    region.from_keys = disappeared.keys;
    return region;
}

float distance(const Vec3& a, const Vec3& b) {
    const float d0 = a[0] - b[0];
    const float d1 = a[1] - b[1];
    const float d2 = a[2] - b[2];
    return std::sqrt((d0 * d0 + d1 * d1) + d2 * d2);
}

struct Candidate {
    std::size_t index;
    float distance;
};

// Distance when the pair passes the move gates, else NaN.
float gated(const Cluster& appeared, const Cluster& disappeared, const Config& config) {
    const float d = distance(appeared.centroid, disappeared.centroid);
    if (!(d <= config.max_move)) return std::nanf("");
    const float hi = static_cast<float>(std::max(appeared.voxel_count, disappeared.voxel_count));
    const float lo = static_cast<float>(std::min(appeared.voxel_count, disappeared.voxel_count));
    if (!(hi / lo <= config.move_count_ratio)) return std::nanf("");
    return d;
}

// The strictly nearest candidate; none when the minimum distance is tied.
const Candidate* unique_nearest(const std::vector<Candidate>& candidates) {
    if (candidates.empty()) return nullptr;
    const Candidate* best = &candidates[0];
    bool unique = true;
    for (std::size_t n = 1; n < candidates.size(); ++n) {
        if (candidates[n].distance < best->distance) {
            best = &candidates[n];
            unique = true;
        } else if (candidates[n].distance == best->distance) {
            unique = false;
        }
    }
    return unique ? best : nullptr;
}

bool ambiguous(const std::vector<Candidate>& candidates, std::size_t partner, float d) {
    const float limit = d * 1.5f;
    for (const Candidate& c : candidates) {
        if (c.index != partner && c.distance < limit) return true;
    }
    return false;
}

struct Pair {
    std::size_t appeared;
    std::size_t disappeared;
    float distance;
};

std::vector<Pair> mutual_pairs(const std::vector<Cluster>& appeared, const std::vector<Cluster>& disappeared,
                               const Config& config) {
    std::vector<std::vector<Candidate>> by_appeared(appeared.size());
    std::vector<std::vector<Candidate>> by_disappeared(disappeared.size());
    for (std::size_t a = 0; a < appeared.size(); ++a) {
        for (std::size_t d = 0; d < disappeared.size(); ++d) {
            const float dist = gated(appeared[a], disappeared[d], config);
            if (std::isnan(dist)) continue;
            by_appeared[a].push_back({d, dist});
            by_disappeared[d].push_back({a, dist});
        }
    }
    std::vector<Pair> pairs;
    for (std::size_t a = 0; a < appeared.size(); ++a) {
        const Candidate* nearest = unique_nearest(by_appeared[a]);
        if (!nearest) continue;
        const Candidate* back = unique_nearest(by_disappeared[nearest->index]);
        if (!back || back->index != a) continue;
        if (ambiguous(by_appeared[a], nearest->index, nearest->distance)) continue;
        if (ambiguous(by_disappeared[nearest->index], a, nearest->distance)) continue;
        pairs.push_back({a, nearest->index, nearest->distance});
    }
    std::stable_sort(pairs.begin(), pairs.end(), [&](const Pair& x, const Pair& y) {
        if (x.distance != y.distance) return x.distance < y.distance;
        if (appeared[x.appeared].min_key != appeared[y.appeared].min_key) {
            return appeared[x.appeared].min_key < appeared[y.appeared].min_key;
        }
        return disappeared[x.disappeared].min_key < disappeared[y.disappeared].min_key;
    });
    return pairs;
}

std::vector<Region> assemble(const std::vector<Cluster>& appeared, const std::vector<Cluster>& disappeared,
                             const std::vector<Cluster>& unobserved, const Config& config) {
    std::vector<Region> regions;
    std::vector<bool> used_appeared(appeared.size(), false);
    std::vector<bool> used_disappeared(disappeared.size(), false);
    if (config.pair_moves && !appeared.empty() && !disappeared.empty()) {
        for (const Pair& pair : mutual_pairs(appeared, disappeared, config)) {
            if (used_appeared[pair.appeared] || used_disappeared[pair.disappeared]) continue;
            used_appeared[pair.appeared] = true;
            used_disappeared[pair.disappeared] = true;
            regions.push_back(moved_of(appeared[pair.appeared], disappeared[pair.disappeared]));
        }
    }
    for (std::size_t n = 0; n < appeared.size(); ++n) {
        if (!used_appeared[n]) regions.push_back(region_of(appeared[n], Kind::appeared));
    }
    for (std::size_t n = 0; n < disappeared.size(); ++n) {
        if (!used_disappeared[n]) regions.push_back(region_of(disappeared[n], Kind::disappeared));
    }
    for (const Cluster& c : unobserved) regions.push_back(region_of(c, Kind::unobserved));
    std::stable_sort(regions.begin(), regions.end(), [](const Region& x, const Region& y) {
        if (x.kind != y.kind) return static_cast<int>(x.kind) < static_cast<int>(y.kind);
        if (x.voxel_count != y.voxel_count) return x.voxel_count > y.voxel_count;
        if (x.min_key != y.min_key) return x.min_key < y.min_key;
        const Key fx = x.kind == Kind::moved ? x.from_min_key : -1;
        const Key fy = y.kind == Kind::moved ? y.from_min_key : -1;
        return fx < fy;
    });
    return regions;
}

}  // namespace

const char* kind_name(Region::Kind kind) {
    switch (kind) {
        case Kind::moved: return "moved";
        case Kind::appeared: return "appeared";
        case Kind::disappeared: return "disappeared";
        case Kind::unobserved: return "unobserved";
    }
    return "";
}

std::vector<Cluster> clusters(std::span<const Key> keys, const Config& config) {
    std::vector<Cluster> found;
    if (keys.empty()) return found;
    const KeyIndex index(keys);
    const std::vector<Index3> steps = offsets(1);
    std::vector<std::uint8_t> visited(keys.size(), 0);
    std::vector<std::uint32_t> queue;
    const float voxel = config.voxel_size;
    const double size = static_cast<double>(voxel);
    for (std::size_t seed = 0; seed < keys.size(); ++seed) {
        if (visited[seed]) continue;
        // Seeds go in ascending key order, so a component's first member is its minimum key.
        visited[seed] = 1;
        queue.assign(1, static_cast<std::uint32_t>(seed));
        for (std::size_t head = 0; head < queue.size(); ++head) {
            const Index3 v = unpack(keys[queue[head]]);
            for (const Index3& d : steps) {
                const std::int64_t i = v.i + d.i, j = v.j + d.j, k = v.k + d.k;
                if (!in_range(i, j, k)) continue;
                const std::uint32_t at = index.find(pack(i, j, k));
                if (at == KeyIndex::kMissing || visited[at]) continue;
                visited[at] = 1;
                queue.push_back(at);
            }
        }
        const auto members = static_cast<std::int64_t>(queue.size());
        if (members < config.min_voxels) continue;
        std::int64_t sum[3] = {0, 0, 0};
        std::int64_t low[3] = {kLimit, kLimit, kLimit};
        std::int64_t high[3] = {-kLimit - 1, -kLimit - 1, -kLimit - 1};
        Keys member_keys;
        member_keys.reserve(queue.size());
        for (const std::uint32_t at : queue) {
            const Index3 v = unpack(keys[at]);
            const std::int64_t lanes[3] = {v.i, v.j, v.k};
            for (int a = 0; a < 3; ++a) {
                sum[a] += lanes[a];
                low[a] = std::min(low[a], lanes[a]);
                high[a] = std::max(high[a], lanes[a]);
            }
            member_keys.push_back(keys[at]);
        }
        bool thin = false;
        for (int a = 0; a < 3; ++a) thin = thin || (high[a] + 1 - low[a] < config.min_extent);
        if (thin) continue;
        std::sort(member_keys.begin(), member_keys.end());
        Cluster cluster;
        cluster.voxel_count = members;
        const double n = static_cast<double>(members);
        for (int a = 0; a < 3; ++a) {
            // Exact index sums in binary64, rounded once to binary32.
            cluster.centroid[a] = static_cast<float>(((static_cast<double>(sum[a]) / n) + 0.5) * size);
            cluster.bounds_min[a] = static_cast<float>(low[a]) * voxel;
            cluster.bounds_max[a] = static_cast<float>(high[a] + 1) * voxel;
        }
        cluster.volume = ((static_cast<float>(members) * voxel) * voxel) * voxel;
        cluster.min_key = member_keys.front();
        cluster.keys = std::move(member_keys);
        found.push_back(std::move(cluster));
    }
    return found;
}

Report diff(const OccupancyMap& before, const OccupancyMap& after, const Config& config, const DiffOptions& options) {
    validate(config);
    if (before.voxel_size != after.voxel_size || before.voxel_size != config.voxel_size) {
        const float other = before.voxel_size == after.voxel_size ? config.voxel_size : after.voxel_size;
        throw VoxelSizeMismatch("voxel size mismatch " + short_float(before.voxel_size) + " m vs " + short_float(other) +
                                " m");
    }
    if (config.evidence_of_absence && (!after.endpoints || after.endpoints->size() != after.occupied.size() * 3)) {
        throw InvalidConfig("evidence requires the after map to be built with evidenceOfAbsence enabled");
    }
    Keys only_after = set_difference(after.occupied, before.occupied);
    if (options.before_known) only_after = set_intersection(only_after, *options.before_known);
    const Keys only_before = set_difference(before.occupied, after.occupied);
    const Keys appeared = surviving(only_after, before.occupied, config.tolerance);

    const bool regions_only = options.scope == DiffScope::regions && !config.report_unobserved;
    Report report;
    Keys disappeared;
    Keys unknown;
    if (!config.evidence_of_absence) {
        disappeared = surviving(only_before, after.occupied, config.tolerance);
    } else if (!only_before.empty()) {
        if (regions_only) {
            const Keys candidates = surviving(only_before, after.occupied, config.tolerance);
            disappeared = cast_rays(after, config, &candidates, options.backend);
        } else {
            const Keys free = cast_rays(after, config, &only_before, options.backend);
            unknown = set_difference(only_before, free);
            disappeared = surviving(free, after.occupied, config.tolerance);
        }
    }
    const std::vector<Cluster> appeared_clusters = clusters(appeared, config);
    const std::vector<Cluster> disappeared_clusters = clusters(disappeared, config);
    const std::vector<Cluster> unobserved = config.evidence_of_absence && config.report_unobserved
                                                ? clusters(unknown, config)
                                                : std::vector<Cluster>{};
    report.regions = assemble(appeared_clusters, disappeared_clusters, unobserved, config);
    report.appeared_voxels = static_cast<std::int64_t>(appeared.size());
    report.disappeared_voxels = static_cast<std::int64_t>(disappeared.size());
    if (!regions_only) report.unknown_voxels = static_cast<std::int64_t>(unknown.size());
    report.before_voxels = static_cast<std::int64_t>(before.occupied.size());
    report.after_voxels = static_cast<std::int64_t>(after.occupied.size());
    return report;
}

Comparison compare(std::span<const float> before, std::span<const float> after, const Config& config) {
    validate(config);
    Comparison out;
    out.before = occupancy_map(before, config);
    out.after = occupancy_map(after, config);
    out.report = diff(out.before, out.after, config);
    return out;
}

}  // namespace room_memory
