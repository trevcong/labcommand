#include "room_memory/memory.hpp"

#include <pwd.h>
#include <unistd.h>

#include <zlib.h>

#include <array>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iterator>
#include <new>
#include <numbers>
#include <sstream>
#include <stdexcept>

#include "parallel.hpp"
#include "room_memory/pointcloud2.hpp"

namespace room_memory {

namespace {

constexpr std::int64_t kInt16Low = -(1 << 15);
constexpr std::int64_t kInt16High = (1 << 15) - 1;

Key brick_key(const Index3& v) { return pack(v.i >> 3, v.j >> 3, v.k >> 3); }

bool plain_name(const std::string& text) {
    const auto ok = [](char c, bool first) {
        return (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') ||
               (!first && (c == '.' || c == '_' || c == '-'));
    };
    if (text.empty() || !ok(text[0], true)) return false;
    for (std::size_t n = 1; n < text.size(); ++n) {
        if (!ok(text[n], false)) return false;
    }
    return true;
}

}  // namespace

std::filesystem::path expand_user(const std::string& path) {
    if (path.empty() || path[0] != '~') return path;
    const std::size_t slash = path.find('/');
    const std::string user = path.substr(1, slash == std::string::npos ? std::string::npos : slash - 1);
    std::string home;
    if (user.empty()) {
        if (const char* env = std::getenv("HOME")) {
            home = env;
        } else if (const passwd* entry = getpwuid(getuid())) {
            home = entry->pw_dir;
        }
    } else if (const passwd* entry = getpwnam(user.c_str())) {
        home = entry->pw_dir;
    }
    if (home.empty()) return path;
    while (home.size() > 1 && home.back() == '/') home.pop_back();
    return home + (slash == std::string::npos ? std::string() : path.substr(slash));
}

namespace {

std::int64_t nanoseconds(const msg::Time& t) { return t.sec * 1'000'000'000 + t.nanosec; }

std::int64_t period_ns(double seconds) { return static_cast<std::int64_t>(seconds * 1e9); }

bool fraction(double value) { return value >= 0.0 && value <= 1.0; }

// Share of the voxels in the index box of `reference.occupied` that the reference observed;
// empty while it has no occupied voxel.
std::optional<float> coverage_of(const VoxelLayer& reference) {
    const Box box = box_of(reference.occupied);
    if (box.empty()) return std::nullopt;
    std::uint64_t inside = 0;
    for (const Key key : reference.known) {
        const Index3 v = unpack(key);
        inside += box.contains(v.i, v.j, v.k) ? 1 : 0;
    }
    return static_cast<float>(static_cast<double>(inside) / static_cast<double>(box.cells()));
}

std::string sensor_list(const std::vector<std::string>& names) {
    std::string text = "[";
    for (std::size_t n = 0; n < names.size(); ++n) text += (n ? ", '" : "'") + names[n] + "'";
    return text + "]";
}

double radians(double degrees) { return degrees * (std::numbers::pi / 180.0); }

// The rig guard's parameters (only checked with rig_guard on).
void validate_guard(const MemoryConfig& config) {
    const std::pair<const char*, double> positive[] = {
        {"imu_shock_frac", config.imu_shock_frac},     {"imu_gyro_dps", config.imu_gyro_dps},
        {"imu_quiet_s", config.imu_quiet_s},           {"tilt_trigger_deg", config.tilt_trigger_deg},
        {"pose_trigger_m", config.pose_trigger_m},     {"check_max_s", config.check_max_s},
        {"move_min_m", config.move_min_m},             {"move_min_deg", config.move_min_deg},
        {"align_max_m", config.align_max_m},           {"align_max_deg", config.align_max_deg},
        {"align_max_rms_m", config.align_max_rms_m},   {"observe_min_eig", config.observe_min_eig},
        {"relearn_after_s", config.relearn_after_s},   {"reg_voxel_m", config.reg_voxel_m},
        {"src_voxel_m", config.src_voxel_m},           {"icp_max_range_m", config.icp_max_range_m},
    };
    for (const auto& [name, value] : positive) {
        if (!(std::isfinite(value) && value > 0 && value < 1e9)) {
            throw std::invalid_argument(std::string(name) + " must be a finite number > 0");
        }
    }
    if (!(std::isfinite(config.guard_check_period_s) && config.guard_check_period_s >= 0 &&
          config.guard_check_period_s < 1e9)) {
        throw std::invalid_argument("guard_check_period_s must be in [0, 1e9) s (0: no periodic check)");
    }
    if (!fraction(config.align_inlier_ratio)) throw std::invalid_argument("align_inlier_ratio must be in [0, 1]");
    if (!fraction(config.gross_inlier_ratio)) throw std::invalid_argument("gross_inlier_ratio must be in [0, 1]");
    if (config.align_stable_windows < 1 || config.reg_max_points < 1 || config.src_max_points < 1) {
        throw std::invalid_argument("align_stable_windows, reg_max_points and src_max_points must be >= 1");
    }
    if (config.align_max_deg > 90) throw std::invalid_argument("align_max_deg must be at most 90");
    // A yaw only the periodic check sees must be decided before a false region's run reaches the gate.
    if (config.guard_check_period_s > 0 &&
        config.guard_check_period_s + 2 * config.window_s >
            static_cast<double>(config.candidate_windows) * config.window_s - 5) {
        throw std::invalid_argument("guard_check_period_s + 2 window_s must be at most candidate_windows window_s - 5 s");
    }
}

// The nominal Mid-360 mount in the JT128 frame (RigTransforms.livoxToWorld: yaw -86.5 deg, roll pi),
// used only to carry a rotation prior from one sensor's frame to the other's, never as an extrinsic.
const rig::Mat3& mount() {
    static const rig::Mat3 R = rig::Pose{rig::rotation_about({0.0, 0.0, 1.0}, -86.5), {}}
                                   .compose(rig::Pose{rig::rotation_about({1.0, 0.0, 0.0}, 180.0), {}})
                                   .R;
    return R;
}

}  // namespace

const std::vector<Sensor>& known_sensors() {
    static const std::vector<Sensor> sensors = {
        {"mid360", "/livox/lidar", msg::kSensorMid360, 0},
        {"jt128", "/lidar_points", msg::kSensorJT128, 1},
    };
    return sensors;
}

// MARK: Layers

VoxelLayer VoxelLayer::observe(const OccupancyMap& occupancy, const Config& config, const RayBackend* backend) {
    VoxelLayer layer;
    layer.occupied = occupancy.occupied;
    layer.known = set_union(occupancy.occupied, ray_marks(occupancy, config, nullptr, backend));
    return layer;
}

VoxelLayer VoxelLayer::merge(const VoxelLayer& other) const {
    VoxelLayer layer;
    layer.occupied = set_union(occupied, other.occupied);
    layer.known = set_union(known, other.known);
    return layer;
}

OccupancyMap VoxelLayer::as_occupancy(float voxel_size) const {
    OccupancyMap map;
    map.voxel_size = voxel_size;
    map.occupied = occupied;
    return map;
}

std::int64_t VoxelLayer::brick_count() const {
    if (!brick_count_) {
        Keys bricks;
        bricks.reserve(known.size());
        for (const Key key : known) bricks.push_back(brick_key(unpack(key)));
        sort_unique(bricks);
        brick_count_ = static_cast<std::int64_t>(bricks.size());
    }
    return *brick_count_;
}

std::vector<msg::Brick> VoxelLayer::to_bricks() const {
    if (known.empty()) return {};
    Keys ids;
    ids.reserve(known.size());
    for (const Key key : known) ids.push_back(brick_key(unpack(key)));
    Keys unique = ids;
    sort_unique(unique);
    std::vector<msg::Brick> bricks(unique.size());
    for (std::size_t n = 0; n < unique.size(); ++n) {
        const Index3 b = unpack(unique[n]);
        if (b.i < kInt16Low || b.j < kInt16Low || b.k < kInt16Low || b.i > kInt16High || b.j > kInt16High ||
            b.k > kInt16High) {
            throw std::invalid_argument("a voxel lies outside the int16 brick range");
        }
        bricks[n].coordinate = {b.i, b.j, b.k};
    }
    const KeyIndex slot(unique);
    auto occupied_at = occupied.begin();
    for (std::size_t n = 0; n < known.size(); ++n) {
        const Index3 v = unpack(known[n]);
        const auto bit = static_cast<std::size_t>((v.i & 7) + 8 * (v.j & 7) + 64 * (v.k & 7));
        msg::Brick& brick = bricks[slot.find(ids[n])];
        brick.known[bit >> 3] |= static_cast<std::uint8_t>(1u << (bit & 7));
        occupied_at = std::lower_bound(occupied_at, occupied.end(), known[n]);
        if (occupied_at != occupied.end() && *occupied_at == known[n]) {
            brick.occupied[bit >> 3] |= static_cast<std::uint8_t>(1u << (bit & 7));
        }
    }
    return bricks;
}

VoxelLayer VoxelLayer::from_bricks(std::span<const msg::Brick> bricks) {
    VoxelLayer layer;
    const auto keys = [&](bool occupied) {
        Keys out;
        for (const msg::Brick& brick : bricks) {
            const auto& mask = occupied ? brick.occupied : brick.known;
            for (std::size_t n = 0; n < msg::kMaskBytes * 8; ++n) {
                if (!((mask[n >> 3] >> (n & 7)) & 1)) continue;
                const auto x = static_cast<std::int64_t>(n & 7);
                const auto y = static_cast<std::int64_t>((n >> 3) & 7);
                const auto z = static_cast<std::int64_t>(n >> 6);
                out.push_back(pack(brick.coordinate[0] * msg::kBrickDim + x, brick.coordinate[1] * msg::kBrickDim + y,
                                   brick.coordinate[2] * msg::kBrickDim + z));
            }
        }
        sort_unique(out);
        return out;
    };
    layer.occupied = keys(true);
    layer.known = keys(false);
    return layer;
}

// MARK: Registration

std::optional<rig::Vec3d> pose_shift(const Keys& recent, const Membership& reference, double voxel_size) {
    constexpr std::int64_t R = kPoseSearchVoxels;
    constexpr std::size_t kShifts = 2 * R + 1;
    std::array<std::array<std::uint64_t, kShifts>, 3> counts{};
#pragma omp parallel if (recent.size() >= detail::kParallelGrain)
    {
        std::array<std::array<std::uint64_t, kShifts>, 3> local{};
#pragma omp for schedule(static) nowait
        for (std::size_t n = 0; n < recent.size(); ++n) {
            const Index3 v = unpack(recent[n]);
            for (std::int64_t s = -R; s <= R; ++s) {
                const auto at = static_cast<std::size_t>(s + R);
                local[0][at] += reference.contains(v.i + s, v.j, v.k) ? 1 : 0;
                local[1][at] += reference.contains(v.i, v.j + s, v.k) ? 1 : 0;
                local[2][at] += reference.contains(v.i, v.j, v.k + s) ? 1 : 0;
            }
        }
#pragma omp critical
        for (int a = 0; a < 3; ++a) {
            for (std::size_t s = 0; s < kShifts; ++s) counts[a][s] += local[a][s];
        }
    }
    rig::Vec3d shifts{};
    for (int a = 0; a < 3; ++a) {
        const auto& c = counts[a];
        // The peak; on ties the shift nearest zero, the negative one first.
        std::int64_t best = 0;
        for (std::int64_t step = 1; step <= R; ++step) {
            for (const std::int64_t s : {-step, step}) {
                if (c[static_cast<std::size_t>(s + R)] > c[static_cast<std::size_t>(best + R)]) best = s;
            }
        }
        if (c[static_cast<std::size_t>(best + R)] == 0) return std::nullopt;
        double shift = static_cast<double>(best);
        if (best > -R && best < R) {
            const auto low = static_cast<double>(c[static_cast<std::size_t>(best + R - 1)]);
            const auto mid = static_cast<double>(c[static_cast<std::size_t>(best + R)]);
            const auto high = static_cast<double>(c[static_cast<std::size_t>(best + R + 1)]);
            const double curvature = low - 2.0 * mid + high;
            if (curvature < 0.0) shift += (low - high) / (2.0 * curvature);
        }
        shifts[static_cast<std::size_t>(a)] = shift * voxel_size;
    }
    return shifts;
}

double shift_norm(const rig::Vec3d& shift) {
    double squared = 0.0;
    for (const double meters : shift) squared += meters * meters;
    return std::sqrt(squared);
}

std::optional<double> pose_offset(const Keys& recent, const Membership& reference, double voxel_size) {
    const auto shift = pose_shift(recent, reference, voxel_size);
    if (!shift) return std::nullopt;
    return shift_norm(*shift);
}

// MARK: Window counts

void VoxelStats::add(const Keys& occupied, const Keys& known) {
    Keys out_keys;
    std::vector<std::uint32_t> out_hits;
    std::vector<std::uint32_t> out_seen;
    const std::size_t capacity = keys.size() + known.size();
    out_keys.reserve(capacity);
    out_hits.reserve(capacity);
    out_seen.reserve(capacity);
    std::size_t a = 0, b = 0, o = 0;
    while (a < keys.size() || b < known.size()) {
        if (b == known.size() || (a < keys.size() && keys[a] < known[b])) {
            out_keys.push_back(keys[a]);
            out_hits.push_back(hits[a]);
            out_seen.push_back(seen[a]);
            ++a;
            continue;
        }
        const Key key = known[b++];
        while (o < occupied.size() && occupied[o] < key) ++o;
        const std::uint32_t hit = o < occupied.size() && occupied[o] == key ? 1 : 0;
        if (a < keys.size() && keys[a] == key) {
            out_keys.push_back(key);
            out_hits.push_back(hits[a] + hit);
            out_seen.push_back(seen[a] + 1);
            ++a;
        } else {
            out_keys.push_back(key);
            out_hits.push_back(hit);
            out_seen.push_back(1);
        }
    }
    keys = std::move(out_keys);
    hits = std::move(out_hits);
    seen = std::move(out_seen);
}

VoxelLayer VoxelStats::settled(const SettleRule& rule) const {
    VoxelLayer layer;
    for (std::size_t n = 0; n < keys.size(); ++n) {
        const auto h = static_cast<double>(hits[n]);
        const auto s = static_cast<double>(seen[n]);
        if (static_cast<std::int64_t>(hits[n]) >= rule.occupied_min_hits && h >= rule.occupied_min_ratio * s) {
            layer.occupied.push_back(keys[n]);
            layer.known.push_back(keys[n]);
        } else if (static_cast<std::int64_t>(seen[n]) >= rule.free_min_seen && h <= rule.free_max_ratio * s) {
            layer.known.push_back(keys[n]);
        }
    }
    return layer;
}

void VoxelStats::erase(const Keys& drop) {
    std::size_t at = 0, d = 0;
    for (std::size_t n = 0; n < keys.size(); ++n) {
        while (d < drop.size() && drop[d] < keys[n]) ++d;
        if (d < drop.size() && drop[d] == keys[n]) continue;
        keys[at] = keys[n];
        hits[at] = hits[n];
        seen[at] = seen[n];
        ++at;
    }
    keys.resize(at);
    hits.resize(at);
    seen.resize(at);
}

MapDelta map_delta(std::span<const msg::Brick> before, std::span<const msg::Brick> after) {
    MapDelta delta;
    std::size_t a = 0, b = 0;
    while (a < before.size() || b < after.size()) {
        if (b == after.size() || (a < before.size() && before[a].coordinate < after[b].coordinate)) {
            delta.removed.push_back(static_cast<std::int64_t>(a++));
        } else if (a == before.size() || after[b].coordinate < before[a].coordinate) {
            delta.bricks.push_back(after[b++]);
        } else {
            if (!(before[a] == after[b])) delta.bricks.push_back(after[b]);
            ++a;
            ++b;
        }
    }
    return delta;
}

// MARK: Persistence

std::filesystem::path room_dir(const std::string& state_dir, const std::string& room_id) {
    if (!plain_name(room_id)) {
        throw std::invalid_argument("room_id '" + room_id + "' is not a plain file name ([A-Za-z0-9._-], no leading dot)");
    }
    return expand_user(state_dir) / room_id;
}

namespace {

msg::Bytes read_bytes(const std::filesystem::path& path) {
    std::ifstream in(path, std::ios::binary);
    if (!in) throw std::runtime_error("cannot read " + path.string());
    return msg::Bytes((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
}

}  // namespace

std::filesystem::path reference_path(const std::string& state_dir, const std::string& room_id, const std::string& sensor) {
    return room_dir(state_dir, room_id) / ("reference-" + sensor + ".cdr");
}

std::filesystem::path epoch_path(const std::string& state_dir, const std::string& room_id) {
    return room_dir(state_dir, room_id) / "map_epoch";
}

void save_reference(const std::filesystem::path& path, const VoxelLayer& layer, const msg::Header& header,
                    float voxel_size) {
    msg::MapBricks message;
    message.header = header;
    message.compression = msg::Compression::zlib;
    message.kind = msg::MapKind::snapshot;
    message.voxel_size = voxel_size;
    message.bricks = layer.to_bricks();
    write_atomic(path, message.to_cdr());
}

void save_epoch(const std::filesystem::path& path, std::uint64_t epoch) {
    const std::string text = std::to_string(epoch) + "\n";
    write_atomic(path, std::span(reinterpret_cast<const std::uint8_t*>(text.data()), text.size()));
}

std::optional<std::uint64_t> load_epoch(const std::filesystem::path& path) {
    if (!std::filesystem::exists(path)) return std::nullopt;
    std::ifstream in(path, std::ios::binary);
    if (!in) throw std::runtime_error("cannot read " + path.string());
    const std::string text((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    const auto malformed = [&] { return std::invalid_argument(path.string() + " does not hold one map_epoch"); };
    if (text.size() < 2 || text.size() > 21 || text.back() != '\n') throw malformed();
    std::uint64_t epoch = 0;
    for (std::size_t n = 0; n + 1 < text.size(); ++n) {
        const char c = text[n];
        if (c < '0' || c > '9') throw malformed();
        const auto digit = static_cast<std::uint64_t>(c - '0');
        if (epoch > (UINT64_MAX - digit) / 10) throw malformed();
        epoch = epoch * 10 + digit;
    }
    return epoch;
}

// MARK: Seq ranges

std::vector<msg::SeqRange> canonical_ranges(std::vector<msg::SeqRange> ranges) {
    std::sort(ranges.begin(), ranges.end(), [](const msg::SeqRange& a, const msg::SeqRange& b) {
        return a.first != b.first ? a.first < b.first : a.last < b.last;
    });
    std::vector<msg::SeqRange> merged;
    for (const msg::SeqRange& range : ranges) {
        if (!merged.empty() && (merged.back().last == msg::kUint64Max || range.first <= merged.back().last + 1)) {
            merged.back().last = std::max(merged.back().last, range.last);
        } else {
            merged.push_back(range);
        }
    }
    return merged;
}

std::vector<msg::SeqRange> subtract_ranges(const std::vector<msg::SeqRange>& ranges,
                                           const std::vector<msg::SeqRange>& removed) {
    std::vector<msg::SeqRange> out;
    for (msg::SeqRange range : ranges) {
        bool left = true;
        for (const msg::SeqRange& cut : removed) {
            if (cut.last < range.first || cut.first > range.last) continue;
            if (cut.first > range.first) out.push_back({range.first, cut.first - 1});
            if (cut.last >= range.last) {
                left = false;
                break;
            }
            range.first = cut.last + 1;
        }
        if (left) out.push_back(range);
    }
    return out;
}

std::pair<msg::MapBricks, VoxelLayer> load_reference(const std::filesystem::path& path) {
    const msg::Bytes data = read_bytes(path);
    msg::MapBricks message = msg::MapBricks::from_cdr(data);
    VoxelLayer layer = VoxelLayer::from_bricks(message.bricks);
    return {std::move(message), std::move(layer)};
}

// MARK: Candidates

std::size_t count_within(const Keys& keys, const Keys& other, std::int64_t tolerance) {
    if (keys.empty() || other.empty()) return 0;
    std::size_t count = 0;
    for (const Key key : keys) {
        const Index3 v = unpack(key);
        bool found = false;
        for (std::int64_t di = -tolerance; di <= tolerance && !found; ++di) {
            for (std::int64_t dj = -tolerance; dj <= tolerance && !found; ++dj) {
                for (std::int64_t dk = -tolerance; dk <= tolerance && !found; ++dk) {
                    const std::int64_t i = v.i + di, j = v.j + dj, k = v.k + dk;
                    found = in_range(i, j, k) && std::binary_search(other.begin(), other.end(), pack(i, j, k));
                }
            }
        }
        if (found) ++count;
    }
    return count;
}

bool same_shape(const Keys& keys, const Keys& anchor, std::int64_t tolerance, double fraction) {
    return static_cast<double>(count_within(keys, anchor, tolerance)) >= fraction * static_cast<double>(keys.size()) &&
           static_cast<double>(count_within(anchor, keys, tolerance)) >= fraction * static_cast<double>(anchor.size());
}

Region region_from_poses(Region::Kind kind, const Keys& keys, const Keys& from_keys, double voxel_size) {
    const float voxel = static_cast<float>(voxel_size);
    const double size = static_cast<double>(voxel);
    struct Geometry {
        std::int64_t count;
        Vec3 centroid, low, high;
        float volume;
        Key first;
    };
    // The diff's cluster geometry: exact index sums in binary64, rounded once to binary32.
    const auto geometry = [&](const Keys& members) {
        std::int64_t sum[3] = {0, 0, 0};
        std::int64_t low[3] = {kLimit, kLimit, kLimit};
        std::int64_t high[3] = {-kLimit - 1, -kLimit - 1, -kLimit - 1};
        for (const Key key : members) {
            const Index3 v = unpack(key);
            const std::int64_t lanes[3] = {v.i, v.j, v.k};
            for (int a = 0; a < 3; ++a) {
                sum[a] += lanes[a];
                low[a] = std::min(low[a], lanes[a]);
                high[a] = std::max(high[a], lanes[a]);
            }
        }
        Geometry g{static_cast<std::int64_t>(members.size()), {}, {}, {}, 0.0f, members.front()};
        const double n = static_cast<double>(members.size());
        for (int a = 0; a < 3; ++a) {
            g.centroid[a] = static_cast<float>(((static_cast<double>(sum[a]) / n) + 0.5) * size);
            g.low[a] = static_cast<float>(low[a]) * voxel;
            g.high[a] = static_cast<float>(high[a] + 1) * voxel;
        }
        g.volume = ((static_cast<float>(members.size()) * voxel) * voxel) * voxel;
        return g;
    };
    Region region;
    region.kind = kind;
    const Geometry g = geometry(keys);
    region.voxel_count = g.count;
    region.centroid = g.centroid;
    region.bounds_min = g.low;
    region.bounds_max = g.high;
    region.volume = g.volume;
    region.min_key = g.first;
    region.keys = keys;
    if (kind == Region::Kind::moved) {
        const Geometry f = geometry(from_keys);
        region.from_voxel_count = f.count;
        region.from_min_key = f.first;
        region.from_centroid = f.centroid;
        region.from_bounds_min = f.low;
        region.from_bounds_max = f.high;
        region.from_keys = from_keys;
    }
    return region;
}

std::int64_t box_gap(const Box& a, const Box& b) {
    std::int64_t gap = 0;
    for (int n = 0; n < 3; ++n) gap = std::max({gap, b.low[n] - a.high[n], a.low[n] - b.high[n]});
    return gap;
}

Keys region_keys(const Region& region) {
    if (region.kind != Region::Kind::moved) return region.keys;
    return set_union(region.keys, region.from_keys);
}

// MARK: Engine

Config MemoryConfig::change_config() const {
    Config config;
    config.voxel_size = static_cast<float>(voxel_size);
    if (max_range) config.max_range = static_cast<float>(*max_range);
    config.tolerance = tolerance;
    config.min_voxels = min_voxels;
    config.min_extent = min_extent;
    config.evidence_of_absence = true;
    return config;
}

SettleRule MemoryConfig::settle_rule() const {
    return SettleRule{occupied_min_hits, occupied_min_ratio, free_min_seen, free_max_ratio};
}

RoomMemory::RoomMemory(MemoryConfig config, const msg::Time& start, std::uint64_t publisher_generation,
                       const RayBackend* backend, DiffScope scope)
    : config_(std::move(config)), publisher_generation_(publisher_generation), backend_(backend), scope_(scope) {
    if (publisher_generation_ == msg::kGenerationUnknown) {
        throw std::invalid_argument("publisher_generation must be in 1..." + std::to_string(msg::kUint64Max) + ", got 0");
    }
    std::vector<std::string> names;
    bool unknown = config_.sensors.empty();
    for (const std::string& name : config_.sensors) {
        const auto& sensors = known_sensors();
        const auto found = std::find_if(sensors.begin(), sensors.end(), [&](const Sensor& s) { return s.name == name; });
        if (found == sensors.end()) {
            unknown = true;
            continue;
        }
        if (std::find(names.begin(), names.end(), name) != names.end()) continue;
        names.push_back(name);
        Channel channel;
        channel.sensor = *found;
        channels_.push_back(std::move(channel));
    }
    if (unknown) {
        throw std::invalid_argument("sensors must be a non-empty subset of ['jt128', 'mid360'], got " +
                                    sensor_list(config_.sensors));
    }
    if (config_.reference_windows < 1 || config_.recent_windows < 1) {
        throw std::invalid_argument("reference_windows and recent_windows must be >= 1");
    }
    if (!(std::isfinite(config_.window_s) && config_.window_s > 0)) throw std::invalid_argument("window_s must be a finite number > 0 s");
    if (config_.occupied_min_hits < 1 || config_.free_min_seen < 1 || !fraction(config_.occupied_min_ratio) ||
        !fraction(config_.free_max_ratio)) {
        throw std::invalid_argument("occupied_min_hits and free_min_seen must be >= 1, the ratios in [0, 1]");
    }
    if (config_.candidate_windows < 1 || config_.expire_windows < 1) {
        throw std::invalid_argument("candidate_windows and expire_windows must be >= 1");
    }
    if (!(config_.learn_max_offset_m >= 0 && config_.learn_max_offset_m < 1e9)) {
        throw std::invalid_argument("learn_max_offset_m must be in [0, 1e9) m");
    }
    if (config_.learn_windows < 0) throw std::invalid_argument("learn_windows must be >= 0");
    if (config_.fresh_min_voxels < 1 || !fraction(config_.fresh_min_fraction) || !fraction(config_.stable_min_fraction)) {
        throw std::invalid_argument("fresh_min_voxels must be >= 1, fresh_min_fraction and stable_min_fraction in [0, 1]");
    }
    if (!(config_.activity_radius_m >= 0 && config_.activity_radius_m < 1e9)) {
        throw std::invalid_argument("activity_radius_m must be in [0, 1e9) m");
    }
    if (!(config_.delta_period_s > 0 && config_.delta_period_s < 1e9) ||
        !(config_.keyframe_period_s > 0 && config_.keyframe_period_s < 1e9) ||
        !(config_.keyframe_delta_ratio >= 0 && config_.keyframe_delta_ratio < 1e9)) {
        throw std::invalid_argument("delta_period_s and keyframe_period_s must be in (0, 1e9) s, "
                                    "keyframe_delta_ratio in [0, 1e9)");
    }
    const std::string map_sensor = config_.map_sensor.empty() ? "jt128" : config_.map_sensor;
    const auto at = std::find_if(channels_.begin(), channels_.end(), [&](const Channel& c) { return c.sensor.name == map_sensor; });
    if (at != channels_.end()) {
        map_channel_ = static_cast<std::size_t>(at - channels_.begin());
    } else if (!config_.map_sensor.empty()) {
        throw std::invalid_argument("map_sensor '" + config_.map_sensor + "' is not one of the sensors " +
                                    sensor_list(config_.sensors));
    }
    if (config_.rig_guard) validate_guard(config_);
    change_ = config_.change_config();
    validate(change_);
    for (Channel& channel : channels_) {
        channel.config = change_;
        if (config_.rig_guard) channel.builder.emplace(config_.reg_voxel_m);
    }
    if (config_.rig_guard) detector_.emplace(config_.imu_shock_frac, config_.imu_gyro_dps, config_.imu_quiet_s);
    // Activity radius in voxels; no activity check at 0.
    const std::optional<std::int64_t> radius =
        config_.activity_radius_m > 0
            ? std::optional<std::int64_t>(static_cast<std::int64_t>(std::floor(config_.activity_radius_m / config_.voxel_size + 0.5)))
            : std::nullopt;
    for (Channel& channel : channels_) {
        channel.tracker = CandidateTracker(config_.candidate_windows, config_.expire_windows, config_.tolerance, radius,
                                           config_.stable_min_fraction, config_.voxel_size);
    }
    const std::int64_t millis = start.sec * 1000 + start.nanosec / 1'000'000;
    next_event_id_ = static_cast<std::uint64_t>(millis) << 20;
    load(start);
}

RoomMemory::Channel& RoomMemory::find(const std::string& sensor) {
    for (Channel& channel : channels_) {
        if (channel.sensor.name == sensor) return channel;
    }
    throw std::out_of_range("sensor '" + sensor + "' is not configured");
}

const RoomMemory::Channel& RoomMemory::channel(const std::string& sensor) const {
    return const_cast<RoomMemory*>(this)->find(sensor);
}

std::vector<std::filesystem::path> RoomMemory::paths() const {
    std::vector<std::filesystem::path> out;
    for (const Channel& channel : channels_) {
        out.push_back(reference_path(*config_.state_dir, config_.room_id, channel.sensor.name));
    }
    return out;
}

void RoomMemory::load(const msg::Time& start) {
    if (!config_.state_dir) return;
    const std::filesystem::path room = room_dir(*config_.state_dir, config_.room_id);
    const std::filesystem::path rig_state = room / "rig_state";
    if (!config_.rig_guard && std::filesystem::exists(rig_state)) {
        throw std::invalid_argument(rig_state.string() +
                                    ": this state was written with rig_guard; enable it, or move the state aside");
    }
    // The room's last epoch, whichever sensors that run had: a subset run moves it on and saves
    // only its own references, so the others' saved epochs may lag behind it.
    const std::optional<std::uint64_t> room_epoch = load_epoch(epoch_path(*config_.state_dir, config_.room_id));
    const std::vector<std::filesystem::path> all = paths();
    std::vector<std::string> missing;
    std::size_t present = 0;
    for (std::size_t n = 0; n < all.size(); ++n) {
        if (std::filesystem::exists(all[n])) {
            ++present;
        } else {
            missing.push_back(channels_[n].sensor.name);
        }
    }
    if (present == 0) {
        // BUILDING publishes no map or events; TRACKING takes the next epoch.
        map_epoch_ = room_epoch.value_or(0);
        return;
    }
    if (!missing.empty()) {
        std::sort(missing.begin(), missing.end());
        throw std::invalid_argument("saved references are incomplete for room '" + config_.room_id + "': missing " +
                                    sensor_list(missing));
    }
    std::uint64_t epoch = room_epoch.value_or(0);
    std::vector<VoxelLayer> layers;
    for (std::size_t n = 0; n < all.size(); ++n) {
        auto [message, layer] = load_reference(all[n]);
        if (message.header.room_id != config_.room_id) {
            throw std::invalid_argument(all[n].string() + " holds room '" + message.header.room_id + "', not '" +
                                        config_.room_id + "'");
        }
        if (message.voxel_size != static_cast<float>(config_.voxel_size)) {
            throw std::invalid_argument(all[n].string() + " uses a different voxel_size");
        }
        epoch = std::max(epoch, message.header.map_epoch);
        layers.push_back(std::move(layer));
    }
    if (config_.rig_guard && !std::filesystem::exists(rig_state)) {
        // Legacy references: no registration cloud and no pose, so a rig moved while the node was off
        // could not be found (pose_offset is translation-only). Archived and rebuilt: a new frame.
        save_epoch(epoch_path(*config_.state_dir, config_.room_id), epoch);
        std::vector<std::string> names;
        for (const std::filesystem::path& path : all) names.push_back(path.filename().string());
        std::vector<std::string> files{"map_epoch"};
        files.insert(files.end(), names.begin(), names.end());
        archive(room, "legacy-" + std::to_string(epoch), files, names);
        map_epoch_ = epoch;
        outcome_ = msg::RigOutcome::relearning;
        causes_ = msg::kCauseStartup;
        return;
    }
    for (std::size_t n = 0; n < all.size(); ++n) {
        set_reference(channels_[n], std::move(layers[n]));
        channels_[n].reference_windows = config_.reference_windows;
    }
    if (config_.rig_guard) load_rig(room, rig_state);
    // seq restarts at 0, so the restart takes a new epoch: no (map_epoch, seq) sync cursor from
    // before it names a message after it. Saved at once, so the next restart moves on again.
    map_epoch_ = epoch + 1;
    save(start);
    state_ = msg::MemoryState::tracking;
    // The rig may have moved while the node was off.
    if (config_.rig_guard) start_check(nanoseconds(start), msg::kCauseStartup);
}

void RoomMemory::save(const msg::Time& stamp) {
    if (!config_.state_dir) return;
    const msg::Header header{config_.room_id, map_epoch_, 0, stamp};
    // The epoch file first: it never lags behind an epoch in use, even if a reference write fails.
    save_epoch(epoch_path(*config_.state_dir, config_.room_id), map_epoch_);
    if (building()) return;  // a frame commits whole: until every sensor built, the last committed one stays on disk
    const std::vector<std::filesystem::path> all = paths();
    for (std::size_t n = 0; n < all.size(); ++n) {
        save_reference(all[n], channels_[n].reference, header, static_cast<float>(config_.voxel_size));
    }
    if (config_.rig_guard) save_rig();
}

void RoomMemory::add_points(const std::string& sensor, std::span<const float> xyz) {
    Channel& channel = find(sensor);
    if (xyz.size() % 3) throw std::invalid_argument("positions must hold 3 floats per point");
    ++channel.received;
    ++channel.frames;
    channel.points.insert(channel.points.end(), xyz.begin(), xyz.end());
}

bool RoomMemory::add_cdr(const std::string& sensor, std::span<const std::uint8_t> data) {
    Channel& channel = find(sensor);
    PointCloud cloud;
    const auto reject = [&](const char* type_name, const std::exception& error) {
        ++channel.received;
        ++channel.failed;
        channel.last_error = RejectedFrame{type_name, error.what()};
        return false;
    };
    try {
        cloud = decode_pointcloud2(data);
    } catch (const PointCloudError& error) {
        return reject("PointCloudError", error);
    } catch (const std::bad_alloc& error) {
        return reject("bad_alloc", error);
    } catch (const std::exception& error) {  // any other decode failure
        return reject("exception", error);
    }
    add_points(sensor, cloud.positions);
    channel.last_error.reset();
    return true;
}

std::uint64_t RoomMemory::next_id() { return next_event_id_++; }

void RoomMemory::set_reference(Channel& channel, VoxelLayer reference) {
    channel.reference = std::move(reference);
    channel.reference_index.reset();
    if (&channel == &channels_[map_channel_]) coverage_ = coverage_of(channel.reference);
}

bool RoomMemory::reference_ready(const Channel& channel) const {
    return channel.reference_windows >= config_.reference_windows && !channel.reference.occupied.empty();
}

void RoomMemory::build(Channel& channel, std::size_t frames, const std::vector<float>& points) {
    // A sensor's reference freezes once it is ready, and only windows with at least one occupied
    // voxel count toward it.
    if (!frames || reference_ready(channel)) return;
    const OccupancyMap window = occupancy_map(points, change_);
    if (window.occupied.empty()) return;
    const VoxelLayer observed = VoxelLayer::observe(window, change_, backend_);
    channel.stats.add(observed.occupied, observed.known);
    ++channel.reference_windows;
    set_reference(channel, channel.stats.settled(config_.settle_rule()));
    if (channel.builder) channel.builder->add(channel.window, config_.max_range);
}

bool RoomMemory::built(std::int64_t now) const {
    // Every sensor's reference is ready. With the guard, a sensor silent for kResumeSilenceNs does
    // not hold the others (silent channels never block): it builds when it returns. At least one
    // must be ready.
    if (std::all_of(channels_.begin(), channels_.end(), [&](const Channel& c) { return reference_ready(c); })) return true;
    if (!config_.rig_guard) return false;
    bool any = false;
    for (const Channel& channel : channels_) {
        if (reference_ready(channel)) {
            any = true;
        } else if (!channel.silent_since || now - *channel.silent_since < kResumeSilenceNs) {
            return false;
        }
    }
    return any;
}

bool RoomMemory::building() const {
    return std::any_of(channels_.begin(), channels_.end(), [](const Channel& c) { return c.builder.has_value(); });
}

std::vector<msg::ChangeEvent> RoomMemory::close_window(const msg::Time& stamp) {
    std::vector<msg::ChangeEvent> events;
    // Every activity region of this window, all sensors.
    struct Moving {
        std::int64_t bit;
        Region region;
    };
    std::vector<Moving> activity;
    bool dropped = false;
    const auto quota = static_cast<std::size_t>(config_.recent_windows);
    // A window counts only when it enters already tracking. The window that crosses
    // BUILDING → TRACKING is the reference, not part of the learning budget.
    const bool tracking = state_ == msg::MemoryState::tracking;
    if (tracking) ++tracking_windows_;
    const bool learning = tracking && tracking_windows_ <= config_.learn_windows;
    const bool guard = config_.rig_guard;
    const std::int64_t now = nanoseconds(stamp);
    if (guard && tracking) {
        guard_triggers(now);
    } else if (guard) {
        guard_building(now);
    }
    // A guard check holds every promotion and all learning until it decides.
    const bool hold = guard && check_.has_value();
    for (Channel& channel : channels_) {
        const std::size_t frames = std::exchange(channel.frames, 0);
        std::vector<float> points = std::move(channel.points);
        channel.points.clear();
        if (channel.failed && frames == 0) dropped = true;
        channel.failed = 0;
        if (guard) {
            channel.window = points;
            if (!channel.window.empty()) {
                channel.silent_since.reset();
            } else if (!channel.silent_since) {
                channel.silent_since = now;
            }
        }
        // A sensor silent through the build (guard only) builds its reference when it returns,
        // while the others track.
        if (state_ == msg::MemoryState::building || channel.builder) {
            build(channel, frames, points);
            continue;
        }
        const Config& config = channel.config;
        if (guard && !channel.pose.is_identity()) points = rig::corrected(points, channel.pose, config_.max_range);
        const auto push_recent = [&](CellSummary summary) {
            channel.recent.push_back(std::move(summary));
            while (channel.recent.size() > quota) channel.recent.pop_front();
        };
        // A window without usable points still ages the recent layer, but sends nothing.
        CellSummary summary;
        OccupancyMap window;
        if (frames) {
            summary = summarize(points, config);
            window = occupancy_from(summary, config);
        }
        if (!channel.reference_index) channel.reference_index = std::make_shared<const Membership>(channel.reference.occupied);
        if (guard && channel.mode != Channel::Mode::normal) {
            // Moved (or moved while silent): no diff, tracker or learning until it is re-aligned.
            channel.tracker.interrupt();
            if (!window.occupied.empty()) {
                channel.shift = pose_shift(window.occupied, *channel.reference_index, config_.voxel_size);
            }
            continue;
        }
        if (window.occupied.empty()) {
            push_recent(CellSummary{});
            channel.tracker.interrupt();
            continue;
        }
        push_recent(std::move(summary));
        OccupancyMap recent;
        if (channel.recent.size() == 1) {
            recent = window;
        } else {
            std::vector<const CellSummary*> parts;
            for (const CellSummary& part : channel.recent) parts.push_back(&part);
            recent = occupancy_from(merge(parts), config);
        }
        // The rig-pose estimate uses this window alone, so a move shows in full at once.
        channel.shift = pose_shift(window.occupied, *channel.reference_index, config_.voxel_size);
        channel.residual = channel.shift ? std::optional<double>(shift_norm(*channel.shift)) : std::nullopt;
        if (guard) {
            const bool over = channel.residual && *channel.residual > config_.pose_trigger_m;
            channel.pose_over = over ? channel.pose_over + 1 : 0;
        }
        // Voxels the reference has not settled keep counting, so new coverage joins it, until
        // the learning budget is spent, and unless the rig may have moved.
        if (learning && !hold && !channel.learn_off && channel.residual && *channel.residual <= config_.learn_max_offset_m) {
            const VoxelLayer observed = VoxelLayer::observe(window, config, backend_);
            channel.stats.add(set_difference(observed.occupied, channel.reference.known),
                              set_difference(observed.known, channel.reference.known));
            settle_pending_ = true;
        }
        DiffOptions options;
        options.before_known = &channel.reference.known;
        options.scope = scope_;
        options.backend = backend_;
        const Report report = diff(channel.reference.as_occupancy(change_.voxel_size), recent, config, options);
        // The recent layer carries a return and its ray for recent_windows windows, so only this
        // window's own evidence counts toward the persistence gate: an appeared or moved region
        // is fresh when this window occupies one of its (new-pose) voxels, a disappeared one
        // when this window's rays traverse one of its voxels.
        Keys gone;
        for (const Region& region : report.regions) {
            if (region.kind == Region::Kind::disappeared) gone = set_union(gone, region.keys);
            if (region.kind == Region::Kind::moved) gone = set_union(gone, region.from_keys);
        }
        const Keys free = gone.empty() ? Keys{} : cast_rays(window, config, &gone, backend_);
        std::vector<bool> supported(report.regions.size());
        for (std::size_t n = 0; n < supported.size(); ++n) {
            supported[n] = region_fresh(report.regions[n], window.occupied, free, config_.fresh_min_voxels,
                                        config_.fresh_min_fraction);
        }
        for (const auto& [state, candidate] :
             channel.tracker.update(report.regions, stamp, [this] { return next_id(); }, supported, hold)) {
            events.push_back(event(channel, state, candidate, stamp));
            log_event(events.back());
        }
        for (const std::size_t n : channel.tracker.activity()) activity.push_back(Moving{channel.sensor.bit, report.regions[n]});
    }
    if (dropped) ++dropped_windows_;
    last_window_dropped_ = dropped;
    if (!activity.empty()) {
        // The largest region, ties to the lower sensor bit, then the earlier region.
        const Moving* largest = &activity.front();
        msg::Activity summary;
        summary.active = true;
        summary.n_regions = static_cast<std::int64_t>(activity.size());
        for (const Moving& moving : activity) {
            summary.voxel_count += moving.region.voxel_count;
            summary.sensors |= moving.bit;
            if (moving.region.voxel_count > largest->region.voxel_count ||
                (moving.region.voxel_count == largest->region.voxel_count && moving.bit < largest->bit)) {
                largest = &moving;
            }
        }
        summary.largest_sensor = largest->bit;
        summary.centroid = largest->region.centroid;
        summary.aabb_min = largest->region.bounds_min;
        summary.aabb_max = largest->region.bounds_max;
        activity_ = std::move(summary);
        last_activity_ns_ = nanoseconds(stamp);
    }
    if (state_ == msg::MemoryState::building && built(now)) {
        state_ = msg::MemoryState::tracking;
        // From here on only the voxels the reference has not settled keep their counts.
        // learn_windows 0 drops those leftovers: this run teaches the reference nothing.
        for (Channel& channel : channels_) {
            if (reference_ready(channel)) channel.stats.erase(channel.reference.known);
        }
        if (config_.learn_windows == 0) {
            for (Channel& channel : channels_) {
                if (reference_ready(channel)) channel.stats = VoxelStats{};
            }
            settle_pending_ = false;
        } else {
            for (const Channel& channel : channels_) {
                if (reference_ready(channel) && !channel.stats.keys.empty()) {
                    settle_pending_ = true;
                    break;
                }
            }
        }
        ++map_epoch_;
        if (guard) guard_built(now);
        save(stamp);
    } else if (guard && tracking) {
        guard_step(now, stamp, events);
    }
    if (guard) {
        for (Channel& channel : channels_) channel.window.clear();
        if (rig_save_pending_) save_rig();
    }
    return events;
}

void RoomMemory::log_event(const msg::ChangeEvent& event) {
    Logged entry{event.header.seq, event.header.map_epoch, event.to_cdr()};
    if (event.state == msg::EventState::candidate || event.state == msg::EventState::confirmed) {
        open_latest_[event.event_id] = entry;
    } else {
        open_latest_.erase(event.event_id);
    }
    event_log_.push_back(std::move(entry));
    if (event_log_.size() > kEventLogDepth) event_log_.pop_front();
}

msg::MapBricks RoomMemory::map_message(msg::MapKind kind, const msg::Time& stamp) const {
    msg::MapBricks message;
    message.header = msg::Header{config_.room_id, map_epoch_, map_seq_, stamp};
    message.compression = msg::Compression::zlib;
    message.kind = kind;
    message.voxel_size = static_cast<float>(config_.voxel_size);
    message.publisher_generation = publisher_generation_;
    return message;
}

msg::Bytes RoomMemory::publish_map(const msg::MapBricks& message) {
    msg::Bytes bytes = message.to_cdr();
    const std::int64_t ns = nanoseconds(message.header.stamp);
    if (message.kind == msg::MapKind::snapshot) {
        last_keyframe_ns_ = ns;
        keyframe_bytes_ = bytes.size();
        delta_bytes_ = 0;
    } else {
        delta_bytes_ += bytes.size();
    }
    last_map_ns_ = ns;
    map_epoch_sent_ = message.header.map_epoch;
    map_log_.push_back(Logged{map_seq_, message.header.map_epoch, bytes});
    if (map_log_.size() > kMapLogDepth) map_log_.pop_front();
    ++map_seq_;
    return bytes;
}

std::optional<msg::Bytes> RoomMemory::map_update(const msg::Time& stamp) {
    if (state_ == msg::MemoryState::building || channels_[map_channel_].builder) return std::nullopt;
    const std::int64_t ns = nanoseconds(stamp);
    if (!map_started_) {
        map_started_ = true;
        map_bricks_ = channels_[map_channel_].reference.to_bricks();
        msg::MapBricks snapshot = map_message(msg::MapKind::snapshot, stamp);
        snapshot.bricks = map_bricks_;
        return publish_map(snapshot);
    }
    const bool keyframe_due = ns - last_keyframe_ns_ >= period_ns(config_.keyframe_period_s);
    if (!keyframe_due && ns - last_map_ns_ < period_ns(config_.delta_period_s)) return std::nullopt;
    bool changed = false;
    if (settle_pending_) {
        for (Channel& channel : channels_) {
            if (channel.builder) continue;  // still building: its counts are its reference's
            VoxelLayer fresh = channel.stats.settled(config_.settle_rule());
            if (fresh.known.empty()) continue;
            channel.stats.erase(fresh.known);
            set_reference(channel, channel.reference.merge(fresh));
            changed = true;
        }
        // The budget is spent: merge whatever just settled, then drop the rest, including
        // counts that never reached the rule. A tick that is not due returned above.
        if (tracking_windows_ >= config_.learn_windows) {
            for (Channel& channel : channels_) {
                if (!channel.builder) channel.stats = VoxelStats{};
            }
            settle_pending_ = false;
        }
    }
    if (!changed && !keyframe_due) return std::nullopt;
    if (changed) {
        ++map_epoch_;
        save(stamp);
    }
    std::vector<msg::Brick> bricks = changed ? channels_[map_channel_].reference.to_bricks() : map_bricks_;
    if (!keyframe_due) {
        const MapDelta change = map_delta(map_bricks_, bricks);
        msg::MapBricks delta = map_message(msg::MapKind::delta, stamp);
        delta.base_epoch = map_epoch_sent_;
        delta.bricks = change.bricks;
        delta.removed = change.removed;
        const std::size_t size = delta.to_cdr().size();
        if (static_cast<double>(delta_bytes_ + size) <=
            config_.keyframe_delta_ratio * static_cast<double>(keyframe_bytes_)) {
            map_bricks_ = std::move(bricks);
            return publish_map(delta);
        }
    }
    map_bricks_ = std::move(bricks);
    msg::MapBricks snapshot = map_message(msg::MapKind::snapshot, stamp);
    snapshot.bricks = map_bricks_;
    return publish_map(snapshot);
}

RoomMemory::SyncAnswer RoomMemory::sync(const msg::RequestSync& request, const msg::Time& stamp) {
    SyncAnswer answer;
    if (request.header.room_id != config_.room_id) return answer;
    const std::uint64_t expected = request.expected_generation.value_or(msg::kGenerationUnknown);
    const std::optional<msg::RequestSync::Since> since =
        expected == publisher_generation_ ? request.since : std::nullopt;
    const auto since_index = [&](const std::deque<Logged>& log) -> std::optional<std::size_t> {
        if (!since) return std::nullopt;
        for (std::size_t n = 0; n < log.size(); ++n) {
            if (log[n].seq == since->seq && log[n].map_epoch == since->epoch) return n;
        }
        return std::nullopt;
    };
    std::vector<msg::Bytes>& messages = answer.messages;
    std::optional<std::uint64_t> bound;
    std::vector<msg::SeqRange> delivered, unavailable;
    if (request.topic == msg::SyncTopic::map_bricks) {
        if (map_started_) bound = map_seq_ - 1;
        const auto at = since_index(map_log_);
        if (!bound || (at && *at + 1 == map_log_.size())) {
            // Nothing published yet (BUILDING), or the Mac holds the newest message.
        } else if (at) {
            for (std::size_t n = *at + 1; n < map_log_.size(); ++n) messages.push_back(map_log_[n].bytes);
            delivered = {{since->seq + 1, *bound}};
        } else {
            // A fresh snapshot of the last live state, under the latest live seq.
            msg::MapBricks snapshot = map_message(msg::MapKind::snapshot, stamp);
            snapshot.header.seq = *bound;
            snapshot.bricks = map_bricks_;
            messages.push_back(snapshot.to_cdr());
            delivered = {{*bound, *bound}};
        }
    } else {
        if (event_seq_) bound = event_seq_ - 1;
        const auto at = since_index(event_log_);
        if (!bound) {
            // Nothing published in this generation: known empty.
        } else if (at) {
            for (std::size_t n = *at + 1; n < event_log_.size(); ++n) messages.push_back(event_log_[n].bytes);
            if (!messages.empty()) delivered = {{since->seq + 1, *bound}};
        } else {
            // Bounded fallback: open events whose latest message is older than the kept ones, then
            // every kept message. What was requested and neither is sent can never be served again.
            const std::uint64_t oldest = event_log_.front().seq;
            std::vector<const Logged*> older;
            for (const auto& [id, entry] : open_latest_) {
                if (entry.seq < oldest) older.push_back(&entry);
            }
            std::sort(older.begin(), older.end(), [](const Logged* a, const Logged* b) { return a->seq < b->seq; });
            std::vector<msg::SeqRange> sent;
            for (const Logged* entry : older) {
                messages.push_back(entry->bytes);
                sent.push_back({entry->seq, entry->seq});
            }
            for (const Logged& entry : event_log_) messages.push_back(entry.bytes);
            sent.push_back({oldest, *bound});
            delivered = canonical_ranges(std::move(sent));
            const std::uint64_t first = since ? since->seq + 1 : 0;
            if (first <= *bound) unavailable = subtract_ranges({{first, *bound}}, delivered);
        }
    }
    msg::SyncEnd& end = answer.end.emplace();
    end.header = msg::Header{config_.room_id, map_epoch_, sync_end_seq_++, stamp};
    end.publisher_generation = publisher_generation_;
    end.token = request.token.value_or(msg::kZeroToken);
    end.topic = request.topic;
    end.bound = bound;
    end.delivered = std::move(delivered);
    end.unavailable = std::move(unavailable);
    return answer;
}

msg::ChangeEvent RoomMemory::event(const Channel& channel, msg::EventState state, const Candidate& candidate,
                                   const msg::Time& stamp) {
    const Region& region = candidate.region;
    msg::ChangeEvent e;
    e.header = msg::Header{config_.room_id, map_epoch_, event_seq_++, stamp};
    e.event_id = candidate.event_id;
    e.kind = candidate.kind;
    e.state = state;
    e.centroid = region.centroid;
    e.aabb_min = region.bounds_min;
    e.aabb_max = region.bounds_max;
    if (region.kind == Region::Kind::moved) e.from_centroid = region.from_centroid;
    e.voxel_count = region.voxel_count;
    e.volume_m3 = region.volume;
    const double present = static_cast<double>(count_present(channel.reference.known, candidate.keys));
    e.evidence = static_cast<float>(present / static_cast<double>(candidate.keys.size()));
    e.sensors = channel.sensor.bit;
    e.first_seen = candidate.first_seen;
    e.last_seen = candidate.last_seen;
    msg::MapBricks mask;
    mask.header = msg::Header{config_.room_id, map_epoch_, 0, stamp};
    mask.compression = msg::Compression::zlib;
    mask.kind = msg::MapKind::snapshot;
    mask.voxel_size = static_cast<float>(config_.voxel_size);
    VoxelLayer layer;
    layer.occupied = candidate.keys;
    layer.known = candidate.keys;
    mask.bricks = layer.to_bricks();
    mask.publisher_generation = publisher_generation_;
    e.mask = std::move(mask);
    e.publisher_generation = publisher_generation_;
    return e;
}

bool region_fresh(const Region& region, const Keys& occupied, const Keys& free, std::int64_t min_voxels,
                  double min_fraction) {
    const auto supported = [&](const Keys& evidence, const Keys& keys) {
        const auto count = static_cast<std::int64_t>(count_present(evidence, keys));
        return count >= min_voxels && static_cast<double>(count) >= min_fraction * static_cast<double>(keys.size());
    };
    if (region.kind == Region::Kind::disappeared) return supported(free, region.keys);
    return supported(occupied, region.keys) && (region.kind != Region::Kind::moved || supported(free, region.from_keys));
}

msg::Activity RoomMemory::activity(const msg::Time& stamp) {
    msg::Activity message = activity_.value_or(msg::Activity{});
    activity_.reset();
    message.header = msg::Header{config_.room_id, map_epoch_, activity_seq_++, stamp};
    message.publisher_generation = publisher_generation_;
    if (last_activity_ns_) {
        message.since_s = static_cast<float>(static_cast<double>(std::max<std::int64_t>(0, nanoseconds(stamp) - *last_activity_ns_)) / 1e9);
    }
    return message;
}

msg::MemoryStatus RoomMemory::status(const msg::Time& stamp, double period_s) {
    std::optional<std::int64_t> hz[2];
    bool silent = false;
    for (Channel& channel : channels_) {
        channel.hz = std::nullopt;
        if (period_s > 0) {
            const double rate = static_cast<double>(channel.received) / period_s + 0.5;
            const auto cap = static_cast<double>(msg::kHzUnavailable - 1);
            channel.hz = rate >= cap ? msg::kHzUnavailable - 1 : static_cast<std::int64_t>(rate);
        }
        channel.received = 0;
        hz[channel.sensor.hz_index] = channel.hz;
        silent = silent || (channel.hz && *channel.hz == 0);
    }
    msg::MemoryStatus message;
    message.header = msg::Header{config_.room_id, map_epoch_, status_seq_++, stamp};
    if (state_ == msg::MemoryState::tracking && any_moved()) {
        message.state = msg::MemoryState::rig_moved;  // outranks DEGRADED: a silent sensor does not hide a move
    } else {
        message.state = last_window_dropped_ || silent ? msg::MemoryState::degraded : state_;
    }
    message.n_bricks = channels_[map_channel_].reference.brick_count();
    for (const Channel& channel : channels_) message.n_open_events += static_cast<std::int64_t>(channel.tracker.open().size());
    message.coverage_known_frac = coverage_;
    if (state_ != msg::MemoryState::building) {
        std::optional<double> residual;
        for (const Channel& channel : channels_) {
            if (channel.residual && (!residual || *channel.residual > *residual)) residual = channel.residual;
        }
        if (residual) message.pose_residual_m = static_cast<float>(*residual);
    }
    message.window_s = static_cast<float>(config_.window_s);
    message.mid360_hz = hz[0];
    message.jt128_hz = hz[1];
    message.dropped_windows = dropped_windows_;
    if (event_seq_) message.events_seq = event_seq_ - 1;
    message.publisher_generation = publisher_generation_;
    return message;
}

// MARK: Rig guard

bool RoomMemory::add_imu(std::span<const std::uint8_t> data, const msg::Time& received) {
    if (!detector_) return false;
    msg::ImuSample sample;
    try {
        sample = msg::decode_imu(data);
    } catch (const std::exception&) {
        ++imu_failed_;
        return false;
    }
    detector_->add(sample, nanoseconds(received));
    return true;
}

bool RoomMemory::any_moved() const {
    return std::any_of(channels_.begin(), channels_.end(), [](const Channel& c) { return c.mode == Channel::Mode::moved; });
}

msg::RigOutcome RoomMemory::rig_outcome() const {
    if (any_moved()) return msg::RigOutcome::moved;
    if (check_) return msg::RigOutcome::checking;
    return outcome_;
}

void RoomMemory::set_pose(Channel& channel, const rig::Pose& pose) {
    channel.pose = pose;
    channel.config = change_;
    if (pose.is_identity()) return;
    channel.config.max_range = std::nullopt;
    for (std::size_t n = 0; n < 3; ++n) channel.config.origin[n] = static_cast<float>(pose.t[n]);
    validate(channel.config);
}

bool RoomMemory::imu_ok(std::int64_t now) const { return detector_ && detector_->ok(now); }

void RoomMemory::start_check(std::int64_t now, std::int64_t causes) {
    check_ = Check{now, causes, 0};
    causes_ = causes;
    tilt_run_ = gross_run_ = 0;
    for (Channel& channel : channels_) channel.results.clear();
    rig_dirty_ = true;
}

void RoomMemory::end_check(std::int64_t now, msg::RigOutcome outcome) {
    check_.reset();
    outcome_ = outcome;
    gyro_.reset();
    const double period = config_.guard_check_period_s;
    next_check_ = period > 0 ? std::optional<std::int64_t>(now + period_ns(period)) : std::nullopt;
    rig_dirty_ = true;
}

void RoomMemory::guard_triggers(std::int64_t now) {
    std::int64_t causes = 0;
    if (detector_->take_shock()) {
        causes |= msg::kCauseImuShock;
        watched_ = false;
    }
    const std::optional<rig::Vec3d> gravity = detector_->take_gravity();
    if (gravity) gravity_ = gravity;
    if (!detector_->ok(now)) {
        watched_ = false;
    } else if (gravity && g_base_ && detector_->quiet(now) &&
               rig::angle_between_deg(*gravity, *g_base_) >= config_.tilt_trigger_deg) {
        causes |= msg::kCauseImuTilt;
    } else if (gravity && !g_base_ && detector_->quiet(now) && !causes && !check_ &&
               std::all_of(channels_.begin(), channels_.end(), [](const Channel& c) { return c.mode == Channel::Mode::normal; })) {
        // No baseline yet (the IMU came up after the build, or a state file without one): quiet
        // gravity with the rig in place starts it, so a tilt is evidence even where ICP never fits.
        g_base_ = gravity;
        save_rig();
    }
    if (next_check_ && now >= *next_check_) causes |= msg::kCausePeriodic;
    for (const Channel& channel : channels_) {
        if (channel.mode == Channel::Mode::normal && channel.pose_over == 2) causes |= msg::kCausePoseOffset;
        if (channel.frames && !channel.builder && channel.silent_since && now - *channel.silent_since >= kResumeSilenceNs) {
            causes |= msg::kCauseSensorResumed;
        }
    }
    if (!causes) return;
    if (check_) {
        check_->causes |= causes;
        causes_ = check_->causes;
    } else if (!any_moved()) {
        start_check(now, causes);
    }
}

void RoomMemory::guard_building(std::int64_t now) {
    // The newest gravity, for the baseline the new frame starts with. A shock or rotation during
    // the build is the reference's, never a trigger after it.
    detector_->take_shock();
    detector_->take_rotation(now);
    if (const std::optional<rig::Vec3d> gravity = detector_->take_gravity()) gravity_ = gravity;
}

void RoomMemory::guard_built(std::int64_t now) {
    for (Channel& channel : channels_) {
        if (reference_ready(channel)) finish_cloud(channel);
    }
    frame_epoch_ = map_epoch_;
    g_base_ = gravity_;
    end_check(now, msg::RigOutcome::in_place);
}

void RoomMemory::finish_cloud(Channel& channel) {
    channel.cloud = std::make_shared<const rig::RegistrationCloud>(channel.builder->finish(
        channel.reference.occupied, config_.voxel_size, static_cast<std::size_t>(config_.reg_max_points)));
    channel.builder.reset();
    channel.registration.reset();
}

void RoomMemory::joined(Channel& channel, const msg::Time& stamp) {
    // A sensor silent through the build finished its reference: it joins the frame, which is then
    // committed whole.
    channel.stats.erase(channel.reference.known);
    if (config_.learn_windows == 0 || tracking_windows_ >= config_.learn_windows) {
        channel.stats = VoxelStats{};
    } else if (!channel.stats.keys.empty()) {
        settle_pending_ = true;
    }
    finish_cloud(channel);
    rig_dirty_ = true;
    if (!building()) save(stamp);
}

void RoomMemory::guard_step(std::int64_t now, const msg::Time& stamp, std::vector<msg::ChangeEvent>& events) {
    for (Channel& channel : channels_) {
        if (channel.builder && reference_ready(channel)) joined(channel, stamp);
        channel.available = !channel.window.empty() && channel.cloud;
        if (channel.mode == Channel::Mode::stale && channel.available) {
            channel_moved(channel, stamp, events);  // moved while it was silent
            if (!moved_since_) moved_since_ = now;  // the others re-aligned while it was silent
        } else if (channel.mode == Channel::Mode::moved && !channel.available) {
            channel.mode = Channel::Mode::stale;
            channel.estimates.clear();
            rig_dirty_ = true;
        }
    }
    if (detector_) {
        if (auto gyro = detector_->take_rotation(now)) gyro_ = gyro;
    }
    if (check_) decide(now, stamp, events);
    if (any_moved()) track_moved(now, stamp, events);
}

std::optional<rig::Mat3> RoomMemory::rotation_prior(const Channel& channel) const {
    std::optional<rig::Mat3> R = gyro_;
    if (!R && gravity_ && g_base_ && rig::angle_between_deg(*gravity_, *g_base_) >= config_.tilt_trigger_deg) {
        R = rig::rotation_between(*gravity_, *g_base_);
    }
    if (!R) return std::nullopt;
    if (channel.sensor.name == "jt128") {
        const rig::Pose M{mount(), {}};
        R = M.compose(rig::Pose{*R, {}}).compose(M.inverse()).R;
    }
    return R;
}

msg::RigChannel RoomMemory::estimate(Channel& channel, const std::optional<rig::Pose>& seed_from_last) {
    const double limit = config_.max_range ? *config_.max_range : config_.icp_max_range_m;
    const std::vector<double> source = rig::source_cloud(channel.window, limit, config_.src_voxel_m,
                                                         static_cast<std::size_t>(config_.src_max_points));
    channel.last_estimate.reset();
    channel.last_fit = false;
    if (source.size() / 3 < kMinSourcePoints) return msg::RigChannel::rejected;
    const double scale = rig::median_range(source);
    const rig::Vec3d shift = channel.shift.value_or(rig::Vec3d{0.0, 0.0, 0.0});
    const double reach = std::sqrt((shift[0] * shift[0] + shift[1] * shift[1]) + shift[2] * shift[2]);
    rig::Pose seed;
    double sigma = 0.0;
    if (seed_from_last) {
        seed = *seed_from_last;
        sigma = 0.01;
    } else if (const auto prior = rotation_prior(channel)) {
        const rig::Pose base = channel.pose.compose(rig::Pose{*prior, {}});
        seed = rig::Pose{base.R, {base.t[0] + shift[0], base.t[1] + shift[1], base.t[2] + shift[2]}};
        sigma = reach + radians(0.2) * scale;
    } else {
        seed = rig::sweep_seed(source, channel.pose, shift, channel.reference.occupied, config_.voxel_size,
                               config_.align_max_deg)
                   .first;
        sigma = reach + radians(0.5 * rig::kSweepStepDeg) * scale;
    }
    channel.last_estimate = rig::align(source, *channel.cloud, seed, sigma);
    if (!channel.last_estimate) return msg::RigChannel::rejected;
    const rig::Estimate& estimate = *channel.last_estimate;
    const double floor = channel.baseline ? std::max(0.5, config_.align_inlier_ratio * *channel.baseline) : 0.5;
    channel.last_fit = estimate.inlier_frac >= floor && estimate.rms_m <= config_.align_max_rms_m &&
                       estimate.observe >= config_.observe_min_eig;
    if (!channel.last_fit) return msg::RigChannel::rejected;
    const rig::Pose increment = channel.pose.increment(estimate.pose);
    const bool moved = increment.distance_m() >= config_.move_min_m ||
                       !(increment.cos_angle() > std::cos(radians(config_.move_min_deg)));
    return moved ? msg::RigChannel::moved : msg::RigChannel::in_place;
}

void RoomMemory::decide(std::int64_t now, const msg::Time& stamp, std::vector<msg::ChangeEvent>& events) {
    const std::int64_t age = now - check_->start;
    const bool imu = imu_ok(now);
    if (imu && !detector_->quiet(now) && age < period_ns(config_.check_max_s)) {
        return;  // the rig is still vibrating: wait for quiet, at most check_max_s
    }
    std::vector<Channel*> available;
    for (Channel& channel : channels_) {
        if (channel.mode == Channel::Mode::normal && channel.available) available.push_back(&channel);
    }
    if (available.empty()) return;  // nothing to look with: the check waits for a channel
    ++check_->decisions;
    decision_.clear();
    for (Channel& channel : channels_) {
        if (channel.mode != Channel::Mode::normal) continue;
        const msg::RigChannel result = channel.available ? estimate(channel) : msg::RigChannel::unavailable;
        channel.results.push_back(result);
        decision_[channel.sensor.name] = result;
    }
    rig_dirty_ = true;
    const auto k = static_cast<std::size_t>(config_.align_stable_windows);
    const auto stable_in_place = [&](const Channel& c) {
        return c.results.size() >= k && std::all_of(c.results.end() - static_cast<std::ptrdiff_t>(k), c.results.end(),
                                                    [](msg::RigChannel r) { return r == msg::RigChannel::in_place; });
    };
    const bool moved =
        std::any_of(available.begin(), available.end(), [](const Channel* c) { return c->results.back() == msg::RigChannel::moved; });
    const bool tilted = imu && gravity_ && g_base_ &&
                        rig::angle_between_deg(*gravity_, *g_base_) >= std::max(config_.tilt_trigger_deg, config_.move_min_deg);
    tilt_run_ = tilted ? tilt_run_ + 1 : 0;
    const bool mismatch = std::all_of(available.begin(), available.end(), [&](const Channel* c) {
        // Measured low overlap only: a window without an estimate is a rejection, never evidence.
        return c->baseline && c->last_estimate && c->last_estimate->inlier_frac < config_.gross_inlier_ratio * *c->baseline;
    });
    gross_run_ = mismatch ? gross_run_ + 1 : 0;
    if (moved || tilt_run_ >= config_.align_stable_windows || (gross_run_ >= 2 && !watched_)) {
        enter_moved(now, stamp, events);
        return;
    }
    if (std::all_of(available.begin(), available.end(), [&](const Channel* c) { return stable_in_place(*c); })) {
        for (Channel* channel : available) {
            channel->baseline = channel->last_estimate->inlier_frac;
            channel->learn_off = false;
        }
        if (imu && gravity_) g_base_ = gravity_;
        watched_ = imu;
        end_check(now, msg::RigOutcome::in_place);
        save_rig();
        return;
    }
    if ((age >= period_ns(config_.check_max_s) && check_->decisions >= config_.align_stable_windows) ||
        age >= period_ns(2 * config_.check_max_s)) {
        // No evidence and no agreement: keep the pose, release the hold, never relearn.
        for (Channel* channel : available) {
            if (channel->results.back() == msg::RigChannel::rejected) channel->learn_off = true;
        }
        end_check(now, msg::RigOutcome::inconclusive);
    }
}

void RoomMemory::enter_moved(std::int64_t now, const msg::Time& stamp, std::vector<msg::ChangeEvent>& events) {
    const auto k = static_cast<std::size_t>(config_.align_stable_windows);
    const std::int64_t start = check_->start;
    bool any = false;
    for (Channel& channel : channels_) {
        if (channel.mode != Channel::Mode::normal) continue;
        const bool in_place = channel.results.size() >= k &&
                              std::all_of(channel.results.end() - static_cast<std::ptrdiff_t>(k), channel.results.end(),
                                          [](msg::RigChannel r) { return r == msg::RigChannel::in_place; });
        if (channel.available && !in_place) {
            channel_moved(channel, stamp, events);
            any = true;
        } else if (!channel.available && !channel.builder) {
            channel.mode = Channel::Mode::stale;
        }
    }
    if (!any) {
        // The IMU or the overlap said moved, every channel's own fit says in place: trust the fit.
        for (Channel& channel : channels_) {
            if (channel.mode == Channel::Mode::stale) channel.mode = Channel::Mode::normal;
        }
        end_check(now, msg::RigOutcome::in_place);
        return;
    }
    for (Channel& channel : channels_) {
        if (channel.builder) restart_build(channel);
    }
    check_.reset();
    moved_since_ = now;
    moved_at_ = msg::Time{start / 1'000'000'000, start % 1'000'000'000};
    watched_ = false;
    rig_dirty_ = true;
}

void RoomMemory::channel_moved(Channel& channel, const msg::Time& stamp, std::vector<msg::ChangeEvent>& events) {
    for (const Candidate& candidate : channel.tracker.drop_all()) {
        events.push_back(event(channel, msg::EventState::expired, candidate, stamp));
        log_event(events.back());
    }
    channel.recent.clear();
    channel.mode = Channel::Mode::moved;
    channel.estimates.clear();
    channel.results.clear();
    rig_dirty_ = true;
}

void RoomMemory::restart_build(Channel& channel) {
    // A sensor still building its reference when the rig moved: the windows so far saw the old pose,
    // so it starts over in the new one.
    set_reference(channel, VoxelLayer{});
    channel.stats = VoxelStats{};
    channel.reference_windows = 0;
    channel.builder.emplace(config_.reg_voxel_m);
}

void RoomMemory::track_moved(std::int64_t now, const msg::Time& stamp, std::vector<msg::ChangeEvent>& events) {
    const auto k = static_cast<std::size_t>(config_.align_stable_windows);
    std::vector<Channel*> moved;
    for (Channel& channel : channels_) {
        if (channel.mode == Channel::Mode::moved) moved.push_back(&channel);
    }
    decision_.clear();
    for (Channel* channel : moved) {
        std::optional<rig::Pose> last;
        if (!channel->estimates.empty()) {
            last = channel->estimates.back().pose;
        } else if (channel->last_fit && channel->last_estimate) {
            last = channel->last_estimate->pose;
        }
        const msg::RigChannel result = estimate(*channel, last);
        decision_[channel->sensor.name] = result;
        if (result == msg::RigChannel::rejected) {
            channel->estimates.clear();
        } else {
            channel->estimates.push_back(*channel->last_estimate);
            if (channel->estimates.size() > k) channel->estimates.erase(channel->estimates.begin());
        }
    }
    rig_dirty_ = true;
    const double cos_max = std::cos(radians(config_.align_max_deg));
    const double cos_stable = std::cos(radians(kStableDeg));
    const auto ready = [&](const Channel* channel) {
        if (channel->estimates.size() < k) return false;
        const rig::Pose& newest = channel->estimates.back().pose;
        for (std::size_t n = 0; n + 1 < channel->estimates.size(); ++n) {
            const rig::Pose difference = channel->estimates[n].pose.increment(newest);
            if (difference.distance_m() > kStableM || difference.cos_angle() < cos_stable) return false;
        }
        return newest.distance_m() <= config_.align_max_m && newest.cos_angle() >= cos_max;
    };
    if (std::all_of(moved.begin(), moved.end(), ready)) {
        bool realigned = false;
        for (Channel* channel : moved) {
            const rig::Estimate estimate = channel->estimates.back();
            const rig::Pose increment = channel->pose.increment(estimate.pose);
            if (increment.distance_m() >= config_.move_min_m ||
                !(increment.cos_angle() > std::cos(radians(config_.move_min_deg)))) {
                channel->delta = std::make_pair(increment.distance_m(), increment.angle_deg());
                set_pose(*channel, estimate.pose);
                realigned = true;
            }
            channel->baseline = estimate.inlier_frac;
            channel->mode = Channel::Mode::normal;
            channel->estimates.clear();
            channel->results.clear();
            channel->residual.reset();
            channel->pose_over = 0;
            channel->learn_off = false;
        }
        if (realigned) ++n_realigned_;
        moved_since_.reset();
        const bool imu = imu_ok(now);
        if (imu && gravity_) g_base_ = gravity_;
        watched_ = imu;
        end_check(now, realigned ? msg::RigOutcome::realigned : msg::RigOutcome::in_place);
        save_rig();
        return;
    }
    // Every moved channel delivered this window (a silent one went stale), so the rebuild has a
    // sensor to build on; silent ones build when they return (built).
    if (config_.auto_relearn && now - *moved_since_ >= period_ns(config_.relearn_after_s)) relearn(stamp, events);
}

void RoomMemory::relearn(const msg::Time& stamp, std::vector<msg::ChangeEvent>& events) {
    // Archives the state and rebuilds the room through BUILDING: a new room frame.
    if (config_.state_dir) {
        std::vector<std::string> files{"map_epoch", "rig_state"};
        for (const std::filesystem::path& path : paths()) files.push_back(path.filename().string());
        for (const Channel& channel : channels_) files.push_back(registration_name(channel.sensor.name));
        // A failed archive (disk full) does not stop the relearn: the old frame's files stay in place
        // until the new frame commits over them, so they are lost then, not now.
        try {
            archive(room_dir(*config_.state_dir, config_.room_id), "frame-" + std::to_string(frame_epoch_), files);
        } catch (const std::exception& error) {
            ++rig_save_failed_;
            rig_save_error_ = error.what();
        }
    }
    rig_save_pending_ = false;  // the old frame's files are moot; the new frame saves when it is built
    for (Channel& channel : channels_) {
        for (const Candidate& candidate : channel.tracker.drop_all()) {
            events.push_back(event(channel, msg::EventState::expired, candidate, stamp));
            log_event(events.back());
        }
        channel.recent.clear();
        set_reference(channel, VoxelLayer{});
        channel.stats = VoxelStats{};
        channel.reference_windows = 0;
        set_pose(channel, rig::Pose{});
        channel.cloud.reset();
        channel.registration.reset();
        channel.builder.emplace(config_.reg_voxel_m);
        channel.mode = Channel::Mode::normal;
        channel.baseline.reset();
        channel.results.clear();
        channel.estimates.clear();
        channel.last_estimate.reset();
        channel.last_fit = false;
        channel.delta.reset();
        channel.residual.reset();
        channel.shift.reset();
        channel.pose_over = 0;
        channel.learn_off = false;
    }
    state_ = msg::MemoryState::building;
    map_started_ = false;
    map_bricks_.clear();
    map_log_.clear();
    settle_pending_ = false;
    tracking_windows_ = 0;
    check_.reset();
    moved_since_.reset();
    next_check_.reset();
    n_realigned_ = 0;
    outcome_ = msg::RigOutcome::relearning;
    rig_dirty_ = true;
}

std::optional<msg::RigPose> RoomMemory::rig_pose(const msg::Time& stamp) {
    if (!config_.rig_guard) return std::nullopt;
    const std::int64_t now = nanoseconds(stamp);
    const msg::RigOutcome outcome = rig_outcome();
    const bool due = rig_dirty_ || !rig_sent_ || rig_sent_->first != outcome || now - rig_sent_->second >= kRigPosePeriodNs;
    if (!due) return std::nullopt;
    msg::RigPose message;
    message.header = msg::Header{config_.room_id, map_epoch_, rig_seq_, stamp};
    message.outcome = outcome;
    message.causes = causes_;
    for (const Channel& channel : channels_) {
        const auto slot = static_cast<std::size_t>(channel.sensor.hz_index);
        (slot == 0 ? message.mid360_pose : message.jt128_pose) = channel.pose.rig_pose();
        if (channel.delta) {
            message.delta_m[slot] = static_cast<float>(channel.delta->first);
            message.delta_deg[slot] = static_cast<float>(channel.delta->second);
        }
        if (channel.last_estimate) {
            message.inlier_frac[slot] = static_cast<float>(channel.last_estimate->inlier_frac);
            message.rms_m[slot] = static_cast<float>(channel.last_estimate->rms_m);
            message.observe[slot] = static_cast<float>(channel.last_estimate->observe);
        }
        const auto found = decision_.find(channel.sensor.name);
        const msg::RigChannel result = found == decision_.end() ? msg::RigChannel::none : found->second;
        message.channel[slot] = result;
        if (result != msg::RigChannel::none && result != msg::RigChannel::unavailable) message.sensors |= channel.sensor.bit;
    }
    if (gravity_ && g_base_) message.tilt_deg = static_cast<float>(rig::angle_between_deg(*gravity_, *g_base_));
    message.frame_epoch = frame_epoch_;
    message.n_realigned = n_realigned_;
    message.imu_ok = imu_ok(now);
    message.moved_at = moved_at_;
    message.publisher_generation = publisher_generation_;
    ++rig_seq_;
    rig_sent_ = std::make_pair(outcome, now);
    rig_dirty_ = false;
    return message;
}

void RoomMemory::load_rig(const std::filesystem::path& room, const std::filesystem::path& path) {
    const RigState state = parse_rig_state(read_bytes(path), path.string());
    if (state.room_id != config_.room_id || state.voxel_size != config_.voxel_size) {
        std::ostringstream text;
        text << path.string() << " holds room '" << state.room_id << "' at voxel_size " << state.voxel_size << ", not '"
             << config_.room_id << "' at " << config_.voxel_size;
        throw std::invalid_argument(text.str());
    }
    std::vector<std::string> saved, configured;
    for (const auto& [name, sensor] : state.sensors) saved.push_back(name);
    for (const Channel& channel : channels_) configured.push_back(channel.sensor.name);
    std::sort(saved.begin(), saved.end());
    std::sort(configured.begin(), configured.end());
    if (saved != configured) {
        throw std::invalid_argument(path.string() + " holds sensors " + sensor_list(saved) + ", not " + sensor_list(configured));
    }
    for (const auto& [name, sensor] : state.sensors) {
        const RegistrationFile& file = sensor.registration;
        if (file.name != registration_name(name)) {
            throw std::invalid_argument(path.string() + ": " + name + " names registration file '" + file.name + "'");
        }
        const std::filesystem::path cloud_path = room / file.name;
        std::optional<msg::Bytes> data;
        if (std::filesystem::exists(cloud_path)) data = read_bytes(cloud_path);
        if (!data || data->size() != file.size ||
            static_cast<std::uint32_t>(::crc32(0L, data->data(), static_cast<uInt>(data->size()))) != file.crc) {
            throw std::invalid_argument(cloud_path.string() + " does not match " + path.string() +
                                        " (missing, or another size or CRC)");
        }
        Registration registration = parse_registration(*data, cloud_path.string());
        if (registration.sensor != name || registration.frame_epoch != state.frame_epoch) {
            throw std::invalid_argument(cloud_path.string() + " is " + registration.sensor + "'s cloud of frame " +
                                        std::to_string(registration.frame_epoch) + ", not " + name + "'s of frame " +
                                        std::to_string(state.frame_epoch));
        }
        Channel& channel = find(name);
        channel.cloud = std::make_shared<const rig::RegistrationCloud>(std::move(registration.points), config_.reg_voxel_m);
        channel.registration = file;
        channel.builder.reset();
        channel.baseline = sensor.baseline;
        set_pose(channel, sensor.pose);
    }
    frame_epoch_ = state.frame_epoch;
    n_realigned_ = state.n_realigned;
    g_base_ = state.g_base;
}

void RoomMemory::save_rig() {
    // The registration clouds not yet written, then rig_state (last: it names them). A failure (disk
    // full, permissions) never leaves close_window: the guard keeps its state in memory, the files
    // on disk stay the last set written, and close_window retries until a save goes through.
    if (!config_.state_dir) return;
    if (std::any_of(channels_.begin(), channels_.end(), [](const Channel& c) { return !c.cloud; })) return;
    try {
        const std::filesystem::path room = room_dir(*config_.state_dir, config_.room_id);
        RigState state{config_.room_id, config_.voxel_size, frame_epoch_, n_realigned_, g_base_, {}};
        for (Channel& channel : channels_) {
            if (!channel.registration) {
                const std::vector<std::uint8_t> data =
                    registration_bytes(channel.sensor.name, frame_epoch_, channel.cloud->points());
                const std::string file = registration_name(channel.sensor.name);
                write_atomic(room / file, data);
                const auto crc = static_cast<std::uint32_t>(::crc32(0L, data.data(), static_cast<uInt>(data.size())));
                channel.registration = RegistrationFile{file, data.size(), crc};
            }
            state.sensors.emplace_back(channel.sensor.name,
                                       SensorState{channel.pose, channel.baseline, *channel.registration});
        }
        write_atomic(room / "rig_state", format_rig_state(state));
        rig_save_pending_ = false;
    } catch (const std::exception& error) {
        rig_save_pending_ = true;
        ++rig_save_failed_;
        rig_save_error_ = error.what();
    }
}

}  // namespace room_memory
