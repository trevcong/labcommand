#include "room_memory/memory.hpp"

#include <fcntl.h>
#include <pwd.h>
#include <unistd.h>

#include <array>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iterator>
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

// Python's os.path.expanduser for "~" and "~user" prefixes.
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

std::optional<float> pose_offset(const Keys& recent, const Membership& reference, double voxel_size) {
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
    double squared = 0.0;
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
        const double meters = shift * voxel_size;
        squared += meters * meters;
    }
    return static_cast<float>(std::sqrt(squared));
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

std::filesystem::path reference_path(const std::string& state_dir, const std::string& room_id, const std::string& sensor) {
    if (!plain_name(room_id)) {
        throw std::invalid_argument("room_id '" + room_id + "' is not a plain file name ([A-Za-z0-9._-], no leading dot)");
    }
    return expand_user(state_dir) / room_id / ("reference-" + sensor + ".cdr");
}

void save_reference(const std::filesystem::path& path, const VoxelLayer& layer, const msg::Header& header,
                    float voxel_size) {
    msg::MapBricks message;
    message.header = header;
    message.compression = msg::Compression::zlib;
    message.kind = msg::MapKind::snapshot;
    message.voxel_size = voxel_size;
    message.bricks = layer.to_bricks();
    const msg::Bytes data = message.to_cdr();
    std::filesystem::create_directories(path.parent_path());
    const std::filesystem::path scratch = path.string() + ".tmp";
    const int fd = ::open(scratch.c_str(), O_WRONLY | O_CREAT | O_TRUNC, 0644);
    if (fd < 0) throw std::runtime_error("cannot write " + scratch.string());
    std::size_t written = 0;
    while (written < data.size()) {
        const ssize_t n = ::write(fd, data.data() + written, data.size() - written);
        if (n < 0) {
            ::close(fd);
            throw std::runtime_error("cannot write " + scratch.string());
        }
        written += static_cast<std::size_t>(n);
    }
    const bool synced = ::fsync(fd) == 0;
    ::close(fd);
    if (!synced) throw std::runtime_error("cannot sync " + scratch.string());
    std::filesystem::rename(scratch, path);
}

std::pair<msg::MapBricks, VoxelLayer> load_reference(const std::filesystem::path& path) {
    std::ifstream in(path, std::ios::binary);
    if (!in) throw std::runtime_error("cannot read " + path.string());
    const msg::Bytes data((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    msg::MapBricks message = msg::MapBricks::from_cdr(data);
    VoxelLayer layer = VoxelLayer::from_bricks(message.bricks);
    return {std::move(message), std::move(layer)};
}

// MARK: Candidates

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

RoomMemory::RoomMemory(MemoryConfig config, const msg::Time& start, const RayBackend* backend, DiffScope scope)
    : config_(std::move(config)), backend_(backend), scope_(scope) {
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
        channel.tracker = CandidateTracker(config_.candidate_windows, config_.expire_windows);
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
    change_ = config_.change_config();
    validate(change_);
    const std::int64_t millis = start.sec * 1000 + start.nanosec / 1'000'000;
    next_event_id_ = static_cast<std::uint64_t>(millis) << 20;
    load();
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

void RoomMemory::load() {
    if (!config_.state_dir) return;
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
    if (present == 0) return;
    if (!missing.empty()) {
        std::sort(missing.begin(), missing.end());
        throw std::invalid_argument("saved references are incomplete for room '" + config_.room_id + "': missing " +
                                    sensor_list(missing));
    }
    std::vector<std::uint64_t> epochs;
    for (std::size_t n = 0; n < all.size(); ++n) {
        auto [message, layer] = load_reference(all[n]);
        if (message.header.room_id != config_.room_id) {
            throw std::invalid_argument(all[n].string() + " holds room '" + message.header.room_id + "', not '" +
                                        config_.room_id + "'");
        }
        if (message.voxel_size != static_cast<float>(config_.voxel_size)) {
            throw std::invalid_argument(all[n].string() + " uses a different voxel_size");
        }
        if (std::find(epochs.begin(), epochs.end(), message.header.map_epoch) == epochs.end()) {
            epochs.push_back(message.header.map_epoch);
        }
        set_reference(channels_[n], std::move(layer));
        channels_[n].reference_windows = config_.reference_windows;
    }
    if (epochs.size() != 1) throw std::invalid_argument("saved references disagree on map_epoch");
    map_epoch_ = epochs.front();
    state_ = msg::MemoryState::tracking;
}

void RoomMemory::save(const msg::Time& stamp) {
    if (!config_.state_dir) return;
    const msg::Header header{config_.room_id, map_epoch_, 0, stamp};
    const std::vector<std::filesystem::path> all = paths();
    for (std::size_t n = 0; n < all.size(); ++n) {
        save_reference(all[n], channels_[n].reference, header, static_cast<float>(config_.voxel_size));
    }
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
    try {
        cloud = decode_pointcloud2(data);
    } catch (const std::exception&) {  // PointCloudError, or any other decode failure (e.g. bad_alloc)
        ++channel.received;
        ++channel.failed;
        return false;
    }
    add_points(sensor, cloud.positions);
    return true;
}

std::uint64_t RoomMemory::next_id() { return next_event_id_++; }

void RoomMemory::set_reference(Channel& channel, VoxelLayer reference) {
    channel.reference = std::move(reference);
    channel.reference_index.reset();
    if (&channel == &channels_[map_channel_]) coverage_ = coverage_of(channel.reference);
}

std::vector<msg::ChangeEvent> RoomMemory::close_window(const msg::Time& stamp) {
    std::vector<msg::ChangeEvent> events;
    bool dropped = false;
    const auto quota = static_cast<std::size_t>(config_.recent_windows);
    for (Channel& channel : channels_) {
        const std::size_t frames = std::exchange(channel.frames, 0);
        std::vector<float> points = std::move(channel.points);
        channel.points.clear();
        if (channel.failed && frames == 0) dropped = true;
        channel.failed = 0;
        if (state_ == msg::MemoryState::building) {
            // A sensor's reference freezes at its own quota while the others catch up, and
            // only windows with at least one occupied voxel count toward it.
            if (frames && channel.reference_windows < config_.reference_windows) {
                const OccupancyMap window = occupancy_map(points, change_);
                if (!window.occupied.empty()) {
                    const VoxelLayer observed = VoxelLayer::observe(window, change_, backend_);
                    channel.stats.add(observed.occupied, observed.known);
                    ++channel.reference_windows;
                    set_reference(channel, channel.stats.settled(config_.settle_rule()));
                }
            }
            continue;
        }
        const auto push_recent = [&](CellSummary summary) {
            channel.recent.push_back(std::move(summary));
            while (channel.recent.size() > quota) channel.recent.pop_front();
        };
        // A window without usable points still ages the recent layer, but sends nothing.
        CellSummary summary;
        OccupancyMap window;
        if (frames) {
            summary = summarize(points, change_);
            window = occupancy_from(summary, change_);
        }
        if (window.occupied.empty()) {
            push_recent(CellSummary{});
            continue;
        }
        push_recent(std::move(summary));
        OccupancyMap recent;
        if (channel.recent.size() == 1) {
            recent = window;
        } else {
            std::vector<const CellSummary*> parts;
            for (const CellSummary& part : channel.recent) parts.push_back(&part);
            recent = occupancy_from(merge(parts), change_);
        }
        // The rig-pose estimate uses this window alone, so a move shows in full at once.
        if (!channel.reference_index) channel.reference_index = std::make_shared<const Membership>(channel.reference.occupied);
        channel.residual = pose_offset(window.occupied, *channel.reference_index, config_.voxel_size);
        // Voxels the reference has not settled keep counting, so new coverage joins it, unless
        // the rig may have moved.
        if (channel.residual && *channel.residual <= config_.learn_max_offset_m) {
            const VoxelLayer observed = VoxelLayer::observe(window, change_, backend_);
            channel.stats.add(set_difference(observed.occupied, channel.reference.known),
                              set_difference(observed.known, channel.reference.known));
        }
        DiffOptions options;
        options.before_known = &channel.reference.known;
        options.scope = scope_;
        options.backend = backend_;
        const Report report = diff(channel.reference.as_occupancy(change_.voxel_size), recent, change_, options);
        for (const auto& [state, candidate] : channel.tracker.update(report.regions, stamp, [this] { return next_id(); })) {
            events.push_back(event(channel, state, candidate, stamp));
            log_event(events.back());
        }
    }
    if (dropped) ++dropped_windows_;
    last_window_dropped_ = dropped;
    if (state_ == msg::MemoryState::building &&
        std::all_of(channels_.begin(), channels_.end(),
                    [&](const Channel& c) { return c.reference_windows >= config_.reference_windows; })) {
        state_ = msg::MemoryState::tracking;
        // From here on only the voxels the reference has not settled keep their counts.
        for (Channel& channel : channels_) channel.stats.erase(channel.reference.known);
        ++map_epoch_;
        save(stamp);
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
    if (state_ == msg::MemoryState::building) return std::nullopt;
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
    for (Channel& channel : channels_) {
        VoxelLayer fresh = channel.stats.settled(config_.settle_rule());
        if (fresh.known.empty()) continue;
        channel.stats.erase(fresh.known);
        set_reference(channel, channel.reference.merge(fresh));
        changed = true;
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

std::vector<msg::Bytes> RoomMemory::sync(const msg::RequestSync& request, const msg::Time& stamp) {
    std::vector<msg::Bytes> answer;
    if (request.header.room_id != config_.room_id) return answer;
    const auto since_index = [&](const std::deque<Logged>& log) -> std::optional<std::size_t> {
        if (!request.since) return std::nullopt;
        for (std::size_t n = 0; n < log.size(); ++n) {
            if (log[n].seq == request.since->seq && log[n].map_epoch == request.since->epoch) return n;
        }
        return std::nullopt;
    };
    if (request.topic == msg::SyncTopic::map_bricks) {
        if (!map_started_) return answer;
        const auto at = since_index(map_log_);
        if (at && *at + 1 < map_log_.size()) {
            for (std::size_t n = *at + 1; n < map_log_.size(); ++n) answer.push_back(map_log_[n].bytes);
            return answer;
        }
        // A fresh snapshot of the last live state, under the latest live seq.
        msg::MapBricks snapshot = map_message(msg::MapKind::snapshot, stamp);
        snapshot.header.seq = map_seq_ - 1;
        snapshot.bricks = map_bricks_;
        answer.push_back(snapshot.to_cdr());
        return answer;
    }
    if (const auto at = since_index(event_log_)) {
        for (std::size_t n = *at + 1; n < event_log_.size(); ++n) answer.push_back(event_log_[n].bytes);
        return answer;
    }
    // Bounded fallback: open events whose latest message is older than the kept ones, then
    // every kept message.
    const std::uint64_t oldest = event_log_.empty() ? ~std::uint64_t{0} : event_log_.front().seq;
    std::vector<const Logged*> older;
    for (const auto& [id, entry] : open_latest_) {
        if (entry.seq < oldest) older.push_back(&entry);
    }
    std::sort(older.begin(), older.end(), [](const Logged* a, const Logged* b) { return a->seq < b->seq; });
    for (const Logged* entry : older) answer.push_back(entry->bytes);
    for (const Logged& entry : event_log_) answer.push_back(entry.bytes);
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
    e.mask = std::move(mask);
    return e;
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
    message.state = last_window_dropped_ || silent ? msg::MemoryState::degraded : state_;
    message.n_bricks = channels_[map_channel_].reference.brick_count();
    for (const Channel& channel : channels_) message.n_open_events += static_cast<std::int64_t>(channel.tracker.open().size());
    message.coverage_known_frac = coverage_;
    if (state_ != msg::MemoryState::building) {
        for (const Channel& channel : channels_) {
            if (channel.residual && (!message.pose_residual_m || *channel.residual > *message.pose_residual_m)) {
                message.pose_residual_m = channel.residual;
            }
        }
    }
    message.window_s = static_cast<float>(config_.window_s);
    message.mid360_hz = hz[0];
    message.jt128_hz = hz[1];
    message.dropped_windows = dropped_windows_;
    return message;
}

}  // namespace room_memory
