#include "room_memory/memory.hpp"

#include <fcntl.h>
#include <pwd.h>
#include <unistd.h>

#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iterator>
#include <stdexcept>

#include "room_memory/key_set.hpp"
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
        channels_.push_back(std::move(channel));
    }
    if (unknown) {
        throw std::invalid_argument("sensors must be a non-empty subset of ['jt128', 'mid360'], got " +
                                    sensor_list(config_.sensors));
    }
    if (config_.reference_windows < 1 || config_.recent_windows < 1) {
        throw std::invalid_argument("reference_windows and recent_windows must be >= 1");
    }
    if (!(config_.window_s > 0)) throw std::invalid_argument("window_s must be > 0 s");
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
        channels_[n].reference = std::move(layer);
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
    } catch (const PointCloudError&) {
        ++channel.received;
        ++channel.failed;
        return false;
    }
    add_points(sensor, cloud.positions);
    return true;
}

std::uint64_t RoomMemory::next_id() { return next_event_id_++; }

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
                    VoxelLayer observed = VoxelLayer::observe(window, change_, backend_);
                    channel.reference = channel.reference_windows == 0 ? std::move(observed)
                                                                       : channel.reference.merge(observed);
                    ++channel.reference_windows;
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
            recent = std::move(window);
        } else {
            std::vector<const CellSummary*> parts;
            for (const CellSummary& part : channel.recent) parts.push_back(&part);
            recent = occupancy_from(merge(parts), change_);
        }
        DiffOptions options;
        options.before_known = &channel.reference.known;
        options.scope = scope_;
        options.backend = backend_;
        const Report report = diff(channel.reference.as_occupancy(change_.voxel_size), recent, change_, options);
        for (const auto& [state, candidate] : channel.tracker.update(report.regions, stamp, [this] { return next_id(); })) {
            events.push_back(event(channel, state, candidate, stamp));
        }
    }
    if (dropped) ++dropped_windows_;
    last_window_dropped_ = dropped;
    if (state_ == msg::MemoryState::building &&
        std::all_of(channels_.begin(), channels_.end(),
                    [&](const Channel& c) { return c.reference_windows >= config_.reference_windows; })) {
        state_ = msg::MemoryState::tracking;
        ++map_epoch_;
        save(stamp);
    }
    return events;
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
    for (const Channel& channel : channels_) message.n_bricks += channel.reference.brick_count();
    for (const Channel& channel : channels_) message.n_open_events += static_cast<std::int64_t>(channel.tracker.open().size());
    message.window_s = static_cast<float>(config_.window_s);
    message.mid360_hz = hz[0];
    message.jt128_hz = hz[1];
    message.dropped_windows = dropped_windows_;
    return message;
}

}  // namespace room_memory
