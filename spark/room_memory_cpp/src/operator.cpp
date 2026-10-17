// Operator actions (protocol 3.6, D46): RoomMemory::operate, the held ACCEPTED, the window hooks
// for suppressions and confirmed candidates, and their durable writes. The Python twin is
// room_memory/operator.py; both write the same bytes.
//
// Every durable effect is written before it is committed in memory: a write that installs
// nothing (StorageError) changes nothing and answers STORAGE_FAILED. A write that installed the
// file but could not sync its directory is committed, because a restart reads the new file.
// Everything here runs on the node's single-threaded executor, like close_window and sync.
#include "room_memory/operator.hpp"

#include <fcntl.h>
#include <unistd.h>

#include <algorithm>
#include <charconv>
#include <fstream>
#include <iterator>
#include <sstream>

namespace room_memory {

namespace {

using msg::EventKind;
using msg::EventState;
using msg::OperatorActionKind;
using Outcome = msg::OperatorOutcome;

WriteFault& write_fault() {
    static WriteFault fault;
    return fault;
}

bool injected(WriteStage stage) {
    const WriteFault& fault = write_fault();
    return fault && fault(stage);
}

const char* kind_name(EventKind kind) {
    switch (kind) {
        case EventKind::appeared: return "appeared";
        case EventKind::disappeared: return "disappeared";
        case EventKind::moved: return "moved";
    }
    return "unknown";
}

// The candidate as its published geometry describes it: what an operator event carries.
Candidate shown(const Candidate& candidate) {
    Candidate out = candidate;
    out.region = candidate.published;
    out.keys = region_keys(candidate.published);
    return out;
}

// The reference with a published change absorbed: an appearance (or a move's new pose) becomes
// occupied, a disappearance (or the rest of a move's old pose) known free.
VoxelLayer absorbed(const VoxelLayer& reference, const Region& change) {
    Keys occupy, clear;
    if (change.kind == Region::Kind::disappeared) {
        clear = change.keys;
    } else {
        occupy = change.keys;
        if (change.kind == Region::Kind::moved) clear = set_difference(change.from_keys, change.keys);
    }
    VoxelLayer layer;
    layer.occupied = set_union(set_difference(reference.occupied, clear), occupy);
    layer.known = set_union(reference.known, set_union(occupy, clear));
    return layer;
}

// The bytes save_reference writes.
msg::Bytes reference_cdr(const VoxelLayer& layer, const msg::Header& header, float voxel_size) {
    msg::MapBricks message;
    message.header = header;
    message.compression = msg::Compression::zlib;
    message.kind = msg::MapKind::snapshot;
    message.voxel_size = voxel_size;
    message.bricks = layer.to_bricks();
    return message.to_cdr();
}

std::span<const std::uint8_t> bytes_of(const std::string& text) {
    return {reinterpret_cast<const std::uint8_t*>(text.data()), text.size()};
}

// How a region of this window relates to a suppression: it shares a voxel with the matching
// pose, and whether it also keeps that pose's shape. A move's suppression also takes its halves:
// an appeared region on its new pose, a disappeared one on its old pose.
struct Match {
    bool touches = false;
    bool same = false;
};

Match match(const Suppression& s, const Region& region, std::int64_t tolerance, double fraction) {
    const auto kind = [&]() -> std::optional<EventKind> {
        switch (region.kind) {
            case Region::Kind::appeared: return EventKind::appeared;
            case Region::Kind::disappeared: return EventKind::disappeared;
            case Region::Kind::moved: return EventKind::moved;
            default: return std::nullopt;
        }
    }();
    if (!kind) return {};
    const auto pose = [&](const Keys& keys, const Keys& suppressed) {
        return Match{intersects(keys, suppressed), same_shape(keys, suppressed, tolerance, fraction)};
    };
    if (*kind == s.kind && s.kind != EventKind::moved) return pose(region.keys, s.keys);
    if (*kind == s.kind) {
        const bool touches = intersects(region.keys, s.keys) || intersects(region.from_keys, s.from_keys);
        return Match{touches, touches && same_shape(region.keys, s.keys, tolerance, fraction) &&
                                  same_shape(region.from_keys, s.from_keys, tolerance, fraction)};
    }
    if (s.kind != EventKind::moved) return {};
    if (*kind == EventKind::appeared) return pose(region.keys, s.keys);
    if (*kind == EventKind::disappeared) return pose(region.keys, s.from_keys);
    return {};
}

Region region_of(const Suppression& s) {
    Region region;
    region.kind = s.kind == EventKind::appeared      ? Region::Kind::appeared
                  : s.kind == EventKind::disappeared ? Region::Kind::disappeared
                                                     : Region::Kind::moved;
    region.keys = s.keys;
    region.from_keys = s.from_keys;
    return region;
}

// The channel's last window, on its own, supports `region` with the freshness thresholds. The
// recent layer keeps a region matched for recent_windows windows after the object left or went
// out of view, so a match alone does not show it is there now.
bool seen_now(const Region& region, const CellSummary& last, const Config& change, const MemoryConfig& config,
              const RayBackend* backend) {
    const OccupancyMap window = occupancy_from(last, change);
    const Keys& removed = region.kind == Region::Kind::disappeared ? region.keys : region.from_keys;
    const Keys free = removed.empty() ? Keys{} : cast_rays(window, change, &removed, backend);
    return region_fresh(region, window.occupied, free, config.fresh_min_voxels, config.fresh_min_fraction);
}

}  // namespace

void set_write_fault(WriteFault fault) { write_fault() = std::move(fault); }

bool write_staged(const std::filesystem::path& path, std::span<const std::uint8_t> data) {
    const std::filesystem::path parent = path.has_parent_path() ? path.parent_path() : std::filesystem::path(".");
    std::error_code error;
    std::filesystem::create_directories(parent, error);
    if (error) throw StorageError("cannot create " + parent.string() + ": " + error.message());
    const std::string scratch = path.string() + ".tmp";
    const auto fail = [&](const std::string& what) {
        ::unlink(scratch.c_str());
        throw StorageError(what);
    };
    const int fd = injected(WriteStage::write) ? -1 : ::open(scratch.c_str(), O_WRONLY | O_CREAT | O_TRUNC, 0644);
    if (fd < 0) fail("cannot write " + scratch);
    std::size_t written = 0;
    while (written < data.size()) {
        const ssize_t n = ::write(fd, data.data() + written, data.size() - written);
        if (n < 0) {
            ::close(fd);
            fail("cannot write " + scratch);
        }
        written += static_cast<std::size_t>(n);
    }
    const bool synced = !injected(WriteStage::sync) && ::fsync(fd) == 0;
    ::close(fd);
    if (!synced) fail("cannot sync " + scratch);
    if (injected(WriteStage::rename) || ::rename(scratch.c_str(), path.c_str()) != 0) fail("cannot rename " + scratch);
    // Installed: what every reader, a restart included, sees from here on.
    if (injected(WriteStage::directory)) return false;
    const int directory = ::open(parent.c_str(), O_RDONLY);
    if (directory < 0) return false;
    const bool directory_synced = ::fsync(directory) == 0;
    ::close(directory);
    return directory_synced;
}

std::filesystem::path suppressions_path(const std::string& state_dir, const std::string& room_id,
                                        const std::string& sensor) {
    return room_dir(state_dir, room_id) / ("suppressed-" + sensor);
}

std::string format_suppressions(const std::vector<Suppression>& list) {
    std::string text = "room_memory suppressed 1\n";
    for (const Suppression& s : list) {
        text += std::to_string(s.event_id) + " " + kind_name(s.kind);
        for (const Keys* pose : {&s.keys, &s.from_keys}) {
            text += " " + std::to_string(pose->size());
            for (const Key key : *pose) text += " " + std::to_string(key);
        }
        text += "\n";
    }
    return text;
}

std::vector<Suppression> parse_suppressions(const std::string& text) {
    const auto malformed = [](const std::string& why) {
        return std::invalid_argument("the suppressions file is malformed: " + why);
    };
    std::vector<Suppression> list;
    std::istringstream in(text);
    std::string line;
    if (!std::getline(in, line) || line != "room_memory suppressed 1") throw malformed("bad header");
    if (text.back() != '\n') throw malformed("no final newline");
    while (std::getline(in, line)) {
        std::vector<std::string> words;
        std::istringstream split(line);
        for (std::string word; split >> word;) words.push_back(word);
        if (words.empty() || line != [&] {
                std::string joined;
                for (const std::string& word : words) joined += (joined.empty() ? "" : " ") + word;
                return joined;
            }()) {
            throw malformed("line '" + line + "' is not single-spaced");
        }
        std::size_t at = 0;
        const auto next = [&]() -> const std::string& {
            if (at >= words.size()) throw malformed("line '" + line + "' is short");
            return words[at++];
        };
        const auto number = [&](auto& value) {
            const std::string& word = next();
            const auto [end, ec] = std::from_chars(word.data(), word.data() + word.size(), value);
            if (ec != std::errc{} || end != word.data() + word.size() || std::to_string(value) != word) {
                throw malformed("'" + word + "' is not a number");
            }
        };
        Suppression s;
        number(s.event_id);
        const std::string& kind = next();
        if (kind == "appeared") {
            s.kind = EventKind::appeared;
        } else if (kind == "disappeared") {
            s.kind = EventKind::disappeared;
        } else if (kind == "moved") {
            s.kind = EventKind::moved;
        } else {
            throw malformed("kind '" + kind + "'");
        }
        for (Keys* pose : {&s.keys, &s.from_keys}) {
            std::size_t count = 0;
            number(count);
            if (count > words.size()) throw malformed("line '" + line + "' is short");
            for (std::size_t n = 0; n < count; ++n) {
                Key key = 0;
                number(key);
                if (!pose->empty() && key <= pose->back()) throw malformed("keys are not sorted and unique");
                pose->push_back(key);
            }
        }
        if (at != words.size()) throw malformed("line '" + line + "' is long");
        if (s.keys.empty() || (s.kind == EventKind::moved) == s.from_keys.empty()) {
            throw malformed("event " + std::to_string(s.event_id) + " has the wrong poses for " + kind);
        }
        if (!list.empty() && s.event_id <= list.back().event_id) throw malformed("event ids are not increasing");
        list.push_back(std::move(s));
    }
    return list;
}

// MARK: CandidateTracker

Candidate* CandidateTracker::find_open(std::uint64_t event_id) {
    for (Candidate& candidate : open_) {
        if (candidate.event_id == event_id) return &candidate;
    }
    return nullptr;
}

std::optional<Candidate> CandidateTracker::release(std::uint64_t event_id) {
    for (auto at = open_.begin(); at != open_.end(); ++at) {
        if (at->event_id != event_id) continue;
        Candidate candidate = std::move(*at);
        open_.erase(at);
        return candidate;
    }
    return std::nullopt;
}

// MARK: RoomMemory

std::size_t RoomMemory::index_of(const Channel& channel) const {
    return static_cast<std::size_t>(&channel - channels_.data());
}

const std::vector<Suppression>& RoomMemory::suppressions(const std::string& sensor) const {
    return suppressions_[index_of(channel(sensor))];
}

bool RoomMemory::supported_by(const Keys& evidence, const Keys& keys) const {
    const auto count = static_cast<std::int64_t>(count_present(evidence, keys));
    return count >= config_.fresh_min_voxels &&
           static_cast<double>(count) >= config_.fresh_min_fraction * static_cast<double>(keys.size());
}

// This window's own evidence shows the region back at the reference state, with the freshness
// thresholds: an appearance's voxels (a move's new pose) traversed by its rays, a disappearance's
// voxels (a move's old pose) occupied.
bool RoomMemory::agrees(const Region& region, const OccupancyMap& window) const {
    const auto free_over = [&](const Keys& keys) { return cast_rays(window, change_, &keys, backend_); };
    switch (region.kind) {
        case Region::Kind::appeared: return supported_by(free_over(region.keys), region.keys);
        case Region::Kind::disappeared: return supported_by(window.occupied, region.keys);
        case Region::Kind::moved:
            return supported_by(free_over(region.keys), region.keys) && supported_by(window.occupied, region.from_keys);
        default: return false;
    }
}

void RoomMemory::load_suppressions(bool references) {
    if (!config_.state_dir) return;
    for (std::size_t n = 0; n < channels_.size(); ++n) {
        const std::filesystem::path path = suppressions_path(*config_.state_dir, config_.room_id, channels_[n].sensor.name);
        if (!std::filesystem::exists(path)) continue;
        if (!references) {
            // A new reference starts without the old room's dismissals.
            std::filesystem::remove(path);
            continue;
        }
        std::ifstream in(path, std::ios::binary);
        if (!in) throw std::runtime_error("cannot read " + path.string());
        suppressions_[n] = parse_suppressions(std::string((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>()));
    }
}

void RoomMemory::persist_suppressions() {
    for (std::size_t n = 0; n < channels_.size(); ++n) {
        if (!suppressions_dirty_[n]) continue;
        if (!config_.state_dir) {
            suppressions_dirty_[n] = false;
            continue;
        }
        const std::string text = format_suppressions(suppressions_[n]);
        try {
            const bool durable =
                write_staged(suppressions_path(*config_.state_dir, config_.room_id, channels_[n].sensor.name), bytes_of(text));
            suppressions_dirty_[n] = false;
            if (durable) {
                storage_error_.reset();
            } else {
                storage_error_ = "suppressions of " + channels_[n].sensor.name + " written, directory not synced";
            }
        } catch (const StorageError& error) {
            // Retried every window until it lands.
            storage_error_ = error.what();
        }
    }
}

void RoomMemory::screen_suppressed(Channel& channel, std::vector<Region>& regions, std::vector<bool>& supported,
                                   const OccupancyMap& window) {
    const std::size_t index = index_of(channel);
    std::vector<Suppression>& list = suppressions_[index];
    if (list.empty()) return;
    std::vector<bool> seen(list.size(), false);
    std::vector<Region> kept;
    std::vector<bool> kept_supported;
    for (std::size_t r = 0; r < regions.size(); ++r) {
        bool drop = false;
        for (std::size_t n = 0; n < list.size(); ++n) {
            const Match m = match(list[n], regions[r], config_.tolerance, config_.stable_min_fraction);
            seen[n] = seen[n] || m.touches;
            drop = drop || (m.touches && m.same);
        }
        if (drop) continue;
        kept.push_back(std::move(regions[r]));
        kept_supported.push_back(supported.empty() || supported[r]);
    }
    regions = std::move(kept);
    supported = std::move(kept_supported);
    // A matching region keeps a suppression alive; a window that sees its voxels back at the
    // reference state counts toward lifting it; any other window leaves the count alone.
    std::vector<Suppression> still;
    for (std::size_t n = 0; n < list.size(); ++n) {
        Suppression& s = list[n];
        if (seen[n]) {
            s.cleared = 0;
        } else if (agrees(region_of(s), window) && ++s.cleared >= config_.suppress_clear_windows) {
            suppressions_dirty_[index] = true;
            continue;
        }
        still.push_back(std::move(s));
    }
    list = std::move(still);
}

void RoomMemory::restore_confirmed(Channel& channel, const OccupancyMap& window, const msg::Time& stamp,
                                   std::vector<msg::ChangeEvent>& events) {
    std::vector<std::uint64_t> back;
    for (const Candidate& candidate : channel.tracker.open()) {
        if (candidate.confirmed && candidate.missed >= config_.expire_windows && agrees(candidate.published, window)) {
            back.push_back(candidate.event_id);
        }
    }
    std::sort(back.begin(), back.end());
    for (const std::uint64_t id : back) {
        const Candidate candidate = *channel.tracker.release(id);
        events.push_back(event(channel, EventState::reverted, shown(candidate), stamp));
        log_event(events.back());
    }
}

RoomMemory::OperatorResult RoomMemory::operate(const msg::OperatorAction& action, const msg::Time& stamp) {
    OperatorResult result;
    if (action.header.room_id != config_.room_id) {
        result.outcome = Outcome::wrong_room;
        return result;
    }
    if (action.expected_generation != publisher_generation_) {
        result.outcome = Outcome::stale_generation;
        return result;
    }
    for (const StoredAction& stored : actions_) {
        if (stored.action.action_id != action.action_id) continue;
        result.outcome = stored.outcome;
        result.repeat = true;
        result.repeat_differs = !(stored.action == action);
        return result;
    }
    switch (action.action) {
        case OperatorActionKind::acknowledge:
        case OperatorActionKind::confirm:
        case OperatorActionKind::dismiss:
        case OperatorActionKind::accept: result.outcome = event_action(action, stamp, result.events); break;
        default: result.outcome = Outcome::unsupported; break;
    }
    // A failed write may succeed on a retry with the same id.
    if (result.outcome != Outcome::storage_failed) {
        actions_.push_back(StoredAction{action, result.outcome});
        if (actions_.size() > kStoredActions) actions_.pop_front();
    }
    return result;
}

Outcome RoomMemory::event_action(const msg::OperatorAction& action, const msg::Time& stamp,
                                 std::vector<msg::ChangeEvent>& events) {
    if (state_ != msg::MemoryState::tracking) return Outcome::not_tracking;
    const std::uint64_t id = *action.event_id;
    // A held accept is ACCEPTED here, though the wire still says CONFIRMED.
    for (const HeldAccept& held : held_) {
        if (held.candidate.event_id == id) {
            return action.action == OperatorActionKind::acknowledge ? Outcome::applied : Outcome::state_mismatch;
        }
    }
    Channel* channel = nullptr;
    Candidate* candidate = nullptr;
    for (Channel& c : channels_) {
        if ((candidate = c.tracker.find_open(id))) {
            channel = &c;
            break;
        }
    }
    if (!candidate) return Outcome::unknown_event;
    if (action.action == OperatorActionKind::acknowledge) return Outcome::applied;
    const EventState state = candidate->confirmed ? EventState::confirmed : EventState::candidate;
    if (*action.expected_state != state) return Outcome::state_mismatch;
    const std::size_t index = index_of(*channel);
    // What the operator saw, still supported: matched in the channel's last window, which had
    // points, and still the shape of its published transition.
    const bool fresh = candidate->missed == 0 && !channel->recent.empty() && !channel->recent.back().keys.empty() &&
                       same_shape(region_keys(candidate->region), region_keys(candidate->published), config_.tolerance,
                                  config_.stable_min_fraction);

    if (action.action == OperatorActionKind::confirm) {
        if (!fresh || !seen_now(candidate->region, channel->recent.back(), change_, config_, backend_)) {
            return Outcome::target_changed;
        }
        candidate->confirmed = true;
        candidate->published = candidate->region;
        events.push_back(event(*channel, EventState::confirmed, shown(*candidate), stamp));
        log_event(events.back());
        return Outcome::applied;
    }

    if (action.action == OperatorActionKind::dismiss) {
        Suppression added;
        added.event_id = id;
        added.kind = candidate->kind;
        added.keys = candidate->published.keys;
        if (candidate->kind == EventKind::moved) added.from_keys = candidate->published.from_keys;
        std::vector<Suppression> next = suppressions_[index];
        next.insert(std::upper_bound(next.begin(), next.end(), id,
                                     [](std::uint64_t value, const Suppression& s) { return value < s.event_id; }),
                    added);
        if (config_.state_dir) {
            const std::string text = format_suppressions(next);
            try {
                const bool durable =
                    write_staged(suppressions_path(*config_.state_dir, config_.room_id, channel->sensor.name), bytes_of(text));
                if (durable) {
                    storage_error_.reset();
                } else {
                    storage_error_ = "suppressions of " + channel->sensor.name + " written, directory not synced";
                }
            } catch (const StorageError& error) {
                storage_error_ = error.what();
                return Outcome::storage_failed;
            }
        }
        suppressions_[index] = std::move(next);
        suppressions_dirty_[index] = false;
        const Candidate released = *channel->tracker.release(id);
        events.push_back(event(*channel, EventState::reverted, shown(released), stamp));
        log_event(events.back());
        return Outcome::applied;
    }

    // ACCEPT: compare-and-set on the published map, and only one accept waits for it at a time.
    if (!map_started_ || action.header.map_epoch != map_epoch_sent_ || !held_.empty()) return Outcome::stale_epoch;
    if (!fresh || !seen_now(candidate->published, channel->recent.back(), change_, config_, backend_)) {
        return Outcome::target_changed;
    }
    if (!channel->residual || *channel->residual > config_.learn_max_offset_m) return Outcome::rig_unsettled;
    VoxelLayer layer = absorbed(channel->reference, candidate->published);
    if (config_.state_dir) {
        try {
            const msg::Bytes bytes = reference_cdr(layer, msg::Header{config_.room_id, map_epoch_, 0, stamp},
                                                   static_cast<float>(config_.voxel_size));
            const bool durable =
                write_staged(reference_path(*config_.state_dir, config_.room_id, channel->sensor.name), bytes);
            if (durable) {
                storage_error_.reset();
            } else {
                storage_error_ = "reference of " + channel->sensor.name + " written, directory not synced";
            }
        } catch (const std::exception& error) {  // StorageError, or a layer the codec cannot carry
            storage_error_ = error.what();
            return Outcome::storage_failed;
        }
    }
    Candidate released = *channel->tracker.release(id);
    channel->stats.erase(region_keys(released.published));
    set_reference(*channel, std::move(layer));
    held_.push_back(HeldAccept{index, std::move(released), std::nullopt});
    return Outcome::applied;
}

std::vector<msg::ChangeEvent> RoomMemory::accepted_after_map(const msg::Time& stamp) {
    std::vector<msg::ChangeEvent> events;
    std::vector<HeldAccept> still;
    for (HeldAccept& held : held_) {
        if (!held.epoch) {
            still.push_back(std::move(held));
            continue;
        }
        events.push_back(event(channels_[held.channel], EventState::accepted, shown(held.candidate), stamp));
        events.back().header.map_epoch = *held.epoch;
        log_event(events.back());
    }
    held_ = std::move(still);
    return events;
}

}  // namespace room_memory
