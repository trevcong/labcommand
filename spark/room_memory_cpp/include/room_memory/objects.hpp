// Named-object registry (protocol §3.15, §3.16). A pure library: the reference edit and the
// durable writer are callbacks, because absorbed() and write_staged live on the wiring branch.
// Association and snapshot_seq are memory only. The file keeps objects, seeds, the id counter,
// the dedupe ring, at most one commit record, and receipts.
#pragma once

#include "room_memory/keys.hpp"
#include "room_memory/messages.hpp"

#include <array>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <vector>

namespace room_memory {

struct RegistryError : std::runtime_error {
    using std::runtime_error::runtime_error;
};

// Thrown before the new bytes are installed. The in-memory registry is unchanged.
struct StorageError : std::runtime_error {
    using std::runtime_error::runtime_error;
};

// true: file and directory are durable. false: the file was renamed into place and the directory
// was not synced. Throw StorageError before the rename to leave the previous file untouched.
using WriteStaged = std::function<bool(const std::filesystem::path&, std::span<const std::uint8_t>)>;
bool install_registry_file(const std::filesystem::path& path, std::span<const std::uint8_t> bytes);

inline constexpr std::uint64_t kRegistryDedupeTtlNs = 120'000'000'000ULL;
inline constexpr std::size_t kRegistryDedupeCap = 1024;
inline constexpr std::size_t kRegistrySeedCap = 64;
inline constexpr std::size_t kRegistryReceiptCap = 64;
inline constexpr std::size_t kRegistryLinkCap = 32;

struct RegistryConfig {
    double name_min_fraction = 0.9;
    double name_count_ratio = 1.25;
    std::int64_t tolerance = 0;
};

// SHA-256. request_digest covers the NameObject fields except action_id, little endian.
std::array<std::uint8_t, 32> sha256(std::span<const std::uint8_t> data);
std::array<std::uint8_t, 32> request_digest(const msg::NameObjectRequest& request);

// same_shape(region, published) and the operator still sees the published pose.
bool accept_matches(const Keys& region, const Keys& published, std::int64_t tolerance, double fraction, bool seen_now);

struct Anchor {
    std::uint64_t frame_epoch = 0;
    Keys keys;
    std::uint64_t set_by_generation = 0;
    std::uint64_t set_by_id = 0;
};

struct ObjectLinkRecord {
    std::uint64_t event_generation = 0;
    std::uint64_t event_id = 0;
    msg::EventKind kind = msg::EventKind::appeared;
    bool recovered = false;
};

struct Object {
    std::uint64_t id = 0;
    std::uint64_t revision = 0;
    std::string name;
    std::uint8_t sensor = 0;
    msg::Baseline baseline = msg::Baseline::anchored;
    std::optional<Anchor> anchor;
    std::vector<ObjectLinkRecord> links;
    msg::Association association = msg::Association::none;
    std::uint64_t association_generation = 0;
    std::uint64_t association_event_id = 0;
};

struct Seed {
    std::uint64_t event_generation = 0;
    std::uint64_t event_id = 0;
    std::uint64_t frame_epoch = 0;
    std::uint8_t sensor = 0;
    Keys keys;
};

struct DedupeEntry {
    std::string action_id;
    std::array<std::uint8_t, 32> digest{};
    msg::NameOutcome outcome = msg::NameOutcome::invalid;
    std::uint64_t object_id = 0;
    std::uint64_t revision = 0;
    std::uint64_t stamp_ns = 0;
};

struct Receipt {
    std::uint64_t event_generation = 0;
    std::uint64_t event_id = 0;
    msg::EventKind kind = msg::EventKind::appeared;
};

// The published change an ACCEPT would absorb into the reference. The registry stores it and
// calls the absorb callback; it does not edit the reference itself.
struct PublishedChange {
    std::uint8_t sensor = 0;
    std::uint64_t frame_epoch = 0;
    msg::EventKind kind = msg::EventKind::disappeared;
    Keys keys;
    Keys from_keys;
};

using Absorb = std::function<void(const PublishedChange&)>;
using Occupied = std::function<bool(Key)>;

struct Finalize {
    std::uint64_t event_generation = 0;
    std::uint64_t event_id = 0;
    msg::EventKind kind = msg::EventKind::appeared;
    std::uint64_t object_id = 0;  // 0: the accept named no object; the receipt is still kept
};

struct SpatialEffect {
    enum class Kind : std::uint8_t { none = 0, move = 1, gone = 2 };
    Kind kind = Kind::none;
    std::uint64_t object_id = 0;
    Keys keys;
    std::vector<std::uint64_t> detach_ids;
};

struct CommitRecord {
    std::uint64_t frame_epoch = 0;
    std::optional<PublishedChange> redo;
    SpatialEffect effect;
    std::optional<Finalize> finalize;
};

// One event in a link window. `keys` is the new pose (MOVED destination, or the DISAPPEARED pose).
// `from_keys` is the MOVED old pose. missed > 0 is not eligible.
struct LinkEvent {
    std::uint64_t event_generation = 0;
    std::uint64_t event_id = 0;
    msg::EventKind kind = msg::EventKind::disappeared;
    std::int64_t missed = 0;
    Keys keys;
    Keys from_keys;
};

// The on-disk registry. Load and the live relearn commit share commit_frame_change.
class Registry {
public:
    static Registry open(const std::filesystem::path& state_dir, const std::string& room_id, double voxel_size,
                         std::uint8_t sensors, std::uint64_t frame_epoch, WriteStaged write = {}, Absorb absorb = {},
                         Occupied occupied = {}, RegistryConfig config = {});

    // Retire this frame's anchors, drop its seeds, and drop a commit record's redo and spatial
    // effect. A finalize still set becomes a receipt and marks the link recovered. One write.
    void commit_frame_change(std::uint64_t frame_epoch);

    msg::NameObjectResponse name(const msg::NameObjectRequest& request, std::uint64_t now_ns);

    // FIFO of 64. A duplicate (generation, id) is ignored. Does not bump an object's revision.
    bool add_seed(std::uint64_t generation, std::uint64_t event_id, std::uint8_t sensor, Keys keys);

    // Record one ACCEPT. Refuses when a record is already outstanding. Does not apply the effect
    // and does not absorb. `store_redo` false keeps finalize with the redo already cleared.
    // Returns the writer's bool (true when the directory was synced).
    bool stage_pending_accept(const PublishedChange& published, std::optional<Finalize> finalize, bool store_redo = true);

    // Recompute PENDING / UNRESOLVED / NONE for this sensor. Memory only.
    void link_window(std::uint8_t sensor, const std::vector<LinkEvent>& events);

    // ANCHORED anchors below name_min_fraction become DETACHED. Refuses while a record is open.
    void settle();

    void set_writer(WriteStaged write) { write_ = std::move(write); }
    void set_occupied(Occupied occupied) { occupied_ = std::move(occupied); }

    msg::ObjectRegistry to_message(const msg::Header& header, std::uint64_t publisher_generation) const;

    const std::string& room_id() const { return room_id_; }
    double voxel_size() const { return voxel_size_; }
    std::uint8_t sensors() const { return sensors_; }
    std::uint64_t frame_epoch() const { return frame_epoch_; }
    std::uint64_t snapshot_seq() const { return snapshot_seq_; }
    std::uint64_t next_id() const { return next_id_; }
    bool storage_error() const { return storage_error_; }
    bool persist_blocked() const { return persist_blocked_; }
    const std::vector<Object>& objects() const { return objects_; }
    const std::vector<Seed>& seeds() const { return seeds_; }
    const std::vector<Receipt>& receipts() const { return receipts_; }
    const std::vector<DedupeEntry>& dedupe() const { return dedupe_; }
    const std::optional<CommitRecord>& record() const { return record_; }

private:
    bool commit_draft(Registry draft, bool required);
    std::vector<std::uint8_t> encode() const;
    static Registry decode(std::span<const std::uint8_t> bytes);

    bool apply_frame_change(std::uint64_t new_frame);
    void replay(const Absorb& absorb);
    void apply_stored_effect();
    void recompute_validity();
    bool add_receipt(const Finalize& finalize);
    void mark_links_recovered(const Finalize& finalize, bool bump);

    bool expired(const DedupeEntry& entry, std::uint64_t now_ns) const;
    void evict_dedupe(std::uint64_t now_ns);
    const DedupeEntry* find_dedupe(const std::string& action_id) const;
    std::size_t live_dedupe(std::uint64_t now_ns) const;
    Object* find_object(std::uint64_t id);
    const Object* find_object(std::uint64_t id) const;
    Seed* find_seed(std::uint64_t generation, std::uint64_t event_id);
    bool enough(const Keys& keys) const;
    bool key_occupied(Key key) const;
    bool overlaps_anchored(const Keys& keys) const;
    static bool text_ok(const std::string& text, std::size_t low, std::size_t high);
    bool translated_match(const Keys& destination, const Keys& from, const Keys& anchor) const;
    static bool within_ratio(std::size_t side, std::size_t anchor, double ratio);

    msg::NameObjectResponse respond(msg::NameOutcome outcome, std::uint64_t object_id, std::uint64_t revision,
                                    bool repeat_differs = false) const;
    msg::NameObjectResponse store_and_commit(Registry draft, const msg::NameObjectRequest& request,
                                             msg::NameObjectResponse response, std::uint64_t now_ns, bool bump_snapshot);

    std::filesystem::path path_;
    WriteStaged write_;
    Occupied occupied_;
    RegistryConfig config_{};
    std::string room_id_;
    double voxel_size_ = 0;
    std::uint8_t sensors_ = 0;
    std::uint64_t frame_epoch_ = 0;
    std::uint64_t next_id_ = 1;
    std::uint64_t snapshot_seq_ = 0;
    bool storage_error_ = false;
    bool persist_blocked_ = false;
    std::vector<Object> objects_;
    std::vector<Seed> seeds_;
    std::vector<DedupeEntry> dedupe_;
    std::vector<Receipt> receipts_;
    std::optional<CommitRecord> record_;
};

}  // namespace room_memory
