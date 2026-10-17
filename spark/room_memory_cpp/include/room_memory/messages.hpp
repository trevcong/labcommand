// CDR codecs for the labcommand_msgs room-memory messages, with no ROS dependency: the C++ port
// of room_memory/messages.py. The contract is docs/ROOM-MEMORY-PROTOCOL.md (Phase 0).
//
// Classic CDR (XCDR1): a 4-byte encapsulation header (00 01 little endian, 00 00 big endian),
// every primitive aligned to its own size counted from the byte after the header, strings as
// a uint32 length that includes the NUL, T[] as a uint32 count, nested messages without
// alignment of their own, at most 3 trailing padding bytes. Decoders read either byte
// order; encoders write little endian. Validation is symmetric (protocol D16): to_cdr refuses
// exactly what from_cdr rejects. Integer fields are held wider than the wire where the Python
// codec range-checks them, so the same values are representable and rejected. A missing
// float measurement is an empty optional here and NaN on the wire.
//
// Protocol v2 appends fields at the end of MemoryStatus, MapBricks, ChangeEvent, Epoch, Activity,
// RequestSync and RhythmMinute (protocol §1.4, §3.9, D45). RhythmMinute skips the common header:
// sensor (1 Mid-360, 2 JT128) is the field after room_id, then map_epoch, seq and stamp.
// The layout comes from the bytes after the last legacy field: at most 3 is a
// legacy message, padding to 8 then exactly the v2 fields (then at most 3) is v2, and anything else
// is invalid. publisher_generation (and the RequestSync v2 fields) are empty for the legacy layout;
// the encoders write it only when they hold one.
#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <vector>

namespace room_memory::msg {

inline constexpr int kBrickDim = 8;
inline constexpr std::size_t kMaskBytes = 64;
inline constexpr std::size_t kCoordinateBytes = 6;
inline constexpr std::size_t kBytesPerBrick = kCoordinateBytes + 2 * kMaskBytes;
inline constexpr std::size_t kMaxPayloadBytes = std::size_t{16} << 20;
inline constexpr std::size_t kMaxTrailingPadding = 3;
inline constexpr std::int64_t kHzUnavailable = 0xFFFF;
inline constexpr std::uint64_t kSinceNone = ~std::uint64_t{0};
inline constexpr std::uint64_t kEventsSeqNone = ~std::uint64_t{0};
// SyncEnd.bound when nothing was published on the topic in this generation.
inline constexpr std::uint64_t kSeqNone = ~std::uint64_t{0};
// RequestSync.expected_generation only; invalid in every Spark -> Mac message.
inline constexpr std::uint64_t kGenerationUnknown = 0;
inline constexpr std::uint64_t kUint64Max = ~std::uint64_t{0};
inline constexpr std::size_t kTokenBytes = 16;
inline constexpr std::int64_t kSensorMid360 = 1;
inline constexpr std::int64_t kSensorJT128 = 2;
inline constexpr std::int64_t kSensorsAll = kSensorMid360 | kSensorJT128;

// Base class for codec failures.
class CDRError : public std::invalid_argument {
public:
    using std::invalid_argument::invalid_argument;
};

// The buffer ends before a field does.
class Truncated : public CDRError {
public:
    using CDRError::CDRError;
};

// A value breaks the protocol contract.
class Invalid : public CDRError {
public:
    using CDRError::CDRError;
};

// Well-formed input this codec does not handle (encapsulation, brick_dim, LZFSE blocks).
class Unsupported : public CDRError {
public:
    using CDRError::CDRError;
};

enum class Compression : std::uint8_t { none = 0, zlib = 1, lzfse = 2 };
enum class MapKind : std::uint8_t { snapshot = 0, delta = 1 };
enum class EventKind : std::uint8_t { appeared = 0, disappeared = 1, moved = 2 };
enum class EventState : std::uint8_t { candidate = 0, confirmed = 1, accepted = 2, reverted = 3, expired = 4 };
enum class MemoryState : std::uint8_t { building = 0, tracking = 1, rig_moved = 2, degraded = 3, paused = 4 };
enum class SyncTopic : std::uint8_t { map_bricks = 0, events = 1 };

const char* name(MemoryState state);
const char* name(Compression compression);

// True when this build decodes compressed LZFSE blocks (Apple's Compression framework).
bool lzfse_blocks_supported();

// builtin_interfaces/Time: seconds and nanoseconds since the Unix epoch.
struct Time {
    std::int64_t sec = 0;
    std::int64_t nanosec = 0;
    friend bool operator==(const Time&, const Time&) = default;
    void validate(const std::string& what) const;
};

// The four common fields that start every labcommand_msgs message except RhythmMinute (D45).
struct Header {
    std::string room_id;
    std::uint64_t map_epoch = 0;
    std::uint64_t seq = 0;
    Time stamp;
    friend bool operator==(const Header&, const Header&) = default;
    void validate(const std::string& context) const;
};

// One 8 x 8 x 8 brick. Voxel (x, y, z) is bit x + 8y + 64z, LSB first.
struct Brick {
    std::array<std::int64_t, 3> coordinate{};
    std::array<std::uint8_t, kMaskBytes> occupied{};
    std::array<std::uint8_t, kMaskBytes> known{};
    friend bool operator==(const Brick&, const Brick&) = default;
};

using Bytes = std::vector<std::uint8_t>;
// RequestSync.request_token / SyncEnd.request_token: 16 random bytes, never all zero in a v2 request.
using Token = std::array<std::uint8_t, kTokenBytes>;
inline constexpr Token kZeroToken{};

// Uncompressed payload bytes: 134 per brick plus 4 per removed index.
std::uint64_t payload_size(std::uint64_t bricks, std::uint64_t removed);

// Snapshot or delta of a room's voxel memory (protocol 3.1).
struct MapBricks {
    Header header;
    Compression compression = Compression::zlib;
    MapKind kind = MapKind::snapshot;
    std::uint64_t base_epoch = 0;
    float voxel_size = 0.05f;
    std::vector<Brick> bricks;
    std::vector<std::int64_t> removed;
    std::optional<std::uint64_t> publisher_generation;  // empty: the legacy layout (saved reference files)
    friend bool operator==(const MapBricks&, const MapBricks&) = default;

    void validate(const std::string& context = "MapBricks") const;
    Bytes to_cdr() const;
    static MapBricks from_cdr(std::span<const std::uint8_t> data);
};

// One lifecycle transition of one change (protocol 3.2). Coordinates in meters.
struct ChangeEvent {
    Header header;
    std::uint64_t event_id = 0;
    EventKind kind = EventKind::appeared;
    EventState state = EventState::candidate;
    std::array<float, 3> centroid{};
    std::array<float, 3> aabb_min{};
    std::array<float, 3> aabb_max{};
    std::optional<std::array<float, 3>> from_centroid;
    std::int64_t voxel_count = 0;
    float volume_m3 = 0.0f;
    std::optional<float> evidence;
    std::optional<float> alpha;
    std::int64_t sensors = 0;
    Time first_seen;
    Time last_seen;
    std::optional<MapBricks> mask;
    std::optional<std::uint64_t> publisher_generation;  // empty: the legacy layout
    friend bool operator==(const ChangeEvent&, const ChangeEvent&) = default;

    void validate() const;
    Bytes to_cdr() const;
    static ChangeEvent from_cdr(std::span<const std::uint8_t> data);
};

// Room-memory health, 1 Hz (protocol 3.3). An empty optional is "not available".
struct MemoryStatus {
    Header header;
    MemoryState state = MemoryState::building;
    std::int64_t n_bricks = 0;
    std::int64_t n_open_events = 0;
    std::optional<float> coverage_known_frac;
    std::optional<float> pose_residual_m;
    std::optional<float> window_s;
    std::optional<std::int64_t> mid360_hz;
    std::optional<std::int64_t> jt128_hz;
    std::int64_t dropped_windows = 0;
    // seq of the newest live events message of this generation; empty before the first
    std::optional<std::uint64_t> events_seq;
    std::optional<std::uint64_t> publisher_generation;  // empty: the legacy layout
    friend bool operator==(const MemoryStatus&, const MemoryStatus&) = default;

    void validate() const;
    Bytes to_cdr() const;
    static MemoryStatus from_cdr(std::span<const std::uint8_t> data);
};

// Timeline tick, 0.2 Hz (protocol 3.4). Counts are cumulative since the room was created.
struct Epoch {
    Header header;
    std::int64_t n_appeared = 0;
    std::int64_t n_disappeared = 0;
    std::int64_t n_moved = 0;
    std::optional<float> coverage_known_frac;
    MemoryState state = MemoryState::building;
    std::optional<std::uint64_t> publisher_generation;  // empty: the legacy layout
    friend bool operator==(const Epoch&, const Epoch&) = default;

    void validate() const;
    Bytes to_cdr() const;
    static Epoch from_cdr(std::span<const std::uint8_t> data);
};

// Transient motion next to the rig, 1 Hz (protocol 3.7). Never a change: activity does not become a
// ChangeEvent. `active` when a tracking window since the previous message had activity; the counts,
// sensor mask and largest region describe the newest such window, the largest region in the frame
// of `largest_sensor`. `since_s` counts from that newest activity window, empty (NaN) before the first.
struct Activity {
    Header header;
    bool active = false;
    std::int64_t sensors = 0;
    std::int64_t largest_sensor = 0;
    std::int64_t n_regions = 0;
    std::int64_t voxel_count = 0;
    std::optional<std::array<float, 3>> centroid;
    std::optional<std::array<float, 3>> aabb_min;
    std::optional<std::array<float, 3>> aabb_max;
    std::optional<float> since_s;
    std::optional<std::uint64_t> publisher_generation;  // empty: the legacy layout
    friend bool operator==(const Activity&, const Activity&) = default;

    void validate() const;
    Bytes to_cdr() const;
    static Activity from_cdr(std::span<const std::uint8_t> data);
};

// One minute of per-lidar region occupancy on /room_memory/rhythm (protocol §3.9, D45).
// It skips the common header: sensor (1 Mid-360, 2 JT128) is the field after room_id.
// Region arrays are strictly increasing in signed (ix, iy, iz).
// publisher_generation is empty for the legacy layout; the .rhy file does not carry it.
struct RhythmMinute {
    std::string room_id;
    std::int64_t sensor = 0;
    std::uint64_t map_epoch = 0;
    std::uint64_t seq = 0;
    Time stamp;
    std::int64_t windows_closed = 0;
    std::int64_t windows_with_points = 0;
    std::int64_t windows_unavailable = 0;
    std::int64_t clipped = 0;
    std::int64_t regions_dropped = 0;
    std::int64_t pose_ok = 0;
    std::int64_t flags = 0;
    std::vector<std::int64_t> ix;
    std::vector<std::int64_t> iy;
    std::vector<std::int64_t> iz;
    std::vector<std::int64_t> hit_windows;
    std::vector<std::int64_t> mean_voxels;
    std::optional<std::uint64_t> publisher_generation;
    friend bool operator==(const RhythmMinute&, const RhythmMinute&) = default;

    void validate() const;
    Bytes to_cdr() const;
    static RhythmMinute from_cdr(std::span<const std::uint8_t> data);
};

// Mac -> Spark resync request (protocol 3.5). `since` is (epoch, seq) or empty.
// expected_generation and token are both empty (the legacy layout) or both set.
struct RequestSync {
    struct Since {
        std::uint64_t epoch = 0;
        std::uint64_t seq = 0;
        friend bool operator==(const Since&, const Since&) = default;
    };
    Header header;
    std::optional<Since> since;
    SyncTopic topic = SyncTopic::map_bricks;
    // kGenerationUnknown (0): the since belongs to no known generation.
    std::optional<std::uint64_t> expected_generation;
    std::optional<Token> token;  // never all zero
    friend bool operator==(const RequestSync&, const RequestSync&) = default;

    void validate() const;
    Bytes to_cdr() const;
    static RequestSync from_cdr(std::span<const std::uint8_t> data);
};

// An inclusive seq range, first <= last.
struct SeqRange {
    std::uint64_t first = 0;
    std::uint64_t last = 0;
    friend bool operator==(const SeqRange&, const SeqRange&) = default;
};

// End-of-answer manifest of one RequestSync (protocol 3.8): what the answer published on the
// topic's sync topic, and what the publisher can no longer serve. `bound` is empty for SEQ_NONE
// (nothing published on the topic in this generation).
struct SyncEnd {
    Header header;
    std::uint64_t publisher_generation = 0;
    Token token{};  // copied from the request; all zero for a legacy request
    SyncTopic topic = SyncTopic::map_bricks;
    std::optional<std::uint64_t> bound;
    std::vector<SeqRange> delivered;
    std::vector<SeqRange> unavailable;
    friend bool operator==(const SyncEnd&, const SyncEnd&) = default;

    void validate() const;
    Bytes to_cdr() const;
    static SyncEnd from_cdr(std::span<const std::uint8_t> data);
};

}  // namespace room_memory::msg
