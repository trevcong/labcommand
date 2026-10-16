// Codec tests against the Phase 0 hand goldens, ported from spark/room_memory/tests/test_messages.py.
#include <zlib.h>
#if defined(ROOM_MEMORY_APPLE_LZFSE)
#include <compression.h>
#endif

#include <cmath>
#include <cstring>
#include <functional>
#include <map>
#include <set>
#include <type_traits>

#include "harness.hpp"
#include "room_memory/messages.hpp"

using namespace room_memory::msg;

namespace {

const std::filesystem::path kHand = std::filesystem::path(ROOM_MEMORY_FIXTURES) / "hand";
// The Swift suite's copy of the hand goldens (absent in the image).
const std::filesystem::path kSwiftHand = ROOM_MEMORY_SWIFT_HAND;
const Time kT10{1760400010, 250000000};
// The one publisher generation of every v2 golden (a start time in ms).
constexpr std::uint64_t kG = 1760400000000ULL;

Token token_of(const char* hex) {
    Token out{};
    for (std::size_t n = 0; n < kTokenBytes; ++n) {
        out[n] = static_cast<std::uint8_t>(std::stoul(std::string(hex + 2 * n, 2), nullptr, 16));
    }
    return out;
}

const Token kTokenA = token_of("5a1e7c3d9b024f618e33c4d5e6f70819");
const Token kTokenB = token_of("c10ffee012344567a9876543210fedcb");
const Token kTokenC = token_of("0e8a3b5c77214d90b4065913c2ae6f01");

// Like CHECK_THROWS, and the message contains `text` (the Python suite's match=).
#define CHECK_THROWS_MATCH(Type, text, expression)                                                         \
    do {                                                                                                   \
        bool thrown = false;                                                                               \
        try {                                                                                              \
            (void)(expression);                                                                            \
        } catch (const Type& error) {                                                                      \
            thrown = std::string(error.what()).find(text) != std::string::npos;                            \
        }                                                                                                  \
        if (!thrown) {                                                                                     \
            throw harness::Failure(harness::location(__FILE__, __LINE__) + ": no " #Type " matching '" text \
                                   "' from " #expression);                                                 \
        }                                                                                                  \
    } while (0)

Bytes hexfile(const std::string& name, const std::filesystem::path& dir = kHand) {
    const std::string text = harness::read_text(dir / name);
    std::string digits;
    std::istringstream lines(text);
    for (std::string line; std::getline(lines, line);) {
        line = line.substr(0, line.find('#'));
        for (const char c : line) {
            if (std::isxdigit(static_cast<unsigned char>(c))) digits.push_back(c);
        }
    }
    Bytes out;
    for (std::size_t n = 0; n + 1 < digits.size(); n += 2) {
        out.push_back(static_cast<std::uint8_t>(std::stoul(digits.substr(n, 2), nullptr, 16)));
    }
    return out;
}

Header head(const std::string& room, std::uint64_t epoch, std::uint64_t seq, Time stamp = kT10) {
    return Header{room, epoch, seq, stamp};
}

std::array<std::uint8_t, kMaskBytes> mask(std::uint8_t first, std::uint8_t last = 0) {
    std::array<std::uint8_t, kMaskBytes> m{};
    m[0] = first;
    m[63] = last;
    return m;
}

std::array<std::uint8_t, kMaskBytes> filled(std::uint8_t value) {
    std::array<std::uint8_t, kMaskBytes> m;
    m.fill(value);
    return m;
}

float f32(double value) { return static_cast<float>(value); }

const std::vector<Brick> kDeltaBricks = {Brick{{-1, 2, -3}, mask(0x01, 0x80), mask(0x03, 0x80)}};

MapBricks delta(Compression compression = Compression::none) {
    return MapBricks{head("lab", 43, 12), compression, MapKind::delta, 42, f32(0.05), kDeltaBricks, {0, 5}};
}

MapBricks snapshot_zlib() {
    std::vector<Brick> bricks = kDeltaBricks;
    bricks.push_back(Brick{{0, 0, 1}, filled(0), filled(0xFF)});
    return MapBricks{head("lab", 43, 13), Compression::zlib, MapKind::snapshot, 0, f32(0.05), bricks, {}};
}

MemoryStatus memory_status() {
    MemoryStatus s;
    s.header = head("lab-a", 42, 7, Time{1760400000, 500000000});
    s.state = MemoryState::tracking;
    s.n_bricks = 3200;
    s.n_open_events = 2;
    s.coverage_known_frac = f32(0.75);
    s.window_s = f32(1.0);
    s.mid360_hz = 10;
    s.dropped_windows = 3;
    s.events_seq = 12;
    return s;
}

Epoch epoch() {
    return Epoch{head("lab", 4294967298ULL, 1000, Time{1760400005, 0}), 5, 1, 16909060, std::nullopt, MemoryState::rig_moved};
}

ChangeEvent moved_event() {
    const MapBricks region{head("lab", 43, 0), Compression::none, MapKind::snapshot, 0, f32(0.05),
                           {Brick{{3, -1, 0}, mask(0x01), mask(0x01)}}, {}};
    ChangeEvent e;
    e.header = head("lab", 43, 3);
    e.event_id = 9;
    e.kind = EventKind::moved;
    e.state = EventState::confirmed;
    e.centroid = {f32(1.5), f32(-0.25), f32(0.125)};
    e.aabb_min = {f32(1.25), f32(-0.5), f32(0.0)};
    e.aabb_max = {f32(1.75), f32(0.0), f32(0.25)};
    e.from_centroid = std::array<float, 3>{f32(0.5), f32(-0.25), f32(0.125)};
    e.voxel_count = 125;
    e.volume_m3 = f32(0.015625);
    e.evidence = f32(0.875);
    e.sensors = kSensorMid360 | kSensorJT128;
    e.first_seen = Time{1760400000, 0};
    e.last_seen = kT10;
    e.mask = region;
    return e;
}

ChangeEvent appeared_event() {
    ChangeEvent e;
    e.header = head("lab", 43, 4);
    e.event_id = 10;
    e.kind = EventKind::appeared;
    e.state = EventState::candidate;
    e.centroid = {f32(2.0), f32(1.0), f32(0.5)};
    e.aabb_min = {f32(1.875), f32(0.875), f32(0.375)};
    e.aabb_max = {f32(2.125), f32(1.125), f32(0.625)};
    e.voxel_count = 8;
    e.volume_m3 = f32(0.001);
    e.alpha = f32(0.05);
    e.sensors = kSensorMid360;
    e.first_seen = Time{1760400010, 0};
    e.last_seen = kT10;
    return e;
}

Activity activity() {
    Activity a;
    a.header = head("lab", 43, 21);
    a.active = true;
    a.sensors = kSensorMid360 | kSensorJT128;
    a.largest_sensor = kSensorJT128;
    a.n_regions = 3;
    a.voxel_count = 16909060;
    a.centroid = std::array<float, 3>{f32(0.5), f32(1.25), f32(0.0625)};
    a.aabb_min = std::array<float, 3>{f32(0.25), f32(1.0), f32(0.0)};
    a.aabb_max = std::array<float, 3>{f32(0.75), f32(1.5), f32(0.125)};
    a.since_s = f32(2.5);
    return a;
}

Activity activity_idle() {
    Activity a;
    a.header = head("lab", 1, 0);
    return a;
}

RigPose rig_pose() {
    RigPose p;
    p.header = head("lab", 43, 22);
    p.outcome = RigOutcome::realigned;
    p.causes = kCauseImuShock | kCausePeriodic;
    p.sensors = kSensorMid360 | kSensorJT128;
    p.channel = {RigChannel::moved, RigChannel::in_place};
    p.frame_epoch = 40;
    p.n_realigned = 16909060;
    p.mid360_pose = std::array<float, 7>{f32(0.03125), f32(-0.0625), 0.0f, 0.5f, 0.5f, 0.5f, 0.5f};
    p.jt128_pose = std::array<float, 7>{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f};
    p.delta_m = {f32(0.03125), std::nullopt};
    p.delta_deg = {f32(0.5), std::nullopt};
    p.inlier_frac = {f32(0.75), f32(0.875)};
    p.rms_m = {f32(0.0078125), f32(0.00390625)};
    p.observe = {f32(0.25), f32(0.125)};
    p.tilt_deg = f32(0.25);
    p.imu_ok = true;
    p.moved_at = Time{1760400005, 500000000};
    return p;
}

ImuSample imu() { return ImuSample{kT10, "livox_frame", {0.015625, -0.03125, 0.5}, {0.0078125, -0.0, -1.0}}; }

RequestSync request_map() { return RequestSync{head("lab", 43, 2), RequestSync::Since{43, 12}, SyncTopic::map_bricks}; }

RequestSync request_events_none() {
    return RequestSync{head("lab-a", kSinceNone, 0, Time{1760400000, 500000000}), std::nullopt, SyncTopic::events};
}

Epoch epoch_v2() {
    Epoch e = epoch();
    e.publisher_generation = kG;
    return e;
}

Activity activity_v2() {
    Activity a = activity();
    a.publisher_generation = kG;
    return a;
}

MemoryStatus memory_status_v2() {
    MemoryStatus s = memory_status();
    s.publisher_generation = kG;
    return s;
}

MapBricks delta_v2() {
    MapBricks m = delta();
    m.publisher_generation = kG;
    return m;
}

ChangeEvent appeared_event_v2() {
    ChangeEvent e = appeared_event();
    e.publisher_generation = kG;
    return e;
}

ChangeEvent moved_event_v2() {
    ChangeEvent e = moved_event();
    e.mask->publisher_generation = kG;
    e.publisher_generation = kG;
    return e;
}

RequestSync request_events_v2() {
    RequestSync r = request_events_none();
    r.expected_generation = 0;
    r.token = kTokenA;
    return r;
}

RequestSync request_map_v2() {
    RequestSync r{head("lab", 43, 2), RequestSync::Since{43, 10}, SyncTopic::map_bricks};
    r.expected_generation = kG;
    r.token = kTokenC;
    return r;
}

SyncEnd sync_end_events_empty() {
    SyncEnd e;
    e.header = head("lab", 43, 0);
    e.publisher_generation = kG;
    e.token = kZeroToken;
    e.topic = SyncTopic::events;
    return e;
}

SyncEnd sync_end_events_fallback() {
    SyncEnd e;
    e.header = head("lab", 43, 1);
    e.publisher_generation = kG;
    e.token = kTokenB;
    e.topic = SyncTopic::events;
    e.bound = 120;
    e.delivered = {{10, 10}, {21, 120}};
    e.unavailable = {{11, 20}};
    return e;
}

SyncEnd sync_end_map_replay() {
    SyncEnd e;
    e.header = head("lab", 45, 2);
    e.publisher_generation = kG;
    e.token = kTokenC;
    e.topic = SyncTopic::map_bricks;
    e.bound = 13;
    e.delivered = {{11, 13}};
    return e;
}

// The v2 field a legacy cut lacks: publisher_generation, or RequestSync.expected_generation.
template <typename T>
std::optional<std::uint64_t> generation_of(const T& value) {
    if constexpr (std::is_same_v<T, RequestSync>) {
        return value.expected_generation;
    } else if constexpr (std::is_same_v<T, SyncEnd>) {
        return value.publisher_generation;
    } else {
        return value.publisher_generation;
    }
}

// Each golden: decode equals the value; `exact` means the encoder reproduces the bytes.
// `legacy_cut` decodes (throwing like decode) and says whether the result is a legacy message
// (no v2 fields) that differs from the golden's value.
struct Golden {
    std::function<bool(const Bytes&)> decodes_to_value;
    std::function<Bytes()> encode;
    std::function<void(const Bytes&)> decode;
    std::function<bool(const Bytes&)> legacy_cut;
    bool exact;
};

template <typename T>
Golden golden(std::function<T()> build, bool exact) {
    return Golden{[build](const Bytes& data) { return T::from_cdr(data) == build(); },
                  [build] { return build().to_cdr(); }, [](const Bytes& data) { (void)T::from_cdr(data); },
                  [build](const Bytes& data) {
                      const T value = T::from_cdr(data);
                      return !generation_of(value) && !(value == build());
                  },
                  exact};
}

const std::map<std::string, Golden>& goldens() {
    static const std::map<std::string, Golden> table = {
        {"memory_status_le.hex", golden<MemoryStatus>(memory_status, true)},
        {"epoch_le.hex", golden<Epoch>(epoch, true)},
        {"epoch_be.hex", golden<Epoch>(epoch, false)},
        {"map_bricks_delta_le.hex", golden<MapBricks>([] { return delta(); }, true)},
        {"map_bricks_delta_be.hex", golden<MapBricks>([] { return delta(); }, false)},
        {"map_bricks_delta_lzfse_le.hex", golden<MapBricks>([] { return delta(Compression::lzfse); }, false)},
        {"map_bricks_snapshot_zlib_le.hex", golden<MapBricks>(snapshot_zlib, false)},
        {"change_event_moved_mask_le.hex", golden<ChangeEvent>(moved_event, true)},
        {"change_event_appeared_le.hex", golden<ChangeEvent>(appeared_event, true)},
        {"request_sync_map_le.hex", golden<RequestSync>(request_map, true)},
        {"request_sync_map_be.hex", golden<RequestSync>(request_map, false)},
        {"request_sync_events_none_le.hex", golden<RequestSync>(request_events_none, true)},
        {"activity_le.hex", golden<Activity>(activity, true)},
        {"activity_idle_le.hex", golden<Activity>(activity_idle, true)},
        {"memory_status_v2_le.hex", golden<MemoryStatus>(memory_status_v2, true)},
        {"map_bricks_delta_v2_le.hex", golden<MapBricks>(delta_v2, true)},
        {"map_bricks_delta_v2_be.hex", golden<MapBricks>(delta_v2, false)},
        {"change_event_appeared_v2_le.hex", golden<ChangeEvent>(appeared_event_v2, true)},
        {"change_event_moved_mask_v2_le.hex", golden<ChangeEvent>(moved_event_v2, true)},
        {"request_sync_events_v2_le.hex", golden<RequestSync>(request_events_v2, true)},
        {"request_sync_map_v2_be.hex", golden<RequestSync>(request_map_v2, false)},
        {"sync_end_events_empty_le.hex", golden<SyncEnd>(sync_end_events_empty, true)},
        {"sync_end_events_fallback_le.hex", golden<SyncEnd>(sync_end_events_fallback, true)},
        {"sync_end_map_replay_be.hex", golden<SyncEnd>(sync_end_map_replay, false)},
        {"epoch_v2_le.hex", golden<Epoch>(epoch_v2, true)},
        {"epoch_v2_be.hex", golden<Epoch>(epoch_v2, false)},
        {"activity_v2_le.hex", golden<Activity>(activity_v2, true)},
        {"rig_pose_le.hex", golden<RigPose>(rig_pose, true)},
        {"imu_le.hex", Golden{[](const Bytes& data) { return decode_imu(data) == imu(); }, [] { return imu().to_cdr(); },
                              [](const Bytes& data) { (void)decode_imu(data); }, true}},
    };
    return table;
}

// v2 goldens whose cut at the legacy length (+ up to 3 bytes) is a valid legacy message.
bool extends_legacy(const std::string& name) { return name.find("_v2_") != std::string::npos; }

Bytes u64le(std::uint64_t value) {
    Bytes out(8);
    for (int b = 0; b < 8; ++b) out[static_cast<std::size_t>(b)] = static_cast<std::uint8_t>(value >> (8 * b));
    return out;
}

// Derived on rig-guard; the Swift suite decodes neither RigPose nor Imu, so it has no copy (PROVENANCE.md).
bool in_swift(const std::string& name) { return name != "rig_pose_le.hex" && name != "imu_le.hex"; }

Bytes inflate_all(const Bytes& data) {
    z_stream stream{};
    inflateInit2(&stream, -15);
    Bytes out(1 << 16);
    stream.next_in = const_cast<Bytef*>(data.data());
    stream.avail_in = static_cast<uInt>(data.size());
    stream.next_out = out.data();
    stream.avail_out = static_cast<uInt>(out.size());
    const int status = inflate(&stream, Z_FINISH);
    out.resize(stream.total_out);
    inflateEnd(&stream);
    if (status != Z_STREAM_END) throw harness::Failure("payload does not inflate");
    return out;
}

Bytes deflate_with(const Bytes& raw, int window_bits) {
    z_stream stream{};
    deflateInit2(&stream, 6, Z_DEFLATED, window_bits, 8, Z_DEFAULT_STRATEGY);
    Bytes out(deflateBound(&stream, static_cast<uLong>(raw.size())));
    stream.next_in = const_cast<Bytef*>(raw.data());
    stream.avail_in = static_cast<uInt>(raw.size());
    stream.next_out = out.data();
    stream.avail_out = static_cast<uInt>(out.size());
    deflate(&stream, Z_FINISH);
    out.resize(stream.total_out);
    deflateEnd(&stream);
    return out;
}

Bytes slice(const Bytes& data, std::size_t begin, std::size_t end) {
    return Bytes(data.begin() + static_cast<std::ptrdiff_t>(begin), data.begin() + static_cast<std::ptrdiff_t>(end));
}

Bytes join(Bytes a, const Bytes& b) {
    a.insert(a.end(), b.begin(), b.end());
    return a;
}

Bytes u32le(std::uint32_t value) {
    return {static_cast<std::uint8_t>(value), static_cast<std::uint8_t>(value >> 8), static_cast<std::uint8_t>(value >> 16),
            static_cast<std::uint8_t>(value >> 24)};
}

}  // namespace

TEST(every_hand_golden_is_covered) {
    std::size_t files = 0;
    for (const auto& entry : std::filesystem::directory_iterator(kHand)) {
        if (entry.path().extension() != ".hex") continue;
        ++files;
        CHECK(goldens().count(entry.path().filename().string()) == 1);
    }
    CHECK_EQ(files, goldens().size());
    CHECK_EQ(files, std::size_t{29});
}

TEST(hand_goldens_match_the_swift_copies) {
    if (!std::filesystem::is_directory(kSwiftHand)) throw harness::Skip("Swift hand goldens not found at " + kSwiftHand.string());
    std::set<std::string> copies, expected;
    for (const auto& entry : std::filesystem::directory_iterator(kSwiftHand)) {
        if (entry.path().extension() == ".hex") copies.insert(entry.path().filename().string());
    }
    for (const auto& [name, g] : goldens()) {
        if (in_swift(name)) expected.insert(name);
    }
    CHECK(copies == expected);
    for (const auto& [name, g] : goldens()) {
        if (in_swift(name) && hexfile(name, kSwiftHand) != hexfile(name)) {
            throw harness::Failure("Swift copy differs: " + name);
        }
    }
    CHECK(memory_status().to_cdr() == hexfile("memory_status_le.hex", kSwiftHand));
}

TEST(golden_decodes_to_value) {
    for (const auto& [name, g] : goldens()) {
        if (!g.decodes_to_value(hexfile(name))) throw harness::Failure("decode differs: " + name);
    }
}

TEST(encoder_reproduces_golden) {
    for (const auto& [name, g] : goldens()) {
        if (g.exact && g.encode() != hexfile(name)) throw harness::Failure("encode differs: " + name);
    }
}

TEST(zlib_golden_header_and_payload) {
    const Bytes golden = hexfile("map_bricks_snapshot_zlib_le.hex");
    const Bytes encoded = snapshot_zlib().to_cdr();
    CHECK(slice(encoded, 0, 68) == slice(golden, 0, 68));
    std::uint32_t length;
    std::memcpy(&length, encoded.data() + 68, 4);
    CHECK_EQ(encoded.size(), 72 + std::size_t{length});
    CHECK(inflate_all(slice(encoded, 72, encoded.size())) == slice(golden, 77, 345));
    CHECK(encoded[72] != 0x78);
    CHECK(MapBricks::from_cdr(encoded) == snapshot_zlib());
}

TEST(lzfse_compressed_block_and_encoder) {
    Bytes golden = hexfile("map_bricks_delta_lzfse_le.hex");
    std::memcpy(golden.data() + 72, "bvx2", 4);
    if (lzfse_blocks_supported()) {
        CHECK_THROWS(Invalid, MapBricks::from_cdr(golden));
    } else {
        CHECK_THROWS(Unsupported, MapBricks::from_cdr(golden));
    }
    CHECK_THROWS(Unsupported, delta(Compression::lzfse).to_cdr());
}

#if defined(ROOM_MEMORY_APPLE_LZFSE)
// Apple's decoder stops at the first end-of-stream block and ignores what follows it, so the
// codec must catch trailing bytes and a second stream itself.
TEST(compressed_lzfse_is_one_complete_stream) {
    const Bytes golden = hexfile("map_bricks_delta_lzfse_le.hex");
    const auto with_payload = [&](const Bytes& payload) {
        return join(join(slice(golden, 0, 68), u32le(static_cast<std::uint32_t>(payload.size()))), payload);
    };
    // The golden payload is one raw bvx- block: magic, byte count, bytes, then bvx$.
    std::uint32_t count;
    std::memcpy(&count, golden.data() + 76, 4);
    const Bytes raw = slice(golden, 80, 80 + count);
    Bytes stream(raw.size() + 4096);
    stream.resize(compression_encode_buffer(stream.data(), stream.size(), raw.data(), raw.size(), nullptr, COMPRESSION_LZFSE));
    CHECK(stream.size() > 8 && std::memcmp(stream.data(), "bvx-", 4) != 0);
    CHECK(MapBricks::from_cdr(with_payload(stream)) == delta(Compression::lzfse));
    for (const Bytes& tail : std::vector<Bytes>{{0}, {0, 1, 2}, {0, 1, 2, 3}, {'b', 'v', 'x', '$'}, stream}) {
        CHECK_THROWS(Invalid, MapBricks::from_cdr(with_payload(join(stream, tail))));
    }
    CHECK_THROWS(Invalid, MapBricks::from_cdr(with_payload(slice(stream, 0, stream.size() - 1))));
    // A raw block in front of the compressed one is still one stream; trailing bytes are not.
    const Bytes mixed = join(join(Bytes{'b', 'v', 'x', '-'}, u32le(0)), stream);
    CHECK(MapBricks::from_cdr(with_payload(mixed)) == delta(Compression::lzfse));
    CHECK_THROWS(Invalid, MapBricks::from_cdr(with_payload(join(mixed, {0}))));
}
#endif

TEST(every_truncation_throws) {
    for (const auto& [name, g] : goldens()) {
        const Bytes data = hexfile(name);
        std::size_t legacy_cuts = 0;
        for (std::size_t length = 0; length < data.size(); ++length) {
            bool decoded = false;
            try {
                // A v2 message cut where its legacy layout ends is that legacy message, never the golden.
                const bool legacy = g.legacy_cut(slice(data, 0, length));
                decoded = true;
                if (!extends_legacy(name) || !legacy) {
                    throw harness::Failure(name + " decodes cut at " + std::to_string(length));
                }
            } catch (const CDRError&) {
            }
            legacy_cuts += decoded;
        }
        CHECK(legacy_cuts <= 4);
    }
}

TEST(trailing_padding) {
    for (const auto& [name, g] : goldens()) {
        const Bytes data = hexfile(name);
        CHECK(g.decodes_to_value(join(data, Bytes(3, 0))));
        CHECK_THROWS(Invalid, g.decode(join(data, Bytes(4, 0))));
    }
}

TEST(encapsulation_and_strings) {
    Bytes data = hexfile("epoch_le.hex");
    data[1] = 0x07;
    CHECK_THROWS(Unsupported, Epoch::from_cdr(data));
    data = hexfile("epoch_le.hex");
    data[4 + 7] = 0x41;
    CHECK_THROWS(Invalid, Epoch::from_cdr(data));
}

TEST(activity_rejects_inconsistent_fields) {
    Bytes data = hexfile("activity_le.hex");
    data[4 + 32] = 2;  // active is a bool
    CHECK_THROWS(Invalid, Activity::from_cdr(data));
    data = hexfile("activity_le.hex");
    data[4 + 44 + 3] = 0x7f;  // centroid.x NaN, the rest finite
    data[4 + 44 + 2] = 0xc0;
    CHECK_THROWS(Invalid, Activity::from_cdr(data));
    data = hexfile("activity_le.hex");
    data[4 + 34] = 1;  // largest_sensor MID360 is set in sensors: still valid
    CHECK(Activity::from_cdr(data).largest_sensor == kSensorMid360);
    data[4 + 33] = 2;  // but not when sensors is JT128 only
    CHECK_THROWS(Invalid, Activity::from_cdr(data));
    data = hexfile("activity_idle_le.hex");
    data[4 + 36] = 1;  // an inactive message with a region
    CHECK_THROWS(Invalid, Activity::from_cdr(data));
    Activity idle = activity_idle();
    idle.since_s = -1.0f;
    CHECK_THROWS(Invalid, idle.to_cdr());
    Activity active = activity();
    active.since_s = std::nullopt;
    CHECK_THROWS(Invalid, active.to_cdr());
}

TEST(rig_pose_is_168_bytes_for_lab) { CHECK_EQ(rig_pose().to_cdr().size(), std::size_t{168}); }

TEST(rig_pose_rejects_bad_codes) {
    const std::pair<std::size_t, std::uint8_t> cases[] = {
        {32, 6},   // outcome past INCONCLUSIVE
        {33, 64},  // an undefined cause bit
        {34, 4},   // an undefined sensor bit
        {35, 5},   // channel past UNAVAILABLE
        {152, 2},  // imu_ok is a bool
    };
    for (const auto& [offset, value] : cases) {
        Bytes data = hexfile("rig_pose_le.hex");
        data[4 + offset] = value;
        CHECK_THROWS(Invalid, RigPose::from_cdr(data));
    }
}

TEST(rig_pose_rejects_inconsistent_fields) {
    Bytes data = hexfile("rig_pose_le.hex");
    data[4 + 34] = static_cast<std::uint8_t>(kSensorJT128);  // the Mid-360 is outside sensors but has a MOVED result
    CHECK_THROWS(Invalid, RigPose::from_cdr(data));
    data[4 + 35] = static_cast<std::uint8_t>(RigChannel::unavailable);  // UNAVAILABLE is allowed for it
    CHECK(RigPose::from_cdr(data).channel[0] == RigChannel::unavailable);
    const Bytes nan = {0x00, 0x00, 0xc0, 0x7f};
    data = hexfile("rig_pose_le.hex");
    std::copy(nan.begin(), nan.end(), data.begin() + 4 + 52);  // mid360 tx NaN, the rest finite
    CHECK_THROWS(Invalid, RigPose::from_cdr(data));
    for (std::size_t n = 0; n < 7; ++n) std::copy(nan.begin(), nan.end(), data.begin() + 4 + 52 + 4 * n);
    CHECK(!RigPose::from_cdr(data).mid360_pose);  // NaN x 7: not configured
    data = hexfile("rig_pose_le.hex");
    const Bytes inf = {0x00, 0x00, 0x80, 0x7f};
    std::copy(inf.begin(), inf.end(), data.begin() + 4 + 124);  // inlier_frac[0] infinite
    CHECK_THROWS(Invalid, RigPose::from_cdr(data));
    RigPose p = rig_pose();
    (*p.mid360_pose)[6] = 1.01f;
    CHECK_THROWS(Invalid, p.to_cdr());
    p = rig_pose();
    p.inlier_frac[0] = 1.5f;
    CHECK_THROWS(Invalid, p.to_cdr());
    p = rig_pose();
    p.tilt_deg = -1.0f;
    CHECK_THROWS(Invalid, p.to_cdr());
    p = rig_pose();
    p.delta_m[0] = INFINITY;
    CHECK_THROWS(Invalid, p.to_cdr());
}

TEST(decode_imu_rejects_non_finite_values) {
    Bytes data = hexfile("imu_le.hex");
    const Bytes nan = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf8, 0x7f};
    std::copy(nan.begin(), nan.end(), data.begin() + 4 + 128);
    CHECK_THROWS(Invalid, decode_imu(data));
}

TEST(payload_size_and_stream_checks) {
    Bytes raw = hexfile("map_bricks_delta_le.hex");
    raw[4 + 60] = 3;
    CHECK_THROWS(CDRError, MapBricks::from_cdr(raw));

    const Bytes golden = hexfile("map_bricks_snapshot_zlib_le.hex");
    const auto with_payload = [&](const Bytes& payload) {
        return join(join(slice(golden, 0, 68), u32le(static_cast<std::uint32_t>(payload.size()))), payload);
    };
    const Bytes raw_payload = slice(golden, 77, 345);
    CHECK_THROWS(Invalid, MapBricks::from_cdr(with_payload(deflate_with(raw_payload, 15))));
    const Bytes good = deflate_with(raw_payload, -15);
    CHECK(MapBricks::from_cdr(with_payload(good)) == snapshot_zlib());
    CHECK_THROWS(Invalid, MapBricks::from_cdr(with_payload(join(good, Bytes{0}))));
    CHECK_THROWS(Invalid, MapBricks::from_cdr(with_payload(deflate_with(slice(raw_payload, 0, raw_payload.size() - 1), -15))));
    CHECK_THROWS(Invalid, MapBricks::from_cdr(with_payload(Bytes{})));
}

TEST(declared_sizes_over_16_mib_are_refused_before_inflating) {
    Bytes data = hexfile("map_bricks_delta_le.hex");
    // n_bricks sits right before n_removed and the payload length; 130,000 bricks need 17.4 MB.
    const Bytes huge = u32le(130'000);
    std::memcpy(data.data() + 4 + 56, huge.data(), 4);
    CHECK_THROWS(Invalid, MapBricks::from_cdr(data));
}

TEST(empty_zlib_payload_is_two_bytes) {
    const MapBricks message{head("lab", 1, 0), Compression::zlib, MapKind::snapshot, 0, f32(0.05), {}, {}};
    const Bytes encoded = message.to_cdr();
    CHECK(encoded[encoded.size() - 2] == 0x03 && encoded.back() == 0x00);
    CHECK(MapBricks::from_cdr(encoded) == message);
}

TEST(corrupt_compressed_lzfse_empty_map_is_rejected) {
    const MapBricks none{head("lab", 1, 0), Compression::none, MapKind::snapshot, 0, f32(0.05), {}, {}};
    MapBricks lzfse_message = none;
    lzfse_message.compression = Compression::lzfse;
    const Bytes plain = none.to_cdr();
    MapBricks zlib_message = none;
    zlib_message.compression = Compression::zlib;
    const Bytes zipped = zlib_message.to_cdr();
    std::size_t at = 0;
    while (plain[at] == zipped[at]) ++at;  // the compression byte is the first difference
    Bytes encoded = slice(plain, 0, plain.size() - 4);
    encoded[at] = static_cast<std::uint8_t>(Compression::lzfse);
    const Bytes empty_stream = {'b', 'v', 'x', '$'};
    CHECK(MapBricks::from_cdr(join(join(encoded, u32le(4)), empty_stream)) == lzfse_message);
    const Bytes corrupt = {'b', 'v', 'x', '2'};
    if (lzfse_blocks_supported()) {
        CHECK_THROWS(Invalid, MapBricks::from_cdr(join(join(encoded, u32le(4)), corrupt)));
    } else {
        CHECK_THROWS(Unsupported, MapBricks::from_cdr(join(join(encoded, u32le(4)), corrupt)));
    }
}

TEST(map_bricks_rejections) {
    const std::vector<std::function<void(MapBricks&)>> changes = {
        [](MapBricks& b) { b.voxel_size = std::nanf(""); },
        [](MapBricks& b) { b.voxel_size = 0; },
        [](MapBricks& b) { b.base_epoch = 43; },
        [](MapBricks& b) { b.kind = MapKind::snapshot; },
        [](MapBricks& b) { b.removed = {5, 5}; },
        [](MapBricks& b) { b.bricks = {kDeltaBricks[0], kDeltaBricks[0]}; },
        [](MapBricks& b) { b.bricks = {Brick{{-1, 2, -3}, mask(0x01), filled(0)}}; },
        [](MapBricks& b) { b.bricks = {Brick{{-1, 2, -3}, filled(0), filled(0)}}; },
        [](MapBricks& b) { b.bricks = {Brick{{40000, 0, 0}, mask(1), mask(1)}}; },
        [](MapBricks& b) { b.header = head("", 43, 12); },
        [](MapBricks& b) { b.header = head(std::string("a\0b", 3), 43, 12); },
        [](MapBricks& b) { b.header = head("lab", 43, 12, Time{0, 1'000'000'000}); },
    };
    for (const auto& change : changes) {
        MapBricks b = delta();
        change(b);
        CHECK_THROWS(Invalid, b.to_cdr());
    }
}

TEST(change_event_rejections) {
    const std::vector<std::function<void(ChangeEvent&)>> changes = {
        [](ChangeEvent& e) { e.from_centroid.reset(); },
        [](ChangeEvent& e) { e.kind = EventKind::appeared; },
        [](ChangeEvent& e) { e.centroid = {std::nanf(""), 0, 0}; },
        [](ChangeEvent& e) { e.aabb_min = {2.0f, -0.5f, 0.0f}; },
        [](ChangeEvent& e) { e.volume_m3 = -1.0f; },
        [](ChangeEvent& e) { e.evidence = 1.5f; },
        [](ChangeEvent& e) { e.alpha = INFINITY; },
        [](ChangeEvent& e) { e.sensors = 4; },
        [](ChangeEvent& e) { e.first_seen = Time{1760400011, 0}; },
        [](ChangeEvent& e) { e.mask->header = head("lab", 43, 1); },
        [](ChangeEvent& e) { e.mask->header = head("lab", 44, 0); },
        [](ChangeEvent& e) { e.mask->header = head("other", 43, 0); },
        [](ChangeEvent& e) { e.mask->header = head("lab", 43, 0, Time{0, 0}); },
    };
    for (const auto& change : changes) {
        ChangeEvent e = moved_event();
        change(e);
        CHECK_THROWS(Invalid, e.to_cdr());
    }
}

TEST(change_event_partial_nan_from_centroid_is_invalid) {
    Bytes data = hexfile("change_event_appeared_le.hex");
    const float one = 1.0f;
    std::memcpy(data.data() + 4 + 80, &one, 4);
    CHECK_THROWS(Invalid, ChangeEvent::from_cdr(data));
}

TEST(memory_status_rejections) {
    const std::vector<std::function<void(MemoryStatus&)>> changes = {
        [](MemoryStatus& s) { s.coverage_known_frac = 1.25f; },
        [](MemoryStatus& s) { s.pose_residual_m = -0.1f; },
        [](MemoryStatus& s) { s.window_s = 0.0f; },
        [](MemoryStatus& s) { s.mid360_hz = kHzUnavailable; },
        [](MemoryStatus& s) { s.state = static_cast<MemoryState>(9); },
        [](MemoryStatus& s) { s.events_seq = kEventsSeqNone; },
    };
    for (const auto& change : changes) {
        MemoryStatus s = memory_status();
        change(s);
        CHECK_THROWS(Invalid, s.to_cdr());
    }
}

TEST(memory_status_events_seq_none) {
    Bytes data = hexfile("memory_status_le.hex");
    CHECK_EQ(data.size(), std::size_t{4 + 80});
    std::memset(data.data() + 4 + 72, 0xFF, 8);
    MemoryStatus expected = memory_status();
    expected.events_seq = std::nullopt;
    CHECK(MemoryStatus::from_cdr(data) == expected);
    CHECK(expected.to_cdr() == data);
}

TEST(request_sync_rules) {
    RequestSync wrong_epoch = request_map();
    wrong_epoch.header = head("lab", 42, 2);
    CHECK_THROWS(Invalid, wrong_epoch.to_cdr());
    RequestSync none_epoch = request_map();
    none_epoch.since = RequestSync::Since{kSinceNone, 12};
    CHECK_THROWS(Invalid, none_epoch.to_cdr());
    Bytes data = hexfile("request_sync_map_le.hex");
    std::memset(data.data() + 4 + 40, 0xFF, 8);
    CHECK_THROWS(Invalid, RequestSync::from_cdr(data));
    data = hexfile("request_sync_map_le.hex");
    data[4 + 48] = 2;
    CHECK_THROWS(Invalid, RequestSync::from_cdr(data));
}

// The node's RequestSync service re-serializes the request with rmw, which leaves alignment
// padding uninitialized (D39): pad bytes are never checked, so the value is unchanged.
TEST(request_sync_ignores_padding_bytes) {
    const RequestSync request = request_map_v2();
    Bytes data = request.to_cdr();
    // "lab" (8 bytes), five uint64 and the stamp end at 48; topic at 48, then 7 pad bytes.
    CHECK_EQ(data[4 + 48], std::uint8_t{0});
    std::memset(data.data() + 4 + 49, 0xA5, 7);
    CHECK(RequestSync::from_cdr(data) == request);
}

TEST(float_fields_hold_binary32) {
    MemoryStatus status = memory_status();
    status.coverage_known_frac = f32(0.1);
    CHECK(MemoryStatus::from_cdr(status.to_cdr()) == status);
}

// MARK: Protocol v2

TEST(layout_detection_from_trailing_bytes) {
    Bytes legacy = hexfile("memory_status_le.hex");
    CHECK(!MemoryStatus::from_cdr(legacy).publisher_generation);
    for (std::size_t extra = 4; extra < 8; ++extra) {
        CHECK_THROWS_MATCH(Invalid, "neither padding", MemoryStatus::from_cdr(join(legacy, Bytes(extra, 0))));
    }
    // Generation 0 rejects 8 appended zero bytes.
    CHECK_THROWS_MATCH(Invalid, "GENERATION_UNKNOWN", MemoryStatus::from_cdr(join(legacy, Bytes(8, 0))));
    CHECK(MemoryStatus::from_cdr(join(join(legacy, u64le(7)), Bytes(3, 0))).publisher_generation == std::uint64_t{7});
    CHECK_THROWS(Invalid, MemoryStatus::from_cdr(join(join(legacy, u64le(7)), Bytes(4, 0))));
    // MapBricks: the pad before the generation depends on the payload length.
    const Bytes v2 = hexfile("map_bricks_delta_v2_le.hex");
    legacy = hexfile("map_bricks_delta_le.hex");
    CHECK(slice(v2, 0, legacy.size()) == legacy && v2.size() == legacy.size() + 6 + 8);
    CHECK_THROWS(Invalid, MapBricks::from_cdr(join(join(legacy, Bytes(6, 0)), slice(u64le(kG), 0, 7))));
    // RequestSync: 24 v2 bytes after the pad.
    const Bytes request = hexfile("request_sync_events_none_le.hex");
    CHECK(!RequestSync::from_cdr(request).token);
    CHECK_THROWS(Invalid, RequestSync::from_cdr(join(request, Bytes(7 + 8, 0))));
    CHECK_THROWS_MATCH(Invalid, "all-zero", RequestSync::from_cdr(join(request, Bytes(7 + 24, 0))));
    // Epoch ends on a uint8 (7 pad bytes before the generation), Activity on a float32 (4).
    const Bytes epoch_legacy = hexfile("epoch_le.hex");
    CHECK(slice(hexfile("epoch_v2_le.hex"), 0, epoch_legacy.size()) == epoch_legacy);
    CHECK(join(join(epoch_legacy, Bytes(7, 0)), u64le(kG)) == hexfile("epoch_v2_le.hex"));
    CHECK_THROWS_MATCH(Invalid, "GENERATION_UNKNOWN", Epoch::from_cdr(join(epoch_legacy, Bytes(7 + 8, 0))));
    CHECK_THROWS_MATCH(Invalid, "neither padding", Epoch::from_cdr(join(join(epoch_legacy, Bytes(3, 0)), u64le(kG))));
    const Bytes activity_legacy = hexfile("activity_le.hex");
    CHECK(join(join(activity_legacy, Bytes(4, 0)), u64le(kG)) == hexfile("activity_v2_le.hex"));
    CHECK_THROWS_MATCH(Invalid, "GENERATION_UNKNOWN", Activity::from_cdr(join(activity_legacy, Bytes(4 + 8, 0))));
    CHECK_THROWS(Invalid, Activity::from_cdr(join(hexfile("activity_v2_le.hex"), Bytes(4, 0))));
}

TEST(v2_mask_follows_the_parent) {
    ChangeEvent event = moved_event_v2();
    event.mask->publisher_generation.reset();
    CHECK_THROWS_MATCH(Invalid, "mask publisher_generation", event.to_cdr());
    event = moved_event_v2();
    event.publisher_generation.reset();
    CHECK_THROWS_MATCH(Invalid, "mask publisher_generation", event.to_cdr());
    event = moved_event_v2();
    event.mask->publisher_generation = kG + 1;
    CHECK_THROWS_MATCH(Invalid, "mask publisher_generation", event.to_cdr());
    Bytes data = hexfile("change_event_moved_mask_v2_le.hex");
    const Bytes other = u64le(kG + 1);
    std::memcpy(data.data() + 4 + 344, other.data(), 8);
    CHECK_THROWS_MATCH(Invalid, "mask publisher_generation", ChangeEvent::from_cdr(data));
    // Only the parent's generation after a v2 mask is a bad length, not a legacy parent.
    data = hexfile("change_event_moved_mask_v2_le.hex");
    CHECK_THROWS_MATCH(Invalid, "neither padding", ChangeEvent::from_cdr(slice(data, 0, data.size() - 8)));
    CHECK(ChangeEvent::from_cdr(hexfile("change_event_moved_mask_le.hex")) == moved_event());
}

// Python's short-token and expected_generation=-1 cases have no C++ value (std::array, uint64).
TEST(request_sync_v2_rejections) {
    const std::vector<std::function<void(RequestSync&)>> changes = {
        [](RequestSync& r) { r.token.reset(); },
        [](RequestSync& r) { r.expected_generation.reset(); },
        [](RequestSync& r) { r.token = kZeroToken; },
    };
    for (const auto& change : changes) {
        RequestSync r = request_map_v2();
        change(r);
        CHECK_THROWS(Invalid, r.to_cdr());
    }
}

TEST(request_sync_v2_round_trip) {
    const RequestSync request = request_map_v2();
    CHECK(RequestSync::from_cdr(request.to_cdr()) == request);
    CHECK_EQ(request.to_cdr().size(), std::size_t{84});
    const Bytes golden = hexfile("request_sync_map_v2_be.hex");
    CHECK(slice(golden, 4 + 64, golden.size()) == Bytes(kTokenC.begin(), kTokenC.end()));  // uint8[16] is never reversed
}

TEST(sync_end_rejections) {
    const std::vector<std::function<void(SyncEnd&)>> changes = {
        [](SyncEnd& s) { s.publisher_generation = 0; },
        [](SyncEnd& s) { s.topic = static_cast<SyncTopic>(2); },
        [](SyncEnd& s) { s.bound = kSeqNone; },
        [](SyncEnd& s) { s.bound = 119; },
        [](SyncEnd& s) { s.bound.reset(); },
        [](SyncEnd& s) { s.delivered = {{21, 120}, {10, 10}}; },
        [](SyncEnd& s) {
            s.delivered = {{10, 20}, {21, 120}};
            s.unavailable = {};
        },
        [](SyncEnd& s) { s.delivered = {{10, 10}, {10, 120}}; },
        [](SyncEnd& s) { s.delivered = {{11, 10}}; },
        [](SyncEnd& s) { s.unavailable = {{11, 21}}; },
        [](SyncEnd& s) { s.topic = SyncTopic::map_bricks; },
    };
    for (const auto& change : changes) {
        SyncEnd s = sync_end_events_fallback();
        change(s);
        CHECK_THROWS(Invalid, s.to_cdr());
    }
}

TEST(sync_end_sizes_and_patches) {
    CHECK_EQ(sync_end_events_empty().to_cdr().size(), std::size_t{84});
    SyncEnd one = sync_end_events_empty();
    one.bound = 5;
    one.delivered = {{0, 5}};
    CHECK_EQ(one.to_cdr().size(), std::size_t{88 + 16});
    one.delivered.clear();
    one.unavailable = {{0, 5}};
    CHECK_EQ(one.to_cdr().size(), std::size_t{84 + 16});
    CHECK_EQ(sync_end_events_fallback().to_cdr().size(), std::size_t{92 + 32 + 16});
    Bytes data = hexfile("sync_end_events_fallback_le.hex");
    data[4 + 72] = 3;  // three delivered ranges: truncated
    CHECK_THROWS(CDRError, SyncEnd::from_cdr(data));
    data = hexfile("sync_end_events_fallback_le.hex");
    const Bytes hostile = u32le(0xFFFFFFFFu);  // a hostile count is refused before allocating
    std::memcpy(data.data() + 4 + 72, hostile.data(), 4);
    CHECK_THROWS(Truncated, SyncEnd::from_cdr(data));
    data = hexfile("sync_end_events_empty_le.hex");
    std::memset(data.data() + 4 + 32, 0, 8);
    CHECK_THROWS_MATCH(Invalid, "GENERATION_UNKNOWN", SyncEnd::from_cdr(data));
}

TEST(room_id_must_be_utf8) {
    MapBricks b = delta();
    b.header.room_id = "lab\xff";
    CHECK_THROWS(Invalid, b.to_cdr());
    b.header.room_id = "lab-\xc3\xa9";
    CHECK(MapBricks::from_cdr(b.to_cdr()) == b);
}

TEST_MAIN
