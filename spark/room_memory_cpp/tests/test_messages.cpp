// Codec tests against the Phase 0 hand goldens, ported from spark/room_memory/tests/test_messages.py.
#include <zlib.h>

#include <cmath>
#include <cstring>
#include <functional>
#include <map>

#include "harness.hpp"
#include "room_memory/messages.hpp"

using namespace room_memory::msg;

namespace {

const std::filesystem::path kHand = std::filesystem::path(ROOM_MEMORY_FIXTURES) / "hand";
const Time kT10{1760400010, 250000000};

Bytes hexfile(const std::string& name) {
    const std::string text = harness::read_text(kHand / name);
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

RequestSync request_map() { return RequestSync{head("lab", 43, 2), RequestSync::Since{43, 12}, SyncTopic::map_bricks}; }

RequestSync request_events_none() {
    return RequestSync{head("lab-a", kSinceNone, 0, Time{1760400000, 500000000}), std::nullopt, SyncTopic::events};
}

// Each golden: decode equals the value; `exact` means the encoder reproduces the bytes.
struct Golden {
    std::function<bool(const Bytes&)> decodes_to_value;
    std::function<Bytes()> encode;
    std::function<void(const Bytes&)> decode;
    bool exact;
};

template <typename T>
Golden golden(std::function<T()> build, bool exact) {
    return Golden{[build](const Bytes& data) { return T::from_cdr(data) == build(); },
                  [build] { return build().to_cdr(); }, [](const Bytes& data) { (void)T::from_cdr(data); }, exact};
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
    };
    return table;
}

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
    CHECK_EQ(files, std::size_t{12});
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

TEST(every_truncation_throws) {
    for (const auto& [name, g] : goldens()) {
        const Bytes data = hexfile(name);
        for (std::size_t length = 0; length < data.size(); ++length) {
            CHECK_THROWS(CDRError, g.decode(slice(data, 0, length)));
        }
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
    };
    for (const auto& change : changes) {
        MemoryStatus s = memory_status();
        change(s);
        CHECK_THROWS(Invalid, s.to_cdr());
    }
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

TEST(float_fields_hold_binary32) {
    MemoryStatus status = memory_status();
    status.coverage_known_frac = f32(0.1);
    CHECK(MemoryStatus::from_cdr(status.to_cdr()) == status);
}

TEST(room_id_must_be_utf8) {
    MapBricks b = delta();
    b.header.room_id = "lab\xff";
    CHECK_THROWS(Invalid, b.to_cdr());
    b.header.room_id = "lab-\xc3\xa9";
    CHECK(MapBricks::from_cdr(b.to_cdr()) == b);
}

TEST_MAIN
