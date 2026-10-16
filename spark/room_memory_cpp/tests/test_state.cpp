// The rig guard's files: the registration cloud and rig_state codecs, their parse errors, atomic
// writes and archives, and byte identity with the Python engine (room_memory/state.py).
#include <zlib.h>

#include <cmath>
#include <cstdlib>
#include <cstring>
#include <limits>

#include "harness.hpp"
#include "room_memory/state.hpp"

using namespace room_memory;

namespace {

// Written by room_memory/state.py (Python 3.9, numpy) from the states below: kGoldenA from
// golden_a(), kGoldenB from golden_b(), kGoldenRegistration from
// registration_bytes("mid360", 2**64 - 1, np.float32 [[0.1, -2.5, 3e-5], [1e30, -0.0, 7.25]]).
const char* const kGoldenA =
    "6c63726d2d7269672d737461746520310a726f6f6d206c61622d610a766f78656c5f73697a6520302e30353030303030"
    "30303030303030303030330a6672616d655f65706f63682031383434363734343037333730393535313030300a6e5f72"
    "65616c69676e65642031320a675f6261736520302e3130303030303030303030303030303031202d30202d392e383036"
    "363439393939393939393939340a73656e736f72206d696433363020706f736520302e39393936353733323439373535"
    "35373236202d302e303236313736393438333037383733313533203020302e3032363137363934383330373837333135"
    "3320302e3939393635373332343937353535373236203020302030203120392e39393939393939393939393939393935"
    "652d3038202d3020342e39343036353634353834313234363534652d33323420626173656c696e6520302e3031323320"
    "726567697374726174696f6e20726567697374726174696f6e2d6d69643336302e62696e203132333435362064656164"
    "626565660a73656e736f72206c3220706f73652031203020302030203120302030203020312030203020302062617365"
    "6c696e65206e6f6e6520726567697374726174696f6e20726567697374726174696f6e2d6c322e62696e203430203030"
    "3030626565660a6372632032663066663936370a";
const char* const kGoldenB =
    "6c63726d2d7269672d737461746520310a726f6f6d206c61622d620a766f78656c5f73697a6520302e31303030303030"
    "303030303030303030310a6672616d655f65706f636820300a6e5f7265616c69676e656420300a675f62617365206e6f"
    "6e650a73656e736f72206d696433363020706f73652031203020302030203120302030203020312031652b323120322e"
    "35202d302e323520626173656c696e6520392e39393939393939393939393939393935652d3038207265676973747261"
    "74696f6e20726567697374726174696f6e2d6d69643336302e62696e2032382030303030303030300a63726320323563"
    "39613839360a";
const char* const kGoldenRegistration =
    "4c43524701000000060000006d6964333630ffffffffffffffff02000000cdcccc3d000020c082a8fb37caf249710000"
    "00800000e8407d2ca398";

std::vector<std::uint8_t> unhex(const char* text) {
    std::vector<std::uint8_t> out;
    for (std::size_t n = 0; text[n] && text[n + 1]; n += 2) {
        out.push_back(static_cast<std::uint8_t>(std::stoul(std::string(text + n, 2), nullptr, 16)));
    }
    return out;
}

std::vector<std::uint8_t> bytes(const std::string& text) { return std::vector<std::uint8_t>(text.begin(), text.end()); }
std::string text(const std::vector<std::uint8_t>& data) { return std::string(data.begin(), data.end()); }

std::uint32_t crc_of(const std::uint8_t* data, std::size_t size) {
    return static_cast<std::uint32_t>(::crc32(0L, data, static_cast<uInt>(size)));
}

// A registration file with its CRC rewritten after an edit.
std::vector<std::uint8_t> recrc(std::vector<std::uint8_t> data) {
    const std::uint32_t crc = crc_of(data.data(), data.size() - 4);
    for (int n = 0; n < 4; ++n) data[data.size() - 4 + static_cast<std::size_t>(n)] = static_cast<std::uint8_t>(crc >> (8 * n));
    return data;
}

// rig_state lines (each ending "\n") with their crc line.
std::vector<std::uint8_t> seal(const std::string& body) {
    char line[32];
    std::snprintf(line, sizeof line, "crc %08x\n", crc_of(reinterpret_cast<const std::uint8_t*>(body.data()), body.size()));
    return bytes(body + line);
}

// The body of a rig_state (everything before the crc line).
std::string body_of(const std::vector<std::uint8_t>& data) { return text(data).substr(0, data.size() - 13); }

std::string replace(std::string text, const std::string& from, const std::string& to) {
    const std::size_t at = text.find(from);
    if (at == std::string::npos) throw harness::Failure("no " + from + " in the text");
    return text.replace(at, from.size(), to);
}

// Runs `body`, which must throw std::invalid_argument "<what>: ..." containing `reason`.
template <typename F>
void expect_invalid(F&& body, const std::string& what, const std::string& reason) {
    try {
        body();
    } catch (const std::invalid_argument& error) {
        const std::string message = error.what();
        if (message.rfind(what + ": ", 0) != 0 || message.find(reason) == std::string::npos) {
            throw harness::Failure("expected \"" + what + ": ..." + reason + "...\", got \"" + message + "\"");
        }
        return;
    }
    throw harness::Failure("no std::invalid_argument for " + reason);
}

void expect_registration_error(const std::vector<std::uint8_t>& data, const std::string& reason) {
    expect_invalid([&] { parse_registration(data, "reg"); }, "reg", reason);
}

void expect_state_error(const std::vector<std::uint8_t>& data, const std::string& reason) {
    expect_invalid([&] { parse_rig_state(data, "rig_state"); }, "rig_state", reason);
}

bool same_bits(double a, double b) { return std::memcmp(&a, &b, sizeof a) == 0; }
bool same_bits(float a, float b) { return std::memcmp(&a, &b, sizeof a) == 0; }

bool same_pose(const rig::Pose& a, const rig::Pose& b) {
    for (std::size_t n = 0; n < 9; ++n) {
        if (!same_bits(a.R[n], b.R[n])) return false;
    }
    for (std::size_t n = 0; n < 3; ++n) {
        if (!same_bits(a.t[n], b.t[n])) return false;
    }
    return true;
}

bool same_optional(const std::optional<double>& a, const std::optional<double>& b) {
    return a.has_value() == b.has_value() && (!a || same_bits(*a, *b) || (std::isnan(*a) && std::isnan(*b)));
}

// Field by field, bit for bit (signed zeros included).
void check_same(const RigState& a, const RigState& b) {
    CHECK_EQ(a.room_id, b.room_id);
    CHECK(same_bits(a.voxel_size, b.voxel_size));
    CHECK_EQ(a.frame_epoch, b.frame_epoch);
    CHECK_EQ(a.n_realigned, b.n_realigned);
    CHECK_EQ(a.g_base.has_value(), b.g_base.has_value());
    if (a.g_base) {
        for (std::size_t n = 0; n < 3; ++n) CHECK(same_bits((*a.g_base)[n], (*b.g_base)[n]));
    }
    CHECK_EQ(a.sensors.size(), b.sensors.size());
    for (std::size_t n = 0; n < a.sensors.size(); ++n) {
        CHECK_EQ(a.sensors[n].first, b.sensors[n].first);
        CHECK(same_pose(a.sensors[n].second.pose, b.sensors[n].second.pose));
        CHECK(same_optional(a.sensors[n].second.baseline, b.sensors[n].second.baseline));
        CHECK(a.sensors[n].second.registration == b.sensors[n].second.registration);
    }
}

RigState golden_a() {
    RigState s;
    s.room_id = "lab-a";
    s.voxel_size = 0.05;
    s.frame_epoch = 18446744073709551000ULL;
    s.n_realigned = 12;
    s.g_base = rig::Vec3d{0.1, -0.0, -9.80665};
    rig::Pose mid360;
    // guard.rotation_about((0, 0, 1), 1.5) in Python (repr).
    mid360.R = {0.9996573249755573, -0.026176948307873153, 0.0, 0.026176948307873153, 0.9996573249755573, 0.0,
                0.0, 0.0, 1.0};
    mid360.t = {1e-7, -0.0, 5e-324};
    s.sensors.emplace_back("mid360", SensorState{mid360, 0.0123, {"registration-mid360.bin", 123456, 0xdeadbeef}});
    s.sensors.emplace_back("l2", SensorState{rig::Pose{}, std::nullopt, {"registration-l2.bin", 40, 0xbeef}});
    return s;
}

RigState golden_b() {
    RigState s;
    s.room_id = "lab-b";
    s.voxel_size = 0.1;
    rig::Pose mid360;
    mid360.t = {1e21, 2.5, -0.25};
    s.sensors.emplace_back("mid360", SensorState{mid360, 1e-7, {"registration-mid360.bin", 28, 0}});
    return s;
}

const std::vector<float> kGoldenPoints{0.1f, -2.5f, 3e-5f, 1e30f, -0.0f, 7.25f};

// A fresh directory under the system temp dir, removed when the test ends.
struct TempDir {
    std::filesystem::path path;
    TempDir() {
        std::string pattern = (std::filesystem::temp_directory_path() / "room-memory-state-XXXXXX").string();
        if (!mkdtemp(pattern.data())) throw harness::Failure("mkdtemp failed");
        path = pattern;
    }
    ~TempDir() { std::filesystem::remove_all(path); }
};

void touch(const std::filesystem::path& path, const std::string& content) {
    write_atomic(path, bytes(content));
}

}  // namespace

// MARK: Registration cloud

TEST(registration_round_trip) {
    CHECK_EQ(registration_name("mid360"), std::string("registration-mid360.bin"));
    const std::vector<float> points{1.0f, 2.0f, 3.0f, -4.5f, std::numeric_limits<float>::quiet_NaN(), 6.0f};
    const std::vector<std::uint8_t> data = registration_bytes("l2", 42, points);
    CHECK_EQ(data.size(), std::size_t{4 + 4 + 4 + 2 + 8 + 4 + 24 + 4});
    const Registration back = parse_registration(data, "reg");
    CHECK_EQ(back.sensor, std::string("l2"));
    CHECK_EQ(back.frame_epoch, std::uint64_t{42});
    CHECK_EQ(back.points.size(), points.size());
    for (std::size_t n = 0; n < points.size(); ++n) CHECK(same_bits(back.points[n], points[n]));
    const Registration empty = parse_registration(registration_bytes("", 0, {}), "reg");
    CHECK(empty.sensor.empty() && empty.points.empty());
    CHECK_THROWS(std::invalid_argument, registration_bytes("l2", 0, std::vector<float>{1.0f, 2.0f}));
}

TEST(registration_parse_errors) {
    const std::vector<std::uint8_t> good = registration_bytes("mid360", 7, kGoldenPoints);
    expect_registration_error(std::vector<std::uint8_t>(good.begin(), good.begin() + 27), "not a registration cloud");
    std::vector<std::uint8_t> data = good;
    data[0] = 'X';
    expect_registration_error(recrc(data), "not a registration cloud");
    data = good;
    data[30] ^= 1;
    expect_registration_error(data, "CRC mismatch");
    data = good;
    data[4] = 2;
    expect_registration_error(recrc(data), "unsupported version");
    data = good;
    data[8] = data[9] = data[10] = data[11] = 0xFF;  // a name past the end
    expect_registration_error(recrc(data), "truncated");
    data = good;
    data[8] = 7;  // a name one byte longer: the rest no longer adds up
    expect_registration_error(recrc(data), "point count does not match the size");
    data = good;
    data[12] = 0xFF;
    expect_registration_error(recrc(data), "sensor name is not UTF-8");
    data = good;
    data[12 + 6 + 8] += 1;  // one point more than the bytes hold
    expect_registration_error(recrc(data), "point count does not match the size");
    data = good;
    data.insert(data.end() - 4, {0, 0, 0, 0});
    expect_registration_error(recrc(data), "point count does not match the size");
}

TEST(registration_matches_python) {
    const std::vector<std::uint8_t> golden = unhex(kGoldenRegistration);
    CHECK(registration_bytes("mid360", UINT64_MAX, kGoldenPoints) == golden);
    const Registration back = parse_registration(golden, "reg");
    CHECK_EQ(back.sensor, std::string("mid360"));
    CHECK_EQ(back.frame_epoch, UINT64_MAX);
    CHECK_EQ(back.points.size(), kGoldenPoints.size());
    for (std::size_t n = 0; n < kGoldenPoints.size(); ++n) CHECK(same_bits(back.points[n], kGoldenPoints[n]));
}

// MARK: rig_state

TEST(rig_state_matches_python) {
    const std::vector<std::uint8_t> a = unhex(kGoldenA);
    const std::vector<std::uint8_t> b = unhex(kGoldenB);
    CHECK_EQ(text(format_rig_state(golden_a())), text(a));
    CHECK_EQ(text(format_rig_state(golden_b())), text(b));
    check_same(parse_rig_state(a, "a"), golden_a());
    check_same(parse_rig_state(b, "b"), golden_b());
}

TEST(rig_state_round_trip) {
    RigState s = golden_a();
    s.sensors[0].second.baseline = std::nullopt;
    s.sensors[1].second.baseline = 0.25;
    s.n_realigned = -3;
    s.voxel_size = std::numeric_limits<double>::infinity();  // not checked, as in Python
    check_same(parse_rig_state(format_rig_state(s), "s"), s);
    s.g_base.reset();
    s.sensors.clear();
    check_same(parse_rig_state(format_rig_state(s), "s"), s);
    // Every NaN is written "nan", as Python formats it.
    s = golden_b();
    s.sensors[0].second.baseline = -std::numeric_limits<double>::quiet_NaN();
    const std::string written = text(format_rig_state(s));
    CHECK(written.find(" baseline nan ") != std::string::npos);
    CHECK(std::isnan(*parse_rig_state(bytes(written), "s").sensors[0].second.baseline));
    s.room_id = "caf\xc3\xa9";
    CHECK_THROWS(std::invalid_argument, format_rig_state(s));
}

TEST(rig_state_lines_parse_as_python) {
    const std::string body = body_of(format_rig_state(golden_a()));
    // A repeated sensor keeps its place and takes the later line; a repeated field takes the later.
    const std::string later = "sensor mid360 pose 1 0 0 0 1 0 0 0 1 0 0 0 baseline none registration r.bin 1 00000001\n";
    RigState s = parse_rig_state(seal(body + later + "room other\n"), "s");
    CHECK_EQ(s.room_id, std::string("other"));
    CHECK_EQ(s.sensors.size(), std::size_t{2});
    CHECK_EQ(s.sensors[0].first, std::string("mid360"));
    CHECK_EQ(s.sensors[0].second.registration.name, std::string("r.bin"));
    CHECK_EQ(s.sensors[1].first, std::string("l2"));
    // Unknown fields are ignored; numbers Python's float() and int() accept.
    s = parse_rig_state(seal(replace(replace(body, "n_realigned 12", "n_realigned +12"), "0000beef", "BEEF") + "x y\n"),
                        "s");
    CHECK_EQ(s.n_realigned, std::int64_t{12});
    CHECK_EQ(s.sensors[1].second.registration.crc, std::uint32_t{0xbeef});
    s = parse_rig_state(seal(replace(body, "baseline 0.0123", "baseline -inf")), "s");
    CHECK(std::isinf(*s.sensors[0].second.baseline));
}

TEST(rig_state_parse_errors) {
    const std::vector<std::uint8_t> good = format_rig_state(golden_a());
    const std::string body = body_of(good);
    expect_state_error(seal(replace(body, "lab-a", "lab-\xc3\xa9")), "not ASCII");
    std::vector<std::uint8_t> data = good;
    data[20] ^= 1;
    expect_state_error(data, "CRC mismatch");
    expect_state_error(bytes(body), "CRC mismatch");
    std::string upper = text(good);
    upper = replace(upper, "crc 2f0ff967", "crc 2F0FF967");
    expect_state_error(bytes(upper), "CRC mismatch");
    expect_state_error(bytes(text(good) + "\n"), "CRC mismatch");
    expect_state_error(seal(""), "CRC mismatch");
    expect_state_error(seal("\n"), "not a rig state");
    expect_state_error(seal(replace(body, "lcrm-rig-state 1", "lcrm-rig-state 2")), "not a rig state");
    // Sensor lines are exactly 21 words.
    expect_state_error(seal(replace(body, " baseline none", "")), "malformed (rig_state: malformed sensor line 'sensor l2");
    expect_state_error(seal(replace(body, "sensor l2 pose", "sensor l2  pose")), "malformed sensor line");
    expect_state_error(seal(replace(body, "40 0000beef", "40 0000beef x")), "malformed sensor line");
    expect_state_error(seal(replace(body, "baseline none", "base none")), "malformed sensor line");
    expect_state_error(seal(replace(body, "registration registration-l2", "file registration-l2")), "malformed sensor line");
    // Values.
    expect_state_error(seal(replace(body, "pose 1 0 0", "pose 1 abc 0")), "malformed (could not convert string to float: 'abc')");
    expect_state_error(seal(replace(body, "pose 1 0 0", "pose 1 inf 0")), "malformed (");
    expect_state_error(seal(replace(body, "pose 1 0 0", "pose 0x1p3 0 0")), "malformed (");
    expect_state_error(seal(replace(body, "baseline 0.0123", "baseline x")), "malformed (");
    // Python's unbounded ints take the next four (no engine writes them, and the size, CRC and
    // epoch checks then refuse them); here they do not fit the fields.
    expect_state_error(seal(replace(body, "40 0000beef", "-40 0000beef")), "malformed (");
    expect_state_error(seal(replace(body, "40 0000beef", "40 100000000")), "malformed (");
    expect_state_error(seal(replace(body, "40 0000beef", "40 ghij")), "malformed (");
    expect_state_error(seal(replace(body, "frame_epoch 18446744073709551000", "frame_epoch 18446744073709551616")), "malformed (");
    expect_state_error(seal(replace(body, "frame_epoch 18446744073709551000", "frame_epoch -1")), "malformed (");
    expect_state_error(seal(replace(body, "n_realigned 12", "n_realigned 1.5")), "malformed (");
    expect_state_error(seal(replace(body, "voxel_size 0.050000000000000003", "voxel_size")), "malformed (list index out of range)");
    expect_state_error(seal(replace(body, "room lab-a\n", "")), "malformed ('room')");
    expect_state_error(seal(replace(body, "g_base 0.10000000000000001 -0 -9.8066499999999994\n", "")), "malformed ('g_base')");
    expect_state_error(seal(replace(body, "g_base 0.10000000000000001", "g_base x")), "malformed (");
    // g_base.
    expect_state_error(seal(replace(body, "g_base 0.10000000000000001 ", "g_base ")), "g_base is not three finite numbers");
    expect_state_error(seal(replace(body, "g_base 0.10000000000000001", "g_base nan")), "g_base is not three finite numbers");
    expect_state_error(seal(replace(body, "g_base 0.10000000000000001 -0 -9.8066499999999994", "g_base")),
                       "g_base is not three finite numbers");
}

// MARK: Files

TEST(write_atomic_creates_and_replaces) {
    TempDir dir;
    const std::filesystem::path path = dir.path / "lab" / "rig_state";
    write_atomic(path, bytes("one"));
    CHECK_EQ(harness::read_text(path), std::string("one"));
    write_atomic(path, bytes("two"));
    CHECK_EQ(harness::read_text(path), std::string("two"));
    CHECK(!std::filesystem::exists(dir.path / "lab" / "rig_state.tmp"));
    write_atomic(path, {});
    CHECK_EQ(std::filesystem::file_size(path), std::uintmax_t{0});
}

TEST(archive_numbers_removes_and_keeps_three) {
    TempDir dir;
    const std::filesystem::path room = dir.path / "lab";
    touch(room / "rig_state", "state 1");
    touch(room / "registration-mid360.bin", "cloud 1");
    const std::filesystem::path first =
        archive(room, "realign", {"rig_state", "registration-mid360.bin", "registration-l2.bin"}, {"registration-mid360.bin"});
    CHECK_EQ(first, room / "prev" / "0001-realign");
    CHECK_EQ(harness::read_text(first / "rig_state"), std::string("state 1"));
    CHECK_EQ(harness::read_text(first / "registration-mid360.bin"), std::string("cloud 1"));
    CHECK(!std::filesystem::exists(first / "registration-l2.bin"));
    CHECK(!std::filesystem::exists(room / "registration-mid360.bin"));
    CHECK(std::filesystem::exists(room / "rig_state"));
    // Names that are not archives are ignored and kept.
    std::filesystem::create_directories(room / "prev" / "notes");
    std::filesystem::create_directories(room / "prev" / "12-short");
    touch(room / "prev" / "0099-file", "not a directory");
    for (int n = 2; n <= 5; ++n) {
        touch(room / "rig_state", "state " + std::to_string(n));
        const std::filesystem::path target = archive(room, "reset", {"rig_state"}, {"missing"});
        char name[32];
        std::snprintf(name, sizeof name, "%04d-reset", n);
        CHECK_EQ(target, room / "prev" / name);
    }
    CHECK(!std::filesystem::exists(room / "prev" / "0001-realign"));
    CHECK(!std::filesystem::exists(room / "prev" / "0002-reset"));
    CHECK_EQ(harness::read_text(room / "prev" / "0003-reset" / "rig_state"), std::string("state 3"));
    CHECK_EQ(harness::read_text(room / "prev" / "0005-reset" / "rig_state"), std::string("state 5"));
    CHECK(std::filesystem::exists(room / "prev" / "notes") && std::filesystem::exists(room / "prev" / "12-short"));
    CHECK(std::filesystem::exists(room / "prev" / "0099-file"));
    // The sequence follows the highest archive, not the count.
    std::filesystem::create_directories(room / "prev" / "0041-old");
    CHECK_EQ(archive(room, "next", {}), room / "prev" / "0042-next");
    CHECK(!std::filesystem::exists(room / "prev" / "0003-reset") && !std::filesystem::exists(room / "prev" / "0004-reset"));
    CHECK(std::filesystem::exists(room / "prev" / "0005-reset") && std::filesystem::exists(room / "prev" / "0041-old"));
    // An existing target is an error (Python's mkdir).
    touch(room / "prev" / "0043-clash", "");
    CHECK_THROWS(std::exception, archive(room, "clash", {}));
}

TEST_MAIN
