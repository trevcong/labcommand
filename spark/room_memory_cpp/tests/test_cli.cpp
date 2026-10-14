// room_memory_offline: text block, flags, errors and decimal parsing, ported from
// spark/room_memory/tests/test_offline.py.
#include <cmath>
#include <sstream>

#include "harness.hpp"
#include "offline.hpp"

using namespace room_memory;

namespace {

const std::filesystem::path kChangeDetection = harness::env_or("ROOM_MEMORY_CD_FIXTURES", ROOM_MEMORY_CD_FIXTURES);

std::string fixture(const char* name) { return (kChangeDetection / name).string(); }

struct Run {
    int code;
    std::string out;
    std::string err;
};

Run run(const std::vector<std::string>& args) {
    std::ostringstream out, err;
    const int code = offline::run(args, out, err);
    return {code, out.str(), err.str()};
}

std::vector<std::string> lines(const std::string& text) {
    std::vector<std::string> out;
    std::istringstream in(text);
    for (std::string line; std::getline(in, line);) out.push_back(line);
    return out;
}

}  // namespace

TEST(text_block) {
    Run r = run({"--text", "--", fixture("a-mid360.bin"), fixture("b-mid360.bin")});
    CHECK_EQ(r.code, 0);
    std::vector<std::string> out = lines(r.out);
    CHECK_EQ(out[0], std::string("before: a-mid360.bin  points=20064  voxels=7592  dropped=0"));
    CHECK_EQ(out[2],
             std::string("voxel_size_m=0.050  tolerance=1  min_points=1  min_voxels=6  min_extent=2  pair_moves=true"));
    CHECK_EQ(out[3], std::string("appeared_voxels=255  disappeared_voxels=104  unknown_voxels=1286"));
    CHECK(out.size() == 6 && out[5] == "regions=0");
    r = run({"--text", "--min-extent", "1", "--", fixture("a-mid360.bin"), fixture("b-mid360.bin")});
    CHECK_EQ(r.code, 0);
    out = lines(r.out);
    CHECK_EQ(out[5], std::string("regions=1"));
    CHECK(out[6].starts_with("region 0 appeared voxels=7 volume_m3=0.000875 centroid=-0.618,1.618,0.975"));
}

TEST(usage_errors) {
    const std::vector<std::vector<std::string>> cases = {
        {}, {"a.bin"}, {"--bogus", "a.bin", "b.bin"}, {"--tolerance", "a.bin", "b.bin"},
        {"--voxel-size", "-1", "a.bin", "b.bin"}, {"--origin", "1,2", "a.bin", "b.bin"},
        {"--min-points", "x", "a.bin", "b.bin"},
    };
    for (const auto& args : cases) {
        const Run r = run(args);
        CHECK(r.code == 1 && r.out.empty() && r.err.starts_with("room_memory_offline"));
    }
}

TEST(input_and_config_errors) {
    const std::filesystem::path missing = std::filesystem::temp_directory_path() / "room-memory-missing";
    const std::string before = (missing / "a.pcd").string(), after = (missing / "b.pcd").string();
    Run r = run({before, after});
    CHECK(r.code == 1 && r.err.find("expected a .bin file") != std::string::npos);
    r = run({"--tolerance", "9", fixture("a-mid360.bin"), fixture("b-mid360.bin")});
    CHECK(r.code == 1 && r.err.find("tolerance") != std::string::npos);
}

TEST(opt_out_flags_select_the_swift_defaults) {
    Run r = run({"--no-evidence", fixture("a-mid360.bin"), fixture("b-mid360.bin")});
    CHECK(r.code == 0 && r.out.find("\"evidence\":false") != std::string::npos);
    r = run({"--no-evidence", "--text", fixture("a-mid360.bin"), fixture("b-mid360.bin")});
    CHECK(r.code == 0 && r.out.find("unknown_voxels") == std::string::npos);
    const auto call = offline::parse_args({fixture("a-mid360.bin"), fixture("b-mid360.bin")}, false, false);
    CHECK(call && !call->config.evidence_of_absence && !call->emit_json);
}

TEST(flags_map_to_config) {
    const auto call = offline::parse_args({"--voxel-size", "0.1", "--max-range", "15", "--min-points", "2", "--tolerance",
                                           "2", "--min-voxels", "3", "--min-extent", "4", "--max-move", "0.5", "--no-pair",
                                           "--evidence", "--origin", "-1,0.5,2", "--ray-stride", "8", "--unobserved",
                                           "--json", "x.bin", "y.bin"},
                                          false, false);
    CHECK(call && call->before == "x.bin" && call->after == "y.bin" && call->emit_json);
    const Config& c = call->config;
    CHECK(c.voxel_size == static_cast<float>(0.1) && c.max_range == 15.0f);
    CHECK(c.min_points == 2 && c.tolerance == 2 && c.min_voxels == 3 && c.min_extent == 4 && c.ray_stride == 8);
    CHECK(!c.pair_moves && c.evidence_of_absence && c.report_unobserved);
    CHECK((c.origin == Vec3{-1.0f, 0.5f, 2.0f}));
}

TEST(parse_f32_rounds_the_decimal_once) {
    CHECK_EQ(float_bits(*offline::parse_f32("1.000000178813934326171874")), 0x3F800001u);
    CHECK_EQ(float_bits(static_cast<float>(1.000000178813934326171874)), 0x3F800002u);
    CHECK_EQ(float_bits(*offline::parse_f32("0.05")), 0x3D4CCCCDu);
    CHECK(std::isinf(*offline::parse_f32("1e39")));
    CHECK(std::isnan(*offline::parse_f32(" nan ")));
    CHECK(!offline::parse_f32("0x1p3") && !offline::parse_f32("1_0") && !offline::parse_f32("") &&
          !offline::parse_f32("."));
    CHECK_EQ(*offline::parse_f32(" -.5e1 "), -5.0f);
}

TEST(integer_flags_follow_python_int) {
    const auto with = [](const char* text) {
        return offline::parse_args({"--min-voxels", text, "a.bin", "b.bin"}, false, false);
    };
    CHECK(with("1_000")->config.min_voxels == 1000);
    CHECK(with(" 7 ")->config.min_voxels == 7);
    CHECK(with("+3")->config.min_voxels == 3);
    CHECK(!with("1__0") && !with("_1") && !with("1.0") && !with("99999999999999999999"));
}

TEST_MAIN
