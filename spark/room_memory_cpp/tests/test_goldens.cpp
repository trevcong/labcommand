// Byte parity with the shared goldens: the Swift-made scene and Phase 0 records in
// spark/room_memory/tests/fixtures/swift, the change-detection fixtures, and the spec literal.
// CTest runs this binary at 1 thread, 3 threads and the maximum.
#include <set>
#include <sstream>

#include "harness.hpp"
#include "offline.hpp"
#include "room_memory/change_detection.hpp"
#include "room_memory/key_set.hpp"
#include "room_memory/pointcloud2.hpp"
#include "scenes.hpp"

using namespace room_memory;

namespace {

const std::filesystem::path kSwift = std::filesystem::path(ROOM_MEMORY_FIXTURES) / "swift";
const std::filesystem::path kChangeDetection = harness::env_or("ROOM_MEMORY_CD_FIXTURES", ROOM_MEMORY_CD_FIXTURES);
const std::filesystem::path kSnapshots = harness::env_or("ROOM_MEMORY_SNAPSHOTS", ROOM_MEMORY_SNAPSHOTS);
const std::filesystem::path kPhase0 = harness::env_or("ROOM_MEMORY_PHASE0_FIXTURES", ROOM_MEMORY_PHASE0_FIXTURES);

const char* kClearLine =
    "{\"schema\":\"labcommand.change-detection.v2\",\"evidence\":true,\"report_unobserved\":false,"
    "\"origin_bits\":[\"00000000\",\"00000000\",\"00000000\"],\"ray_stride\":1,\"config\":{\"voxel_size_bits\":"
    "\"3d4ccccd\",\"max_range_bits\":null,\"min_points\":1,\"tolerance\":0,\"min_voxels\":6,\"min_extent\":1,"
    "\"pair_moves\":true,\"max_move_bits\":\"3fc00000\",\"move_count_ratio_bits\":\"40000000\"},\"before\":{\"points\":9,"
    "\"accepted\":9,\"dropped\":0,\"voxels\":9},\"after\":{\"points\":1,\"accepted\":1,\"dropped\":0,\"voxels\":1},"
    "\"appeared_voxels\":0,\"disappeared_voxels\":8,\"unknown_voxels\":0,\"regions\":[{\"kind\":\"disappeared\","
    "\"voxel_count\":8,\"min_key\":\"4611776178381914112\",\"centroid_bits\":[\"3f99999a\",\"3ccccccd\",\"3ccccccd\"],"
    "\"bounds_min_bits\":[\"3f800000\",\"00000000\",\"00000000\"],\"bounds_max_bits\":[\"3fb33333\",\"3d4ccccd\","
    "\"3d4ccccd\"],\"volume_bits\":\"3a83126f\"}]}";

Config config_for(std::vector<std::string> flags) {
    flags.push_back("before.bin");
    flags.push_back("after.bin");
    const auto call = offline::parse_args(flags, false, false);
    if (!call) throw harness::Failure("golden flags do not parse");
    return call->config;
}

std::string record(const scenes::Cloud& before, const scenes::Cloud& after, const Config& config) {
    const Comparison c = compare(before, after, config);
    return to_json(c.report, c.before, c.after, config) + "\n";
}

std::vector<float> positions(const std::filesystem::path& path) {
    return decode_pointcloud2(harness::read_bytes(path)).positions;
}

void require_phase0() {
    if (std::filesystem::is_directory(kPhase0)) return;
    if (ROOM_MEMORY_REQUIRE_PHASE0) throw harness::Failure("Phase 0 fixtures missing at " + kPhase0.string());
    throw harness::Skip("Phase 0 fixtures not found at " + kPhase0.string());
}

void scene_goldens() {
    for (const scenes::GoldenCase& c : scenes::golden_cases()) {
        const scenes::Pair pair = c.build();
        const std::string text = record(pair.before, pair.after, config_for(c.flags));
        if (text != harness::read_text(kSwift / (c.name + ".json"))) throw harness::Failure("golden differs: " + c.name);
    }
}

void phase0_goldens() {
    require_phase0();
    for (const scenes::Phase0Case& c : scenes::phase0_cases()) {
        const std::string text =
            record(positions(kPhase0 / c.before), positions(kPhase0 / c.after), config_for({"--evidence", "--json"}));
        if (text != harness::read_text(kSwift / (c.name + ".json"))) throw harness::Failure("golden differs: " + c.name);
    }
}

void snapshot_goldens() {
    Config config;
    config.evidence_of_absence = true;
    for (const char* sensor : {"jt128", "mid360"}) {
        const std::string text = record(positions(kSnapshots / "a" / (std::string(sensor) + ".bin")),
                                        positions(kSnapshots / "b" / (std::string(sensor) + ".bin")), config);
        if (text != harness::read_text(kChangeDetection / (std::string(sensor) + ".json"))) {
            throw harness::Failure(std::string("snapshot golden differs: ") + sensor);
        }
    }
}

// Runs `body` with every membership structure, mark set and point aggregation forced onto
// its sparse (hashed) path.
template <typename Body>
void sparse(Body&& body) {
    const Tuning saved = tuning();
    tuning().dense_cell_limit = 0;
    tuning().direct_partition_bits = -1;
    try {
        body();
    } catch (...) {
        tuning() = saved;
        throw;
    }
    tuning() = saved;
}

}  // namespace

TEST(swift_scene_goldens) { scene_goldens(); }

TEST(phase0_goldens_bytes) { phase0_goldens(); }

TEST(snapshot_golden_bytes) { snapshot_goldens(); }

TEST(sparse_paths_keep_every_golden) {
    sparse([] {
        scene_goldens();
        snapshot_goldens();
        if (std::filesystem::is_directory(kPhase0)) phase0_goldens();
    });
}

TEST(every_stored_swift_golden_has_a_case) {
    std::set<std::string> stored, scenes_stored, cases, phase0;
    for (const auto& entry : std::filesystem::directory_iterator(kSwift)) {
        if (entry.path().extension() != ".json") continue;
        const std::string name = entry.path().stem().string();
        stored.insert(name);
        if (!name.starts_with("phase0-")) scenes_stored.insert(name);
    }
    for (const auto& c : scenes::golden_cases()) cases.insert(c.name);
    for (const auto& c : scenes::phase0_cases()) phase0.insert(c.name);
    CHECK(scenes_stored == cases);
    for (const std::string& name : phase0) CHECK(stored.count(name) == 1);
    CHECK_EQ(stored.size(), std::size_t{37});
}

TEST(clear_line_is_the_spec_literal) {
    Config config;
    config.evidence_of_absence = true;
    config.tolerance = 0;
    config.min_extent = 1;
    const Comparison c = compare(scenes::column({20, 21, 22, 23, 24, 25, 26, 27, 40}), scenes::column({40}), config);
    CHECK(to_json(c.report, c.before, c.after, config) == kClearLine);
    CHECK_EQ(c.report.regions[0].min_key, Key{4611776178381914112});
}

TEST(shuffled_snapshot_keeps_the_golden) {
    Config config;
    config.evidence_of_absence = true;
    const auto before = scenes::shuffled(positions(kSnapshots / "a" / "mid360.bin"), 7);
    const auto after = scenes::shuffled(positions(kSnapshots / "b" / "mid360.bin"), 11);
    CHECK(record(before, after, config) == harness::read_text(kChangeDetection / "mid360.json"));
}

TEST(cli_prints_the_snapshot_goldens) {
    for (const char* sensor : {"jt128", "mid360"}) {
        std::ostringstream out, err;
        const int code = offline::run({"--evidence", "--json", (kSnapshots / "a" / (std::string(sensor) + ".bin")).string(),
                                       (kSnapshots / "b" / (std::string(sensor) + ".bin")).string()},
                                      out, err);
        CHECK_EQ(code, 0);
        CHECK(out.str() == harness::read_text(kChangeDetection / (std::string(sensor) + ".json")));
    }
}

TEST_MAIN
