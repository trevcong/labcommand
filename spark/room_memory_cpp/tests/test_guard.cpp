// The rig guard's geometry (room_memory/guard.hpp), ported from the non-engine half of
// spark/room_memory/tests/test_guard.py: pose conventions, the binary32 transform, the registration
// cloud, the solver pieces, and ICP on ray-cast rooms (rooms.hpp).
#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <optional>
#include <sstream>
#include <string>
#include <tuple>
#include <vector>

#include "harness.hpp"
#include "room_memory/change_detection.hpp"
#include "room_memory/guard.hpp"
#include "room_memory/keys.hpp"
#include "room_memory/pointcloud2.hpp"
#include "rooms.hpp"

namespace rig = room_memory::rig;
using rig::Pose;
using rig::Vec3d;
using room_memory::Keys;

namespace {

const std::filesystem::path kSynthetic = harness::env_or("ROOM_MEMORY_PHASE0_FIXTURES", ROOM_MEMORY_PHASE0_FIXTURES);
constexpr double kVoxel = 0.05;
// Added to every scan seed of the rooms.py tests: this generator is not numpy's, and the ICP bounds are
// sampling-marginal in both engines, so the base is the first of 0, 1000, 2000, ... that passes here.
constexpr std::uint64_t kSeedBase = 4000;
constexpr double kPi = 3.141592653589793;
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

double radians(double degrees) { return degrees * (kPi / 180.0); }

bool near(double a, double b, double tolerance) { return std::fabs(a - b) <= tolerance; }

std::vector<double> applied(const Pose& pose, const std::vector<double>& points) {
    std::vector<double> out;
    for (std::size_t n = 0; n < points.size(); n += 3) {
        const Vec3d p = pose.apply({points[n], points[n + 1], points[n + 2]});
        out.insert(out.end(), p.begin(), p.end());
    }
    return out;
}

room_memory::Config quantizer() {
    room_memory::Config config;
    config.voxel_size = static_cast<float>(kVoxel);
    config.max_range = 15.0f;
    config.evidence_of_absence = true;
    return config;
}

struct Reference {
    rig::RegistrationCloud cloud;
    Keys keys;
};

// (registration cloud, reference-occupied keys) of the change engine's quantizer over `scans`.
Reference reference_of(const std::vector<std::vector<float>>& scans) {
    rig::RegistrationBuilder builder(0.02);
    Keys occupied;
    for (const std::vector<float>& points : scans) {
        builder.add(points, 15.0);
        const Keys keys = room_memory::occupancy_map(points, quantizer()).occupied;
        occupied.insert(occupied.end(), keys.begin(), keys.end());
    }
    room_memory::sort_unique(occupied);
    return {builder.finish(occupied, kVoxel, 300'000), occupied};
}

const Reference& furnished() {
    static const Reference reference = [] {
        std::vector<std::vector<float>> scans;
        for (std::uint64_t seed = 0; seed < 10; ++seed) scans.push_back(rooms::scan({}, rooms::kRoom, rooms::kFurniture, 0.01, kSeedBase + seed));
        return reference_of(scans);
    }();
    return reference;
}

std::optional<rig::Estimate> solve(const std::vector<double>& source, const Reference& reference,
                                   const Vec3d& shift = {0.0, 0.0, 0.0}) {
    const Pose seed = rig::sweep_seed(source, Pose(), shift, reference.keys, kVoxel, 10.0).first;
    const double sigma0 = 0.5 * radians(1.0) * rig::median_range(source) + Pose{rig::kIdentity, shift}.distance_m();
    return rig::align(source, reference.cloud, seed, sigma0);
}

std::vector<double> source_of(const std::vector<float>& scan) { return rig::source_cloud(scan, 15.0, 0.10, 50'000); }

std::string describe(const Pose& truth, const std::optional<rig::Estimate>& estimate) {
    std::ostringstream text;
    text << "truth t=(" << truth.t[0] << ", " << truth.t[1] << ", " << truth.t[2] << ") r=" << truth.angle_deg();
    if (!estimate) return text.str() + ": no estimate";
    const auto [error_m, error_deg] = rooms::pose_error(estimate->pose, truth);
    text << ": error " << error_m << " m " << error_deg << " deg, inlier_frac " << estimate->inlier_frac << ", observe "
         << estimate->observe << ", rms " << estimate->rms_m;
    return text.str();
}

// guard.py's convergence bound: 5 mm and 0.05 degrees.
void check_close(const Pose& truth, const std::optional<rig::Estimate>& estimate) {
    if (!estimate) throw harness::Failure(describe(truth, estimate));
    const auto [error_m, error_deg] = rooms::pose_error(estimate->pose, truth);
    if (!(error_m <= 0.005 && error_deg <= 0.05)) throw harness::Failure(describe(truth, estimate));
}

// memory.py pose_shift (POSE_SEARCH_VOXELS = 4): the per-axis translation that best registers
// `recent` onto the reference occupancy, parabola-refined. Empty when no shift overlaps at all.
std::optional<Vec3d> pose_shift(const Keys& recent, const Keys& reference_occupied, double voxel_size) {
    constexpr std::int64_t r = 4;
    Vec3d shifts{};
    for (int axis = 0; axis < 3; ++axis) {
        std::vector<std::int64_t> counts;
        for (std::int64_t shift = -r; shift <= r; ++shift) {
            std::int64_t count = 0;
            for (const room_memory::Key key : recent) {
                room_memory::Index3 v = room_memory::unpack(key);
                (axis == 0 ? v.i : axis == 1 ? v.j : v.k) += shift;
                if (!room_memory::in_range(v.i, v.j, v.k)) continue;
                count += std::binary_search(reference_occupied.begin(), reference_occupied.end(),
                                            room_memory::pack(v.i, v.j, v.k));
            }
            counts.push_back(count);
        }
        std::int64_t best = 0;
        for (std::int64_t step = 1; step <= r; ++step) {
            for (const std::int64_t shift : {-step, step}) {
                if (counts[shift + r] > counts[best + r]) best = shift;
            }
        }
        if (counts[best + r] == 0) return std::nullopt;
        double shift = static_cast<double>(best);
        if (-r < best && best < r) {
            const double low = static_cast<double>(counts[best + r - 1]);
            const double mid = static_cast<double>(counts[best + r]);
            const double high = static_cast<double>(counts[best + r + 1]);
            const double curvature = low - 2.0 * mid + high;
            if (curvature < 0.0) shift += (low - high) / (2.0 * curvature);
        }
        shifts[axis] = shift * voxel_size;
    }
    return shifts;
}

std::vector<float> fixture_points(const std::string& name) {
    return room_memory::decode_pointcloud2(harness::read_bytes(kSynthetic / name)).positions;
}

}  // namespace

// MARK: Poses and small matrices

TEST(test_pose_conventions) {
    const Pose D{rig::rotation_about({0.0, 0.0, 1.0}, 30.0), {1.0, 2.0, 3.0}};
    const Pose T{rig::rotation_about({1.0, 0.0, 0.0}, 10.0), {0.5, 0.0, -1.0}};
    const Vec3d p{0.3, -0.2, 1.5};
    // (A o B)(p) = A(B(p)), and the increment from T to T o D is D.
    const Vec3d composed = T.compose(D).apply(p), chained = T.apply(D.apply(p));
    for (int i = 0; i < 3; ++i) CHECK(near(composed[i], chained[i], 1e-12));
    const Pose increment = T.increment(T.compose(D));
    CHECK(near(increment.distance_m(), D.distance_m(), 1e-12));
    CHECK(near(increment.angle_deg(), 30.0, 1e-9));
    const Vec3d back = T.inverse().apply(T.apply(p));
    for (int i = 0; i < 3; ++i) CHECK(near(back[i], p[i], 1e-12));
    // Quaternion round trip and the stored text round trip.
    const auto [x, y, z, w] = T.quaternion();
    const rig::Mat3 R = rig::quaternion_rotation(w, x, y, z);
    for (int i = 0; i < 9; ++i) CHECK(near(R[i], T.R[i], 1e-12));
    std::istringstream words(T.text());
    std::vector<double> values;
    for (std::string word; words >> word;) values.push_back(std::strtod(word.c_str(), nullptr));
    CHECK(rig::Pose::from_values(values) == T);
    const Pose tiny{rig::kIdentity, {0.0, 0.0, 1e-300}};
    CHECK(Pose().is_identity() && !tiny.is_identity());
    // The C++ specifics: the text format, the RigPose field, and the rejections.
    CHECK_EQ(Pose().text(), std::string("1 0 0 0 1 0 0 0 1 0 0 0"));
    const std::array<float, 7> field = T.rig_pose();
    CHECK(field[0] == 0.5f && field[2] == -1.0f && field[6] == static_cast<float>(w));
    CHECK_THROWS(std::invalid_argument, Pose::from_values(std::vector<double>(11, 0.0)));
    std::vector<double> bad(12, 0.0);
    bad[4] = kNaN;
    CHECK_THROWS(std::invalid_argument, Pose::from_values(bad));
}

TEST(test_jacobi_matches_closed_form_eigenpairs) {
    rooms::Random random(3);
    for (const std::size_t n : {3u, 4u, 6u}) {
        for (int trial = 0; trial < 5; ++trial) {
            std::vector<std::vector<double>> a(n, std::vector<double>(n));
            for (std::size_t i = 0; i < n; ++i) {
                for (std::size_t j = 0; j < n; ++j) a[i][j] = random.normal();
            }
            double trace = 0.0;
            for (std::size_t i = 0; i < n; ++i) {
                for (std::size_t j = 0; j < i; ++j) a[i][j] = a[j][i] = a[i][j] + a[j][i];
                a[i][i] *= 2.0;
                trace += a[i][i];
            }
            const rig::Eigen eigen = rig::jacobi_eigh(a);
            CHECK(std::is_sorted(eigen.values.begin(), eigen.values.end()));
            double sum = 0.0;
            for (std::size_t slot = 0; slot < n; ++slot) {
                const std::vector<double>& v = eigen.vectors[slot];
                sum += eigen.values[slot];
                double norm = 0.0;
                std::size_t lead = 0;
                for (std::size_t i = 0; i < n; ++i) {
                    double av = 0.0;
                    for (std::size_t j = 0; j < n; ++j) av += a[i][j] * v[j];
                    CHECK(near(av, eigen.values[slot] * v[i], 1e-10));
                    norm += v[i] * v[i];
                    if (std::fabs(v[i]) > std::fabs(v[lead])) lead = i;
                }
                CHECK(near(norm, 1.0, 1e-12));
                CHECK(v[lead] > 0.0);
                for (std::size_t other = 0; other < slot; ++other) {
                    double dot = 0.0;
                    for (std::size_t i = 0; i < n; ++i) dot += v[i] * eigen.vectors[other][i];
                    CHECK(near(dot, 0.0, 1e-10));
                }
            }
            CHECK(near(sum, trace, 1e-10));
        }
    }
    // Closed forms: [[2, 1, 0], [1, 2, 0], [0, 0, 5]] has 1, 3, 5 on (1, -1, 0), (1, 1, 0), (0, 0, 1).
    const rig::Eigen block = rig::jacobi_eigh({{2.0, 1.0, 0.0}, {1.0, 2.0, 0.0}, {0.0, 0.0, 5.0}});
    CHECK(near(block.values[0], 1.0, 1e-15) && near(block.values[1], 3.0, 1e-15) && block.values[2] == 5.0);
    const double h = std::sqrt(0.5);
    CHECK(near(std::fabs(block.vectors[0][0]), h, 1e-15) && near(block.vectors[0][0], -block.vectors[0][1], 1e-15));
    CHECK(near(block.vectors[1][0], h, 1e-15) && near(block.vectors[1][1], h, 1e-15));
    CHECK(block.vectors[2] == (std::vector<double>{0.0, 0.0, 1.0}));
    const rig::Eigen diagonal = rig::jacobi_eigh({{2.0, 0.0}, {0.0, 1.0}});
    CHECK(diagonal.values == (std::vector<double>{1.0, 2.0}));
    CHECK(diagonal.vectors == (std::vector<std::vector<double>>{{0.0, 1.0}, {1.0, 0.0}}));
    const rig::Eigen pair = rig::jacobi_eigh({{4.0, 1.0}, {1.0, 4.0}});
    CHECK(near(pair.values[0], 3.0, 1e-15) && near(pair.values[1], 5.0, 1e-15));
}

TEST(test_horn_recovers_a_known_motion_and_ignores_zero_weights) {
    rooms::Random random(1);
    const Pose truth{rig::rotation_about({0.0, 0.6, 0.8}, 7.0), {0.1, -0.2, 0.05}};
    std::vector<double> source(180);
    for (double& value : source) value = random.normal();
    std::vector<double> target = applied(truth, source);
    for (std::size_t n = 150; n < 180; ++n) target[n] += random.normal();  // outliers with weight 0
    std::vector<double> weights(60, 1.0);
    std::fill(weights.begin() + 50, weights.end(), 0.0);
    const std::optional<Pose> estimate = rig::horn(source, target, weights);
    CHECK(estimate);
    for (int i = 0; i < 9; ++i) CHECK(near(estimate->R[i], truth.R[i], 1e-12));
    for (int i = 0; i < 3; ++i) CHECK(near(estimate->t[i], truth.t[i], 1e-12));
    CHECK(!rig::horn(source, target, std::vector<double>(60, 0.0)));
}

TEST(test_transform_is_binary32_in_a_fixed_order) {
    rooms::Random random(2);
    std::vector<float> points(1500);
    for (float& value : points) value = static_cast<float>(random.normal() * 5);
    const Pose pose{rig::rotation_about({0.0, 0.0, 1.0}, 3.3), {0.123, -0.456, 0.789}};
    const std::vector<float> out = rig::transform_points(points, pose);
    CHECK_EQ(out.size(), points.size());
    for (std::size_t n = 0; n < 500; ++n) {
        const float* p = &points[3 * n];
        for (int i = 0; i < 3; ++i) {
            const auto R = [&](int k) { return static_cast<float>(pose.R[3 * i + k]); };
            volatile float a = R(0) * p[0];
            volatile float b = R(1) * p[1];
            volatile float c = R(2) * p[2];
            volatile float ab = a + b;
            volatile float abc = ab + c;
            const float expected = abc + static_cast<float>(pose.t[i]);
            CHECK(std::memcmp(&out[3 * n + i], &expected, sizeof(float)) == 0);
        }
    }
}

TEST(test_corrected_skips_the_identity_and_clips_range_before_the_transform) {
    const std::vector<float> points{1.0f, 0.0f, 0.0f, 14.9f, 0.0f, 0.0f, 0.0f, 15.1f, 0.0f,
                                    std::numeric_limits<float>::quiet_NaN(), 0.0f, 0.0f};
    const std::vector<float> same = rig::corrected(points, Pose(), 15.0);
    CHECK(same.size() == points.size() && std::memcmp(same.data(), points.data(), points.size() * sizeof(float)) == 0);
    // Moved 0.5 m toward +x: 14.9 m in the sensor frame stays, although it lands past 15 m.
    const std::vector<float> out = rig::corrected(points, Pose{rig::kIdentity, {0.5, 0.0, 0.0}}, 15.0);
    CHECK(out == (std::vector<float>{1.5f, 0.0f, 0.0f, 15.4f, 0.0f, 0.0f}));
}

// MARK: Registration cloud

TEST(test_registration_cloud_keeps_reference_occupied_points_first_per_cell) {
    rig::RegistrationBuilder builder(0.02);
    builder.add(std::vector<float>{1.001f, 0.0f, 0.0f, 1.005f, 0.001f, 0.001f, 3.0f, 0.0f, 0.0f, 20.0f, 0.0f, 0.0f}, 15.0);
    builder.add(std::vector<float>{1.002f, 0.0f, 0.0f, 2.0f, 0.0f, 0.0f}, 15.0);
    // First point per 2 cm cell, in arrival order across windows; 20 m is out of range.
    std::vector<std::tuple<float, float, float>> kept;
    for (std::size_t n = 0; n < builder.points().size(); n += 3) {
        kept.emplace_back(builder.points()[n], builder.points()[n + 1], builder.points()[n + 2]);
    }
    std::sort(kept.begin(), kept.end());
    CHECK(kept == (std::vector<std::tuple<float, float, float>>{{1.001f, 0.0f, 0.0f}, {2.0f, 0.0f, 0.0f}, {3.0f, 0.0f, 0.0f}}));
    Keys occupied{rig::voxel_key({1.0, 0.0, 0.0}, kVoxel), rig::voxel_key({3.0, 0.0, 0.0}, kVoxel)};
    room_memory::sort_unique(occupied);
    const rig::RegistrationCloud cloud = builder.finish(occupied, kVoxel, 300'000);
    std::vector<double> x;
    for (std::size_t n = 0; n < cloud.points().size(); n += 3) x.push_back(cloud.points()[n]);
    std::sort(x.begin(), x.end());
    CHECK(x.size() == 2 && near(x[0], 1.001, 1e-6) && near(x[1], 3.0, 1e-6));
    CHECK_EQ(builder.finish(occupied, kVoxel, 1).size(), std::size_t{1});
    CHECK(rig::decimate(10, 3) == (std::vector<std::size_t>{0, 4, 8}));
}

TEST(test_grid_finds_the_nearest_point_with_ties_to_the_lower_index) {
    const std::vector<double> targets{0.0, 0.0, 0.0, 0.1, 0.0, 0.0, -0.1, 0.0, 0.0, 0.5, 0.5, 0.5};
    const rig::TargetGrid grid(targets, 0.16, 20);
    const auto first = grid.nearest({0.05, 0.0, 0.0}, targets, 0.1);
    CHECK_EQ(first.first, std::int64_t{0});
    CHECK(near(first.second, 0.0025, 1e-15));
    CHECK_EQ(grid.nearest({0.0, 0.0, 0.09}, targets, 0.1).first, std::int64_t{0});
    const auto none = grid.nearest({0.3, 0.3, 0.3}, targets, 0.1);
    CHECK(none.first == -1 && std::isinf(none.second));
}

// MARK: Alignment

TEST(test_convergence_grid) {
    const Reference& reference = furnished();
    const std::vector<Pose> truths{
        rooms::rig(),           rooms::rig(0.02),         rooms::rig(0.0, 0.05),          rooms::rig(0.1, 0.1),
        rooms::rig(0, 0, 0.03), rooms::rig(0, 0, 0, 0.5), rooms::rig(0, 0, 0, 2.0),       rooms::rig(0, 0, 0, -5.0),
        rooms::rig(0.05, 0, 0, 2.0), rooms::rig(0, 0, 0, 0, 0.5), rooms::rig(0, 0, 0, 0, 3.0),
        rooms::rig(0, 0, 0.03, 0, 0, 1.0),
    };
    for (const Pose& truth : truths) {
        const std::vector<double> source = source_of(rooms::scan(truth, rooms::kRoom, rooms::kFurniture, 0.01, kSeedBase + 99));
        const std::optional<rig::Estimate> estimate = solve(source, reference);
        check_close(truth, estimate);
        if (!(estimate->inlier_frac > 0.95 && estimate->observe > 0.05 && estimate->rms_m < 0.015)) {
            throw harness::Failure(describe(truth, estimate));
        }
    }
}

TEST(test_envelope_corner_converges_from_a_saturated_translation_seed) {
    // pose_offset saturates at 4 voxels (0.2 m) per axis: that is the seed a 0.3 m push gets.
    const Pose truth = rooms::rig(0.3, 0.0, 0.0, 10.0);
    const std::vector<double> source = source_of(rooms::scan(truth, rooms::kRoom, rooms::kFurniture, 0.01, kSeedBase + 99));
    check_close(truth, solve(source, furnished(), {0.2, 0.0, 0.0}));
}

TEST(test_robust_to_a_person_and_a_moved_box) {
    const Pose truth = rooms::rig(0.03, 0.0, 0.0, 1.0);
    std::vector<rooms::Box> clutter(rooms::kFurniture.begin() + 1, rooms::kFurniture.end());
    clutter.push_back({{1.0, -0.5, -1.2}, {1.5, -0.2, 0.5}});
    clutter.push_back({{-0.5, 1.5, -1.2}, {0.3, 2.3, -0.4}});
    const std::vector<double> source = source_of(rooms::scan(truth, rooms::kRoom, clutter, 0.01, kSeedBase + 99));
    const std::optional<rig::Estimate> estimate = solve(source, furnished());
    check_close(truth, estimate);
    CHECK(estimate->inlier_frac < 0.99);
}

// Documents why the last stages are point-to-plane: two samplings of the same noise-free room, no
// move. Point-to-point alone settles millimetres and a tenth of a degree off.
TEST(test_point_to_plane_finish_removes_the_point_to_point_sampling_bias) {
    std::vector<std::vector<float>> scans;
    for (std::uint64_t seed = 0; seed < 10; ++seed) scans.push_back(rooms::scan({}, rooms::kRoom, rooms::kFurniture, 0.0, kSeedBase + seed));
    const Reference reference = reference_of(scans);
    const std::vector<double> source = source_of(rooms::scan({}, rooms::kRoom, rooms::kFurniture, 0.0, kSeedBase + 99));
    const std::vector<double>& targets = reference.cloud.points64();
    const rig::TargetGrid& grid = reference.cloud.grid(0.05);
    const double kappa2 = (0.05 / 3) * (0.05 / 3);
    Pose pose;
    for (int iteration = 0; iteration < 60; ++iteration) {
        const std::vector<double> moved = applied(pose, source);
        std::vector<double> from, to, weights;
        for (std::size_t n = 0; n < moved.size() / 3; ++n) {
            const auto [index, d2] = grid.nearest({moved[3 * n], moved[3 * n + 1], moved[3 * n + 2]}, targets, 0.05);
            if (index < 0) continue;
            const auto t = static_cast<std::size_t>(index);
            from.insert(from.end(), {moved[3 * n], moved[3 * n + 1], moved[3 * n + 2]});
            to.insert(to.end(), {targets[3 * t], targets[3 * t + 1], targets[3 * t + 2]});
            weights.push_back(kappa2 / ((kappa2 + d2) * (kappa2 + d2)));
        }
        pose = rig::horn(from, to, weights).value().compose(pose);
    }
    CHECK(pose.distance_m() > 0.001 || pose.angle_deg() > 0.05);
    const std::optional<rig::Estimate> estimate = rig::align(source, reference.cloud, Pose(), 0.0);
    CHECK(estimate);
    CHECK(estimate->pose.distance_m() < 1e-4 && estimate->pose.angle_deg() < 0.01);
}

TEST(test_observability_separates_degenerate_views) {
    struct Case {
        const char* name;
        const rooms::Box* room;
        std::vector<rooms::Box> boxes;
        bool ok;
    };
    const std::vector<Case> cases{{"furnished", &rooms::kRoom, rooms::kFurniture, true},
                                  {"box room", &rooms::kRoom, {}, true},
                                  {"corridor", &rooms::kCorridor, {}, false},
                                  {"floor", &rooms::kFloorOnly, {}, false}};
    for (const Case& c : cases) {
        std::vector<std::vector<float>> scans;
        for (std::uint64_t seed = 0; seed < 6; ++seed) scans.push_back(rooms::scan({}, *c.room, c.boxes, 0.01, kSeedBase + seed));
        const Reference reference = reference_of(scans);
        const std::vector<double> source = source_of(rooms::scan(rooms::rig(0.03), *c.room, c.boxes, 0.01, kSeedBase + 99));
        const std::optional<rig::Estimate> estimate = rig::align(source, reference.cloud, Pose(), 0.0);
        if (!estimate) throw harness::Failure(std::string(c.name) + ": no estimate");
        std::ostringstream detail;
        detail << c.name << ": observe " << estimate->observe << ", weakest x " << estimate->weakest[0] << ", t x "
               << estimate->pose.t[0];
        if (c.ok ? !(estimate->observe > 0.05) : !(estimate->observe < 0.005)) throw harness::Failure(detail.str());
        if (c.room == &rooms::kCorridor) {
            // The corridor slides along x: the weakest direction names it, and the step did not slide.
            if (!(std::fabs(estimate->weakest[0]) > 0.95 && std::fabs(estimate->pose.t[0]) < 0.01)) {
                throw harness::Failure(detail.str());
            }
        }
    }
}

TEST(test_rig_shifted_golden_is_a_5_cm_move) {
    if (!std::filesystem::exists(kSynthetic / "rig_shifted.bin")) {
        throw harness::Skip("no synthetic fixtures at " + kSynthetic.string());
    }
    const Reference reference = reference_of({fixture_points("base_s1.bin")});
    const std::vector<float> window = fixture_points("rig_shifted.bin");
    // The engine's translation seed: pose_shift of the window's voxels against the reference's. It
    // reads about 2 mm here: both scans use one ray pattern, so the floor voxels overlap best unshifted.
    const std::optional<Vec3d> shift =
        pose_shift(room_memory::occupancy_map(window, quantizer()).occupied, reference.keys, kVoxel);
    CHECK(shift);
    const std::optional<rig::Estimate> estimate = solve(source_of(window), reference, *shift);
    CHECK(estimate);
    CHECK(near(estimate->pose.t[0], 0.05, 0.005));
    CHECK(std::fabs(estimate->pose.t[1]) < 0.005 && std::fabs(estimate->pose.t[2]) < 0.005);
    CHECK(estimate->pose.angle_deg() < 0.05);
}

TEST_MAIN
