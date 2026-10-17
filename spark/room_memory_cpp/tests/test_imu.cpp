// The rig guard's IMU detector on synthetic 200 Hz streams, ported from
// spark/room_memory/tests/test_imu.py. numpy's generator is not reproducible here, so the noise is
// splitmix64 + Box-Muller at the same levels.
#include <cmath>
#include <numbers>
#include <vector>

#include "harness.hpp"
#include "room_memory/imu.hpp"

using namespace room_memory;

namespace {

constexpr int kRate = 200;
constexpr std::int64_t kNs = 1'000'000'000;
constexpr std::int64_t kStart = 1'760'400'000 * kNs;

// Deterministic normals: splitmix64 uniforms in (0, 1), one Box-Muller cosine per draw.
class Normal {
public:
    explicit Normal(std::uint64_t seed) : state_(seed) {}
    double operator()(double mean, double sigma) {
        const double u1 = uniform();
        const double u2 = uniform();
        return mean + sigma * (std::sqrt(-2.0 * std::log(u1)) * std::cos(2.0 * std::numbers::pi * u2));
    }

private:
    double uniform() {
        std::uint64_t z = (state_ += 0x9E3779B97F4A7C15ULL);
        z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ULL;
        z = (z ^ (z >> 27)) * 0x94D049BB133111EBULL;
        z ^= z >> 31;
        return (static_cast<double>(z >> 11) + 0.5) * 0x1.0p-53;
    }
    std::uint64_t state_;
};

struct Event {
    double t0;
    double t1;
    rig::Vec3d accel;
    rig::Vec3d rate;
};

struct Options {
    double g0 = 1.0;
    double noise = 0.002;
    double gyro_noise = 0.001;
    std::uint64_t seed = 0;
    double start_s = 0.0;
    rig::Vec3d gravity{0.0, 0.0, -1.0};
    std::vector<Event> events;
};

using Stream = std::vector<std::pair<std::int64_t, msg::ImuSample>>;

// (received ns, sample) at kRate Hz; `events` adds accel offsets (x g0) and gyro rates over [t0, t1).
Stream stream(double seconds, const Options& o = {}) {
    Normal normal(o.seed);
    Stream out;
    const int count = static_cast<int>(seconds * kRate);
    for (int n = 0; n < count; ++n) {
        const double t = o.start_s + static_cast<double>(n) / kRate;
        rig::Vec3d a, w;
        for (int k = 0; k < 3; ++k) a[k] = o.gravity[k] * o.g0 + normal(0.0, o.noise * o.g0);
        const rig::Vec3d bias{0.001, -0.002, 0.0005};  // a constant bias
        for (int k = 0; k < 3; ++k) w[k] = normal(0.0, o.gyro_noise) + bias[k];
        for (const Event& event : o.events) {
            if (event.t0 <= t && t < event.t1) {
                for (int k = 0; k < 3; ++k) {
                    a[k] = a[k] + event.accel[k] * o.g0;
                    w[k] = w[k] + event.rate[k];
                }
            }
        }
        const std::int64_t ns = kStart + static_cast<std::int64_t>(std::nearbyint(t * static_cast<double>(kNs)));
        msg::ImuSample sample;
        sample.stamp = msg::Time{ns / kNs, ns % kNs};
        sample.frame_id = "livox_frame";
        sample.angular_velocity = w;
        sample.linear_acceleration = a;
        out.emplace_back(ns, sample);
    }
    return out;
}

std::int64_t feed(ImuDetector& detector, const Stream& samples) {
    for (const auto& [ns, sample] : samples) detector.add(sample, ns);
    return samples.back().first;
}

bool near(double actual, double expected, double tolerance) { return std::fabs(actual - expected) <= tolerance; }

}  // namespace

TEST(calibrates_in_either_unit_and_finds_the_bias) {
    for (const double g0 : {1.0, 9.80665}) {
        ImuDetector detector(0.05, 2.0, 2.0);
        feed(detector, stream(1.5, {.g0 = g0}));
        CHECK(!detector.calibrated());  // still within quiet_s of the stream start
        const std::int64_t end = feed(detector, stream(1.0, {.g0 = g0, .start_s = 1.5}));
        CHECK(near(*detector.g0(), g0, 0.01 * g0));
        const rig::Vec3d bias = *detector.bias();
        CHECK(near(bias[0], 0.001, 3e-4) && near(bias[1], -0.002, 3e-4) && near(bias[2], 0.0005, 3e-4));
        CHECK(detector.ok(end) && !detector.take_shock());
    }
}

TEST(a_50_ms_shock_triggers_and_noise_does_not) {
    {
        ImuDetector detector(0.05, 2.0, 2.0);
        const std::int64_t end = feed(detector, stream(600.0, {.noise = 0.006, .gyro_noise = 0.006}));  // 3x, 10 min
        CHECK(!detector.take_shock() && detector.quiet(end));
    }
    ImuDetector detector(0.05, 2.0, 2.0);
    feed(detector, stream(5.0));
    std::int64_t end = feed(detector, stream(0.05, {.start_s = 5.0, .events = {{5.0, 5.05, {0.4, 0.0, 0.0}, {0.0, 0.0, 0.0}}}}));
    CHECK(detector.take_shock() && !detector.take_shock());
    CHECK(!detector.quiet(end));
    end = feed(detector, stream(2.5, {.seed = 1, .start_s = 5.05}));
    CHECK(detector.quiet(end));
}

TEST(a_stream_gap_recalibrates_without_a_shock) {
    ImuDetector detector(0.05, 2.0, 2.0);
    std::int64_t end = feed(detector, stream(5.0));
    CHECK(detector.ok(end) && !detector.ok(end + 2 * kNs));
    // The stream returns 30 s later in m/s^2: a new g0, no shock.
    end = feed(detector, stream(5.0, {.g0 = 9.80665, .start_s = 35.0}));
    CHECK(near(*detector.g0(), 9.80665, 0.01 * 9.80665));
    CHECK(!detector.take_shock() && detector.ok(end));
}

TEST(the_gyro_integral_gives_the_rotation_of_a_shock) {
    ImuDetector detector(0.05, 2.0, 2.0);
    feed(detector, stream(5.0, {.gyro_noise = 0.0005}));
    // A 2 degree yaw over 0.5 s, then rest.
    const double rate = 2.0 * (std::numbers::pi / 180.0) / 0.5;
    feed(detector, stream(0.5, {.gyro_noise = 0.0005, .start_s = 5.0, .events = {{5.0, 5.5, {0.0, 0.0, 0.0}, {0.0, 0.0, rate}}}}));
    CHECK(detector.take_shock());
    std::int64_t end = feed(detector, stream(1.0, {.gyro_noise = 0.0005, .start_s = 5.5}));
    CHECK(!detector.take_rotation(end));  // not quiet yet
    end = feed(detector, stream(1.5, {.gyro_noise = 0.0005, .start_s = 6.5}));
    const std::optional<rig::Mat3> R = detector.take_rotation(end);
    CHECK(R.has_value());
    const rig::Pose rotation{*R, {0.0, 0.0, 0.0}};
    CHECK(near(rotation.angle_deg(), 2.0, 0.05));
    CHECK(rotation.R[3] > 0.0);  // about +z
    CHECK(!detector.take_rotation(end));
}

TEST(gravity_is_the_mean_direction_of_undisturbed_samples) {
    ImuDetector detector(0.05, 2.0, 2.0);
    feed(detector, stream(3.0));
    detector.take_gravity();
    const double tilt = 0.3 * (std::numbers::pi / 180.0);
    feed(detector, stream(1.0, {.noise = 0.0005, .start_s = 3.0, .gravity = {std::sin(tilt), 0.0, -std::cos(tilt)}}));
    const std::optional<rig::Vec3d> gravity = detector.take_gravity();
    CHECK(gravity.has_value());
    CHECK(near(rig::angle_between_deg(*gravity, {0.0, 0.0, -1.0}), 0.3, 0.02));
    CHECK(!detector.take_gravity());
}

TEST(a_zero_magnitude_calibration_stays_uncalibrated) {
    ImuDetector detector(0.05, 2.0, 2.0);
    const std::int64_t end = feed(detector, stream(3.0, {.g0 = 0.0}));
    CHECK(!detector.calibrated() && !detector.ok(end) && !detector.quiet(end));
    feed(detector, stream(5.0, {.start_s = 3.0}));
    CHECK(detector.calibrated() && !detector.take_shock());
}

TEST_MAIN
