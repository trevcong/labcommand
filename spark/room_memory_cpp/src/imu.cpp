#include "room_memory/imu.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>

namespace room_memory {

namespace {

std::int64_t stamp_ns(const msg::Time& t) { return t.sec * 1'000'000'000 + t.nanosec; }

double norm(const rig::Vec3d& v) { return std::sqrt((v[0] * v[0] + v[1] * v[1]) + v[2] * v[2]); }

}  // namespace

ImuDetector::ImuDetector(double shock_frac, double gyro_dps, double quiet_s)
    : shock_frac_(shock_frac),
      gyro_rad_(gyro_dps * (std::numbers::pi / 180.0)),  // math.radians
      quiet_ns_(static_cast<std::int64_t>(quiet_s * 1e9)) {}

void ImuDetector::add(const msg::ImuSample& sample, std::int64_t received_ns) {
    const rig::Vec3d& a = sample.linear_acceleration;
    const rig::Vec3d& w = sample.angular_velocity;
    if (last_received_ && received_ns - *last_received_ > static_cast<std::int64_t>(kImuGapS * 1e9)) restart();
    last_received_ = received_ns;
    const double magnitude = norm(a);
    const std::int64_t stamp = stamp_ns(sample.stamp);
    if (!calibrated()) {
        if (!calibration_start_) calibration_start_ = received_ns;
        calibration_.emplace_back(magnitude, w);
        if (received_ns - *calibration_start_ >= quiet_ns_) calibrate();
        last_stamp_ = stamp;
        return;
    }
    const rig::Vec3d rate{w[0] - (*bias_)[0], w[1] - (*bias_)[1], w[2] - (*bias_)[2]};
    const double spin = norm(rate);
    const bool jolt = std::fabs(magnitude / *g0_ - 1.0) > shock_frac_;
    const bool turning = spin > gyro_rad_;
    run_ = turning ? run_ + 1 : 0;
    if (jolt || turning) last_disturbance_ = received_ns;
    if ((jolt || run_ >= kImuGyroRun) && !integrating_) {
        shock_ = true;
        integrating_ = true;
        rotation_ = rig::kIdentity;
        finished_.reset();
    }
    if (integrating_ && last_stamp_) {
        const double step = static_cast<double>(stamp - *last_stamp_) / 1e9;
        if (0.0 < step && step <= kImuMaxStepS) {
            const rig::Mat3 increment = rig::rotation_vector(rate[0] * step, rate[1] * step, rate[2] * step);
            rotation_ = rig::Pose{rotation_, {0.0, 0.0, 0.0}}.compose(rig::Pose{increment, {0.0, 0.0, 0.0}}).R;
        }
    }
    if (!(jolt || turning)) {
        for (int n = 0; n < 3; ++n) gravity_[n] += a[n];
        ++gravity_count_;
    }
    last_stamp_ = stamp;
}

void ImuDetector::calibrate() {
    std::vector<double> magnitudes;
    magnitudes.reserve(calibration_.size());
    for (const auto& [magnitude, w] : calibration_) magnitudes.push_back(magnitude);
    std::sort(magnitudes.begin(), magnitudes.end());
    g0_ = magnitudes[magnitudes.size() / 2];
    // The mean in sample order (CPython before 3.12 sums plainly).
    const auto count = static_cast<double>(calibration_.size());
    rig::Vec3d bias{};
    for (int n = 0; n < 3; ++n) {
        double sum = 0.0;
        for (const auto& [magnitude, w] : calibration_) sum += w[n];
        bias[n] = sum / count;
    }
    bias_ = bias;
    calibration_.clear();
    if (!(*g0_ > 0.0)) {
        g0_.reset();
        calibration_start_.reset();
    }
}

void ImuDetector::restart() {
    g0_.reset();
    bias_.reset();
    calibration_.clear();
    calibration_start_.reset();
    run_ = 0;
    integrating_ = false;
    rotation_ = rig::kIdentity;
    last_stamp_.reset();
}

bool ImuDetector::ok(std::int64_t now_ns) const {
    return calibrated() && last_received_ && now_ns - *last_received_ <= 1'000'000'000;
}

bool ImuDetector::quiet(std::int64_t now_ns) const {
    return calibrated() && (!last_disturbance_ || now_ns - *last_disturbance_ >= quiet_ns_);
}

bool ImuDetector::take_shock() { return std::exchange(shock_, false); }

std::optional<rig::Mat3> ImuDetector::take_rotation(std::int64_t now_ns) {
    if (integrating_ && quiet(now_ns)) {
        integrating_ = false;
        finished_ = rotation_;
    }
    return std::exchange(finished_, std::nullopt);
}

std::optional<rig::Vec3d> ImuDetector::take_gravity() {
    if (!gravity_count_) return std::nullopt;
    const auto count = static_cast<double>(gravity_count_);
    const rig::Vec3d gravity{gravity_[0] / count, gravity_[1] / count, gravity_[2] / count};
    gravity_ = {0.0, 0.0, 0.0};
    gravity_count_ = 0;
    if (gravity[0] == 0.0 && gravity[1] == 0.0 && gravity[2] == 0.0) return std::nullopt;
    return gravity;
}

}  // namespace room_memory
