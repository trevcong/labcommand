// The rig guard's IMU detector (Mid-360 sensor_msgs/Imu): shocks, quiet, gravity and the gyro
// rotation over a shock. The C++ port of room_memory/imu.py; read its docstring for the model.
// Times are receive times in nanoseconds; samples arrive in order. No ROS dependency.
#pragma once

#include <cstdint>
#include <optional>
#include <utility>
#include <vector>

#include "room_memory/guard.hpp"
#include "room_memory/messages.hpp"

namespace room_memory {

inline constexpr double kImuGapS = 1.0;
inline constexpr int kImuGyroRun = 3;
// The longest gap between two header stamps the gyro integral bridges (a dropped sample or two).
inline constexpr double kImuMaxStepS = 0.05;

class ImuDetector {
public:
    ImuDetector(double shock_frac, double gyro_dps, double quiet_s);

    void add(const msg::ImuSample& sample, std::int64_t received_ns);
    bool calibrated() const { return g0_.has_value(); }
    // Samples within the last second, and calibrated.
    bool ok(std::int64_t now_ns) const;
    // Calibrated and no disturbance within quiet_s before now_ns.
    bool quiet(std::int64_t now_ns) const;
    // Whether a shock started since the last call.
    bool take_shock();
    // The gyro rotation of the last shock (the increment D's rotation, sensor frame), once the IMU is
    // quiet after it; empty while it is still moving or without one. Taken once.
    std::optional<rig::Mat3> take_rotation(std::int64_t now_ns);
    // The mean acceleration of the undisturbed samples since the last call, or empty.
    std::optional<rig::Vec3d> take_gravity();

    std::optional<double> g0() const { return g0_; }
    std::optional<rig::Vec3d> bias() const { return bias_; }

private:
    void calibrate();
    void restart();

    double shock_frac_;
    double gyro_rad_;
    std::int64_t quiet_ns_;
    std::optional<double> g0_;
    std::optional<rig::Vec3d> bias_;
    std::optional<std::int64_t> last_received_;
    std::vector<std::pair<double, rig::Vec3d>> calibration_;  // (|a|, w) since the stream (re)started
    std::optional<std::int64_t> calibration_start_;
    int run_ = 0;
    std::optional<std::int64_t> last_disturbance_;
    bool shock_ = false;
    bool integrating_ = false;
    rig::Mat3 rotation_ = rig::kIdentity;
    std::optional<std::int64_t> last_stamp_;
    std::optional<rig::Mat3> finished_;
    rig::Vec3d gravity_{0.0, 0.0, 0.0};
    std::int64_t gravity_count_ = 0;
};

}  // namespace room_memory
