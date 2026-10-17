#include "rooms.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "room_memory/keys.hpp"

namespace rooms {

using room_memory::rig::Pose;
using room_memory::rig::Vec3d;

const Box kRoom{{-3.0, -2.0, -1.2}, {5.0, 4.0, 1.8}};
const std::vector<Box> kFurniture{
    {{1.5, 1.0, -1.2}, {2.3, 1.8, -0.4}},  {{-2.5, -1.8, -1.2}, {-1.7, -0.6, 0.6}},
    {{3.5, -1.5, -1.2}, {4.2, -0.5, 0.3}}, {{0.5, -2.0, -0.2}, {1.5, -1.6, 0.5}},
    {{-1.0, 3.0, -1.2}, {0.0, 3.6, -0.1}}, {{2.8, 2.6, -1.2}, {3.4, 3.2, 1.1}},
};
const Box kCorridor{{-100.0, -1.0, -1.2}, {100.0, 1.0, 100.0}};
const Box kFloorOnly{{-100.0, -100.0, -1.2}, {100.0, 100.0, 100.0}};

namespace {

constexpr double kPi = 3.141592653589793;
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

double radians(double degrees) { return degrees * (kPi / 180.0); }

// numpy's minimum and maximum propagate NaN; nanmax and nanmin skip it (all NaN: NaN).
double nan_min(double a, double b) { return std::isnan(a) || std::isnan(b) ? kNaN : std::min(a, b); }
double nan_max(double a, double b) { return std::isnan(a) || std::isnan(b) ? kNaN : std::max(a, b); }

// (near, far) of the ray origin + s * direction against the box's slabs.
std::pair<double, double> slabs(const Vec3d& origin, const Vec3d& direction, const Box& box) {
    double near = kNaN, far = kNaN;
    for (int axis = 0; axis < 3; ++axis) {
        const double t1 = (box.first[axis] - origin[axis]) / direction[axis];
        const double t2 = (box.second[axis] - origin[axis]) / direction[axis];
        const double low = nan_min(t1, t2), high = nan_max(t1, t2);
        if (!std::isnan(low)) near = std::isnan(near) ? low : std::max(near, low);
        if (!std::isnan(high)) far = std::isnan(far) ? high : std::min(far, high);
    }
    return {near, far};
}

}  // namespace

std::uint64_t Random::next() {
    const std::uint64_t out = room_memory::splitmix64(state_);
    state_ += 0x9E3779B97F4A7C15ULL;
    return out;
}

double Random::uniform() { return static_cast<double>(next() >> 11) * 0x1.0p-53; }

double Random::normal() {
    if (spare_) {
        const double value = *spare_;
        spare_.reset();
        return value;
    }
    const double u = 1.0 - uniform();  // (0, 1]
    const double v = uniform();
    const double radius = std::sqrt(-2.0 * std::log(u));
    spare_ = radius * std::sin(2.0 * kPi * v);
    return radius * std::cos(2.0 * kPi * v);
}

std::vector<double> fan(int rows, int columns, double low_deg, double high_deg, std::optional<std::uint64_t> seed) {
    double shift[2] = {0.0, 0.0};
    if (seed) {
        Random random(*seed);
        shift[0] = random.uniform();
        shift[1] = random.uniform();
    }
    const double row_step = (high_deg - low_deg) / (rows - 1);
    std::vector<double> out;
    out.reserve(3 * static_cast<std::size_t>(rows) * static_cast<std::size_t>(columns));
    for (int r = 0; r < rows; ++r) {
        // np.linspace: r * step + low, the last one exactly high.
        const double line = r == rows - 1 && rows > 1 ? high_deg : r * row_step + low_deg;
        const double e = radians(line + (shift[0] - 0.5) * row_step);
        for (int c = 0; c < columns; ++c) {
            const double a = radians((c + shift[1]) * (360.0 / columns));
            out.insert(out.end(), {std::cos(e) * std::cos(a), std::cos(e) * std::sin(a), std::sin(e)});
        }
    }
    return out;
}

std::vector<float> scan(const Pose& pose, const Box& room, const std::vector<Box>& boxes, double noise_m,
                        std::uint64_t seed, double max_range_m) {
    const std::vector<double> directions = fan(32, 720, -35.0, 30.0, seed);
    const auto& R = pose.R;
    Random random(seed);
    std::vector<float> out;
    for (std::size_t n = 0; n < directions.size() / 3; ++n) {
        const double* d = &directions[3 * n];
        Vec3d world{};
        for (int i = 0; i < 3; ++i) world[i] = (d[0] * R[3 * i] + d[1] * R[3 * i + 1]) + d[2] * R[3 * i + 2];
        double distance = slabs(pose.t, world, room).second;  // the room is seen from inside: its far face
        for (const Box& box : boxes) {
            const auto [near, far] = slabs(pose.t, world, box);
            if (near <= far && near > 0 && near < distance) distance = near;
        }
        distance = distance + noise_m * random.normal();
        if (!(std::isfinite(distance) && distance > 0.1 && distance < max_range_m)) continue;
        out.insert(out.end(), {static_cast<float>(d[0] * distance), static_cast<float>(d[1] * distance),
                               static_cast<float>(d[2] * distance)});
    }
    return out;
}

Pose rig(double dx, double dy, double dz, double yaw_deg, double roll_deg, double pitch_deg) {
    using room_memory::rig::rotation_about;
    Pose pose{rotation_about({0.0, 0.0, 1.0}, yaw_deg), {dx, dy, dz}};
    if (pitch_deg != 0.0) pose.R = Pose{pose.R, {}}.compose(Pose{rotation_about({0.0, 1.0, 0.0}, pitch_deg), {}}).R;
    if (roll_deg != 0.0) pose.R = Pose{pose.R, {}}.compose(Pose{rotation_about({1.0, 0.0, 0.0}, roll_deg), {}}).R;
    return pose;
}

std::pair<double, double> pose_error(const Pose& estimate, const Pose& truth) {
    const Pose difference = truth.inverse().compose(estimate);
    return {difference.distance_m(), difference.angle_deg()};
}

}  // namespace rooms
