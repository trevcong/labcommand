#include "room_memory/guard.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <stdexcept>

#include "parallel.hpp"
#include "room_memory/key_set.hpp"

namespace room_memory::rig {

namespace {

constexpr double kPi = 3.141592653589793;
constexpr double kInf = std::numeric_limits<double>::infinity();

// CPython's math.radians and math.degrees: one multiply by a rounded constant.
double to_radians(double degrees) { return degrees * (kPi / 180.0); }
double to_degrees(double radians) { return radians * (180.0 / kPi); }

// 3 x 3 row-major, each entry ((a0 b0 + a1 b1) + a2 b2).
Mat3 matmul(const Mat3& a, const Mat3& b) {
    Mat3 out{};
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            out[3 * i + j] = (a[3 * i] * b[j] + a[3 * i + 1] * b[3 + j]) + a[3 * i + 2] * b[6 + j];
        }
    }
    return out;
}

Vec3d matvec(const Mat3& a, const Vec3d& v) {
    Vec3d out{};
    for (int i = 0; i < 3; ++i) out[i] = (a[3 * i] * v[0] + a[3 * i + 1] * v[1]) + a[3 * i + 2] * v[2];
    return out;
}

Mat3 transpose(const Mat3& a) { return {a[0], a[3], a[6], a[1], a[4], a[7], a[2], a[5], a[8]}; }

Key cell_key(double x, double y, double z, double size) {
    return pack(static_cast<std::int64_t>(std::floor(x / size)), static_cast<std::int64_t>(std::floor(y / size)),
                static_cast<std::int64_t>(std::floor(z / size)));
}

// Packed cell keys of binary32 points, through binary64: floor(p / size) per axis.
Keys float_cells(std::span<const float> xyz, double size) {
    Keys keys(xyz.size() / 3);
    for (std::size_t n = 0; n < keys.size(); ++n) {
        keys[n] = cell_key(xyz[3 * n], xyz[3 * n + 1], xyz[3 * n + 2], size);
    }
    return keys;
}

std::vector<float> kept_points(std::span<const float> xyz, std::span<const std::uint8_t> keep) {
    std::vector<float> out;
    out.reserve(xyz.size());
    for (std::size_t n = 0; n < keep.size(); ++n) {
        if (keep[n]) out.insert(out.end(), xyz.begin() + 3 * n, xyz.begin() + 3 * n + 3);
    }
    return out;
}

Vec3d point(std::span<const double> xyz, std::size_t n) { return {xyz[3 * n], xyz[3 * n + 1], xyz[3 * n + 2]}; }

std::vector<double> apply_all(const Pose& pose, std::span<const double> source) {
    std::vector<double> out(source.size());
    for (std::size_t n = 0; n < source.size() / 3; ++n) {
        const Vec3d p = pose.apply(point(source, n));
        out[3 * n] = p[0];
        out[3 * n + 1] = p[1];
        out[3 * n + 2] = p[2];
    }
    return out;
}

// Cyclic Jacobi on the row-major n x n `a` (destroyed), eigenvectors accumulated as the columns
// of `v`, then guard.py's ordering and signs.
template <typename Flat>
void jacobi(std::size_t n, Flat& a, Flat& v, std::vector<double>& values, std::vector<std::vector<double>>& vectors) {
    for (std::size_t i = 0; i < n; ++i) {
        for (std::size_t j = 0; j < n; ++j) v[i * n + j] = i == j ? 1.0 : 0.0;
    }
    for (int sweep = 0; sweep < kJacobiSweeps; ++sweep) {
        double off = 0.0;
        for (std::size_t p = 0; p + 1 < n; ++p) {
            for (std::size_t q = p + 1; q < n; ++q) off += a[p * n + q] * a[p * n + q];
        }
        if (off == 0.0) break;
        for (std::size_t p = 0; p + 1 < n; ++p) {
            for (std::size_t q = p + 1; q < n; ++q) {
                if (a[p * n + q] == 0.0) continue;
                const double theta = (a[q * n + q] - a[p * n + p]) / (2.0 * a[p * n + q]);
                const double tan = std::fabs(theta) > 1e150
                                       ? 0.5 / theta
                                       : (theta >= 0.0 ? 1.0 : -1.0) / (std::fabs(theta) + std::sqrt(theta * theta + 1.0));
                const double cos = 1.0 / std::sqrt(tan * tan + 1.0);
                const double sin = tan * cos;
                for (std::size_t k = 0; k < n; ++k) {
                    const double akp = a[k * n + p], akq = a[k * n + q];
                    a[k * n + p] = cos * akp - sin * akq;
                    a[k * n + q] = sin * akp + cos * akq;
                }
                for (std::size_t k = 0; k < n; ++k) {
                    const double apk = a[p * n + k], aqk = a[q * n + k];
                    a[p * n + k] = cos * apk - sin * aqk;
                    a[q * n + k] = sin * apk + cos * aqk;
                }
                for (std::size_t k = 0; k < n; ++k) {
                    const double vkp = v[k * n + p], vkq = v[k * n + q];
                    v[k * n + p] = cos * vkp - sin * vkq;
                    v[k * n + q] = sin * vkp + cos * vkq;
                }
            }
        }
    }
    std::vector<std::size_t> order(n);
    for (std::size_t i = 0; i < n; ++i) order[i] = i;
    std::stable_sort(order.begin(), order.end(),
                     [&](std::size_t x, std::size_t y) { return a[x * n + x] < a[y * n + y]; });
    values.resize(n);
    vectors.assign(n, std::vector<double>(n));
    for (std::size_t slot = 0; slot < n; ++slot) {
        const std::size_t i = order[slot];
        values[slot] = a[i * n + i];
        std::vector<double>& column = vectors[slot];
        std::size_t lead = 0;
        for (std::size_t k = 0; k < n; ++k) {
            column[k] = v[k * n + i];
            if (std::fabs(column[k]) > std::fabs(column[lead])) lead = k;
        }
        if (column[lead] < 0.0) {
            for (double& value : column) value = -value;
        }
    }
}

// The nearest kept target of every query (3 doubles each); each query is independent.
void nearest_all(const TargetGrid& grid, std::span<const double> queries, std::span<const double> points,
                 double radius, std::vector<std::int64_t>& match, std::vector<double>& d2) {
    const std::size_t count = queries.size() / 3;
    match.resize(count);
    d2.resize(count);
#pragma omp parallel for schedule(static) if (count >= 4096)
    for (std::size_t n = 0; n < count; ++n) {
        const auto [index, distance] = grid.nearest(point(queries, n), points, radius);
        match[n] = index;
        d2[n] = distance;
    }
}

struct PlaneSystem {
    std::vector<std::vector<double>> H;
    std::vector<double> g;
    double total = 0.0;
    std::int64_t count = 0;
    double squares = 0.0;
};

// guard.py _plane_system: sums in correspondence order.
PlaneSystem plane_system(std::span<const double> moved, const Pose& pose, const RegistrationCloud& cloud,
                         std::span<const std::int64_t> match, double tau, double scale) {
    PlaneSystem system;
    system.H.assign(6, std::vector<double>(6, 0.0));
    system.g.assign(6, 0.0);
    const std::vector<double>& normals = cloud.normals();
    const std::vector<double>& targets = cloud.points64();
    const double kappa2 = (tau / 3.0) * (tau / 3.0);
    for (std::size_t row = 0; row < match.size(); ++row) {
        if (match[row] < 0) continue;
        const auto target = static_cast<std::size_t>(match[row]);
        const Vec3d n = point(normals, target);
        if (!std::isfinite(n[0]) || !std::isfinite(n[1]) || !std::isfinite(n[2])) continue;
        const Vec3d p = point(moved, row);
        const Vec3d q = point(targets, target);
        const Vec3d d{p[0] - q[0], p[1] - q[1], p[2] - q[2]};
        const double r = (d[0] * n[0] + d[1] * n[1]) + d[2] * n[2];
        const Vec3d c{p[0] - pose.t[0], p[1] - pose.t[1], p[2] - pose.t[2]};
        const std::array<double, 6> J{n[0], n[1], n[2], (c[1] * n[2] - c[2] * n[1]) / scale,
                                      (c[2] * n[0] - c[0] * n[2]) / scale, (c[0] * n[1] - c[1] * n[0]) / scale};
        const double w = kappa2 / ((kappa2 + r * r) * (kappa2 + r * r));
        for (int i = 0; i < 6; ++i) {
            for (int j = 0; j < 6; ++j) system.H[i][j] += w * J[i] * J[j];
            system.g[i] += w * r * J[i];
        }
        system.total += w;
        system.count += 1;
        system.squares += r * r;
    }
    return system;
}

// guard.py _plane_step: the Gauss-Newton step without the directions below kStepEigFloor of the
// largest eigenvalue, rotating about the sensor's current position.
Pose plane_step(const PlaneSystem& system, const Pose& pose, double scale) {
    const Eigen eigen = jacobi_eigh(system.H);
    const double floor = kStepEigFloor * eigen.values.back();
    std::array<double, 6> x{};
    for (std::size_t slot = 0; slot < eigen.values.size(); ++slot) {
        const double value = eigen.values[slot];
        if (!(value > floor)) continue;
        const std::vector<double>& vector = eigen.vectors[slot];
        double dot = 0.0;
        for (int k = 0; k < 6; ++k) dot += vector[k] * system.g[k];
        const double along = -dot / value;
        for (int k = 0; k < 6; ++k) x[k] = x[k] + along * vector[k];
    }
    const Mat3 R = rotation_vector(x[3] / scale, x[4] / scale, x[5] / scale);
    const Vec3d Rt = matvec(R, pose.t);
    Pose step;
    step.R = R;
    for (int n = 0; n < 3; ++n) step.t[n] = (pose.t[n] - Rt[n]) + x[n];
    return step;
}

std::int64_t overlap_in(std::span<const double> source, const Pose& pose, const Membership& reference,
                        double voxel_size) {
    const std::size_t count = source.size() / 3;
    std::int64_t hits = 0;
#pragma omp parallel for schedule(static) reduction(+ : hits) if (count >= 4096)
    for (std::size_t n = 0; n < count; ++n) {
        hits += reference.contains(voxel_key(pose.apply(point(source, n)), voxel_size)) ? 1 : 0;
    }
    return hits;
}

}  // namespace

// MARK: Poses

Pose Pose::compose(const Pose& other) const {
    const Vec3d Rt = matvec(R, other.t);
    return Pose{matmul(R, other.R), {Rt[0] + t[0], Rt[1] + t[1], Rt[2] + t[2]}};
}

Pose Pose::inverse() const {
    const Mat3 Rt = transpose(R);
    const Vec3d back = matvec(Rt, t);
    return Pose{Rt, {-back[0], -back[1], -back[2]}};
}

double Pose::cos_angle() const { return std::max(-1.0, std::min(1.0, (((R[0] + R[4]) + R[8]) - 1.0) / 2.0)); }

double Pose::angle_deg() const { return to_degrees(std::acos(cos_angle())); }

double Pose::distance_m() const { return std::sqrt((t[0] * t[0] + t[1] * t[1]) + t[2] * t[2]); }

Vec3d Pose::apply(const Vec3d& p) const {
    Vec3d out{};
    for (int i = 0; i < 3; ++i) out[i] = ((R[3 * i] * p[0] + R[3 * i + 1] * p[1]) + R[3 * i + 2] * p[2]) + t[i];
    return out;
}

std::array<double, 4> Pose::quaternion() const { return rotation_quaternion(R); }

std::array<float, 7> Pose::rig_pose() const {
    const std::array<double, 4> q = quaternion();
    return {static_cast<float>(t[0]), static_cast<float>(t[1]), static_cast<float>(t[2]), static_cast<float>(q[0]),
            static_cast<float>(q[1]), static_cast<float>(q[2]), static_cast<float>(q[3])};
}

std::string Pose::text() const {
    std::string out;
    char buffer[32];
    for (int n = 0; n < 12; ++n) {
        std::snprintf(buffer, sizeof buffer, "%.17g", n < 9 ? R[n] : t[n - 9]);
        if (n) out += ' ';
        out += buffer;
    }
    return out;
}

Pose Pose::from_values(std::span<const double> values) {
    if (values.size() != 12 || !std::all_of(values.begin(), values.end(), [](double v) { return std::isfinite(v); })) {
        throw std::invalid_argument("a pose is 12 finite numbers");
    }
    Pose pose;
    std::copy(values.begin(), values.begin() + 9, pose.R.begin());
    std::copy(values.begin() + 9, values.end(), pose.t.begin());
    return pose;
}

Mat3 rotation_about(const Vec3d& axis, double degrees) {
    const double angle = to_radians(degrees);
    const double c = std::cos(angle), s = std::sin(angle);
    const double x = axis[0], y = axis[1], z = axis[2];
    const double v = 1.0 - c;
    return {c + x * x * v,         x * y * v - z * s, x * z * v + y * s,  //
            y * x * v + z * s,     c + y * y * v,     y * z * v - x * s,  //
            z * x * v - y * s,     z * y * v + x * s, c + z * z * v};
}

Mat3 rotation_vector(double rx, double ry, double rz) {
    const double angle = std::sqrt((rx * rx + ry * ry) + rz * rz);
    if (angle == 0.0) return kIdentity;
    return rotation_about({rx / angle, ry / angle, rz / angle}, to_degrees(angle));
}

Mat3 quaternion_rotation(double w, double x, double y, double z) {
    return {1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - w * z),       2.0 * (x * z + w * y),
            2.0 * (x * y + w * z),       1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - w * x),
            2.0 * (x * z - w * y),       2.0 * (y * z + w * x),       1.0 - 2.0 * (x * x + y * y)};
}

std::array<double, 4> rotation_quaternion(const Mat3& R) {
    const double trace = (R[0] + R[4]) + R[8];
    double w, x, y, z;
    if (trace > 0.0) {
        const double s = std::sqrt(trace + 1.0) * 2.0;
        w = 0.25 * s, x = (R[7] - R[5]) / s, y = (R[2] - R[6]) / s, z = (R[3] - R[1]) / s;
    } else if (R[0] > R[4] && R[0] > R[8]) {
        const double s = std::sqrt(1.0 + R[0] - R[4] - R[8]) * 2.0;
        w = (R[7] - R[5]) / s, x = 0.25 * s, y = (R[1] + R[3]) / s, z = (R[2] + R[6]) / s;
    } else if (R[4] > R[8]) {
        const double s = std::sqrt(1.0 + R[4] - R[0] - R[8]) * 2.0;
        w = (R[2] - R[6]) / s, x = (R[1] + R[3]) / s, y = 0.25 * s, z = (R[5] + R[7]) / s;
    } else {
        const double s = std::sqrt(1.0 + R[8] - R[0] - R[4]) * 2.0;
        w = (R[3] - R[1]) / s, x = (R[2] + R[6]) / s, y = (R[5] + R[7]) / s, z = 0.25 * s;
    }
    if (w < 0.0) w = -w, x = -x, y = -y, z = -z;
    const double norm = std::sqrt(((w * w + x * x) + y * y) + z * z);
    return {x / norm, y / norm, z / norm, w / norm};
}

double angle_between_deg(const Vec3d& a, const Vec3d& b) {
    const double dot = (a[0] * b[0] + a[1] * b[1]) + a[2] * b[2];
    const double na = std::sqrt((a[0] * a[0] + a[1] * a[1]) + a[2] * a[2]);
    const double nb = std::sqrt((b[0] * b[0] + b[1] * b[1]) + b[2] * b[2]);
    return to_degrees(std::acos(std::max(-1.0, std::min(1.0, dot / (na * nb)))));
}

Mat3 rotation_between(const Vec3d& a, const Vec3d& b) {
    const double na = std::sqrt((a[0] * a[0] + a[1] * a[1]) + a[2] * a[2]);
    const double nb = std::sqrt((b[0] * b[0] + b[1] * b[1]) + b[2] * b[2]);
    const Vec3d u{a[0] / na, a[1] / na, a[2] / na};
    const Vec3d v{b[0] / nb, b[1] / nb, b[2] / nb};
    const Vec3d axis{u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0]};
    const double s = std::sqrt((axis[0] * axis[0] + axis[1] * axis[1]) + axis[2] * axis[2]);
    if (s == 0.0) return kIdentity;
    const double c = std::max(-1.0, std::min(1.0, (u[0] * v[0] + u[1] * v[1]) + u[2] * v[2]));
    return rotation_about({axis[0] / s, axis[1] / s, axis[2] / s}, to_degrees(std::atan2(s, c)));
}

// MARK: Window points

std::vector<std::uint8_t> range_keep(std::span<const float> xyz, std::optional<double> max_range) {
    std::vector<std::uint8_t> keep(xyz.size() / 3);
    const float limit = max_range ? static_cast<float>(*max_range) : 0.0f;
    for (std::size_t n = 0; n < keep.size(); ++n) {
        const float x = xyz[3 * n], y = xyz[3 * n + 1], z = xyz[3 * n + 2];
        bool ok = std::isfinite(x) && std::isfinite(y) && std::isfinite(z);
        if (max_range) ok = ok && !(((x * x + y * y) + z * z) > limit * limit);
        keep[n] = ok;
    }
    return keep;
}

std::vector<float> transform_points(std::span<const float> xyz, const Pose& pose) {
    std::array<float, 9> R{};
    std::array<float, 3> t{};
    for (int n = 0; n < 9; ++n) R[n] = static_cast<float>(pose.R[n]);
    for (int n = 0; n < 3; ++n) t[n] = static_cast<float>(pose.t[n]);
    std::vector<float> out(xyz.size() / 3 * 3);
    for (std::size_t n = 0; n < out.size(); n += 3) {
        const float x = xyz[n], y = xyz[n + 1], z = xyz[n + 2];
        for (int i = 0; i < 3; ++i) out[n + i] = ((R[3 * i] * x + R[3 * i + 1] * y) + R[3 * i + 2] * z) + t[i];
    }
    return out;
}

std::vector<float> corrected(std::span<const float> xyz, const Pose& pose, std::optional<double> max_range) {
    if (pose.is_identity()) return std::vector<float>(xyz.begin(), xyz.end());
    return transform_points(kept_points(xyz, range_keep(xyz, max_range)), pose);
}

std::vector<std::size_t> first_per_cell(std::span<const float> xyz, double size) {
    const Keys keys = float_cells(xyz, size);
    const std::vector<std::uint32_t> order = sort_order(keys);
    std::vector<std::size_t> first;
    for (std::size_t n = 0; n < order.size(); ++n) {
        if (n == 0 || keys[order[n]] != keys[order[n - 1]]) first.push_back(order[n]);
    }
    return first;
}

std::vector<std::size_t> decimate(std::size_t count, std::size_t limit) {
    std::vector<std::size_t> out;
    if (count <= limit) {
        out.resize(count);
        for (std::size_t n = 0; n < count; ++n) out[n] = n;
        return out;
    }
    if (limit == 0) throw std::invalid_argument("decimate needs a positive limit");
    const std::size_t step = (count + limit - 1) / limit;
    for (std::size_t n = 0; n < count; n += step) out.push_back(n);
    return out;
}

std::vector<double> source_cloud(std::span<const float> xyz, std::optional<double> max_range, double voxel_m,
                                 std::size_t limit) {
    const std::vector<float> points = kept_points(xyz, range_keep(xyz, max_range));
    const std::vector<std::size_t> picked = first_per_cell(points, voxel_m);
    const std::vector<std::size_t> kept = decimate(picked.size(), limit);
    std::vector<double> out;
    out.reserve(3 * kept.size());
    for (const std::size_t n : kept) {
        const std::size_t at = 3 * picked[n];
        out.insert(out.end(), {points[at], points[at + 1], points[at + 2]});
    }
    return out;
}

Key voxel_key(const Vec3d& p, double voxel_size) {
    const float inverse = 1.0f / static_cast<float>(voxel_size);
    const float x = static_cast<float>(p[0]) * inverse;
    const float y = static_cast<float>(p[1]) * inverse;
    const float z = static_cast<float>(p[2]) * inverse;
    return pack(static_cast<std::int64_t>(std::floor(x)), static_cast<std::int64_t>(std::floor(y)),
                static_cast<std::int64_t>(std::floor(z)));
}

// MARK: Small symmetric eigenproblems

Eigen jacobi_eigh(const std::vector<std::vector<double>>& matrix) {
    const std::size_t n = matrix.size();
    std::vector<double> a(n * n), v(n * n);
    for (std::size_t i = 0; i < n; ++i) {
        for (std::size_t j = 0; j < n; ++j) a[i * n + j] = matrix[i][j];
    }
    Eigen eigen;
    jacobi(n, a, v, eigen.values, eigen.vectors);
    return eigen;
}

// MARK: Target grids

TargetGrid::TargetGrid(std::span<const double> points, double cell, std::size_t cap) : cell_(cell) {
    const std::size_t count = points.size() / 3;
    Keys keys(count);
    for (std::size_t n = 0; n < count; ++n) keys[n] = cell_key(points[3 * n], points[3 * n + 1], points[3 * n + 2], cell);
    const std::vector<std::uint32_t> order = sort_order(keys);
    for (std::size_t first = 0; first < count;) {
        std::size_t end = first + 1;
        while (end < count && keys[order[end]] == keys[order[first]]) ++end;
        const std::size_t members = end - first;
        const std::size_t step = cap && members > cap ? (members + cap - 1) / cap : 1;
        const std::size_t kept = (members + step - 1) / step;
        keys_.push_back(keys[order[first]]);
        start_.push_back(indices_.size());
        count_.push_back(kept);
        for (std::size_t o = 0; o < kept; ++o) indices_.push_back(order[first + o * step]);
        first = end;
    }
}

std::int64_t TargetGrid::find(Key key) const {
    const auto at = std::lower_bound(keys_.begin(), keys_.end(), key);
    return at != keys_.end() && *at == key ? at - keys_.begin() : -1;
}

std::pair<std::int64_t, double> TargetGrid::nearest(const Vec3d& query, std::span<const double> points,
                                                    double radius) const {
    double best = kInf;
    std::int64_t best_index = -1;
    if (keys_.empty()) return {best_index, best};
    const std::int64_t base[3] = {static_cast<std::int64_t>(std::floor(query[0] / cell_)),
                                  static_cast<std::int64_t>(std::floor(query[1] / cell_)),
                                  static_cast<std::int64_t>(std::floor(query[2] / cell_))};
    // The result is the minimum of (squared distance, index) over the points within `radius`, so
    // the visiting order does not matter and a neighbour cell farther than `radius` (plus a margin
    // far above the rounding of the bound) cannot change it. gap[a][d + 1]^2 bounds the squared
    // distance along axis a to the cells at offset d.
    const double reach = radius + 1e-9;
    double gap[3][3];
    for (int a = 0; a < 3; ++a) {
        const double below = query[a] - static_cast<double>(base[a]) * cell_;
        const double above = static_cast<double>(base[a] + 1) * cell_ - query[a];
        gap[a][0] = below > 0.0 ? below * below : 0.0;
        gap[a][1] = 0.0;
        gap[a][2] = above > 0.0 ? above * above : 0.0;
    }
    const double reach2 = reach * reach;
    // The k cells of one (i, j) column are consecutive keys: one search per column.
    for (std::int64_t di = -1; di <= 1; ++di) {
        for (std::int64_t dj = -1; dj <= 1; ++dj) {
            const double column = gap[0][di + 1] + gap[1][dj + 1];
            if (column > reach2) continue;
            const std::int64_t low_k = column + gap[2][0] > reach2 ? base[2] : base[2] - 1;
            const std::int64_t high_k = column + gap[2][2] > reach2 ? base[2] : base[2] + 1;
            const Key low = pack(base[0] + di, base[1] + dj, low_k);
            const Key high = low + (high_k - low_k);
            for (auto at = std::lower_bound(keys_.begin(), keys_.end(), low); at != keys_.end() && *at <= high;
                 ++at) {
                const auto slot = static_cast<std::size_t>(at - keys_.begin());
                for (std::size_t m = start_[slot]; m < start_[slot] + count_[slot]; ++m) {
                    const std::size_t target = indices_[m];
                    const double dx = query[0] - points[3 * target];
                    const double dy = query[1] - points[3 * target + 1];
                    const double dz = query[2] - points[3 * target + 2];
                    const double d2 = (dx * dx + dy * dy) + dz * dz;
                    const auto index = static_cast<std::int64_t>(target);
                    if (d2 < best || (d2 == best && index < best_index)) {
                        best = d2;
                        best_index = index;
                    }
                }
            }
        }
    }
    if (best > radius * radius) return {-1, kInf};
    return {best_index, best};
}

// MARK: Registration cloud

void RegistrationBuilder::add(std::span<const float> xyz, std::optional<double> max_range) {
    const std::vector<float> points = kept_points(xyz, range_keep(xyz, max_range));
    if (points.empty()) return;
    const Keys keys = float_cells(points, voxel_m_);
    const std::vector<std::size_t> first = first_per_cell(points, voxel_m_);
    // Merge by key; an old point beats a new one in its cell.
    Keys merged_keys;
    std::vector<float> merged_points;
    merged_keys.reserve(keys_.size() + first.size());
    merged_points.reserve(points_.size() + 3 * first.size());
    std::size_t a = 0, b = 0;
    while (a < keys_.size() || b < first.size()) {
        const bool take_old = b == first.size() || (a < keys_.size() && keys_[a] <= keys[first[b]]);
        if (take_old) {
            if (b < first.size() && keys_[a] == keys[first[b]]) ++b;
            merged_keys.push_back(keys_[a]);
            merged_points.insert(merged_points.end(), points_.begin() + 3 * a, points_.begin() + 3 * a + 3);
            ++a;
        } else {
            merged_keys.push_back(keys[first[b]]);
            merged_points.insert(merged_points.end(), points.begin() + 3 * first[b], points.begin() + 3 * first[b] + 3);
            ++b;
        }
    }
    keys_ = std::move(merged_keys);
    points_ = std::move(merged_points);
}

RegistrationCloud RegistrationBuilder::finish(const Keys& reference_occupied, double voxel_size,
                                              std::size_t limit) const {
    std::vector<float> kept;
    for (std::size_t n = 0; n < keys_.size(); ++n) {
        const Vec3d p{points_[3 * n], points_[3 * n + 1], points_[3 * n + 2]};
        if (std::binary_search(reference_occupied.begin(), reference_occupied.end(), voxel_key(p, voxel_size))) {
            kept.insert(kept.end(), points_.begin() + 3 * n, points_.begin() + 3 * n + 3);
        }
    }
    std::vector<float> points;
    for (const std::size_t n : decimate(kept.size() / 3, limit)) {
        points.insert(points.end(), kept.begin() + 3 * n, kept.begin() + 3 * n + 3);
    }
    return RegistrationCloud(std::move(points), voxel_m_);
}

namespace {

// 3 per point; NaN where a point has fewer than kNormalMinPoints sampled neighbours within
// kNormalRadiusM (itself included) or its neighbourhood is not planar.
std::vector<double> cloud_normals(const std::vector<double>& points) {
    const std::size_t count = points.size() / 3;
    std::vector<double> normals(points.size(), std::numeric_limits<double>::quiet_NaN());
    if (count) {
        const double radius = kNormalRadiusM;
        const TargetGrid grid(points, radius, kGridCap);
        const double r2 = radius * radius;
#pragma omp parallel for schedule(dynamic, 1024) if (count >= 4096)
        for (std::size_t n = 0; n < count; ++n) {
            const Vec3d p = point(points, n);
            const std::int64_t bi = static_cast<std::int64_t>(std::floor(p[0] / radius));
            const std::int64_t bj = static_cast<std::int64_t>(std::floor(p[1] / radius));
            const std::int64_t bk = static_cast<std::int64_t>(std::floor(p[2] / radius));
            double k = 0.0;
            std::array<double, 3> s{};
            std::array<double, 6> ss{};  // xx xy xz yy yz zz
            // guard.py's offset and slot order, so each sum adds in the oracle's order.
            for (std::int64_t di = -1; di <= 1; ++di) {
                for (std::int64_t dj = -1; dj <= 1; ++dj) {
                    for (std::int64_t dk = -1; dk <= 1; ++dk) {
                        const std::int64_t slot = grid.find(pack(bi + di, bj + dj, bk + dk));
                        if (slot < 0) continue;
                        for (const std::size_t m : grid.members(static_cast<std::size_t>(slot))) {
                            const Vec3d o = point(points, m);
                            const double dx = p[0] - o[0], dy = p[1] - o[1], dz = p[2] - o[2];
                            if (!((dx * dx + dy * dy) + dz * dz <= r2)) continue;
                            k += 1.0;
                            for (int a = 0; a < 3; ++a) s[a] += o[a];
                            ss[0] += o[0] * o[0];
                            ss[1] += o[0] * o[1];
                            ss[2] += o[0] * o[2];
                            ss[3] += o[1] * o[1];
                            ss[4] += o[1] * o[2];
                            ss[5] += o[2] * o[2];
                        }
                    }
                }
            }
            if (!(k >= static_cast<double>(kNormalMinPoints))) continue;
            const std::array<double, 3> mean{s[0] / k, s[1] / k, s[2] / k};
            std::array<double, 6> c{};
            for (int a = 0; a < 6; ++a) c[a] = ss[a] / k;
            std::array<double, 9> cov{};
            constexpr int kPairs[6][3] = {{0, 0, 0}, {0, 1, 1}, {0, 2, 2}, {1, 1, 3}, {1, 2, 4}, {2, 2, 5}};
            for (const auto& [i, j, slot] : kPairs) cov[3 * i + j] = cov[3 * j + i] = c[slot] - mean[i] * mean[j];
            std::array<double, 9> v{};
            std::vector<double> values;
            std::vector<std::vector<double>> vectors;
            jacobi(3, cov, v, values, vectors);
            if (values[0] <= kNormalPlanarity * values[1]) {
                for (int a = 0; a < 3; ++a) normals[3 * n + a] = vectors[0][a];
            }
        }
    }
    return normals;
}

}  // namespace

RegistrationCloud::RegistrationCloud(std::vector<float> points, double voxel_m)
    : points_(std::move(points)), points64_(points_.begin(), points_.end()), voxel_m_(voxel_m) {
    points_.resize(points_.size() / 3 * 3);
    points64_.resize(points_.size());
    grids_.reserve(kGridLevels.size());
    for (const double level : kGridLevels) grids_.emplace_back(points64_, level, kGridCap);
    normals_ = cloud_normals(points64_);
}

const TargetGrid& RegistrationCloud::grid(double radius) const {
    for (std::size_t n = 0; n < kGridLevels.size(); ++n) {
        if (kGridLevels[n] >= radius) return grids_[n];
    }
    return grids_.back();
}

// MARK: Alignment

std::optional<Pose> horn(std::span<const double> source, std::span<const double> target,
                         std::span<const double> weights) {
    const std::size_t count = weights.size();
    double total = 0.0;
    for (const double w : weights) total += w;
    if (!(total > 0.0)) return std::nullopt;
    Vec3d ps{}, qs{};
    for (std::size_t n = 0; n < count; ++n) {
        for (int i = 0; i < 3; ++i) {
            ps[i] += weights[n] * source[3 * n + i];
            qs[i] += weights[n] * target[3 * n + i];
        }
    }
    for (int i = 0; i < 3; ++i) {
        ps[i] /= total;
        qs[i] /= total;
    }
    std::array<double, 9> S{};
    for (std::size_t n = 0; n < count; ++n) {
        const Vec3d a{source[3 * n] - ps[0], source[3 * n + 1] - ps[1], source[3 * n + 2] - ps[2]};
        const Vec3d b{target[3 * n] - qs[0], target[3 * n + 1] - qs[1], target[3 * n + 2] - qs[2]};
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) S[3 * i + j] += weights[n] * a[i] * b[j];
        }
    }
    const double xx = S[0], xy = S[1], xz = S[2], yx = S[3], yy = S[4], yz = S[5], zx = S[6], zy = S[7], zz = S[8];
    const std::vector<std::vector<double>> N = {{xx + yy + zz, yz - zy, zx - xz, xy - yx},
                                                {yz - zy, xx - yy - zz, xy + yx, zx + xz},
                                                {zx - xz, xy + yx, -xx + yy - zz, yz + zy},
                                                {xy - yx, zx + xz, yz + zy, -xx - yy + zz}};
    const Eigen eigen = jacobi_eigh(N);
    const std::vector<double>& q = eigen.vectors.back();
    const double w = q[0], x = q[1], y = q[2], z = q[3];
    const double norm = std::sqrt(((w * w + x * x) + y * y) + z * z);
    const Mat3 R = quaternion_rotation(w / norm, x / norm, y / norm, z / norm);
    const Vec3d Rp = matvec(R, ps);
    return Pose{R, {qs[0] - Rp[0], qs[1] - Rp[1], qs[2] - Rp[2]}};
}

double median_range(std::span<const double> source) {
    const std::size_t count = source.size() / 3;
    if (!count) return 0.0;
    std::vector<double> ranges(count);
    for (std::size_t n = 0; n < count; ++n) {
        const double x = source[3 * n], y = source[3 * n + 1], z = source[3 * n + 2];
        ranges[n] = std::sqrt((x * x + y * y) + z * z);
    }
    std::sort(ranges.begin(), ranges.end());
    return ranges[count / 2];
}

std::optional<Estimate> align(std::span<const double> source, const RegistrationCloud& cloud, const Pose& seed,
                              double sigma0) {
    const std::size_t count = source.size() / 3;
    if (!count || !cloud.size()) return std::nullopt;
    const std::vector<double>& targets = cloud.points64();
    Pose pose = seed;
    double tau = std::min(kTauMaxM, std::max(kTauFirstMinM, 3.0 * sigma0));
    const double stop_cos = std::cos(to_radians(kStepStopDeg));
    std::vector<std::int64_t> match;
    std::vector<double> d2, moved_used, target_used, weights;
    for (int stage = 0; stage < kMaxStages; ++stage) {
        const TargetGrid& grid = cloud.grid(tau);
        const double kappa2 = (tau / 3.0) * (tau / 3.0);
        for (int iteration = 0; iteration < kMaxIterations; ++iteration) {
            const std::vector<double> moved = apply_all(pose, source);
            nearest_all(grid, moved, targets, tau, match, d2);
            moved_used.clear();
            target_used.clear();
            weights.clear();
            for (std::size_t n = 0; n < count; ++n) {
                if (match[n] < 0) continue;
                const auto t = static_cast<std::size_t>(match[n]);
                moved_used.insert(moved_used.end(), {moved[3 * n], moved[3 * n + 1], moved[3 * n + 2]});
                target_used.insert(target_used.end(), {targets[3 * t], targets[3 * t + 1], targets[3 * t + 2]});
                weights.push_back(kappa2 / ((kappa2 + d2[n]) * (kappa2 + d2[n])));
            }
            if (weights.size() < 3) return std::nullopt;
            const std::optional<Pose> step = horn(moved_used, target_used, weights);
            if (!step) return std::nullopt;
            pose = step->compose(pose);
            if (step->distance_m() < kStepStopM && step->cos_angle() >= stop_cos) break;
        }
        nearest_all(grid, apply_all(pose, source), targets, tau, match, d2);
        std::size_t used = 0;
        double sum = 0.0;
        for (std::size_t n = 0; n < count; ++n) {
            if (match[n] < 0) continue;
            ++used;
            sum += d2[n];
        }
        if (used < 3) return std::nullopt;
        const double shrunk = std::max(kTauMinM, 3.0 * std::sqrt(sum / static_cast<double>(used)));
        if (shrunk >= 0.95 * tau) break;
        tau = shrunk;
        if (tau <= kTauMinM) break;
    }
    const double scale = median_range(source);
    if (!(scale > 0.0)) return std::nullopt;
    const TargetGrid& grid = cloud.grid(tau);
    for (int iteration = 0; iteration < kFineIterations; ++iteration) {
        const std::vector<double> moved = apply_all(pose, source);
        nearest_all(grid, moved, targets, tau, match, d2);
        const PlaneSystem system = plane_system(moved, pose, cloud, match, tau, scale);
        if (system.count < 6) return std::nullopt;
        const Pose step = plane_step(system, pose, scale);
        pose = step.compose(pose);
        if (step.distance_m() < kStepStopM && step.cos_angle() >= stop_cos) break;
    }
    const std::vector<double> moved = apply_all(pose, source);
    nearest_all(grid, moved, targets, tau, match, d2);
    const PlaneSystem system = plane_system(moved, pose, cloud, match, tau, scale);
    if (system.count < 6) return std::nullopt;
    const Eigen eigen = jacobi_eigh(system.H);
    nearest_all(cloud.grid(kInlierRadiusM), moved, targets, kInlierRadiusM, match, d2);
    std::size_t near = 0;
    for (const std::int64_t index : match) near += index >= 0 ? 1 : 0;
    Estimate estimate;
    estimate.pose = pose;
    estimate.inlier_frac = static_cast<double>(near) / static_cast<double>(count);
    estimate.rms_m = std::sqrt(system.squares / static_cast<double>(system.count));
    estimate.observe = std::max(0.0, eigen.values[0]) / system.total;
    estimate.correspondences = system.count;
    estimate.tau_m = tau;
    std::copy(eigen.vectors[0].begin(), eigen.vectors[0].end(), estimate.weakest.begin());
    return estimate;
}

std::int64_t overlap(std::span<const double> source, const Pose& pose, const Keys& reference_occupied,
                     double voxel_size) {
    return overlap_in(source, pose, Membership(reference_occupied), voxel_size);
}

std::pair<Pose, double> sweep_seed(std::span<const double> source, const Pose& base, const Vec3d& shift,
                                   const Keys& reference_occupied, double voxel_size, double max_deg,
                                   const Vec3d& axis) {
    const Membership reference(reference_occupied);
    std::optional<std::int64_t> best_score;
    Pose best_pose;
    double best_psi = 0.0;
    const auto steps = static_cast<std::int64_t>(std::floor(max_deg / kSweepStepDeg));
    for (std::int64_t magnitude = 0; magnitude <= steps; ++magnitude) {
        std::vector<double> angles{0.0};
        if (magnitude) angles = {static_cast<double>(-magnitude) * kSweepStepDeg, static_cast<double>(magnitude) * kSweepStepDeg};
        for (const double psi : angles) {
            const Pose rotated = base.compose(Pose{rotation_about(axis, psi), {0.0, 0.0, 0.0}});
            for (const Vec3d& v : {shift, Vec3d{0.0, 0.0, 0.0}}) {
                const Pose candidate{rotated.R, {rotated.t[0] + v[0], rotated.t[1] + v[1], rotated.t[2] + v[2]}};
                const std::int64_t score = overlap_in(source, candidate, reference, voxel_size);
                if (!best_score || score > *best_score) {
                    best_score = score;
                    best_pose = candidate;
                    best_psi = psi;
                }
            }
        }
    }
    return {best_pose, best_psi};
}

}  // namespace room_memory::rig
