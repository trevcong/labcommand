// The rig-pose guard's geometry: poses, the per-window point transform, the registration cloud, and
// ICP against it. The C++ port of room_memory/guard.py; read its docstrings for the algorithms.
// No ROS dependency.
//
// Conventions (PLAN.md "Rig guard", both engines):
// - A channel's pose T = (R, t) maps a point in the sensor's current frame into its reference
//   frame: p_ref = R p + t. Composition is (A o B)(p) = A(B(p)).
// - A physical move of the rig is an increment D in the pre-move sensor frame, so the new pose is
//   T o D and the increment an estimate implies is T^-1 o estimate.
// - Every estimate, seed and stored pose is absolute; increments are only derived from them.
//
// The same algorithms run in the same order as the Python oracle (the same grids, tie rules,
// Jacobi sweeps and iteration limits), so the engines reach the same decisions; sums are not
// order-free, so their floats agree to rounding, not to the bit (PLAN.md v1, "Parity").
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <utility>
#include <vector>

#include "room_memory/keys.hpp"

namespace room_memory::rig {

using Mat3 = std::array<double, 9>;  // row-major
using Vec3d = std::array<double, 3>;

inline constexpr Mat3 kIdentity{1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};

// Target grids: one per cell size, each cell keeping at most kGridCap points (every k-th of the
// cell, in target order). A search with threshold tau uses the finest grid whose cell is at least
// tau and looks at the 27 cells around the query.
inline constexpr std::array<double, 5> kGridLevels{0.04, 0.08, 0.16, 0.32, 0.64};
inline constexpr std::size_t kGridCap = 20;
inline constexpr double kInlierRadiusM = 0.10;
inline constexpr int kMaxStages = 5;
inline constexpr int kMaxIterations = 10;
inline constexpr int kFineIterations = 10;
inline constexpr double kStepEigFloor = 1e-4;
inline constexpr double kStepStopM = 1e-4;
inline constexpr double kStepStopDeg = 1e-3;
inline constexpr double kTauMinM = 0.04;
inline constexpr double kTauMaxM = 0.5;
inline constexpr double kTauFirstMinM = 0.05;
inline constexpr double kNormalRadiusM = 0.15;
inline constexpr std::size_t kNormalMinPoints = 5;
inline constexpr double kNormalPlanarity = 0.2;
inline constexpr double kSweepStepDeg = 1.0;
inline constexpr int kJacobiSweeps = 50;

// p_ref = R p + t, binary64.
struct Pose {
    Mat3 R = kIdentity;
    Vec3d t{0.0, 0.0, 0.0};
    friend bool operator==(const Pose&, const Pose&) = default;

    // Exactly the identity (the transform pass is skipped, so bytes match an unguarded run).
    bool is_identity() const { return R == kIdentity && t == Vec3d{0.0, 0.0, 0.0}; }
    // this o other.
    Pose compose(const Pose& other) const;
    Pose inverse() const;
    // this^-1 o estimate: the move `estimate` implies from this pose, in this pose's frame.
    Pose increment(const Pose& estimate) const { return inverse().compose(estimate); }
    // cos of the rotation angle, from the trace, clamped to [-1, 1].
    double cos_angle() const;
    double angle_deg() const;
    double distance_m() const;
    // One binary64 point through the pose: ((r0 x + r1 y) + r2 z) + t per row.
    Vec3d apply(const Vec3d& p) const;
    // (x, y, z, w), w >= 0.
    std::array<double, 4> quaternion() const;
    // The RigPose pose field: tx ty tz qx qy qz qw (binary32).
    std::array<float, 7> rig_pose() const;
    // Twelve %.17g numbers (R row-major, then t), space-separated: what the rig state file stores.
    std::string text() const;
    // Throws std::invalid_argument unless `values` holds 12 finite numbers.
    static Pose from_values(std::span<const double> values);
};

// Rotation by `degrees` about the unit `axis` (Rodrigues; scalar libm cos and sin).
Mat3 rotation_about(const Vec3d& axis, double degrees);
// The rotation of the rotation vector (rx, ry, rz) (radians).
Mat3 rotation_vector(double rx, double ry, double rz);
// The rotation of the unit quaternion (w, x, y, z).
Mat3 quaternion_rotation(double w, double x, double y, double z);
// (x, y, z, w) of a rotation matrix (Shepperd's method), w >= 0, unit length.
std::array<double, 4> rotation_quaternion(const Mat3& R);
// Angle between two nonzero vectors, degrees.
double angle_between_deg(const Vec3d& a, const Vec3d& b);
// The smallest rotation taking direction a onto direction b.
Mat3 rotation_between(const Vec3d& a, const Vec3d& b);

// MARK: Window points (3 floats per point)

// Whether each point is finite and within max_range by the quantizer's binary32 rule
// (x*x + y*y) + z*z > limit*limit drops it. No max_range: finite only.
std::vector<std::uint8_t> range_keep(std::span<const float> xyz, std::optional<double> max_range);
// Binary32 points through `pose` in binary32: ((r0 x + r1 y) + r2 z) + t, no FMA.
std::vector<float> transform_points(std::span<const float> xyz, const Pose& pose);
// A window's points for the change engine: a copy at the identity; otherwise range-clipped in the
// sensor frame, then moved into the reference frame (the quantizer then has no range clip).
std::vector<float> corrected(std::span<const float> xyz, const Pose& pose, std::optional<double> max_range);
// Indices of the first point of each `size` cell (floor(p / size) of the binary64 point, in input
// order), ordered by cell key.
std::vector<std::size_t> first_per_cell(std::span<const float> xyz, double size);
// Every k-th of `count` items, k = ceil(count / limit), so at most `limit` remain.
std::vector<std::size_t> decimate(std::size_t count, std::size_t limit);
// The ICP source: finite points within max_range (sensor frame), the first per voxel_m cell in
// arrival order, ordered by cell key, then decimated to at most `limit`. 3 doubles per point.
std::vector<double> source_cloud(std::span<const float> xyz, std::optional<double> max_range, double voxel_m,
                                 std::size_t limit);
// The change engine's voxel key of a binary64 point: binary32 q = f32(p) * (f32(1) / f32(voxel)), floored.
Key voxel_key(const Vec3d& p, double voxel_size);

// MARK: Small symmetric eigenproblems

struct Eigen {
    std::vector<double> values;                // ascending
    std::vector<std::vector<double>> vectors;  // unit, one per value; each one's largest component positive
};
// Cyclic Jacobi rotations in a fixed order, at most kJacobiSweeps sweeps; ties in the value order
// go to the lower index; a largest-magnitude tie goes to the lower component.
Eigen jacobi_eigh(const std::vector<std::vector<double>>& matrix);

// MARK: Target grids

// A cell table over binary64 target points (3 per point): per cell (sorted keys) the start and
// count of its kept indices (every k-th of the cell's points in target order, at most `cap`).
class TargetGrid {
public:
    TargetGrid(std::span<const double> points, double cell, std::size_t cap);
    // The nearest kept target within `radius` of the query in the 27 cells around it (ties to the
    // lower target index): (index, squared distance), or (-1, inf).
    std::pair<std::int64_t, double> nearest(const Vec3d& query, std::span<const double> points, double radius) const;
    double cell() const { return cell_; }
    const Keys& keys() const { return keys_; }
    std::span<const std::size_t> members(std::size_t slot) const {
        return std::span<const std::size_t>(indices_).subspan(start_[slot], count_[slot]);
    }
    // The slot of a cell key, or -1.
    std::int64_t find(Key key) const;

private:
    double cell_;
    Keys keys_;
    std::vector<std::size_t> start_;
    std::vector<std::size_t> count_;
    std::vector<std::size_t> indices_;
};

// MARK: Registration cloud

class RegistrationCloud;

// Collects a sensor's real points during BUILDING: the first point (arrival order) in each voxel_m
// cell of the sensor frame, from every window that counts toward the reference.
class RegistrationBuilder {
public:
    explicit RegistrationBuilder(double voxel_m) : voxel_m_(voxel_m) {}
    void add(std::span<const float> xyz, std::optional<double> max_range);
    // The cloud: points whose change-engine voxel is in `reference_occupied`, decimated to `limit`.
    RegistrationCloud finish(const Keys& reference_occupied, double voxel_size, std::size_t limit) const;
    const std::vector<float>& points() const { return points_; }

private:
    double voxel_m_;
    Keys keys_;                 // sorted cell keys
    std::vector<float> points_;  // parallel to keys_, 3 floats each
};

// A sensor's ICP target: real points (binary32, 3 per point), their normals and the correspondence
// grids, all computed when the cloud is built or loaded, so no check pays for them.
class RegistrationCloud {
public:
    RegistrationCloud(std::vector<float> points, double voxel_m);
    std::size_t size() const { return points_.size() / 3; }
    const std::vector<float>& points() const { return points_; }
    const std::vector<double>& points64() const { return points64_; }
    double voxel_m() const { return voxel_m_; }
    // The finest grid whose cell is at least `radius` (the coarsest level beyond that).
    const TargetGrid& grid(double radius) const;
    // 3 per point; NaN where a point has fewer than kNormalMinPoints sampled neighbours within
    // kNormalRadiusM (itself included) or its neighbourhood is not planar.
    const std::vector<double>& normals() const { return normals_; }

private:
    std::vector<float> points_;
    std::vector<double> points64_;
    double voxel_m_;
    std::vector<TargetGrid> grids_;  // one per kGridLevels entry
    std::vector<double> normals_;
};

// MARK: Alignment

// The weighted least-squares rigid motion taking `source` onto `target` (Horn 1987), 3 doubles per
// point: target ~ R source + t. Empty without weight.
std::optional<Pose> horn(std::span<const double> source, std::span<const double> target, std::span<const double> weights);

// One ICP result: the absolute pose and its metrics. rms_m is the RMS point-to-plane residual of
// the final correspondences; observe is lambda_min(H) / sum w of their point-to-plane information
// matrix, weakest its eigenvector (translation x y z, then rotation as meters at the median range).
struct Estimate {
    Pose pose;
    double inlier_frac = 0.0;
    double rms_m = 0.0;
    double observe = 0.0;
    std::int64_t correspondences = 0;
    double tau_m = 0.0;
    std::array<double, 6> weakest{};
};

// The median distance of the source points (3 doubles each) from the sensor: element n / 2 of the sorted ranges.
double median_range(std::span<const double> source);
// ICP of the binary64 source (sensor frame) onto `cloud` from the absolute `seed` (guard.py align).
std::optional<Estimate> align(std::span<const double> source, const RegistrationCloud& cloud, const Pose& seed,
                              double sigma0);
// How many source points land, through `pose`, in a voxel of sorted `reference_occupied`.
std::int64_t overlap(std::span<const double> source, const Pose& pose, const Keys& reference_occupied, double voxel_size);
// The yaw sweep without a rotation prior: candidates (I, v) o base o (R_axis(psi), 0) for psi in
// whole degrees within +-max_deg and v in (shift, 0), scored by overlap; ties to the smaller |psi|,
// the negative one, then v = shift. Returns (seed, psi).
std::pair<Pose, double> sweep_seed(std::span<const double> source, const Pose& base, const Vec3d& shift,
                                   const Keys& reference_occupied, double voxel_size, double max_deg,
                                   const Vec3d& axis = {0.0, 0.0, 1.0});

}  // namespace room_memory::rig
