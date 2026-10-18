"""Synthetic and fixture tests for the offline JT128 ↔ Mid-360 extrinsic.

The runtime event-box frame bug is documented in docs/D48-DUAL-LIDAR.md and is not
exercised here: this suite does not import the node or the Swift app.
"""

import json
import math
from pathlib import Path

import numpy as np
import pytest

from tools.extrinsic.calibrate import (
    CHARACTERISTIC_LENGTH_M,
    SCHEMA,
    SolverSettings,
    SpatialHash,
    _observability,
    apply_rigid,
    calibrate_points,
    condition_ratio,
    inverse_rigid,
    load_init,
    main,
    nominal_mount,
    rotation_ypr,
)

ROOT = Path(__file__).resolve().parents[2]
SNAP_A = ROOT / "Tests" / "Fixtures" / "room" / "snapshots" / "a"
SNAP_B = ROOT / "Tests" / "Fixtures" / "room" / "snapshots" / "b"


def settings(**overrides):
    base = SolverSettings(min_range_m=0.0, max_range_m=30.0)
    values = {key: getattr(base, key) for key in base.__dataclass_fields__}
    values.update(overrides)
    return SolverSettings(**values)


def grid_plane(origin, axis_u, axis_v, u_span, v_span, step, shift):
    u_values = np.arange(u_span[0] + shift, u_span[1], step)
    v_values = np.arange(v_span[0] + shift * 0.5, v_span[1], step)
    uu, vv = np.meshgrid(u_values, v_values, indexing="ij")
    origin = np.asarray(origin, dtype=np.float64)
    axis_u = np.asarray(axis_u, dtype=np.float64)
    axis_v = np.asarray(axis_v, dtype=np.float64)
    return origin + uu.reshape(-1, 1) * axis_u + vv.reshape(-1, 1) * axis_v


def three_planes(shift, step=0.06, floor=((0.4, 3.2), (-2.4, 2.6)), wall_x=((0.2, 2.4), (0.15, 2.3)),
                 wall_y=((0.3, 2.8), (0.1, 2.2))):
    floor_pts = grid_plane((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0), floor[0], floor[1], step, shift)
    wall_x_pts = grid_plane((3.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0), wall_x[0], wall_x[1], step, shift)
    wall_y_pts = grid_plane((0.0, -2.0, 0.0), (1.0, 0.0, 0.0), (0.0, 0.0, 1.0), wall_y[0], wall_y[1], step, shift)
    return np.vstack([floor_pts, wall_x_pts, wall_y_pts])


def noisy(points, rng, sigma):
    return points + rng.normal(0.0, sigma, points.shape)


def child_from_parent(points, rotation, translation):
    rotation_inv, translation_inv = inverse_rigid(rotation, translation)
    return apply_rigid(points, rotation_inv, translation_inv)


def pose_error(result, rotation, translation):
    estimate = np.asarray(result["T_parent_child"], dtype=np.float64)
    got_r = estimate[:3, :3]
    got_t = estimate[:3, 3]
    delta_t = float(np.linalg.norm(got_t - translation))
    delta = got_r @ rotation.T
    cosine = float(np.clip((np.trace(delta) - 1.0) / 2.0, -1.0, 1.0))
    delta_deg = float(np.degrees(np.arccos(cosine)))
    return delta_t, delta_deg


def assert_proper(result):
    matrix = np.asarray(result["T_parent_child"], dtype=np.float64)
    rotation = matrix[:3, :3]
    assert matrix.shape == (4, 4)
    assert np.allclose(matrix[3], [0.0, 0.0, 0.0, 1.0])
    assert abs(np.linalg.det(rotation) - 1.0) < 1e-6
    assert np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-6)
    assert np.isfinite(matrix).all()


def test_nominal_mount_matches_the_swift_basis():
    rotation, translation = nominal_mount()
    # RigTransformsTests: cos(-86.5°) ≈ 0.0610485, sin(-86.5°) ≈ -0.9981348, roll π.
    got_x = rotation @ np.array([1.0, 0.0, 0.0])
    got_z = rotation @ np.array([0.0, 0.0, 1.0])
    assert got_x == pytest.approx([0.0610485, -0.9981348, 0.0], abs=1e-5)
    assert got_z == pytest.approx([0.0, 0.0, -1.0], abs=1e-8)
    assert translation == pytest.approx([0.0, 0.0, -0.110])
    assert abs(np.linalg.det(rotation) - 1.0) < 1e-9


def test_small_motion_recovers_a_known_transform():
    rng = np.random.default_rng(1)
    yaw, pitch, roll = np.radians(2.0), np.radians(-0.4), np.radians(0.6)
    rotation = rotation_ypr(yaw, pitch, roll)
    translation = np.array([0.012, -0.008, 0.014])
    parent = noisy(three_planes(0.0), rng, 0.003)
    child = noisy(child_from_parent(three_planes(0.02), rotation, translation), rng, 0.003)
    result = calibrate_points(
        parent, child, init=(np.eye(3), np.zeros(3)), init_source="matrix", settings=settings()
    )
    delta_t, delta_deg = pose_error(result, rotation, translation)
    assert delta_t < 0.005
    assert delta_deg < 0.1
    assert result["accepted"] is True
    assert result["observable"] is True
    assert result["residual_rms_m"] < 0.01
    assert result["inlier_fraction"] > 0.8
    assert result["metrics_before"]["inlier_radius_m"] == result["metrics_after"]["inlier_radius_m"] == 0.10
    assert result["metrics_after"]["residual_rms_m"] < result["metrics_before"]["residual_rms_m"]
    assert_proper(result)


def test_mount_scale_recovers_and_identity_is_rejected():
    rng = np.random.default_rng(2)
    nominal_r, nominal_t = nominal_mount()
    delta_r = rotation_ypr(np.radians(0.25), np.radians(-0.15), np.radians(0.2))
    delta_t = np.array([0.008, -0.006, 0.011])
    rotation = delta_r @ nominal_r
    translation = delta_r @ nominal_t + delta_t
    parent = noisy(three_planes(0.0), rng, 0.003)
    child = noisy(child_from_parent(three_planes(0.017), rotation, translation), rng, 0.003)
    seeded = calibrate_points(
        parent, child, init=(nominal_r, nominal_t), init_source="file", settings=settings()
    )
    delta_t, delta_deg = pose_error(seeded, rotation, translation)
    assert delta_t < 0.005
    assert delta_deg < 0.1
    assert seeded["accepted"] is True
    identity = calibrate_points(
        parent, child, init=(np.eye(3), np.zeros(3)), init_source="matrix", settings=settings()
    )
    assert identity["accepted"] is False
    assert identity["activation"]["eligible"] is False


def test_one_plane_is_unobservable_and_keeps_a_finite_seed():
    rng = np.random.default_rng(3)
    floor = grid_plane((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (-2.0, 2.0), (-2.0, 2.0), 0.06, 0.0)
    parent = noisy(floor, rng, 0.002)
    rotation = rotation_ypr(np.radians(1.0), 0.0, 0.0)
    translation = np.array([0.01, -0.01, 0.0])
    child = noisy(child_from_parent(
        grid_plane((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (-2.0, 2.0), (-2.0, 2.0), 0.06, 0.02),
        rotation, translation,
    ), rng, 0.002)
    seed = (np.eye(3), np.zeros(3))
    result = calibrate_points(parent, child, init=seed, init_source="matrix", settings=settings())
    assert result["observable"] is False
    assert result["accepted"] is False
    assert result["reject_reason"] == "unobservable"
    assert result["activation"]["eligible"] is False
    assert_proper(result)
    assert np.allclose(result["T_parent_child"], result["T_init"])


def test_two_wall_corridor_is_unobservable():
    rng = np.random.default_rng(4)

    def corridor(shift):
        left = grid_plane((-2.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0), (-3.0, 3.0), (0.0, 2.5), 0.07, shift)
        right = grid_plane((2.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0), (-3.0, 3.0), (0.0, 2.5), 0.07, shift)
        return np.vstack([left, right])

    rotation = rotation_ypr(0.0, np.radians(0.4), 0.0)
    translation = np.array([0.0, 0.01, 0.0])
    parent = noisy(corridor(0.0), rng, 0.001)
    child = noisy(child_from_parent(corridor(0.02), rotation, translation), rng, 0.001)
    result = calibrate_points(
        parent, child, init=(np.eye(3), np.zeros(3)), init_source="matrix", settings=settings()
    )
    assert result["observable"] is False
    assert result["accepted"] is False
    assert result["reject_reason"] == "unobservable"
    ratio = result["observability"]["ratio"]
    assert ratio is not None and ratio < result["observability"]["threshold"]
    assert_proper(result)


def _z_noise(points, rng, sigma=0.015):
    out = np.array(points, dtype=np.float64, copy=True)
    out[:, 2] += rng.normal(0.0, sigma, len(out))
    return out


def _along(points, direction, rng, sigma=0.015):
    direction = np.asarray(direction, dtype=np.float64)
    direction = direction / np.linalg.norm(direction)
    return points + rng.normal(0.0, sigma, len(points))[:, None] * direction


def _assert_noise_rejected(result):
    """The ratio can clear 1e-4. The noise floor is what has to reject the solve."""
    assert result["observable"] is False
    assert result["accepted"] is False
    assert result["reject_reason"] == "unobservable"
    assert result["activation"]["eligible"] is False
    assert_proper(result)
    assert np.allclose(result["T_parent_child"], result["T_init"])
    obs = result["observability"]
    assert obs["ratio"] is not None and obs["ratio"] >= obs["threshold"]
    assert obs["noise_floor"] > 0.0
    assert obs["lambda_min"] < obs["noise_margin"] * obs["noise_floor"]


def test_realistic_single_plane_is_unobservable():
    """Astra, PR #42 at 636a9a9: one plane, 1.5 cm of z noise, default solver.

    z = 2, x and y in [-1.5, 1.5) every 0.06 m. ``default_rng(2)`` draws the
    parent cloud, then the child. The seed is the identity. Unfixed, the ratio
    is about 6.6e-4, the RMS about 0.020 m, and the solve walks about 0.014 m.
    """
    xs = np.arange(-1.5, 1.5, 0.06)
    ys = np.arange(-1.5, 1.5, 0.06)
    xx, yy = np.meshgrid(xs, ys, indexing="ij")
    plane = np.stack([xx.ravel(), yy.ravel(), np.full(xx.size, 2.0)], axis=1)
    rng = np.random.default_rng(2)
    parent = _z_noise(plane, rng)
    child = _z_noise(plane, rng)
    result = calibrate_points(
        parent, child, init=(np.eye(3), np.zeros(3)), init_source="matrix", settings=SolverSettings(),
    )
    _assert_noise_rejected(result)


def test_two_parallel_planes_with_realistic_noise_are_unobservable():
    rng = np.random.default_rng(2)
    lower = grid_plane((0.0, 0.0, 2.0), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (-1.5, 1.5), (-1.5, 1.5), 0.06, 0.0)
    upper = grid_plane((0.0, 0.0, 3.2), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (-1.5, 1.5), (-1.2, 1.2), 0.06, 0.0)
    clean = np.vstack([lower, upper])
    parent = _z_noise(clean, rng)
    child = _z_noise(clean, rng)
    result = calibrate_points(
        parent, child, init=(np.eye(3), np.zeros(3)), init_source="matrix", settings=SolverSettings(),
    )
    _assert_noise_rejected(result)


def test_planes_sharing_an_axis_are_unobservable():
    """Two vertical planes. Every normal is horizontal, so translation along z is free."""
    rng = np.random.default_rng(2)
    wall_x = grid_plane((2.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0), (-1.5, 1.5), (0.2, 2.4), 0.06, 0.0)
    wall_y = grid_plane((0.0, 2.0, 0.0), (1.0, 0.0, 0.0), (0.0, 0.0, 1.0), (-1.5, 1.5), (0.2, 2.4), 0.06, 0.0)

    def cloud():
        return np.vstack([
            _along(wall_x, (1.0, 0.0, 0.0), rng),
            _along(wall_y, (0.0, 1.0, 0.0), rng),
        ])

    result = calibrate_points(
        cloud(), cloud(), init=(np.eye(3), np.zeros(3)), init_source="matrix", settings=SolverSettings(),
    )
    _assert_noise_rejected(result)


def test_three_planes_with_realistic_noise_stay_observable():
    rng = np.random.default_rng(2)
    yaw, pitch, roll = np.radians(1.0), np.radians(-0.3), np.radians(0.2)
    rotation = rotation_ypr(yaw, pitch, roll)
    translation = np.array([0.010, -0.008, 0.012])
    parent = noisy(three_planes(0.0), rng, 0.015)
    child = noisy(child_from_parent(three_planes(0.02), rotation, translation), rng, 0.015)
    result = calibrate_points(
        parent, child, init=(np.eye(3), np.zeros(3)), init_source="matrix", settings=settings()
    )
    delta_t, delta_deg = pose_error(result, rotation, translation)
    assert result["accepted"] is True
    assert result["observable"] is True
    assert result["activation"]["eligible"] is True
    assert delta_t < 0.01
    assert delta_deg < 0.25
    obs = result["observability"]
    assert obs["lambda_min"] >= obs["noise_margin"] * obs["noise_floor"]
    assert_proper(result)


def test_condition_ratio_is_invariant_when_rows_are_duplicated():
    rng = np.random.default_rng(5)
    jacobian = rng.normal(size=(80, 6))
    # Rotational columns already stand in for (p × n) / L.
    ratio, lam_min, lam_max = condition_ratio(jacobian)
    duplicated = np.vstack([jacobian, jacobian, jacobian, jacobian])
    ratio_dup, lam_min_dup, lam_max_dup = condition_ratio(duplicated)
    assert ratio_dup == pytest.approx(ratio, rel=1e-9)
    assert (lam_min_dup / lam_min) == pytest.approx(4.0, rel=1e-6)
    assert (lam_max_dup / lam_max) == pytest.approx(4.0, rel=1e-6)
    count = jacobian.shape[0]
    bad = (lam_min / count) / lam_max
    bad_dup = (lam_min_dup / duplicated.shape[0]) / lam_max_dup
    assert bad_dup == pytest.approx(bad / 4.0, rel=1e-6)
    both = lam_min / count
    both_dup = lam_min_dup / duplicated.shape[0]
    assert both_dup == pytest.approx(both, rel=1e-6)
    assert (lam_max_dup / duplicated.shape[0]) == pytest.approx(lam_max / count, rel=1e-6)
    # The noise floor is a sum over the same rows, so it scales with the copies
    # and the margin does not flip.
    variances = rng.random(count) * 1e-3 + 1e-6
    weights = rng.random(count) * 0.5 + 0.5
    once = _observability(jacobian, weights, variances, 1e-4, CHARACTERISTIC_LENGTH_M)
    copied = _observability(
        duplicated, np.tile(weights, 4), np.tile(variances, 4), 1e-4, CHARACTERISTIC_LENGTH_M
    )
    assert copied["noise_floor"] / once["noise_floor"] == pytest.approx(4.0, rel=1e-12)
    assert copied["lambda_min"] / copied["noise_floor"] == pytest.approx(
        once["lambda_min"] / once["noise_floor"], rel=1e-9
    )
    assert copied["observable"] is once["observable"]


def test_duplicated_samples_keep_the_same_observability_decision():
    rng = np.random.default_rng(6)
    rotation = rotation_ypr(np.radians(1.0), np.radians(-0.3), np.radians(0.2))
    translation = np.array([0.01, 0.0, -0.008])
    parent = noisy(three_planes(0.0, step=0.08), rng, 0.002)
    child = noisy(child_from_parent(three_planes(0.02, step=0.08), rotation, translation), rng, 0.002)
    # No voxel grid: a 5 cm downsample would collapse exact copies and hide the bug.
    loose = settings(voxel_m=0.0)
    once = calibrate_points(parent, child, init=(np.eye(3), np.zeros(3)), init_source="matrix", settings=loose)
    parent_dup = np.vstack([parent, parent, parent, parent])
    child_dup = np.vstack([child, child, child, child])
    duplicated = calibrate_points(
        parent_dup, child_dup, init=(np.eye(3), np.zeros(3)), init_source="matrix", settings=loose
    )
    assert once["observable"] is True
    assert duplicated["observable"] is True
    assert duplicated["accepted"] is True
    assert duplicated["observability"]["ratio"] == pytest.approx(once["observability"]["ratio"], rel=0.05)
    # Both eigenvalues were divided by the inlier count. The ratio of those
    # normalized values matches the raw ratio, so the decision does not depend
    # on how many times each sample was copied.
    for result in (once, duplicated):
        obs = result["observability"]
        assert obs["lambda_min_over_n"] / obs["lambda_max_over_n"] == pytest.approx(obs["ratio"], rel=1e-6)
        assert obs["characteristic_length_m"] == CHARACTERISTIC_LENGTH_M


def test_partial_overlap_and_unequal_visibility_and_outliers_recover():
    rng = np.random.default_rng(7)
    rotation = rotation_ypr(np.radians(1.2), np.radians(0.3), np.radians(-0.4))
    translation = np.array([0.01, -0.012, 0.006])
    parent = noisy(three_planes(0.0, step=0.07), rng, 0.003)
    # Child surfaces sit inside the parent's, so the overlap is real and partial.
    child_region = three_planes(
        0.02,
        step=0.07,
        floor=((1.0, 3.0), (-1.2, 2.2)),
        wall_x=((0.8, 2.2), (0.4, 2.0)),
        wall_y=((0.8, 2.4), (0.3, 1.8)),
    )
    child = noisy(child_from_parent(child_region, rotation, translation), rng, 0.003)
    partial = calibrate_points(
        parent, child, init=(np.eye(3), np.zeros(3)), init_source="matrix", settings=settings()
    )
    delta_t, delta_deg = pose_error(partial, rotation, translation)
    assert partial["accepted"] is True
    assert delta_t < 0.01
    assert delta_deg < 0.25

    # Both clouds see every plane. The parent is dense on the floor and thinner on the walls.
    dense_floor = grid_plane((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.2, 3.2), (-2.2, 2.4), 0.05, 0.0)
    thin_walls = np.vstack([
        grid_plane((3.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0), (0.0, 2.0), (0.2, 2.0), 0.08, 0.0),
        grid_plane((0.0, -2.0, 0.0), (1.0, 0.0, 0.0), (0.0, 0.0, 1.0), (0.2, 2.6), (0.2, 2.0), 0.08, 0.0),
    ])
    parent_unequal = noisy(np.vstack([dense_floor, thin_walls]), rng, 0.003)
    child_floor = grid_plane((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.2, 3.2), (-2.2, 2.4), 0.09, 0.02)
    child_walls = np.vstack([
        grid_plane((3.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0), (0.0, 2.0), (0.2, 2.0), 0.05, 0.01),
        grid_plane((0.0, -2.0, 0.0), (1.0, 0.0, 0.0), (0.0, 0.0, 1.0), (0.2, 2.6), (0.2, 2.0), 0.05, 0.01),
    ])
    child_unequal = noisy(child_from_parent(np.vstack([child_floor, child_walls]), rotation, translation), rng, 0.003)
    unequal = calibrate_points(
        parent_unequal, child_unequal, init=(np.eye(3), np.zeros(3)), init_source="matrix", settings=settings()
    )
    delta_t, delta_deg = pose_error(unequal, rotation, translation)
    assert unequal["accepted"] is True
    assert delta_t < 0.01
    assert delta_deg < 0.25

    outliers = rng.uniform(-6.0, 6.0, size=(child.shape[0] // 5, 3))
    contaminated = np.vstack([child, outliers])
    robust = calibrate_points(
        parent, contaminated, init=(np.eye(3), np.zeros(3)), init_source="matrix", settings=settings()
    )
    delta_t, delta_deg = pose_error(robust, rotation, translation)
    assert robust["accepted"] is True
    assert delta_t < 0.01
    assert delta_deg < 0.25


def test_no_overlap_is_rejected():
    rng = np.random.default_rng(8)
    parent = noisy(
        grid_plane((3.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0), (0.0, 2.0), (0.2, 2.0), 0.06, 0.0),
        rng, 0.002,
    )
    other = grid_plane((-3.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0), (0.0, 2.0), (0.2, 2.0), 0.06, 0.01)
    rotation = rotation_ypr(np.radians(1.0), 0.0, 0.0)
    translation = np.array([0.01, 0.0, 0.0])
    child = noisy(child_from_parent(other, rotation, translation), rng, 0.002)
    result = calibrate_points(
        parent, child, init=(np.eye(3), np.zeros(3)), init_source="matrix", settings=settings()
    )
    assert result["accepted"] is False
    assert result["reject_reason"] == "insufficient_correspondences"
    assert result["activation"]["eligible"] is False


def test_hash_matches_brute_force_and_is_not_the_downsample_grid():
    rng = np.random.default_rng(9)
    points = rng.normal(scale=0.4, size=(70, 3))
    radius = 0.25
    spatial = SpatialHash(points, radius)
    for index in (0, 17, 41, 69):
        got = set(int(value) for value in spatial.indices_within(points[index], radius))
        distance = np.linalg.norm(points - points[index], axis=1)
        want = set(int(value) for value in np.nonzero(distance <= radius + 1e-12)[0])
        assert got == want
    nearest, dist = spatial.nearest_batch(points[:12], radius)
    for row, query in enumerate(points[:12]):
        distance = np.linalg.norm(points - query, axis=1)
        inside = np.nonzero(distance <= radius + 1e-12)[0]
        assert int(nearest[row]) == int(inside[np.argmin(distance[inside])])
        assert dist[row] == pytest.approx(float(distance[int(nearest[row])]), abs=1e-9)

    pair = np.array([[0.0, 0.0, 0.0], [0.12, 0.0, 0.0]])
    fine = SpatialHash(pair, 0.05)
    with pytest.raises(ValueError, match="27-cell"):
        fine.indices_within(pair[0], 0.20)
    coarse = SpatialHash(pair, 0.25)
    assert 1 in set(int(value) for value in coarse.indices_within(pair[0], 0.20))
    # search_cell_m of the solver is 0.25 m. The 5 cm figure is only the downsample voxel.
    assert settings().search_cell_m >= settings().normal_radius_m
    assert settings().search_cell_m > settings().voxel_m


def test_range_gate_keeps_an_object_inside_the_band():
    # A person at 2 m is inside 1.5–8 m. The range gate is not a static-structure test.
    person = np.array([[2.0, 0.0, 0.0], [1.0, 0.0, 0.0], [9.0, 0.0, 0.0], [0.2, 0.2, 0.2]])
    wall = grid_plane((3.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0), (-1.0, 1.0), (0.5, 2.0), 0.08, 0.0)
    cloud = np.vstack([person, wall])
    result = calibrate_points(
        cloud,
        cloud.copy(),
        init=(np.eye(3), np.zeros(3)),
        init_source="matrix",
        settings=settings(min_range_m=1.5, max_range_m=8.0, max_iterations=0),
    )
    assert result["source_points"] < len(cloud)
    assert result["iterations"] == 0
    assert result["reject_reason"] == "not_refined"
    assert result["accepted"] is False
    assert result["scene_precondition"] == "stationary_rig_and_static_scene"


def test_iteration_exhaustion_insufficient_points_and_normals():
    rng = np.random.default_rng(10)
    rotation = rotation_ypr(np.radians(2.0), np.radians(-0.5), 0.0)
    translation = np.array([0.02, -0.01, 0.0])
    parent = noisy(three_planes(0.0, step=0.08), rng, 0.002)
    child = noisy(child_from_parent(three_planes(0.02, step=0.08), rotation, translation), rng, 0.002)
    exhausted = calibrate_points(
        parent, child, init=(np.eye(3), np.zeros(3)), init_source="matrix",
        settings=settings(max_iterations=1),
    )
    assert exhausted["accepted"] is False
    assert exhausted["reject_reason"] == "iteration_exhaustion"
    assert exhausted["converged"] is False
    assert exhausted["iterations"] == 1
    assert_proper(exhausted)

    tiny = rng.normal(size=(12, 3))
    few = calibrate_points(tiny, tiny + 0.01, init=(np.eye(3), np.zeros(3)), init_source="matrix", settings=settings())
    assert few["accepted"] is False
    assert few["reject_reason"] == "insufficient_points"

    sparse = rng.uniform(-8.0, 8.0, size=(180, 3))
    bare = calibrate_points(
        sparse, sparse + np.array([0.02, 0.0, 0.0]), init=(np.eye(3), np.zeros(3)),
        init_source="matrix", settings=settings(),
    )
    assert bare["accepted"] is False
    assert bare["reject_reason"] == "insufficient_normals"


def test_json_schema_quaternion_and_init_reload(tmp_path):
    rng = np.random.default_rng(11)
    rotation = rotation_ypr(np.radians(1.5), np.radians(0.2), np.radians(-0.3))
    translation = np.array([0.01, 0.004, -0.008])
    parent = noisy(three_planes(0.0, step=0.08), rng, 0.002)
    child = noisy(child_from_parent(three_planes(0.015, step=0.08), rotation, translation), rng, 0.002)
    output = tmp_path / "extrinsic.json"
    code = main([
        "--parent-npy", _save(tmp_path / "parent.npy", parent),
        "--child-npy", _save(tmp_path / "child.npy", child),
        "--init-matrix", _save_matrix(tmp_path / "init.json", np.eye(3), np.zeros(3)),
        "--min-range", "0",
        "--max-range", "30",
        "-o", str(output),
    ])
    assert code == 0
    payload = json.loads(output.read_text())
    assert output.read_text().endswith("\n")
    assert payload["schema"] == SCHEMA
    assert payload["parent_frame"] == "hesai_lidar"
    assert payload["child_frame"] == "livox_frame"
    assert payload["convention"] == "p_parent = R * p_child + t"
    matrix = np.asarray(payload["T_parent_child"])
    quaternion = np.asarray(payload["quaternion_xyzw"])
    assert quaternion[3] >= 0.0
    assert np.linalg.norm(quaternion) == pytest.approx(1.0, abs=1e-8)
    stored = np.asarray(payload["translation_m"])
    assert stored == pytest.approx(matrix[:3, 3])
    assert abs(np.linalg.det(matrix[:3, :3]) - 1.0) < 1e-6
    loaded_r, loaded_t = load_init(output)
    assert np.allclose(loaded_r, matrix[:3, :3])
    assert np.allclose(loaded_t, matrix[:3, 3])
    # The written init matrix is the seed that was actually used, not a label.
    assert np.allclose(payload["T_init"], np.eye(4))
    assert "nominal_mount" in payload
    assert payload["solver"]["characteristic_length_m"] == CHARACTERISTIC_LENGTH_M
    assert payload["solver"]["search_cell_m"] >= payload["solver"]["normal_radius_m"]
    assert payload["solver"]["search_cell_m"] > payload["solver"]["voxel_m"]
    assert payload["room_level"]["status"] == "deferred"
    scored = tmp_path / "scored.json"
    score_code = main([
        "--parent-npy", str(tmp_path / "parent.npy"),
        "--child-npy", str(tmp_path / "child.npy"),
        "--init", str(output),
        "--min-range", "0",
        "--max-range", "30",
        "--max-iterations", "0",
        "-o", str(scored),
    ])
    assert score_code == 2
    frozen = json.loads(scored.read_text())
    assert frozen["iterations"] == 0
    assert frozen["accepted"] is False
    assert frozen["reject_reason"] == "not_refined"
    assert np.allclose(frozen["T_parent_child"], payload["T_parent_child"])
    assert frozen["metrics_before"] == frozen["metrics_after"]


def _save(path, points):
    np.save(path, points)
    return str(path)


def _save_matrix(path, rotation, translation):
    matrix = np.eye(4)
    matrix[:3, :3] = rotation
    matrix[:3, 3] = translation
    path.write_text(json.dumps({
        "schema": SCHEMA,
        "parent_frame": "hesai_lidar",
        "child_frame": "livox_frame",
        "convention": "p_parent = R * p_child + t",
        "T_parent_child": matrix.tolist(),
    }))
    return str(path)


@pytest.mark.parametrize("mutate, message", [
    ("schema", "schema"),
    ("frames", "frame"),
    ("convention", "convention"),
    ("bottom", "bottom row"),
    ("nan", "finite"),
    ("reflection", "rotation"),
    ("shear", "rotation"),
    ("translation", "translation_m"),
])
def test_malformed_init_is_rejected(tmp_path, mutate, message):
    rotation, translation = nominal_mount()
    path = Path(_save_matrix(tmp_path / "init.json", rotation, translation))
    payload = json.loads(path.read_text())
    if mutate == "schema":
        payload["schema"] = "other"
    elif mutate == "frames":
        payload["parent_frame"] = "livox_frame"
        payload["child_frame"] = "hesai_lidar"
    elif mutate == "convention":
        payload["convention"] = "p_child = R * p_parent + t"
    elif mutate == "bottom":
        payload["T_parent_child"][3] = [0.0, 0.0, 0.0, 2.0]
    elif mutate == "nan":
        payload["T_parent_child"][0][3] = float("nan")
    elif mutate == "reflection":
        payload["T_parent_child"][0][0] *= -1.0
    elif mutate == "shear":
        payload["T_parent_child"][0][1] += 0.2
    elif mutate == "translation":
        payload["translation_m"] = [1.0, 0.0, 0.0]
    path.write_text(json.dumps(payload))
    with pytest.raises(Exception, match=message):
        load_init(path)
    output = tmp_path / "out.json"
    code = main([
        "--parent-npy", _save(tmp_path / "p.npy", np.zeros((10, 3))),
        "--child-npy", _save(tmp_path / "c.npy", np.zeros((10, 3))),
        "--init", str(path),
        "-o", str(output),
    ])
    assert code == 1
    assert not output.exists()


def test_cli_rejects_a_bad_path(tmp_path):
    output = tmp_path / "out.json"
    code = main([str(tmp_path / "missing"), "-o", str(output)])
    assert code == 1
    assert not output.exists()


def test_one_plane_cli_writes_a_rejected_candidate(tmp_path):
    rng = np.random.default_rng(12)
    floor = grid_plane((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (-1.5, 1.5), (-1.5, 1.5), 0.07, 0.0)
    parent = noisy(floor, rng, 0.001)
    child = noisy(floor + np.array([0.01, 0.0, 0.0]), rng, 0.001)
    output = tmp_path / "rejected.json"
    code = main([
        "--parent-npy", _save(tmp_path / "parent.npy", parent),
        "--child-npy", _save(tmp_path / "child.npy", child),
        "--init-matrix", _save_matrix(tmp_path / "init.json", np.eye(3), np.zeros(3)),
        "--min-range", "0",
        "--max-range", "30",
        "-o", str(output),
    ])
    assert code == 2
    payload = json.loads(output.read_text())
    assert payload["accepted"] is False
    assert payload["activation"]["eligible"] is False
    assert payload["activation"]["authoritative_path"] == "~/room-memory/<room_id>/T_jt128_mid360.json"
    matrix = np.asarray(payload["T_parent_child"])
    assert np.isfinite(matrix).all()
    assert abs(np.linalg.det(matrix[:3, :3]) - 1.0) < 1e-6


def test_fixture_is_reproducible_and_snapshot_b_is_held_out(tmp_path):
    output_a = tmp_path / "a.json"
    output_a2 = tmp_path / "a2.json"
    output_b = tmp_path / "b.json"
    code_a = main([str(SNAP_A), "-o", str(output_a)])
    code_a2 = main([str(SNAP_A), "-o", str(output_a2)])
    assert code_a in (0, 2)
    assert code_a2 == code_a
    first = json.loads(output_a.read_text())
    second = json.loads(output_a2.read_text())
    assert first["parent_frame"] == "hesai_lidar"
    assert first["child_frame"] == "livox_frame"
    assert first["T_parent_child"] == second["T_parent_child"]
    assert first["observability"] == second["observability"]
    assert first["metrics_before"]["inlier_radius_m"] == 0.10
    assert first["metrics_after"]["inlier_radius_m"] == 0.10
    assert first["metrics_before"]["residual_trim_note"] == first["metrics_after"]["residual_trim_note"]
    assert math.isfinite(first["observability"]["ratio"]) or first["observability"]["ratio"] is None
    # No ground truth. Reproducibility and a frozen score on B are the claim.
    code_b = main([
        str(SNAP_B),
        "--init", str(output_a),
        "--max-iterations", "0",
        "-o", str(output_b),
    ])
    assert code_b == 2
    held = json.loads(output_b.read_text())
    assert held["iterations"] == 0
    assert held["reject_reason"] == "not_refined"
    assert held["accepted"] is False
    # Reloading the JSON and re-orthonormalizing the seed is not bit-identical.
    # Nothing was refined: the held-out matrix matches A's, and the two metric blocks match each other.
    assert np.allclose(held["T_parent_child"], first["T_parent_child"], atol=1e-8)
    assert held["metrics_before"] == held["metrics_after"]
    for metric in (held["metrics_after"]["inlier_fraction"],):
        assert 0.0 <= metric <= 1.0
    assert first["activation"]["eligible"] is first["accepted"]
    if not first["accepted"]:
        assert first["activation"]["eligible"] is False
