import numpy as np
import pytest

from spatio.walkcheck.defaults import PROPOSED_WALL_LABEL
from spatio.walkcheck.synthetic import doubled_wall, four_wall_room, sharp_wall
from spatio.walkcheck.walls import wall_sharpness


def test_sharp_wall_scores_better_than_a_doubled_wall():
    sharp = wall_sharpness(sharp_wall())
    doubled = wall_sharpness(doubled_wall(separation_m=0.10))
    assert sharp["status"] == "OK"
    assert doubled["status"] == "OK"
    assert sharp["p95_point_to_plane_cm"] < 2.0
    assert doubled["p95_point_to_plane_cm"] > 5.0
    assert sharp["p95_point_to_plane_cm"] < doubled["p95_point_to_plane_cm"]
    assert sharp["doubled_wall"] is False
    assert doubled["doubled_wall"] is True
    assert doubled["revisit_separation_cm"] > 5.0
    assert sharp["proposed_gate_result"] == "PASS"
    assert doubled["proposed_gate_result"] == "FAIL"
    assert sharp["proposed_gate_label"] == PROPOSED_WALL_LABEL
    assert doubled["proposed_gate_label"] == PROPOSED_WALL_LABEL


def test_clean_room_is_not_called_a_doubled_wall():
    room = wall_sharpness(four_wall_room())
    assert room["status"] == "OK"
    assert room["doubled_wall"] is False
    assert room["p95_point_to_plane_cm"] < 2.0
    assert room["proposed_gate_result"] == "PASS"


def test_one_doubled_wall_in_a_room_is_flagged():
    rng = np.random.default_rng(1)
    room = four_wall_room(count_per_wall=3000, seed=1)
    # A second sheet 8 cm in front of the x=0 wall.
    extra = room[:3000].copy()
    extra[:, 0] += 0.08
    extra[:, 0] += rng.normal(0.0, 0.003, len(extra))
    scored = wall_sharpness(np.vstack([room, extra]))
    assert scored["doubled_wall"] is True
    assert scored["p95_point_to_plane_cm"] > wall_sharpness(room)["p95_point_to_plane_cm"]


def test_shifted_doubled_sheet_cannot_pass():
    """Codex P2. A 15 cm double wall fails the gate at every cell alignment.

    Translating the cloud by 5 cm used to drop the mixed cells as
    non-planar. The leftover sliver was 1,057 of 8,000 points, p95 was
    1.67 cm, and the wall gate passed.
    """
    cloud = doubled_wall(separation_m=0.15)
    for shift_m in (0.0, 0.05, 0.10, 0.15, 0.20, 0.25, 0.30):
        scored = wall_sharpness(cloud + np.array([shift_m, 0.0, 0.0]))
        assert scored["status"] == "OK", shift_m
        assert scored["doubled_wall"] is True, shift_m
        assert scored["proposed_gate_result"] == "FAIL", shift_m
        assert scored["revisit_separation_cm"] is not None, shift_m
        assert scored["revisit_separation_cm"] > 5.0, shift_m
    sharp = wall_sharpness(sharp_wall() + np.array([0.05, 0.0, 0.0]))
    assert sharp["doubled_wall"] is False
    assert sharp["proposed_gate_result"] == "PASS"
    room = wall_sharpness(four_wall_room() + np.array([0.05, 0.0, 0.0]))
    assert room["doubled_wall"] is False
    assert room["proposed_gate_result"] == "PASS"


def _yaw(points: np.ndarray, deg: float) -> np.ndarray:
    """Rotate a cloud about Z. Element-wise, so it does not use matmul."""
    theta = np.deg2rad(deg)
    cosine = float(np.cos(theta))
    sine = float(np.sin(theta))
    rotated = np.empty_like(points)
    rotated[:, 0] = cosine * points[:, 0] - sine * points[:, 1]
    rotated[:, 1] = sine * points[:, 0] + cosine * points[:, 1]
    rotated[:, 2] = points[:, 2]
    return rotated


def test_rotated_doubled_sheet_cannot_pass():
    """Codex P2. A 20 cm double wall at 45 degrees about Z used to pass.

    Unrotated, the gate failed. Rotated onto the voxel diagonal, the
    headline kept 2,595 of 8,000 points, p95 was 3.86 cm, and
    doubled_wall was false. The cell normal was noise, so the thick
    cells were dropped and pairing never saw the sheets. A heading
    change must not turn that wall into a pass. Nearby headings and a
    5 cm shift are the same hole. A sharp wall and a clean room at
    that heading still pass.
    """
    cloud = doubled_wall(separation_m=0.20)
    for deg in (43.5, 45.0, 45.5, 49.0):
        scored = wall_sharpness(_yaw(cloud, deg))
        assert scored["status"] == "OK", deg
        assert scored["doubled_wall"] is True, deg
        assert scored["proposed_gate_result"] == "FAIL", deg
        assert scored["p95_point_to_plane_cm"] > 5.0, deg
        assert scored["revisit_separation_cm"] is not None, deg
        assert scored["revisit_separation_cm"] > 10.0, deg
    exact = wall_sharpness(_yaw(cloud, 45.0))
    # The bug's headline was a 2,595-point sliver at 3.86 cm.
    assert exact["planar_point_count"] > 4000
    assert exact["p95_point_to_plane_cm"] > 5.0
    shifted = wall_sharpness(_yaw(cloud, 45.0) + np.array([0.05, 0.0, 0.0]))
    assert shifted["doubled_wall"] is True
    assert shifted["proposed_gate_result"] == "FAIL"
    assert shifted["p95_point_to_plane_cm"] > 5.0
    assert shifted["revisit_separation_cm"] > 10.0
    sharp = wall_sharpness(_yaw(sharp_wall(), 45.0))
    assert sharp["doubled_wall"] is False
    assert sharp["proposed_gate_result"] == "PASS"
    assert sharp["p95_point_to_plane_cm"] < 2.0
    room = wall_sharpness(_yaw(four_wall_room(), 45.0))
    assert room["doubled_wall"] is False
    assert room["proposed_gate_result"] == "PASS"
    assert room["p95_point_to_plane_cm"] < 2.0


def test_wide_doubled_sheet_cannot_pass():
    """Codex P2. A doubled wall wider than 40 cm used to pass.

    wall_sharpness(doubled_wall(separation_m=0.45)) was PASS, doubled_wall
    was false, and p95 was 0.57 cm. The neighbor loop had measured the
    pair and the 40 cm cap then threw it away. Each sheet is a thin plane,
    so the residual stays under 5 cm and the gate used to pass. A wider
    gap, a shift, and a change of heading are the same hole. A sharp wall
    and a clean room still pass.
    """
    for separation_m in (0.45, 0.70, 1.20):
        for deg in (0.0, 45.0, 90.0):
            scored = wall_sharpness(_yaw(doubled_wall(separation_m=separation_m), deg))
            assert scored["status"] == "OK", (separation_m, deg)
            assert scored["doubled_wall"] is True, (separation_m, deg)
            assert scored["proposed_gate_result"] == "FAIL", (separation_m, deg)
            assert scored["revisit_separation_cm"] is not None, (separation_m, deg)
            assert scored["revisit_separation_cm"] > 40.0, (separation_m, deg)
    cloud = doubled_wall(separation_m=0.45)
    for shift_m in (0.05, 0.15, 0.30):
        scored = wall_sharpness(cloud + np.array([shift_m, 0.0, 0.0]))
        assert scored["doubled_wall"] is True, shift_m
        assert scored["proposed_gate_result"] == "FAIL", shift_m
        assert scored["revisit_separation_cm"] > 40.0, shift_m
    sharp = wall_sharpness(sharp_wall())
    assert sharp["doubled_wall"] is False
    assert sharp["proposed_gate_result"] == "PASS"
    room = wall_sharpness(four_wall_room())
    assert room["doubled_wall"] is False
    assert room["proposed_gate_result"] == "PASS"


def test_too_few_points_is_not_evaluable():
    tiny = np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]], dtype=np.float64)
    scored = wall_sharpness(tiny)
    assert scored["status"] == "NOT-EVALUABLE"
    assert scored["proposed_gate_result"] == "NOT-EVALUABLE"
    assert scored["doubled_wall"] is None
    assert scored["proposed_gate_label"] == PROPOSED_WALL_LABEL
