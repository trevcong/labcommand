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


def test_too_few_points_is_not_evaluable():
    tiny = np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]], dtype=np.float64)
    scored = wall_sharpness(tiny)
    assert scored["status"] == "NOT-EVALUABLE"
    assert scored["proposed_gate_result"] == "NOT-EVALUABLE"
    assert scored["doubled_wall"] is None
    assert scored["proposed_gate_label"] == PROPOSED_WALL_LABEL
