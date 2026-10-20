"""Both public converters score identical synthetic evidence identically."""

import json

import numpy as np
import pytest

from spatio.series.cli import main as series_main
from spatio.series.validate import load_walk_result
from spatio.walkcheck.synthetic import four_wall_room, vertical_wall
from spatio.walkday import result
from spatio.walkday.tests.test_result import make_session, quiet
from spatio.wall_scoring import WallScoringError, patch_provenance, score_walls


@pytest.mark.parametrize("report", [[], {"patches": None}, {"patches": [None]}])
def test_malformed_patch_report_is_rejected(report):
    with pytest.raises(WallScoringError, match="wallpatch"):
        patch_provenance("preselected", None, None, report, None)


@pytest.mark.parametrize("gate", [0, -1, float("nan"), float("inf"), True])
def test_invalid_wall_gate_is_rejected(gate):
    with pytest.raises(WallScoringError, match="wall gate"):
        score_walls({}, None, gate, None)


@pytest.mark.parametrize("case,expected", [
    ("sharp", "PASS"), ("other_walk", "PASS"), ("return_fail", "FAIL"),
    ("gap", "NOT_EVALUABLE"), ("pillar", "FAIL"),
    ("no_report", "NOT_EVALUABLE"), ("empty", "NOT_EVALUABLE"),
    ("auto", "NOT_EVALUABLE"), ("no_spec", "NOT_EVALUABLE"),
    ("one_verified", "NOT_EVALUABLE"), ("zero_verified", "NOT_EVALUABLE"),
    ("zero_verified_failed", "NOT_EVALUABLE"),
    ("unknown_patch", "NOT_EVALUABLE"), ("duplicate", "NOT_EVALUABLE"),
    ("box_mismatch", "NOT_EVALUABLE"), ("missing_gate", "NOT_EVALUABLE"),
    ("undetermined", "NOT_EVALUABLE"), ("whole_missing", "NOT_EVALUABLE"),
    ("whole_p95_fail", "FAIL"), ("patch_p95_fail", "FAIL"),
    ("patch_separation_fail", "FAIL"), ("patch_doubled", "FAIL"),
    ("gate_boundary", "PASS"), ("explicit_gate", "FAIL"),
])
def test_converter_wall_parity(tmp_path, case, expected):
    xyz = four_wall_room(seed=17 if case == "other_walk" else 3)
    if case == "pillar":
        rng = np.random.default_rng(2)
        pillar = [vertical_wall(2.25, "x", 1500, 1.75, 2.25, 0, 2.4, .004, rng),
                  vertical_wall(2.75, "x", 1500, 1.75, 2.25, 0, 2.4, .004, rng),
                  vertical_wall(1.75, "y", 1500, 2.25, 2.75, 0, 2.4, .004, rng),
                  vertical_wall(2.25, "y", 1500, 2.25, 2.75, 0, 2.4, .004, rng)]
        xyz = np.vstack([xyz, *pillar])
    boxes = [
        {"center_m": [0, 2, 1.2], "normal_hint": [1, 0, 0], "half_extent_m": [.2, 1, 1]},
        {"center_m": [2.5, 0, 1.2], "normal_hint": [0, 1, 0], "half_extent_m": [1, .2, 1]},
    ]
    offset = .15 if case == "return_fail" else (.07 if case == "other_walk" else .05)
    session = make_session(tmp_path, case, xyz=xyz, offset=(offset, 0, 0),
                           boxes=boxes, gap=case == "gap")
    wc_path = session / "walkcheck.json"
    wp_path = session / "wallpatch" / "wallpatch.json"
    wc = json.loads(wc_path.read_text())
    wp = json.loads(wp_path.read_text())
    if case == "pillar":
        assert wc["result"] == "PASS"
        assert wc["walls"]["doubled_wall"] is True
        assert wc["walls"]["max_revisit_separation_cm"] > 45
        assert all(p["doubled"] == "false" for p in wp["patches"])
    if case == "empty":
        wp["patches"] = []
    elif case == "one_verified":
        wp["patches"].pop()
    elif case == "zero_verified":
        for p in wp["patches"]:
            p["doubled"] = "undetermined"
    elif case == "zero_verified_failed":
        for p in wp["patches"]:
            p["doubled"] = "true"
            p["gate_separation_pass"] = False
            p["separation_m"] = .08
    elif case == "unknown_patch":
        wp["patches"][1]["name"] = "not preselected"
    elif case == "duplicate":
        wp["patches"].append(wp["patches"][1].copy())
    elif case == "box_mismatch":
        wp["patches"][1]["box_map_frame"]["center_m"][0] += .5
    elif case == "missing_gate":
        del wp["patches"][1]["gate_p95_pass"]
    elif case == "undetermined":
        wp["patches"][1]["doubled"] = "undetermined"
    elif case == "whole_missing":
        wc["walls"]["status"] = "NOT-EVALUABLE"
    elif case in ("whole_p95_fail", "explicit_gate", "gate_boundary"):
        wc["walls"]["p95_point_to_plane_cm"] = 5 if case == "gate_boundary" else 6
    elif case == "patch_p95_fail":
        wp["patches"][1]["gate_p95_pass"] = False
        wp["patches"][1]["residuals"]["p95_abs_m"] = .06
    elif case == "patch_separation_fail":
        wp["patches"][1]["gate_separation_pass"] = False
        wp["patches"][1]["separation_m"] = .06
    elif case == "patch_doubled":
        wp["patches"][1]["doubled"] = "true"
        wp["patches"][1]["separation_m"] = .08
    wc_path.write_text(json.dumps(wc))
    wp_path.write_text(json.dumps(wp))
    if case == "no_report":
        wp_path.unlink()
    gate = .04 if case == "explicit_gate" else .05
    origin = "auto_suggested" if case == "auto" else "preselected"
    common = ["--patch-origin", origin, "--wall-gate-m", str(gate), "--wall-gate-state", "approved"]
    if case != "no_spec":
        common += ["--spec", str(session / "test-spec.json")]
    assert quiet(result.main, [str(session), *common]) == result.EXIT[expected]
    walkday = json.loads((session / "walk_result.json").read_text())
    metadata = json.loads((session / "manifest.json").read_text())
    metadata.update(recorded_at=walkday["recorded_at"], physical_reference=None)
    meta_path = session / "series-metadata.json"
    meta_path.write_text(json.dumps(metadata))
    output = session / "adapted.json"
    args = ["adapt", str(wc_path), "--metadata", str(meta_path), "--out", str(output), *common,
            "--patches", str(session / "wallpatch" / "patches.json")]
    if wp_path.exists():
        args += ["--wallpatch", str(wp_path)]
    assert quiet(series_main, args) == 0
    adapted = json.loads(output.read_text())
    assert walkday["status"] == adapted["status"] == expected
    assert walkday["reasons"] == adapted["reasons"]
    assert walkday["walls"] == adapted["walls"]
    assert walkday["wall_patches"] == adapted["wall_patches"]
    assert load_walk_result(str(output)).status == expected
    assert load_walk_result(str(session / "walk_result.json")).status == expected
    if case in ("no_report", "empty", "auto", "no_spec", "zero_verified", "zero_verified_failed"):
        assert not adapted["wall_patches"]["verified"]
        assert adapted["walls"]["doubled_wall_suspected"] is None
    if case == "zero_verified_failed":
        assert any("doubled wall" in reason for reason in adapted["reasons"])
