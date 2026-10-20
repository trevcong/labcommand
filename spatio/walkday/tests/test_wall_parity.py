"""Both public converters score identical synthetic evidence identically."""

import hashlib
import json

import numpy as np
import pytest

from spatio.series.cli import main as series_main
from spatio.series.validate import load_walk_result
from spatio.walkcheck.synthetic import four_wall_room, vertical_wall
from spatio.walkday import result
from spatio.walkday.tests.test_result import make_session, quiet
from spatio.wall_scoring import WallScoringError, patch_provenance, score_walls, wall_verdict
from spatio.walkday.tests.test_result import write_spec


@pytest.mark.parametrize("report", [[], {"patches": None}, {"patches": [None]}])
def test_malformed_patch_report_is_rejected(report):
    with pytest.raises(WallScoringError, match="wallpatch"):
        patch_provenance("preselected", None, None, report, None, gate_m=.05)


@pytest.mark.parametrize("gate", [0, -1, float("nan"), float("inf"), True])
def test_invalid_wall_gate_is_rejected(gate):
    with pytest.raises(WallScoringError, match="wall gate"):
        score_walls({}, None, gate, None, map_path=None)


@pytest.mark.parametrize("case,expected", [
    ("sharp", "PASS"), ("other_walk", "PASS"), ("return_fail", "FAIL"),
    # Round 7: the whole-map doubled flag is a warning again; pillar was FAIL (b1b7922).
    ("gap", "NOT_EVALUABLE"), ("pillar", "PASS"),
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
    ("strict_gate_patch_p95", "FAIL"), ("strict_gate_band", "NOT_EVALUABLE"),
    ("strict_gate_clean", "PASS"), ("guard_band_review", "NOT_EVALUABLE"),
    ("guard_band_p95", "NOT_EVALUABLE"), ("guard_band_edge", "PASS"),
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
    elif case.startswith("strict_gate"):
        # At a requested 1 cm gate the tool's 5 cm flags stay True; only the
        # measured numbers can show patch B is over the effective gate.
        wc["walls"]["p95_point_to_plane_cm"] = .485
        for p in wp["patches"]:
            p["residuals"].update(p95_abs_m=.005, core_band_m=.004, stepped_band_m=.004)
            assert p["gate_p95_pass"] is p["gate_separation_pass"] is True and p["doubled"] == "false"
        if case == "strict_gate_patch_p95":
            wp["patches"][1]["residuals"]["p95_abs_m"] = .01545
        elif case == "strict_gate_band":
            wp["patches"][1]["residuals"]["core_band_m"] = .012
    elif case.startswith("guard_band"):
        # Review repro numbers (sp-rv-m1final3): tool flags True at its 5 cm gate.
        b = {"guard_band_review": dict(p95_abs_m=.0329, core_band_m=.04888, stepped_band_m=.04963),
             "guard_band_p95": dict(p95_abs_m=.0450, core_band_m=.02, stepped_band_m=.02),
             "guard_band_edge": dict(p95_abs_m=.04, core_band_m=.04, stepped_band_m=.04)}[case]
        wp["patches"][1]["residuals"].update(b)
        assert wp["patches"][1]["gate_p95_pass"] is wp["patches"][1]["gate_separation_pass"] is True
    elif case == "patch_doubled":
        wp["patches"][1]["doubled"] = "true"
        wp["patches"][1]["separation_m"] = .08
    wc_path.write_text(json.dumps(wc))
    wp_path.write_text(json.dumps(wp))
    if case == "no_report":
        wp_path.unlink()
    gate = .04 if case == "explicit_gate" else .01 if case.startswith("strict_gate") else .05
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
    if case == "strict_gate_patch_p95":
        assert any("p95 1.5" in r and "effective gate 1 cm" in r for r in adapted["reasons"])
    if case in ("guard_band_review", "guard_band_p95"):
        assert any("pass limit 4 cm" in r for r in adapted["reasons"])
        assert adapted["wall_patches"]["verified"] == [wp["patches"][0]["name"]]
    if case == "pillar":
        assert adapted["walls"]["doubled_wall_suspected"] is False
    if case.startswith("strict_gate"):
        names = [p["name"] for p in wp["patches"]]
        expect = names if case == "strict_gate_clean" else names[:1]
        assert adapted["wall_patches"]["verified"] == expect


@pytest.mark.parametrize("case", [
    "cross_session", "changed_map", "missing_map", "map_directory",
    "missing_hash", "null_hash", "malformed_hash", "nonstring_hash",
])
def test_converter_rejects_unbound_wall_report(tmp_path, capsys, case):
    boxes = [
        {"center_m": [0, 2, 1.2], "normal_hint": [1, 0, 0], "half_extent_m": [.2, 1, 1]},
        {"center_m": [2.5, 0, 1.2], "normal_hint": [0, 1, 0], "half_extent_m": [1, .2, 1]},
    ]
    session = make_session(tmp_path, "target", xyz=four_wall_room(seed=3), boxes=boxes)
    spec = session / "test-spec.json"
    common = ["--patch-origin", "preselected", "--spec", str(spec),
              "--wall-gate-state", "approved"]
    assert quiet(result.main, [str(session), *common]) == 0
    original = json.loads((session / "walk_result.json").read_text())
    metadata = json.loads((session / "manifest.json").read_text())
    metadata.update(recorded_at=original["recorded_at"], physical_reference=None)
    meta_path = session / "series-metadata.json"
    meta_path.write_text(json.dumps(metadata))
    wp_path = session / "wallpatch" / "wallpatch.json"
    wp = json.loads(wp_path.read_text())
    adapt_wp_path = wp_path
    message = "wallpatch map_sha256"
    if case == "cross_session":
        other = make_session(tmp_path, "other", xyz=four_wall_room(seed=17), boxes=boxes)
        # Both walks pass with their own evidence and the exact same frozen spec.
        assert (other / "test-spec.json").read_bytes() == spec.read_bytes()
        assert quiet(result.main, [str(other), *common]) == 0
        adapt_wp_path = other / "wallpatch" / "wallpatch.json"
        foreign = json.loads(adapt_wp_path.read_text())
        assert foreign["map_sha256"] != wp["map_sha256"]
        wp = foreign
    elif case == "changed_map":
        with (session / "map.pcd").open("ab") as handle:
            handle.write(b"\n")
    elif case in ("missing_map", "map_directory"):
        (session / "map.pcd").unlink()
        if case == "map_directory":
            (session / "map.pcd").mkdir()
        message = "cannot read run map"
    elif case == "missing_hash":
        del wp["map_sha256"]
    else:
        wp["map_sha256"] = {"null_hash": None, "malformed_hash": "g" * 64,
                            "nonstring_hash": 123}[case]
    wp_path.write_text(json.dumps(wp))

    assert result.main([str(session), *common]) == 3
    walkday_error = capsys.readouterr().err.removeprefix("result: ").strip()
    assert message in walkday_error
    assert not (session / "walk_result.json").exists()
    assert json.loads((session / result.RETIRED).read_text()) == original
    rejected = json.loads((session / "walkday.json").read_text())
    assert rejected["status"] == "NOT_EVALUABLE"
    assert any(walkday_error in reason for reason in rejected["reasons"])

    output = session / "adapted.json"
    assert series_main([
        "adapt", str(session / "walkcheck.json"), "--metadata", str(meta_path),
        "--wallpatch", str(adapt_wp_path), "--out", str(output), *common,
    ]) == 1
    assert capsys.readouterr().err.strip().endswith(walkday_error)
    assert not output.exists()


def _strict_report(map_path, b_residuals, **b_fields):
    """A hand-built report: flags as the patch tool sets them at its 5 cm gate."""
    def patch(name, residuals, **extra):
        return dict({"name": name, "status": "MEASURED", "doubled": "false", "separation_m": None,
                     "gate_p95_pass": True, "gate_separation_pass": True, "reasons": [],
                     "residuals": residuals}, **extra)
    clean = {"p95_abs_m": .005, "core_band_m": .004, "stepped_band_m": .004}
    return {"map_sha256": hashlib.sha256(map_path.read_bytes()).hexdigest(),
            "gates": {"p95_point_to_plane_max_m": .05, "revisit_separation_max_m": .05},
            "patches": [patch("A", clean), patch("B", b_residuals, **b_fields)]}


def _strict_verdict(tmp_path, wallpatch, gate_m):
    spec, _ = write_spec(tmp_path / "spec.json", ["A", "B"])
    walkcheck = {"walls": {"status": "OK", "p95_point_to_plane_cm": .485, "doubled_wall": False}}
    prov = patch_provenance("preselected", spec, None, wallpatch, None, gate_m=gate_m)
    _, doubled, failures, notes, _, _ = score_walls(walkcheck, wallpatch, gate_m, prov,
                                                    map_path=str(tmp_path / "map.pcd"))
    status, reasons = wall_verdict("PASS", [], doubled, failures, notes)
    return status, reasons, prov["verified"]


def test_requested_gate_stricter_than_patch_tool_gate(tmp_path):
    # Review repro: 1 cm gate, patch p95 1.545 cm, whole map 0.485 cm returned PASS.
    (tmp_path / "map.pcd").write_bytes(b"synthetic map")
    report = _strict_report(tmp_path / "map.pcd", {"p95_abs_m": .01545, "core_band_m": .004,
                                                   "stepped_band_m": .004})
    status, reasons, verified = _strict_verdict(tmp_path, report, .01)
    assert status == "FAIL"
    assert any("patch B: p95 1.5" in r and "effective gate 1 cm" in r for r in reasons)
    assert verified == ["A"]
    # The same patch at the tool's own 5 cm gate is a clean pass.
    assert _strict_verdict(tmp_path, report, .05) == ("PASS", [], ["A", "B"])
    # A looser requested gate never relaxes the tool's 5 cm gate.
    loose = _strict_report(tmp_path / "map.pcd", {"p95_abs_m": .055, "core_band_m": .004,
                                                  "stepped_band_m": .004})
    assert _strict_verdict(tmp_path, loose, .08)[0] == "FAIL"


def test_strict_gate_band_wider_than_gate_is_not_evaluable(tmp_path):
    (tmp_path / "map.pcd").write_bytes(b"synthetic map")
    report = _strict_report(tmp_path / "map.pcd", {"p95_abs_m": .008, "core_band_m": .012,
                                                   "stepped_band_m": .004})
    status, reasons, verified = _strict_verdict(tmp_path, report, .01)
    assert status == "NOT_EVALUABLE"
    assert any("core_band_m 1.20 cm >= effective gate 1 cm" in r for r in reasons)
    assert verified == ["A"]


def test_tool_fail_without_stepped_band_stays_fail(tmp_path):
    # The patch tool leaves stepped_band_m null unless both its gates pass.
    (tmp_path / "map.pcd").write_bytes(b"synthetic map")
    report = _strict_report(tmp_path / "map.pcd", {"p95_abs_m": .06, "core_band_m": .03,
                                                   "stepped_band_m": None}, gate_p95_pass=False)
    status, reasons, verified = _strict_verdict(tmp_path, report, .05)
    assert status == "FAIL", reasons
    assert verified == ["A"]


def test_guard_band_review_case_is_not_evaluable(tmp_path):
    # sp-rv-m1final3: offset sheets 5.2 cm apart, 5 mm noise, -4 deg hint. The patch
    # tool before Round 7 said MEASURED, doubled=false, both gates True.
    (tmp_path / "map.pcd").write_bytes(b"synthetic map")
    report = _strict_report(tmp_path / "map.pcd", {"p95_abs_m": .032896, "core_band_m": .048876,
                                                   "stepped_band_m": .049625})
    status, reasons, verified = _strict_verdict(tmp_path, report, .05)
    assert status == "NOT_EVALUABLE"
    assert any("core_band_m 4.89 cm > pass limit 4 cm" in r for r in reasons)
    assert any("stepped_band_m 4.96 cm > pass limit 4 cm" in r for r in reasons)
    assert verified == ["A"]


@pytest.mark.parametrize("key,value,expected", [
    ("p95_abs_m", .040, "PASS"), ("p95_abs_m", .0401, "NOT_EVALUABLE"),
    ("p95_abs_m", .050, "NOT_EVALUABLE"), ("p95_abs_m", .0501, "FAIL"),
    ("core_band_m", .040, "PASS"), ("core_band_m", .0401, "NOT_EVALUABLE"),
    ("core_band_m", .050, "NOT_EVALUABLE"), ("core_band_m", .0501, "NOT_EVALUABLE"),
    ("stepped_band_m", .040, "PASS"), ("stepped_band_m", .0401, "NOT_EVALUABLE"),
    ("stepped_band_m", .050, "NOT_EVALUABLE"), ("stepped_band_m", .0501, "NOT_EVALUABLE"),
])
def test_guard_band_boundaries(tmp_path, key, value, expected):
    # <= 4.0 cm PASS; 4.0 < x <= 5.0 not checkable; p95 > 5 cm FAIL. A band never
    # fails: a thick or noisy band stays not checkable (kept from g8safe).
    (tmp_path / "map.pcd").write_bytes(b"synthetic map")
    residuals = {"p95_abs_m": .005, "core_band_m": .004, "stepped_band_m": .004, key: value}
    report = _strict_report(tmp_path / "map.pcd", residuals)
    status, reasons, verified = _strict_verdict(tmp_path, report, .05)
    assert status == expected, reasons
    assert verified == (["A", "B"] if expected == "PASS" else ["A"])


def test_guard_band_uses_the_stricter_reported_pass_limit(tmp_path):
    (tmp_path / "map.pcd").write_bytes(b"synthetic map")
    report = _strict_report(tmp_path / "map.pcd", {"p95_abs_m": .035, "core_band_m": .004,
                                                   "stepped_band_m": .004})
    assert _strict_verdict(tmp_path, report, .05)[0] == "PASS"
    report["gates"]["pass_max_m"] = .03
    status, reasons, _ = _strict_verdict(tmp_path, report, .05)
    assert status == "NOT_EVALUABLE"
    assert any("pass limit 3 cm" in r for r in reasons)
    # A looser reported limit never relaxes the shared 4 cm one.
    report["gates"]["pass_max_m"] = .05
    report["patches"][1]["residuals"]["p95_abs_m"] = .045
    assert _strict_verdict(tmp_path, report, .05)[0] == "NOT_EVALUABLE"


def test_whole_map_doubled_flag_is_only_a_warning(tmp_path):
    (tmp_path / "map.pcd").write_bytes(b"synthetic map")
    spec, _ = write_spec(tmp_path / "spec.json", ["A", "B"])
    report = _strict_report(tmp_path / "map.pcd", {"p95_abs_m": .005, "core_band_m": .004,
                                                   "stepped_band_m": .004})
    prov = patch_provenance("preselected", spec, None, report, None, gate_m=.05)
    for flag in (True, None):
        walkcheck = {"walls": {"status": "OK", "p95_point_to_plane_cm": .485, "doubled_wall": flag,
                               "revisit_separation_cm": 50, "max_revisit_separation_cm": 52}}
        _, doubled, failures, notes, _, warnings = score_walls(walkcheck, report, .05, prov,
                                                               map_path=str(tmp_path / "map.pcd"))
        assert wall_verdict("PASS", [], doubled, failures, notes) == ("PASS", [])
        assert doubled is False
        assert any("warning only" in w.lower() for w in warnings)
    # The scoring patches still fail a run: a doubled patch B is FAIL.
    report["patches"][1].update(doubled="true", separation_m=.08, gate_separation_pass=False)
    _, doubled, failures, notes, _, _ = score_walls(walkcheck, report, .05, prov,
                                                    map_path=str(tmp_path / "map.pcd"))
    assert wall_verdict("PASS", [], doubled, failures, notes)[0] == "FAIL"
