import copy
import json
from pathlib import Path
import subprocess
import sys

import numpy as np
import pytest

from spatio.clearance import EvidenceGrid, classify, parse_scans, parse_zones
from spatio.clearance.__main__ import markdown, report
from spatio.clearance.model import load_json
from spatio.clearance.synthetic import first_hit, scene


def box(low=(0, 0, 0), high=(1, .1, .1), axis=0, direction=1):
    return {"schema_version": 1, "frame_id": "test",
            "zones": [{"id": "zone", "shape": "box", "rule_id": "EXAMPLE",
                       "provision": "EXAMPLE, NOT APPROVED; test geometry only",
                       "required_volume": {"min": list(low), "max": list(high),
                                           "clearance_axis": axis,
                                           "clearance_direction": direction}}]}


def scans(*rays):
    return {"schema_version": 1, "frame_id": "test",
            "scans": [{"timestamp": t, "origin": list(origin),
                       "hits": [list(hit) for hit in hits]}
                      for t, origin, hits in rays]}


def evaluate(zones, observations, size=.1, uncertainty=.02):
    return report(zones, observations, size, [0, 0, 0], uncertainty)["zones"][0]


def test_synthetic_panel_hidden_and_gap():
    zones, observations = scene()
    result = report(zones, observations, .1, [0, 0, 0], .02)
    panel, hidden, gap = result["zones"]
    assert panel["status"] == "FAIL"
    assert panel["min_clearance_m"] == pytest.approx(.8)
    assert panel["occupied_cells"] == 1
    assert hidden["status"] == "NOT_MEASURED"
    assert hidden["unknown_cells"] == hidden["required_cells"] == 80
    assert gap["status"] == "NOT_MEASURED"
    assert gap["free_only_cells"] == 5
    assert gap["unknown_cells"] == 95
    assert result["physical_accuracy_evaluated"] is False
    assert all("EXAMPLE, NOT APPROVED" in z["provision"] for z in result["zones"])


def test_new_walk_transformed_geometry_origins_and_times():
    shift, scale = [-3.7, 8.2, -1.6], 1.7
    zones, observations = scene(scale, shift, 17500.25)
    result = report(zones, observations, .1 * scale, shift, .02 * scale)
    assert [z["status"] for z in result["zones"]] == ["FAIL", "NOT_MEASURED", "NOT_MEASURED"]
    assert result["zones"][0]["min_clearance_m"] == pytest.approx(.8 * scale)
    assert result["scan_time_range"] == [17500.25, 17501.75]
    assert result["zones"][2]["free_only_cells"] == 5
    assert result["zones"][2]["required_cells"] == 100


def test_complete_observation_passes():
    result = evaluate(box(), scans((10, (-.15, .05, .05),
                                   [(1.25, .05, .05), (1.25, .06, .05)])))
    assert result["status"] == "PASS"
    assert result["free_only_cells"] == result["required_cells"] == 10
    assert result["min_clearance_m"] == 1.25
    assert result["evidence_time_range"] == [10, 10]


def test_no_rays_is_unknown_not_clear():
    result = evaluate(box(), scans((10, (-.15, .05, .05), [])))
    assert result["status"] == "NOT_MEASURED"
    assert result["unknown_cells"] == 10
    assert result["min_clearance_m"] is None
    assert result["evidence_time_range"] is None


def test_point_cloud_endpoints_do_not_fill_between_rays():
    result = evaluate(box(high=(1, .3, .1)),
                      scans((1, (-.15, .05, .05), [(1.5, .05, .05)])))
    assert result["status"] == "NOT_MEASURED"
    assert result["free_only_cells"] == 10
    assert result["unknown_cells"] == 20


@pytest.mark.parametrize("hit,uncertainty,status", [
    (.8, .02, "FAIL"), (.98, .02, "INDETERMINATE"),
    (.99, .02, "INDETERMINATE"), (.99, 0, "FAIL"),
    (1.01, .02, "INDETERMINATE"), (1.02, .02, "INDETERMINATE"),
    (1.03, .02, "PASS"), (1.0, 0, "PASS"),
])
def test_margin_uncertainty_both_sides(hit, uncertainty, status):
    result = evaluate(box(), scans((1, (-.15, .05, .05),
                                   [(hit, .05, .05), (hit, .06, .05)])),
                      uncertainty=uncertainty)
    assert result["status"] == status
    assert result["decision_margin_m"] == pytest.approx(hit - 1)


def test_unknown_beats_nearby_outside_hit():
    result = evaluate(box(high=(1, .2, .1)),
                      scans((1, (-.15, .05, .05), [(1.01, .05, .05)])))
    assert result["status"] == "NOT_MEASURED"


@pytest.mark.parametrize("order", [(0.8, 1.2), (1.2, 0.8)])
def test_conflicts_are_reported_and_never_cleared(order):
    observations = scans(*[(i + 1, (-.15, .05, .05), [(x, .05, .05)])
                           for i, x in enumerate(order)])
    result = report(box(), observations, .1, [0, 0, 0], .02)
    zone = result["zones"][0]
    assert zone["status"] == "FAIL"
    assert zone["conflicting_cells"] == [[8, 0, 0]]
    assert result["conflicting_cells"] == [[8, 0, 0]]
    assert zone["occupied_cells"] == 1
    assert zone["unknown_cells"] == 0


def test_ray_stops_at_first_return_and_keeps_origin_unknown():
    grid = EvidenceGrid(.1)
    _, observations = parse_scans(scans((1, (.05, .05, .05), [(.35, .05, .05)])))
    grid.add_scan(observations[0])
    assert (0, 0, 0) not in grid.cells
    assert set(grid.cells) == {(1, 0, 0), (2, 0, 0), (3, 0, 0)}
    assert grid.cells[(3, 0, 0)].occupied_times == {1}
    assert not grid.cells[(3, 0, 0)].free_times


def test_reverse_ray_with_negative_coordinates():
    grid = EvidenceGrid(1)
    assert grid.ray_cells([2.5, .5, .5], [-2.5, .5, .5]) == {
        (-2, 0, 0), (-1, 0, 0), (0, 0, 0), (1, 0, 0)}


def test_corner_crossing_does_not_free_side_cells():
    grid = EvidenceGrid(1)
    assert grid.ray_cells([.5, .5, .5], [3.5, 3.5, 3.5]) == {(1, 1, 1), (2, 2, 2)}


def test_ray_on_voxel_boundary_does_not_choose_a_free_side():
    grid = EvidenceGrid(1)
    assert grid.ray_cells([.5, 0, .5], [3.5, 0, .5]) == set()


def test_same_cell_return_has_no_free_space():
    assert EvidenceGrid(1).ray_cells([.2, .2, .2], [.8, .2, .2]) == set()


def test_partial_cells_are_in_coverage_denominator():
    zone = parse_zones(box(low=(.05, .05, .05), high=(.15, .15, .15)))[1][0]
    assert len(list(EvidenceGrid(.1).volume_cells(zone))) == 8


def test_decimal_boundaries_do_not_add_phantom_cells():
    zone = parse_zones(box(low=(.3, .3, .3), high=(.6, .6, .6)))[1][0]
    assert len(list(EvidenceGrid(.1).volume_cells(zone))) == 27


def test_occupied_partial_cell_cannot_disappear_when_hit_outside_volume():
    result = evaluate(box(low=(.05, 0, 0), high=(1, .1, .1)),
                      scans((1, (-.15, .05, .05), [(.01, .05, .05)])))
    assert result["status"] == "FAIL"
    assert result["min_clearance_m"] is None
    assert result["occupied_voxel_clearance_lower_bound_m"] == 0


@pytest.mark.parametrize("axis,direction", [(a, d) for a in range(3) for d in (-1, 1)])
def test_box_clearance_axes_and_directions(axis, direction):
    low, high = [0, 0, 0], [.1, .1, .1]
    high[axis] = 1
    origin, hit = [.05] * 3, [.05] * 3
    origin[axis] = -.15 if direction == 1 else 1.15
    hit[axis] = .8 if direction == 1 else .2
    result = evaluate(box(low, high, axis, direction), scans((1, origin, [hit])))
    assert result["status"] == "FAIL"
    assert result["min_clearance_m"] == pytest.approx(.8)


def test_reference_plane_is_not_measured_as_an_obstruction_distance():
    zones, observations = scene()
    panel = report(zones, observations, .1, [0, 0, 0], .02)["zones"][0]
    assert panel["min_clearance_m"] == pytest.approx(.8)


@pytest.mark.parametrize("change", [
    lambda d: d.update(extra=0),
    lambda d: d.update(schema_version=True),
    lambda d: d.update(frame_id=""),
    lambda d: d.update(zones=[]),
    lambda d: d["zones"].append(copy.deepcopy(d["zones"][0])),
    lambda d: d["zones"][0].update(shape="sphere"),
    lambda d: d["zones"][0].update(provision=" "),
    lambda d: d["zones"][0].update(rule_id=None),
    lambda d: d["zones"][0].update(extra=1),
    lambda d: d["zones"][0]["required_volume"].update(min=[0, 0]),
    lambda d: d["zones"][0]["required_volume"].update(min=[float("nan"), 0, 0]),
    lambda d: d["zones"][0]["required_volume"].update(min=[True, 0, 0]),
    lambda d: d["zones"][0]["required_volume"].update(max=[0, 0, 0]),
    lambda d: d["zones"][0]["required_volume"].update(clearance_axis=True),
    lambda d: d["zones"][0]["required_volume"].update(clearance_axis=3),
    lambda d: d["zones"][0]["required_volume"].update(clearance_direction=0),
])
def test_strict_zone_rejections(change):
    data = box()
    change(data)
    with pytest.raises(ValueError):
        parse_zones(data)


@pytest.mark.parametrize("change", [
    {"normal": [1, 1, 0]}, {"normal": [0, 0, 0]}, {"normal": [.5, 0, 0]},
    {"depth_m": 0}, {"u_min": .5, "u_max": .1}, {"depth_m": float("inf")},
])
def test_strict_plane_rejections(change):
    data, _ = scene()
    data["zones"][0]["required_volume"].update(change)
    with pytest.raises(ValueError):
        parse_zones(data)


@pytest.mark.parametrize("change", [
    lambda d: d.update(frame_id=None),
    lambda d: d.update(scans={}),
    lambda d: d["scans"][0].update(timestamp=True),
    lambda d: d["scans"][0].update(timestamp=float("inf")),
    lambda d: d["scans"][0].update(origin=[0, 0]),
    lambda d: d["scans"][0].update(hits=[[float("nan"), 0, 0]]),
    lambda d: d["scans"][0].update(hits=[[0, 0, 0]]),
    lambda d: d["scans"][0].update(hits=None),
    lambda d: d["scans"][0].update(extra=1),
    lambda d: d["scans"].append(copy.deepcopy(d["scans"][0])),
])
def test_strict_scan_rejections(change):
    data = scans((10, (0, 0, 0), [(1, 1, 1)]))
    change(data)
    with pytest.raises(ValueError):
        parse_scans(data)


@pytest.mark.parametrize("content", ['{"x":1,"x":2}', '{"x":NaN}', '{"x":Infinity}', '{"x":1e999}'])
def test_json_rejects_duplicates_and_nonfinite_values(tmp_path, content):
    path = tmp_path / "bad.json"
    path.write_text(content)
    if "1e999" in content:
        # Overflowed JSON numbers are rejected by the numeric field validator.
        data = box()
        data["zones"][0]["required_volume"]["min"][0] = load_json(path)["x"]
        with pytest.raises(ValueError):
            parse_zones(data)
    else:
        with pytest.raises(ValueError):
            load_json(path)


@pytest.mark.parametrize("size", [0, -1, float("nan"), float("inf"), True])
def test_invalid_voxel_size(size):
    with pytest.raises(ValueError):
        EvidenceGrid(size)


@pytest.mark.parametrize("uncertainty", [-.01, float("nan"), float("inf"), True])
def test_invalid_uncertainty(uncertainty):
    with pytest.raises(ValueError):
        evaluate(box(), scans(), uncertainty=uncertainty)


def test_frame_mismatch():
    observations = scans()
    observations["frame_id"] = "another_map"
    with pytest.raises(ValueError, match="frame_id"):
        evaluate(box(), observations)


def test_resource_bounds():
    grid = EvidenceGrid(.1, max_cells=5)
    zone = parse_zones(box())[1][0]
    with pytest.raises(ValueError, match="max_cells"):
        grid.volume_cells(zone)
    with pytest.raises(ValueError, match="max_cells"):
        grid.ray_cells([.05, .05, .05], [2.05, .05, .05])
    with pytest.raises(ValueError, match="precision"):
        grid.index([1e300, 0, 0])


def test_generator_uses_first_surface_not_hidden_target():
    assert first_hit([0, .5, .5], [3, .5, .5],
                     [([1, 0, 0], [2, 1, 1])]) == [1, .5, .5]


def test_markdown_matches_json_and_escapes_supplied_text():
    data = box()
    data["zones"][0]["id"] = "a|b\nc"
    result = report(data, scans(), .1, [0, 0, 0], .02)
    rendered = markdown(result)
    assert "a\\|b c" in rendered
    assert "| NOT_MEASURED | not measured | 0 / 10 | 0 / 10 / 0 |" in rendered
    assert "EXAMPLE, NOT APPROVED" in rendered


def test_cli_synthetic_and_explicit_fresh_inputs(tmp_path):
    first, second = tmp_path / "first", tmp_path / "second"
    base = [sys.executable, "-m", "spatio.clearance", "--voxel-size", ".1",
            "--uncertainty-m", ".02"]
    completed = subprocess.run(base + ["--synthetic", "--output", str(first)],
                               capture_output=True, text=True)
    assert completed.returncode == 0, completed.stderr
    result = json.loads((first / "report.json").read_text())
    assert result["zones"][0]["min_clearance_m"] == pytest.approx(.8)
    assert completed.stdout == (first / "report.md").read_text()
    zones, observations = scene(2.0, [10, 20, 30], 8000)
    (tmp_path / "zones.json").write_text(json.dumps(zones))
    (tmp_path / "scans.json").write_text(json.dumps(observations))
    completed = subprocess.run(
        [sys.executable, "-m", "spatio.clearance", "--voxel-size", ".2",
         "--grid-origin", "10", "20", "30", "--uncertainty-m", ".04",
         "--zones", str(tmp_path / "zones.json"), "--scans", str(tmp_path / "scans.json"),
         "--output", str(second)], capture_output=True, text=True)
    assert completed.returncode == 0, completed.stderr
    result = json.loads((second / "report.json").read_text())
    assert result["zones"][0]["min_clearance_m"] == pytest.approx(1.6)
    repeated = subprocess.run(base + ["--synthetic", "--output", str(first)],
                              capture_output=True, text=True)
    assert repeated.returncode == 2


def test_cli_rejects_mixed_modes_before_writing(tmp_path):
    output = tmp_path / "output"
    result = subprocess.run(
        [sys.executable, "-m", "spatio.clearance", "--synthetic", "--zones", "unused",
         "--voxel-size", ".1", "--uncertainty-m", ".02", "--output", str(output)],
        capture_output=True, text=True)
    assert result.returncode == 2
    assert not output.exists()


@pytest.mark.parametrize("axis,direction", [(a, d) for a in range(3) for d in (-1, 1)])
@pytest.mark.parametrize("tangent_index,side", [(t, s) for t in (0, 1) for s in (-1, 1)])
def test_hit_5mm_sideways_outside_footprint_abstains(axis, direction, tangent_index, side):
    high = [.1] * 3
    high[axis] = 1
    origin, clear_hit, nearby_hit = [.05] * 3, [.05] * 3, [.05] * 3
    origin[axis] = -.15 if direction == 1 else 1.15
    clear_hit[axis] = 1.25 if direction == 1 else -.25
    nearby_hit[axis] = .5
    tangent = [a for a in range(3) if a != axis][tangent_index]
    nearby_hit[tangent] = -.005 if side == -1 else .105
    clear_hit_2 = clear_hit.copy()
    clear_hit_2[tangent] = .06
    nearby_origin = nearby_hit.copy()
    nearby_origin[axis] -= .2
    observations = scans((1, origin, [clear_hit, clear_hit_2]),
                         (2, nearby_origin, [nearby_hit]))
    result = evaluate(box(high=high, axis=axis, direction=direction), observations)
    assert result["status"] == "INDETERMINATE"
    assert result["occupied_cells"] == result["insufficient_evidence_cells"] == 0
    assert result["min_clearance_m"] == pytest.approx(.5)
    assert result["evidence_time_range"] == [1, 2]


@pytest.mark.parametrize("y,depth,uncertainty,status", [
    (.12, .5, .02, "INDETERMINATE"),   # Inclusive outer sideways boundary.
    (-.02, .5, .02, "INDETERMINATE"),
    (.12001, .5, .02, "PASS"),
    (-.02001, .5, .02, "PASS"),
    (.105, 1.02, .02, "INDETERMINATE"),
    (.105, 1.02001, .02, "PASS"),
    (.105, .5, 0, "PASS"),
])
def test_sideways_band_boundaries(y, depth, uncertainty, status):
    observations = scans((1, (-.15, .05, .05),
                          [(1.25, .05, .05), (1.25, .06, .05)]),
                         (2, (depth - .2, y, .05), [(depth, y, .05)]))
    result = evaluate(box(), observations, uncertainty=uncertainty)
    assert result["status"] == status
    assert result["occupied_cells"] == 0


@pytest.mark.parametrize("ray_count,status", [(1, "NOT_MEASURED"), (2, "PASS"), (3, "PASS")])
@pytest.mark.parametrize("separate_scans", [False, True])
def test_default_minimum_counts_rays_not_scan_timestamps(ray_count, status, separate_scans):
    hits = [(1.25, .05 + i * .005, .05) for i in range(ray_count)]
    observations = (scans(*[(i + 1, (-.15, .05, .05), [hit])
                            for i, hit in enumerate(hits)]) if separate_scans else
                    scans((1, (-.15, .05, .05), hits)))
    result = evaluate(box(), observations)
    assert result["status"] == status
    assert result["min_free_rays"] == 2
    assert result["insufficient_evidence_cells"] == (10 if ray_count == 1 else 0)
    assert result["free_only_cells"] == 10
    assert result["unknown_cells"] == 0


def test_one_under_supported_cell_prevents_pass():
    observations = scans((1, (-.15, .05, .05), [(1.25, .05, .05)]),
                         (2, (.05, .05, .05), [(1.25, .06, .05)]))
    result = evaluate(box(), observations)
    assert result["status"] == "NOT_MEASURED"
    assert result["insufficient_evidence_cells"] == 1
    assert result["free_only_cells"] == 10


def test_minimum_evidence_precedes_sideways_uncertainty():
    result = evaluate(box(), scans((1, (-.15, .05, .05), [(1.25, .05, .05)]),
                                   (2, (.3, .105, .05), [(.5, .105, .05)])))
    assert result["status"] == "NOT_MEASURED"
    assert result["insufficient_evidence_cells"] == 10


@pytest.mark.parametrize("hit,status", [(.8, "FAIL"), (.99, "INDETERMINATE")])
def test_occupied_precedence_survives_high_minimum(hit, status):
    result = report(box(), scans((1, (-.15, .05, .05), [(hit, .05, .05)])),
                    .1, [0, 0, 0], .02, min_free_rays=100)["zones"][0]
    assert result["status"] == status
    assert result["insufficient_evidence_cells"] > 0


@pytest.mark.parametrize("minimum", [0, -1, 1.5, True, "2", None, float("inf")])
def test_invalid_minimum_evidence_rejected_by_report_and_classifier(minimum):
    zone = parse_zones(box())[1][0]
    with pytest.raises(ValueError, match="min_free_rays"):
        report(box(), scans(), .1, [0, 0, 0], .02, min_free_rays=minimum)
    with pytest.raises(ValueError, match="min_free_rays"):
        classify(zone, EvidenceGrid(.1), .02, min_free_rays=minimum)


@pytest.mark.parametrize("minimum,status", [(None, "PASS"), (1, "PASS"), (3, "NOT_MEASURED")])
def test_cli_minimum_evidence_with_fresh_transformed_scene(tmp_path, minimum, status):
    zones = box(low=(4, 5, 6), high=(6, 5.2, 6.2))
    observations = scans((300, (3.7, 5.1, 6.1),
                          [(6.5, 5.1, 6.1), (6.5, 5.12, 6.1)]))
    for name, data in [("zones", zones), ("scans", observations)]:
        (tmp_path / f"{name}.json").write_text(json.dumps(data))
    output = tmp_path / "result"
    args = [sys.executable, "-m", "spatio.clearance", "--voxel-size", ".2",
            "--grid-origin", "4", "5", "6", "--uncertainty-m", ".04",
            "--zones", str(tmp_path / "zones.json"), "--scans", str(tmp_path / "scans.json"),
            "--output", str(output)]
    if minimum is not None:
        args += ["--min-free-rays", str(minimum)]
    completed = subprocess.run(args, capture_output=True, text=True)
    assert completed.returncode == 0, completed.stderr
    result = json.loads((output / "report.json").read_text())
    assert result["min_free_rays"] == (2 if minimum is None else minimum)
    assert result["minimum_evidence_status"] == "PROVISIONAL"
    assert result["zones"][0]["status"] == status
    assert result["zones"][0]["insufficient_evidence_cells"] == (10 if minimum == 3 else 0)
    assert completed.stdout == (output / "report.md").read_text()
    assert f"minimum free rays per required cell: {result['min_free_rays']}" in completed.stdout


@pytest.mark.parametrize("minimum", ["0", "-1", "1.5"])
def test_cli_rejects_invalid_minimum_before_writing(tmp_path, minimum):
    output = tmp_path / "result"
    completed = subprocess.run(
        [sys.executable, "-m", "spatio.clearance", "--synthetic",
         "--voxel-size", ".1", "--uncertainty-m", ".02", "--min-free-rays", minimum,
         "--output", str(output)], capture_output=True, text=True)
    assert completed.returncode == 2
    assert not output.exists()
