import copy
import json
import os
import subprocess
import sys

import pytest

from spatio.clearance.__main__ import report
from spatio.clearance.report import build_report, json_text, markdown_report


def four_statuses(scale=1, shift=(0, 0, 0), timestamp=100):
    """Independent first-return rows, including a wholly unobserved volume."""
    zones = {"schema_version": 1, "frame_id": "synthetic-map", "zones": []}
    scans = {"schema_version": 1, "frame_id": "synthetic-map", "scans": []}

    def point(x, y, z):
        return [scale * value + offset for value, offset in zip((x, y, z), shift)]

    for i, (zid, hit) in enumerate((
            ("z-clear", 1.25), ("c-blocked", .8), ("b-unseen", None), ("a-near", .99))):
        zones["zones"].append({
            "id": zid, "shape": "box", "rule_id": f"example-{i}",
            "provision": "EXAMPLE, NOT APPROVED; synthetic minimum only",
            "required_volume": {"min": point(0, 2*i, 0), "max": point(1, 2*i+.1, .1),
                                "clearance_axis": 0, "clearance_direction": 1}})
        scans["scans"].append({
            "timestamp": timestamp + i,
            "origin": point(-.15, 2*i+.05, .05),
            "hits": [] if hit is None else [
                point(hit, 2*i+.05, .05), point(hit, 2*i+.06, .05)]})
    results = report(zones, scans, .1*scale, list(shift), .02*scale)
    return zones, results


def test_four_statuses_fidelity_and_exact_fail_measurement():
    zones, source = four_statuses()
    before = copy.deepcopy(source)
    result = build_report(source, zones, "walk-test")
    assert result["schema"] == "spatio.clearance_report.v1"
    assert result["walk_id"] == "walk-test"
    assert result["counts"] == {"PASS": 1, "FAIL": 1, "NOT_MEASURED": 1, "INDETERMINATE": 1}
    assert result["zone_count"] == 4
    by_id = {z["id"]: z for z in source["zones"]}
    for item in result["zones"]:
        assert item["evidence"] == by_id[item["evidence"]["id"]]
    failure, = result["failures"]
    assert failure["evidence"]["id"] == "c-blocked"
    assert failure["evidence"]["min_clearance_m"] == .8
    assert failure["evidence"]["required_depth_m"] == 1
    assert failure["map_position_m"] == [0.5, 2.05, .05]
    assert failure["evidence"]["evidence_time_range"] == [101, 101]
    assert source == before  # Presentation must not rewrite upstream evidence.
    rendered = markdown_report(result)
    assert "0.8 / 1.0" in rendered
    assert "PASS=1; FAIL=1; NOT MEASURED=1; INDETERMINATE=1" in rendered
    assert "1. a\\-near" in rendered and "2. b\\-unseen" in rendered


def test_recheck_order_is_lexical_and_includes_both_abstention_types():
    zones, source = four_statuses()
    source["zones"].reverse()
    zones["zones"].reverse()
    result = build_report(source, zones, "walk")
    assert [(z["number"], z["evidence"]["id"], z["evidence"]["status"])
            for z in result["rechecks"]] == [
                (1, "a-near", "INDETERMINATE"), (2, "b-unseen", "NOT_MEASURED")]
    assert result["rechecks"][0]["map_position_m"] == [.5, 6.05, .05]
    assert result["rechecks"][1]["evidence"]["reason"] == \
        "required cells lack minimum free-ray evidence"
    assert result["rechecks"][1]["evidence"]["evidence_time_range"] is None


def test_determinism_even_when_zone_order_and_object_key_order_differ():
    zones, source = four_statuses()
    first = build_report(source, zones, "walk")
    source = dict(reversed(list(source.items())))
    source["zones"].reverse()
    zones["zones"].reverse()
    second = build_report(source, zones, "walk")
    assert json_text(first).encode() == json_text(second).encode()
    assert markdown_report(first).encode() == markdown_report(second).encode()


def test_empty_zone_lists_have_zero_counts_and_no_pass_conclusion():
    zones, source = four_statuses()
    zones["zones"], source["zones"] = [], []
    result = build_report(source, zones, "empty")
    assert result["zone_count"] == 0
    assert all(count == 0 for count in result["counts"].values())
    assert result["failures"] == result["rechecks"] == result["zones"] == []
    assert "No zones supplied; no clearance conclusions." in markdown_report(result)


def test_new_walk_uses_its_own_settings_geometry_and_times():
    zones, source = four_statuses(2, (10, -4, 6), 9000)
    result = build_report(source, zones, "different-walk")
    assert result["counts"] == {"PASS": 1, "FAIL": 1, "NOT_MEASURED": 1, "INDETERMINATE": 1}
    assert result["settings"]["grid_origin"] == [10, -4, 6]
    assert result["settings"]["voxel_size_m"] == .2
    assert result["settings"]["uncertainty_m"] == .04
    failure, = result["failures"]
    assert failure["map_position_m"] == pytest.approx([11, .1, 6.1])
    assert failure["evidence"]["min_clearance_m"] == pytest.approx(1.6)
    assert result["scan_time_range"] == [9000, 9003]


def test_plane_position_and_null_measurement_are_not_invented():
    zones = {"schema_version": 1, "frame_id": "plane-test", "zones": [{
        "id": "panel", "shape": "plane-offset", "rule_id": "example",
        "provision": "EXAMPLE ONLY",
        "required_volume": {"plane_point": [2.05, 3, 4], "normal": [-1, 0, 0],
                            "u_min": 0, "u_max": .1, "v_min": 0, "v_max": .1,
                            "depth_m": 1}}]}
    observations = {"schema_version": 1, "frame_id": "plane-test", "scans": [{
        "timestamp": 12, "origin": [3, 3.05, 4.05], "hits": [[2.08, 3.05, 4.05]]}]}
    source = report(zones, observations, .1, [0, 0, 0], 0)
    result = build_report(source, zones, "boundary")
    failure, = result["failures"]
    assert failure["map_position_m"] == pytest.approx([1.55, 3.05, 4.05])
    assert failure["evidence"]["min_clearance_m"] is None
    assert failure["evidence"]["occupied_voxel_clearance_lower_bound_m"] == 0
    assert "not measured / 1.0" in markdown_report(result)


@pytest.mark.parametrize("index", [2, 3])
def test_relabelling_unsupported_zone_as_pass_is_rejected(index):
    zones, source = four_statuses()
    source["zones"][index]["status"] = "PASS"
    with pytest.raises(ValueError, match="PASS lacks"):
        build_report(source, zones, "bad")


@pytest.mark.parametrize("change", [
    lambda z: z.update(evidence_time_range=None),
    lambda z: z.update(insufficient_evidence_cells=1),
    lambda z: z.update(free_only_cells=0, unknown_cells=10,
                       insufficient_evidence_cells=10, free_coverage=0),
])
def test_pass_without_summary_evidence_is_rejected(change):
    zones, source = four_statuses()
    change(source["zones"][0])
    with pytest.raises(ValueError, match="PASS lacks"):
        build_report(source, zones, "bad")


def all_free_zone(hit=1.25, sideways=False, ray_count=2):
    zones, _ = four_statuses()
    zones["zones"] = zones["zones"][:1]
    scans = {"schema_version": 1, "frame_id": zones["frame_id"], "scans": [{
        "timestamp": 42, "origin": [-.15, .05, .05],
        "hits": [[1.25 if sideways else hit, y, .05]
                 for y in (.05, .06)[:ray_count]]}]}
    if sideways:
        scans["scans"].append({
            "timestamp": 43, "origin": [hit - .2, .105, .05],
            "hits": [[hit, .105, .05]]})
    return zones, report(zones, scans, .1, [0, 0, 0], .02)


@pytest.mark.parametrize("hit,sideways", [
    (.5, True), (1.01, False), (1.02, False), (1.02 + 5e-13, False),
])
def test_all_free_indeterminate_relabelled_pass_is_rejected(hit, sideways):
    zones, source = all_free_zone(hit, sideways)
    record, = source["zones"]
    assert record["status"] == "INDETERMINATE"
    assert record["free_only_cells"] == record["required_cells"]
    assert record["occupied_cells"] == record["insufficient_evidence_cells"] == 0
    assert record["occupied_voxel_clearance_lower_bound_m"] is None
    assert record["conflicting_cells"] == []
    assert build_report(source, zones, "original")["counts"]["INDETERMINATE"] == 1
    record["status"] = "PASS"
    with pytest.raises(ValueError, match="PASS lacks"):
        build_report(source, zones, "relabelled")


@pytest.mark.parametrize("change", [
    {"min_clearance_m": .5},
    {"min_clearance_m": .5, "decision_margin_m": -.5},
    {"min_clearance_m": 1.0, "decision_margin_m": 0},
    {"occupied_voxel_clearance_lower_bound_m": 0},
    {"decision_margin_m": .3},
    {"decision_margin_m": None},
    {"min_clearance_m": None},
])
def test_pass_with_inconsistent_clearance_evidence_is_rejected(change):
    zones, source = all_free_zone()
    source["zones"][0].update(change)
    with pytest.raises(ValueError):
        build_report(source, zones, "bad")


@pytest.mark.parametrize("hit,sideways,ray_count,status", [
    (1.25, False, 1, "NOT_MEASURED"),
    (.5, True, 2, "INDETERMINATE"),
])
@pytest.mark.parametrize("change", [
    {"decision_margin_m": .3},
    {"decision_margin_m": None},
    {"min_clearance_m": None},
    {"occupied_voxel_clearance_lower_bound_m": 0},
])
def test_nonpass_unoccupied_margin_consistency(hit, sideways, ray_count, status, change):
    zones, source = all_free_zone(hit, sideways, ray_count)
    assert source["zones"][0]["status"] == status
    assert build_report(source, zones, "original")["counts"][status] == 1
    source["zones"][0].update(change)
    with pytest.raises(ValueError):
        build_report(source, zones, "bad")


@pytest.mark.parametrize("change", [
    {"decision_margin_m": None},
    {"decision_margin_m": -2},
    {"decision_margin_m": 1},
    {"occupied_voxel_clearance_lower_bound_m": None},
])
def test_occupied_margin_consistency(change):
    zones, source = four_statuses()
    source["zones"][1].update(change)
    with pytest.raises(ValueError):
        build_report(source, zones, "bad")


def test_pass_without_hit_measurement_preserves_null_margin():
    zones, _ = all_free_zone()
    scans = {"schema_version": 1, "frame_id": zones["frame_id"], "scans": [{
        "timestamp": 42, "origin": [-.15, .05, .05],
        "hits": [[10.25, .25, .05], [10.25, .26, .05]]}]}
    source = report(zones, scans, .1, [0, 0, 0], .02)
    record, = source["zones"]
    assert record["status"] == "PASS"
    assert record["min_clearance_m"] is record["decision_margin_m"] is None
    assert build_report(source, zones, "no-hit")["counts"]["PASS"] == 1


@pytest.mark.parametrize("clearance,uncertainty,accepted", [
    (1.02001, .02, True), (1.0, 0, False), (1.00001, 0, True),
])
def test_pass_requires_margin_strictly_beyond_uncertainty(clearance, uncertainty, accepted):
    zones, source = all_free_zone()
    source["uncertainty_m"] = uncertainty
    source["zones"][0].update(
        min_clearance_m=clearance, decision_margin_m=clearance - 1,
        uncertainty_m=uncertainty)
    if accepted:
        assert build_report(source, zones, "beyond")["counts"]["PASS"] == 1
    else:
        with pytest.raises(ValueError, match="PASS lacks"):
            build_report(source, zones, "boundary")


def test_occupied_margin_uses_volume_evidence_not_sideways_measurement():
    zones, _ = all_free_zone()
    scans = {"schema_version": 1, "frame_id": zones["frame_id"], "scans": [
        {"timestamp": 42, "origin": [-.15, .05, .05],
         "hits": [[.8, .05, .05]]},
        {"timestamp": 43, "origin": [.3, .105, .05],
         "hits": [[.5, .105, .05]]},
    ]}
    source = report(zones, scans, .1, [0, 0, 0], .02)
    record, = source["zones"]
    assert record["status"] == "FAIL"
    assert record["min_clearance_m"] == .5
    assert record["decision_margin_m"] == pytest.approx(-.2)
    assert build_report(source, zones, "occupied")["failures"][0]["evidence"] == record


def test_cli_all_free_relabelled_pass_rejected_before_writing(tmp_path):
    zones, source = all_free_zone(.5, sideways=True)
    assert source["zones"][0]["status"] == "INDETERMINATE"
    source["zones"][0]["status"] = "PASS"
    output = tmp_path / "bad-sideways"
    result = run_cli(tmp_path, source, zones, output)
    assert result.returncode == 2
    assert "PASS lacks" in result.stderr
    assert not output.exists()


@pytest.mark.parametrize("change", [
    lambda r: r.update(frame_id="wrong"),
    lambda r: r.update(schema_version=True),
    lambda r: r.update(physical_accuracy_evaluated=True),
    lambda r: r["zones"].append(copy.deepcopy(r["zones"][0])),
    lambda r: r["zones"].pop(),
    lambda r: r["zones"][0].update(status="CLEAR"),
    lambda r: r["zones"][0].update(rule_id="other"),
    lambda r: r["zones"][0].update(provision="other"),
    lambda r: r["zones"][0].update(required_depth_m=2),
    lambda r: r["zones"][0].update(min_free_rays=1),
    lambda r: r["zones"][0].update(reason=""),
    lambda r: r["zones"][0].update(required_cells=True),
    lambda r: r["zones"][0].update(free_coverage=.9),
    lambda r: r["zones"][0].update(evidence_time_range=[20, 10]),
    lambda r: r["zones"][0].update(evidence_time_range=[0, 1]),
    lambda r: r["zones"][0].pop("min_clearance_m"),
    lambda r: r.update(scan_count=0, scan_time_range=None),
    lambda r: r.update(voxel_size_m=float("nan")),
    lambda r: r.update(uncertainty_m=-1),
    lambda r: r.update(min_free_rays=0),
])
def test_invalid_or_mismatched_results_fail_explicitly(change):
    zones, source = four_statuses()
    change(source)
    with pytest.raises(ValueError):
        build_report(source, zones, "bad")


def test_supplied_markdown_and_html_are_literal_single_line_text():
    zones, source = four_statuses()
    label = "<script>|**unsafe**\n# title"
    zones["zones"][1]["id"] = source["zones"][1]["id"] = label
    rendered = markdown_report(build_report(source, zones, label))
    assert "<script>" not in rendered
    assert "&lt;script&gt;\\|\\*\\*unsafe\\*\\*" in rendered
    assert "\n# title" not in rendered
    # Literal text remains intact in the machine-readable evidence.
    assert build_report(source, zones, label)["failures"][0]["evidence"]["id"] == label


def run_cli(tmp_path, source, zones, output, seed="1"):
    for name, data in (("source", source), ("zones", zones)):
        (tmp_path / f"{name}.json").write_text(json.dumps(data))
    return subprocess.run([
        sys.executable, "-m", "spatio.clearance", "report",
        "--results", str(tmp_path / "source.json"),
        "--zones", str(tmp_path / "zones.json"), "--walk-id", "cli-walk",
        "--output", str(output)], capture_output=True, text=True,
        env={**os.environ, "PYTHONHASHSEED": seed, "PYTHONDONTWRITEBYTECODE": "1"})


def test_cli_byte_identical_across_processes_and_refuses_overwrite(tmp_path):
    zones, source = four_statuses()
    first, second = tmp_path / "first", tmp_path / "second"
    a = run_cli(tmp_path, source, zones, first)
    b = run_cli(tmp_path, source, zones, second, "789")
    assert a.returncode == b.returncode == 0, a.stderr + b.stderr
    for name in ("report.json", "report.md"):
        assert (first / name).read_bytes() == (second / name).read_bytes()
        assert (first / name).read_bytes().endswith(b"\n")
    assert a.stdout == b.stdout == (first / "report.md").read_text()
    assert json.loads((first / "report.json").read_text())["schema"] == \
        "spatio.clearance_report.v1"
    original = (first / "report.json").read_bytes()
    assert run_cli(tmp_path, source, zones, first).returncode == 2
    assert (first / "report.json").read_bytes() == original


def test_cli_bad_pass_rejected_before_writing(tmp_path):
    zones, source = four_statuses()
    source["zones"][2]["status"] = "PASS"
    output = tmp_path / "bad"
    result = run_cli(tmp_path, source, zones, output)
    assert result.returncode == 2
    assert "PASS lacks" in result.stderr
    assert not output.exists()


def test_cli_empty_walk(tmp_path):
    zones, source = four_statuses()
    zones["zones"], source["zones"] = [], []
    result = run_cli(tmp_path, source, zones, tmp_path / "empty")
    assert result.returncode == 0, result.stderr
    assert "No zones supplied" in result.stdout
