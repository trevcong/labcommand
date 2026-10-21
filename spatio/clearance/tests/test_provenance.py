"""B2: clearance results are bound to their inputs; reuse refuses changed inputs."""

import json
import os
import subprocess
import sys

import pytest

from spatio.clearance import provenance as prov
from spatio.clearance.__main__ import markdown, report
from spatio.clearance.report import build_report, markdown_report


TRAJECTORY, EXTRINSICS, RECORDING = "a" * 64, "b" * 64, "c" * 64
INPUTS = {"recording": RECORDING, "extrinsics": EXTRINSICS, "trajectory": TRAJECTORY,
          "time_basis": "sensor header stamp, seconds"}


def zones_doc(y=0.0):
    return {"schema_version": 1, "frame_id": "prov-map", "zones": [{
        "id": "door", "shape": "box", "rule_id": "example",
        "provision": "EXAMPLE, NOT APPROVED",
        "required_volume": {"min": [0, y, 0], "max": [1, y + .1, .1],
                            "clearance_axis": 0, "clearance_direction": 1}}]}


def scans_doc(*rays):
    rays = rays or ((10, [-.15, .05, .05], [[1.25, .05, .05], [1.25, .06, .05]]),)
    return {"schema_version": 1, "frame_id": "prov-map",
            "scans": [{"timestamp": t, "origin": o, "hits": h} for t, o, h in rays]}


def result(inputs=None, **kw):
    return report(zones_doc(), scans_doc(), .1, [0, 0, 0], .02, inputs=inputs, **kw)


def supplied(**changes):
    out = {"recording": RECORDING, "extrinsics": EXTRINSICS, "trajectory": TRAJECTORY,
           "scans": prov.canonical_sha256(scans_doc())}
    out.update(changes)
    return out


def test_bound_result_records_every_identity_and_reuses_cleanly():
    source = result(INPUTS)
    p = source["provenance"]
    assert p["schema"] == prov.SCHEMA
    assert (p["recording_sha256"], p["extrinsics_sha256"], p["trajectory_sha256"]) == \
        (RECORDING, EXTRINSICS, TRAJECTORY)
    assert p["time_basis"] == INPUTS["time_basis"]
    assert p["scans_sha256"] == prov.canonical_sha256(scans_doc())
    assert p["limitations"] == []
    zone, = source["zones"]
    assert zone["status"] == "PASS"
    assert zone["provenance_limitations"] == []
    assert len(zone["zone_geometry_sha256"]) == 64
    out = build_report(source, zones_doc(), "walk", supplied())
    assert out["counts"]["PASS"] == 1
    assert out["reuse_limitations"] == []
    assert out["provenance"] == p
    assert "Provenance limitations" not in markdown_report(out)


@pytest.mark.parametrize("name", ["trajectory", "extrinsics", "recording", "scans"])
def test_changed_input_blocks_reuse(name):
    with pytest.raises(ValueError, match=f"{name}.*sha256"):
        build_report(result(INPUTS), zones_doc(), "walk", supplied(**{name: "d" * 64}))


@pytest.mark.parametrize("name", ["trajectory", "extrinsics", "scans"])
def test_bound_input_must_be_resupplied(name):
    with pytest.raises(ValueError, match=f"bound to {name}"):
        build_report(result(INPUTS), zones_doc(), "walk", supplied(**{name: None}))


@pytest.mark.parametrize("name", ["trajectory", "extrinsics"])
def test_input_supplied_for_unbound_result_is_a_mismatch(name):
    with pytest.raises(ValueError, match="not bound"):
        build_report(result(), zones_doc(), "walk", {name: "d" * 64})


def test_recording_supplied_for_unbound_result_is_a_mismatch():
    with pytest.raises(ValueError, match="recording sha256"):
        build_report(result(), zones_doc(), "walk",
                     {"recording": RECORDING, "scans": prov.canonical_sha256(scans_doc())})


def test_translated_zone_with_same_id_rule_and_depth_blocks_reuse():
    # Previously undetectable: IDs, rule, provision and depth all still match.
    with pytest.raises(ValueError, match="door: zone geometry changed"):
        build_report(result(INPUTS), zones_doc(y=.5), "walk", supplied())


def test_reordered_zone_document_keys_still_match_geometry():
    zones = zones_doc()
    volume = zones["zones"][0]["required_volume"]
    volume = dict(reversed(list(volume.items())), max=[1.0, .1, .1])  # int vs float
    zones["zones"][0]["required_volume"] = volume
    assert build_report(result(INPUTS), zones, "walk", supplied())["counts"]["PASS"] == 1


@pytest.mark.parametrize("change", [
    lambda r: r.pop("provenance"),
    lambda r: r["provenance"].update(schema="other"),
    lambda r: r["provenance"].update(scans_sha256=None),
    lambda r: r["provenance"].update(trajectory_sha256="A" * 64),
    lambda r: r["provenance"].update(limitations=["unknown"]),
    lambda r: r["zones"][0].pop("zone_geometry_sha256"),
    lambda r: r["zones"][0].update(zone_geometry_sha256="e" * 64),
])
def test_missing_or_malformed_provenance_fails_explicitly(change):
    source = result(INPUTS)
    change(source)
    with pytest.raises(ValueError):
        build_report(source, zones_doc(), "walk", supplied())


def test_missing_timing_and_ray_provenance_is_a_limitation_on_the_pass():
    source = result()
    assert source["provenance"]["limitations"] == [
        "no_extrinsics_identity", "no_recording_identity", "no_time_basis",
        "no_trajectory_identity"]
    zone, = source["zones"]
    assert zone["status"] == "PASS"  # The classifier is unchanged; the gap is explicit.
    assert zone["provenance_limitations"] == source["provenance"]["limitations"]
    rendered = markdown(source)
    assert "Provenance limitations (apply to every zone, including PASS):" in rendered
    for key in source["provenance"]["limitations"]:
        assert f"- {key}: {prov.LIMITATIONS[key]}" in rendered
    with pytest.raises(ValueError, match="bound to scans"):
        build_report(source, zones_doc(), "walk")
    out = build_report(source, zones_doc(), "walk",
                       {"scans": prov.canonical_sha256(scans_doc())})
    assert out["counts"]["PASS"] == 1
    assert out["zones"][0]["evidence"]["provenance_limitations"] == zone["provenance_limitations"]
    assert out["reuse_limitations"] == []
    rendered = markdown_report(out)
    assert "Provenance limitations (apply to every zone, including PASS):" in rendered
    assert "no\\_time\\_basis" in rendered and "scans\\_not\\_reverified" not in rendered


def test_time_basis_is_literal_single_line_markdown():
    source = result(dict(INPUTS, time_basis="stamp | sec\n# heading"))
    assert source["provenance"]["time_basis"] == "stamp | sec\n# heading"
    rendered = markdown(source)
    assert "Time basis: stamp \\| sec # heading." in rendered
    assert "\n# heading" not in rendered
    rendered = markdown_report(build_report(source, zones_doc(), "walk", supplied()))
    assert "\n# heading" not in rendered


def test_only_missing_time_basis_is_listed():
    source = result(dict(INPUTS, time_basis=None))
    assert source["provenance"]["limitations"] == ["no_time_basis"]
    assert source["zones"][0]["provenance_limitations"] == ["no_time_basis"]


def test_bound_result_without_resupplied_recording_says_so():
    out = build_report(result(INPUTS), zones_doc(), "walk", supplied(recording=None))
    assert out["reuse_limitations"] == ["recording_not_reverified"]


@pytest.mark.parametrize("change", [
    lambda p: p.update(limitations=[]),
    lambda p: p["limitations"].remove("no_time_basis"),
    lambda p: p.update(time_basis="claimed later"),
    lambda p: p["limitations"].reverse(),
])
def test_limitations_cannot_be_hand_removed(change):
    source = result()
    change(source["provenance"])
    with pytest.raises(ValueError):
        build_report(source, zones_doc(), "walk",
                     {"scans": prov.canonical_sha256(scans_doc())})


def test_zone_limitations_must_match_result():
    source = result()
    source["zones"][0]["provenance_limitations"] = []
    with pytest.raises(ValueError, match="limitations differ"):
        build_report(source, zones_doc(), "walk",
                     {"scans": prov.canonical_sha256(scans_doc())})


@pytest.mark.parametrize("bad", [{"trajectory": "short"}, {"recording": "A" * 64},
                                 {"time_basis": " "}])
def test_malformed_supplied_identity_rejected(bad):
    with pytest.raises(ValueError):
        result(dict(INPUTS, **bad))


def test_directory_recording_hash_is_deterministic_and_content_bound(tmp_path):
    bag = tmp_path / "bag"
    (bag / "sub").mkdir(parents=True)
    (bag / "metadata.yaml").write_bytes(b"meta")
    (bag / "sub" / "data.mcap").write_bytes(b"\x00\x01")
    first = prov.sha256_path(bag)
    assert first == prov.sha256_path(bag)
    (bag / "sub" / "data.mcap").write_bytes(b"\x00\x02")
    assert prov.sha256_path(bag) != first
    (tmp_path / "empty").mkdir()
    with pytest.raises(ValueError, match="no files"):
        prov.sha256_path(tmp_path / "empty")


# CLI end to end: classify with real files, then reuse.

def run(*args):
    return subprocess.run([sys.executable, "-m", "spatio.clearance", *map(str, args)],
                          capture_output=True, text=True,
                          env={**os.environ, "PYTHONDONTWRITEBYTECODE": "1"})


@pytest.fixture
def bound_cli(tmp_path):
    (tmp_path / "zones.json").write_text(json.dumps(zones_doc()))
    (tmp_path / "scans.json").write_text(json.dumps(scans_doc()))
    (tmp_path / "trajectory.txt").write_text("0 0 0 0\n1 1 0 0\n")
    (tmp_path / "extrinsics.yaml").write_text("lidar: [0, 0, 0]\n")
    (tmp_path / "bag").mkdir()
    (tmp_path / "bag" / "data.mcap").write_bytes(b"\x00raw")
    done = run("--zones", tmp_path / "zones.json", "--scans", tmp_path / "scans.json",
               "--voxel-size", ".1", "--uncertainty-m", ".02",
               "--recording", tmp_path / "bag", "--trajectory", tmp_path / "trajectory.txt",
               "--extrinsics", tmp_path / "extrinsics.yaml", "--time-basis", "header stamp",
               "--output", tmp_path / "result")
    assert done.returncode == 0, done.stderr
    return tmp_path


def reuse(tmp_path, output, *extra):
    return run("report", "--results", tmp_path / "result" / "report.json",
               "--zones", tmp_path / "zones.json", "--walk-id", "w",
               "--output", tmp_path / output, *extra)


def test_cli_binds_inputs_and_refuses_changed_trajectory(bound_cli):
    t = bound_cli
    p = json.loads((t / "result" / "report.json").read_text())["provenance"]
    assert p["trajectory_sha256"] == prov.sha256_path(t / "trajectory.txt")
    assert p["recording_sha256"] == prov.sha256_path(t / "bag")
    assert p["limitations"] == []
    assert str(t) not in (t / "result" / "report.json").read_text()  # hashes, not paths
    same = ["--trajectory", t / "trajectory.txt", "--extrinsics", t / "extrinsics.yaml",
            "--scans", t / "scans.json"]
    ok = reuse(t, "ok", *same, "--recording", t / "bag")
    assert ok.returncode == 0, ok.stderr
    assert json.loads((t / "ok" / "report.json").read_text())["reuse_limitations"] == []
    missing = reuse(t, "missing", "--extrinsics", t / "extrinsics.yaml",
                    "--scans", t / "scans.json")
    assert missing.returncode == 2 and "bound to trajectory" in missing.stderr
    assert not (t / "missing").exists()
    (t / "trajectory.txt").write_text("0 0 0 0\n1 1.01 0 0\n")
    changed = reuse(t, "changed", *same)
    assert changed.returncode == 2 and "trajectory changed" in changed.stderr
    assert not (t / "changed").exists()


def test_cli_refuses_changed_extrinsics_and_zone(bound_cli):
    t = bound_cli
    same = ["--trajectory", t / "trajectory.txt", "--extrinsics", t / "extrinsics.yaml",
            "--scans", t / "scans.json"]
    (t / "extrinsics.yaml").write_text("lidar: [0, 0, 0.01]\n")
    changed = reuse(t, "extrinsics", *same)
    assert changed.returncode == 2 and "extrinsics changed" in changed.stderr
    (t / "extrinsics.yaml").write_text("lidar: [0, 0, 0]\n")
    assert reuse(t, "restored", *same).returncode == 0
    (t / "zones.json").write_text(json.dumps(zones_doc(y=.3)))
    moved = reuse(t, "zone", *same)
    assert moved.returncode == 2 and "zone geometry changed" in moved.stderr
    assert not (t / "zone").exists()


@pytest.mark.parametrize("inputs", [None, INPUTS])
@pytest.mark.parametrize("scan_identity", [{}, {"scans": None}])
def test_reuse_requires_current_scan_identity(inputs, scan_identity):
    current = supplied() if inputs else {}
    current.pop("scans", None)
    current.update(scan_identity)
    with pytest.raises(ValueError, match="bound to scans"):
        build_report(result(inputs), zones_doc(), "walk", current)


def test_reuse_without_any_supplied_inputs_refuses_scans():
    with pytest.raises(ValueError, match="bound to scans"):
        build_report(result(), zones_doc(), "walk")


@pytest.mark.parametrize("changed", [False, True])
def test_cli_reuse_without_scans_is_a_hard_error(bound_cli, changed):
    t = bound_cli
    if changed:
        (t / "scans.json").write_text(json.dumps(
            scans_doc((10, [-.15, .05, .05], [[.8, .05, .05]]))))
    done = reuse(t, "without-scans", "--trajectory", t / "trajectory.txt",
                 "--extrinsics", t / "extrinsics.yaml", "--recording", t / "bag")
    assert done.returncode == 2 and "--scans" in done.stderr
    assert not (t / "without-scans").exists()


@pytest.mark.parametrize("problem", ["changed", "missing", "malformed", "directory"])
def test_cli_unverifiable_scans_refused_before_writing(bound_cli, problem):
    t = bound_cli
    path = t / "scans.json"
    if problem == "changed":
        path.write_text(json.dumps(scans_doc((10, [-.15, .05, .05], [[.8, .05, .05]]))))
    elif problem == "missing":
        path.unlink()
    elif problem == "malformed":
        path.write_text('{"scans":')
    else:
        path.unlink()
        path.mkdir()
    done = reuse(t, "bad-scans", "--trajectory", t / "trajectory.txt",
                 "--extrinsics", t / "extrinsics.yaml", "--scans", path)
    assert done.returncode == 2
    assert "scans" in done.stderr
    assert not (t / "bad-scans").exists()


def test_cli_scan_identity_ignores_json_formatting(bound_cli):
    t = bound_cli
    scans = json.loads((t / "scans.json").read_text())
    (t / "scans.json").write_text(json.dumps(
        dict(reversed(list(scans.items()))), indent=4))
    done = reuse(t, "reformatted", "--trajectory", t / "trajectory.txt",
                 "--extrinsics", t / "extrinsics.yaml", "--scans", t / "scans.json")
    assert done.returncode == 0, done.stderr
    out = json.loads((t / "reformatted" / "report.json").read_text())
    assert out["counts"]["PASS"] == 1
    assert out["reuse_limitations"] == ["recording_not_reverified"]
