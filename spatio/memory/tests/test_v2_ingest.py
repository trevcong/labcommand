"""V2 public-path acceptance tests; generated inputs are local test fixtures only."""
import itertools
import json
import os
import subprocess
import sys

import numpy as np
import pytest

from spatio.clearance import provenance
from spatio.clearance.__main__ import report as clearance_report
from spatio.clearance.model import parse_zones
from spatio.clearance.tests.test_report import four_statuses
from spatio.io.tum import read_tum
from spatio.manifest.build import build_manifest
from spatio.memory.evidence import EvidenceStore, ValidationError
from spatio.memory.ingest import IngestRefused, ingest_walkday
from spatio.series.tests.test_series import make_replay
from spatio.series.validate import parse_walk_result
from spatio.wallpatch.measure import measure_map
from spatio.wallpatch.pcd import write_ascii_pcd

CODE = "1" * 40
FASTLIO = "2" * 40


def write(path, data):
    path.write_text(json.dumps(data, allow_nan=False))
    return path


def edit(path, change):
    data = json.loads(path.read_text())
    change(data)
    write(path, data)


@pytest.fixture
def run(tmp_path):
    recording = tmp_path / "recording"
    (recording / "bag").mkdir(parents=True)
    (recording / "bag/metadata.yaml").write_text("rosbag2_bagfile_information: {}")
    (recording / "bag/walk.db3").write_bytes(b"raw local test bytes")
    session = tmp_path / "walk-one"
    session.mkdir()
    (session / "trajectory.tum").write_text("100 0 0 0 0 0 0 1\n104 1 0 0 0 0 0 1\n")
    assert list(read_tum(session / "trajectory.tum").t) == [100, 104]
    yz = np.linspace(-.4, .4, 15)
    write_ascii_pcd(session / "map.pcd", np.array([[0., y, z] for y in yz for z in yz]))
    config = session / "config.yaml"
    config.write_text("test: config")
    manifest = build_manifest(session, session.name, str(recording), "mid360", FASTLIO,
                              str(config), "test-image", "test fixture", hash_bag=True)
    walk = make_replay(session.name, fastlio_revision=FASTLIO,
                       config_sha256=manifest["config_sha256"])
    parse_walk_result(walk, "fixture")
    write(session / "walk_result.json", walk)
    patches = write(session / "patches.json", {
        "schema": "spatio.wall_patches/1", "map_sha256": provenance.sha256_path(session / "map.pcd"),
        "patches": [{"name": name, "box_map_frame": {
            "center_m": [0, 0, 0], "normal_hint": [1, 0, 0], "half_extent_m": [.5, .2, .5]}}
            for name in ("wall-a", "wall-b")] + [{"name": "unmeasured"}]})
    measure_map(session / "map.pcd", patches, session / "wallpatch")
    zones, _, scans = four_statuses()
    parse_zones(zones)
    extrinsics = tmp_path / "extrinsics.yaml"
    extrinsics.write_text("lidar_to_base: identity")
    clearance = (tmp_path / "clearance.json", write(tmp_path / "zones.json", zones),
                 write(tmp_path / "scans.json", scans), extrinsics)
    write(clearance[0], classify(session, recording, clearance))
    return session, recording, clearance


def classify(session, recording, clearance):
    """Classifier report.json from the clearance files' current bytes."""
    zones, scans = (json.loads(p.read_text()) for p in clearance[1:3])
    return clearance_report(zones, scans, .1, [0, 0, 0], .02, inputs={
        "recording": provenance.sha256_path(recording),
        "trajectory": provenance.sha256_path(session / "trajectory.tum"),
        "extrinsics": provenance.sha256_path(clearance[3]), "time_basis": "sensor seconds"})


@pytest.fixture
def store(tmp_path):
    with EvidenceStore(tmp_path / "evidence.sqlite", wallclock=lambda: 1000) as value:
        yield value


def ingest(store, run, clearance=True, code=CODE):
    return ingest_walkday(store, run[0], run[1], code, run[2] if clearance else None)


def snapshot(store):
    return list(store.connection.iterdump())


def observations(store):
    return [dict(r) for r in store.connection.execute("SELECT * FROM observations ORDER BY observation_id")]


def test_ingest_binds_recording_trajectory_map_and_code(store, run):
    result = ingest(store, run)
    s = dict(store.connection.execute("SELECT * FROM sessions").fetchone())
    t = dict(store.connection.execute("SELECT * FROM trajectory_revisions").fetchone())
    m = dict(store.connection.execute("SELECT * FROM map_revisions").fetchone())
    assert s["source_recording_sha256"] == provenance.sha256_path(run[1])
    assert (s["t_start"], s["t_end"]) == (100, 104)
    assert t["tum_sha256"] == provenance.sha256_path(run[0] / "trajectory.tum")
    assert t["method_version"] == m["method_version"] == FASTLIO
    assert t["params_sha256"] == m["params_sha256"] == json.loads((run[0] / "manifest.json").read_text())["config_sha256"]
    assert t["parent_revision_id"] is None and t["revision"] == m["revision"] == 1
    assert m["trajectory_revision_id"] == result.trajectory_revision_id
    assert m["map_pcd_sha256"] == provenance.sha256_path(run[0] / "map.pcd")
    assert all(o["map_revision_id"] == result.map_revision_id and o["method_version"] == CODE
               and o["acquired_at"] == 104 and o["recorded_txn"] == result.transaction_id
               and o["sensor_kind"] == "lidar" and o["geometry_frame"] == "map"
               and o["expires_at"] is None for o in observations(store))
    assert s["recorded_txn"] == t["recorded_txn"] == m["recorded_txn"] == result.transaction_id
    assert store.connection.execute("SELECT count(*) FROM interpretations").fetchone()[0] == 0
    assert not store.connection.execute("SELECT name FROM sqlite_master WHERE name LIKE 'walkday_%'").fetchall()


def test_reingest_identical_writes_nothing(store, run):
    first = ingest(store, run)
    before = snapshot(store)
    hashes = [store.row_sha256("observations", i) for i in first.observation_ids]
    second = ingest(store, run)
    assert second.transaction_id is None and second.inserted_observation_ids == ()
    assert second.observation_ids == first.observation_ids
    assert store.latest_txn() == first.transaction_id
    assert snapshot(store) == before
    assert hashes == [store.row_sha256("observations", i) for i in first.observation_ids]


def test_regenerated_report_same_content_is_noop(store, run):
    ingest(store, run)
    before = snapshot(store)
    def regenerate(data):
        data["created_at"] = "different generation time"
        data["map_path"] = "/untrusted/elsewhere"
        for p in data["patches"]:
            p["figure"] = "new-render.png"
    edit(run[0] / "wallpatch/wallpatch.json", regenerate)
    assert ingest(store, run).transaction_id is None
    assert snapshot(store) == before


@pytest.mark.parametrize("change", ["record", "code", "params"])
def test_reingest_conflict_refused(store, run, change):
    ingest(store, run)
    before = snapshot(store)
    if change == "record":
        edit(run[0] / "wallpatch/wallpatch.json", lambda d: d["patches"][0]["residuals"].update(p95_abs_m=.01))
    if change == "params":
        edit(run[0] / "wallpatch/wallpatch.json", lambda d: d["gates"].update(label="changed"))
    with pytest.raises(IngestRefused, match="conflicting"):
        ingest(store, run, code="3" * 40 if change == "code" else CODE)
    assert snapshot(store) == before


def test_late_result_file_appends_new_txn(store, run):
    first = ingest(store, run, clearance=False)
    second = ingest(store, run)
    assert second.transaction_id > first.transaction_id
    assert len(second.inserted_observation_ids) == 1
    assert second.trajectory_revision_id == first.trajectory_revision_id
    assert second.map_revision_id == first.map_revision_id
    assert len(observations(store)) == 3


def test_reingest_retracted_session_refused_and_history_preserved(store, run):
    first = ingest(store, run)
    a, b = first.observation_ids[:2]
    comparison = store.compare_positions(a, b, first.transaction_id)
    store.retract("session", first.session_id, "bad source", "operator")
    before = snapshot(store)
    with pytest.raises(IngestRefused, match="evidence is retracted; it needs an operator"):
        ingest(store, run)
    assert snapshot(store) == before
    assert store.is_invalid("observation", a)
    assert not store.is_invalid("observation", a, first.transaction_id)
    assert store.compare_positions(a, b, first.transaction_id) == comparison
    assert not store.compare_positions(a, b).comparable


def test_retracted_recording_under_new_run_id_refused(store, run):
    first = ingest(store, run)
    store.retract("session", first.session_id, "bad source", "operator")
    for name in ("manifest.json", "walk_result.json"):
        edit(run[0] / name, lambda d: d.update(run_id="walk-two"))
    before = snapshot(store)
    # The derived_from key names the recording, not the session: re-processing
    # the same recording under a new run ID is not a way around a retraction.
    with pytest.raises(IngestRefused, match="conflicting derived_from"):
        ingest(store, run)
    assert snapshot(store) == before


def test_supersession_and_known_at_on_ingested_observations(store, run):
    first = ingest(store, run)
    a, b = first.observation_ids[:2]
    before = store.compare_positions(a, b, first.transaction_id)
    hashes = [store.row_sha256("observations", i) for i in (a, b)]
    store.add_trajectory_revision(first.session_id, 2, "e" * 64, first.trajectory_revision_id,
                                  "correction", CODE, "d" * 64, "test correction")
    assert not store.compare_positions(a, b).comparable
    assert store.compare_positions(a, b, first.transaction_id) == before
    assert hashes == [store.row_sha256("observations", i) for i in (a, b)]


@pytest.mark.parametrize("relative", ["bag/metadata.yaml", "bag/walk.db3"])
def test_recording_bytes_mismatch_refused(store, run, relative):
    (run[1] / relative).write_bytes(b"changed raw input")
    before = snapshot(store)
    with pytest.raises(IngestRefused, match="source_fingerprint"):
        ingest(store, run)
    assert snapshot(store) == before


@pytest.mark.parametrize("code", [None, "", "a" * 39, "A" * 40, "a" * 64])
def test_missing_code_revision_refused(store, run, code):
    with pytest.raises(IngestRefused, match="code_revision"):
        ingest(store, run, code=code)
    assert store.latest_txn() == 0


@pytest.mark.parametrize("field", ["fastlio_revision", "source_fingerprint", "config_sha256"])
def test_missing_manifest_provenance_refused(store, run, field):
    edit(run[0] / "manifest.json", lambda d: d.pop(field))
    with pytest.raises(IngestRefused, match=field):
        ingest(store, run)
    assert store.latest_txn() == 0


@pytest.mark.parametrize("name", ["trajectory.tum", "map.pcd"])
def test_manifest_artifact_hash_mismatch_refused(store, run, name):
    with (run[0] / name).open("ab") as f:
        f.write(b" ")
    with pytest.raises(IngestRefused, match="sha256 mismatch"):
        ingest(store, run)
    assert store.latest_txn() == 0


@pytest.mark.parametrize("field,value", [("run_id", "other"), ("fastlio_revision", "f" * 40),
                                         ("config_sha256", "f" * 64)])
def test_walk_result_must_match_manifest(store, run, field, value):
    edit(run[0] / "walk_result.json", lambda d: d.update({field: value}))
    with pytest.raises(IngestRefused, match="differs from manifest"):
        ingest(store, run)
    assert store.latest_txn() == 0


@pytest.mark.parametrize("missing,message", [(True, "missing regular input file: .*walk_result.json"),
                                             (False, "unscored or NOT_EVALUABLE run")])
def test_unscored_or_not_evaluable_run_refused(store, run, missing, message):
    path = run[0] / "walk_result.json"
    if missing:
        path.unlink()
    else:
        def not_evaluable(d):
            d["status"] = "NOT_EVALUABLE"
            d["reasons"] = ["return error unmeasured"]
            d["return_error"]["d3_m"] = None
        edit(path, not_evaluable)
        parse_walk_result(json.loads(path.read_text()), str(path))
    with pytest.raises(IngestRefused, match=message):
        ingest(store, run)
    assert store.latest_txn() == 0


def test_synthetic_recording_refused(store, run):
    write(run[1] / "SYNTHETIC.json", {})
    with pytest.raises(IngestRefused, match="synthetic"):
        ingest(store, run)
    assert store.latest_txn() == 0


def test_wallpatch_map_hash_mismatch_refused(store, run):
    edit(run[0] / "wallpatch/wallpatch.json", lambda d: d.update(map_sha256="f" * 64))
    with pytest.raises(IngestRefused, match="wallpatch map"):
        ingest(store, run)
    assert store.latest_txn() == 0


@pytest.mark.parametrize("problem,message", [
    ("limitations", "provenance limitations must be empty"),
    ("recording", "recording sha256 [0-9a-f]{64} is not the results'"),
    ("trajectory", "trajectory changed"),
    ("geometry", "zone geometry changed"),
    ("time", "acquisition times outside trajectory session"),
    ("coverage", "free coverage disagrees with evidence counts")])
def test_clearance_bad_provenance_or_measurement_refused(store, run, problem, message):
    def corrupt(d):
        if problem == "limitations":
            d["provenance"]["limitations"] = ["no_time_basis"]
        elif problem in ("recording", "trajectory"):
            d["provenance"][problem + "_sha256"] = "f" * 64
        elif problem == "geometry":
            d["zones"][0]["zone_geometry_sha256"] = "f" * 64
        elif problem == "time":
            d["scan_time_range"] = [99, 104]
        else:
            next(z for z in d["zones"] if z["status"] == "PASS")["free_coverage"] = .5
    edit(run[2][0], corrupt)
    before = snapshot(store)
    with pytest.raises(IngestRefused, match=message):
        ingest(store, run)
    assert snapshot(store) == before


@pytest.mark.parametrize("change,message", [
    ("scans file", "scans changed"),
    ("extrinsics file", "extrinsics changed"),
    ("recorded scans", "scans changed"),
    ("recorded extrinsics", "extrinsics changed")])
def test_clearance_scans_or_extrinsics_mismatch_refused(store, run, change, message):
    # The results' own scans/extrinsics hashes must not be their only check.
    if change == "scans file":
        edit(run[2][2], lambda d: d["scans"][0].update(timestamp=100.5))
    elif change == "extrinsics file":
        run[2][3].write_text("lidar_to_base: recalibrated")
    else:
        name = change.split()[1] + "_sha256"
        edit(run[2][0], lambda d: d["provenance"].update({name: "1" * 64}))
    before = snapshot(store)
    with pytest.raises(IngestRefused, match=message):
        ingest(store, run)
    assert snapshot(store) == before


@pytest.mark.parametrize("change", ["scans", "extrinsics"])
def test_clearance_rederived_from_changed_inputs_conflicts(store, run, change):
    first = ingest(store, run)
    rows = observations(store)
    before = snapshot(store)
    if change == "scans":
        # A scan with no hits adds no rays: every zone record is unchanged.
        edit(run[2][2], lambda d: d["scans"].append(
            {"timestamp": 103.5, "origin": [50, 50, 50], "hits": []}))
    else:
        run[2][3].write_text("lidar_to_base: recalibrated")
    old = json.loads(run[2][0].read_text())
    write(run[2][0], classify(run[0], run[1], run[2]))
    assert json.loads(run[2][0].read_text())["zones"] == old["zones"]
    with pytest.raises(IngestRefused, match="conflicting derived_from item: .*source:clearance"):
        ingest(store, run)
    assert snapshot(store) == before and observations(store) == rows
    assert store.latest_txn() == first.transaction_id


@pytest.mark.parametrize("count", [2, 3, 5])
def test_clearance_without_scans_and_extrinsics_refused(store, run, tmp_path, count):
    paths = (*run[2], tmp_path / "extra.json")[:count]
    with pytest.raises(IngestRefused, match="clearance must be .*scans.json, extrinsics"):
        ingest_walkday(store, run[0], run[1], CODE, paths)
    assert store.latest_txn() == 0


def test_only_measured_patches_and_pass_zones_ingested(store, run):
    result = ingest(store, run)
    rows = observations(store)
    assert [r["kind"] for r in rows] == ["detection", "detection", "absence"]
    assert len(result.skipped) == 4
    assert rows[-1]["coverage"] == 1.0 and rows[-1]["sigma_m"] == .02
    report = json.loads((run[0] / "wallpatch/wallpatch.json").read_text())
    assert rows[0]["sigma_m"] == report["patches"][0]["residuals"]["p95_abs_m"]


def test_oriented_patch_box_stored_as_enclosing_aabb(store, run):
    def rotate(d):
        d["patches"][0]["box_map_frame"] = {
            "center_m": [2, 3, 4], "normal_hint": [1, 1, 0], "half_extent_m": [2, 1, .5]}
    edit(run[0] / "wallpatch/wallpatch.json", rotate)
    ingest(store, run)
    row = observations(store)[0]
    bounds = [row[k] for k in ("aabb_min_x", "aabb_min_y", "aabb_min_z",
                              "aabb_max_x", "aabb_max_y", "aabb_max_z")]
    span = 3 / np.sqrt(2)
    assert bounds == pytest.approx([2-span, 3-span, 3.5, 2+span, 3+span, 4.5])
    # Independent geometric construction, not the implementation's box_axes.
    corners = [np.array([2, 3, 4]) + a*np.array([-2, 2, 0])/np.sqrt(2)
               + b*np.array([1, 1, 0])/np.sqrt(2) + c*np.array([0, 0, .5])
               for a, b, c in itertools.product((-1, 1), repeat=3)]
    assert all(np.all(p >= np.array(bounds[:3])-1e-12) and
               np.all(p <= np.array(bounds[3:])+1e-12) for p in corners)


def test_series_output_refused_as_evidence(store, run, tmp_path):
    series = tmp_path / "series"
    series.mkdir()
    write(series / "series.json", {"schema": "spatio.series/1"})
    with pytest.raises(IngestRefused, match="series output: aggregate verdicts, no geometry"):
        ingest_walkday(store, series, run[1], CODE)
    with pytest.raises(IngestRefused, match="clearance schema"):
        ingest_walkday(store, run[0], run[1], CODE, (series / "series.json", *run[2][1:]))
    assert store.latest_txn() == 0


def test_refusal_leaves_no_rows_or_txn_even_after_write_started(store, run, monkeypatch):
    real = store.add_observation
    calls = []
    def failing(**kwargs):
        calls.append(kwargs)
        if len(calls) == 2:
            raise ValidationError("injected observation failure")
        return real(**kwargs)
    monkeypatch.setattr(store, "add_observation", failing)
    before = snapshot(store)
    with pytest.raises(ValidationError, match="injected"):
        ingest(store, run)
    assert len(calls) == 2 and snapshot(store) == before


def test_ingested_observations_comparable_rule2(store, run):
    result = ingest(store, run)
    a, b = result.observation_ids[:2]
    comparison = store.compare_positions(a, b)
    assert comparison.comparable and comparison.frame == result.map_revision_id
    assert comparison.distance_m == 0
    assert comparison.revisions_used == {"map_revisions": [result.map_revision_id],
                                          "trajectory_revisions": [result.trajectory_revision_id]}


def test_inputs_unmodified(store, run):
    paths = [p for root in run[:2] for p in root.rglob("*") if p.is_file()] + list(run[2])
    before = [(p.read_bytes(), p.stat().st_mtime_ns) for p in paths]
    hashes = [provenance.sha256_path(p) for p in run[:2]]
    ingest(store, run)
    assert before == [(p.read_bytes(), p.stat().st_mtime_ns) for p in paths]
    assert hashes == [provenance.sha256_path(p) for p in run[:2]]


@pytest.mark.parametrize("raw,message", [('{"schema":1,"schema":2}', "duplicate JSON key: schema"),
                                         ('{"x":NaN}', "nonfinite JSON value: NaN"),
                                         ('{"x":1e999}', "Out of range float")])
def test_ambiguous_or_nonfinite_json_refused(store, run, raw, message):
    (run[0] / "walk_result.json").write_text(raw)
    with pytest.raises(IngestRefused, match=message):
        ingest(store, run)
    assert store.latest_txn() == 0


def test_symlink_and_special_file_refused(store, run):
    link = run[1] / "link"
    link.symlink_to(run[0] / "map.pcd")
    with pytest.raises(IngestRefused, match="symlink"):
        ingest(store, run)
    link.unlink()
    os.mkfifo(link)
    with pytest.raises(IngestRefused, match="special file"):
        ingest(store, run)
    assert store.latest_txn() == 0


def test_unreadable_recording_subtree_refused(store, run, monkeypatch):
    real = os.walk
    def broken(path, *, onerror):
        if path == run[1]:
            onerror(PermissionError("unreadable subtree"))
        return real(path, onerror=onerror)
    monkeypatch.setattr("spatio.memory.ingest.os.walk", broken)
    with pytest.raises(IngestRefused, match="unreadable subtree"):
        ingest(store, run)
    assert store.latest_txn() == 0


def test_null_bag_hash_manifest_matches_writer_contract(store, run):
    edit(run[0] / "manifest.json", lambda d: d["source_fingerprint"]["bag_files"][0].update(sha256=None))
    assert ingest(store, run).transaction_id == 1


def test_cli_success_noop_and_input_database_protection(run, tmp_path):
    database = tmp_path / "cli.sqlite"
    command = [sys.executable, "-m", "spatio.memory.ingest", str(run[0]),
               "--recording-dir", str(run[1]), "--code-revision", CODE, "--database", str(database)]
    first = subprocess.run(command, capture_output=True, text=True, stdin=subprocess.DEVNULL)
    assert first.returncode == 0, first.stderr
    second = subprocess.run(command, capture_output=True, text=True, stdin=subprocess.DEVNULL)
    assert second.returncode == 0 and json.loads(second.stdout)["transaction_id"] is None
    bad = run[1] / "new.sqlite"
    refused = subprocess.run(command[:-1] + [str(bad)], capture_output=True, text=True, stdin=subprocess.DEVNULL)
    assert refused.returncode == 2 and "database" in refused.stderr and not bad.exists()
    hardlink = run[1] / "hardlink"
    os.link(database, hardlink)
    refused = subprocess.run(command, capture_output=True, text=True, stdin=subprocess.DEVNULL)
    assert refused.returncode == 2 and "hard link" in refused.stderr


def test_cli_clearance_rehashes_scans_and_extrinsics(run, tmp_path):
    command = [sys.executable, "-m", "spatio.memory.ingest", str(run[0]),
               "--recording-dir", str(run[1]), "--code-revision", CODE,
               "--database", str(tmp_path / "cli.sqlite"), "--clearance", *map(str, run[2])]
    first = subprocess.run(command, capture_output=True, text=True, stdin=subprocess.DEVNULL)
    assert first.returncode == 0, first.stderr
    assert len(json.loads(first.stdout)["inserted_observation_ids"]) == 3
    run[2][3].write_text("lidar_to_base: recalibrated")
    refused = subprocess.run(command, capture_output=True, text=True, stdin=subprocess.DEVNULL)
    assert refused.returncode == 2 and "extrinsics changed" in refused.stderr


def test_changed_measurement_to_skipped_is_conflict(store, run):
    ingest(store, run)
    before = snapshot(store)
    edit(run[0] / "wallpatch/wallpatch.json",
         lambda d: d["patches"][0].update(status="NOT_EVALUABLE"))
    with pytest.raises(IngestRefused, match="conflicting derived_from"):
        ingest(store, run)
    assert snapshot(store) == before


def test_preexisting_native_ids_are_reused(store, run):
    store.add_session("unrelated", 0, 1, "f" * 64)
    other = store.add_trajectory_revision("unrelated", 1, "e" * 64, None,
                                          "test", CODE, "d" * 64, "test")
    store.add_map_revision("unrelated", 1, "c" * 64, other, "test", "test", CODE, "d" * 64)
    first = ingest(store, run, clearance=False)
    assert first.trajectory_revision_id == 2 and first.map_revision_id == 2
    result = ingest(store, run)
    assert result.trajectory_revision_id == first.trajectory_revision_id
    assert result.map_revision_id == first.map_revision_id
    assert observations(store)[-1]["map_revision_id"] == 2


def test_preexisting_observation_values_conflict_before_new_rows(store, run):
    result = ingest(store, run, clearance=False)
    row = observations(store)[0]
    # V1 allows arbitrary derived_from strings: duplicate keys outside this
    # importer must not bypass its stored-value consistency check.
    store.add_observation(result.session_id, "detection", "lidar", "map", 104,
                          [0, 0, 0, 1, 1, 1], .1, 1,
                          map_revision_id=result.map_revision_id, derived_from=row["derived_from"])
    before = snapshot(store)
    with pytest.raises(IngestRefused, match="conflicting derived_from"):
        ingest(store, run)
    assert snapshot(store) == before


@pytest.mark.parametrize("content,message", [
    ("", "at least two strictly increasing stamps"),
    ("104 0 0 0 0 0 0 1\n100 1 0 0 0 0 0 1\n", "at least two strictly increasing stamps"),
    ("100 0 0 0 0 0 0 0\n104 1 0 0 0 0 0 0 1\n", "quaternion norm 0")])
def test_tum_is_parsed_even_with_matching_hash(store, run, content, message):
    (run[0] / "trajectory.tum").write_text(content)
    edit(run[0] / "manifest.json", lambda d: d["files"]["trajectory.tum"].update(
        sha256=provenance.sha256_path(run[0] / "trajectory.tum")))
    with pytest.raises(IngestRefused, match=message):
        ingest(store, run, clearance=False)
    assert store.latest_txn() == 0


def test_changed_session_times_refused(store, run):
    ingest(store, run, clearance=False)
    before = snapshot(store)
    (run[0] / "trajectory.tum").write_text("100 0 0 0 0 0 0 1\n105 1 0 0 0 0 0 1\n")
    edit(run[0] / "manifest.json", lambda d: d["files"]["trajectory.tum"].update(
        sha256=provenance.sha256_path(run[0] / "trajectory.tum")))
    with pytest.raises(IngestRefused, match="conflicting sessions"):
        ingest(store, run, clearance=False)
    assert snapshot(store) == before


def test_fail_scored_walk_can_supply_measured_geometry(store, run):
    edit(run[0] / "walk_result.json", lambda d: d.update(status="FAIL", reasons=["failed walk"]))
    assert len(ingest(store, run).observation_ids) == 3


def test_map_directory_is_not_a_map_file_even_with_matching_tree_hash(store, run):
    path = run[0] / "map.pcd"
    path.unlink()
    path.mkdir()
    (path / "data").write_bytes(b"not a map file")
    digest = provenance.sha256_path(path)
    edit(run[0] / "manifest.json", lambda d: d["files"]["map.pcd"].update(sha256=digest))
    edit(run[0] / "wallpatch/wallpatch.json", lambda d: d.update(map_sha256=digest))
    with pytest.raises(IngestRefused, match="map.pcd must be a regular file"):
        ingest(store, run)
    assert store.latest_txn() == 0


def test_same_size_bag_edit_fails_manifest_hash_binding(store, run):
    path = run[1] / "bag/walk.db3"
    original = path.read_bytes()
    path.write_bytes(b"X" + original[1:])
    with pytest.raises(IngestRefused, match="bag hash mismatch"):
        ingest(store, run)
    assert store.latest_txn() == 0
