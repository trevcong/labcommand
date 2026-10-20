"""Regression evidence uses synthetic recordings, never a historical walk."""

import json
from datetime import datetime, timedelta, timezone

import pytest

from spatio.series.cli import main
from spatio.series.report import build_series, default_series_id
from spatio.series.validate import InputRejected, load_walk_result, parse_walk_result
from spatio.series.tests.test_series import make_walk, make_return_fail, make_replay


def timed(data, minute):
    data["recorded_at"] = (
        datetime(2030, 1, 1, tzinfo=timezone.utc) + timedelta(minutes=minute)
    ).isoformat()
    return data


def write_result(tmp_path, data):
    path = tmp_path / (data["run_id"] + ".json")
    path.write_text(json.dumps(data), encoding="utf-8")
    return str(path)


@pytest.mark.parametrize("missing", ["start_window_s", "end_window_s"])
@pytest.mark.parametrize("status", ["FAIL", "NOT_EVALUABLE"])
def test_zero_duration_checks_remaining_endpoint_window(missing, status):
    # Positive-duration checks already exist on main. Zero used to bypass them
    # when null disabled the paired-span check and no motion was claimed.
    data = make_walk(
        status=status, reasons=["endpoint timing invalid"], duration_s=0,
        path_length_m=0, validity={"moving": False},
        return_error={missing: None, "d3_m": None if status == "NOT_EVALUABLE" else 0,
                      "horizontal_m": None, "vertical_m": None},
    )
    present = "end_window_s" if missing == "start_window_s" else "start_window_s"
    with pytest.raises(InputRejected, match=present + " covers"):
        parse_walk_result(data, "zero.json")


@pytest.mark.parametrize("verdict,code", [
    ("PASS", 0), ("FAIL", 3), ("INCOMPLETE", 4),
    ("NOT_EVALUABLE", 5), ("REPLAY_ONLY", 6),
])
def test_compare_exit_matches_written_verdict(tmp_path, verdict, code):
    data = [timed(make_walk(f"r{i}"), i) for i in range(3)]
    if verdict == "FAIL":
        data[1] = timed(make_return_fail("failed"), 1)
    elif verdict == "INCOMPLETE":
        data = data[:2]
    elif verdict == "NOT_EVALUABLE":
        data[1]["config_sha256"] = "f" * 64
    elif verdict == "REPLAY_ONLY":
        data = [timed(make_replay(f"h{i}"), i) for i in range(3)]
    paths = [write_result(tmp_path, item) for item in data]
    out = tmp_path / "report"
    assert main(["compare", *paths, "--out", str(out)]) == code
    assert json.loads((out / "series.json").read_text())["verdict"] == verdict
    assert f"Verdict: {verdict}" in (out / "series.md").read_text()


def test_recording_order_prevents_cli_from_hiding_intervening_failure():
    data = [timed(make_walk("first"), 0), timed(make_return_fail("failed"), 1),
            timed(make_walk("third"), 2), timed(make_walk("fourth"), 3)]
    walks = [parse_walk_result(data[i], f"{i}.json") for i in (0, 2, 3, 1)]
    report = build_series(walks, "ordered")
    assert report["verdict"] == "FAIL"
    assert [r["run_id"] for r in report["runs"]] == ["first", "failed", "third", "fourth"]
    assert default_series_id(walks) == "first--fourth"
    assert report["runs"][1]["recorded_at"] == data[1]["recorded_at"]


def test_recording_order_uses_timezone_and_earliest_triple():
    data = [timed(make_walk(f"r{i}"), i) for i in range(4)]
    data[0]["recorded_at"] = "2029-12-31T18:00:00-06:00"
    walks = [parse_walk_result(data[i], f"{i}.json") for i in (3, 1, 0, 2)]
    report = build_series(walks, "ordered")
    assert report["counted_run_ids"] == ["r0", "r1", "r2"]


@pytest.mark.parametrize("stamp", [None, "", "yesterday", "2030-01-01T00:00:00",
                                  True, 123])
def test_compare_requires_explicit_timezone_aware_recording_time(stamp):
    with pytest.raises(InputRejected, match="recorded_at"):
        walk = parse_walk_result(make_walk(recorded_at=stamp), "missing-time.json")
        build_series([walk], "missing-time")


def test_equal_recording_times_cannot_fall_back_to_cli_order():
    walks = [parse_walk_result(timed(make_walk(name), 0), name) for name in ("a", "b")]
    with pytest.raises(InputRejected, match="recorded_at"):
        build_series(walks, "ties")


def test_legacy_explain_without_recorded_time_remains_available(tmp_path):
    assert main(["explain", write_result(tmp_path, make_walk())]) == 0


def produced_walkcheck(tmp_path, *, offset=0.05, variant=0, physical=False,
                       map_kind="sharp", gap=False):
    import numpy as np
    from spatio.walkcheck.evaluate import evaluate_session
    from spatio.walkcheck.synthetic import (
        walking_loop, write_session, four_wall_room, doubled_wall, insert_time_gap,
    )
    from spatio.wallpatch.cli import main as wallpatch_main
    from spatio.walkday.tests.test_result import quiet, write_spec
    metadata = timed(make_walk(f"synthetic-{variant}"), variant)
    metadata = {key: metadata[key] for key in (
        "run_id", "recorded_at", "fastlio_revision", "config_sha256",
        "physical_accuracy_evaluated", "physical_reference",
    )}
    metadata["physical_accuracy_evaluated"] = physical
    if not physical:
        metadata["physical_reference"] = None
    # Change clock origin, route geometry, rate and walking speeds; no old bags.
    rows = walking_loop(side=2.5 + variant, offset=(offset, 0, 0),
                        speeds=(0.3, 0.6, 0.8, 0.5), rate=8 + variant,
                        origin=12345 + 1000 * variant)
    if gap:
        rows = insert_time_gap(rows, after_t=rows[0][0] + 12, extra_s=2)
    xyz = {
        "sharp": lambda: four_wall_room(seed=variant),
        "doubled": lambda: np.vstack([four_wall_room(seed=variant),
                                      doubled_wall(count_per_sheet=1500, seed=variant)]),
        "tiny": lambda: [[0, 0, 0], [1, 0, 0]],
        # Enough points for walkcheck's map minimum (#72) but no wall plane.
        "planeless": lambda: np.random.default_rng(variant).uniform(-3, 3, (200, 3)).tolist(),
        "missing": lambda: None,
    }[map_kind]()
    session = tmp_path / metadata["run_id"]
    write_session(session, rows, xyz=xyz, manifest=metadata)
    report = evaluate_session(session, write=False)
    report_path = session / "walkcheck.json"
    report_path.write_text(json.dumps(report), encoding="utf-8")
    if map_kind in ("sharp", "doubled"):
        # Supply actual measured patches and an explicit test spec, as the
        # shared rule requires. Suggestions alone are tested separately.
        patch_file = session / "patches.json"
        assert quiet(wallpatch_main, ["suggest", str(session / "map.pcd"),
                                      "--n", "2", "--out", str(patch_file)]) == 0
        patches = json.loads(patch_file.read_text())["patches"]
        _, sha = write_spec(session / "spec.json", [p["name"] for p in patches],
                            [p["box_map_frame"] for p in patches])
        assert quiet(wallpatch_main, ["measure", str(session), "--patches", str(patch_file),
                                      "--out", str(session / "wallpatch")]) == 0
        if physical:
            metadata["physical_reference"]["spec_sha256"] = sha
    meta_path = session / "metadata.json"
    meta_path.write_text(json.dumps(metadata), encoding="utf-8")
    return report, metadata, report_path, meta_path


def wall_args(report_path):
    session = report_path.parent
    if not (session / "spec.json").exists():
        return []
    return ["--wallpatch", str(session / "wallpatch" / "wallpatch.json"),
            "--spec", str(session / "spec.json"), "--patches", str(session / "patches.json"),
            "--patch-origin", "preselected"]


def wall_kwargs(report_path):
    session = report_path.parent
    if not (session / "spec.json").exists():
        return {}
    return dict(wallpatch=json.loads((session / "wallpatch" / "wallpatch.json").read_text()),
                spec_path=str(session / "spec.json"), patches_file=str(session / "patches.json"),
                patch_origin="preselected")


@pytest.mark.parametrize("offset,variant,status", [(0.05, 0, "PASS"), (0.15, 2, "FAIL")])
def test_adapter_consumes_real_walkcheck_output(tmp_path, offset, variant, status):
    report, metadata, report_path, meta_path = produced_walkcheck(
        tmp_path, offset=offset, variant=variant
    )
    assert report["schema"] == "spatio.walkcheck.v1"
    assert report["result"] == status
    out = tmp_path / "walk_result.json"
    assert main(["adapt", str(report_path), "--metadata", str(meta_path), "--out", str(out),
                 *wall_args(report_path)]) == 0
    walk = load_walk_result(str(out))
    assert walk.status == status
    assert walk.return_error.d3_m == pytest.approx(offset)
    assert walk.return_error.horizontal_m == pytest.approx(offset)
    assert walk.return_error.vertical_m == pytest.approx(0)
    assert walk.return_error.start_window_s == report["return_to_start"]["start_window_s"]
    assert walk.walls.p95_point_to_plane_m == pytest.approx(report["walls"]["p95_point_to_plane_cm"] / 100)
    assert walk.validity.max_gap_s == report["timestamps"]["max_dt_s"]
    assert walk.config_sha256 == metadata["config_sha256"]
    assert walk.physical_accuracy_evaluated is False
    assert build_series([walk], "historical")["verdict"] == "REPLAY_ONLY"
    assert walk.accuracy_label == "replay demonstrated; physical accuracy not evaluated"


@pytest.mark.parametrize("map_kind,gap,producer,adapted", [
    ("sharp", True, "NOT-EVALUABLE", "NOT_EVALUABLE"),
    ("missing", False, "NOT-EVALUABLE", "NOT_EVALUABLE"),
    # Since #72 walkcheck itself says NOT-EVALUABLE for a 2-point map; this row
    # was written against the pre-#72 producer (PASS). "planeless" keeps the
    # real-output case of a producer PASS whose wall evidence is missing.
    ("tiny", False, "NOT-EVALUABLE", "NOT_EVALUABLE"),
    ("planeless", False, "PASS", "NOT_EVALUABLE"),
    ("doubled", False, "PASS", "FAIL"),
])
def test_adapter_never_upgrades_missing_or_failed_evidence(tmp_path, map_kind, gap, producer, adapted):
    from spatio.series.adapter import adapt_walkcheck

    report, metadata, path, _ = produced_walkcheck(tmp_path, map_kind=map_kind, gap=gap)
    assert report["result"] == producer
    data = adapt_walkcheck(report, metadata, str(path), **wall_kwargs(path))
    assert data["walkcheck_result"] == producer
    assert data["status"] == adapted
    walk = parse_walk_result(data, str(path))
    assert walk.reasons
    if gap:
        assert walk.return_error.d3_m is None
        assert walk.validity.moving is None
    if map_kind in ("tiny", "planeless", "missing"):
        assert walk.walls.doubled_wall_suspected is None


def test_adapter_end_to_end_three_varied_synthetic_walks(tmp_path):
    # "physical=True" is synthetic reference metadata, not physical validation.
    paths = []
    for i in range(3):
        _, _, report_path, meta_path = produced_walkcheck(
            tmp_path, variant=i, offset=0.03 + 0.02 * i, physical=True
        )
        out = tmp_path / f"adapted-{i}.json"
        assert main(["adapt", str(report_path), "--metadata", str(meta_path), "--out", str(out),
                     *wall_args(report_path)]) == 0
        paths.append(str(out))
    outdir = tmp_path / "series"
    assert main(["compare", *reversed(paths), "--out", str(outdir)]) == 0
    report = json.loads((outdir / "series.json").read_text())
    assert report["counted_run_ids"] == ["synthetic-0", "synthetic-1", "synthetic-2"]
    assert report["series_id"] == "synthetic-0--synthetic-2"


def test_adapter_requires_metadata_without_guessing(tmp_path):
    from spatio.series.adapter import adapt_walkcheck

    report, metadata, path, _ = produced_walkcheck(tmp_path)
    for key in metadata:
        incomplete = {k: v for k, v in metadata.items() if k != key}
        with pytest.raises(InputRejected, match=key):
            adapt_walkcheck(report, incomplete, str(path))
    for key, value in (("run_id", "different-walk"), ("physical_accuracy_evaluated", True),
                       ("config_sha256", None), ("recorded_at", None)):
        with pytest.raises(InputRejected, match=key):
            adapt_walkcheck(report, dict(metadata, **{key: value}), str(path))
    # A report generation time must never silently substitute for recording time.
    assert report["created_at"] != metadata["recorded_at"]


@pytest.mark.parametrize("status,code", [("PASS", 0), ("FAIL", 3), ("NOT_EVALUABLE", 5)])
def test_explain_exit_codes(tmp_path, status, code):
    data = make_walk(status=status, reasons=[] if status == "PASS" else ["synthetic"])
    if status == "NOT_EVALUABLE":
        data["return_error"]["d3_m"] = None
    assert main(["explain", write_result(tmp_path, data)]) == code
