"""S2 persistence failures and the explicit missing-measurement hand-off."""

import json
import shutil
from pathlib import Path

import numpy as np
import pytest

from spatio.manifest import jsonio
from spatio.series.adapter import adapt_walkcheck
from spatio.series.validate import InputRejected
from spatio.walkcheck.cli import main as walkcheck_main
from spatio.walkcheck.report import plain, render_markdown, write_json
from spatio.walkcheck.values import measurement_values
from spatio.walkday import result
from spatio.walkday.tests.test_result import quiet, write_gate, write_recording
from spatio.wallpatch.cli import main as wallpatch_main

FIXTURES = Path(__file__).resolve().parents[2] / "io" / "fixtures"


@pytest.mark.parametrize("earlier", [False, True])
@pytest.mark.parametrize("failure", [OSError, KeyboardInterrupt])
def test_partial_dump_preserves_result_and_cleans_temp(tmp_path, monkeypatch, earlier, failure):
    target = tmp_path / "walk_result.json"
    previous = b'{"earlier": true}\n'
    if earlier:
        target.write_bytes(previous)

    def interrupted_dump(obj, handle, **kwargs):
        handle.write('{"partial": ')
        handle.flush()
        raise failure("interrupted during JSON output")

    monkeypatch.setattr(jsonio.json, "dump", interrupted_dump)
    with pytest.raises(failure):
        result._write(tmp_path, target.name, {"status": "PASS"})
    assert target.read_bytes() == previous if earlier else not target.exists()
    assert set(tmp_path.iterdir()) == ({target} if earlier else set())


@pytest.mark.parametrize("operation", ["fsync", "replace"])
def test_atomic_os_failure_preserves_old_output(tmp_path, monkeypatch, operation):
    target = tmp_path / "walk_result.json"
    target.write_bytes(b'{"earlier": true}\n')

    def fail(*args):
        raise OSError("injected " + operation)

    monkeypatch.setattr(jsonio.os, operation, fail)
    with pytest.raises(OSError, match=operation):
        result._write(tmp_path, target.name, {"new": True})
    assert target.read_bytes() == b'{"earlier": true}\n'
    assert list(tmp_path.iterdir()) == [target]


@pytest.mark.parametrize("value", [float("nan"), float("inf"), -float("inf"), np.float32("nan")])
def test_nonfinite_report_never_becomes_null(tmp_path, value):
    target = tmp_path / "walkcheck.json"
    target.write_bytes(b'{"earlier": true}\n')
    report = {"return_to_start": {"error_3d_m": value}}
    with pytest.raises(ValueError, match="non-finite"):
        plain(report)
    with pytest.raises(ValueError, match="non-finite"):
        write_json(target, report)
    assert target.read_bytes() == b'{"earlier": true}\n'
    assert list(tmp_path.iterdir()) == [target]


def test_result_writer_rejects_nan_and_unknown_report_types(tmp_path):
    with pytest.raises(ValueError):
        result._write(tmp_path, "walk_result.json", {"error": float("nan")})
    assert list(tmp_path.iterdir()) == []
    with pytest.raises(TypeError, match="unsupported report value"):
        plain({"error": object()})


@pytest.mark.parametrize("name,walkcheck_status", [
    ("map_sharp_wall", "PASS"),
    ("loop_5cm", "NOT-EVALUABLE"),
    ("loop_15cm", "NOT-EVALUABLE"),
    ("timestamp_reset", "NOT-EVALUABLE"),
    ("stationary_60s", "NOT-EVALUABLE"),
])
def test_fixture_verdicts_and_both_readers_with_explicit_missing_values(tmp_path, name, walkcheck_status):
    # Committed input fixtures, copied before any command writes reports.
    session = tmp_path / name
    shutil.copytree(FIXTURES / name, session)
    manifest = json.loads((session / "manifest.json").read_text())
    manifest.update(run_id=name, source_recording=str(write_recording(tmp_path / "rec")),
                    fastlio_revision="abc1234", config_sha256="ab" * 32)
    if name == "map_sharp_wall":
        # A PASS walk can lack a stop comparison, but a null error is not a
        # missing measurement. Supply one visit so the comparison is unavailable.
        start = float(np.loadtxt(session / "trajectory.tum")[0, 0])
        manifest["intermediate_stops"] = [
            {"name": "single_visit", "visits": [{"t0": start, "t1": start + 5}]},
        ]
    (session / "manifest.json").write_text(json.dumps(manifest))
    (session / "logs").mkdir()
    write_gate(session)
    assert quiet(walkcheck_main, [str(session)]) == {"PASS": 0, "FAIL": 1, "NOT-EVALUABLE": 2}[walkcheck_status]
    path = session / "walkcheck.json"
    explicit = json.loads(path.read_text())
    assert explicit["result"] == walkcheck_status
    assert '"status": "not_measured"' in path.read_text()
    assert explicit["intermediate_stops"]["error"] is None
    if name == "map_sharp_wall":
        stop = explicit["intermediate_stops"]["stops"][0]
        assert stop["visits_used"] == 1
        assert stop["max_mean_separation_cm"]["status"] == "not_measured"
    legacy = measurement_values(explicit)
    assert render_markdown(explicit) == render_markdown(legacy)

    kwargs = {}
    if name == "map_sharp_wall":
        patches = session / "wallpatch" / "patches.json"
        patches.parent.mkdir()
        assert quiet(wallpatch_main, ["suggest", str(session / "map.pcd"), "--n", "2", "--out", str(patches)]) == 0
        assert all(p["origin"] == "auto_suggested" for p in json.loads(patches.read_text())["patches"])
        assert quiet(wallpatch_main, ["measure", str(session), "--patches", str(patches),
                                    "--out", str(patches.parent)]) == 0
        kwargs = dict(wallpatch=json.loads((patches.parent / "wallpatch.json").read_text()),
                      patches_file=str(patches))
    metadata = dict(manifest, recorded_at="2030-01-01T00:00:00Z", physical_reference=None)
    adapted = adapt_walkcheck(explicit, metadata, str(path), **kwargs)
    assert adapted == adapt_walkcheck(legacy, metadata, str(path), **kwargs)
    explicit_walk, _ = result.build(str(session), .05, "approved", "auto suggestions")
    path.write_text(json.dumps(legacy))
    legacy_walk, _ = result.build(str(session), .05, "approved", "auto suggestions")
    assert explicit_walk == legacy_walk
    assert explicit_walk["status"] == adapted["status"] == "NOT_EVALUABLE"
    if name == "timestamp_reset":
        assert explicit["return_to_start"]["status"] == "not_measured"
        assert explicit["return_to_start"]["reason"]
        assert adapted["return_error"]["d3_m"] is None
        assert explicit_walk["return_error"]["d3_m"] is None


@pytest.mark.parametrize("bad", [
    {"status": "not_measured", "reason": ""},
    {"status": "not_measured", "reason": "missing", "value": 0.01},
    float("nan"),
    float("inf"),
])
def test_readers_reject_invalid_missing_or_nonfinite_values(tmp_path, bad):
    path = tmp_path / "walkcheck.json"
    data = {"return_to_start": {"error_3d_m": bad}}
    path.write_text(json.dumps(data))
    with pytest.raises(result.ResultError):
        result._load(path)
    with pytest.raises(InputRejected):
        adapt_walkcheck(data, {}, str(path))
