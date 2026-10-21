"""S2 persistence failures and the explicit missing-measurement hand-off."""

import json
import os
import shutil
from pathlib import Path

import numpy as np
import pytest

from spatio.manifest import jsonio
from spatio.series import cli as series_cli
from spatio.series.adapter import adapt_walkcheck
from spatio.series.tests.test_fixes import produced_walkcheck, timed, wall_args, write_result
from spatio.series.tests.test_series import make_walk
from spatio.series.validate import InputRejected
from spatio.walkcheck.cli import main as walkcheck_main
from spatio.walkcheck import report as report_module
from spatio.walkcheck.report import plain, render_markdown, write_json, write_markdown
from spatio.walkcheck.values import measurement_values
from spatio.walkday import gate, result
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


@pytest.mark.parametrize("earlier", [False, True])
@pytest.mark.parametrize("operation", ["fsync", "replace"])
@pytest.mark.parametrize("name,write", [
    ("walkday.md", lambda path: result._write(path.parent, path.name, "# new report\n")),
    ("walkcheck.md", lambda path: write_markdown(path, {})),
])
def test_markdown_os_failure_preserves_old_report(tmp_path, monkeypatch, earlier, operation, name, write):
    # Issue #102: Markdown results go through the same temp+fsync+replace helper.
    target = tmp_path / name
    previous = b"# earlier report\n"
    if earlier:
        target.write_bytes(previous)

    def fail(*args):
        raise OSError("injected " + operation)

    monkeypatch.setattr(report_module, "render_markdown", lambda report: "# new report\n")
    monkeypatch.setattr(jsonio.os, operation, fail)
    with pytest.raises(OSError, match=operation):
        write(target)
    assert target.read_bytes() == previous if earlier else not target.exists()
    assert set(tmp_path.iterdir()) == ({target} if earlier else set())


def test_walkcheck_writes_markdown_atomically(tmp_path, monkeypatch):
    session = tmp_path / "map_sharp_wall"
    shutil.copytree(FIXTURES / "map_sharp_wall", session)
    written = []
    real = jsonio.write_text_atomic

    def spy(path, text):
        written.append(Path(path).name)
        real(path, text)

    monkeypatch.setattr(report_module, "write_text_atomic", spy)
    quiet(walkcheck_main, [str(session)])
    assert written == ["walkcheck.md"]
    assert (session / "walkcheck.md").read_text() == render_markdown(json.loads((session / "walkcheck.json").read_text()))
    assert not list(session.glob("*.tmp"))


def fail_while_pending(monkeypatch, operation, target):
    """Make os.<operation> fail, but only while target's temp file exists.

    That hits the fsync of target's own temp file and its replace, not an earlier
    write in the same command or the directory fsync after the rename."""
    target = Path(target)
    real = getattr(os, operation)

    def fail(*args):
        if list(target.parent.glob(target.name + ".*.tmp")):
            raise OSError("injected " + operation)
        return real(*args)

    monkeypatch.setattr(jsonio.os, operation, fail)


def spy_writes(monkeypatch, module):
    """Record the file names module writes through the atomic helpers."""
    written = []
    for helper in ("write_json_atomic", "write_text_atomic"):
        if hasattr(module, helper):
            def spy(path, value, real=getattr(jsonio, helper)):
                written.append(Path(path).name)
                real(path, value)
            monkeypatch.setattr(module, helper, spy)
    return written


def series_inputs(tmp_path):
    return [write_result(tmp_path, timed(make_walk("run-%d" % i), i)) for i in range(3)]


def test_series_compare_writes_verdict_atomically(tmp_path, monkeypatch, capsys):
    # Issue #115: the series verdict goes through the temp+fsync+replace helper.
    written = spy_writes(monkeypatch, series_cli)
    out = tmp_path / "series"
    assert series_cli.main(["compare", *series_inputs(tmp_path), "--out", str(out)]) == 0
    assert written == ["series.md", "series.json"]
    report = json.loads((out / "series.json").read_text(encoding="utf-8"))
    assert report["verdict"] == "PASS"
    # Same bytes as the in-place writer it replaces.
    assert (out / "series.json").read_text(encoding="utf-8") == json.dumps(report, indent=2, ensure_ascii=False) + "\n"
    assert sorted(p.name for p in out.iterdir()) == ["series.json", "series.md"]


@pytest.mark.parametrize("earlier", [False, True])
@pytest.mark.parametrize("operation", ["fsync", "replace"])
@pytest.mark.parametrize("name", ["series.md", "series.json"])
def test_series_compare_os_failure_preserves_old_verdict(tmp_path, monkeypatch, capsys, earlier, operation, name):
    out = tmp_path / "series"
    out.mkdir()
    target = out / name
    previous = b"earlier verdict\n"
    if earlier:
        target.write_bytes(previous)
    inputs = series_inputs(tmp_path)
    fail_while_pending(monkeypatch, operation, target)
    assert series_cli.main(["compare", *inputs, "--out", str(out)]) == 2
    assert "injected " + operation in capsys.readouterr().err
    assert target.read_bytes() == previous if earlier else not target.exists()
    assert not list(out.glob("*.tmp"))


@pytest.mark.parametrize("earlier", [False, True])
@pytest.mark.parametrize("operation", ["fsync", "replace"])
def test_series_explain_os_failure_preserves_old_summary(tmp_path, monkeypatch, capsys, earlier, operation):
    walk = series_inputs(tmp_path)[0]
    target = tmp_path / "out" / "explain.txt"
    target.parent.mkdir()
    previous = b"earlier summary\n"
    if earlier:
        target.write_bytes(previous)
    written = spy_writes(monkeypatch, series_cli)
    fail_while_pending(monkeypatch, operation, target)
    assert series_cli.main(["explain", walk, "--out", str(target)]) == 2
    assert written == ["explain.txt"]
    assert target.read_bytes() == previous if earlier else not target.exists()
    assert list(target.parent.iterdir()) == ([target] if earlier else [])


@pytest.mark.parametrize("earlier", [False, True])
@pytest.mark.parametrize("operation", ["fsync", "replace"])
def test_series_adapt_os_failure_preserves_old_result(tmp_path, monkeypatch, capsys, earlier, operation):
    _, _, report_path, meta_path = produced_walkcheck(tmp_path)
    target = tmp_path / "out" / "walk_result.json"
    target.parent.mkdir()
    previous = b'{"earlier": true}\n'
    if earlier:
        target.write_bytes(previous)
    written = spy_writes(monkeypatch, series_cli)
    fail_while_pending(monkeypatch, operation, target)
    assert series_cli.main(["adapt", str(report_path), "--metadata", str(meta_path),
                            "--out", str(target), *wall_args(report_path)]) == 2
    assert written == ["walk_result.json"]
    assert target.read_bytes() == previous if earlier else not target.exists()
    assert list(target.parent.iterdir()) == ([target] if earlier else [])


def bagtiming(tmp_path):
    lidar, imu = "/test/lidar", "/test/imu"
    bt = {"schema": "spatio.bagtiming/1",
          "topics": [{"name": lidar, "count": 100, "header": {}}, {"name": imu, "count": 2000, "header": {}}],
          "findings": [],
          "imu_coverage": [{"lidar_topic": lidar, "imu_topic": imu, "scans": 100, "evaluable_scans": 100}]}
    path = tmp_path / "bagtiming.json"
    path.write_text(json.dumps(bt))
    return bt, [str(path), "--lidar", lidar, "--imu", imu]


def test_input_gate_writes_atomically(tmp_path, monkeypatch, capsys):
    # Issue #115: result.py decides from input_gate.json whether a run is scored.
    bt, argv = bagtiming(tmp_path)
    target = tmp_path / "logs" / "input_gate.json"
    target.parent.mkdir()
    written = spy_writes(monkeypatch, gate)
    assert gate.main(argv + ["--out", str(target)]) == 0
    assert written == ["input_gate.json"]
    assert json.loads(target.read_text()) == gate.evaluate(bt, argv[2], argv[4])
    assert list(target.parent.iterdir()) == [target]


@pytest.mark.parametrize("earlier", [False, True])
@pytest.mark.parametrize("operation", ["fsync", "replace"])
def test_input_gate_os_failure_preserves_old_gate(tmp_path, monkeypatch, capsys, earlier, operation):
    _, argv = bagtiming(tmp_path)
    target = tmp_path / "logs" / "input_gate.json"
    target.parent.mkdir()
    previous = b'{"earlier": true}\n'
    if earlier:
        target.write_bytes(previous)
    fail_while_pending(monkeypatch, operation, target)
    with pytest.raises(OSError, match=operation):
        gate.main(argv + ["--out", str(target)])
    assert target.read_bytes() == previous if earlier else not target.exists()
    assert list(target.parent.iterdir()) == ([target] if earlier else [])
