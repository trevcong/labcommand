"""Q121: remaining walk-day writers preserve complete files on failure."""

import json
import os
import subprocess
import sys
from pathlib import Path

import numpy as np
import pytest

from spatio.manifest import jsonio
from spatio.replay import manifest
from spatio.series import cli as series_cli
from spatio.walkday import provenance, seriesgate, startend
from spatio.walkday.tests.test_atomic_results import fail_while_pending, series_inputs
from spatio.wallpatch import util


WRITERS = ["series.md", "series.json", "wallpatch.json", "provenance.json",
           "manifest.json", "startend.json"]


def writer(tmp_path, monkeypatch, name):
    """Exercise production entry points; stub only expensive scan registration."""
    out = tmp_path / "out"
    out.mkdir()
    target = out / name
    if name.startswith("series."):
        inputs = series_inputs(tmp_path)
        staged = tmp_path / "staged"
        assert series_cli.main(["compare", *inputs, "--out", str(staged)]) == 0
        report = seriesgate.gate(json.loads((staged / "series.json").read_text()), inputs)
        expected = seriesgate.render(report) if name.endswith(".md") else report
        invoke = lambda: seriesgate.main([str(staged), str(out), *inputs])
    elif name == "wallpatch.json":
        expected = {"values": [1.25, 2], "selected": True, "missing": None}
        data = {"values": np.array([1.25, 2]), "selected": np.bool_(True), "missing": np.nan}
        invoke = lambda: util.write_json(target, data)
    elif name == "provenance.json":
        config = tmp_path / "config.yaml"
        config.write_text("fixture config\n")
        pairs = [f"{key}=1" for key in provenance.SETTINGS_KEYS]
        expected = provenance.build(str(tmp_path), str(config), pairs)
        invoke = lambda: provenance.main(["write", str(target), "--rec", str(tmp_path),
                                          "--config", str(config), *pairs])
    elif name == "manifest.json":
        expected = manifest.build_manifest("run", "/rec", "fixture", "abc", "/absent", None,
                                           "image", "", created_at="2030-01-01T00:00:00Z")
        real_build = manifest.build_manifest
        monkeypatch.setattr(manifest, "build_manifest",
                            lambda *args: real_build(*args, created_at=expected["created_at"]))
        monkeypatch.setattr(sys, "argv", ["manifest.py", str(out), "--run-id", "run",
                            "--source-recording", "/rec", "--sensor", "fixture",
                            "--fastlio-revision", "abc", "--config-file", "/absent", "--image", "image"])
        invoke = manifest.main
    else:
        expected = {"schema": startend.SCHEMA, "computed": False, "reason": "no raw scans inside a hold window"}
        monkeypatch.setattr(startend, "run", lambda *args: expected)
        invoke = lambda: startend.main(["/rec", "/trajectory", str(out), "--lidar", "/scan"])
    return target, invoke, expected


@pytest.mark.parametrize("name", WRITERS)
def test_remaining_writer_success(tmp_path, monkeypatch, name):
    target, invoke, expected = writer(tmp_path, monkeypatch, name)
    target.write_text("earlier output\n")
    invoke()
    text = target.read_text(encoding="utf-8")
    assert (text if name.endswith(".md") else json.loads(text)) == expected
    assert text.endswith("\n")
    assert not list(target.parent.glob("*.tmp"))


@pytest.mark.parametrize("name", WRITERS)
@pytest.mark.parametrize("earlier", [False, True])
@pytest.mark.parametrize("operation", ["fsync", "replace"])
def test_remaining_writer_os_failure(tmp_path, monkeypatch, name, earlier, operation):
    target, invoke, _ = writer(tmp_path, monkeypatch, name)
    previous = b"earlier complete output\n"
    if earlier:
        target.write_bytes(previous)
    fail_while_pending(monkeypatch, operation, target)
    with pytest.raises(OSError, match="injected " + operation):
        invoke()
    assert target.read_bytes() == previous if earlier else not target.exists()
    assert not list(target.parent.glob("*.tmp"))


@pytest.mark.parametrize("name", [name for name in WRITERS if name.endswith(".json")])
@pytest.mark.parametrize("earlier", [False, True])
@pytest.mark.parametrize("failure", [OSError, KeyboardInterrupt])
def test_remaining_writer_interrupted_json(tmp_path, monkeypatch, name, earlier, failure):
    target, invoke, _ = writer(tmp_path, monkeypatch, name)
    previous = b'{"earlier": true}\n'
    if earlier:
        target.write_bytes(previous)

    def interrupted_dump(obj, handle, **kwargs):
        handle.write('{"partial": ')
        handle.flush()
        raise failure("interrupted JSON write")

    monkeypatch.setattr(jsonio.json, "dump", interrupted_dump)
    with pytest.raises(failure, match="interrupted JSON write"):
        invoke()
    assert target.read_bytes() == previous if earlier else not target.exists()
    assert not list(target.parent.glob("*.tmp"))


def test_replay_manifest_direct_script_outside_repo(tmp_path):
    # replay.sh uses this invocation, so importing the shared helper must work
    # without relying on cwd or a caller-supplied PYTHONPATH.
    env = dict(os.environ)
    env.pop("PYTHONPATH", None)
    completed = subprocess.run(
        [sys.executable, str(Path(manifest.__file__).resolve()), str(tmp_path),
         "--run-id", "run", "--source-recording", "/rec", "--sensor", "fixture",
         "--fastlio-revision", "abc", "--config-file", "/absent", "--image", "image"],
        cwd=tmp_path, env=env, stdin=subprocess.DEVNULL, capture_output=True, text=True,
    )
    assert completed.returncode == 0, completed.stderr
    report = json.loads((tmp_path / "manifest.json").read_text())
    assert report["run_id"] == "run"
    assert report["physical_accuracy_evaluated"] is False
