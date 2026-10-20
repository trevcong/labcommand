"""Run the real process.sh flow with local recording/replay stages stubbed."""

import hashlib
import json
import os
import shutil
import subprocess
import sys
from pathlib import Path

import pytest

from spatio.walkday.patch_origin import classify
from spatio.walkday.tests.test_result import quiet, write_recording
from spatio.wallpatch.cli import main as wallpatch_main

REPO = Path(__file__).resolve().parents[3]
FIXTURE = REPO / "spatio" / "io" / "fixtures" / "map_sharp_wall"

# A temporary repo-shaped shell harness: no production test switches and no
# opportunity to invoke the real replay.sh (which would require Docker).
STUB = r'''
import hashlib, json, os, shutil, sys
from pathlib import Path
from spatio.walkday.tests.test_result import write_gate

args = sys.argv[1:]
root = Path(os.environ["TEST_ROOT"])
rec = root / "rec"
config = rec / "config.yaml"
if args[0] == "replay":
    session = Path(args[args.index("--sessions-root") + 1]) / args[2]
    fixture = Path(os.environ["TEST_FIXTURE"])
    for name in ("trajectory.tum", "map.pcd"):
        shutil.copyfile(fixture / name, session / name)
    manifest = dict(run_id=args[2], source_recording=str(rec), sensor="fixture",
                    fastlio_revision="abc1234", config_file=str(config),
                    config_sha256=hashlib.sha256(config.read_bytes()).hexdigest(),
                    image="stub", notes="stub replay of committed fixture",
                    physical_accuracy_evaluated=False)
    (session / "manifest.json").write_text(json.dumps(manifest))
elif args[0].endswith("/spatio/replay/recording.py"):
    if args[1] == "recorded-at":
        print("2030-01-01T00:00:00Z")
    elif args[1] == "find-config":
        print(config)
    elif args[1] == "profile":
        print("LIDAR=/test/lidar IMU=/test/imu LIDAR_MSG_TYPE=test")
    elif args[1] != "complete":
        raise AssertionError(args)
elif args[:3] == ["-m", "spatio.walkday.provenance", "write"]:
    pass
elif args[:2] == ["-m", "spatio.bagtiming"]:
    pass
elif args[:2] == ["-m", "spatio.walkday.gate"]:
    out = Path(args[args.index("--out") + 1])
    write_gate(out.parent.parent)
elif args[:2] == ["-m", "spatio.manifest"]:
    session = Path(args[3])
    if args[2] == "build":
        shutil.copyfile(session / "logs" / "replay_manifest.json", session / "manifest.json")
    elif args[2] != "validate":
        raise AssertionError(args)
elif args[:2] == ["-m", "spatio.walkday.startend"]:
    pass
else:
    # Walkcheck, patch suggest/measure, patch-origin validation and result are
    # real commands. Inline Python and settings validation are also real.
    os.execv(sys.executable, [sys.executable, *args])
'''


@pytest.fixture
def process_harness(tmp_path):
    shell_root = tmp_path / "shell"
    walkday = shell_root / "spatio" / "walkday"
    walkday.mkdir(parents=True)
    for name in ("process.sh", "walkday.conf"):
        shutil.copyfile(REPO / "spatio" / "walkday" / name, walkday / name)
    bin_dir = tmp_path / "bin"
    bin_dir.mkdir()
    python_stub = bin_dir / "python3"
    python_stub.write_text(f"#!{sys.executable}\n" + STUB)
    python_stub.chmod(0o755)
    replay = shell_root / "spatio" / "replay" / "replay.sh"
    replay.parent.mkdir()
    replay.write_text('#!/bin/sh\nexec python3 replay "$@"\n')
    replay.chmod(0o755)
    rec = write_recording(tmp_path / "rec")
    config = rec / "config.yaml"
    config.write_text("fixture config\n")
    spec = tmp_path / "spec.json"
    patches = tmp_path / "patches.json"
    assert quiet(wallpatch_main, ["suggest", str(FIXTURE / "map.pcd"),
                                 "--n", "2", "--out", str(patches)]) == 0
    doc = json.loads(patches.read_text())
    assert len(doc["patches"]) == 2
    spec.write_text(json.dumps({
        "config_sha256": hashlib.sha256(config.read_bytes()).hexdigest(),
        "fastlio_revision": "abc1234",
        "wall_patches": [{"name": p["name"], "box_map_frame": p["box_map_frame"]}
                         for p in doc["patches"]],
    }))
    env = dict(os.environ, PATH=str(bin_dir) + os.pathsep + os.environ["PATH"],
               PYTHONPATH=str(REPO), TEST_ROOT=str(tmp_path), TEST_FIXTURE=str(FIXTURE))
    env.pop("SPATIO_WALK_SPEC", None)
    return tmp_path, walkday / "process.sh", rec, spec, patches, env


def run_process(harness, *, spec=True, patches=True):
    root, script, rec, spec_path, patch_path, env = harness
    args = ["bash", str(script), str(rec), "run", "--sessions-root", str(root / "sessions")]
    if spec:
        args += ["--spec", str(spec_path)]
    if patches:
        args += ["--patches", str(patch_path)]
    proc = subprocess.run(args, env=env, stdin=subprocess.DEVNULL,
                          capture_output=True, text=True, timeout=120)
    return proc, root / "sessions" / "run"


@pytest.mark.parametrize("supplied", [True, False])
def test_suggestions_keep_origin_and_cannot_count(process_harness, supplied):
    proc, session = run_process(process_harness, patches=supplied)
    assert proc.returncode == 2, proc.stdout + proc.stderr
    walk = json.loads((session / "walk_result.json").read_text())
    assert walk["wall_patches"]["origin"] == "auto_suggested"
    assert walk["wall_patches"]["verified"] == []
    assert walk["status"] == "NOT_EVALUABLE"
    copied = json.loads((session / "wallpatch" / "patches.json").read_text())
    assert all(p["origin"] == "auto_suggested" for p in copied["patches"])
    if supplied:
        assert "cannot be labelled preselected" in proc.stdout


def test_matching_preselected_patches_are_stamped(process_harness):
    path = process_harness[4]
    doc = json.loads(path.read_text())
    for patch in doc["patches"]:
        del patch["origin"]  # legacy hand-selected file, names/boxes match spec
    path.write_text(json.dumps(doc))
    before = path.read_bytes()
    proc, session = run_process(process_harness)
    assert proc.returncode == 0, proc.stdout + proc.stderr
    walk = json.loads((session / "walk_result.json").read_text())
    assert walk["wall_patches"]["origin"] == "preselected"
    assert len(walk["wall_patches"]["verified"]) == 2
    copied = json.loads((session / "wallpatch" / "patches.json").read_text())
    assert all(p["origin"] == "preselected" for p in copied["patches"])
    assert path.read_bytes() == before


@pytest.mark.parametrize("case", ["missing_spec", "wrong_names", "duplicates", "unknown_origin", "invalid_json"])
def test_unproven_preselection_stops_loudly(process_harness, case):
    path = process_harness[4]
    doc = json.loads(path.read_text())
    for patch in doc["patches"]:
        patch["origin"] = "preselected"
    if case == "wrong_names":
        doc["patches"][0]["name"] = "elsewhere"
    elif case == "duplicates":
        doc["patches"][1]["name"] = doc["patches"][0]["name"]
    elif case == "unknown_origin":
        doc["patches"][0]["origin"] = "unknown"
    path.write_text("{" if case == "invalid_json" else json.dumps(doc))
    proc, session = run_process(process_harness, spec=case != "missing_spec")
    assert proc.returncode == 2, proc.stdout + proc.stderr
    assert "cannot establish preselection" in proc.stdout
    assert not (session / "walk_result.json").exists()
    assert json.loads((session / "walkday.json").read_text())["stopped_at"] == "wall-patches"


def test_copy_failure_stops_before_measurement(process_harness):
    root, _, _, _, path, _ = process_harness
    cp = root / "bin" / "cp"
    # Fail just the supplied-patch copy, allowing earlier settings copies.
    cp.write_text('#!/bin/sh\ncase "$1" in */patches.json) exit 77;; esac\nexec /bin/cp "$@"\n')
    cp.chmod(0o755)
    proc, session = run_process(process_harness)
    assert proc.returncode == 2, proc.stdout + proc.stderr
    assert "cannot copy patches" in proc.stdout
    assert not (session / "walk_result.json").exists()
    assert not (session / "wallpatch" / "wallpatch.json").exists()


def test_one_auto_patch_prevents_aggregate_preselection(tmp_path):
    doc = {"patches": [{"name": "a", "origin": "preselected"},
                       {"name": "b", "origin": "auto_suggested"}]}
    assert classify(doc) == "auto_suggested"
