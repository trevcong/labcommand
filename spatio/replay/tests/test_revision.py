"""Exercise build/replay scripts with Docker stubbed; never contact a daemon."""

import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys

import pytest

from spatio.walkday.tests.test_recording import write_rec


REPLAY = Path(__file__).resolve().parents[1]
PIN = (REPLAY / "FASTLIO_REVISION").read_text().strip()
IMAGE_ID = "sha256:" + "a" * 64

DOCKER = r'''
import json, os, shutil, sys
from pathlib import Path
args = sys.argv[1:]
with open(os.environ["DOCKER_CALLS"], "a") as f:
    f.write(json.dumps(args) + "\n")
assert args[:2] == ["--context", "test-only"], args
args = args[2:]
if args[:2] == ["image", "inspect"]:
    if args[3] == "{{.Id}}":
        if os.environ.get("MISSING_IMAGE"):
            sys.exit(1)
        print(os.environ["IMAGE_ID"])
    else:
        assert args[-1] == os.environ["IMAGE_ID"], args
        if os.environ.get("INSPECT_FAILURE"):
            sys.exit(1)
        print(os.environ["IMAGE_REV"])
elif args[0] == "build":
    context = Path(args[-1])
    snapshot = {"args": args,
                "provenance": json.loads((context / "provenance.json").read_text()),
                "dockerfile": (context / "Dockerfile").read_text()}
    Path(os.environ["BUILD_SNAPSHOT"]).write_text(json.dumps(snapshot))
elif args[0] == "run":
    assert os.environ["IMAGE_ID"] in args, args
    if args[-1] == "/rec/.spatio_loaded":
        print(os.environ["BAG_SHA"])
    elif args[-1] == "/opt/spatio/provenance.json":
        print(json.dumps({"fast_lio_ros2": {"revision": os.environ["IMAGE_REV"]}}))
    else:
        raise AssertionError(args)
elif args[0] == "create":
    assert os.environ["IMAGE_ID"] in args, args
    print("stub-container")
elif args[0] == "cp":
    if args[1].endswith(":/out/."):
        fixture = Path(os.environ["FIXTURE"])
        for name in ("trajectory.tum", "map.pcd"):
            shutil.copyfile(fixture / name, Path(args[2]) / name)
        shutil.copyfile(fixture / "map.pcd", Path(args[2]) / "map_display.pcd")
elif args[0] not in ("start", "rm"):
    raise AssertionError(args)
'''


def executable(path, body):
    path.write_text(f"#!{sys.executable}\n" + body)
    path.chmod(0o755)


@pytest.fixture
def harness(tmp_path):
    bin_dir = tmp_path / "bin"
    bin_dir.mkdir()
    executable(bin_dir / "docker", DOCKER)
    # Keep script subprocesses on the pytest interpreter, including Python 3.9.
    (bin_dir / "python3").symlink_to(sys.executable)
    rec = tmp_path / "rec"
    write_rec(str(rec), mcap_counts={"/rig7/points": 3, "/rig7/inertial": 5})
    listing = "".join(f"{hashlib.sha256(p.read_bytes()).hexdigest()}  {p.name}\n"
                      for p in sorted((rec / "bag").iterdir()))
    env = dict(os.environ, PATH=str(bin_dir) + os.pathsep + os.environ["PATH"],
               DOCKER_CONTEXT="test-only", DOCKER_CALLS=str(tmp_path / "docker.jsonl"),
               IMAGE_ID=IMAGE_ID, IMAGE_REV=PIN, SPATIO_DATA=str(tmp_path / "data"),
               TMPDIR=str(tmp_path), BUILD_SNAPSHOT=str(tmp_path / "build.json"),
               BAG_SHA=hashlib.sha256(listing.encode()).hexdigest(),
               FIXTURE=str(REPLAY.parent / "io" / "fixtures" / "map_sharp_wall"))
    for key in ("SPATIO_WALK_SPEC", "MISSING_IMAGE", "INSPECT_FAILURE"):
        env.pop(key, None)
    return tmp_path, env


def run_script(harness, script, *args):
    _, env = harness
    return subprocess.run(["bash", str(REPLAY / script), *map(str, args)],
                          env=env, stdin=subprocess.DEVNULL, capture_output=True,
                          text=True, timeout=30)


def calls(harness):
    path = Path(harness[1]["DOCKER_CALLS"])
    return [json.loads(line)[2:] for line in path.read_text().splitlines()] if path.exists() else []


@pytest.mark.parametrize("revision", ["b" * 40, "", "<no value>", "unknown", PIN[:7], PIN + "-dirty"])
def test_replay_refuses_unpinned_image_without_spec(harness, revision):
    root, env = harness
    env["IMAGE_REV"] = revision
    result = run_script(harness, "replay.sh", root / "rec", "test")
    assert result.returncode == 2, result.stdout + result.stderr
    assert "does not match pinned revision" in result.stderr
    assert "build_image.sh" in result.stderr
    assert all(c[:2] == ["image", "inspect"] for c in calls(harness))
    assert not (root / "data" / "sessions" / "test").exists()


@pytest.mark.parametrize("failure", ["MISSING_IMAGE", "INSPECT_FAILURE"])
def test_replay_inspection_failure_stops_before_containers(harness, failure):
    root, env = harness
    env[failure] = "1"
    result = run_script(harness, "replay.sh", root / "rec", "test")
    assert result.returncode == 2, result.stdout + result.stderr
    assert "build_image.sh" in result.stderr
    assert all(c[:2] == ["image", "inspect"] for c in calls(harness))


def test_matching_image_replays_without_source_checkout_or_spec(harness):
    root, _ = harness
    assert not (root / "data" / "spark-src").exists()
    result = run_script(harness, "replay.sh", root / "rec", "test")
    assert result.returncode == 0, result.stdout + result.stderr
    session = root / "data" / "sessions" / "test"
    manifest = json.loads((session / "manifest.json").read_text())
    assert manifest["fastlio_revision"] == PIN
    assert manifest["image"] == f"spatio/replay:{PIN[:7]}@{IMAGE_ID}"
    commands = calls(harness)
    assert commands[0][-1] == f"spatio/replay:{PIN[:7]}"
    assert commands[1][-1] == IMAGE_ID
    assert any(c[0] == "start" for c in commands)
    assert all(IMAGE_ID in c for c in commands if c[0] in ("run", "create"))


def build_source(harness, revision):
    root, env = harness
    src = root / "source"
    for relative in ("FAST_LIO_ROS2/src/laserMapping.cpp",
                     "FAST_LIO_ROS2/include/ikd-Tree/ikd_Tree.cpp",
                     "livox_ros_driver2/msg/CustomMsg.msg",
                     "livox_ros_driver2/msg/CustomPoint.msg", "launchers/map_saver.py"):
        path = src / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text("fixture source\n")
    env["SPATIO_SRC"] = str(src)
    env["SOURCE_REV"] = revision
    executable(root / "bin" / "git", r'''
import os, sys
args = sys.argv[1:]
assert args[0] == "-C" and args[1].startswith(os.environ["SPATIO_SRC"]), args
if args[2:] == ["rev-parse", "HEAD"]:
    print(os.environ["SOURCE_REV"])
elif args[2:] not in (["status", "--porcelain"], ["diff", "HEAD"]):
    raise AssertionError(args)
''')


def test_build_records_exact_source_revision(harness):
    root, _ = harness
    build_source(harness, PIN)
    result = run_script(harness, "build_image.sh")
    assert result.returncode == 0, result.stdout + result.stderr
    snapshot = json.loads((root / "build.json").read_text())
    assert snapshot["provenance"]["fast_lio_ros2"]["revision"] == PIN
    args = snapshot["args"]
    assert args[args.index("--build-arg") + 1] == f"FASTLIO_REVISION={PIN}"
    assert args[args.index("-t") + 1] == f"spatio/replay:{PIN[:7]}"
    assert "LABEL spatio.fastlio_revision=$FASTLIO_REVISION" in snapshot["dockerfile"]


def test_build_refuses_source_drift_before_docker(harness):
    build_source(harness, "b" * 40)
    result = run_script(harness, "build_image.sh")
    assert result.returncode == 2, result.stdout + result.stderr
    assert "does not match pinned revision" in result.stderr
    assert calls(harness) == []
