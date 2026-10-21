"""synth/dryrun.sh wiring without Docker: every run gets the frozen spec and
its preselected patches, and those boxes hold on a synthetic map."""

import hashlib
import json
import math
import os
import shutil
import subprocess
import sys
from pathlib import Path

import numpy as np
import pytest

from spatio.walkday.patch_origin import bind_spec_boxes, classify
from spatio.walkday.synth.scene import Path as LoopPath, SceneParams, raycast, rotz, scan_count
from spatio.thresholds import GUARD_BAND_M, WALL_GATE_M
from spatio.wallpatch.measure import measure_map
from spatio.wallpatch.pcd import write_binary_pcd

REPO = Path(__file__).resolve().parents[3]
SYNTH = REPO / "spatio" / "walkday" / "synth"
RUNS = ["pass-a", "fail", "pass-b", "pass-c"]

# Records its argv as one JSON line in $TEST_LOG, tagged with the stub's name.
RECORD = f'''#!{sys.executable}
import json, os, sys
with open(os.environ["TEST_LOG"], "a") as f:
    f.write(json.dumps([os.path.basename(sys.argv[0])] + sys.argv[1:]) + "\\n")
'''
TRUTH = f'''#!{sys.executable}
import json
print(json.dumps({{"position_error_m": {{"median": 0.0, "max": 0.0}}, "drift_while_parked_at_start_m": 0.0}}))
'''
RECORDED_AT = f'''#!{sys.executable}
import os, sys
assert sys.argv[1] == "recorded-at", sys.argv
print("2030-01-01T00:00:%02dZ" % ["pass-a", "fail", "pass-b", "pass-c"].index(os.path.basename(sys.argv[2])))
'''


def _script(path, text):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text)
    path.chmod(0o755)


@pytest.fixture
def dryrun(tmp_path):
    shell = tmp_path / "shell" / "spatio"
    synth = shell / "walkday" / "synth"
    synth.mkdir(parents=True)
    for name in ("dryrun.sh", "spec_fill.json"):
        shutil.copy2(SYNTH / name, synth / name)
    # Recordings: the real prepare.py (numpy only), without the Docker bag writer.
    _script(synth / "make_recording.sh",
            f'#!/bin/sh\nout=$1 off=$2; shift 2\nexec "{sys.executable}" "{SYNTH / "prepare.py"}" '
            '"$out" --end-offset-m "$off" "$@" >/dev/null\n')
    _script(synth / "truth.py", TRUTH)
    _script(shell / "walkday" / "process.sh", RECORD)
    _script(shell / "walkday" / "series.sh", RECORD)
    _script(shell / "replay" / "recording.py", RECORDED_AT)
    bin_dir = tmp_path / "bin"
    _script(bin_dir / "git", "#!/bin/sh\necho " + "ab12cd3" * 5 + "abcde\n")
    log = tmp_path / "calls.jsonl"
    env = dict(os.environ, PATH=str(bin_dir) + os.pathsep + os.environ["PATH"],
               PYTHONPATH=str(REPO), TEST_LOG=str(log), SPATIO_SRC=str(tmp_path / "src"))
    out = tmp_path / "out"
    proc = subprocess.run(["bash", str(synth / "dryrun.sh"), str(out)], env=env, stdin=subprocess.DEVNULL,
                          capture_output=True, text=True, timeout=120)
    assert proc.returncode == 0, proc.stdout + proc.stderr
    calls = [json.loads(line) for line in log.read_text().splitlines()]
    return out, calls


def _opt(argv, flag):
    assert argv.count(flag) == 1, (flag, argv)
    return argv[argv.index(flag) + 1]


def test_every_run_gets_the_frozen_spec_and_preselected_patches(dryrun):
    out, calls = dryrun
    runs = [c for c in calls if c[0] == "process.sh"]
    assert [Path(c[1]).name for c in runs] == RUNS
    specs = {_opt(c, "--spec") for c in runs}
    patches = {_opt(c, "--patches") for c in runs}
    assert len(specs) == 1 and len(patches) == 1  # one spec for the series (G1)
    spec_path, patch_path = specs.pop(), patches.pop()
    assert spec_path.endswith(".frozen.json") and Path(spec_path).parent == out / "spec"
    assert _opt(runs[0], "--placement-cm") == "0.4"
    doc = json.loads(Path(patch_path).read_text())
    assert classify(doc, spec_path) == "preselected"
    spec = json.loads(Path(spec_path).read_text())
    assert len(spec["wall_patches"]) >= 2
    assert all(p["box_map_frame"] is not None for p in spec["wall_patches"])
    assert "map_sha256" not in doc
    # process.sh binds the spec's fixed boxes to each run's own map.
    run_map = out / "map.pcd"
    run_map.write_bytes(b"any run's map bytes")
    bind_spec_boxes(doc, spec_path, run_map)
    assert doc["map_sha256"] == hashlib.sha256(run_map.read_bytes()).hexdigest()
    series = [c for c in calls if c[0] == "series.sh"]
    assert len(series) == 1 and series[0][1:5] == [Path(c[2]).name for c in runs]


def test_changed_spec_fill_freezes_a_new_spec(dryrun, tmp_path):
    out, calls = dryrun
    old = _opt(calls[0], "--spec")
    fill = tmp_path / "shell" / "spatio" / "walkday" / "synth" / "spec_fill.json"
    doc = json.loads(fill.read_text())
    doc["spec_id"] += "-changed"
    fill.write_text(json.dumps(doc))
    log = tmp_path / "calls.jsonl"
    log.unlink()
    env = dict(os.environ, PATH=str(tmp_path / "bin") + os.pathsep + os.environ["PATH"],
               PYTHONPATH=str(REPO), TEST_LOG=str(log), SPATIO_SRC=str(tmp_path / "src"))
    proc = subprocess.run(["bash", str(fill.parent / "dryrun.sh"), str(out)], env=env,
                          stdin=subprocess.DEVNULL, capture_output=True, text=True, timeout=120)
    assert proc.returncode == 0, proc.stdout + proc.stderr
    new = _opt(json.loads(log.read_text().splitlines()[0]), "--spec")
    assert new != old
    assert json.loads(Path(new).read_text())["spec_id"].endswith("-changed")


@pytest.mark.parametrize("seed", [11, 12, 13])
def test_spec_boxes_measure_single_walls_on_the_synthetic_scene(tmp_path, seed):
    """Ground-truth map in the start frame (FAST-LIO's map frame): each fixed
    box holds one wall well inside the guard band. FAST-LIO's own drift is
    only seen in the Docker dry run."""
    p = SceneParams(seed=seed)
    loop = LoopPath(p)
    rng = np.random.default_rng(seed)
    start = loop.state(np.array([p.t0]))[0][0]
    pts = []
    for k in range(0, scan_count(loop), 6):
        n = p.points_per_scan
        t = p.t0 + k / p.lidar_rate_hz + np.sort(rng.uniform(0, 1 / p.lidar_rate_hz, n))
        az = rng.uniform(-math.pi, math.pi, n)
        lo, hi = np.radians(p.elevation_deg)
        el = np.arcsin(rng.uniform(math.sin(lo), math.sin(hi), n))
        d_s = np.column_stack([np.cos(el) * np.cos(az), np.cos(el) * np.sin(az), np.sin(el)])
        pos, yaw, _, _ = loop.state(t)
        d_w = np.einsum("nij,nj->ni", rotz(yaw), d_s)
        r = raycast(p, pos, d_w) + rng.normal(0, p.range_noise_m, n)
        pts.append(pos + d_w * r[:, None] - start)
    map_path = tmp_path / "map.pcd"
    write_binary_pcd(str(map_path), np.concatenate(pts))
    fill = json.loads((SYNTH / "spec_fill.json").read_text())
    patches = tmp_path / "patches.json"
    patches.write_text(json.dumps({
        "schema": "spatio.wall_patches/1",
        "map_sha256": hashlib.sha256(map_path.read_bytes()).hexdigest(),
        "patches": [{"name": w["name"], "box_map_frame": w["box_map_frame"]} for w in fill["wall_patches"]],
    }))
    measure_map(str(map_path), str(patches), str(tmp_path / "wp"), gate_m=WALL_GATE_M)
    report = json.loads((tmp_path / "wp" / "wallpatch.json").read_text())
    assert len(report["patches"]) == 2
    for q in report["patches"]:
        assert q["status"] == "MEASURED" and q["doubled"] == "false", q
        assert q["gate_p95_pass"] is True and q["gate_separation_pass"] is True, q
        res = q["residuals"]
        assert max(res["p95_abs_m"], res["core_band_m"], res["stepped_band_m"]) <= GUARD_BAND_M - 0.01, q
