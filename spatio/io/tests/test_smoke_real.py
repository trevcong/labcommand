"""Exit codes and explicit paths for the read-only smoke command."""

from __future__ import annotations

import json
import subprocess
import sys
import tempfile
import unittest
from contextlib import redirect_stdout
from io import StringIO
from pathlib import Path

import numpy as np

from spatio.io.pcd import write_pcd
from spatio.io.smoke_real import main
from spatio.io.tum import Trajectory, write_tum

REPO = Path(__file__).resolve().parents[3]
# Recording ids that must not appear as defaults or sample names.
_RECORDING_TOKENS = (
    "20260808",
    "20260826",
    "jt128_map",
    "JT128_room_walk",
    "session_20260808",
)


def _quat(n):
    quat = np.zeros((n, 4), dtype=np.float64)
    quat[:, 3] = 1.0
    return quat


def _write_traj(path, n, t0):
    t = float(t0) + np.arange(n, dtype=np.float64) * 0.1
    xyz = np.zeros((n, 3), dtype=np.float64)
    xyz[:, 0] = np.arange(n, dtype=np.float64)
    write_tum(str(path), Trajectory(t, xyz, _quat(n), str(path)))


def _write_map(path, n):
    count = int(n)
    values = np.arange(count, dtype=np.float32)
    write_pcd(
        str(path),
        {"x": values, "y": np.zeros(count, dtype=np.float32), "z": np.ones(count, dtype=np.float32)},
        data="ascii",
    )


def _run(args):
    buf = StringIO()
    with redirect_stdout(buf):
        code = main(list(args))
    text = buf.getvalue()
    return code, json.loads(text), text


class SmokeRealTests(unittest.TestCase):
    def test_smoke_missing_inputs_and_parse_failures_exit_nonzero(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            out = root / "report.json"
            code, report, text = _run(["--out", str(out)])
            self.assertEqual(code, 1)
            self.assertEqual(report["status"], "missing required inputs")
            self.assertEqual(report["maps"], [])
            self.assertEqual(report["trajectories"], [])
            self.assertEqual(json.loads(out.read_text(encoding="utf-8")), report)
            for token in _RECORDING_TOKENS:
                self.assertNotIn(token, text)

            proc = subprocess.run(
                [sys.executable, "-m", "spatio.io.smoke_real", "--out", str(out)],
                cwd=str(REPO),
                capture_output=True,
                text=True,
            )
            self.assertEqual(proc.returncode, 1, proc.stdout + "\n" + proc.stderr)
            cli_report = json.loads(proc.stdout)
            self.assertEqual(cli_report["status"], "missing required inputs")
            self.assertEqual(cli_report["maps"], [])
            self.assertEqual(cli_report["trajectories"], [])

            empty = root / "empty_room_b"
            empty.mkdir()
            code, report, _ = _run(["--session", str(empty), "--out", str(out)])
            self.assertEqual(code, 1)
            self.assertEqual(len(report["maps"]), 1)
            self.assertEqual(len(report["trajectories"]), 1)
            self.assertEqual(report["maps"][0]["status"], "missing")
            self.assertEqual(report["trajectories"][0]["status"], "missing")
            self.assertTrue(report["maps"][0]["path"].endswith("/empty_room_b/map.pcd"))
            self.assertTrue(
                report["trajectories"][0]["path"].endswith("/empty_room_b/trajectory.tum")
            )

            missing_map = root / "no_such_cloud.pcd"
            missing_traj = root / "no_such_track.tum"
            code, report, _ = _run(
                ["--map", str(missing_map), "--trajectory", str(missing_traj), "--out", str(out)]
            )
            self.assertEqual(code, 1)
            self.assertEqual(report["maps"][0]["status"], "missing")
            self.assertEqual(report["maps"][0]["path"], str(missing_map))
            self.assertEqual(report["trajectories"][0]["status"], "missing")
            self.assertEqual(report["trajectories"][0]["path"], str(missing_traj))

            good_map = root / "small_cloud.pcd"
            _write_map(good_map, 2)
            bad_traj = root / "broken_track.tum"
            bad_traj.write_text(
                "1.0 0 0 0 0 0 0 1\nthis line is not eight numbers\n",
                encoding="utf-8",
            )
            code, report, _ = _run(
                ["--map", str(good_map), "--trajectory", str(bad_traj), "--out", str(out)]
            )
            self.assertEqual(code, 1)
            self.assertEqual(report["maps"][0]["status"], "read")
            self.assertEqual(report["maps"][0]["points_read"], 2)
            self.assertEqual(report["trajectories"][0]["status"], "TumFormatError")
            self.assertIn("line 2", report["trajectories"][0]["error"])

            bad_map = root / "broken_cloud.pcd"
            bad_map.write_text("not a point cloud\n", encoding="utf-8")
            code, report, _ = _run(["--map", str(bad_map), "--out", str(out)])
            self.assertEqual(code, 1)
            self.assertEqual(report["maps"][0]["status"], "PcdFormatError")
            self.assertEqual(report["trajectories"], [])

            # A later run with no paths must not keep files named by an earlier run.
            code, report, _ = _run(["--out", str(out)])
            self.assertEqual(code, 1)
            self.assertEqual(report["status"], "missing required inputs")
            self.assertEqual(report["maps"], [])
            self.assertEqual(report["trajectories"], [])

    def test_smoke_explicit_paths_read_a_different_synthetic_session(self):
        source = Path(main.__code__.co_filename).read_text(encoding="utf-8")
        for token in _RECORDING_TOKENS:
            self.assertNotIn(token, source)
        import spatio.io.smoke_real as smoke_real

        self.assertFalse(hasattr(smoke_real, "MAPS"))
        self.assertFalse(hasattr(smoke_real, "SESSIONS"))

        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            hall = root / "walk_east_hall"
            stairs = root / "north_stairs"
            for session, points, poses, t0 in (
                (hall, 4, 3, 100.0),
                (stairs, 6, 7, 200.0),
            ):
                session.mkdir()
                _write_map(session / "map.pcd", points)
                _write_traj(session / "trajectory.tum", poses, t0)
            cabinet = root / "cabinet_scan.pcd"
            track = root / "push_track.tum"
            _write_map(cabinet, 2)
            _write_traj(track, 5, 50.0)
            out = root / "report.json"
            proc = subprocess.run(
                [
                    sys.executable,
                    "-m",
                    "spatio.io.smoke_real",
                    "--session",
                    str(hall),
                    "--session",
                    str(stairs),
                    "--map",
                    str(cabinet),
                    "--trajectory",
                    str(track),
                    "--out",
                    str(out),
                ],
                cwd=str(REPO),
                capture_output=True,
                text=True,
            )
            self.assertEqual(proc.returncode, 0, proc.stdout + "\n" + proc.stderr)
            report = json.loads(proc.stdout)
            self.assertNotIn("status", report)
            self.assertEqual(json.loads(out.read_text(encoding="utf-8")), report)
            blob = json.dumps(report)
            for token in _RECORDING_TOKENS:
                self.assertNotIn(token, blob)
            self.assertEqual(
                [(item["path"], item["status"], item["points_read"]) for item in report["maps"]],
                [
                    (str(cabinet), "read", 2),
                    (str(hall / "map.pcd"), "read", 4),
                    (str(stairs / "map.pcd"), "read", 6),
                ],
            )
            self.assertEqual(
                [(item["path"], item["status"], item["n_poses"]) for item in report["trajectories"]],
                [
                    (str(track), "read", 5),
                    (str(hall / "trajectory.tum"), "read", 3),
                    (str(stairs / "trajectory.tum"), "read", 7),
                ],
            )
            hall_traj = report["trajectories"][1]
            self.assertEqual(hall_traj["issues"], [])
            self.assertLessEqual(abs(hall_traj["duration_s"] - 0.2), 1e-9)
            self.assertEqual(report["maps"][1]["points_header"], 4)
            self.assertEqual(report["maps"][1]["data"], "ascii")
