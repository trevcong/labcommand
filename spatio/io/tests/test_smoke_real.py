"""Exit codes and explicit paths for the read-only smoke command."""

from __future__ import annotations

import json
import os
import subprocess
import sys
import tempfile
import unittest
from contextlib import redirect_stderr, redirect_stdout
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


def _other_case(path):
    """The same name with each letter's case flipped. The OS decides if it collides."""

    chars = []
    changed = False
    for char in path.name:
        if char.islower():
            chars.append(char.upper())
            changed = True
        elif char.isupper():
            chars.append(char.lower())
            changed = True
        else:
            chars.append(char)
    if not changed:
        raise AssertionError("name has no letters to flip: %s" % path.name)
    return path.with_name("".join(chars))


def _run_capture(args):
    stdout = StringIO()
    stderr = StringIO()
    with redirect_stdout(stdout), redirect_stderr(stderr):
        code = main(list(args))
    return code, stdout.getvalue(), stderr.getvalue()


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

    def test_smoke_refuses_output_that_aliases_an_input(self):
        # A report path that names an input used to replace that file and exit 0.
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            traj = root / "push_west.tum"
            cloud = root / "west_corridor.pcd"
            _write_traj(traj, 4, 11.0)
            _write_map(cloud, 3)
            before_traj = traj.read_bytes()
            before_cloud = cloud.read_bytes()

            code, stdout, stderr = _run_capture(
                ["--trajectory", str(traj), "--out", str(traj)]
            )
            self.assertEqual(code, 1, stderr)
            self.assertEqual(traj.read_bytes(), before_traj)
            self.assertNotIn("{", stdout)
            self.assertIn("same file", stderr)

            code, _, stderr = _run_capture(
                ["--map", str(cloud), "--out", str(cloud)]
            )
            self.assertEqual(code, 1, stderr)
            self.assertEqual(cloud.read_bytes(), before_cloud)

            # `..` and a resolved path (/var -> /private/var on this Mac) are the same file.
            code, _, stderr = _run_capture(
                ["--trajectory", str(traj), "--out", str(traj.parent / "." / traj.name)]
            )
            self.assertEqual(code, 1, stderr)
            self.assertEqual(traj.read_bytes(), before_traj)
            code, _, stderr = _run_capture(
                ["--map", str(cloud), "--out", str(cloud.resolve())]
            )
            self.assertEqual(code, 1, stderr)
            self.assertEqual(cloud.read_bytes(), before_cloud)

            cwd = os.getcwd()
            try:
                os.chdir(root)
                code, _, stderr = _run_capture(
                    ["--trajectory", traj.name, "--out", str(traj)]
                )
            finally:
                os.chdir(cwd)
            self.assertEqual(code, 1, stderr)
            self.assertEqual(traj.read_bytes(), before_traj)

            link = root / "track_alias.tum"
            link.symlink_to(traj.name)
            code, _, stderr = _run_capture(
                ["--trajectory", str(traj), "--out", str(link)]
            )
            self.assertEqual(code, 1, stderr)
            self.assertTrue(link.is_symlink())
            self.assertEqual(traj.read_bytes(), before_traj)
            code, _, stderr = _run_capture(
                ["--trajectory", str(link), "--out", str(traj)]
            )
            self.assertEqual(code, 1, stderr)
            self.assertEqual(traj.read_bytes(), before_traj)

            hard = root / "same_inode.tum"
            os.link(traj, hard)
            self.assertNotEqual(hard.resolve(), traj.resolve())
            self.assertEqual(hard.stat().st_ino, traj.stat().st_ino)
            code, _, stderr = _run_capture(
                ["--trajectory", str(traj), "--out", str(hard)]
            )
            self.assertEqual(code, 1, stderr)
            self.assertEqual(traj.read_bytes(), before_traj)
            via = root / "via_hard.tum"
            via.symlink_to(hard.name)
            code, _, stderr = _run_capture(
                ["--trajectory", str(traj), "--out", str(via)]
            )
            self.assertEqual(code, 1, stderr)
            self.assertEqual(traj.read_bytes(), before_traj)
            self.assertTrue(via.is_symlink())

            session = root / "north_atrium"
            session.mkdir()
            _write_map(session / "map.pcd", 2)
            _write_traj(session / "trajectory.tum", 3, 20.0)
            before_s_map = (session / "map.pcd").read_bytes()
            before_s_traj = (session / "trajectory.tum").read_bytes()
            for name in ("map.pcd", "trajectory.tum"):
                code, _, stderr = _run_capture(
                    ["--session", str(session), "--out", str(session / name)]
                )
                self.assertEqual(code, 1, stderr)
                self.assertEqual((session / "map.pcd").read_bytes(), before_s_map)
                self.assertEqual((session / "trajectory.tum").read_bytes(), before_s_traj)
            session_link = root / "atrium_link"
            session_link.symlink_to(session.name)
            code, _, stderr = _run_capture(
                [
                    "--session",
                    str(session_link),
                    "--out",
                    str(session / "trajectory.tum"),
                ]
            )
            self.assertEqual(code, 1, stderr)
            self.assertEqual((session / "trajectory.tum").read_bytes(), before_s_traj)

            missing = root / "fresh_hall" / "trajectory.tum"
            code, _, stderr = _run_capture(
                ["--trajectory", str(missing), "--out", str(missing)]
            )
            self.assertEqual(code, 1, stderr)
            self.assertIn("same file", stderr)
            self.assertFalse(missing.exists())
            self.assertFalse(missing.parent.exists())

            flipped = _other_case(traj)
            try:
                collides = os.path.samefile(traj, flipped)
            except OSError:
                collides = False
            code, _, stderr = _run_capture(
                ["--trajectory", str(traj), "--out", str(flipped)]
            )
            self.assertEqual(traj.read_bytes(), before_traj)
            if collides:
                self.assertEqual(code, 1, stderr)
                self.assertIn("same file", stderr)
            else:
                self.assertEqual(code, 0, stderr)
                self.assertNotEqual(flipped.resolve(), traj.resolve())
                json.loads(flipped.read_text(encoding="utf-8"))

            spelled = root / "Fresh_Hall.tum"
            other = root / "fresh_hall.tum"
            self.assertFalse(spelled.exists())
            try:
                parent_ci = os.path.samefile(root, _other_case(root))
            except OSError:
                parent_ci = False
            code, _, stderr = _run_capture(
                ["--trajectory", str(spelled), "--out", str(other)]
            )
            if parent_ci:
                self.assertEqual(code, 1, stderr)
                self.assertFalse(spelled.exists())
                self.assertFalse(other.exists())
            else:
                self.assertEqual(code, 1, stderr)
                self.assertFalse(spelled.exists())
                report = json.loads(other.read_text(encoding="utf-8"))
                self.assertEqual(report["trajectories"][0]["status"], "missing")

            loop = root / "loop_link"
            loop.symlink_to(loop.name)
            loop_report = root / "loop_report.json"
            code, text, stderr = _run_capture(
                ["--trajectory", str(loop), "--out", str(loop_report)]
            )
            self.assertEqual(code, 1, stderr)
            self.assertTrue(loop.is_symlink())
            report = json.loads(loop_report.read_text(encoding="utf-8"))
            self.assertEqual(json.loads(text), report)
            self.assertEqual(report["trajectories"][0]["status"], "missing")

            notes = session / "notes.json"
            code, text, stderr = _run_capture(
                [
                    "--session",
                    str(session),
                    "--map",
                    str(cloud),
                    "--trajectory",
                    str(traj),
                    "--out",
                    str(notes),
                ]
            )
            self.assertEqual(code, 0, stderr)
            report = json.loads(notes.read_text(encoding="utf-8"))
            self.assertEqual(json.loads(text), report)
            self.assertEqual(traj.read_bytes(), before_traj)
            self.assertEqual(cloud.read_bytes(), before_cloud)
            self.assertEqual((session / "map.pcd").read_bytes(), before_s_map)
            self.assertEqual((session / "trajectory.tum").read_bytes(), before_s_traj)
            self.assertEqual(
                [item["points_read"] for item in report["maps"]],
                [3, 2],
            )
            self.assertEqual(
                [item["n_poses"] for item in report["trajectories"]],
                [4, 3],
            )

            east = root / "push_east.tum"
            _write_traj(east, 5, 30.0)
            before_east = east.read_bytes()
            proc = subprocess.run(
                [
                    sys.executable,
                    "-m",
                    "spatio.io.smoke_real",
                    "--trajectory",
                    str(east),
                    "--out",
                    str(east),
                ],
                cwd=str(REPO),
                capture_output=True,
                text=True,
            )
            self.assertEqual(proc.returncode, 1, proc.stdout + "\n" + proc.stderr)
            self.assertEqual(east.read_bytes(), before_east)
            self.assertNotIn("{", proc.stdout)
            self.assertIn("same file", proc.stderr)
