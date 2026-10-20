"""Strict TUM reader, writer, and checker."""

from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

import numpy as np

from spatio.io.synth import square_loop
from spatio.io.tum import (
    Trajectory,
    TumFormatError,
    check_trajectory,
    read_tum,
    write_tum,
)


def _identity(n):
    quat = np.zeros((n, 4), dtype=np.float64)
    quat[:, 3] = 1.0
    return quat


def _memory(times, xyz=None, quat=None):
    t = np.asarray(times, dtype=np.float64)
    n = int(t.shape[0])
    if xyz is None:
        xyz = np.zeros((n, 3), dtype=np.float64)
    if quat is None:
        quat = _identity(n)
    return Trajectory(t, xyz, quat, "memory")


class TumTests(unittest.TestCase):
    def test_tum_roundtrip_exact(self):
        # %.9f keeps time, position, and quaternion inside 1e-9.
        traj = square_loop(end_offset_m=(0.03, 0.04, 0.0), end_yaw_deg=10.0)
        with tempfile.TemporaryDirectory() as tmp:
            path = str(Path(tmp) / "trajectory.tum")
            write_tum(path, traj, comments=("round trip",))
            text = Path(path).read_text(encoding="utf-8")
            self.assertTrue(text.startswith("# round trip\n"))
            self.assertNotIn("\r", text)
            back = read_tum(path)
        self.assertLessEqual(float(np.max(np.abs(back.t - traj.t))), 1e-9)
        self.assertLessEqual(float(np.max(np.abs(back.xyz - traj.xyz))), 1e-9)
        self.assertLessEqual(
            float(np.max(np.abs(back.quat_xyzw - traj.quat_xyzw))), 1e-9
        )
        self.assertEqual(check_trajectory(back), [])
        # File order is the array order, including the opening hold.
        self.assertLess(float(back.t[0]), float(back.t[-1]))

    def test_tum_position_roundtrip_within_1e9(self):
        # 1.100083214967233 is the square-loop coordinate whose %.9g form
        # moved by 4.967e-9 m. The writer must keep that point inside 1e-9 m.
        bad = 1.100083214967233
        self.assertGreater(abs(float("%.9g" % bad) - bad), 1e-9)
        xyz = np.array(
            [
                [bad, -4.000000123, 0.03],
                [9.999999994, 0.150000001, -0.08],
                [0.0, 10.5, 100.123456789],
            ],
            dtype=np.float64,
        )
        traj = _memory([1786223548.0, 1786223549.0, 1786223550.0], xyz=xyz)
        with tempfile.TemporaryDirectory() as tmp:
            path = str(Path(tmp) / "trajectory.tum")
            write_tum(path, traj)
            text = Path(path).read_text(encoding="utf-8")
            back = read_tum(path)
        self.assertIn("%.9f" % bad, text)
        self.assertLessEqual(float(np.max(np.abs(back.t - traj.t))), 1e-9)
        self.assertLessEqual(float(np.max(np.abs(back.xyz - traj.xyz))), 1e-9)
        self.assertLessEqual(
            float(np.max(np.abs(back.quat_xyzw - traj.quat_xyzw))), 1e-9
        )

    def test_tum_comment_and_blank_lines(self):
        body = "\n".join(
            [
                "# header",
                "",
                "   ",
                "0 0 0 0 0 0 0 1",
                "",
                "# mid",
                "1 2 3 4 0 0 0 1",
                "",
            ]
        )
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "trajectory.tum"
            path.write_text(body, encoding="utf-8")
            traj = read_tum(str(path))
        self.assertEqual(len(traj), 2)
        self.assertEqual(float(traj.t[0]), 0.0)
        self.assertEqual(float(traj.t[1]), 1.0)
        np.testing.assert_array_equal(traj.xyz[1], np.array([2.0, 3.0, 4.0]))
        np.testing.assert_array_equal(traj.quat_xyzw[0], np.array([0.0, 0.0, 0.0, 1.0]))

    def test_tum_rejects_wrong_column_count_with_line_number(self):
        body = "\n".join(
            [
                "# comment",
                "",
                "0 0 0 0 0 0 0 1",
                "1 0 0 0 0 0 0",
                "2 0 0 0 0 0 0 1 9",
            ]
        )
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "trajectory.tum"
            path.write_text(body + "\n", encoding="utf-8")
            with self.assertRaises(TumFormatError) as caught:
                read_tum(str(path))
        self.assertEqual(caught.exception.line_number, 4)
        self.assertIn("8 columns", str(caught.exception))

    def test_tum_rejects_nan_and_inf(self):
        samples = ("0 nan 0 0 0 0 0 1\n", "0 0 inf 0 0 0 0 1\n", "0 0 0 -inf 0 0 0 1\n")
        for body in samples:
            with tempfile.TemporaryDirectory() as tmp:
                path = Path(tmp) / "trajectory.tum"
                path.write_text("# c\n" + body, encoding="utf-8")
                with self.assertRaises(TumFormatError) as caught:
                    read_tum(str(path))
            self.assertEqual(caught.exception.line_number, 2)
            self.assertIn("non-finite", str(caught.exception))

    def test_tum_rejects_non_unit_quaternion_unless_normalize(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            bad = root / "bad.tum"
            bad.write_text("0 0 0 0 0.5 0 0 0\n", encoding="utf-8")
            with self.assertRaises(TumFormatError) as caught:
                read_tum(str(bad))
            self.assertEqual(caught.exception.line_number, 1)
            zero = root / "zero.tum"
            zero.write_text("0 0 0 0 0 0 0 0\n", encoding="utf-8")
            with self.assertRaises(TumFormatError) as caught_zero:
                read_tum(str(zero), normalize_quat=True)
            self.assertEqual(caught_zero.exception.line_number, 1)
            half = root / "half.tum"
            half.write_text("0 0 0 0 0.5 0 0 0\n", encoding="utf-8")
            normalized = read_tum(str(half), normalize_quat=True)
            np.testing.assert_allclose(normalized.quat_xyzw[0], [1.0, 0.0, 0.0, 0.0])
            on_edge = root / "edge.tum"
            on_edge.write_text("0 0 0 0 %.17g 0 0 0\n" % (1.0 + 1e-3), encoding="utf-8")
            accepted = read_tum(str(on_edge))
            self.assertEqual(len(accepted), 1)
            over = root / "over.tum"
            over.write_text("0 0 0 0 %.17g 0 0 0\n" % (1.0 + 1.1e-3), encoding="utf-8")
            with self.assertRaises(TumFormatError) as caught_over:
                read_tum(str(over))
            self.assertEqual(caught_over.exception.line_number, 1)

    def test_tum_never_sorts_or_dedupes(self):
        times = [3.0, 1.0, 1.0, 2.0]
        xyz = np.zeros((4, 3), dtype=np.float64)
        xyz[:, 0] = np.array([0.0, 1.0, 2.0, 3.0])
        body = []
        for i, stamp in enumerate(times):
            body.append(
                "%.17g %.17g 0 0 0 0 0 1" % (stamp, float(xyz[i, 0]))
            )
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "trajectory.tum"
            path.write_text("\n".join(body) + "\n", encoding="utf-8")
            traj = read_tum(str(path))
        self.assertEqual(len(traj), 4)
        np.testing.assert_array_equal(traj.t, np.array(times))
        np.testing.assert_array_equal(traj.xyz[:, 0], xyz[:, 0])

    def test_epoch_timestamps_keep_microsecond_precision(self):
        t0 = 1786223548.870123
        t = t0 + np.arange(1000, dtype=np.float64) * 1e-6
        traj = _memory(t)
        with tempfile.TemporaryDirectory() as tmp:
            path = str(Path(tmp) / "trajectory.tum")
            write_tum(path, traj)
            back = read_tum(path)
        self.assertTrue(np.array_equal(back.t, t))
        self.assertEqual(int(np.unique(back.t).size), 1000)
        self.assertLess(float(np.max(np.abs(back.t - t))), 1e-6)

    def test_check_trajectory_flags_backwards_duplicate_gap_short(self):
        empty = check_trajectory(_memory([]))
        one = check_trajectory(_memory([1786223548.0]))
        self.assertEqual(len(empty), 1)
        self.assertEqual(empty[0].code, "TOO_SHORT")
        self.assertIsNone(empty[0].index)
        self.assertEqual(one[0].code, "TOO_SHORT")
        times = np.array([0.0, 1.0, 1.0, 0.5, 1.0, 1.5 + 1e-9], dtype=np.float64)
        traj = _memory(times)
        before = traj.t.copy()
        issues = check_trajectory(traj, max_gap_s=0.5)
        np.testing.assert_array_equal(traj.t, before)
        self.assertEqual(
            [issue.code for issue in issues],
            ["GAP", "DUPLICATE_T", "BACKWARDS", "GAP"],
        )
        self.assertEqual([issue.index for issue in issues], [1, 2, 3, 5])
        self.assertEqual(issues[2].dt, -0.5)
        self.assertEqual(issues[0].gap_s, 1.0)
        self.assertGreater(issues[3].gap_s, 0.5)
        self.assertNotIn(4, [issue.index for issue in issues])
        grid = _memory(1786223548.0 + np.arange(50, dtype=np.float64) / 10.0)
        self.assertEqual(check_trajectory(grid), [])


if __name__ == "__main__":
    unittest.main()
