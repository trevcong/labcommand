import math
import os
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import tum  # noqa: E402

IDENTITY = (0.0, 0.0, 0.0, 1.0)


def yaw_quat(deg):
    h = math.radians(deg) / 2.0
    return (0.0, 0.0, math.sin(h), math.cos(h))


class StampTests(unittest.TestCase):
    def test_nanoseconds_are_kept(self):
        t = tum.stamp_to_seconds(1754687533, 123456789)
        self.assertEqual(tum.format_line(tum.make_pose(t, (0, 0, 0), IDENTITY)).split()[0],
                         "1754687533.123456717")  # float64 limit, still sub-microsecond

    def test_rejects_bad_nanosec(self):
        with self.assertRaises(tum.TumError):
            tum.stamp_to_seconds(1, 1_000_000_000)


class PoseValidationTests(unittest.TestCase):
    def test_rejects_non_unit_quaternion(self):
        with self.assertRaises(tum.TumError):
            tum.make_pose(0.0, (0, 0, 0), (0.0, 0.0, 0.0, 0.9))

    def test_rejects_nan(self):
        with self.assertRaises(tum.TumError):
            tum.make_pose(0.0, (float("nan"), 0, 0), IDENTITY)

    def test_field_order_is_x_y_z_qx_qy_qz_qw(self):
        p = tum.make_pose(1.5, (1, 2, 3), (0.0, 0.0, math.sqrt(0.5), math.sqrt(0.5)))
        fields = tum.format_line(p).split()
        self.assertEqual([float(v) for v in fields[:4]], [1.5, 1.0, 2.0, 3.0])
        self.assertAlmostEqual(float(fields[6]), math.sqrt(0.5), places=8)
        self.assertAlmostEqual(float(fields[7]), math.sqrt(0.5), places=8)


class RoundTripTests(unittest.TestCase):
    def test_write_then_read(self):
        poses = [tum.make_pose(10.0 + i * 0.1, (i * 0.5, -i, 0.01 * i), yaw_quat(i * 10))
                 for i in range(5)]
        with tempfile.TemporaryDirectory() as d:
            path = os.path.join(d, "t.tum")
            n = tum.write_tum(path, poses, comments=["source: test", "two\nlines"])
            self.assertEqual(n, 5)
            with open(path) as f:
                text = f.read()
            self.assertTrue(text.startswith("# source: test\n# two\n# lines\n"))
            back = tum.read_tum(path)
        self.assertEqual(len(back), 5)
        for a, b in zip(poses, back):
            for u, v in zip(a, b):
                self.assertAlmostEqual(u, v, places=6)

    def test_read_rejects_short_line(self):
        with tempfile.TemporaryDirectory() as d:
            path = os.path.join(d, "bad.tum")
            with open(path, "w") as f:
                f.write("# ok\n1 2 3 4 0 0 0\n")
            with self.assertRaisesRegex(tum.TumError, "line 2"):
                tum.read_tum(path)


class SummaryTests(unittest.TestCase):
    def test_closed_loop_reports_raw_endpoint_distance(self):
        # Square loop that ends 5 cm east and 2 cm up from the start.
        pts = [(0, 0, 0), (4, 0, 0), (4, 3, 0), (0, 3, 0), (0.05, 0, 0.02)]
        poses = [tum.make_pose(float(i), p, IDENTITY) for i, p in enumerate(pts)]
        s = tum.summarize(poses)
        self.assertAlmostEqual(s["start_end_3d_m"], math.hypot(0.05, 0.02), places=9)
        self.assertAlmostEqual(s["start_end_horizontal_m"], 0.05, places=9)
        self.assertAlmostEqual(s["start_end_vertical_m"], 0.02, places=9)
        self.assertAlmostEqual(s["path_length_m"], 4 + 3 + 4 + math.dist((0, 3, 0), (0.05, 0, 0.02)))
        self.assertAlmostEqual(s["max_distance_from_start_m"], 5.0)

    def test_open_walk_is_not_hidden(self):
        poses = [tum.make_pose(float(i), (i * 1.0, 0, 0), IDENTITY) for i in range(6)]
        self.assertAlmostEqual(tum.summarize(poses)["start_end_3d_m"], 5.0)

    def test_yaw_change_reported_and_wrapped(self):
        poses = [tum.make_pose(0.0, (0, 0, 0), yaw_quat(170)),
                 tum.make_pose(1.0, (0, 0, 0), yaw_quat(-170))]
        s = tum.summarize(poses)
        self.assertAlmostEqual(s["start_end_yaw_deg"], 20.0, places=6)
        self.assertAlmostEqual(s["start_end_rotation_deg"], 20.0, places=6)

    def test_time_problems_are_counted_not_fixed(self):
        ts = [0.0, 0.1, 0.1, 0.05, 2.0]
        poses = [tum.make_pose(t, (0, 0, 0), IDENTITY) for t in ts]
        s = tum.summarize(poses)
        self.assertEqual(s["duplicate_time_steps"], 1)
        self.assertEqual(s["backward_time_steps"], 1)
        self.assertEqual(len(s["gaps_over_threshold"]), 1)
        self.assertAlmostEqual(s["gaps_over_threshold"][0]["dt"], 1.95)
        self.assertEqual(s["poses"], 5)

    def test_empty_trajectory_fails(self):
        with self.assertRaises(tum.TumError):
            tum.summarize([])


class WindowClosureTests(unittest.TestCase):
    def test_means_of_head_and_tail_windows(self):
        # Parked at x=0 (0..4 s, jittering ±1 cm), drive, parked at x=0.10 (16..20 s).
        poses = [tum.make_pose(float(t), (0.01 * (-1) ** t, 0, 0), IDENTITY) for t in range(0, 5)]
        poses += [tum.make_pose(float(t), (1.0, 2.0, 0), IDENTITY) for t in range(5, 16)]
        poses += [tum.make_pose(float(t), (0.10, 0, 0.03), IDENTITY) for t in range(16, 21)]
        c = tum.window_closure(poses, 4.0)
        self.assertEqual((c["head_poses"], c["tail_poses"]), (5, 5))
        self.assertAlmostEqual(c["closure_horizontal_m"], 0.10 - 0.002, places=9)
        self.assertAlmostEqual(c["closure_vertical_m"], 0.03, places=9)
        self.assertAlmostEqual(c["closure_3d_m"], math.hypot(0.098, 0.03), places=9)

    def test_open_walk_is_not_hidden(self):
        poses = [tum.make_pose(float(i), (i * 1.0, 0, 0), IDENTITY) for i in range(30)]
        self.assertAlmostEqual(tum.window_closure(poses, 4.0)["closure_3d_m"], 25.0)

    def test_empty_trajectory_fails(self):
        with self.assertRaises(tum.TumError):
            tum.window_closure([])


if __name__ == "__main__":
    unittest.main()
