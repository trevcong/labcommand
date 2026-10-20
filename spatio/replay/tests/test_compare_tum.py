import math
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import compare_tum  # noqa: E402
import tum  # noqa: E402

I = (0.0, 0.0, 0.0, 1.0)


def traj(ts, f):
    return [tum.make_pose(t, f(t), I) for t in ts]


class CompareTests(unittest.TestCase):
    def test_identical(self):
        a = traj([0.0, 0.1, 0.2], lambda t: (t, 0, 0))
        r = compare_tum.compare(a, a)
        self.assertEqual(r["matched"], 3)
        self.assertEqual(r["position_diff_m"]["max"], 0.0)

    def test_constant_offset_is_not_aligned_away(self):
        a = traj([0.0, 0.1, 0.2], lambda t: (t, 0, 0))
        b = traj([0.0, 0.1, 0.2], lambda t: (t + 0.3, 0, 0))
        r = compare_tum.compare(a, b)
        self.assertAlmostEqual(r["position_diff_m"]["median"], 0.3)
        self.assertAlmostEqual(r["position_diff_m"]["last"], 0.3)

    def test_only_close_timestamps_match(self):
        a = traj([0.0, 0.1, 0.2], lambda t: (0, 0, 0))
        b = traj([0.002, 0.15, 0.199], lambda t: (0, 0, 0))
        r = compare_tum.compare(a, b, max_dt=0.005)
        self.assertEqual(r["matched"], 2)

    def test_rotation_difference(self):
        h = math.radians(10) / 2
        a = [tum.make_pose(0.0, (0, 0, 0), I)]
        b = [tum.make_pose(0.0, (0, 0, 0), (0, 0, math.sin(h), math.cos(h)))]
        self.assertAlmostEqual(compare_tum.compare(a, b)["rotation_diff_deg"]["max"], 10.0, places=6)

    def test_no_overlap(self):
        a = traj([0.0], lambda t: (0, 0, 0))
        b = traj([5.0], lambda t: (0, 0, 0))
        r = compare_tum.compare(a, b)
        self.assertEqual(r["matched"], 0)
        self.assertNotIn("position_diff_m", r)


if __name__ == "__main__":
    unittest.main()
