"""Raw-scan start->end registration: CDR decoding and recovery of a known offset."""

import os
import sys
import unittest

import numpy as np

SYNTH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "synth")
sys.path.insert(0, SYNTH)
from scene import Path, SceneParams, scan  # noqa: E402
from write_bag import custom_msg_cdr  # noqa: E402

from spatio.walkday.startend import _cdr_custom_msg, register  # noqa: E402


class StartEndTest(unittest.TestCase):
    def test_custom_msg_cdr_roundtrip(self):
        rng = np.random.default_rng(3)
        for frame in ("a", "synth_lidar", "frame_with_len_8"):
            n = 257
            xyz = rng.normal(0, 5, (n, 3)).astype(np.float32)
            data = custom_msg_cdr(1_800_000_000_123_456_789, frame, np.arange(n, dtype=np.uint32),
                                  xyz, np.full(n, 9, np.uint8), np.zeros(n, np.uint8))
            t, out = _cdr_custom_msg(data)
            self.assertAlmostEqual(t, 1_800_000_000.123456789, places=6)
            np.testing.assert_array_equal(out, xyz.astype(np.float64))

    def _hold_cloud(self, path, t, rng, scans=20):
        k0 = int(round((t - path.p.t0) * path.p.lidar_rate_hz))
        return np.vstack([scan(path, k, rng)[2].astype(np.float64) for k in range(k0, k0 + scans)])

    def test_recovers_true_end_offset(self):
        p = SceneParams(end_offset_m=(0.12, -0.05, 0.0), points_per_scan=6000, range_noise_m=0.005)
        path = Path(p)
        rng = np.random.default_rng(5)
        start = self._hold_cloud(path, p.t0, rng)
        end = self._hold_cloud(path, path.t_move1 + 1.0, rng)
        winner, res = register(start, end, grid=False)
        r = res[winner]
        self.assertAlmostEqual(r["translation_m"], 0.13, delta=0.01)
        self.assertLess(abs(r["yaw_deg"]), 0.3)
        self.assertGreater(r["inlier_frac_5cm"], 0.9)


if __name__ == "__main__":
    unittest.main()
