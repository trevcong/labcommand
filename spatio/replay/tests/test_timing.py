import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import timing  # noqa: E402


class StreamStatsTests(unittest.TestCase):
    def test_constant_receipt_lag(self):
        h = [100.0 + 0.1 * i for i in range(11)]
        r = [t + 0.02 for t in h]
        s = timing.stream_stats(h, r)
        self.assertEqual(s["count"], 11)
        self.assertAlmostEqual(s["receipt_minus_header_s"]["median"], 0.02)
        self.assertAlmostEqual(s["header_rate_hz"], 10.0)
        self.assertEqual(s["header_backward_steps"], 0)
        self.assertEqual(s["header_gaps_over_3x_median"], 0)

    def test_backward_jump_and_gap_counted(self):
        h = [0.0, 0.1, 0.2, 0.15, 0.3, 1.3, 1.4]
        r = [0.0, 0.1, 0.2, 0.25, 0.3, 1.3, 1.4]
        s = timing.stream_stats(h, r)
        self.assertEqual(s["header_backward_steps"], 1)
        self.assertEqual(s["header_out_of_order_vs_receipt"], 1)
        self.assertEqual(s["header_gaps_over_3x_median"], 1)
        self.assertAlmostEqual(s["header_dt_max_s"], 1.0)

    def test_length_mismatch_rejected(self):
        with self.assertRaises(ValueError):
            timing.stream_stats([0.0], [])


class ImuCoverageTests(unittest.TestCase):
    def setUp(self):
        self.imu = [i * 0.005 for i in range(0, 401)]  # 200 Hz, 0..2 s

    def test_all_scans_covered(self):
        scans = [(0.1 * i + 0.01, 0.1 * i + 0.11) for i in range(10)]
        c = timing.imu_coverage(scans, self.imu)
        self.assertEqual(c["covered"], 10)
        self.assertAlmostEqual(c["worst_imu_gap_inside_scan_s"], 0.005)
        self.assertAlmostEqual(c["scan_span_s"]["median"], 0.1)

    def test_scan_past_imu_end_is_not_covered(self):
        scans = [(1.95, 2.05), (2.1, 2.2)]
        c = timing.imu_coverage(scans, self.imu)
        self.assertEqual(c["covered"], 0)
        self.assertEqual(c["scans_without_imu_after_end"], 2)

    def test_scan_before_imu_start_is_not_covered(self):
        c = timing.imu_coverage([(-0.05, 0.05)], self.imu)
        self.assertEqual(c["scans_without_imu_before_start"], 1)
        self.assertEqual(c["covered"], 0)

    def test_imu_gap_inside_scan_reported(self):
        imu = [t for t in self.imu if not (0.52 < t < 0.58)]
        c = timing.imu_coverage([(0.5, 0.6)], imu)
        self.assertEqual(c["covered"], 1)
        self.assertAlmostEqual(c["worst_imu_gap_inside_scan_s"], 0.06, places=6)

    def test_unsorted_imu_rejected(self):
        with self.assertRaises(ValueError):
            timing.imu_coverage([(0, 1)], [0.0, 0.2, 0.1])


class AccelUnitsTests(unittest.TestCase):
    def test_guesses(self):
        self.assertTrue(timing.accel_units_guess([9.81, 9.79, 9.8])["guess"].startswith("m/s^2"))
        self.assertTrue(timing.accel_units_guess([1.0, 0.99, 1.01])["guess"].startswith("g "))
        self.assertEqual(timing.accel_units_guess([4.0])["guess"], "unclear")


if __name__ == "__main__":
    unittest.main()
