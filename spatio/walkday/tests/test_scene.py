"""Synthetic marked loop: kinematics, IMU and raycast agree with each other."""

import math
import os
import sys
import unittest

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "synth"))
from scene import G, Path, SceneParams, ground_truth, imu_samples, raycast, rotz, scan  # noqa: E402


class SceneTest(unittest.TestCase):
    def test_truth_end_offset_and_heading(self):
        for off in ((0.05, 0, 0), (0.15, 0, 0), (0.0, 0.08, 0.0)):
            gt = ground_truth(Path(SceneParams(end_offset_m=off)))
            self.assertAlmostEqual(gt["true_return_error_3d_m"], math.hypot(*off), places=9)
            self.assertAlmostEqual(gt["true_heading_change_deg"], 0.0, places=6)

    def test_parked_at_both_ends(self):
        p = SceneParams(t0=0.0)
        path = Path(p)
        t = np.concatenate([np.linspace(0, p.hold_start_s, 50), np.linspace(path.t_move1, path.t_end, 50)])
        self.assertTrue(np.all(path.theta_dot(t) == 0.0))

    def test_acceleration_is_second_derivative_of_position(self):
        p = SceneParams(t0=0.0)        # relative time: float precision for finite differences
        path = Path(p)
        t = np.linspace(0, path.t_end, 100001)
        dt = t[1] - t[0]
        pos, yaw, acc, yaw_rate = path.state(t)
        num = np.gradient(np.gradient(pos, dt, axis=0), dt, axis=0)
        self.assertLess(np.abs(num - acc)[10:-10].max(), 1e-3)
        self.assertLess(np.abs(np.gradient(np.unwrap(yaw), dt) - yaw_rate)[5:-5].max(), 1e-3)

    def test_noise_free_imu_dead_reckons_the_loop(self):
        p = SceneParams(t0=0.0, acc_noise_m_s2=0.0, gyro_noise_rad_s=0.0)
        path = Path(p)
        t, acc_g, gyro = imu_samples(path, np.random.default_rng(0))
        pos0, yaw0, _, _ = path.state(t[:1])
        x, v, y, h = pos0[0].copy(), np.zeros(3), yaw0[0], 1.0 / p.imu_rate_hz
        for i in range(len(t) - 1):
            a = rotz(np.array([y]))[0] @ (acc_g[i] * G) - np.array([0, 0, G])
            x, v, y = x + v * h + 0.5 * a * h * h, v + a * h, y + gyro[i, 2] * h
        truth = path.state(t[-1:])[0][0]
        self.assertLess(np.linalg.norm(x - truth), 0.02)

    def test_raycast_hits_walls_and_boxes(self):
        p = SceneParams(boxes=[((2.0, -0.5, 0.0), (3.0, 0.5, 2.0))])
        o = np.array([[0.0, 0.0, 1.0]] * 3)
        d = np.array([[1.0, 0, 0], [-1.0, 0, 0], [0, 0, 1.0]])
        r = raycast(p, o, d)
        np.testing.assert_allclose(r, [2.0, 1.5, 2.2])

    def test_scan_points_are_in_sensor_frame(self):
        p = SceneParams(range_noise_m=0.0, points_per_scan=2000, boxes=[])
        path = Path(p)
        _, off, xyz = scan(path, 0, np.random.default_rng(1))   # parked: pose is the start pose
        pos, yaw, _, _ = path.state(np.array([p.t0]))
        world = np.einsum("ij,nj->ni", rotz(yaw)[0], xyz.astype(float)) + pos[0]
        lo, hi = np.array(p.room_min), np.array(p.room_max)
        on_face = np.min(np.minimum(np.abs(world - lo), np.abs(world - hi)), axis=1)
        self.assertLess(on_face.max(), 1e-3)
        self.assertTrue(np.all(np.diff(off.astype(np.int64)) >= 0))


if __name__ == "__main__":
    unittest.main()
