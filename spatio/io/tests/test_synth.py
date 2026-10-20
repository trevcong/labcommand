"""Deterministic synthetic loops. Scoring here is local, not make_fixtures."""

from __future__ import annotations

import math
import unittest

import numpy as np

from spatio.io.synth import room_map, square_loop, stationary, with_gap, with_reset

T0 = 1786223548.0
WINDOW_S = 10.0
EDGE_S = 1e-6


def _polyline_length(offset):
    corners = np.array(
        [[0.0, 0.0, 0.0], [4.0, 0.0, 0.0], [4.0, 4.0, 0.0], [0.0, 4.0, 0.0], list(offset)],
        dtype=np.float64,
    )
    return float(np.linalg.norm(np.diff(corners, axis=0), axis=1).sum())


def _yaw(quat):
    x = quat[:, 0]
    y = quat[:, 1]
    z = quat[:, 2]
    w = quat[:, 3]
    return np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _return_error(traj):
    """Mean of the first WINDOW_S seconds versus the mean of the last WINDOW_S."""

    t = traj.t
    xyz = traj.xyz
    start = t <= float(t[0]) + WINDOW_S + EDGE_S
    end = t >= float(t[-1]) - WINDOW_S - EDGE_S
    delta = xyz[end].mean(axis=0) - xyz[start].mean(axis=0)
    start_yaw = math.atan2(float(np.sin(_yaw(traj.quat_xyzw[start])).mean()), float(np.cos(_yaw(traj.quat_xyzw[start])).mean()))
    end_yaw = math.atan2(float(np.sin(_yaw(traj.quat_xyzw[end])).mean()), float(np.cos(_yaw(traj.quat_xyzw[end])).mean()))
    heading = math.degrees(math.atan2(math.sin(end_yaw - start_yaw), math.cos(end_yaw - start_yaw)))
    start_span = float(np.max(xyz[start].max(axis=0) - xyz[start].min(axis=0)))
    end_span = float(np.max(xyz[end].max(axis=0) - xyz[end].min(axis=0)))
    return {
        "d3": float(np.linalg.norm(delta)),
        "horizontal": float(np.hypot(delta[0], delta[1])),
        "vertical": float(delta[2]),
        "heading": float(heading),
        "start_span": start_span,
        "end_span": end_span,
        "path": float(np.linalg.norm(np.diff(xyz, axis=0), axis=1).sum()),
    }


class SynthTests(unittest.TestCase):
    def test_synth_deterministic_same_seed(self):
        first = square_loop(noise_m=0.01, seed=3)
        again = square_loop(noise_m=0.01, seed=3)
        other = square_loop(noise_m=0.01, seed=4)
        np.testing.assert_array_equal(first.xyz, again.xyz)
        np.testing.assert_array_equal(first.t, again.t)
        self.assertFalse(np.array_equal(first.xyz, other.xyz))
        quiet_a = square_loop(noise_m=0.0, seed=3)
        quiet_b = square_loop(noise_m=0.0, seed=4)
        np.testing.assert_array_equal(quiet_a.xyz, quiet_b.xyz)
        hold_a = stationary(60.0, 10.0, T0, 0.02, 7)
        hold_b = stationary(60.0, 10.0, T0, 0.02, 7)
        hold_c = stationary(60.0, 10.0, T0, 0.02, 8)
        np.testing.assert_array_equal(hold_a.xyz, hold_b.xyz)
        self.assertFalse(np.array_equal(hold_a.xyz, hold_c.xyz))
        self.assertEqual(len(stationary(60.0, 10.0, T0, 0.0, 0)), 601)
        room_a = room_map(noise_m=0.005, seed=1, doubled_wall_offset_m=0.06)
        room_b = room_map(noise_m=0.005, seed=1, doubled_wall_offset_m=0.06)
        room_c = room_map(noise_m=0.005, seed=2, doubled_wall_offset_m=0.06)
        np.testing.assert_array_equal(room_a["x"], room_b["x"])
        self.assertFalse(np.array_equal(room_a["x"], room_c["x"]))

    def test_synth_loop_end_offsets_exact(self):
        cases = [
            ((0.03, 0.04, 0.0), 0.0, 0.05, 0.05, 0.0, 0.0),
            ((0.09, 0.12, 0.0), 0.0, 0.15, 0.15, 0.0, 0.0),
            ((0.0, 0.0, 0.08), 0.0, 0.08, 0.0, 0.08, 0.0),
            ((0.0, 0.0, 0.0), 10.0, 0.0, 0.0, 0.0, 10.0),
        ]
        for offset, yaw, d3, horizontal, vertical, heading in cases:
            traj = square_loop(end_offset_m=offset, end_yaw_deg=yaw, noise_m=0.0, seed=0, t0=T0)
            scored = _return_error(traj)
            self.assertLessEqual(abs(scored["d3"] - d3), 1e-9)
            self.assertLessEqual(abs(scored["horizontal"] - horizontal), 1e-9)
            self.assertLessEqual(abs(scored["vertical"] - vertical), 1e-9)
            self.assertLessEqual(abs(scored["heading"] - heading), 1e-9)
            self.assertLessEqual(scored["start_span"], 1e-12)
            self.assertLessEqual(scored["end_span"], 1e-12)
            self.assertLessEqual(abs(scored["path"] - _polyline_length(offset)), 1e-9)
        closed = square_loop(noise_m=0.0, seed=0, t0=T0)
        self.assertLessEqual(abs(_return_error(closed)["path"] - 16.0), 1e-12)
        reset = with_reset(closed, len(closed) // 2)
        self.assertLess(float(reset.t[len(closed) // 2]), float(reset.t[len(closed) // 2 - 1]))
        np.testing.assert_array_equal(reset.xyz, closed.xyz)
        gapped = with_gap(closed, T0 + 15.0, 2.0)
        gap = float(np.max(np.diff(gapped.t)))
        self.assertGreater(gap, 2.0)
        self.assertLess(gap, 2.2)
        np.testing.assert_array_equal(gapped.xyz, closed.xyz)


if __name__ == "__main__":
    unittest.main()
