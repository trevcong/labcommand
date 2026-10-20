"""Synthetic checks for replay-vs-historical comparison."""

from __future__ import annotations

import json
import math
import os
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np

from spatio.replaycompare.compare import (
    DIAGNOSTIC_LABEL, compare_sessions, _rotation_angles_deg,
    _rotation_angle_deg_from_matrix,
)
from spatio.replaycompare.pcd import read_pcd, voxel_downsample, write_pcd
from spatio.replaycompare.report import format_markdown
from spatio.replaycompare.tum import write_tum

REPO = Path(__file__).resolve().parents[3]
IDENTITY = (0.0, 0.0, 0.0, 1.0)
TINY_MAP = [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0)]


def _line_poses(quat=IDENTITY, shift=(0.0, 0.0, 0.0)):
    sx, sy, sz = shift
    samples = (
        (0.0, 0.0, 0.0, 0.0),
        (1.0, 1.0, 0.0, 0.0),
        (2.0, 1.0, 1.0, 0.0),
        (3.0, 1.0, 1.0, 0.5),
    )
    return [
        (t, x + sx, y + sy, z + sz, quat[0], quat[1], quat[2], quat[3])
        for t, x, y, z in samples
    ]


def _write_session(root, run_id, poses, points, binary=False, double=False):
    folder = Path(root) / run_id
    folder.mkdir(parents=True, exist_ok=True)
    write_tum(folder / "trajectory.tum", poses)
    write_pcd(folder / "map.pcd", points, binary=binary, double=double)
    manifest = {
        "run_id": run_id,
        "source_recording": "synthetic",
        "sensor": "synthetic",
        "fastlio_revision": "none",
        "config_file": "none",
        "config_sha256": "none",
        "image": "none",
        "created_at": "2026-10-19T00:00:00Z",
        "notes": "synthetic test",
        "physical_accuracy_evaluated": False,
    }
    (folder / "manifest.json").write_text(json.dumps(manifest), encoding="utf-8")
    return folder


class ReplayCompareTests(unittest.TestCase):
    def test_rotation_precision_at_zero_and_half_turn(self):
        # Nontrivial equal quaternions must not acquire an arccos roundoff angle.
        quat = np.array([[0.1, 0.2, 0.3, 0.4]])
        for other in (quat, -quat):
            np.testing.assert_array_equal(_rotation_angles_deg(quat, other), [0.0])
        for angle in (1e-9, 0.12, math.pi - 1e-9, math.pi):
            with self.subTest(angle=angle):
                q = np.array([[0., 0., math.sin(angle / 2), math.cos(angle / 2)]])
                want = math.degrees(angle)
                for sign in (-1, 1):
                    got = _rotation_angles_deg(np.array([IDENTITY]), sign * q)[0]
                    self.assertAlmostEqual(got / want, 1.0, places=12)
                c, s = math.cos(angle), math.sin(angle)
                matrix = np.array([[c, -s, 0.], [s, c, 0.], [0., 0., 1.]])
                got = _rotation_angle_deg_from_matrix(matrix)
                self.assertAlmostEqual(got / want, 1.0, places=12)
        self.assertEqual(_rotation_angle_deg_from_matrix(np.eye(3)), 0.0)

    def test_identical_sessions_zero_difference(self):
        poses = _line_poses()
        with tempfile.TemporaryDirectory() as tmp:
            folder_a = _write_session(tmp, "run-a", list(reversed(poses)), TINY_MAP)
            folder_b = _write_session(tmp, "run-b", poses, TINY_MAP)
            result, _series = compare_sessions(folder_a, folder_b)

        self.assertEqual(result["status"], "COMPARED")
        self.assertEqual(result["a"]["run_id"], "run-a")
        association = result["trajectory"]["association"]
        self.assertEqual(association["matched"], 4)
        self.assertEqual(association["unmatched_a"], 0)
        self.assertEqual(association["unmatched_b"], 0)
        self.assertAlmostEqual(association["overlap_duration_s"], 3.0, places=9)
        self.assertAlmostEqual(association["a_rate_hz"], 1.0, places=9)
        self.assertAlmostEqual(association["b_rate_hz"], 1.0, places=9)

        position = result["trajectory"]["raw"]["position_m"]
        rotation = result["trajectory"]["raw"]["rotation_deg"]
        for name in ("mean", "median", "p95", "max", "final"):
            self.assertAlmostEqual(position[name], 0.0, places=12)
        for name in ("median", "p95", "max"):
            self.assertAlmostEqual(rotation[name], 0.0, places=12)

        raw = result["trajectory"]["raw"]
        self.assertAlmostEqual(raw["start_to_end_m"]["a"], 1.5, places=12)
        self.assertAlmostEqual(raw["start_to_end_m"]["b"], 1.5, places=12)
        self.assertAlmostEqual(raw["path_length_m"]["a"], 2.5, places=12)
        self.assertAlmostEqual(raw["path_length_m"]["b"], 2.5, places=12)
        self.assertAlmostEqual(raw["first_pose"]["a"]["t"], 0.0, places=12)
        self.assertAlmostEqual(raw["first_pose"]["a"]["x"], 0.0, places=12)
        self.assertAlmostEqual(raw["first_pose"]["a"]["qw"], 1.0, places=12)

        diagnostic = result["trajectory"]["diagnostic_aligned"]
        self.assertTrue(diagnostic["computed"])
        self.assertEqual(diagnostic["label"], DIAGNOSTIC_LABEL)
        self.assertAlmostEqual(diagnostic["ate_rmse_m"], 0.0, places=12)
        self.assertEqual(result["time_offset_s"], None)

    def test_translation_offset_in_raw_numbers_and_removed_only_in_diagnostic(self):
        with tempfile.TemporaryDirectory() as tmp:
            folder_a = _write_session(tmp, "run-a", _line_poses(), TINY_MAP)
            folder_b = _write_session(
                tmp, "run-b", _line_poses(shift=(0.05, 0.0, 0.0)), TINY_MAP
            )
            result, _series = compare_sessions(folder_a, folder_b)

        position = result["trajectory"]["raw"]["position_m"]
        for name in ("mean", "median", "p95", "max", "final"):
            self.assertAlmostEqual(position[name], 0.05, places=9)
        diagnostic = result["trajectory"]["diagnostic_aligned"]
        self.assertAlmostEqual(diagnostic["ate_rmse_m"], 0.0, places=8)
        self.assertAlmostEqual(diagnostic["translation_m"][0], 0.05, places=8)
        self.assertAlmostEqual(diagnostic["translation_m"][1], 0.0, places=8)
        self.assertAlmostEqual(diagnostic["translation_m"][2], 0.0, places=8)
        self.assertAlmostEqual(diagnostic["rotation_deg"], 0.0, places=6)
        self.assertGreater(position["mean"], diagnostic["ate_rmse_m"])

    def test_rotation_offset_reported_in_degrees(self):
        half = math.radians(12.0) / 2.0
        quat = (0.0, 0.0, math.sin(half), math.cos(half))
        flipped = tuple(-value for value in quat)
        poses_b = _line_poses(quat=quat)
        last = list(poses_b[-1])
        last[4:8] = flipped
        poses_b[-1] = tuple(last)
        with tempfile.TemporaryDirectory() as tmp:
            folder_a = _write_session(tmp, "run-a", _line_poses(), TINY_MAP)
            folder_b = _write_session(tmp, "run-b", poses_b, TINY_MAP)
            result, _series = compare_sessions(folder_a, folder_b)

        rotation = result["trajectory"]["raw"]["rotation_deg"]
        for name in ("median", "p95", "max"):
            self.assertAlmostEqual(rotation[name], 12.0, places=6)
        self.assertAlmostEqual(result["trajectory"]["raw"]["position_m"]["mean"], 0.0, places=9)

    def test_association_respects_max_dt_and_is_one_to_one(self):
        poses_a = [
            (0.0, 0.0, 0.0, 0.0, *IDENTITY),
            (0.04, 10.0, 0.0, 0.0, *IDENTITY),
            (1.0, 100.0, 0.0, 0.0, *IDENTITY),
        ]
        poses_b = [
            (0.03, 10.0, 0.0, 0.0, *IDENTITY),
            (1.20, 100.0, 0.0, 0.0, *IDENTITY),
        ]
        with tempfile.TemporaryDirectory() as tmp:
            folder_a = _write_session(tmp, "run-a", poses_a, TINY_MAP)
            folder_b = _write_session(tmp, "run-b", poses_b, TINY_MAP)
            result, _series = compare_sessions(folder_a, folder_b, max_dt=0.05)

        association = result["trajectory"]["association"]
        self.assertEqual(association["matched"], 1)
        self.assertEqual(association["unmatched_a"], 2)
        self.assertEqual(association["unmatched_b"], 1)
        self.assertAlmostEqual(result["trajectory"]["raw"]["position_m"]["mean"], 0.0, places=9)

    def test_different_rates_associate_correctly(self):
        poses_a = [(i / 10.0, i / 10.0, 0.0, 0.0, *IDENTITY) for i in range(11)]
        poses_b = [(t, t, 0.0, 0.0, *IDENTITY) for t in (0.0, 0.5, 1.0)]
        with tempfile.TemporaryDirectory() as tmp:
            folder_a = _write_session(tmp, "run-a", poses_a, TINY_MAP)
            folder_b = _write_session(tmp, "run-b", poses_b, TINY_MAP)
            result, _series = compare_sessions(folder_a, folder_b, max_dt=0.05)

        association = result["trajectory"]["association"]
        self.assertEqual(association["matched"], 3)
        self.assertEqual(association["unmatched_a"], 8)
        self.assertEqual(association["unmatched_b"], 0)
        self.assertAlmostEqual(association["a_rate_hz"], 10.0, places=9)
        self.assertAlmostEqual(association["b_rate_hz"], 2.0, places=9)
        self.assertAlmostEqual(result["trajectory"]["raw"]["position_m"]["mean"], 0.0, places=9)

    def test_partial_overlap_reported(self):
        poses_a = [(float(t), 0.0, 0.0, 0.0, *IDENTITY) for t in range(4)]
        poses_b = [
            (2.0, 0.25, 0.0, 0.0, *IDENTITY),
            (3.0, 0.75, 0.0, 0.0, *IDENTITY),
            (4.0, 0.0, 0.0, 0.0, *IDENTITY),
            (5.0, 0.0, 0.0, 0.0, *IDENTITY),
        ]
        with tempfile.TemporaryDirectory() as tmp:
            folder_a = _write_session(tmp, "run-a", poses_a, TINY_MAP)
            folder_b = _write_session(tmp, "run-b", poses_b, TINY_MAP)
            result, _series = compare_sessions(folder_a, folder_b, max_dt=0.05)

        association = result["trajectory"]["association"]
        self.assertAlmostEqual(association["overlap_duration_s"], 1.0, places=9)
        self.assertEqual(association["matched"], 2)
        self.assertEqual(association["unmatched_a"], 2)
        self.assertEqual(association["unmatched_b"], 2)
        position = result["trajectory"]["raw"]["position_m"]
        self.assertAlmostEqual(position["mean"], 0.5, places=9)
        self.assertAlmostEqual(position["median"], 0.5, places=9)
        self.assertAlmostEqual(position["max"], 0.75, places=9)
        self.assertAlmostEqual(position["final"], 0.75, places=9)
        self.assertAlmostEqual(position["p95"], 0.725, places=9)

    def test_no_time_overlap_not_evaluable_and_no_auto_shift(self):
        poses_a = [(float(t), float(t), 0.0, 0.0, *IDENTITY) for t in (0, 1, 2)]
        poses_b = [(1000.0 + t, 5.0, 0.0, 0.0, *IDENTITY) for t in (0, 1, 2)]
        with tempfile.TemporaryDirectory() as tmp:
            folder_a = _write_session(tmp, "run-a", poses_a, TINY_MAP)
            folder_b = _write_session(tmp, "run-b", poses_b, TINY_MAP)
            result, _series = compare_sessions(folder_a, folder_b)

        self.assertEqual(result["status"], "NOT_EVALUABLE")
        self.assertIsNone(result["time_offset_s"])
        self.assertIsNone(result["time_offset_applied_to"])
        self.assertIsNone(result["trajectory"]["raw"]["position_m"])
        self.assertIsNone(result["trajectory"]["raw"]["rotation_deg"])
        diagnostic = result["trajectory"]["diagnostic_aligned"]
        self.assertFalse(diagnostic["computed"])
        self.assertNotIn("ate_rmse_m", diagnostic)
        self.assertEqual(
            diagnostic["reason"],
            "not computed because trajectory time ranges do not overlap",
        )
        association = result["trajectory"]["association"]
        self.assertEqual(association["matched"], 0)
        self.assertEqual(association["a_range_s"], [0.0, 2.0])
        self.assertEqual(association["b_range_s"], [1000.0, 1002.0])
        self.assertEqual(association["b_range_compared_s"], [1000.0, 1002.0])
        self.assertIn("no automatic time shift was applied", " ".join(result["reasons"]))
        text = format_markdown(result)
        self.assertIn("NOT_EVALUABLE", text)
        self.assertIn("1000.000000", text)
        self.assertNotIn("ATE RMSE", text)

    def test_overlapping_spans_with_zero_matches_not_evaluable(self):
        """Spans can overlap while every sample is still outside max_dt.

        The disjoint-range test never reaches pairing. These stamps do, and
        pairing returns nothing. That must not be reported as COMPARED.
        """
        poses_a = [(float(t), float(t), 0.0, 0.0, *IDENTITY) for t in (0, 1, 2)]
        poses_b = [(t, 1.0, 0.0, 0.0, *IDENTITY) for t in (0.5, 1.5, 2.5)]
        with tempfile.TemporaryDirectory() as tmp:
            folder_a = _write_session(tmp, "run-a", poses_a, TINY_MAP)
            folder_b = _write_session(tmp, "run-b", poses_b, TINY_MAP)
            result, _series = compare_sessions(folder_a, folder_b)

        self.assertEqual(result["status"], "NOT_EVALUABLE")
        association = result["trajectory"]["association"]
        self.assertEqual(association["matched"], 0)
        self.assertEqual(association["unmatched_a"], 3)
        self.assertEqual(association["unmatched_b"], 3)
        self.assertEqual(association["range_gap_s"], 0.0)
        self.assertAlmostEqual(association["overlap_duration_s"], 1.5, places=9)
        self.assertEqual(association["max_dt_s"], 0.05)
        self.assertIsNone(result["trajectory"]["raw"]["position_m"])
        self.assertIsNone(result["trajectory"]["raw"]["rotation_deg"])
        diagnostic = result["trajectory"]["diagnostic_aligned"]
        self.assertFalse(diagnostic["computed"])
        self.assertNotIn("ate_rmse_m", diagnostic)
        self.assertEqual(diagnostic["reason"], "no matched poses to align")
        self.assertIn("no pose pairs within max_dt", result["reasons"])
        self.assertNotIn("trajectory time ranges do not overlap", result["reasons"])
        self.assertIsNotNone(result["map"])
        text = format_markdown(result)
        self.assertIn("NOT_EVALUABLE", text)
        self.assertIn("no matched poses to align", text)
        self.assertNotIn("time ranges do not overlap", text)
        self.assertNotIn("ATE RMSE", text)

    def test_explicit_time_offset_is_reported(self):
        poses_a = [(float(t), float(t), 0.0, 0.0, *IDENTITY) for t in (0, 1, 2)]
        poses_b = [(1000.0 + t, float(t), 0.0, 0.0, *IDENTITY) for t in (0, 1, 2)]
        with tempfile.TemporaryDirectory() as tmp:
            folder_a = _write_session(tmp, "run-a", poses_a, TINY_MAP)
            folder_b = _write_session(tmp, "run-b", poses_b, TINY_MAP)
            untouched, _series = compare_sessions(folder_a, folder_b)
            shifted, _series = compare_sessions(folder_a, folder_b, time_offset=-1000.0)

        self.assertEqual(untouched["status"], "NOT_EVALUABLE")
        self.assertIsNone(untouched["time_offset_s"])
        self.assertEqual(shifted["status"], "COMPARED")
        self.assertEqual(shifted["time_offset_s"], -1000.0)
        self.assertEqual(shifted["time_offset_applied_to"], "b")
        self.assertEqual(shifted["trajectory"]["association"]["matched"], 3)
        self.assertEqual(shifted["trajectory"]["association"]["b_range_s"], [1000.0, 1002.0])
        compared = shifted["trajectory"]["association"]["b_range_compared_s"]
        self.assertAlmostEqual(compared[0], 0.0, places=9)
        self.assertAlmostEqual(compared[1], 2.0, places=9)
        self.assertAlmostEqual(shifted["trajectory"]["raw"]["position_m"]["mean"], 0.0, places=9)
        self.assertIn("explicit", " ".join(shifted["reasons"]))
        text = format_markdown(shifted)
        self.assertIn("explicit", text)
        self.assertIn("-1000", text)

    def test_map_identical_zero_distance(self):
        points = [
            (0.0, 0.0, 0.0),
            (0.01, 0.0, 0.0),
            (1.0, 0.0, 0.0),
            (0.0, 1.0, 0.0),
        ]
        poses = [(0.0, 0.0, 0.0, 0.0, *IDENTITY), (1.0, 0.0, 0.0, 0.0, *IDENTITY)]
        with tempfile.TemporaryDirectory() as tmp:
            folder_a = _write_session(tmp, "run-a", poses, points)
            folder_b = _write_session(tmp, "run-b", poses, points)
            result, _series = compare_sessions(folder_a, folder_b, voxel=0.05)

        cloud = result["map"]
        self.assertEqual(cloud["a_points_before"], 4)
        self.assertEqual(cloud["b_points_before"], 4)
        self.assertEqual(cloud["a_points_after"], cloud["b_points_after"])
        self.assertLess(cloud["a_points_after"], cloud["a_points_before"])
        for side in ("a_to_b", "b_to_a"):
            stats = cloud[side]
            for name in ("p50_m", "p95_m", "p99_m"):
                self.assertEqual(stats[name], 0.0)
            self.assertEqual(stats["fraction_within_0_05m"], 1.0)
            self.assertEqual(stats["fraction_within_0_10m"], 1.0)
        self.assertEqual(cloud["centroid_offset_norm_m"], 0.0)

    def test_map_shifted_5cm_distances(self):
        points_a = [(0.0, float(y), float(z)) for z in (0.0, 1.0) for y in range(5)]
        points_b = [(0.05, float(y), float(z)) for z in (0.0, 1.0) for y in range(5)]
        poses = [(0.0, 0.0, 0.0, 0.0, *IDENTITY), (1.0, 0.0, 0.0, 0.0, *IDENTITY)]
        with tempfile.TemporaryDirectory() as tmp:
            folder_a = _write_session(tmp, "run-a", poses, points_a, binary=True, double=True)
            folder_b = _write_session(tmp, "run-b", poses, points_b, binary=True, double=True)
            result, _series = compare_sessions(folder_a, folder_b, voxel=0.05)

        cloud = result["map"]
        self.assertEqual(cloud["a_points_before"], 10)
        self.assertEqual(cloud["a_points_after"], 10)
        self.assertEqual(cloud["b_points_after"], 10)
        for side in ("a_to_b", "b_to_a"):
            stats = cloud[side]
            for name in ("p50_m", "p95_m", "p99_m"):
                self.assertAlmostEqual(stats[name], 0.05, places=12)
            self.assertEqual(stats["fraction_within_0_05m"], 1.0)
            self.assertEqual(stats["fraction_within_0_10m"], 1.0)
        offset = cloud["centroid_offset_m"]
        self.assertAlmostEqual(offset[0], 0.05, places=12)
        self.assertAlmostEqual(offset[1], 0.0, places=12)
        self.assertAlmostEqual(offset[2], 0.0, places=12)

    def test_map_missing_region_gives_asymmetric_distances(self):
        grid = [(float(x), float(y), 0.0) for x in range(5) for y in range(5)]
        far = [(100.0, 0.0, 0.0), (100.0, 1.0, 0.0), (100.0, 2.0, 0.0)]
        poses = [(0.0, 0.0, 0.0, 0.0, *IDENTITY), (1.0, 0.0, 0.0, 0.0, *IDENTITY)]
        with tempfile.TemporaryDirectory() as tmp:
            folder_a = _write_session(tmp, "run-a", poses, grid + far)
            folder_b = _write_session(tmp, "run-b", poses, grid)
            result, _series = compare_sessions(folder_a, folder_b, voxel=0.05)

        cloud = result["map"]
        self.assertGreater(cloud["a_to_b"]["p95_m"], 10.0)
        self.assertEqual(cloud["b_to_a"]["p95_m"], 0.0)
        self.assertGreater(cloud["a_to_b"]["p95_m"], cloud["b_to_a"]["p95_m"])
        self.assertLess(cloud["a_to_b"]["fraction_within_0_10m"], 1.0)
        self.assertEqual(cloud["b_to_a"]["fraction_within_0_05m"], 1.0)

    def test_voxel_downsample_deterministic(self):
        points = np.array(
            [
                [0.0, 0.0, 0.0],
                [0.5, 0.25, 0.25],
                [0.25, 0.5, 0.5],
                [2.0, 0.0, 0.0],
                [2.5, 0.0, 0.0],
            ],
            dtype=np.float64,
        )
        expected = np.array(
            [
                [0.25, 0.25, 0.25],
                [2.25, 0.0, 0.0],
            ],
            dtype=np.float64,
        )
        once = voxel_downsample(points, 1.0)
        twice = voxel_downsample(points, 1.0)
        order = np.random.RandomState(0).permutation(len(points))
        shuffled = voxel_downsample(points[order], 1.0)
        self.assertTrue(np.array_equal(once, expected))
        self.assertTrue(np.array_equal(once, twice))
        self.assertTrue(np.array_equal(once, shuffled))

    def test_markdown_headline_is_raw_not_aligned(self):
        with tempfile.TemporaryDirectory() as tmp:
            folder_a = _write_session(tmp, "run-a", _line_poses(), TINY_MAP)
            folder_b = _write_session(
                tmp, "run-b", _line_poses(shift=(0.05, 0.0, 0.0)), TINY_MAP
            )
            result, _series = compare_sessions(folder_a, folder_b)

        text = format_markdown(result)
        marker = "diagnostic only (after SE(3) alignment)"
        self.assertIn(marker, text)
        head, _found, tail = text.partition(marker)
        self.assertLess(text.index("## Headline"), text.index(marker))
        self.assertIn("Position difference (m): mean 0.050000", head)
        self.assertNotIn("ATE", head)
        self.assertNotIn("SE(3)", head)
        self.assertNotIn("Umeyama", head)
        self.assertNotIn("umeyama", head)
        ate = result["trajectory"]["diagnostic_aligned"]["ate_rmse_m"]
        self.assertIn(f"ATE RMSE (m): {ate:.6f}", tail)
        self.assertNotIn("ATE RMSE", head)
        self.assertGreater(result["trajectory"]["raw"]["position_m"]["mean"], 0.04)
        self.assertLess(ate, 1e-8)

    def test_binary_and_ascii_pcd_readers_agree(self):
        points = np.array(
            [
                [0.5, -2.0, 3.25],
                [1.0, 0.0, -0.5],
                [0.0, 0.25, 8.0],
            ],
            dtype=np.float64,
        )
        expected = points.astype(np.float32).astype(np.float64)
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            ascii_path = root / "cloud_ascii.pcd"
            binary_path = root / "cloud_binary.pcd"
            extra_path = root / "cloud_intensity.pcd"
            write_pcd(ascii_path, points, binary=False)
            write_pcd(binary_path, points, binary=True)
            self._write_intensity_binary(extra_path, points.astype(np.float32))
            ascii_cloud = read_pcd(ascii_path)
            binary_cloud = read_pcd(binary_path)
            extra_cloud = read_pcd(extra_path)
            offset = extra_path.read_bytes().index(b"DATA binary\n") + len(b"DATA binary\n")

        self.assertTrue(np.array_equal(ascii_cloud, expected))
        self.assertTrue(np.array_equal(binary_cloud, expected))
        self.assertTrue(np.array_equal(ascii_cloud, binary_cloud))
        self.assertTrue(np.array_equal(extra_cloud, expected))
        self.assertNotEqual(offset % 4, 0)

    def test_cli_outputs_written(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            folder_a = _write_session(root, "run-a", _line_poses(), TINY_MAP)
            folder_b = _write_session(
                root, "run-b", _line_poses(shift=(0.05, 0.0, 0.0)), TINY_MAP
            )
            out = root / "out"
            proc = subprocess.run(
                [
                    sys.executable,
                    "-m",
                    "spatio.replaycompare",
                    str(folder_a),
                    str(folder_b),
                    "--out",
                    str(out),
                ],
                cwd=str(REPO),
                env=dict(os.environ, MPLBACKEND="Agg"),
                capture_output=True,
                text=True,
                timeout=60,
                check=False,
            )
            self.assertEqual(proc.returncode, 0, proc.stderr)
            payload = json.loads((out / "compare.json").read_text(encoding="utf-8"))
            markdown = (out / "compare.md").read_text(encoding="utf-8")
            png = (out / "overlay.png").read_bytes()

        self.assertEqual(payload["schema"], "spatio.replaycompare/1")
        self.assertEqual(payload["status"], "COMPARED")
        for key in ("a", "b", "reasons", "trajectory", "map"):
            self.assertIn(key, payload)
        self.assertIn("raw", payload["trajectory"])
        self.assertIn("diagnostic_aligned", payload["trajectory"])
        self.assertIn("association", payload["trajectory"])
        self.assertIn("## Headline", markdown)
        self.assertIn(DIAGNOSTIC_LABEL, markdown)
        self.assertTrue(png.startswith(b"\x89PNG\r\n\x1a\n"))
        self.assertGreater(len(png), 1000)

    def _write_intensity_binary(self, path, xyz):
        intensity = np.array([1.0, 2.0, 4.0], dtype=np.float32)
        packed = np.concatenate(
            [np.ascontiguousarray(xyz, dtype=np.float32), intensity[:, None]],
            axis=1,
        )
        payload = np.ascontiguousarray(packed, dtype="<f4").tobytes()
        count = len(xyz)
        # The comment length is chosen so the binary payload is not 4-byte aligned.
        comment = "# c\n"
        header = (
            f"{comment}"
            "VERSION 0.7\n"
            "FIELDS x y z intensity\n"
            "SIZE 4 4 4 4\n"
            "TYPE F F F F\n"
            "COUNT 1 1 1 1\n"
            f"WIDTH {count}\n"
            "HEIGHT 1\n"
            "VIEWPOINT 0 0 0 1 0 0 0\n"
            f"POINTS {count}\n"
            "DATA binary\n"
        ).encode("ascii")
        path.write_bytes(header + payload)


if __name__ == "__main__":
    unittest.main()
