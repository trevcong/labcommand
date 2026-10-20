"""Synthetic tests for per-patch wall measurement.

Every cloud is generated here with a fixed seed. Nothing is skipped.
"""

from __future__ import annotations

import json
import os
import subprocess
import sys
import tempfile
import unittest

import numpy as np

from spatio.wallpatch.doubled import _histogram, _smooth, analyze_distances, reconcile_axes
from spatio.wallpatch.errors import MapShaMismatch, WallpatchError
from spatio.wallpatch.geometry import box_axes, crop_points, fit_plane, parse_box
from spatio.wallpatch.measure import _guard_band_reason, _measure_one, measure_map
from spatio.wallpatch.numbers import parse_real, parse_suggest_n, parse_voxel_m
from spatio.wallpatch.pcd import (
    DEFAULT_CHUNK,
    read_xyz,
    sha256_file,
    write_ascii_pcd,
    write_binary_pcd,
)
from spatio.wallpatch.suggest import suggest_map

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
# Phi^{-1}(0.975). p95 of |N(0, sigma)| equals this times sigma.
_GAUSS_P95 = 1.959963984540054
_REVISIT = (
    "map.pcd has no per-point time or visit id. "
    "separation_m is the distance between two parallel surfaces found inside the patch. "
    "It cannot be attributed to a specific visit or revisit."
)
_GATE_LABEL = "provisional (approved for now, revisit after real data)"


def _box(name, center, hint, half):
    return {
        "name": name,
        "box_map_frame": {
            "center_m": [float(v) for v in center],
            "normal_hint": [float(v) for v in hint],
            "half_extent_m": [float(v) for v in half],
        },
    }


def _write_patches(path, map_path, patches, sha=None):
    if sha is None:
        sha = sha256_file(map_path)
    with open(path, "w", encoding="utf-8") as handle:
        json.dump(
            {
                "schema": "spatio.wall_patches/1",
                "map_sha256": sha,
                "patches": patches,
            },
            handle,
        )


def _measure(directory, points, patches):
    map_path = os.path.join(directory, "map.pcd")
    patches_path = os.path.join(directory, "patches.json")
    out_dir = os.path.join(directory, "out")
    write_binary_pcd(map_path, points)
    _write_patches(patches_path, map_path, patches)
    report = measure_map(map_path, patches_path, out_dir)
    return report, out_dir


class WallpatchTests(unittest.TestCase):
    def test_single_plane_p95_matches_noise(self):
        sigma = 0.01
        n_points = 8000
        rng = np.random.default_rng(12345)
        points = np.column_stack(
            [
                rng.uniform(-0.5, 0.5, n_points),
                rng.normal(0.0, sigma, n_points),
                rng.uniform(-0.5, 0.5, n_points),
            ]
        )
        with tempfile.TemporaryDirectory() as directory:
            report, _out = _measure(
                directory,
                points,
                [_box("plane", (0, 0, 0), (0, 1, 0), (0.6, 0.5, 0.6))],
            )
        patch = report["patches"][0]
        self.assertEqual(patch["status"], "MEASURED")
        self.assertGreaterEqual(patch["n_measured"], 5000)
        self.assertEqual(patch["n_measured"], n_points)
        expected = _GAUSS_P95 * sigma
        p95 = patch["residuals"]["p95_abs_m"]
        self.assertGreaterEqual(p95, expected * 0.85)
        self.assertLessEqual(p95, expected * 1.15)

    def test_single_plane_not_flagged_doubled(self):
        rng = np.random.default_rng(99)
        n_points = 6000
        points = np.column_stack(
            [
                rng.uniform(-0.5, 0.5, n_points),
                rng.normal(0.0, 0.01, n_points),
                rng.uniform(-0.5, 0.5, n_points),
            ]
        )
        with tempfile.TemporaryDirectory() as directory:
            report, _out = _measure(
                directory,
                points,
                [_box("plane", (0, 0, 0), (0, 1, 0), (0.6, 0.5, 0.6))],
            )
        patch = report["patches"][0]
        self.assertEqual(patch["doubled"], "false")
        self.assertIsNone(patch["separation_m"])

    def test_doubled_wall_6cm_detected_separation_within_5mm(self):
        points = _two_sheets(separation=0.06, sigma=0.005, n_each=4000, seed=7)
        with tempfile.TemporaryDirectory() as directory:
            report, out_dir = _measure(
                directory,
                points,
                [_box("double", (0, 0, 0), (0, 1, 0), (0.6, 0.2, 0.6))],
            )
            with open(os.path.join(out_dir, "wallpatch.md"), encoding="utf-8") as handle:
                text = handle.read()
        patch = report["patches"][0]
        self.assertEqual(patch["status"], "MEASURED")
        self.assertEqual(patch["doubled"], "true")
        self.assertAlmostEqual(patch["separation_m"], 0.06, delta=0.005)
        self.assertFalse(patch["gate_separation_pass"])
        self.assertIn("40 mm", report["doubled_wall_rule"]["text"])
        self.assertIn("at least 20%", report["doubled_wall_rule"]["text"])
        self.assertIn(report["doubled_wall_rule"]["text"], text)

    def test_doubled_wall_3cm_detected_or_undetermined_never_false_single(self):
        # Observed behaviour of this rule at σ = 5 mm: undetermined.
        # Confident detection starts at 40 mm, so 30 mm is inside the
        # unresolved band. It must not be called a single wall.
        points = _two_sheets(separation=0.03, sigma=0.005, n_each=4000, seed=8)
        with tempfile.TemporaryDirectory() as directory:
            report, _out = _measure(
                directory,
                points,
                [_box("close", (0, 0, 0), (0, 1, 0), (0.6, 0.2, 0.6))],
            )
        patch = report["patches"][0]
        self.assertIn(patch["doubled"], ("true", "undetermined"))
        self.assertNotEqual(patch["doubled"], "false")
        self.assertEqual(patch["doubled"], "undetermined")
        self.assertIsNotNone(patch["separation_m"])
        self.assertAlmostEqual(patch["separation_m"], 0.03, delta=0.005)
        self.assertIsNone(patch["gate_separation_pass"])

    def test_tilted_wall_normal_fitted(self):
        rng = np.random.default_rng(11)
        n_points = 6000
        theta = np.deg2rad(10.0)
        true_normal = np.array([np.sin(theta), np.cos(theta), 0.0])
        x = rng.uniform(-0.40, 0.40, n_points)
        z = rng.uniform(-0.40, 0.40, n_points)
        y = -x * np.tan(theta) + rng.normal(0.0, 0.004, n_points)
        points = np.column_stack([x, y, z])
        with tempfile.TemporaryDirectory() as directory:
            report, _out = _measure(
                directory,
                points,
                [_box("tilt", (0, 0, 0), (0, 1, 0), (0.5, 0.2, 0.5))],
            )
        patch = report["patches"][0]
        self.assertEqual(patch["status"], "MEASURED")
        self.assertEqual(patch["n_measured"], n_points)
        fitted = np.array(patch["plane"]["normal"], dtype=np.float64)
        fitted = fitted / np.linalg.norm(fitted)
        cosine = float(np.clip(abs(np.dot(fitted, true_normal)), -1.0, 1.0))
        angle_to_truth = float(np.degrees(np.arccos(cosine)))
        self.assertLess(angle_to_truth, 1.5)
        self.assertAlmostEqual(patch["plane"]["angle_from_hint_deg"], 10.0, delta=1.5)

    def test_outliers_do_not_dominate_fit(self):
        rng = np.random.default_rng(12)
        n_in = 4750
        n_out = 250  # 5% of 5000
        self.assertEqual(n_out, int(round(0.05 * (n_in + n_out))))
        points = np.vstack(
            [
                np.column_stack(
                    [
                        rng.uniform(-0.5, 0.5, n_in),
                        rng.normal(0.0, 0.008, n_in),
                        rng.uniform(-0.5, 0.5, n_in),
                    ]
                ),
                np.column_stack(
                    [
                        rng.uniform(-0.5, 0.5, n_out),
                        rng.uniform(0.30, 0.45, n_out),
                        rng.uniform(-0.5, 0.5, n_out),
                    ]
                ),
            ]
        )
        with tempfile.TemporaryDirectory() as directory:
            report, _out = _measure(
                directory,
                points,
                [_box("outliers", (0, 0, 0), (0, 1, 0), (0.6, 0.6, 0.6))],
            )
        patch = report["patches"][0]
        # Round 7 (Trevor's provisional guard band): p95 here is 4.24 cm, above
        # the 4 cm pass limit and within the 5 cm gate, so the patch is not
        # checkable. It used to be MEASURED with both gates passing. The fit
        # itself is still checked below on the same points.
        self.assertEqual(patch["status"], "NOT_EVALUABLE")
        self.assertEqual(patch["n_cropped"], n_in + n_out)
        self.assertIn("p95 0.042", patch["reasons"][0])
        self.assertIn("provisional 0.040 m pass limit", patch["reasons"][0])
        fitted = fit_plane(points, np.array([0.0, 1.0, 0.0]))
        self.assertLess(fitted["angle_from_hint_deg"], 2.0)
        offset = abs(float(np.dot(fitted["point"], fitted["normal"])))
        self.assertLess(offset, 0.01)

    def test_box_crop_exact_counts(self):
        # Axis-aligned box. Membership is decided from x, y, z directly.
        center = np.array([0.0, 0.0, 0.0])
        hint = np.array([0.0, 1.0, 0.0])
        half = np.array([0.5, 0.25, 0.5])
        inside = np.array(
            [
                [0.0, 0.0, 0.0],
                [0.5, 0.25, 0.5],  # closed corner
                [0.0, -0.25, 0.0],  # closed face
                [-0.2, 0.1, 0.3],
            ],
            dtype=np.float64,
        )
        outside = np.array(
            [
                [0.51, 0.0, 0.0],
                [0.0, 0.26, 0.0],
                [0.0, 0.0, -0.51],
                [2.0, 2.0, 2.0],
            ],
            dtype=np.float64,
        )
        cloud = np.vstack([inside, outside])
        cropped = crop_points(cloud, center, hint, half)
        self.assertEqual(len(cropped), len(inside))

        # Rotated 45° around Z. A point that sits inside the axis-aligned
        # bounds of the box can still fall outside the oriented box.
        rot_hint = np.array([1.0, 1.0, 0.0]) / np.sqrt(2.0)
        rot_half = np.array([0.40, 0.15, 0.30])
        rot_center = np.array([1.0, 2.0, 3.0])
        along, across, vertical = _independent_axes(rot_hint)
        local_inside = np.array(
            [
                [0.0, 0.0, 0.0],
                [0.40, 0.15, 0.30],
                [-0.20, -0.10, 0.05],
            ]
        )
        world_inside = (
            rot_center
            + local_inside[:, 0:1] * along
            + local_inside[:, 1:2] * across
            + local_inside[:, 2:3] * vertical
        )
        # 0.20 along the hint is past the 0.15 across half-extent.
        past_across = rot_center + 0.20 * rot_hint
        # Inside a loose axis-aligned neighbourhood of the centre, but the
        # across coordinate is 0.5/sqrt(2) ≈ 0.35 > 0.15.
        disguised = rot_center + np.array([0.50, 0.0, 0.0])
        rotated = np.vstack([world_inside, past_across[None, :], disguised[None, :]])
        cropped_rot = crop_points(rotated, rot_center, rot_hint, rot_half)
        self.assertEqual(len(cropped_rot), len(world_inside))

        with tempfile.TemporaryDirectory() as directory:
            binary_path = os.path.join(directory, "cloud.pcd")
            ascii_path = os.path.join(directory, "cloud_ascii.pcd")
            write_binary_pcd(binary_path, cloud)
            write_ascii_pcd(ascii_path, cloud)
            self.assertEqual(len(read_xyz(binary_path)), len(cloud))
            self.assertEqual(len(read_xyz(ascii_path)), len(cloud))
            binary_crop = crop_points(read_xyz(binary_path), center, hint, half)
            ascii_crop = crop_points(read_xyz(ascii_path), center, hint, half)
        self.assertEqual(len(binary_crop), len(inside))
        self.assertEqual(len(ascii_crop), len(inside))

    def test_no_downsampling_before_measure(self):
        # 1 mm grid, 40 by 50. A 1 cm voxel step would keep about 100 points.
        xs = np.linspace(-0.05, 0.05, 50)
        zs = np.linspace(-0.05, 0.05, 40)
        xx, zz = np.meshgrid(xs, zs)
        sign = np.where((np.indices(xx.shape).sum(axis=0) % 2) == 0, 1.0, -1.0)
        yy = 0.01 * sign
        points = np.column_stack([xx.ravel(), yy.ravel(), zz.ravel()])
        self.assertEqual(len(points), 2000)
        with tempfile.TemporaryDirectory() as directory:
            report, _out = _measure(
                directory,
                points,
                [_box("dense", (0, 0, 0), (0, 1, 0), (0.2, 0.2, 0.2))],
            )
        patch = report["patches"][0]
        self.assertEqual(patch["status"], "MEASURED")
        self.assertEqual(patch["n_cropped"], 2000)
        self.assertEqual(patch["n_measured"], 2000)
        self.assertFalse(patch["downsampled_before_measure"])
        self.assertFalse(report["downsampled_before_measure"])
        self.assertAlmostEqual(patch["residuals"]["p95_abs_m"], 0.01, delta=1e-6)

    def test_empty_or_sparse_patch_not_evaluable(self):
        rng = np.random.default_rng(3)
        sparse = rng.normal(size=(100, 3)) * 0.05
        # A second cloud of points that miss the empty box entirely.
        elsewhere = np.column_stack(
            [
                np.full(50, 10.0),
                np.zeros(50),
                np.zeros(50),
            ]
        )
        points = np.vstack([sparse, elsewhere])
        with tempfile.TemporaryDirectory() as directory:
            report, _out = _measure(
                directory,
                points,
                [
                    _box("sparse", (0, 0, 0), (0, 1, 0), (0.5, 0.5, 0.5)),
                    _box("empty", (5, 5, 5), (1, 0, 0), (0.2, 0.2, 0.2)),
                ],
            )
        sparse_patch = report["patches"][0]
        empty_patch = report["patches"][1]
        self.assertEqual(sparse_patch["status"], "NOT_EVALUABLE")
        self.assertTrue(sparse_patch["reasons"])
        self.assertIn("200", sparse_patch["reasons"][0])
        self.assertEqual(sparse_patch["n_cropped"], 100)
        self.assertEqual(empty_patch["status"], "NOT_EVALUABLE")
        self.assertTrue(empty_patch["reasons"])
        self.assertIn("empty", empty_patch["reasons"][0])
        self.assertEqual(empty_patch["n_cropped"], 0)

    def test_map_sha_mismatch_rejected(self):
        points = np.zeros((10, 3), dtype=np.float64)
        with tempfile.TemporaryDirectory() as directory:
            map_path = os.path.join(directory, "map.pcd")
            patches_path = os.path.join(directory, "patches.json")
            out_dir = os.path.join(directory, "out")
            write_binary_pcd(map_path, points)
            _write_patches(
                patches_path,
                map_path,
                [_box("a", (0, 0, 0), (0, 1, 0), (0.5, 0.5, 0.5))],
                sha="0" * 64,
            )
            with self.assertRaises(MapShaMismatch):
                measure_map(map_path, patches_path, out_dir)
            self.assertFalse(os.path.exists(os.path.join(out_dir, "wallpatch.json")))

    def test_gates_labelled_proposed(self):
        # Spec correction: the 5 cm gates are approved as provisional.
        # The numbers are unchanged. The old "not yet approved" phrase is retired.
        rng = np.random.default_rng(21)
        n_points = 4000
        clean = np.column_stack(
            [
                rng.uniform(-0.4, 0.4, n_points),
                rng.normal(0.0, 0.01, n_points),
                rng.uniform(-0.4, 0.4, n_points),
            ]
        )
        doubled = _two_sheets(separation=0.06, sigma=0.005, n_each=2000, seed=22)
        doubled[:, 0] += 3.0
        points = np.vstack([clean, doubled])
        with tempfile.TemporaryDirectory() as directory:
            report, out_dir = _measure(
                directory,
                points,
                [
                    _box("clean", (0, 0, 0), (0, 1, 0), (0.5, 0.2, 0.5)),
                    _box("wide", (3, 0, 0), (0, 1, 0), (0.5, 0.2, 0.5)),
                ],
            )
            with open(os.path.join(out_dir, "wallpatch.md"), encoding="utf-8") as handle:
                text = handle.read()
        self.assertEqual(report["gates"]["label"], _GATE_LABEL)
        self.assertEqual(report["gates"]["p95_point_to_plane_max_m"], 0.05)
        self.assertEqual(report["gates"]["revisit_separation_max_m"], 0.05)
        self.assertIn(_GATE_LABEL, text)
        self.assertEqual(report["revisit_note"], _REVISIT)
        self.assertIn(_REVISIT, text)
        clean_patch = report["patches"][0]
        wide_patch = report["patches"][1]
        self.assertTrue(clean_patch["gate_p95_pass"])
        self.assertTrue(clean_patch["gate_separation_pass"])
        self.assertEqual(wide_patch["doubled"], "true")
        self.assertFalse(wide_patch["gate_separation_pass"])
        self.assertGreater(wide_patch["separation_m"], 0.05)

    def test_suggest_finds_synthetic_room_walls(self):
        points = _room(seed=0)
        walls = [
            ("x0", np.array([0.0, 0.0, 0.0]), np.array([1.0, 0.0, 0.0])),
            ("x5", np.array([5.0, 0.0, 0.0]), np.array([-1.0, 0.0, 0.0])),
            ("y0", np.array([0.0, 0.0, 0.0]), np.array([0.0, 1.0, 0.0])),
            ("y4", np.array([0.0, 4.0, 0.0]), np.array([0.0, -1.0, 0.0])),
        ]
        with tempfile.TemporaryDirectory() as directory:
            map_path = os.path.join(directory, "map.pcd")
            out_path = os.path.join(directory, "patches.json")
            write_binary_pcd(map_path, points)
            document = suggest_map(map_path, out_path, n=6)
            self.assertEqual(document["schema"], "spatio.wall_patches/1")
            self.assertEqual(document["map_sha256"], sha256_file(map_path))
            self.assertGreaterEqual(len(document["patches"]), 4)
            ids = []
            for patch in document["patches"][:4]:
                box = patch["box_map_frame"]
                half = box["half_extent_m"]
                self.assertEqual(len(half), 3)
                self.assertTrue(all(value > 0.0 for value in half))
                ids.append(_which_wall(box, walls))
            self.assertEqual(
                sorted(ids),
                ["x0", "x5", "y0", "y4"],
                msg=json.dumps(document["patches"][:4]),
            )

    def test_png_written_per_patch(self):
        rng = np.random.default_rng(4)
        n_points = 800
        points = np.column_stack(
            [
                rng.uniform(-0.3, 0.3, n_points),
                rng.normal(0.0, 0.005, n_points),
                rng.uniform(-0.3, 0.3, n_points),
            ]
        )
        with tempfile.TemporaryDirectory() as directory:
            report, out_dir = _measure(
                directory,
                points,
                [_box("patch A", (0, 0, 0), (0, 1, 0), (0.4, 0.2, 0.4))],
            )
            patch = report["patches"][0]
            figure = os.path.join(out_dir, patch["figure"])
            self.assertTrue(os.path.isfile(figure))
            self.assertTrue(patch["figure"].endswith(".png"))
            with open(figure, "rb") as handle:
                magic = handle.read(8)
            self.assertEqual(magic, b"\x89PNG\r\n\x1a\n")
            self.assertGreater(os.path.getsize(figure), 2000)

    def test_cli_exit_codes(self):
        usage = _cli([])
        self.assertEqual(usage.returncode, 2)
        missing = _cli(["suggest", "no-such-map.pcd", "--n", "2", "--out", "out.json"])
        self.assertEqual(missing.returncode, 1)
        with tempfile.TemporaryDirectory() as directory:
            map_path = os.path.join(directory, "map.pcd")
            patches_path = os.path.join(directory, "patches.json")
            out_dir = os.path.join(directory, "out")
            suggest_path = os.path.join(directory, "patches.json")
            points = _room(seed=1)
            # Fewer points than the suggest test, still a room with four walls.
            write_binary_pcd(map_path, points)
            suggested = _cli(
                ["suggest", map_path, "--n", "4", "--out", suggest_path]
            )
            self.assertEqual(suggested.returncode, 0, suggested.stderr)
            with open(suggest_path, encoding="utf-8") as handle:
                document = json.load(handle)
            self.assertEqual(document["schema"], "spatio.wall_patches/1")
            self.assertGreaterEqual(len(document["patches"]), 4)

            _write_patches(
                patches_path,
                map_path,
                [_box("a", (0, 0, 0), (1, 0, 0), (0.5, 0.3, 0.5))],
                sha="ab" * 32,
            )
            mismatched = _cli(
                ["measure", map_path, "--patches", patches_path, "--out", out_dir]
            )
            self.assertEqual(mismatched.returncode, 1, mismatched.stderr)
            self.assertIn("map_sha256", mismatched.stderr)

            _write_patches(
                patches_path,
                map_path,
                [_box("a", (0, 2, 1.2), (1, 0, 0), (0.5, 0.3, 0.5))],
            )
            ok = _cli(
                ["measure", map_path, "--patches", patches_path, "--out", out_dir]
            )
            self.assertEqual(ok.returncode, 0, ok.stderr)
            self.assertTrue(os.path.isfile(os.path.join(out_dir, "wallpatch.json")))

            absent = _cli(
                [
                    "measure",
                    map_path,
                    "--patches",
                    os.path.join(directory, "missing.json"),
                    "--out",
                    out_dir,
                ]
            )
            self.assertEqual(absent.returncode, 1)

    def test_minimum_count_same_sheet_peaks_do_not_hide_the_gap(self):
        # Review case: seed 65, 100 points on each sheet, 6 cm apart, σ = 8 mm.
        # The two tallest peaks sit on one sheet. The y-mean gap is 5.94 cm.
        # That must not come back as a single wall with both gates passing.
        separation = 0.06
        sigma = 0.008
        n_each = 100
        points = _two_sheets(separation=separation, sigma=sigma, n_each=n_each, seed=65)
        gap = abs(float(points[:n_each, 1].mean() - points[n_each:, 1].mean()))
        self.assertAlmostEqual(gap, 0.0594, delta=0.0001)
        with tempfile.TemporaryDirectory() as directory:
            report, _out = _measure(
                directory,
                points,
                [_box("thin", (0, 0, 0), (0, 1, 0), (0.6, 0.2, 0.6))],
            )
        patch = report["patches"][0]
        self.assertEqual(patch["status"], "MEASURED")
        self.assertEqual(patch["n_measured"], n_each * 2)
        self.assertEqual(patch["doubled"], "true")
        self.assertAlmostEqual(patch["separation_m"], gap, delta=0.002)
        self.assertGreater(patch["separation_m"], 0.05)
        self.assertTrue(patch["gate_p95_pass"])
        self.assertFalse(patch["gate_separation_pass"])

    def test_zero_noise_plane_keeps_histogram_length(self):
        # A flat patch is one histogram bin. The old smoother emitted three
        # bins and raised IndexError. Two close values are two bins.
        counts, _edges = _histogram(np.zeros(400))
        self.assertEqual(len(counts), 1)
        self.assertEqual(len(_smooth(counts)), 1)
        # 0 mm and 3 mm land in two 2 mm bins. 1 mm would still share a bin.
        close = np.array([0.0] * 10 + [0.003] * 10)
        counts_two, _edges_two = _histogram(close)
        self.assertEqual(len(counts_two), 2)
        self.assertEqual(len(_smooth(counts_two)), 2)
        flat = analyze_distances(np.zeros(400))
        self.assertEqual(flat["doubled"], "false")
        self.assertIsNone(flat["separation_m"])
        spanned = analyze_distances(close)
        self.assertEqual(spanned["doubled"], "false")

        xs = np.linspace(-0.2, 0.2, 20)
        zs = np.linspace(-0.2, 0.2, 20)
        xx, zz = np.meshgrid(xs, zs)
        points = np.column_stack([xx.ravel(), np.zeros(400), zz.ravel()])
        self.assertEqual(len(points), 400)
        with tempfile.TemporaryDirectory() as directory:
            report, _out = _measure(
                directory,
                points,
                [_box("flat", (0, 0, 0), (0, 1, 0), (0.3, 0.2, 0.3))],
            )
        patch = report["patches"][0]
        self.assertEqual(patch["status"], "MEASURED")
        self.assertEqual(patch["n_measured"], 400)
        self.assertEqual(patch["doubled"], "false")
        self.assertIsNone(patch["separation_m"])
        self.assertEqual(patch["residuals"]["p95_abs_m"], 0.0)

    def test_ascii_pcd_uses_cumulative_field_counts(self):
        # Independent of write_ascii_pcd. COUNT 3 on the leading field means
        # xyz starts at token 3. Field indices would read (20, 30, 1).
        text = (
            "# .PCD v0.7 - Point Cloud Data file format\n"
            "VERSION 0.7\n"
            "FIELDS descriptor x y z\n"
            "SIZE 4 4 4 4\n"
            "TYPE F F F F\n"
            "COUNT 3 1 1 1\n"
            "WIDTH 2\n"
            "HEIGHT 1\n"
            "VIEWPOINT 0 0 0 1 0 0 0\n"
            "POINTS 2\n"
            "DATA ascii\n"
            "10 20 30 1 2 3\n"
            "10 20 30 4 5 6\n"
        )
        with tempfile.TemporaryDirectory() as directory:
            path = os.path.join(directory, "count.pcd")
            with open(path, "w", encoding="ascii") as handle:
                handle.write(text)
            cloud = read_xyz(path)
        self.assertEqual(cloud.shape, (2, 3))
        np.testing.assert_allclose(cloud, [[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]])
        self.assertFalse(np.allclose(cloud[0], [20.0, 30.0, 1.0]))

    def test_absurd_numbers_rejected_by_one_rule(self):
        # Every rejection below is the shared parser. The phrase is one rule.
        with self.assertRaises(WallpatchError) as huge:
            parse_real(1e20, "x")
        with self.assertRaises(WallpatchError) as nan:
            parse_real(float("nan"), "x")
        with self.assertRaises(WallpatchError) as infinite:
            parse_real(float("inf"), "x")
        with self.assertRaises(WallpatchError) as over:
            parse_real(10000.1, "x")
        for caught in (huge, nan, infinite, over):
            self.assertIn("outside the accepted range", str(caught.exception))
        self.assertEqual(parse_real(10000, "x"), 10000.0)
        self.assertEqual(parse_real(-10000, "y"), -10000.0)
        self.assertEqual(parse_real(104.0611572265625, "z"), 104.0611572265625)

        with self.assertRaises(WallpatchError) as box:
            parse_box(
                {
                    "center_m": [0, 0, 0],
                    "normal_hint": [0, 1, 0],
                    "half_extent_m": [0.5, 1e20, 0.5],
                }
            )
        self.assertIn("outside the accepted range", str(box.exception))
        with self.assertRaises(WallpatchError) as negative:
            parse_box(
                {
                    "center_m": [0, 0, 0],
                    "normal_hint": [0, 1, 0],
                    "half_extent_m": [0.5, -0.2, 0.5],
                }
            )
        self.assertIn("must be positive", str(negative.exception))
        with self.assertRaises(WallpatchError) as flag:
            parse_real(True, "x")
        self.assertIn("not a real number", str(flag.exception))

        with self.assertRaises(WallpatchError) as voxel:
            parse_voxel_m(float("inf"))
        self.assertIn("outside the accepted range", str(voxel.exception))
        with self.assertRaises(WallpatchError) as big_n:
            parse_suggest_n(10 ** 12)
        self.assertIn("outside the accepted range", str(big_n.exception))
        with self.assertRaises(WallpatchError):
            parse_suggest_n(0)
        with self.assertRaises(WallpatchError):
            parse_voxel_m(0.0)

        ascii_huge = (
            "# .PCD v0.7\n"
            "VERSION 0.7\n"
            "FIELDS x y z\n"
            "SIZE 4 4 4\n"
            "TYPE F F F\n"
            "COUNT 1 1 1\n"
            "WIDTH 1\n"
            "HEIGHT 1\n"
            "VIEWPOINT 0 0 0 1 0 0 0\n"
            "POINTS 1\n"
            "DATA ascii\n"
            "1e20 0 0\n"
        )
        absurd_count = (
            "# .PCD v0.7\n"
            "VERSION 0.7\n"
            "FIELDS x y z\n"
            "SIZE 4 4 4\n"
            "TYPE F F F\n"
            "COUNT 1 1 1\n"
            "WIDTH 1\n"
            "HEIGHT 1\n"
            "VIEWPOINT 0 0 0 1 0 0 0\n"
            "POINTS 999999999999\n"
            "DATA ascii\n"
        )
        with tempfile.TemporaryDirectory() as directory:
            huge_path = os.path.join(directory, "huge.pcd")
            count_path = os.path.join(directory, "count.pcd")
            binary_path = os.path.join(directory, "far.pcd")
            near_path = os.path.join(directory, "near.pcd")
            with open(huge_path, "w", encoding="ascii") as handle:
                handle.write(ascii_huge)
            with open(count_path, "w", encoding="ascii") as handle:
                handle.write(absurd_count)
            with self.assertRaises(WallpatchError) as ascii_err:
                read_xyz(huge_path)
            with self.assertRaises(WallpatchError) as count_err:
                read_xyz(count_path)
            write_binary_pcd(binary_path, np.array([[20000.0, 0.0, 0.0]]))
            with self.assertRaises(WallpatchError) as binary_err:
                read_xyz(binary_path)
            write_binary_pcd(near_path, np.array([[104.0, -26.0, 0.5]]))
            near = read_xyz(near_path)
        self.assertIn("outside the accepted range", str(ascii_err.exception))
        self.assertIn("outside the accepted range", str(count_err.exception))
        self.assertIn("outside the accepted range", str(binary_err.exception))
        np.testing.assert_allclose(near, [[104.0, -26.0, 0.5]])

    def test_binary_pcd_spans_more_than_one_chunk(self):
        # The chunk length must stay an integer across iterations. A cloud
        # one point past the reader chunk used to crash on the second read.
        n_points = DEFAULT_CHUNK + 1
        points = np.zeros((n_points, 3), dtype=np.float64)
        points[0] = (1.0, 2.0, 3.0)
        points[-1] = (4.0, 5.0, 6.0)
        with tempfile.TemporaryDirectory() as directory:
            path = os.path.join(directory, "wide.pcd")
            write_binary_pcd(path, points)
            cloud = read_xyz(path)
        self.assertEqual(len(cloud), n_points)
        np.testing.assert_allclose(cloud[0], [1.0, 2.0, 3.0])
        np.testing.assert_allclose(cloud[-1], [4.0, 5.0, 6.0])
        self.assertEqual(float(np.max(np.abs(cloud[1:-1]))), 0.0)

    def test_plane_perpendicular_to_hint_does_not_pass_doubled_wall(self):
        # Review repro: two strips 6 cm apart, 1 m along x, 1 cm along z.
        # The thinnest axis is vertical, 90 degrees from normal_hint. Scoring
        # that plane reports a 5 mm residual and both gates pass. The same
        # sheets with a 1 m vertical span are a doubled wall and fail.
        narrow = _two_strips(z_half=0.005)
        wide = _two_strips(z_half=0.5)
        self.assertEqual(len(narrow), 400)
        self.assertEqual(len(wide), 400)
        box = [_box("strips", (0, 0, 0), (0, 1, 0), (0.6, 0.2, 0.6))]
        with tempfile.TemporaryDirectory() as directory:
            narrow_report, _out = _measure(directory, narrow, box)
            wide_report, _wide_out = _measure(directory, wide, box)
        hidden = narrow_report["patches"][0]
        self.assertEqual(hidden["n_cropped"], 400)
        self.assertEqual(hidden["status"], "NOT_EVALUABLE")
        self.assertIsNone(hidden["n_measured"])
        self.assertIsNone(hidden["doubled"])
        self.assertIsNone(hidden["separation_m"])
        self.assertIsNone(hidden["gate_p95_pass"])
        self.assertIsNone(hidden["gate_separation_pass"])
        self.assertNotEqual(hidden["gate_p95_pass"], True)
        self.assertNotEqual(hidden["gate_separation_pass"], True)
        reason = hidden["reasons"][0]
        self.assertIn("normal_hint", reason)
        self.assertIn("not supported", reason)
        self.assertGreater(float(reason.split()[3]), 45.0)
        shown = wide_report["patches"][0]
        self.assertEqual(shown["status"], "MEASURED")
        self.assertEqual(shown["n_measured"], 400)
        self.assertEqual(shown["doubled"], "true")
        self.assertAlmostEqual(shown["separation_m"], 0.06, delta=1e-6)
        self.assertFalse(shown["gate_separation_pass"])
        self.assertLess(shown["plane"]["angle_from_hint_deg"], 1.0)

    def test_understated_pcd_count_does_not_hide_the_second_wall(self):
        # Review repro: 400 points on two sheets 6 cm apart, header POINTS 200
        # and WIDTH 400. Reading 200 points keeps the first sheet. That sheet
        # passes both gates. The full cloud fails the separation gate.
        points = _two_strips(z_half=0.5)
        self.assertEqual(len(points), 400)
        self.assertEqual(len(points[:200]), 200)
        box = [_box("sheets", (0, 0, 0), (0, 1, 0), (0.6, 0.2, 0.6))]
        with tempfile.TemporaryDirectory() as directory:
            full, _out = _measure(directory, points, box)
            first_only, _first_out = _measure(directory, points[:200], box)
        self.assertEqual(full["patches"][0]["status"], "MEASURED")
        self.assertEqual(full["patches"][0]["n_measured"], 400)
        self.assertEqual(full["patches"][0]["doubled"], "true")
        self.assertAlmostEqual(full["patches"][0]["separation_m"], 0.06, delta=1e-6)
        self.assertFalse(full["patches"][0]["gate_separation_pass"])
        self.assertEqual(first_only["patches"][0]["doubled"], "false")
        self.assertTrue(first_only["patches"][0]["gate_p95_pass"])
        self.assertTrue(first_only["patches"][0]["gate_separation_pass"])

        exact_header = (
            "VERSION 0.7\n"
            "FIELDS x y z\n"
            "SIZE 4 4 4\n"
            "TYPE F F F\n"
            "COUNT 1 1 1\n"
            "WIDTH 400\n"
            "HEIGHT 1\n"
            "POINTS 200\n"
            "DATA ascii\n"
        )
        with tempfile.TemporaryDirectory() as directory:
            ascii_path = os.path.join(directory, "map.pcd")
            with open(ascii_path, "w", encoding="ascii") as handle:
                handle.write(exact_header + _ascii_rows(points))
            patches_path = os.path.join(directory, "patches.json")
            out_dir = os.path.join(directory, "out")
            _write_patches(patches_path, ascii_path, box)
            with self.assertRaises(WallpatchError) as ascii_exact:
                measure_map(ascii_path, patches_path, out_dir)
            self.assertFalse(os.path.isfile(os.path.join(out_dir, "wallpatch.json")))
            binary_path = os.path.join(directory, "under.pcd")
            _write_raw_pcd(binary_path, points, 400, 1, 200, binary=True)
            with self.assertRaises(WallpatchError) as binary_exact:
                read_xyz(binary_path)
        for caught in (ascii_exact, binary_exact):
            self.assertIn("does not match", str(caught.exception))
            self.assertIn("WIDTH 400", str(caught.exception))
            self.assertIn("HEIGHT 1", str(caught.exception))
            self.assertIn("POINTS 200", str(caught.exception))

        # Counts agree with each other. The extra sheet is still in the file.
        with tempfile.TemporaryDirectory() as directory:
            ascii_extra = os.path.join(directory, "extra.pcd")
            binary_extra = os.path.join(directory, "extra_bin.pcd")
            _write_raw_pcd(ascii_extra, points, 200, 1, 200, binary=False)
            _write_raw_pcd(binary_extra, points, 200, 1, 200, binary=True)
            with self.assertRaises(WallpatchError) as ascii_extra_err:
                read_xyz(ascii_extra)
            with self.assertRaises(WallpatchError) as binary_extra_err:
                read_xyz(binary_extra)
        self.assertIn(
            "more points than the header POINTS 200",
            str(ascii_extra_err.exception),
        )
        self.assertIn("payload", str(binary_extra_err.exception))
        self.assertNotIn("does not match", str(binary_extra_err.exception))

        # Payload length matches POINTS. WIDTH x HEIGHT does not, including
        # a height other than 1 so the product, not the width alone, is checked.
        with tempfile.TemporaryDirectory() as directory:
            ascii_short = os.path.join(directory, "short.pcd")
            binary_short = os.path.join(directory, "short_bin.pcd")
            product = os.path.join(directory, "product.pcd")
            _write_raw_pcd(ascii_short, points[:200], 400, 1, 200, binary=False)
            _write_raw_pcd(binary_short, points[:200], 400, 1, 200, binary=True)
            _write_raw_pcd(product, points[:200], 200, 2, 200, binary=False)
            with self.assertRaises(WallpatchError) as ascii_short_err:
                read_xyz(ascii_short)
            with self.assertRaises(WallpatchError) as binary_short_err:
                read_xyz(binary_short)
            with self.assertRaises(WallpatchError) as product_err:
                read_xyz(product)
        for caught in (ascii_short_err, binary_short_err):
            self.assertIn("WIDTH 400", str(caught.exception))
            self.assertIn("POINTS 200", str(caught.exception))
        self.assertIn("WIDTH 200 x HEIGHT 2", str(product_err.exception))
        self.assertIn("POINTS 200", str(product_err.exception))

        with tempfile.TemporaryDirectory() as directory:
            ok_path = os.path.join(directory, "ok.pcd")
            write_ascii_pcd(ok_path, points)
            cloud = read_xyz(ok_path)
        self.assertEqual(len(cloud), 400)
        np.testing.assert_allclose(cloud[:200, 1], -0.03)
        np.testing.assert_allclose(cloud[200:, 1], 0.03)

    def test_partial_overlap_pooled_tilt_does_not_pass_doubled_wall(self):
        # Review repro. Two noiseless 40 by 20 grids, y = ±0.03 m.
        # x ranges [-0.5, 0.2] and [-0.2, 0.5], z spans [-0.5, 0.5],
        # hint (0, 1, 0). The pooled fit tilts 3.97 deg. Distances along
        # that normal used to report a single wall, p95 4.14 cm, and both
        # gates passed. The sheets are 6 cm apart along the hint.
        points = _partial_overlap_grids(separation=0.06, overlap=0.2)
        self.assertEqual(len(points), 1600)
        self.assertEqual(int(np.count_nonzero(np.isclose(points[:, 1], -0.03))), 800)
        self.assertEqual(int(np.count_nonzero(np.isclose(points[:, 1], 0.03))), 800)
        box = [_box("overlap", (0, 0, 0), (0, 1, 0), (0.6, 0.2, 0.6))]
        with tempfile.TemporaryDirectory() as directory:
            report, _out = _measure(directory, points, box)
        patch = report["patches"][0]
        self.assertEqual(patch["n_cropped"], 1600)
        self.assertEqual(patch["n_measured"], 1600)
        self.assertEqual(patch["status"], "MEASURED")
        self.assertAlmostEqual(patch["plane"]["angle_from_hint_deg"], 3.97, delta=0.02)
        self.assertGreater(patch["residuals"]["p95_abs_m"], 0.04)
        self.assertLess(patch["residuals"]["p95_abs_m"], 0.05)
        self.assertTrue(patch["gate_p95_pass"])
        self.assertEqual(patch["doubled"], "true")
        self.assertEqual(patch["separation_axis"], "normal_hint")
        # map.pcd stores float32. 0.03 m is not bit-exact after the round trip.
        self.assertAlmostEqual(patch["separation_m"], 0.06, delta=1e-6)
        self.assertFalse(patch["gate_separation_pass"])
        self.assertFalse(
            patch["gate_p95_pass"] is True and patch["gate_separation_pass"] is True
        )

    def test_partial_overlap_candidate_gap_is_not_evaluable(self):
        # Same grids, 3 cm apart. The pooled fit reports a single wall.
        # The hint reports an undetermined candidate. That disagreement
        # must not pass either gate.
        points = _partial_overlap_grids(separation=0.03, overlap=0.2)
        self.assertEqual(len(points), 1600)
        box = [_box("overlap", (0, 0, 0), (0, 1, 0), (0.6, 0.2, 0.6))]
        with tempfile.TemporaryDirectory() as directory:
            report, _out = _measure(directory, points, box)
        patch = report["patches"][0]
        self.assertEqual(patch["n_cropped"], 1600)
        self.assertEqual(patch["status"], "NOT_EVALUABLE")
        self.assertIsNone(patch["n_measured"])
        self.assertIsNone(patch["doubled"])
        self.assertIsNone(patch["separation_m"])
        self.assertIsNone(patch["separation_axis"])
        self.assertIsNone(patch["gate_p95_pass"])
        self.assertIsNone(patch["gate_separation_pass"])
        self.assertNotEqual(patch["gate_p95_pass"], True)
        self.assertNotEqual(patch["gate_separation_pass"], True)
        reason = patch["reasons"][0]
        self.assertIn("doubled=false", reason)
        self.assertIn("doubled=undetermined", reason)
        self.assertIn("not evaluable", reason)

    def test_splits_on_opposite_sides_of_the_gate_are_not_evaluable(self):
        # Both axes see two surfaces, but one gap would pass 5 cm and the
        # other would fail. Scoring the smaller gap would be a pass.
        decision = reconcile_axes(
            {"doubled": "true", "separation_m": 0.04},
            {"doubled": "true", "separation_m": 0.06},
            0.05,
        )
        self.assertEqual(decision["kind"], "not_evaluable")
        self.assertIsNone(decision["stats"])
        self.assertIn("opposite sides", decision["reason"])
        self.assertIn("not evaluable", decision["reason"])
        self.assertIn("0.05", decision["reason"])
        agreed = reconcile_axes(
            {"doubled": "true", "separation_m": 0.06},
            {"doubled": "true", "separation_m": 0.08},
            0.05,
        )
        self.assertEqual(agreed["kind"], "measured")
        self.assertEqual(agreed["stats"]["separation_m"], 0.06)

    def test_rotated_hint_on_6cm_overlap_never_passes(self):
        # Review repro: the 6 cm partial-overlap grids with normal_hint rotated
        # 4 deg about z came back doubled=false, p95 4.14 cm, both gates
        # passing. Before this fix, rotations about z of -6, -4 and -2 deg,
        # and of -4 and +4 deg about x, all passed. At -4 deg about z the
        # hint lies 0.03 deg from the fitted plane, so no angle check sees it.
        points = _partial_overlap_grids(separation=0.06, overlap=0.2)
        self.assertEqual(len(points), 1600)
        passed_before = {("z", -6), ("z", -4), ("z", -2), ("x", -4), ("x", 4)}
        seen = 0
        with tempfile.TemporaryDirectory() as directory:
            for axis in ("z", "x"):
                for degrees in (-6, -5, -4, -3, -2, -1, 1, 2, 3, 4, 5, 6):
                    theta = np.radians(degrees)
                    if axis == "z":
                        hint = (np.sin(theta), np.cos(theta), 0.0)
                    else:
                        hint = (0.0, np.cos(theta), np.sin(theta))
                    box = [_box("overlap", (0, 0, 0), hint, (0.6, 0.2, 0.6))]
                    report, _out = _measure(directory, points, box)
                    patch = report["patches"][0]
                    label = "%s %+d deg" % (axis, degrees)
                    self.assertEqual(patch["n_cropped"], 1600, label)
                    self.assertFalse(
                        patch["gate_p95_pass"] is True
                        and patch["gate_separation_pass"] is True,
                        label,
                    )
                    self.assertIn(patch["status"], ("MEASURED", "NOT_EVALUABLE"), label)
                    if patch["status"] == "MEASURED":
                        self.assertEqual(patch["doubled"], "true", label)
                        self.assertFalse(patch["gate_separation_pass"], label)
                        self.assertGreater(patch["separation_m"], 0.05, label)
                    if (axis, degrees) in passed_before:
                        seen += 1
                        self.assertEqual(patch["status"], "NOT_EVALUABLE", label)
                        self.assertIsNone(patch["gate_p95_pass"], label)
                        self.assertIsNone(patch["gate_separation_pass"], label)
                        self.assertIn("not evaluable", patch["reasons"][0], label)
        self.assertEqual(seen, len(passed_before))

    def test_one_axis_split_under_the_gate_is_not_a_pass(self):
        # Review case at z -2 deg: the hint saw a split the fit missed, and the
        # gap along that tilted hint read under 5 cm, so the gate passed.
        for fit_d, hint_d in (("false", "true"), ("true", "false")):
            short = {"doubled": "true", "separation_m": 0.045}
            single = {"doubled": "false", "separation_m": None}
            fit_stats = short if fit_d == "true" else single
            hint_stats = short if hint_d == "true" else single
            refused = reconcile_axes(fit_stats, hint_stats, 0.05)
            self.assertEqual(refused["kind"], "not_evaluable")
            self.assertIn("not evaluable", refused["reason"])
            self.assertIn("0.0450 m", refused["reason"])
            wide = {"doubled": "true", "separation_m": 0.06}
            fit_stats = wide if fit_d == "true" else single
            hint_stats = wide if hint_d == "true" else single
            failed = reconcile_axes(fit_stats, hint_stats, 0.05)
            self.assertEqual(failed["kind"], "measured")
            self.assertEqual(failed["stats"]["separation_m"], 0.06)

    def test_64_clutter_points_in_the_valley_do_not_pass(self):
        # 64 points on the mid-plane of the overlap, where they fill the
        # valley between the two sheets and could merge the split.
        sheets = _partial_overlap_grids(separation=0.06, overlap=0.2)
        xx, zz = np.meshgrid(np.linspace(-0.2, 0.2, 8), np.linspace(-0.5, 0.5, 8))
        clutter = np.column_stack([xx.ravel(), np.zeros(xx.size), zz.ravel()])
        self.assertEqual(len(clutter), 64)
        points = np.vstack([sheets, clutter])
        theta = np.radians(-4.0)
        hints = ((0.0, 1.0, 0.0), (np.sin(theta), np.cos(theta), 0.0))
        with tempfile.TemporaryDirectory() as directory:
            for hint in hints:
                box = [_box("cluttered", (0, 0, 0), hint, (0.6, 0.2, 0.6))]
                report, _out = _measure(directory, points, box)
                patch = report["patches"][0]
                self.assertEqual(patch["n_cropped"], 1664, hint)
                self.assertFalse(
                    patch["gate_p95_pass"] is True
                    and patch["gate_separation_pass"] is True,
                    hint,
                )
                if patch["status"] == "MEASURED":
                    self.assertEqual(patch["doubled"], "true", hint)
                    self.assertFalse(patch["gate_separation_pass"], hint)

    def test_clean_single_wall_with_exact_hint_still_passes(self):
        rng = np.random.default_rng(4242)
        n_points = 6000
        points = np.column_stack(
            [
                rng.uniform(-0.5, 0.5, n_points),
                rng.normal(0.0, 0.005, n_points),
                rng.uniform(-0.5, 0.5, n_points),
            ]
        )
        with tempfile.TemporaryDirectory() as directory:
            report, _out = _measure(
                directory,
                points,
                [_box("clean", (0, 0, 0), (0, 1, 0), (0.6, 0.2, 0.6))],
            )
        patch = report["patches"][0]
        self.assertEqual(patch["status"], "MEASURED")
        self.assertEqual(patch["reasons"], [])
        self.assertEqual(patch["n_measured"], n_points)
        self.assertEqual(patch["doubled"], "false")
        self.assertLess(patch["residuals"]["thickness_m"], 0.05)
        self.assertLess(patch["residuals"]["core_band_m"], 0.05)
        self.assertLess(patch["residuals"]["stepped_band_m"], 0.05)
        self.assertIs(patch["gate_p95_pass"], True)
        self.assertIs(patch["gate_separation_pass"], True)
        self.assertEqual(report["doubled_wall_rule"]["pass_needs_band_under_m"], 0.05)

    def test_thick_band_blocks_only_a_would_be_pass(self):
        # A 7 cm slab of uniform depth: p95 is about 3.3 cm and passes the
        # p95 gate, but the narrowest band holding 80% is about 5.6 cm.
        # Two surfaces 5 cm apart fit inside it. That is not a pass.
        rng = np.random.default_rng(77)
        n_points = 6000
        points = np.column_stack(
            [
                rng.uniform(-0.5, 0.5, n_points),
                rng.uniform(-0.035, 0.035, n_points),
                rng.uniform(-0.5, 0.5, n_points),
            ]
        )
        with tempfile.TemporaryDirectory() as directory:
            report, _out = _measure(
                directory,
                points,
                [_box("wide", (0, 0, 0), (0, 1, 0), (0.6, 0.2, 0.6))],
            )
        patch = report["patches"][0]
        self.assertEqual(patch["status"], "NOT_EVALUABLE")
        self.assertIsNone(patch["gate_p95_pass"])
        self.assertIsNone(patch["gate_separation_pass"])
        self.assertIn("narrowest band holding 80%", patch["reasons"][0])
        self.assertIn("not evaluable", patch["reasons"][0])
        # The guard never turns a fail into NOT_EVALUABLE: the 6 cm sheets
        # with an exact hint have an even wider band and stay a measured fail.
        wide = _partial_overlap_grids(separation=0.06, overlap=0.2)
        with tempfile.TemporaryDirectory() as directory:
            failed, _out = _measure(
                directory, wide, [_box("overlap", (0, 0, 0), (0, 1, 0), (0.6, 0.2, 0.6))]
            )
        kept = failed["patches"][0]
        self.assertEqual(kept["status"], "MEASURED")
        self.assertGreater(kept["residuals"]["core_band_m"], 0.05)
        self.assertFalse(kept["gate_separation_pass"])
        self.assertIsNone(kept["residuals"]["stepped_band_m"])

    def test_offset_sheets_tilting_the_fit_are_not_a_pass(self):
        # The blind spot left by the 80% band. Two sheets 5.2 to 7 cm apart,
        # end to end, with a 20 cm gap, or with a 10 cm overlap. The hint is
        # rotated 4 deg toward the pooled tilt. The fit tilts and takes up
        # the offset, so the band around it stays under 5 cm. Every case
        # here passed both gates before the stepped-band guard.
        theta = np.radians(-4.0)
        hint = (np.sin(theta), np.cos(theta), 0.0)
        cases = (
            (0.0, 0.052),
            (0.0, 0.055),
            (0.0, 0.06),
            (-0.1, 0.052),
            (-0.1, 0.06),
            (-0.1, 0.07),
            (0.1, 0.052),
        )
        with tempfile.TemporaryDirectory() as directory:
            for layout, gap in cases:
                label = "layout %+.1f gap %.3f" % (layout, gap)
                points = _offset_sheets(gap, layout, 0.0, 0.0)
                box = [_box("offset", (0, 0, 0), hint, (0.6, 0.2, 0.6))]
                report, _out = _measure(directory, points, box)
                patch = report["patches"][0]
                self.assertEqual(patch["n_cropped"], len(points), label)
                self.assertEqual(patch["status"], "NOT_EVALUABLE", label)
                self.assertIsNone(patch["gate_p95_pass"], label)
                self.assertIsNone(patch["gate_separation_pass"], label)
                self.assertIn("tilt a step", patch["reasons"][0], label)
                self.assertIn("not evaluable", patch["reasons"][0], label)
        self.assertEqual(report["doubled_wall_rule"]["stepped_band_split_step"], 0.01)
        self.assertEqual(report["doubled_wall_rule"]["stepped_band_turn_deg"], 15.0)

    def test_offset_sheet_matrix_never_passes(self):
        # Step position, step direction in the wall plane, layout, gap and
        # hint all vary. A sheet under MIN_PEAK_MASS of the points is below
        # the split rule's own floor, so those layouts are left out.
        hints = {"exact": (0.0, 1.0, 0.0)}
        for degrees in (-4, -2, 4):
            theta = np.radians(degrees)
            hints["z%+d" % degrees] = (np.sin(theta), np.cos(theta), 0.0)
            hints["x%+d" % degrees] = (0.0, np.cos(theta), np.sin(theta))
        checked = 0
        for layout in (0.0, -0.1, 0.1, 0.2):
            for gap in (0.052, 0.06, 0.07):
                for position in (-0.3, -0.15, 0.0, 0.2):
                    for turn in (0.0, 30.0, 45.0, 90.0):
                        points = _offset_sheets(gap, layout, position, turn)
                        minority = min(np.mean(points[:, 1] < 0), np.mean(points[:, 1] > 0))
                        if minority < 0.2:
                            continue
                        for name, hint in sorted(hints.items()):
                            label = "layout %+.1f gap %.3f at %+.2f turn %.0f hint %s" % (
                                layout,
                                gap,
                                position,
                                turn,
                                name,
                            )
                            spec = (np.zeros(3), np.asarray(hint), np.array([0.6, 0.2, 0.6]))
                            cropped = crop_points(points, *spec)
                            patch = _measure_one("matrix", cropped, spec, None)
                            checked += 1
                            self.assertFalse(
                                patch["gate_p95_pass"] is True
                                and patch["gate_separation_pass"] is True,
                                label,
                            )
        self.assertGreater(checked, 500)

    def test_bowed_single_wall_still_passes(self):
        # The stepped-band guard must not refuse a smooth wall: 1 cm of bow
        # over 1 m, sigma 5 mm, exact hint.
        rng = np.random.default_rng(31)
        n_points = 6000
        x = rng.uniform(-0.5, 0.5, n_points)
        z = rng.uniform(-0.5, 0.5, n_points)
        y = 0.04 * x * x + rng.normal(0.0, 0.005, n_points)
        with tempfile.TemporaryDirectory() as directory:
            report, _out = _measure(
                directory,
                np.column_stack([x, y, z]),
                [_box("bowed", (0, 0, 0), (0, 1, 0), (0.6, 0.2, 0.6))],
            )
        patch = report["patches"][0]
        self.assertEqual(patch["status"], "MEASURED")
        self.assertEqual(patch["reasons"], [])
        self.assertIs(patch["gate_p95_pass"], True)
        self.assertIs(patch["gate_separation_pass"], True)
        self.assertLess(patch["residuals"]["stepped_band_m"], 0.05)

    def test_noisy_offset_sheets_review_case_never_passes(self):
        # sp-rv-m1final3: sheets 5.2 cm apart, 10 cm overlap, join at +0.1 m,
        # 5 mm Gaussian noise on the wall normal (default_rng(3)), hint -4 deg.
        # Before Round 7 the z case was MEASURED, doubled=false, both gates True
        # (80% band 4.888 cm, stepped band 4.963 cm, p95 3.29 cm).
        points = _offset_sheets(0.052, 0.1, 0.1, 0.0)
        points[:, 1] += np.random.default_rng(3).normal(0.0, 0.005, len(points))
        theta = np.radians(-4.0)
        hints = {
            "z-4": (np.sin(theta), np.cos(theta), 0.0),
            "x-4": (0.0, np.cos(theta), np.sin(theta)),
        }
        for label, hint in sorted(hints.items()):
            with tempfile.TemporaryDirectory() as directory:
                report, _out = _measure(
                    directory, points, [_box(label, (0, 0, 0), hint, (0.6, 0.2, 0.6))]
                )
            patch = report["patches"][0]
            self.assertFalse(
                patch["gate_p95_pass"] is True
                and patch["gate_separation_pass"] is True
                and patch["doubled"] == "false",
                label,
            )
            self.assertEqual(report["gates"]["pass_max_m"], 0.04)
            self.assertIn("provisional", report["gates"]["pass_max_label"])
        # The z case is caught by the guard band itself, not by the 5 cm checks.
        spec =(np.zeros(3), np.asarray(hints["z-4"]), np.array([0.6, 0.2, 0.6]))
        patch = _measure_one("z-4", crop_points(points, *spec), spec, None)
        self.assertEqual(patch["status"], "NOT_EVALUABLE")
        self.assertIn("80% band 0.0489 m, stepped 80% band 0.0496 m", patch["reasons"][0])
        self.assertIn("provisional 0.040 m pass limit", patch["reasons"][0])

    def test_guard_band_boundaries(self):
        # <= 4.0 cm passes; above it the would-be pass is not checkable.
        self.assertIsNone(_guard_band_reason(0.04, 0.04, 0.04))
        for values in ((0.0401, 0.01, 0.01), (0.01, 0.0401, 0.01), (0.01, 0.01, 0.0401),
                       (0.05, 0.01, 0.01)):
            reason = _guard_band_reason(*values)
            self.assertIsNotNone(reason, values)
            self.assertIn("not evaluable", reason)


def _offset_sheets(separation, layout, position, turn_deg):
    """Two noiseless sheets at y = ±separation / 2, 40 points per metre by 20 rows.

    The join is at u = position. layout > 0 overlaps each sheet that far past
    the join; layout < 0 leaves a gap of twice that. The pair is then turned
    by turn_deg about y, so the join can run across the wall at any angle.
    """
    sheets = []
    for side in (-1, 1):
        if side < 0:
            low, high = -0.5, min(position + layout, 0.5)
        else:
            low, high = max(position - layout, -0.5), 0.5
        uu, vv = np.meshgrid(
            np.linspace(low, high, max(2, int(round(40 * (high - low))))),
            np.linspace(-0.5, 0.5, 20),
        )
        sheets.append(
            np.column_stack([uu.ravel(), np.full(uu.size, side * 0.5 * separation), vv.ravel()])
        )
    points = np.vstack(sheets)
    turn = np.radians(turn_deg)
    u, y, v = points.T
    return np.column_stack(
        [u * np.cos(turn) - v * np.sin(turn), y, u * np.sin(turn) + v * np.cos(turn)]
    )


def _partial_overlap_grids(separation, overlap):
    """Two noiseless 40 by 20 sheets. Left x ends at +overlap, right x starts at -overlap."""
    sheets = []
    half = 0.5 * separation
    for side in (-1, 1):
        if side < 0:
            xs = np.linspace(-0.5, overlap, 40)
        else:
            xs = np.linspace(-overlap, 0.5, 40)
        xx, zz = np.meshgrid(xs, np.linspace(-0.5, 0.5, 20))
        sheets.append(
            np.column_stack([xx.ravel(), np.full(xx.size, side * half), zz.ravel()])
        )
    return np.vstack(sheets)


def _two_strips(z_half):
    """200 points on each of two sheets at y = ±0.03. x spans 1 m."""
    xx, zz = np.meshgrid(
        np.linspace(-0.5, 0.5, 20),
        np.linspace(-z_half, z_half, 10),
    )
    first = np.column_stack([xx.ravel(), np.full(xx.size, -0.03), zz.ravel()])
    second = first.copy()
    second[:, 1] = 0.03
    return np.vstack([first, second])


def _ascii_rows(points):
    return "".join("%.9g %.9g %.9g\n" % (row[0], row[1], row[2]) for row in points)


def _write_raw_pcd(path, points, width, height, n_header, binary):
    """A PCD whose header counts are chosen by the test, not by the writer."""
    data_line = "DATA binary\n" if binary else "DATA ascii\n"
    header = (
        "VERSION 0.7\n"
        "FIELDS x y z\n"
        "SIZE 4 4 4\n"
        "TYPE F F F\n"
        "COUNT 1 1 1\n"
        "WIDTH %d\n"
        "HEIGHT %d\n"
        "POINTS %d\n"
        "%s"
    ) % (width, height, n_header, data_line)
    with open(path, "wb") as handle:
        handle.write(header.encode("ascii"))
        if binary:
            handle.write(np.ascontiguousarray(points, dtype="<f4").tobytes())
        else:
            handle.write(_ascii_rows(points).encode("ascii"))


def _two_sheets(separation, sigma, n_each, seed):
    rng = np.random.default_rng(seed)
    sheets = []
    for centre in (-0.5 * separation, 0.5 * separation):
        sheets.append(
            np.column_stack(
                [
                    rng.uniform(-0.5, 0.5, n_each),
                    np.full(n_each, centre) + rng.normal(0.0, sigma, n_each),
                    rng.uniform(-0.5, 0.5, n_each),
                ]
            )
        )
    return np.vstack(sheets)


def _room(seed):
    """Four vertical walls, a floor, and a clutter blob."""
    rng = np.random.default_rng(seed)
    specs = [
        ((0, 0, 0), (0, 1, 0), (0, 0, 1), 4.0, 2.5, (1.0, 0.0, 0.0), 4000),
        ((5, 0, 0), (0, 1, 0), (0, 0, 1), 4.0, 2.5, (-1.0, 0.0, 0.0), 4000),
        ((0, 0, 0), (1, 0, 0), (0, 0, 1), 5.0, 2.5, (0.0, 1.0, 0.0), 5000),
        ((0, 4, 0), (1, 0, 0), (0, 0, 1), 5.0, 2.5, (0.0, -1.0, 0.0), 5000),
    ]
    clouds = []
    for origin, along, vertical, length_a, length_v, normal, count in specs:
        origin = np.asarray(origin, dtype=np.float64)
        along = np.asarray(along, dtype=np.float64)
        vertical = np.asarray(vertical, dtype=np.float64)
        normal = np.asarray(normal, dtype=np.float64)
        coord_a = rng.uniform(0.0, length_a, size=count)
        coord_v = rng.uniform(0.0, length_v, size=count)
        noise = rng.normal(0.0, 0.005, size=count)
        clouds.append(
            origin
            + coord_a[:, None] * along
            + coord_v[:, None] * vertical
            + noise[:, None] * normal
        )
    floor_x = rng.uniform(0.0, 5.0, size=2000)
    floor_y = rng.uniform(0.0, 4.0, size=2000)
    clouds.append(
        np.column_stack([floor_x, floor_y, rng.normal(0.0, 0.005, size=2000)])
    )
    clouds.append(rng.normal(loc=(2.5, 2.0, 1.0), scale=0.15, size=(300, 3)))
    return np.vstack(clouds)


def _which_wall(box, walls):
    center = np.asarray(box["center_m"], dtype=np.float64)
    normal = np.asarray(box["normal_hint"], dtype=np.float64)
    normal = normal / np.linalg.norm(normal)
    hits = []
    for name, point, wall_normal in walls:
        distance = abs(float(np.dot(center - point, wall_normal)))
        cosine = float(np.clip(abs(np.dot(normal, wall_normal)), -1.0, 1.0))
        angle = float(np.degrees(np.arccos(cosine)))
        if distance < 0.15 and angle < 20.0:
            hits.append(name)
    if len(hits) != 1:
        raise AssertionError("box %s matched %s" % (box, hits))
    return hits[0]


def _independent_axes(normal_hint):
    """Same frame as box_axes, written out here so the crop oracle is local."""
    across = np.asarray(normal_hint, dtype=np.float64)
    across = across / np.linalg.norm(across)
    up = np.array([0.0, 0.0, 1.0])
    along = np.cross(up, across)
    along = along / np.linalg.norm(along)
    vertical = np.cross(across, along)
    vertical = vertical / np.linalg.norm(vertical)
    # Keep box_axes honest: the test oracle and the library must agree.
    lib_along, lib_across, lib_vertical = box_axes(normal_hint)
    if (
        np.linalg.norm(lib_along - along) > 1e-9
        or np.linalg.norm(lib_across - across) > 1e-9
        or np.linalg.norm(lib_vertical - vertical) > 1e-9
    ):
        raise AssertionError("box frame drifted from the test oracle")
    return along, across, vertical


def _cli(args):
    env = os.environ.copy()
    env["MPLBACKEND"] = "Agg"
    env["PYTHONPATH"] = ROOT + os.pathsep + env.get("PYTHONPATH", "")
    return subprocess.run(
        [sys.executable, "-m", "spatio.wallpatch"] + args,
        cwd=ROOT,
        env=env,
        capture_output=True,
        text=True,
    )


if __name__ == "__main__":
    unittest.main()
