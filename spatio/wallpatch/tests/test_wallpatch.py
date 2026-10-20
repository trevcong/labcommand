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

from spatio.wallpatch.errors import MapShaMismatch
from spatio.wallpatch.geometry import box_axes, crop_points
from spatio.wallpatch.measure import measure_map
from spatio.wallpatch.pcd import read_xyz, sha256_file, write_ascii_pcd, write_binary_pcd
from spatio.wallpatch.suggest import suggest_map

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
# Phi^{-1}(0.975). p95 of |N(0, sigma)| equals this times sigma.
_GAUSS_P95 = 1.959963984540054
_REVISIT = (
    "map.pcd has no per-point time or visit id. "
    "separation_m is the distance between two parallel surfaces found inside the patch. "
    "It cannot be attributed to a specific visit or revisit."
)
_GATE_LABEL = "proposed, not yet approved by Trevor"


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
        self.assertEqual(patch["status"], "MEASURED")
        self.assertEqual(patch["n_measured"], n_in + n_out)
        self.assertLess(patch["plane"]["angle_from_hint_deg"], 2.0)
        offset = abs(float(np.dot(patch["plane"]["point_m"], patch["plane"]["normal"])))
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
