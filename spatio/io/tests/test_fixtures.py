"""Golden fixtures. Metrics below are written here and do not call synth.py."""

from __future__ import annotations

import json
import math
import subprocess
import sys
import unittest
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[3]
FIXTURES = Path(__file__).resolve().parents[1] / "fixtures"
WINDOW_S = 10.0
EDGE_S = 1e-6
LABEL = "replay demonstrated; physical accuracy not evaluated"
MANIFEST_KEYS = [
    "run_id",
    "source_recording",
    "sensor",
    "fastlio_revision",
    "config_file",
    "config_sha256",
    "image",
    "created_at",
    "notes",
    "physical_accuracy_evaluated",
]
EXPECTED_KEYS = [
    "n_poses",
    "duration_s",
    "path_length_m",
    "monotonic",
    "max_gap_s",
    "resets",
    "return_error_d3_m",
    "return_error_horizontal_m",
    "return_error_vertical_m",
    "heading_error_deg",
    "stationary_window_s",
    "map_points",
    "wall_separation_m",
    "notes",
]
# Face grid at 20 points per metre, endpoints included:
# floor/ceiling 121*101, X walls 101*51, Y walls 121*51.
SHARP_POINTS = 121 * 101 * 2 + 101 * 51 * 2 + 121 * 51 * 2
DOUBLED_POINTS = SHARP_POINTS + 101 * 51


def _load_tum(path):
    rows = []
    for line in path.read_text(encoding="utf-8").splitlines():
        text = line.strip()
        if not text or text.startswith("#"):
            continue
        parts = text.split()
        if len(parts) != 8:
            raise AssertionError("%s has a %d-column pose" % (path, len(parts)))
        rows.append([float(part) for part in parts])
    data = np.asarray(rows, dtype=np.float64)
    return data[:, 0], data[:, 1:4], data[:, 4:8]


def _load_xyz(path):
    """Read a binary xyz float32 cloud without the library reader."""

    with path.open("rb") as handle:
        fields = sizes = types = counts = None
        points = None
        kind = None
        while True:
            raw = handle.readline()
            if raw == b"":
                raise AssertionError("missing DATA in %s" % path)
            text = raw.decode("ascii").strip()
            if not text or text.startswith("#"):
                continue
            parts = text.split()
            key = parts[0]
            if key == "FIELDS":
                fields = parts[1:]
            elif key == "SIZE":
                sizes = [int(value) for value in parts[1:]]
            elif key == "TYPE":
                types = parts[1:]
            elif key == "COUNT":
                counts = [int(value) for value in parts[1:]]
            elif key == "POINTS":
                points = int(parts[1])
            elif key == "DATA":
                kind = parts[1]
                break
        payload = handle.read()
    if fields != ["x", "y", "z"] or sizes != [4, 4, 4] or types != ["F", "F", "F"]:
        raise AssertionError("fixture map is not xyz float32: %s" % path)
    if counts not in (None, [1, 1, 1]) or kind != "binary":
        raise AssertionError("fixture map.pcd is not binary COUNT-1: %s" % path)
    need = points * 12
    if len(payload) < need:
        raise AssertionError("truncated fixture map %s" % path)
    cloud = np.frombuffer(payload[:need], dtype="<f4").reshape(points, 3)
    return cloud


def _mean_yaw(quat):
    x = quat[:, 0]
    y = quat[:, 1]
    z = quat[:, 2]
    w = quat[:, 3]
    ang = np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return math.atan2(float(np.sin(ang).mean()), float(np.cos(ang).mean()))


def _wall_separation(x_values):
    values = np.asarray(x_values, dtype=np.float64).reshape(-1)
    if values.size < 500:
        return None
    rounded = np.round(values, 4)
    planes, counts = np.unique(rounded, return_counts=True)
    threshold = max(500, int(math.ceil(0.05 * float(values.size))))
    peaks = np.sort(planes[counts >= threshold])
    if peaks.size < 2:
        return None
    gaps = [float(gap) for gap in np.diff(peaks) if 0.01 <= float(gap) <= 0.5]
    if not gaps:
        return None
    return float(min(gaps))


def _measure(path):
    t, xyz, quat = _load_tum(path / "trajectory.tum")
    cloud = _load_xyz(path / "map.pcd")
    n = int(t.shape[0])
    duration = float(t[-1] - t[0])
    steps = np.diff(xyz, axis=0)
    path_length = float(np.linalg.norm(steps, axis=1).sum())
    dt = np.diff(t)
    monotonic = bool(np.all(dt > 0.0))
    positive = dt[dt > 0.0]
    max_gap = float(positive.max()) if positive.size else 0.0
    resets = int(np.sum(dt < 0.0))
    start = t <= float(t[0]) + WINDOW_S + EDGE_S
    end = t >= float(t[-1]) - WINDOW_S - EDGE_S
    delta = xyz[end].mean(axis=0) - xyz[start].mean(axis=0)
    heading = math.degrees(
        math.atan2(
            math.sin(_mean_yaw(quat[end]) - _mean_yaw(quat[start])),
            math.cos(_mean_yaw(quat[end]) - _mean_yaw(quat[start])),
        )
    )
    start_span = float(np.max(xyz[start].max(axis=0) - xyz[start].min(axis=0)))
    end_span = float(np.max(xyz[end].max(axis=0) - xyz[end].min(axis=0)))
    return {
        "n_poses": n,
        "duration_s": duration,
        "path_length_m": path_length,
        "monotonic": monotonic,
        "max_gap_s": max_gap,
        "resets": resets,
        "return_error_d3_m": float(np.linalg.norm(delta)),
        "return_error_horizontal_m": float(np.hypot(delta[0], delta[1])),
        "return_error_vertical_m": float(delta[2]),
        "heading_error_deg": float(heading),
        "map_points": int(cloud.shape[0]),
        "wall_separation_m": _wall_separation(cloud[:, 0]),
        "start_span_m": start_span,
        "end_span_m": end_span,
        "x": cloud[:, 0],
    }


def _near(test, name, got, want, tol):
    if want is None:
        test.assertIsNone(got, name)
        return
    if isinstance(want, bool):
        test.assertIs(got, want, name)
        return
    if isinstance(want, int) and not isinstance(want, bool):
        test.assertEqual(got, want, name)
        return
    test.assertLessEqual(abs(float(got) - float(want)), tol, "%s got %r want %r" % (name, got, want))


class FixtureTests(unittest.TestCase):
    def test_golden_fixtures_match_regeneration(self):
        proc = subprocess.run(
            [
                sys.executable,
                "-m",
                "spatio.io.make_fixtures",
                "--out",
                str(FIXTURES),
                "--check",
            ],
            cwd=str(REPO),
            capture_output=True,
            text=True,
        )
        self.assertEqual(proc.returncode, 0, proc.stdout + "\n" + proc.stderr)
        self.assertIn("fixtures match", proc.stdout)

    def test_expected_json_matches_independent_computation(self):
        names = sorted(path.name for path in FIXTURES.iterdir() if path.is_dir())
        self.assertEqual(
            names,
            [
                "gap_2s",
                "loop_15cm",
                "loop_5cm",
                "loop_heading_10deg",
                "loop_vertical_8cm",
                "map_doubled_wall_6cm",
                "map_sharp_wall",
                "pcd_formats",
                "stationary_60s",
                "timestamp_reset",
            ],
        )
        measured = {}
        for name in names:
            folder = FIXTURES / name
            got = _measure(folder)
            measured[name] = got
            expected = json.loads((folder / "EXPECTED.json").read_text(encoding="utf-8"))
            self.assertEqual(list(expected.keys()), EXPECTED_KEYS, name)
            for key in EXPECTED_KEYS:
                if key == "notes":
                    self.assertIsInstance(expected[key], str)
                    self.assertIn("no alignment", expected[key])
                    continue
                if key == "stationary_window_s":
                    _near(self, name + " window", expected[key], WINDOW_S, 0.0)
                    continue
                tol = 1e-6 if key == "heading_error_deg" else 1e-9
                _near(self, "%s %s" % (name, key), got[key], expected[key], tol)
        five = measured["loop_5cm"]
        _near(self, "5cm d3", five["return_error_d3_m"], 0.05, 1e-9)
        _near(self, "5cm h", five["return_error_horizontal_m"], 0.05, 1e-9)
        _near(self, "5cm v", five["return_error_vertical_m"], 0.0, 1e-9)
        _near(self, "5cm yaw", five["heading_error_deg"], 0.0, 1e-9)
        fifteen = measured["loop_15cm"]
        _near(self, "15cm d3", fifteen["return_error_d3_m"], 0.15, 1e-9)
        _near(self, "15cm h", fifteen["return_error_horizontal_m"], 0.15, 1e-9)
        vertical = measured["loop_vertical_8cm"]
        _near(self, "8cm d3", vertical["return_error_d3_m"], 0.08, 1e-9)
        _near(self, "8cm v", vertical["return_error_vertical_m"], 0.08, 1e-9)
        _near(self, "8cm h", vertical["return_error_horizontal_m"], 0.0, 1e-9)
        heading = measured["loop_heading_10deg"]
        _near(self, "10deg", heading["heading_error_deg"], 10.0, 1e-6)
        _near(self, "closed path", heading["path_length_m"], 16.0, 1e-6)
        hold = measured["stationary_60s"]
        self.assertEqual(hold["n_poses"], 601)
        _near(self, "60s", hold["duration_s"], 60.0, 1e-9)
        _near(self, "60s path", hold["path_length_m"], 0.0, 1e-9)
        _near(self, "60s return", hold["return_error_d3_m"], 0.0, 1e-9)
        reset = measured["timestamp_reset"]
        self.assertIs(reset["monotonic"], False)
        self.assertEqual(reset["resets"], 1)
        gap = measured["gap_2s"]
        self.assertIs(gap["monotonic"], True)
        self.assertEqual(gap["resets"], 0)
        self.assertGreater(gap["max_gap_s"], 2.0)
        self.assertLess(gap["max_gap_s"], 2.2)
        _near(self, "gap path", gap["path_length_m"], 16.0, 1e-6)
        _near(self, "gap return", gap["return_error_d3_m"], 0.0, 1e-9)
        sharp = measured["map_sharp_wall"]
        self.assertIsNone(sharp["wall_separation_m"])
        self.assertEqual(sharp["map_points"], SHARP_POINTS)
        self.assertGreater(sharp["map_points"], 10000)
        doubled = measured["map_doubled_wall_6cm"]
        self.assertEqual(doubled["map_points"], DOUBLED_POINTS)
        self.assertGreater(doubled["map_points"], sharp["map_points"])
        self.assertIsNotNone(doubled["wall_separation_m"])
        _near(self, "wall", doubled["wall_separation_m"], 0.06, 1e-6)
        formats = measured["pcd_formats"]
        self.assertEqual(formats["map_points"], 1000)
        self.assertIsNone(formats["wall_separation_m"])
        index = np.arange(1000, dtype=np.float32)
        np.testing.assert_array_equal(formats["x"], index * np.float32(0.125))
        for name, got in measured.items():
            if name == "timestamp_reset":
                continue
            self.assertLessEqual(got["start_span_m"], 1e-6, name)
            self.assertLessEqual(got["end_span_m"], 1e-6, name)
            self.assertIs(got["monotonic"], True, name)
            self.assertEqual(got["resets"], 0, name)

    def test_fixture_manifests_follow_shared_contract(self):
        total = 0
        folders = [path for path in FIXTURES.iterdir() if path.is_dir()]
        self.assertGreaterEqual(len(folders), 10)
        for folder in folders:
            names = {path.name for path in folder.iterdir() if path.name != ".DS_Store"}
            required = {"trajectory.tum", "map.pcd", "manifest.json", "EXPECTED.json"}
            if folder.name == "pcd_formats":
                required |= {
                    "points_ascii.pcd",
                    "points_binary.pcd",
                    "points_binary_compressed.pcd",
                }
            self.assertEqual(names, required, folder.name)
            for path in folder.iterdir():
                if not path.is_file() or path.name == ".DS_Store":
                    continue
                size = path.stat().st_size
                total += size
                self.assertLess(size, 1000000, str(path))
            raw = (folder / "manifest.json").read_text(encoding="utf-8")
            manifest = json.loads(raw)
            self.assertEqual(list(manifest.keys()), MANIFEST_KEYS, folder.name)
            self.assertEqual(manifest["run_id"], folder.name)
            self.assertEqual(manifest["source_recording"], "synthetic://" + folder.name)
            self.assertEqual(manifest["sensor"], "synthetic")
            self.assertEqual(manifest["fastlio_revision"], "none")
            self.assertEqual(manifest["config_file"], "spatio/io/synth.py")
            self.assertRegex(manifest["config_sha256"], r"^[0-9a-f]{64}$")
            self.assertEqual(manifest["image"], "none")
            self.assertEqual(manifest["created_at"], "2026-10-19T00:00:00Z")
            self.assertIn(LABEL, manifest["notes"])
            self.assertIn("not a FAST-LIO replay", manifest["notes"])
            self.assertIs(manifest["physical_accuracy_evaluated"], False)
            expected = json.loads((folder / "EXPECTED.json").read_text(encoding="utf-8"))
            self.assertIn(LABEL, (folder / "trajectory.tum").read_text(encoding="utf-8"))
            self.assertIsInstance(expected["notes"], str)
            self.assertTrue(expected["notes"])
        self.assertLess(total, 5000000)


if __name__ == "__main__":
    unittest.main()
