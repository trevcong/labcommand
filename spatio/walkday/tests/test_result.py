"""Per-run result builder and the hand-off to spatio.series.

Sessions are built from walkcheck's synthetic helpers (square loop with
parked ends, four-wall room), scored by the real walk checker and wall-patch
tool, then turned into walk_result.json."""

import json
import os
import shutil
import tempfile
import unittest
from contextlib import redirect_stdout, redirect_stderr
from io import StringIO
from pathlib import Path

from spatio.series.cli import main as series_main
from spatio.series.validate import load_walk_result
from spatio.walkcheck.cli import main as walkcheck_main
from spatio.walkcheck.synthetic import doubled_wall, four_wall_room, hold_and_square, insert_time_gap, write_tum, write_pcd_binary
from spatio.wallpatch.cli import main as wallpatch_main
from spatio.walkday import result

CONFIG_SHA = "ab" * 32


def quiet(fn, *a):
    with redirect_stdout(StringIO()), redirect_stderr(StringIO()):
        return fn(*a)


def make_session(root, run_id, offset=(0.05, 0.0, 0.0), xyz=None, gap=False, evaluated=False,
                 reference=None, source=None, patches=True):
    s = Path(root) / run_id
    (s / "logs").mkdir(parents=True)
    rows = hold_and_square(offset=offset)
    if gap:
        rows = insert_time_gap(rows, after_t=20.0, extra_s=3.0)
    write_tum(s / "trajectory.tum", rows)
    write_pcd_binary(s / "map.pcd", four_wall_room() if xyz is None else xyz)
    manifest = {"run_id": run_id, "source_recording": source or str(Path(root) / "rec"),
                "sensor": "test", "fastlio_revision": "abc1234", "config_file": "/x.yaml",
                "config_sha256": CONFIG_SHA, "image": "img", "created_at": "2026-10-19T00:00:00Z",
                "notes": "test", "physical_accuracy_evaluated": evaluated}
    (s / "manifest.json").write_text(json.dumps(manifest))
    if reference is not None:
        (s / "physical_reference.json").write_text(json.dumps(reference))
    assert quiet(walkcheck_main, [str(s)]) in (0, 1, 2)
    if patches:
        (s / "wallpatch").mkdir()
        assert quiet(wallpatch_main, ["suggest", str(s / "map.pcd"), "--n", "2",
                                      "--out", str(s / "wallpatch" / "patches.json")]) == 0
        assert quiet(wallpatch_main, ["measure", str(s), "--patches", str(s / "wallpatch" / "patches.json"),
                                      "--out", str(s / "wallpatch")]) == 0
    return s


def ref(start=0.004, end=0.006, heading="ok"):
    return {"start_placement_residual_m": start, "end_placement_residual_m": end,
            "heading_check": heading, "placement_tolerance_m": 0.01}


def run(session):
    return quiet(result.main, [str(session)])


class ResultTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp()

    def tearDown(self):
        shutil.rmtree(self.tmp)

    def walk(self, s):
        return json.loads((s / "walk_result.json").read_text())

    def test_5cm_loop_passes_and_series_accepts_it(self):
        s = make_session(self.tmp, "r5")
        self.assertEqual(run(s), 0)
        w = self.walk(s)
        self.assertEqual(w["status"], "PASS")
        self.assertAlmostEqual(w["return_error"]["d3_m"], 0.05, places=3)
        self.assertFalse(w["physical_accuracy_evaluated"])
        self.assertIsNone(w["physical_reference"])
        load_walk_result(str(s / "walk_result.json"))           # series parser accepts it
        self.assertIn("PASS", (s / "walkday.md").read_text())

    def test_15cm_loop_fails(self):
        s = make_session(self.tmp, "r15", offset=(0.15, 0.0, 0.0))
        self.assertEqual(run(s), 1)
        w = self.walk(s)
        self.assertEqual(w["status"], "FAIL")
        self.assertIn("return_error", w["reasons"])

    def test_vertical_offset_cannot_hide(self):
        s = make_session(self.tmp, "rz", offset=(0.0, 0.0, 0.12))
        self.assertEqual(run(s), 1)

    def test_gap_is_not_evaluable_and_return_withheld(self):
        s = make_session(self.tmp, "rgap", gap=True)
        self.assertEqual(run(s), 2)
        w = self.walk(s)
        self.assertEqual(w["status"], "NOT_EVALUABLE")
        self.assertIsNone(w["return_error"]["d3_m"])
        load_walk_result(str(s / "walk_result.json"))

    def test_doubled_wall_fails_a_good_loop(self):
        room = four_wall_room()
        xyz = __import__("numpy").vstack([room, doubled_wall(separation_m=0.08)])
        s = make_session(self.tmp, "rdbl", xyz=xyz)
        code = run(s)
        w = self.walk(s)
        self.assertNotEqual(w["status"], "PASS")
        self.assertIn(code, (1, 2))

    def test_missing_wall_patches_is_not_evaluable(self):
        s = make_session(self.tmp, "rnp", patches=False)
        self.assertEqual(run(s), 2)
        self.assertIsNone(self.walk(s)["walls"]["doubled_wall_suspected"])

    def test_undetermined_patch_is_not_evaluable(self):
        s = make_session(self.tmp, "rund")
        wp = s / "wallpatch" / "wallpatch.json"
        data = json.loads(wp.read_text())
        data["patches"][0]["doubled"] = "undetermined"
        wp.write_text(json.dumps(data))
        self.assertEqual(run(s), 2)

    def test_doubled_patch_fails(self):
        s = make_session(self.tmp, "rdp")
        wp = s / "wallpatch" / "wallpatch.json"
        data = json.loads(wp.read_text())
        data["patches"][1]["doubled"] = "true"
        data["patches"][1]["separation_m"] = 0.06
        wp.write_text(json.dumps(data))
        self.assertEqual(run(s), 1)
        self.assertTrue(self.walk(s)["walls"]["doubled_wall_suspected"])

    def test_synthetic_source_never_claims_physical_accuracy(self):
        rec = Path(self.tmp) / "synthrec"
        rec.mkdir()
        (rec / "SYNTHETIC.json").write_text(json.dumps({"statement": "synthetic",
                                                         "ground_truth": {"true_return_error_3d_m": 0.05}}))
        s = make_session(self.tmp, "rsyn_bad", evaluated=True, reference=ref(), source=str(rec))
        self.assertEqual(run(s), 3)
        self.assertFalse((s / "walk_result.json").exists())
        s2 = make_session(self.tmp, "rsyn", source=str(rec))
        self.assertEqual(run(s2), 0)
        rep = json.loads((s2 / "walkday.json").read_text())
        self.assertTrue(rep["synthetic"])
        self.assertAlmostEqual(rep["synthetic_truth"]["estimated_minus_true_return_m"], 0.0, places=3)
        self.assertIn("SYNTHETIC", (s2 / "walkday.md").read_text())

    def test_stopped_run_has_report_but_no_walk_result(self):
        s = Path(self.tmp) / "rstop"
        (s / "logs").mkdir(parents=True)
        self.assertEqual(quiet(result.main, [str(s), "--stopped-at", "input-gate", "--reason", "B1 missing"]), 2)
        self.assertFalse((s / "walk_result.json").exists())
        self.assertIn("stopped at", (s / "walkday.md").read_text())

    def series(self, sessions):
        out = Path(self.tmp) / "series"
        files = [str(s / "walk_result.json") for s in sessions]
        self.assertEqual(quiet(series_main, ["compare"] + files + ["--out", str(out)]), 0)
        return json.loads((out / "series.json").read_text())["verdict"]

    def test_series_pass_needs_three_measured_placements(self):
        runs = [make_session(self.tmp, f"p{i}", evaluated=True, reference=ref()) for i in range(3)]
        for s in runs:
            self.assertEqual(run(s), 0)
        self.assertEqual(self.series(runs), "PASS")

    def test_series_rejects_placement_outside_tolerance(self):
        runs = [make_session(self.tmp, f"q{i}", evaluated=True, reference=ref(end=0.015 if i == 1 else 0.006))
                for i in range(3)]
        for s in runs:
            run(s)
        rep = json.loads((runs[1] / "walkday.json").read_text())
        self.assertFalse(rep["placement"]["within_tolerance"])
        self.assertNotEqual(self.series(runs), "PASS")

    def test_series_without_placements_is_replay_only(self):
        runs = [make_session(self.tmp, f"u{i}") for i in range(3)]
        for s in runs:
            run(s)
        self.assertEqual(self.series(runs), "REPLAY_ONLY")


if __name__ == "__main__":
    unittest.main()
