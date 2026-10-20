"""The walk-day placement path with the real CLIs, no Docker.

process.sh stages 4-9 on a session whose trajectory/map stand in for a
replay: g5 `record` -> g2 `build --physical-reference` -> g2 `validate` ->
walkcheck -> wallpatch -> result -> series.sh (spatio.series + the walk-day
gate). This is the path a real scored walk takes (the synthetic dry run
deliberately never applies a placement)."""

import json
import os
import shutil
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

from spatio.series.cli import VERDICT_EXIT
from spatio.walkcheck.synthetic import four_wall_room, hold_and_square, write_pcd_binary, write_tum
from spatio.walkday.tests.test_result import TEST_SETTINGS, walkcheck_args, write_gate, write_recording

REPO = Path(__file__).resolve().parents[3]
SPEC_FILL = REPO / "spatio" / "walkday" / "synth" / "spec_fill.json"
SERIES_SH = REPO / "spatio" / "walkday" / "series.sh"
REV = "2fffc570a25d0df172720bac034fbdb6a13d2162"
PRESELECTED = ["west wall", "south wall"]      # this test's spec names; nothing recording-specific


def py(*args, ok=True):
    proc = subprocess.run([sys.executable, "-m", *args], cwd=REPO, capture_output=True, text=True,
                          env=dict(os.environ, PYTHONPATH=str(REPO)), timeout=300)
    if ok and proc.returncode != 0:
        raise AssertionError(f"{args[:2]} exit {proc.returncode}: {proc.stdout}{proc.stderr}")
    return proc


class PlacementPathTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = Path(tempfile.mkdtemp())
        cls.config = cls.tmp / "walk_config.yaml"
        cls.config.write_text('common:\n  lid_topic: "/cart/lidar"\n  imu_topic: "/cart/imu"\n'
                              "mapping:\n  extrinsic_T: [ 0.0, 0.0, 0.0 ]\n")
        draft = cls.tmp / "spec.draft.json"
        py("spatio.walkday.spec_draft", "--config", str(cls.config), "--fastlio-revision", REV,
           "--fill", str(SPEC_FILL), "--out", str(draft))
        cls.spec = cls.tmp / "spec.frozen.json"
        py("spatio.expspec", "freeze", str(draft), "--out", str(cls.spec))
        # A second frozen spec whose wall patches are named, as Trevor's will be.
        fill = json.loads(SPEC_FILL.read_text())
        fill["wall_patches"] = [{"name": n, "description": "test: boxed on each run's map", "box_map_frame": None}
                                for n in PRESELECTED]
        (cls.tmp / "fill.named.json").write_text(json.dumps(fill))
        py("spatio.walkday.spec_draft", "--config", str(cls.config), "--fastlio-revision", REV,
           "--fill", str(cls.tmp / "fill.named.json"), "--out", str(cls.tmp / "spec.named.draft.json"))
        cls.named_spec = cls.tmp / "spec.named.frozen.json"
        py("spatio.expspec", "freeze", str(cls.tmp / "spec.named.draft.json"), "--out", str(cls.named_spec))

    @classmethod
    def tearDownClass(cls):
        shutil.rmtree(cls.tmp)

    def test_walk_spec_hold_is_15_s(self):
        # Trevor's decision (2026-10-20): hold 15 s at each end (was 10 s).
        # The synthetic dry-run spec states its own 12 s holds instead.
        from spatio.walkday.spec_draft import draft
        self.assertEqual(draft(str(self.config), REV)["stationary_windows_s"], {"start": 15, "end": 15})
        committed = json.loads((REPO / "spatio" / "walkday" / "spec" / "SPEC-mid360-walk.DRAFT.json").read_text())
        self.assertEqual(committed["stationary_windows_s"], {"start": 15, "end": 15})
        self.assertEqual(json.loads(self.spec.read_text())["stationary_windows_s"], {"start": 12, "end": 12})

    def scored_run(self, run_id, start_m, end_m, heading="ok", preselected=False):
        spec = self.named_spec if preselected else self.spec
        s = self.tmp / "sessions" / run_id
        (s / "logs").mkdir(parents=True)
        write_gate(s)                         # stage 2's input gate, clean
        write_tum(s / "trajectory.tum", hold_and_square(offset=(0.04, 0.0, 0.0)))
        write_pcd_binary(s / "map.pcd", four_wall_room())
        ref = s / "logs" / "physical_reference.input.json"
        py("spatio.expspec", "record", "--spec", str(spec), "--run-id", run_id,
           "--start-residual-m", str(start_m), "--end-residual-m", str(end_m), "--heading", heading,
           "--operator", "test", "--method", "test", "--out", str(ref))
        rec = write_recording(self.tmp / "recordings" / run_id)   # its own, later start time
        py("spatio.walkday.provenance", "write", str(s / "logs" / "provenance.json"), "--rec", str(rec),
           "--config", str(self.config), *[f"{k}={v}" for k, v in TEST_SETTINGS.items()])
        py("spatio.manifest", "build", str(s), "--run-id", run_id, "--source-recording",
           str(rec), "--sensor", "test sensor", "--fastlio-revision", REV,
           "--config-file", str(self.config), "--image", "test-image", "--notes", "placement path test",
           "--physical-reference", str(ref))
        py("spatio.manifest", "validate", str(s))
        self.assertIn(py("spatio.walkcheck", str(s), *walkcheck_args(TEST_SETTINGS), ok=False).returncode, (0, 1))
        (s / "wallpatch").mkdir()
        patches = s / "wallpatch" / "patches.json"
        py("spatio.wallpatch", "suggest", str(s / "map.pcd"), "--n", "2", "--out", str(patches))
        extra = []
        if preselected:   # Trevor boxes the spec's named patches on this run's map (process.sh --patches)
            doc = json.loads(patches.read_text())
            for p, name in zip(doc["patches"], PRESELECTED):
                p["name"] = name
            patches.write_text(json.dumps(doc))
            extra = ["--patch-origin", "preselected"]
        py("spatio.wallpatch", "measure", str(s), "--patches", str(patches), "--out", str(s / "wallpatch"))
        rc = py("spatio.walkday.result", str(s), "--spec", str(spec), *extra, ok=False).returncode
        return s, rc

    def series(self, sessions, name):
        proc = subprocess.run(["bash", str(SERIES_SH), *[s.name for s in sessions], "--sessions-root",
                               str(self.tmp / "sessions"), "--series-id", name],
                              capture_output=True, text=True, timeout=300)
        report = json.loads((self.tmp / "sessions" / "series" / name / "series.json").read_text())
        self.assertEqual(proc.returncode, VERDICT_EXIT[report["verdict"]], proc.stdout + proc.stderr)
        self.last = report
        return report["verdict"]

    def check_placement(self, s):
        manifest = json.loads((s / "manifest.json").read_text())
        self.assertTrue(manifest["physical_accuracy_evaluated"])
        walk = json.loads((s / "walk_result.json").read_text())
        self.assertTrue(walk["physical_accuracy_evaluated"])
        self.assertAlmostEqual(walk["physical_reference"]["end_placement_residual_m"], 0.006)
        self.assertAlmostEqual(walk["physical_reference"]["placement_tolerance_m"], 0.01)
        return walk

    def test_measured_placements_on_preselected_patches_flow_through_to_series_pass(self):
        runs = []
        for i in range(3):
            s, rc = self.scored_run(f"ok{i}", 0.003, 0.006, preselected=True)
            self.assertEqual(rc, 0)
            walk = self.check_placement(s)
            self.assertEqual(walk["wall_patches"]["verified"], PRESELECTED)
            runs.append(s)
        self.assertEqual(self.series(runs, "ok"), "PASS")
        self.assertEqual(self.last["counted_run_ids"], ["ok0", "ok1", "ok2"])

    def test_auto_suggested_patches_block_series_pass(self):
        # Spec correction (walkfix P1). This test used to expect PASS here:
        # three measured placements scored on auto-suggested wall patches. Those
        # are not Trevor's two preselected patches, so they never count.
        runs = []
        for i in range(3):
            s, rc = self.scored_run(f"auto{i}", 0.003, 0.006)
            self.assertEqual(rc, 2)
            self.assertEqual(self.check_placement(s)["wall_patches"]["origin"], "auto_suggested")
            runs.append(s)
        self.assertEqual(self.series(runs, "auto"), "FAIL")
        self.assertEqual(self.last["spatio_series_verdict"], "FAIL")

    def test_one_placement_over_tolerance_blocks_series_pass(self):
        # Preselected patches, so only the placement can be what blocks the series.
        runs = [self.scored_run(f"bad{i}", 0.003, 0.015 if i == 1 else 0.006, preselected=True)[0] for i in range(3)]
        report = json.loads((runs[1] / "walkday.json").read_text())
        self.assertFalse(report["placement"]["within_tolerance"])
        self.assertNotEqual(self.series(runs, "bad"), "PASS")
        self.assertNotEqual(self.last["spatio_series_verdict"], "PASS")

    def test_heading_not_checked_cannot_count(self):
        runs = [self.scored_run(f"nh{i}", 0.003, 0.006, heading="not_checked" if i == 0 else "ok",
                                preselected=True)[0] for i in range(3)]
        self.assertNotEqual(self.series(runs, "nh"), "PASS")
        self.assertNotEqual(self.last["spatio_series_verdict"], "PASS")


if __name__ == "__main__":
    unittest.main()
