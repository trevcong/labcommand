"""The walk-day placement path with the real CLIs, no Docker.

process.sh stages 4-9 on a session whose trajectory/map stand in for a
replay: g5 `record` -> g2 `build --physical-reference` -> g2 `validate` ->
walkcheck -> wallpatch -> result -> series. This is the path a real scored
walk takes (the synthetic dry run deliberately never applies a placement)."""

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
from spatio.walkday.tests.test_result import write_recording

REPO = Path(__file__).resolve().parents[3]
SPEC_FILL = REPO / "spatio" / "walkday" / "synth" / "spec_fill.json"
REV = "2fffc570a25d0df172720bac034fbdb6a13d2162"


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

    @classmethod
    def tearDownClass(cls):
        shutil.rmtree(cls.tmp)

    def scored_run(self, run_id, start_m, end_m, heading="ok"):
        s = self.tmp / "sessions" / run_id
        (s / "logs").mkdir(parents=True)
        write_tum(s / "trajectory.tum", hold_and_square(offset=(0.04, 0.0, 0.0)))
        write_pcd_binary(s / "map.pcd", four_wall_room())
        ref = s / "logs" / "physical_reference.input.json"
        py("spatio.expspec", "record", "--spec", str(self.spec), "--run-id", run_id,
           "--start-residual-m", str(start_m), "--end-residual-m", str(end_m), "--heading", heading,
           "--operator", "test", "--method", "test", "--out", str(ref))
        rec = write_recording(self.tmp / "recordings" / run_id)   # its own, later start time
        py("spatio.manifest", "build", str(s), "--run-id", run_id, "--source-recording",
           str(rec), "--sensor", "test sensor", "--fastlio-revision", REV,
           "--config-file", str(self.config), "--image", "test-image", "--notes", "placement path test",
           "--physical-reference", str(ref))
        py("spatio.manifest", "validate", str(s))
        self.assertIn(py("spatio.walkcheck", str(s), ok=False).returncode, (0, 1))
        (s / "wallpatch").mkdir()
        py("spatio.wallpatch", "suggest", str(s / "map.pcd"), "--n", "2", "--out", str(s / "wallpatch" / "patches.json"))
        py("spatio.wallpatch", "measure", str(s), "--patches", str(s / "wallpatch" / "patches.json"),
           "--out", str(s / "wallpatch"))
        rc = py("spatio.walkday.result", str(s), ok=False).returncode
        return s, rc

    def series(self, sessions, name):
        out = self.tmp / "series" / name
        rc = py("spatio.series", "compare", *[str(s / "walk_result.json") for s in sessions], "--out", str(out),
                ok=False).returncode
        verdict = json.loads((out / "series.json").read_text())["verdict"]
        self.assertEqual(rc, VERDICT_EXIT[verdict])
        return verdict

    def test_measured_placements_flow_through_to_series_pass(self):
        runs = []
        for i in range(3):
            s, rc = self.scored_run(f"ok{i}", 0.003, 0.006)
            self.assertEqual(rc, 0)
            manifest = json.loads((s / "manifest.json").read_text())
            self.assertTrue(manifest["physical_accuracy_evaluated"])
            walk = json.loads((s / "walk_result.json").read_text())
            self.assertTrue(walk["physical_accuracy_evaluated"])
            self.assertAlmostEqual(walk["physical_reference"]["end_placement_residual_m"], 0.006)
            self.assertAlmostEqual(walk["physical_reference"]["placement_tolerance_m"], 0.01)
            runs.append(s)
        self.assertEqual(self.series(runs, "ok"), "PASS")

    def test_one_placement_over_tolerance_blocks_series_pass(self):
        runs = [self.scored_run(f"bad{i}", 0.003, 0.015 if i == 1 else 0.006)[0] for i in range(3)]
        report = json.loads((runs[1] / "walkday.json").read_text())
        self.assertFalse(report["placement"]["within_tolerance"])
        self.assertNotEqual(self.series(runs, "bad"), "PASS")

    def test_heading_not_checked_cannot_count(self):
        runs = [self.scored_run(f"nh{i}", 0.003, 0.006, heading="not_checked" if i == 0 else "ok")[0]
                for i in range(3)]
        self.assertNotEqual(self.series(runs, "nh"), "PASS")


if __name__ == "__main__":
    unittest.main()
