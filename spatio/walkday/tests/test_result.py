"""Per-run result builder and the hand-off to spatio.series.

Sessions are built from walkcheck's synthetic helpers (square loop with
parked ends, four-wall room), scored by the real walk checker and wall-patch
tool, then turned into walk_result.json."""

import hashlib
import itertools
import json
import os
import shutil
import subprocess
import tempfile
import unittest
from contextlib import redirect_stdout, redirect_stderr
from io import StringIO
from pathlib import Path

from spatio.series.cli import VERDICT_EXIT, main as series_main
from spatio.series.validate import load_walk_result
from spatio.walkcheck.cli import main as walkcheck_main
import numpy as np

from spatio.walkcheck.synthetic import (doubled_wall, four_wall_room, hold_and_square, insert_time_gap,
                                      vertical_wall, write_pcd_binary, write_tum)
from spatio.wallpatch.cli import main as wallpatch_main
from spatio.walkday import provenance, result, seriesgate

CONFIG_SHA = "ab" * 32
REPO = Path(__file__).resolve().parents[3]
SERIES_SH = REPO / "spatio" / "walkday" / "series.sh"
# Arbitrary recorder clock (2030), not taken from any recording. Each new
# recording starts 10 minutes after the previous one, in creation order.
BASE_NS = 1_900_000_000 * 10**9
_NEXT = itertools.count()
# Arbitrary walk-day settings for these tests: not walkday.conf's values and not
# fitted to any recording. Only their identity matters here.
TEST_SETTINGS = {"STATIONARY_WINDOW_S": "4", "MAX_GAP_S": "0.8", "MIN_PATH_LENGTH_M": "1.5",
                 "MIN_EXCURSION_M": "0.4", "MAX_ENDPOINT_SPREAD_M": "0.05", "MAX_ENDPOINT_SPEED_M_S": "0.1",
                 "MAX_HOLD_SPEED_M_S": "0.1", "MIN_HOLD_S": "6", "WALL_GATE_M": "0.05",
                 "WALL_GATE_STATE": "approved", "SUGGEST_PATCHES": "2", "BAGTIMING_SAMPLE_EVERY": "2",
                 "MAP_VOXEL_M": "0.04", "PLAYBACK_RATE": "0.5"}
PATCH_NAMES = ["east face", "north face"]      # a test spec's preselected patch names


def write_provenance(session, settings=TEST_SETTINGS, driver_sha=None, config_sha=CONFIG_SHA):
    """logs/provenance.json as process.sh stage 1 writes it."""
    driver = None if driver_sha is None else {"file": "/drv.yaml", "sha256": driver_sha,
                                              "topics_in_bag": ["/t"], "note": provenance.DRIVER_NOTE}
    (Path(session) / "logs" / "provenance.json").write_text(json.dumps({
        "schema": provenance.SCHEMA, "settings": provenance.settings_identity(settings),
        "fastlio_config": {"file": "/x.yaml", "sha256": config_sha}, "driver_config": driver}))


def write_spec(path, names=PATCH_NAMES, boxes=None):
    """A spec holding only what result.py reads: wall_patches. Returns (path, sha256)."""
    boxes = boxes or [None] * len(names)
    Path(path).write_text(json.dumps({"spec_id": "test", "wall_patches": [
        {"name": n, "description": "test", "box_map_frame": b} for n, b in zip(names, boxes)]}))
    return str(path), hashlib.sha256(Path(path).read_bytes()).hexdigest()


def write_recording(rec, starting_ns=None):
    """A recording folder with only what result.py reads: bag/metadata.yaml."""
    rec = Path(rec)
    (rec / "bag").mkdir(parents=True, exist_ok=True)
    if starting_ns is None:
        starting_ns = BASE_NS + next(_NEXT) * 600 * 10**9
    start = "" if starting_ns is False else (
        "  starting_time:\n    nanoseconds_since_epoch: %d\n" % starting_ns)
    (rec / "bag" / "metadata.yaml").write_text(
        "rosbag2_bagfile_information:\n  version: 9\n  storage_identifier: mcap\n" + start)
    return rec


def quiet(fn, *a):
    with redirect_stdout(StringIO()), redirect_stderr(StringIO()):
        return fn(*a)


def make_session(root, run_id, offset=(0.05, 0.0, 0.0), xyz=None, gap=False, evaluated=False,
                 reference=None, source=None, patches=True, boxes=None, patch_names=None,
                 settings=TEST_SETTINGS, driver_sha=None):
    s = Path(root) / run_id
    (s / "logs").mkdir(parents=True)
    if settings is not None:
        write_provenance(s, settings, driver_sha)
    rows = hold_and_square(offset=offset)
    if gap:
        rows = insert_time_gap(rows, after_t=20.0, extra_s=3.0)
    write_tum(s / "trajectory.tum", rows)
    write_pcd_binary(s / "map.pcd", four_wall_room() if xyz is None else xyz)
    if source is None:
        source = str(write_recording(Path(root) / f"{run_id}.rec"))
    manifest = {"run_id": run_id, "source_recording": source,
                "sensor": "test", "fastlio_revision": "abc1234", "config_file": "/x.yaml",
                "config_sha256": CONFIG_SHA, "image": "img", "created_at": "2026-10-19T00:00:00Z",
                "notes": "test", "physical_accuracy_evaluated": evaluated}
    (s / "manifest.json").write_text(json.dumps(manifest))
    if reference is not None:
        (s / "physical_reference.json").write_text(json.dumps(reference))
    assert quiet(walkcheck_main, [str(s)]) in (0, 1, 2)
    if boxes is not None:              # explicit patch boxes, as with process.sh --patches
        (s / "wallpatch").mkdir()
        digest = hashlib.sha256((s / "map.pcd").read_bytes()).hexdigest()
        (s / "wallpatch" / "patches.json").write_text(json.dumps({
            "schema": "spatio.wall_patches/1", "map_sha256": digest,
            "patches": [{"name": f"chosen-{i}", "box_map_frame": b} for i, b in enumerate(boxes)]}))
        assert quiet(wallpatch_main, ["measure", str(s), "--patches", str(s / "wallpatch" / "patches.json"),
                                      "--out", str(s / "wallpatch")]) == 0
    elif patches:
        (s / "wallpatch").mkdir()
        assert quiet(wallpatch_main, ["suggest", str(s / "map.pcd"), "--n", "2",
                                      "--out", str(s / "wallpatch" / "patches.json")]) == 0
        if patch_names is not None:    # boxes found on this map, named as the spec's preselected patches
            doc = json.loads((s / "wallpatch" / "patches.json").read_text())
            for p, name in zip(doc["patches"], patch_names):
                p["name"] = name
            (s / "wallpatch" / "patches.json").write_text(json.dumps(doc))
        assert quiet(wallpatch_main, ["measure", str(s), "--patches", str(s / "wallpatch" / "patches.json"),
                                      "--out", str(s / "wallpatch")]) == 0
    return s


def ref(start=0.004, end=0.006, heading="ok", spec_sha=None):
    out = {"start_placement_residual_m": start, "end_placement_residual_m": end,
           "heading_check": heading, "placement_tolerance_m": 0.01}
    if spec_sha is not None:
        out["spec_sha256"] = spec_sha
    return out


def run(session, *extra):
    return quiet(result.main, [str(session), *extra])


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

    def test_doubled_wall_in_a_chosen_patch_fails_a_good_loop(self):
        # Rewritten (task I): doubled walls are decided on measured patches, as the
        # workplan defines them. A patch box over an 8 cm doubled wall must FAIL a
        # loop whose return error alone would pass.
        xyz = np.vstack([four_wall_room(), doubled_wall(separation_m=0.08)])
        box = {"center_m": [1.04, 1.5, 1.1], "normal_hint": [1.0, 0.0, 0.0], "half_extent_m": [0.5, 0.25, 0.5]}
        s = make_session(self.tmp, "rdbl", xyz=xyz, boxes=[box])
        self.assertEqual(run(s), 1)
        w = self.walk(s)
        self.assertEqual(w["status"], "FAIL")
        self.assertTrue(w["walls"]["doubled_wall_suspected"])

    def test_solid_pillar_is_a_warning_not_a_failure(self):
        # Since fd3cf19 the whole-map check reports the two faces of a 0.5 m pillar
        # as a ~50 cm doubled wall. That stays visible as a WARNING; patches decide.
        rng = np.random.default_rng(2)
        pillar = [vertical_wall(2.25, "x", 1500, 1.75, 2.25, 0, 2.4, 0.004, rng),
                  vertical_wall(2.75, "x", 1500, 1.75, 2.25, 0, 2.4, 0.004, rng),
                  vertical_wall(1.75, "y", 1500, 2.25, 2.75, 0, 2.4, 0.004, rng),
                  vertical_wall(2.25, "y", 1500, 2.25, 2.75, 0, 2.4, 0.004, rng)]
        s = make_session(self.tmp, "rpillar", xyz=np.vstack([four_wall_room()] + pillar))
        wc = json.loads((s / "walkcheck.json").read_text())
        self.assertTrue(wc["walls"]["doubled_wall"])          # the whole-map flag fires
        self.assertEqual(run(s), 0)
        self.assertFalse(self.walk(s)["walls"]["doubled_wall_suspected"])
        report = json.loads((s / "walkday.json").read_text())
        self.assertTrue(report["walls"]["whole_map_doubled_flag"])
        self.assertTrue(any("cannot tell a ghost wall" in x for x in report["walls"]["warnings"]))
        self.assertIn("WARNING", (s / "walkday.md").read_text())

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
        rec = write_recording(Path(self.tmp) / "synthrec")
        (rec / "SYNTHETIC.json").write_text(json.dumps({"statement": "synthetic",
                                                         "ground_truth": {"true_return_error_3d_m": 0.05}}))
        s = make_session(self.tmp, "rsyn_bad", evaluated=True, reference=ref(), source=str(rec))
        self.assertEqual(run(s), 3)
        with self.assertRaisesRegex(result.ResultError, "synthetic recording"):
            result.build(str(s), 0.05, "approved", "test")
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

    def test_recorded_at_comes_from_the_recording_metadata(self):
        rec = write_recording(Path(self.tmp) / "timed.rec", starting_ns=1_950_000_000_123_456_789)
        s = make_session(self.tmp, "rtime", source=str(rec))
        self.assertEqual(run(s), 0)
        want = "2031-10-17T10:40:00.123456+00:00"      # 1950000000 s, never processing time
        self.assertEqual(self.walk(s)["recorded_at"], want)
        report = json.loads((s / "walkday.json").read_text())
        self.assertEqual(report["recorded_at"], want)
        self.assertNotEqual(report["created_at"][:4], "2031")
        self.assertEqual(load_walk_result(str(s / "walk_result.json")).recorded_at.isoformat(), want)

    def test_missing_recording_start_time_is_refused(self):
        for name, rec in (("rnostart", write_recording(Path(self.tmp) / "nostart.rec", starting_ns=False)),
                          ("rnorec", Path(self.tmp) / "never-copied")):
            s = make_session(self.tmp, name, source=str(rec))
            self.assertEqual(run(s), 3)
            self.assertFalse((s / "walk_result.json").exists())
            with self.assertRaisesRegex(result.ResultError, "recording start time"):
                result.build(str(s), 0.05, "approved", "test")

    def series(self, sessions):
        # The exit code must be spatio.series' code for the verdict it wrote.
        out = Path(self.tmp) / "series"
        files = [str(s / "walk_result.json") for s in sessions]
        rc = quiet(series_main, ["compare"] + files + ["--out", str(out)])
        verdict = json.loads((out / "series.json").read_text())["verdict"]
        self.assertEqual(rc, VERDICT_EXIT[verdict])
        return verdict

    def series_sh(self, sessions, series_id):
        proc = subprocess.run(["bash", str(SERIES_SH), *[s.name for s in sessions],
                               "--sessions-root", self.tmp, "--series-id", series_id],
                              capture_output=True, text=True, timeout=120)
        path = Path(self.tmp) / "series" / series_id / "series.json"
        verdict = json.loads(path.read_text())["verdict"] if path.exists() else None
        return proc.returncode, verdict, proc.stdout + proc.stderr

    def preselected(self, run_id, spec=None, **kw):
        """A physically referenced run scored on the spec's preselected patches."""
        spec, sha = spec or write_spec(Path(self.tmp) / "spec.json")
        s = make_session(self.tmp, run_id, evaluated=True, reference=ref(spec_sha=sha),
                         patch_names=PATCH_NAMES, **kw)
        rc = run(s, "--patch-origin", "preselected", "--spec", spec)
        return s, rc

    def test_series_sh_exit_codes_are_spatio_series_codes(self):
        # Spec correction (walkfix P1): runs now use the spec's preselected
        # patches, as a scored walk must; auto-suggested patches can no longer
        # PASS a series (test_auto_suggested_patches_never_count_through_series_sh).
        good, far = {}, {"offset": (0.15, 0.0, 0.0)}
        cases = {
            "pass": ([good] * 3, True, "PASS"),
            "fail": ([good, far, good], True, "FAIL"),
            "short": ([good] * 2, True, "INCOMPLETE"),
            "replay": ([{}] * 3, False, "REPLAY_ONLY"),
        }
        for name, (kwargs, pre, verdict) in cases.items():
            if pre:
                runs = [self.preselected(f"sh-{name}{i}", **kw)[0] for i, kw in enumerate(kwargs)]
            else:
                runs = [make_session(self.tmp, f"sh-{name}{i}", **kw) for i, kw in enumerate(kwargs)]
                for s in runs:
                    run(s)
            rc, got, out = self.series_sh(runs, f"sh-{name}")
            self.assertEqual(got, verdict, out)
            self.assertEqual(rc, VERDICT_EXIT[verdict], out)
            self.assertIn(f"SERIES sh-{name}: {verdict}", out)

    def final_and_staged(self, series_id):
        out = Path(self.tmp) / "series" / series_id
        return (json.loads((out / "series.json").read_text()),
                json.loads((out / "spatio_series" / "series.json").read_text()))

    def test_auto_suggested_patches_never_count_through_series_sh(self):
        # Review P1 (result.py:204): three physically referenced 5 cm loops on
        # auto-suggested patches. spatio.series alone says PASS; walk day must not.
        runs = [make_session(self.tmp, f"auto{i}", evaluated=True, reference=ref()) for i in range(3)]
        for s in runs:
            self.assertEqual(run(s), 0)
            self.assertEqual(self.walk(s)["wall_patches"]["origin"], "auto_suggested")
        rc, verdict, out = self.series_sh(runs, "auto")
        self.assertEqual(verdict, "FAIL", out)
        self.assertEqual(rc, VERDICT_EXIT["FAIL"])
        final, staged = self.final_and_staged("auto")
        self.assertEqual(staged["verdict"], "PASS")
        self.assertEqual(final["spatio_series_verdict"], "PASS")
        self.assertEqual(final["counted_run_ids"], [])
        for r in final["runs"]:
            self.assertFalse(r["counts_toward_series"])
            self.assertTrue(any("auto_suggested" in b for b in r["count_blockers"]))
        self.assertIn("spatio.series alone said PASS", (Path(self.tmp) / "series" / "auto" / "series.md").read_text())

    def test_preselected_patch_not_verified_blocks_series(self):
        # One spec patch missing from the measured set: the run itself still
        # passes on the patches it has, but it cannot count.
        spec = write_spec(Path(self.tmp) / "spec.json")
        runs = [self.preselected(f"pv{i}", spec=spec)[0] for i in range(2)]
        s = make_session(self.tmp, "pv2", evaluated=True, reference=ref(spec_sha=spec[1]),
                         patch_names=[PATCH_NAMES[0], "some other wall"])
        self.assertEqual(run(s, "--patch-origin", "preselected", "--spec", spec[0]), 0)
        wp = self.walk(s)["wall_patches"]
        self.assertEqual(wp["verified"], [PATCH_NAMES[0]])
        self.assertIn("not verified: north face", "; ".join(result.acceptance_blockers(wp)))
        rc, verdict, out = self.series_sh(runs + [s], "pv")
        self.assertEqual(verdict, "FAIL", out)
        self.assertEqual(self.final_and_staged("pv")[1]["verdict"], "PASS")

    def test_patch_provenance_rules(self):
        good = {"status": "MEASURED", "doubled": "false", "gate_p95_pass": True, "gate_separation_pass": True,
                "box_map_frame": {"center_m": [1, 2, 1], "normal_hint": [1, 0, 0], "half_extent_m": [1, .2, 1]}}
        wp = {"patches": [dict(good, name="A"), dict(good, name="B")]}
        spec, sha = write_spec(Path(self.tmp) / "s.json", ["A", "B"])
        out = result.patch_provenance("preselected", spec, {"spec_sha256": sha}, wp, "/none")
        self.assertEqual(out["verified"], ["A", "B"])
        self.assertEqual(result.acceptance_blockers(out), [])
        # The same patches, declared auto-suggested, never count.
        self.assertTrue(result.acceptance_blockers(dict(out, origin="auto_suggested")))
        # A spec that is not the one the physical reference names is contradictory input.
        with self.assertRaisesRegex(result.ResultError, "not the spec the physical reference names"):
            result.patch_provenance("preselected", spec, {"spec_sha256": "0" * 64}, wp, "/none")
        # Two measured patches with one name cannot verify it.
        dup = {"patches": [dict(good, name="A"), dict(good, name="A"), dict(good, name="B")]}
        self.assertEqual(result.patch_provenance("preselected", spec, None, dup, "/none")["verified"], ["B"])
        # A box frozen in the spec must be the box measured (same axis, other length: still equal).
        box = {"center_m": [1, 2, 1], "normal_hint": [2, 0, 0], "half_extent_m": [1, .2, 1]}
        spec2, _ = write_spec(Path(self.tmp) / "s2.json", ["A", "B"], [box, dict(box, center_m=[1, 2.5, 1])])
        self.assertEqual(result.patch_provenance("preselected", spec2, None, wp, "/none")["verified"], ["A"])
        # One named patch is not two; a doubled or gate-failing patch is not verified.
        spec1, _ = write_spec(Path(self.tmp) / "s1.json", ["A"])
        self.assertTrue(result.acceptance_blockers(result.patch_provenance("preselected", spec1, None, wp, "/none")))
        bad = {"patches": [dict(good, name="A", doubled="true"), dict(good, name="B", gate_p95_pass=False)]}
        self.assertEqual(result.patch_provenance("preselected", spec, None, bad, "/none")["verified"], [])
        # No provenance at all (an older walk_result) never counts.
        self.assertTrue(result.acceptance_blockers(None))

    def test_mixed_settings_make_the_series_not_evaluable(self):
        # Review P2 (result.py:216): a changed map voxel (or hold limit, ...) starts a new series.
        changed = dict(TEST_SETTINGS, MAP_VOXEL_M="0.05")
        runs = [self.preselected(f"ms{i}", settings=changed if i == 2 else TEST_SETTINGS)[0] for i in range(3)]
        self.assertNotEqual(self.walk(runs[0])["walkday_settings"]["sha256"],
                            self.walk(runs[2])["walkday_settings"]["sha256"])
        rc, verdict, out = self.series_sh(runs, "ms")
        self.assertEqual((verdict, rc), ("NOT_EVALUABLE", VERDICT_EXIT["NOT_EVALUABLE"]), out)
        final, staged = self.final_and_staged("ms")
        self.assertEqual(staged["verdict"], "PASS")
        self.assertIn("MAP_VOXEL_M", " ".join(final["reasons"]))
        # Same values written differently are the same identity.
        same = dict(TEST_SETTINGS, PLAYBACK_RATE="0.50", MAX_HOLD_SPEED_M_S="0.10")
        self.assertEqual(provenance.settings_identity(same), provenance.settings_identity(TEST_SETTINGS))
        for key in ("MAX_HOLD_SPEED_M_S", "MIN_HOLD_S", "MAX_ENDPOINT_SPREAD_M", "MAX_ENDPOINT_SPEED_M_S",
                    "PLAYBACK_RATE", "MAP_VOXEL_M"):
            self.assertNotEqual(provenance.settings_identity(dict(TEST_SETTINGS, **{key: "9"}))["sha256"],
                                provenance.settings_identity(TEST_SETTINGS)["sha256"], key)

    def test_missing_settings_or_mixed_driver_config_make_the_series_not_evaluable(self):
        runs = [self.preselected(f"md{i}", settings=None if i == 1 else TEST_SETTINGS)[0] for i in range(3)]
        self.assertIsNone(self.walk(runs[1])["walkday_settings"])
        rc, verdict, out = self.series_sh(runs, "md")
        self.assertEqual(verdict, "NOT_EVALUABLE", out)
        runs = [self.preselected(f"dc{i}", driver_sha=("c" if i == 0 else "d") * 64)[0] for i in range(3)]
        self.assertEqual(self.walk(runs[0])["driver_config_sha256"], "c" * 64)
        rc, verdict, out = self.series_sh(runs, "dc")
        self.assertEqual(verdict, "NOT_EVALUABLE", out)
        self.assertIn("driver_config_sha256 values differ", " ".join(self.final_and_staged("dc")[0]["reasons"]))

    def test_config_change_reasons_survive_a_settings_change(self):
        # spatio.series R1 (FAST-LIO config differs) and G1 (settings differ) together: both reasons stay.
        runs = [self.preselected(f"cs{i}", settings=dict(TEST_SETTINGS, MIN_HOLD_S="7") if i == 2 else TEST_SETTINGS)[0]
                for i in range(3)]
        w = json.loads((runs[2] / "walk_result.json").read_text())
        w["config_sha256"] = "cd" * 32
        (runs[2] / "walk_result.json").write_text(json.dumps(w))
        rc, verdict, out = self.series_sh(runs, "cs")
        self.assertEqual(verdict, "NOT_EVALUABLE", out)
        reasons = " ".join(self.final_and_staged("cs")[0]["reasons"])
        self.assertIn("config_sha256 values differ", reasons)
        self.assertIn("MIN_HOLD_S", reasons)

    def test_tampered_settings_identity_is_refused(self):
        s = make_session(self.tmp, "tamper")
        prov = json.loads((s / "logs" / "provenance.json").read_text())
        prov["settings"]["values"]["MAP_VOXEL_M"] = 0.5          # sha256 left as it was
        (s / "logs" / "provenance.json").write_text(json.dumps(prov))
        self.assertEqual(run(s), 3)
        self.assertFalse((s / "walk_result.json").exists())
        self.assertFalse(seriesgate.check_identity(prov["settings"]))

    def test_series_sh_rejects_tied_recording_times(self):
        runs = []
        for i in range(3):
            rec = write_recording(Path(self.tmp) / f"tie{i}.rec", starting_ns=BASE_NS - 10**12)
            runs.append(make_session(self.tmp, f"tie{i}", evaluated=True, reference=ref(), source=str(rec)))
            self.assertEqual(run(runs[-1]), 0)
        rc, verdict, out = self.series_sh(runs, "ties")
        self.assertEqual(rc, 1, out)
        self.assertIsNone(verdict)
        self.assertIn("recorded_at", out)

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
