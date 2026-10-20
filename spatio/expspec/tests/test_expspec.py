"""Behaviour tests for the experiment spec, the run sheet, and the tape record."""

from __future__ import annotations

import copy
import hashlib
import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

from spatio.expspec.canonical import canonical_bytes
from spatio.expspec.constants import (
    PROPOSED_BANNER,
    REFERENCE_KEYS,
    REQUIRED_FORBIDDEN,
    RETURN_ERROR_SENTENCE,
)
from spatio.expspec.freeze import FreezeError, freeze_spec
from spatio.expspec.record import RecordError, record_physical_reference
from spatio.expspec.render import render_markdown
from spatio.expspec.runsheet import write_runsheet
from spatio.expspec.template import build_template
from spatio.expspec.validate import validate

ROOT = Path(__file__).resolve().parents[3]

# Synthetic. These strings are not a description of the real cart.
_BASE = {
    "schema": "spatio.experiment_spec/1",
    "spec_id": "m1-mid360-v1",
    "sensor": "mid360",
    "fastlio_revision": "2fffc57",
    "config_file": "mid360.yaml",
    "config_sha256": "0fba8e5d317235b964b84813c9f6019dabd4267f29bedfa03595f29c0cf80aa1",
    "route": {
        "description": "Synthetic room loop used only by the unit tests.",
        "duration_target_s": [180, 300],
        "intermediate_stops": ["stop A", "stop B"],
    },
    "reference_point": "synthetic fixed point on the cart frame",
    "start_marks": "synthetic tape cross for position and a second mark for heading",
    "frames": {
        "map": "camera_init",
        "body": "body",
        "lidar_to_imu": "Livox factory extrinsic [-0.011, -0.02329, 0.04412], not a cart mount",
        "cart_mount": "synthetic mount description",
    },
    "wall_patches": [
        {"name": "patch A", "description": "synthetic long wall", "box_map_frame": None},
        {"name": "patch B", "description": "synthetic end wall", "box_map_frame": None},
    ],
    "stationary_windows_s": {"start": 10, "end": 10},
    "acceptance": {
        "return_error_threshold_m": 0.10,
        "placement_tolerance_m": 0.01,
        "wall_p95_gate_m": 0.05,
        "revisit_separation_gate_m": 0.05,
        "runs_required": 3,
        "state": "proposed",
    },
    "forbidden": list(REQUIRED_FORBIDDEN),
}


def valid_spec():
    return copy.deepcopy(_BASE)


def _errors(spec):
    return [item for item in validate(spec) if item.level == "error"]


def _warnings(spec):
    return [item for item in validate(spec) if item.level == "warning"]


class ExpspecTests(unittest.TestCase):
    def _run_cli(self, args):
        return subprocess.run(
            [sys.executable, "-m", "spatio.expspec"] + list(args),
            cwd=str(ROOT),
            capture_output=True,
            text=True,
        )

    def _write_spec(self, directory, spec, name="spec.json"):
        path = Path(directory) / name
        path.write_text(json.dumps(spec, indent=2) + "\n", encoding="utf-8")
        return path

    def _freeze(self, directory, spec, name="SPEC-m1-mid360-v1.frozen.json"):
        path = Path(directory) / name
        digest, _issues = freeze_spec(spec, path)
        return path, digest

    def test_template_has_every_schema_key(self):
        spec = build_template()
        for key in (
            "schema",
            "spec_id",
            "sensor",
            "fastlio_revision",
            "config_file",
            "config_sha256",
            "route",
            "reference_point",
            "start_marks",
            "frames",
            "wall_patches",
            "stationary_windows_s",
            "acceptance",
            "forbidden",
        ):
            self.assertIn(key, spec)
        for key in ("description", "duration_target_s", "intermediate_stops"):
            self.assertIn(key, spec["route"])
        for key in ("map", "body", "lidar_to_imu", "cart_mount"):
            self.assertIn(key, spec["frames"])
        for key in ("start", "end"):
            self.assertIn(key, spec["stationary_windows_s"])
        for key in (
            "return_error_threshold_m",
            "placement_tolerance_m",
            "wall_p95_gate_m",
            "revisit_separation_gate_m",
            "runs_required",
            "state",
        ):
            self.assertIn(key, spec["acceptance"])
        self.assertGreaterEqual(len(spec["wall_patches"]), 2)
        for patch in spec["wall_patches"]:
            for key in ("name", "description", "box_map_frame"):
                self.assertIn(key, patch)
        self.assertEqual(spec["schema"], "spatio.experiment_spec/1")
        self.assertIn("end-onto-start alignment", spec["forbidden"])
        self.assertIn("post-hoc loop correction", spec["forbidden"])
        self.assertIn("smoothing away failures", spec["forbidden"])

    def test_validate_rejects_todo_placeholders(self):
        spec = valid_spec()
        spec["reference_point"] = "TODO(Trevor)"
        messages = [item.message for item in _errors(spec)]
        self.assertTrue(any("TODO" in message for message in messages))
        template_messages = [item.message for item in _errors(build_template())]
        self.assertTrue(any("TODO" in message for message in template_messages))

    def test_validate_rejects_missing_route(self):
        spec = valid_spec()
        del spec["route"]
        found = [item for item in _errors(spec) if item.path == "route"]
        self.assertTrue(found)
        self.assertIn("missing", found[0].message)

    def test_validate_rejects_runs_required_below_3(self):
        spec = valid_spec()
        spec["acceptance"]["runs_required"] = 2
        paths = [item.path for item in _errors(spec)]
        self.assertIn("acceptance.runs_required", paths)
        spec["acceptance"]["runs_required"] = 3
        self.assertEqual(_errors(spec), [])
        spec["acceptance"]["runs_required"] = True
        self.assertTrue(_errors(spec))

    def test_validate_requires_all_forbidden_phrases(self):
        for phrase in REQUIRED_FORBIDDEN:
            spec = valid_spec()
            spec["forbidden"] = [item for item in REQUIRED_FORBIDDEN if item != phrase]
            messages = [item.message for item in _errors(spec)]
            self.assertTrue(
                any(phrase in message for message in messages),
                phrase,
            )
        # A longer sentence that contains the phrase is not the phrase.
        spec = valid_spec()
        spec["forbidden"] = [
            "end-onto-start alignment is allowed",
            "post-hoc loop correction",
            "smoothing away failures",
        ]
        messages = [item.message for item in _errors(spec)]
        self.assertTrue(any("end-onto-start alignment" in message for message in messages))
        self.assertEqual(_errors(valid_spec()), [])

    def test_validate_rejects_short_stationary_window(self):
        for key in ("start", "end"):
            spec = valid_spec()
            spec["stationary_windows_s"][key] = 4
            paths = [item.path for item in _errors(spec)]
            self.assertIn("stationary_windows_s.{0}".format(key), paths)
        spec = valid_spec()
        spec["stationary_windows_s"]["start"] = 5
        self.assertEqual(_errors(spec), [])

    def test_validate_warns_threshold_differs_from_workplan(self):
        self.assertEqual(validate(valid_spec()), [])
        spec = valid_spec()
        spec["acceptance"]["return_error_threshold_m"] = 0.2
        warnings = _warnings(spec)
        self.assertEqual(_errors(spec), [])
        self.assertEqual(len(warnings), 1)
        self.assertEqual(warnings[0].path, "acceptance.return_error_threshold_m")
        self.assertIn("differs from the workplan", warnings[0].message)

    def test_validate_rejects_non_positive_threshold(self):
        for bad in (0, -0.1):
            spec = valid_spec()
            spec["acceptance"]["wall_p95_gate_m"] = bad
            levels = [item.level for item in validate(spec) if item.path == "acceptance.wall_p95_gate_m"]
            self.assertIn("error", levels)
            self.assertNotIn("warning", levels)

    def test_patch_box_validation(self):
        spec = valid_spec()
        spec["wall_patches"][0]["box_map_frame"] = {
            "center_m": [1.2, -3.4, 1.0],
            "normal_hint": [0.0, 1.0, 0.0],
            "half_extent_m": [0.5, 0.25, 0.5],
        }
        self.assertEqual(validate(spec), [])

        zero_hint = valid_spec()
        zero_hint["wall_patches"][0]["box_map_frame"] = {
            "center_m": [0.0, 0.0, 0.0],
            "normal_hint": [0.0, 0.0, 0.0],
            "half_extent_m": [0.5, 0.25, 0.5],
        }
        zero_messages = [item.message for item in _errors(zero_hint)]
        self.assertTrue(any("zero normal_hint" in message for message in zero_messages))

        flat = valid_spec()
        flat["wall_patches"][1]["box_map_frame"] = {
            "center_m": [1.0, 2.0, 3.0],
            "normal_hint": [1.0, 0.0, 0.0],
            "half_extent_m": [0.5, 0.0, 0.5],
        }
        flat_messages = [item.message for item in _errors(flat)]
        self.assertTrue(any("half extent must be positive" in message for message in flat_messages))

        negative = valid_spec()
        negative["wall_patches"][1]["box_map_frame"] = {
            "center_m": [1.0, 2.0, 3.0],
            "normal_hint": [0.0, 0.0, 1.0],
            "half_extent_m": [0.5, -0.1, 0.5],
        }
        self.assertTrue(_errors(negative))

    def test_render_one_page_line_limit(self):
        text = render_markdown(valid_spec())
        self.assertLessEqual(len(text.splitlines()), 70)
        self.assertTrue(text.endswith("\n"))
        self.assertNotIn("\r", text)
        crowded = valid_spec()
        crowded["route"]["description"] = "\n".join(["segment"] * 40)
        collapsed = render_markdown(crowded)
        self.assertLessEqual(len(collapsed.splitlines()), 70)
        self.assertIn("segment segment", collapsed)
        self.assertLessEqual(len(render_markdown(build_template()).splitlines()), 70)

    def test_render_marks_proposed_unless_approved(self):
        proposed = render_markdown(valid_spec())
        self.assertIn(PROPOSED_BANNER, proposed)
        approved_spec = valid_spec()
        approved_spec["acceptance"]["state"] = "approved_by_trevor"
        approved = render_markdown(approved_spec)
        self.assertNotIn(PROPOSED_BANNER, approved)
        self.assertIn("APPROVED by Trevor.", approved)

    def test_render_states_no_alignment_and_separate_heading(self):
        text = render_markdown(valid_spec())
        self.assertIn(RETURN_ERROR_SENTENCE, text)
        self.assertIn("fixed stationary windows", text)
        self.assertIn("no end-onto-start alignment", text)
        self.assertIn("heading is reported separately", text)

    def test_render_explains_each_frame(self):
        text = render_markdown(valid_spec())
        # Wrapping inserts newlines between words. The words still have to be there.
        flat = " ".join(text.split())
        for label in ("map:", "body:", "lidar_to_imu:", "cart_mount:"):
            self.assertIn(label, flat)
        self.assertIn("this run only", flat)
        self.assertIn("not the cart mount", flat)
        self.assertIn("not the floor-mark measurement", flat)
        self.assertIn("display transform is not a calibration", flat)

    def test_freeze_canonical_hash_stable_and_changes_on_edit(self):
        spec = valid_spec()
        flipped = _reverse_keys(spec)
        with tempfile.TemporaryDirectory() as tmp:
            first, digest_a = self._freeze(tmp, spec, "a.frozen.json")
            second, digest_b = self._freeze(tmp, flipped, "b.frozen.json")
            self.assertEqual(digest_a, digest_b)
            self.assertEqual(first.read_bytes(), second.read_bytes())
            raw = first.read_bytes()
            self.assertTrue(raw.endswith(b"\n"))
            self.assertNotIn(b"\r", raw)
            parsed = json.loads(raw.decode("utf-8"))
            self.assertEqual(list(parsed.keys()), sorted(parsed.keys()))
            self.assertEqual(list(parsed["acceptance"].keys()), sorted(parsed["acceptance"].keys()))
            self.assertEqual(hashlib.sha256(raw).hexdigest(), digest_a)
            sidecar = Path(str(first) + ".sha256").read_text(encoding="ascii").strip()
            self.assertEqual(sidecar, digest_a)
            edited = valid_spec()
            edited["sensor"] = "jt128"
            _third, digest_c = self._freeze(tmp, edited, "c.frozen.json")
            self.assertNotEqual(digest_a, digest_c)

    def test_freeze_blocked_by_validation_errors(self):
        spec = valid_spec()
        spec["reference_point"] = "TODO(Trevor)"
        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp) / "SPEC.frozen.json"
            with self.assertRaises(FreezeError):
                freeze_spec(spec, out)
            self.assertFalse(out.exists())
            self.assertFalse(Path(str(out) + ".sha256").exists())
            source = self._write_spec(tmp, spec)
            completed = self._run_cli(["freeze", str(source), "--out", str(out)])
            self.assertEqual(completed.returncode, 1)
            self.assertFalse(out.exists())
            self.assertIn("TODO", completed.stderr)

    def test_runsheet_has_blanks_for_each_run(self):
        spec = valid_spec()
        spec["route"]["intermediate_stops"] = ["stop A by the door", "stop B by the window"]
        with tempfile.TemporaryDirectory() as tmp:
            frozen, _digest = self._freeze(tmp, spec)
            out = Path(tmp) / "runsheet.md"
            text, _loaded = write_runsheet(frozen, 3, out)
            self.assertEqual(out.read_text(encoding="utf-8"), text)
            parts = text.split("## Run ")
            self.assertEqual(len(parts), 4)
            for index, part in enumerate(parts[1:], start=1):
                self.assertTrue(part.startswith("{0} of 3".format(index)))
                self.assertIn("Run id: ____________________", part)
                self.assertIn("Operator: ____________________", part)
                self.assertIn("Start clock: ____________________", part)
                self.assertIn("End clock: ____________________", part)
                self.assertIn("[ ] held still 10 s at start", part)
                self.assertIn("[ ] held still 10 s at end", part)
                self.assertIn("Start placement residual (cm): __________", part)
                self.assertIn("End placement residual (cm): __________", part)
                self.assertIn("Heading mark check (ok / off / not_checked): __________", part)
                self.assertIn("[ ] stop A by the door", part)
                self.assertIn("[ ] stop B by the window", part)
                self.assertIn("Notes: ____________________", part)
            self.assertIn("divide cm by 100", text)
            self.assertIn("null is not zero", text)

    def test_record_writes_valid_physical_reference(self):
        with tempfile.TemporaryDirectory() as tmp:
            frozen, digest = self._freeze(tmp, valid_spec())
            out = Path(tmp) / "physical_reference.json"
            result = record_physical_reference(
                spec_path=frozen,
                run_id="run-a",
                start_residual_m=0.004,
                end_residual_m=0.006,
                heading="ok",
                operator="Trevor",
                method="steel tape from the reference point to tape mark A",
                notes="synthetic test record",
                out_path=out,
                measured_at="2026-10-19T21:30:00-05:00",
            )
            self.assertTrue(result.within_tolerance)
            self.assertEqual(result.reasons, [])
            parsed = json.loads(out.read_text(encoding="utf-8"))
            self.assertEqual(sorted(parsed.keys()), sorted(REFERENCE_KEYS))
            self.assertEqual(parsed["schema"], "spatio.physical_reference/1")
            self.assertEqual(parsed["run_id"], "run-a")
            self.assertEqual(parsed["spec_id"], "m1-mid360-v1")
            self.assertEqual(parsed["spec_sha256"], digest)
            self.assertEqual(parsed["spec_sha256"], hashlib.sha256(frozen.read_bytes()).hexdigest())
            self.assertEqual(parsed["operator"], "Trevor")
            self.assertEqual(parsed["measured_at"], "2026-10-19T21:30:00-05:00")
            self.assertEqual(parsed["start_placement_residual_m"], 0.004)
            self.assertEqual(parsed["end_placement_residual_m"], 0.006)
            self.assertEqual(parsed["heading_check"], "ok")
            self.assertEqual(parsed["placement_tolerance_m"], 0.01)
            self.assertTrue(parsed["method"])
            self.assertEqual(parsed["notes"], "synthetic test record")
            self.assertEqual(out.read_bytes(), canonical_bytes(parsed))

    def test_record_rejects_negative_or_nan_residual(self):
        with tempfile.TemporaryDirectory() as tmp:
            frozen, _digest = self._freeze(tmp, valid_spec())
            for bad in (-0.01, float("nan"), float("inf"), float("-inf")):
                out = Path(tmp) / "physical_reference.json"
                out.write_text("sentinel", encoding="utf-8")
                with self.assertRaises(RecordError):
                    record_physical_reference(
                        spec_path=frozen,
                        run_id="run-a",
                        start_residual_m=bad,
                        end_residual_m=0.001,
                        heading="ok",
                        operator="Trevor",
                        method="tape",
                        notes="",
                        out_path=out,
                        measured_at="2026-10-19T21:30:00-05:00",
                    )
                self.assertEqual(out.read_text(encoding="utf-8"), "sentinel")

    def test_record_null_residual_allowed_and_flagged_not_within_tolerance(self):
        with tempfile.TemporaryDirectory() as tmp:
            frozen, _digest = self._freeze(tmp, valid_spec())
            out = Path(tmp) / "physical_reference.json"
            result = record_physical_reference(
                spec_path=frozen,
                run_id="run-a",
                start_residual_m=None,
                end_residual_m=0.004,
                heading="ok",
                operator="Trevor",
                method="tape",
                notes="",
                out_path=out,
            )
            self.assertFalse(result.within_tolerance)
            self.assertTrue(any("null" in reason for reason in result.reasons))
            parsed = json.loads(out.read_text(encoding="utf-8"))
            self.assertIsNone(parsed["start_placement_residual_m"])
            self.assertEqual(parsed["end_placement_residual_m"], 0.004)
            self.assertRegex(
                parsed["measured_at"],
                r"^\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2}[+-]\d{2}:\d{2}$",
            )
            cli_out = Path(tmp) / "from-cli.json"
            completed = self._run_cli([
                "record",
                "--spec", str(frozen),
                "--run-id", "run-a",
                "--start-residual-m", "null",
                "--end-residual-m", "0.004",
                "--heading", "ok",
                "--operator", "Trevor",
                "--method", "tape",
                "--out", str(cli_out),
            ])
            self.assertEqual(completed.returncode, 0, completed.stderr)
            self.assertIn("placement_within_tolerance: false", completed.stdout)
            self.assertIn("null", completed.stdout)
            cli_parsed = json.loads(cli_out.read_text(encoding="utf-8"))
            self.assertIsNone(cli_parsed["start_placement_residual_m"])

    def test_record_rejects_spec_changed_after_freeze(self):
        with tempfile.TemporaryDirectory() as tmp:
            frozen, _digest = self._freeze(tmp, valid_spec())
            original = frozen.read_text(encoding="utf-8")
            self.assertIn("mid360", original)
            frozen.write_text(original.replace("mid360", "mid361", 1), encoding="utf-8")
            out = Path(tmp) / "physical_reference.json"
            with self.assertRaises(RecordError) as caught:
                record_physical_reference(
                    spec_path=frozen,
                    run_id="run-a",
                    start_residual_m=0.004,
                    end_residual_m=0.006,
                    heading="ok",
                    operator="Trevor",
                    method="tape",
                    notes="",
                    out_path=out,
                    measured_at="2026-10-19T21:30:00-05:00",
                )
            self.assertIn("changed after freezing", str(caught.exception))
            self.assertFalse(out.exists())

    def test_record_heading_values_restricted(self):
        with tempfile.TemporaryDirectory() as tmp:
            frozen, _digest = self._freeze(tmp, valid_spec())
            for bad in ("yes", "OK", "good", "", "checked"):
                out = Path(tmp) / "bad.json"
                out.write_text("sentinel", encoding="utf-8")
                with self.assertRaises(RecordError):
                    record_physical_reference(
                        spec_path=frozen,
                        run_id="run-a",
                        start_residual_m=0.001,
                        end_residual_m=0.001,
                        heading=bad,
                        operator="Trevor",
                        method="tape",
                        notes="",
                        out_path=out,
                        measured_at="2026-10-19T21:30:00-05:00",
                    )
                self.assertEqual(out.read_text(encoding="utf-8"), "sentinel")
            for heading, within in (("ok", True), ("off", False), ("not_checked", False)):
                out = Path(tmp) / "heading-{0}.json".format(heading)
                result = record_physical_reference(
                    spec_path=frozen,
                    run_id="run-a",
                    start_residual_m=0.001,
                    end_residual_m=0.001,
                    heading=heading,
                    operator="Trevor",
                    method="tape",
                    notes="",
                    out_path=out,
                    measured_at="2026-10-19T21:30:00-05:00",
                )
                self.assertEqual(result.document["heading_check"], heading)
                self.assertEqual(result.within_tolerance, within)
                self.assertTrue(out.is_file())

    def test_record_copies_placement_tolerance_from_frozen_spec(self):
        spec = valid_spec()
        spec["acceptance"]["placement_tolerance_m"] = 0.02
        with tempfile.TemporaryDirectory() as tmp:
            frozen, _digest = self._freeze(tmp, spec)
            out = Path(tmp) / "physical_reference.json"
            result = record_physical_reference(
                spec_path=frozen,
                run_id="run-a",
                start_residual_m=0.015,
                end_residual_m=0.015,
                heading="ok",
                operator="Trevor",
                method="tape",
                notes="",
                out_path=out,
                measured_at="2026-10-19T21:30:00-05:00",
            )
            parsed = json.loads(out.read_text(encoding="utf-8"))
            self.assertEqual(parsed["placement_tolerance_m"], 0.02)
            self.assertTrue(result.within_tolerance)

    def test_record_flags_residual_above_tolerance(self):
        with tempfile.TemporaryDirectory() as tmp:
            frozen, _digest = self._freeze(tmp, valid_spec())
            out = Path(tmp) / "physical_reference.json"
            result = record_physical_reference(
                spec_path=frozen,
                run_id="run-a",
                start_residual_m=0.02,
                end_residual_m=0.001,
                heading="ok",
                operator="Trevor",
                method="tape",
                notes="",
                out_path=out,
                measured_at="2026-10-19T21:30:00-05:00",
            )
            self.assertTrue(out.is_file())
            self.assertFalse(result.within_tolerance)
            self.assertTrue(any("above" in reason for reason in result.reasons))

    def test_cli_exit_codes(self):
        with tempfile.TemporaryDirectory() as tmp:
            good = self._write_spec(tmp, valid_spec(), "good.json")
            okay = self._run_cli(["validate", str(good)])
            self.assertEqual(okay.returncode, 0, okay.stderr)
            self.assertIn("ok", okay.stdout)

            warned_spec = valid_spec()
            warned_spec["acceptance"]["placement_tolerance_m"] = 0.02
            warned = self._write_spec(tmp, warned_spec, "warned.json")
            warning = self._run_cli(["validate", str(warned)])
            self.assertEqual(warning.returncode, 0, warning.stderr)
            self.assertIn("WARNING", warning.stderr)
            self.assertIn("differs from the workplan", warning.stderr)

            bad = self._write_spec(tmp, build_template(), "template.json")
            failed = self._run_cli(["validate", str(bad)])
            self.assertEqual(failed.returncode, 1)
            self.assertIn("ERROR", failed.stderr)

            missing = self._run_cli(["validate", str(Path(tmp) / "nope.json")])
            self.assertEqual(missing.returncode, 1)

            frozen, _digest = self._freeze(tmp, valid_spec())
            rejected = self._run_cli([
                "record",
                "--spec", str(frozen),
                "--run-id", "run-a",
                "--start-residual-m", "-1",
                "--end-residual-m", "0.001",
                "--heading", "ok",
                "--operator", "Trevor",
                "--method", "tape",
                "--out", str(Path(tmp) / "nope-record.json"),
            ])
            self.assertEqual(rejected.returncode, 1)
            self.assertFalse((Path(tmp) / "nope-record.json").exists())

            nan_rejected = self._run_cli([
                "record",
                "--spec", str(frozen),
                "--run-id", "run-a",
                "--start-residual-m", "nan",
                "--end-residual-m", "0.001",
                "--heading", "ok",
                "--operator", "Trevor",
                "--method", "tape",
                "--out", str(Path(tmp) / "nan-record.json"),
            ])
            self.assertEqual(nan_rejected.returncode, 1)
            self.assertFalse((Path(tmp) / "nan-record.json").exists())


def _reverse_keys(value):
    if isinstance(value, dict):
        return {key: _reverse_keys(value[key]) for key in reversed(list(value.keys()))}
    if isinstance(value, list):
        return [_reverse_keys(item) for item in value]
    return value


if __name__ == "__main__":
    unittest.main()
