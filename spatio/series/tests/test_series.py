"""Series verdict tests. Each required name is one method. Inputs are synthetic."""

from __future__ import annotations

import json
import os
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

from spatio.series.report import build_series, explain_text, render_markdown
from spatio.series.validate import InputRejected, load_walk_result

ROOT = Path(__file__).resolve().parents[3]
GOLDEN = Path(__file__).resolve().parent / "golden"
CONFIG = "0123456789abcdef" * 4
OTHER_CONFIG = "b" * 64
REV = "2fffc57"
REPLAY = "replay demonstrated; physical accuracy not evaluated"
TREVOR = "Other failed attempts do not void this series (workplan rule; Trevor to confirm)"
CONFIG_CHANGE = "a changed configuration starts a new series"


def make_walk(run_id="run-a", **overrides):
    data = {
        "schema": "spatio.walk_result/1",
        "run_id": run_id,
        "status": "PASS",
        "reasons": [],
        "physical_accuracy_evaluated": True,
        "fastlio_revision": REV,
        "config_sha256": CONFIG,
        "duration_s": 210.0,
        "path_length_m": 42.0,
        "return_error": {
            "d3_m": 0.05,
            "horizontal_m": 0.03,
            "vertical_m": 0.04,
            "heading_deg": 0.8,
            "threshold_m": 0.10,
            "start_window_s": [0.0, 10.0],
            "end_window_s": [200.0, 210.0],
        },
        "drift_pct_of_path": 0.12,
        "validity": {
            "timestamps_monotonic": True,
            "resets": 0,
            "max_gap_s": 0.11,
            "moving": True,
        },
        "walls": {
            "p95_point_to_plane_m": 0.031,
            "doubled_wall_suspected": False,
            "gate_m": 0.05,
            "gate_state": "proposed",
        },
        "physical_reference": {
            "start_placement_residual_m": 0.004,
            "end_placement_residual_m": 0.006,
            "heading_check": "ok",
            "placement_tolerance_m": 0.01,
        },
    }
    for key, value in overrides.items():
        nested = key in ("return_error", "validity", "walls", "physical_reference")
        if nested and isinstance(value, dict) and isinstance(data.get(key), dict):
            merged = dict(data[key])
            merged.update(value)
            data[key] = merged
        else:
            data[key] = value
    return data


def make_replay(run_id, **overrides):
    return make_walk(
        run_id,
        physical_accuracy_evaluated=False,
        physical_reference=None,
        accuracy_label=REPLAY,
        **overrides,
    )


def make_return_fail(run_id, **overrides):
    base = {
        "status": "FAIL",
        "reasons": ["3D return error is not under 0.10 m"],
        "return_error": {"d3_m": 0.13, "horizontal_m": 0.12, "vertical_m": 0.05},
    }
    base.update(overrides)
    return make_walk(run_id, **base)


def golden_walks():
    first = make_walk(
        "g-pass-1",
        duration_s=210.0,
        path_length_m=42.0,
        drift_pct_of_path=0.12,
        return_error={
            "d3_m": 0.05,
            "horizontal_m": 0.03,
            "vertical_m": 0.04,
            "heading_deg": 0.4,
            "end_window_s": [200.0, 210.0],
        },
        validity={"max_gap_s": 0.11},
        walls={"p95_point_to_plane_m": 0.031},
        physical_reference={
            "start_placement_residual_m": 0.004,
            "end_placement_residual_m": 0.006,
        },
    )
    second = make_walk(
        "g-pass-2",
        duration_s=198.5,
        path_length_m=38.5,
        drift_pct_of_path=0.16,
        return_error={
            "d3_m": 0.06,
            "horizontal_m": 0.036,
            "vertical_m": 0.048,
            "heading_deg": 0.8,
            "end_window_s": [188.5, 198.5],
        },
        validity={"max_gap_s": 0.10},
        walls={"p95_point_to_plane_m": 0.028},
        physical_reference={
            "start_placement_residual_m": 0.003,
            "end_placement_residual_m": 0.005,
        },
    )
    third = make_walk(
        "g-pass-3",
        duration_s=225.0,
        path_length_m=45.0,
        drift_pct_of_path=0.07,
        return_error={
            "d3_m": 0.03,
            "horizontal_m": 0.018,
            "vertical_m": 0.024,
            "heading_deg": 1.1,
            "end_window_s": [215.0, 225.0],
        },
        validity={"max_gap_s": 0.09},
        walls={"p95_point_to_plane_m": 0.022},
        physical_reference={
            "start_placement_residual_m": 0.002,
            "end_placement_residual_m": 0.004,
        },
    )
    return [first, second, third]


def _sentence_lines(text):
    return [line for line in text.splitlines() if line.strip()]


def _table_rows(markdown):
    rows = []
    for line in markdown.splitlines():
        if not line.startswith("|"):
            continue
        cells = [cell.strip() for cell in line.strip().strip("|").split("|")]
        if not cells or cells[0] == "attempt" or cells[0].startswith("---"):
            continue
        rows.append(cells)
    return rows


class SeriesTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)

    def load(self, data, name="walk.json"):
        path = os.path.join(self.tmp.name, name)
        with open(path, "w", encoding="utf-8") as handle:
            json.dump(data, handle)
        return load_walk_result(path)

    def series_of(self, datas, series_id="series-test"):
        walks = []
        for index, data in enumerate(datas):
            walks.append(self.load(data, "run-%s.json" % index))
        return build_series(walks, series_id)

    def assert_rejected(self, data, fragment, name="bad.json"):
        with self.assertRaises(InputRejected) as caught:
            self.load(data, name)
        text = str(caught.exception)
        self.assertIn(name, text)
        self.assertIn(fragment, text)

    def test_three_counting_runs_pass_and_names_them(self):
        report = self.series_of(golden_walks(), "golden-pass")
        self.assertEqual(report["verdict"], "PASS")
        self.assertEqual(report["counted_run_ids"], ["g-pass-1", "g-pass-2", "g-pass-3"])
        self.assertEqual(report["retained_failed"], [])
        self.assertNotIn(TREVOR, report["reasons"])
        four = [
            make_walk("a"),
            make_walk("b"),
            make_walk("c"),
            make_walk("d"),
        ]
        longer = self.series_of(four, "four")
        self.assertEqual(longer["verdict"], "PASS")
        self.assertEqual(longer["counted_run_ids"], ["a", "b", "c"])
        self.assertTrue(longer["runs"][3]["counts_toward_series"])
        self.assertNotIn(TREVOR, longer["reasons"])

    def test_fail_in_window_breaks_series(self):
        report = self.series_of(
            [
                make_walk("a"),
                make_walk("b"),
                make_return_fail("c"),
                make_walk("d"),
            ]
        )
        self.assertEqual(report["verdict"], "FAIL")
        self.assertEqual(report["counted_run_ids"], [])
        flags = [run["counts_toward_series"] for run in report["runs"]]
        self.assertEqual(flags, [True, True, False, True])
        self.assertIn("c", report["retained_failed"])
        self.assertEqual(report["reasons"][0], "No three consecutive attempts count toward the series.")

    def test_pass_with_other_failed_attempt_reports_retained_and_trevor_line(self):
        report = self.series_of(
            [
                make_return_fail("failed-early"),
                make_walk("pass-1"),
                make_walk("pass-2"),
                make_walk("pass-3"),
            ]
        )
        markdown = render_markdown(report)
        self.assertEqual(report["verdict"], "PASS")
        self.assertEqual(report["counted_run_ids"], ["pass-1", "pass-2", "pass-3"])
        self.assertEqual(report["retained_failed"], ["failed-early"])
        self.assertIn(TREVOR, report["reasons"])
        self.assertIn(TREVOR, markdown)
        self.assertIn("failed-early", markdown)
        self.assertIn("## Retained failed attempts", markdown)

    def test_failed_attempts_always_in_table(self):
        broken = self.series_of(
            [make_walk("a"), make_walk("b"), make_return_fail("broken"), make_walk("d")]
        )
        kept = self.series_of(
            [make_return_fail("kept-fail"), make_walk("p1"), make_walk("p2"), make_walk("p3")],
            "kept",
        )
        for report, failed_id in ((broken, "broken"), (kept, "kept-fail")):
            rows = _table_rows(render_markdown(report))
            matched = [
                row for row in rows if failed_id in row and "FAIL" in row
            ]
            self.assertEqual(len(matched), 1)
            self.assertIn(failed_id, report["retained_failed"])

    def test_mixed_config_not_evaluable(self):
        report = self.series_of(
            [
                make_walk("a"),
                make_walk("b"),
                make_walk("c", config_sha256=OTHER_CONFIG),
            ]
        )
        markdown = render_markdown(report)
        self.assertEqual(report["verdict"], "NOT_EVALUABLE")
        self.assertEqual(report["reasons"][0], CONFIG_CHANGE)
        self.assertEqual(report["counted_run_ids"], [])
        for run_id in ("a", "b", "c"):
            self.assertIn(run_id, markdown)
        self.assertEqual(len(_table_rows(markdown)), 3)

    def test_mixed_fastlio_revision_not_evaluable(self):
        report = self.series_of(
            [
                make_walk("a"),
                make_walk("b"),
                make_walk("c", fastlio_revision="deadbee"),
            ]
        )
        self.assertEqual(report["verdict"], "NOT_EVALUABLE")
        self.assertEqual(report["reasons"][0], CONFIG_CHANGE)
        self.assertIn("deadbee", " ".join(report["reasons"]))
        self.assertIn(REV, " ".join(report["reasons"]))
        self.assertEqual(len(_table_rows(render_markdown(report))), 3)

    def test_fewer_than_three_incomplete(self):
        two = self.series_of([make_walk("a"), make_walk("b")])
        self.assertEqual(two["verdict"], "INCOMPLETE")
        self.assertEqual(two["counted_run_ids"], [])
        one = self.series_of([make_walk("only")])
        self.assertEqual(one["verdict"], "INCOMPLETE")
        # A failure does not rule the series out before three attempts exist.
        mixed = self.series_of([make_return_fail("bad"), make_walk("good")])
        self.assertEqual(mixed["verdict"], "INCOMPLETE")
        self.assertIn("bad", mixed["retained_failed"])
        self.assertNotIn(TREVOR, mixed["reasons"])

    def test_all_historical_replay_only_with_exact_label(self):
        report = self.series_of(
            [make_replay("h1"), make_replay("h2"), make_replay("h3")],
            "replay",
        )
        markdown = render_markdown(report)
        self.assertEqual(report["verdict"], "REPLAY_ONLY")
        self.assertEqual(report["reasons"], [REPLAY])
        self.assertIn(REPLAY, markdown)
        self.assertEqual(report["counted_run_ids"], [])
        self.assertEqual(report["retained_failed"], ["h1", "h2", "h3"])
        self.assertTrue(all(not run["counts_toward_series"] for run in report["runs"]))

    def test_historical_run_never_counts(self):
        historical = make_replay("hist")
        loaded = self.load(historical, "hist.json")
        self.assertEqual(loaded.status, "PASS")
        self.assertTrue(loaded.w1_supports_pass())
        from spatio.series.report import counts_toward_series

        self.assertFalse(counts_toward_series(loaded))
        report = self.series_of(
            [make_walk("a"), historical, make_walk("b"), make_walk("c")]
        )
        self.assertEqual(report["verdict"], "FAIL")
        self.assertIn("hist", report["retained_failed"])
        self.assertNotIn("hist", report["counted_run_ids"])
        self.assertFalse(report["runs"][1]["counts_toward_series"])

    def test_null_placement_residual_blocks_counting(self):
        from spatio.series.report import counts_toward_series

        data = make_walk(
            "null-start",
            physical_reference={"start_placement_residual_m": None},
        )
        self.assertFalse(counts_toward_series(self.load(data, "null-start.json")))
        report = self.series_of(
            [
                make_walk("a", physical_reference={"start_placement_residual_m": None}),
                make_walk("b", physical_reference={"end_placement_residual_m": None}),
                make_walk(
                    "c",
                    physical_reference={
                        "start_placement_residual_m": None,
                        "end_placement_residual_m": None,
                    },
                ),
            ]
        )
        self.assertEqual(report["verdict"], "FAIL")
        self.assertEqual(report["counted_run_ids"], [])
        self.assertEqual(report["retained_failed"], ["a", "b", "c"])

    def test_heading_not_checked_blocks_counting(self):
        from spatio.series.report import counts_toward_series

        unchecked = make_walk(
            "unchecked",
            physical_reference={"heading_check": "not_checked"},
        )
        off = make_walk("off", physical_reference={"heading_check": "off"})
        self.assertFalse(counts_toward_series(self.load(unchecked, "unchecked.json")))
        self.assertFalse(counts_toward_series(self.load(off, "off.json")))
        report = self.series_of(
            [
                make_walk("a", physical_reference={"heading_check": "not_checked"}),
                make_walk("b"),
                make_walk("c"),
            ]
        )
        self.assertEqual(report["verdict"], "FAIL")
        self.assertFalse(report["runs"][0]["counts_toward_series"])
        self.assertIn("a", report["retained_failed"])

    def test_placement_over_tolerance_blocks_counting(self):
        from spatio.series.report import counts_toward_series

        start_over = make_walk(
            "start-over",
            physical_reference={"start_placement_residual_m": 0.011},
        )
        end_over = make_walk(
            "end-over",
            physical_reference={"end_placement_residual_m": 0.011},
        )
        exact = make_walk(
            "exact",
            physical_reference={
                "start_placement_residual_m": 0.01,
                "end_placement_residual_m": 0.01,
            },
        )
        self.assertFalse(counts_toward_series(self.load(start_over, "start-over.json")))
        self.assertFalse(counts_toward_series(self.load(end_over, "end-over.json")))
        self.assertTrue(counts_toward_series(self.load(exact, "exact.json")))
        report = self.series_of([start_over, end_over, make_walk("ok")])
        self.assertEqual(report["verdict"], "FAIL")
        self.assertEqual(report["retained_failed"], ["start-over", "end-over"])

    def test_boundary_d3_exactly_0_10_fails(self):
        from spatio.series.report import counts_toward_series

        on_boundary = {
            "d3_m": 0.10,
            "horizontal_m": 0.08,
            "vertical_m": 0.06,
        }
        claimed_pass = make_walk("claimed", return_error=on_boundary)
        self.assert_rejected(claimed_pass, "not <", "claimed.json")
        honest = make_walk(
            "honest",
            status="FAIL",
            reasons=["3D return error is not under 0.10 m"],
            return_error=on_boundary,
        )
        loaded = self.load(honest, "honest.json")
        self.assertFalse(loaded.w1_supports_pass())
        self.assertFalse(counts_toward_series(loaded))
        under = make_walk(
            "under",
            return_error={"d3_m": 0.09, "horizontal_m": 0.054, "vertical_m": 0.072},
        )
        self.assertTrue(counts_toward_series(self.load(under, "under.json")))
        report = self.series_of(
            [
                make_walk(
                    "a",
                    status="FAIL",
                    reasons=["3D return error is not under 0.10 m"],
                    return_error=on_boundary,
                ),
                make_walk(
                    "b",
                    status="FAIL",
                    reasons=["3D return error is not under 0.10 m"],
                    return_error=on_boundary,
                ),
                make_walk(
                    "c",
                    status="FAIL",
                    reasons=["3D return error is not under 0.10 m"],
                    return_error=on_boundary,
                ),
            ]
        )
        self.assertEqual(report["verdict"], "FAIL")
        self.assertEqual(report["counted_run_ids"], [])

    def test_rejects_pass_contradicting_numbers(self):
        self.assert_rejected(
            make_walk("big", return_error={"d3_m": 0.15, "horizontal_m": 0.09, "vertical_m": 0.12}),
            "contradicts",
            "big.json",
        )
        self.assert_rejected(
            make_walk("resets", validity={"resets": 2}),
            "resets",
            "resets.json",
        )
        self.assert_rejected(
            make_walk("still", validity={"moving": False}),
            "moving",
            "still.json",
        )
        self.assert_rejected(
            make_walk("jump", validity={"timestamps_monotonic": False}),
            "timestamps_monotonic",
            "jump.json",
        )
        self.assert_rejected(
            make_walk("double", walls={"doubled_wall_suspected": True}),
            "doubled_wall_suspected",
            "double.json",
        )
        self.assert_rejected(
            make_walk("null-d3", return_error={"d3_m": None}),
            "NOT_EVALUABLE",
            "null-d3.json",
        )
        self.assert_rejected(
            make_walk("null-wall", walls={"doubled_wall_suspected": None}),
            "NOT_EVALUABLE",
            "null-wall.json",
        )
        self.assert_rejected(
            make_walk(
                "loose",
                return_error={
                    "threshold_m": 0.20,
                    "d3_m": 0.15,
                    "horizontal_m": 0.09,
                    "vertical_m": 0.12,
                },
            ),
            "0.10",
            "loose.json",
        )
        self.assert_rejected(
            make_walk(
                "mismatch",
                return_error={"d3_m": 0.01, "horizontal_m": 0.03, "vertical_m": 0.04},
            ),
            "sqrt",
            "mismatch.json",
        )
        legal_unknown = make_walk(
            "unknown",
            status="NOT_EVALUABLE",
            reasons=["return error was not measured"],
            return_error={"d3_m": None},
        )
        loaded = self.load(legal_unknown, "unknown.json")
        self.assertEqual(loaded.status, "NOT_EVALUABLE")
        self.assert_rejected(
            make_walk(
                "null-fail",
                status="FAIL",
                reasons=["return error was not measured"],
                return_error={"d3_m": None},
            ),
            "NOT_EVALUABLE",
            "null-fail.json",
        )
        self.assert_rejected(
            make_walk(
                "evaluable",
                status="NOT_EVALUABLE",
                reasons=["producer was unsure"],
            ),
            "evaluable",
            "evaluable.json",
        )

    def test_rejects_non_pass_without_reasons(self):
        self.assert_rejected(
            make_walk("empty", status="FAIL", reasons=[]),
            "reasons",
            "empty.json",
        )
        self.assert_rejected(
            make_walk("blank", status="NOT_EVALUABLE", reasons=[""]),
            "reasons",
            "blank.json",
        )
        missing = make_walk("missing", status="FAIL", reasons=["3D return error is not under 0.10 m"])
        del missing["reasons"]
        self.assert_rejected(missing, "reasons", "missing.json")

    def test_null_vertical_cannot_hide_horizontal_over_d3(self):
        wide = {"d3_m": 0.05, "horizontal_m": 0.20, "vertical_m": None}
        self.assert_rejected(
            make_walk("wide-h", return_error=wide),
            "horizontal_m",
            "wide-h.json",
        )
        with self.assertRaises(InputRejected) as caught:
            self.series_of(
                [
                    make_walk("h1", return_error=wide),
                    make_walk("h2", return_error=wide),
                    make_walk("h3", return_error=wide),
                ],
                "wide-horizontal",
            )
        self.assertIn("horizontal_m", str(caught.exception))
        self.assertIn("exceeds", str(caught.exception))
        # Under the 10 cm return gate, but still larger than this d3.
        self.assert_rejected(
            make_walk(
                "under-gate",
                return_error={"d3_m": 0.05, "horizontal_m": 0.09, "vertical_m": None},
            ),
            "horizontal_m",
            "under-gate.json",
        )
        # Exactly 1 mm over d3 stays inside the component tolerance.
        self.load(
            make_walk(
                "mm",
                return_error={"d3_m": 0.05, "horizontal_m": 0.05 + 0.001, "vertical_m": None},
            ),
            "mm.json",
        )
        self.assert_rejected(
            make_walk(
                "past-mm",
                return_error={"d3_m": 0.05, "horizontal_m": 0.052, "vertical_m": None},
            ),
            "horizontal_m",
            "past-mm.json",
        )
        legal = self.series_of(
            [
                make_walk("n1", return_error={"horizontal_m": 0.03, "vertical_m": None}),
                make_walk("n2", return_error={"horizontal_m": None, "vertical_m": 0.04}),
                make_walk(
                    "n3",
                    return_error={"d3_m": 0.04, "horizontal_m": 0.0, "vertical_m": None},
                ),
            ],
            "null-component-ok",
        )
        self.assertEqual(legal["verdict"], "PASS")
        self.assertEqual(legal["counted_run_ids"], ["n1", "n2", "n3"])

    def test_null_horizontal_cannot_hide_vertical_over_d3(self):
        wide = {"d3_m": 0.05, "horizontal_m": None, "vertical_m": 0.20}
        self.assert_rejected(
            make_walk("wide-v", return_error=wide),
            "vertical_m",
            "wide-v.json",
        )
        with self.assertRaises(InputRejected) as caught:
            self.series_of(
                [
                    make_walk("v1", return_error=wide),
                    make_walk("v2", return_error=wide),
                    make_walk("v3", return_error=wide),
                ],
                "wide-vertical",
            )
        self.assertIn("vertical_m", str(caught.exception))
        self.assertIn("exceeds", str(caught.exception))
        fitted = self.load(
            make_walk(
                "fit-v",
                return_error={"d3_m": 0.05, "horizontal_m": None, "vertical_m": 0.05},
            ),
            "fit-v.json",
        )
        self.assertIsNone(fitted.return_error.horizontal_m)
        self.assertEqual(fitted.return_error.vertical_m, 0.05)
        from spatio.series.report import counts_toward_series

        self.assertTrue(counts_toward_series(fitted))

    def test_zero_length_path_with_moving_true_cannot_pass(self):
        zero_return = {"d3_m": 0.0, "horizontal_m": 0.0, "vertical_m": 0.0}
        zero_and_still = dict(
            path_length_m=0.0,
            drift_pct_of_path=None,
            return_error=zero_return,
        )
        self.assert_rejected(
            make_walk("zero-both", **zero_and_still),
            "zero-length path",
            "zero-both.json",
        )
        self.assert_rejected(
            make_walk("zero-path", path_length_m=0.0),
            "validity.moving is true",
            "zero-path.json",
        )
        with self.assertRaises(InputRejected) as zero_series:
            self.series_of(
                [
                    make_walk("z1", **zero_and_still),
                    make_walk("z2", **zero_and_still),
                    make_walk("z3", **zero_and_still),
                ],
                "zero-return",
            )
        self.assertIn("zero-length path", str(zero_series.exception))
        self.assertIn("run-0.json", str(zero_series.exception))
        with self.assertRaises(InputRejected) as positive_series:
            self.series_of(
                [
                    make_walk("p1", path_length_m=0.0),
                    make_walk("p2", path_length_m=0.0),
                    make_walk("p3", path_length_m=0.0),
                ],
                "positive-return",
            )
        self.assertIn("path_length_m is 0", str(positive_series.exception))
        self.assertIn("moving", str(positive_series.exception))
        # A claimed failure is still a contradictory motion claim.
        self.assert_rejected(
            make_walk(
                "fail-zero",
                status="FAIL",
                reasons=["path length was zero"],
                path_length_m=0.0,
            ),
            "zero-length path",
            "fail-zero.json",
        )
        # Stationary and not measured are not the same claim.
        still = self.load(
            make_walk(
                "still-zero",
                status="FAIL",
                reasons=["the cart did not move"],
                path_length_m=0.0,
                drift_pct_of_path=None,
                validity={"moving": False},
                return_error=zero_return,
            ),
            "still-zero.json",
        )
        self.assertEqual(still.path_length_m, 0.0)
        self.assertIs(still.validity.moving, False)
        self.assertFalse(still.w1_supports_pass())
        unmeasured = self.load(make_walk("null-path", path_length_m=None), "null-path.json")
        self.assertIsNone(unmeasured.path_length_m)
        from spatio.series.report import counts_toward_series

        self.assertTrue(counts_toward_series(unmeasured))
        # Exact zero is the cutoff. No must-move minimum is applied.
        small = self.load(
            make_walk(
                "small-path",
                path_length_m=0.01,
                drift_pct_of_path=None,
                return_error=zero_return,
            ),
            "small-path.json",
        )
        self.assertEqual(small.path_length_m, 0.01)
        self.assertTrue(counts_toward_series(small))

    def test_identical_endpoint_windows_cannot_pass(self):
        same = {
            "start_window_s": [0.0, 10.0],
            "end_window_s": [0.0, 10.0],
            "d3_m": 0.0,
            "horizontal_m": 0.0,
            "vertical_m": 0.0,
        }
        self.assert_rejected(
            make_walk("same-window", return_error=same),
            "overlap",
            "same-window.json",
        )
        with self.assertRaises(InputRejected) as caught:
            self.series_of(
                [
                    make_walk("w1", return_error=same),
                    make_walk("w2", return_error=same),
                    make_walk("w3", return_error=same),
                ],
                "same-windows",
            )
        self.assertIn("overlap", str(caught.exception))
        self.assertIn("[0.0, 10.0]", str(caught.exception))
        self.assertNotIn("reversed", str(caught.exception))
        separated = self.load(
            make_walk(
                "separated",
                return_error={"start_window_s": [0.0, 10.0], "end_window_s": [11.0, 21.0]},
            ),
            "separated.json",
        )
        self.assertEqual(separated.return_error.start_window_s, [0.0, 10.0])
        self.assertEqual(separated.return_error.end_window_s, [11.0, 21.0])
        missing_end = self.load(
            make_walk("null-end", return_error={"end_window_s": None}),
            "null-end.json",
        )
        self.assertIsNone(missing_end.return_error.end_window_s)
        from spatio.series.report import counts_toward_series

        self.assertTrue(counts_toward_series(missing_end))

    def test_overlapping_or_reversed_windows_rejected(self):
        self.assert_rejected(
            make_walk(
                "partial",
                return_error={"start_window_s": [0.0, 10.0], "end_window_s": [5.0, 15.0]},
            ),
            "overlap",
            "partial.json",
        )
        self.assert_rejected(
            make_walk(
                "contained",
                return_error={"start_window_s": [0.0, 30.0], "end_window_s": [10.0, 20.0]},
            ),
            "overlap",
            "contained.json",
        )
        self.assert_rejected(
            make_walk(
                "touch",
                return_error={"start_window_s": [0.0, 10.0], "end_window_s": [10.0, 20.0]},
            ),
            "overlap",
            "touch.json",
        )
        self.assert_rejected(
            make_walk(
                "reversed",
                return_error={"start_window_s": [20.0, 30.0], "end_window_s": [0.0, 10.0]},
            ),
            "reversed",
            "reversed.json",
        )
        with self.assertRaises(InputRejected) as caught:
            self.load(
                make_walk(
                    "reversed-check",
                    return_error={"start_window_s": [20.0, 30.0], "end_window_s": [0.0, 10.0]},
                ),
                "reversed-check.json",
            )
        self.assertNotIn("overlap", str(caught.exception))
        self.assert_rejected(
            make_walk(
                "crossed",
                return_error={"start_window_s": [10.0, 20.0], "end_window_s": [0.0, 15.0]},
            ),
            "overlap",
            "crossed.json",
        )

    def test_component_on_return_gate_cannot_pass_inside_tolerance(self):
        # The 1 mm consistency allowance must not pull a 10 cm length under the gate.
        on_horizontal = {"d3_m": 0.0995, "horizontal_m": 0.10, "vertical_m": 0.0}
        on_vertical = {"d3_m": 0.0995, "horizontal_m": 0.0, "vertical_m": 0.10}
        null_vertical = {"d3_m": 0.0995, "horizontal_m": 0.10, "vertical_m": None}
        null_horizontal = {"d3_m": 0.0995, "horizontal_m": None, "vertical_m": 0.10}
        # 0.08/0.06 is a 10 cm combined length. Each component is under 10 cm.
        combined = {"d3_m": 0.0995, "horizontal_m": 0.08, "vertical_m": 0.06}
        # Still inside the 1 mm band, and the combined length is over 10 cm.
        over_band = {"d3_m": 0.0995, "horizontal_m": 0.08032, "vertical_m": 0.06024}
        wider_than_d3 = {"d3_m": 0.0995, "horizontal_m": 0.1004, "vertical_m": 0.0}
        cases = (
            ("h-boundary", on_horizontal, "horizontal_m"),
            ("v-boundary", on_vertical, "vertical_m"),
            ("h-null", null_vertical, "horizontal_m"),
            ("v-null", null_horizontal, "vertical_m"),
            ("norm", combined, "sqrt"),
            ("norm-over", over_band, "sqrt"),
            ("h-over-band", wider_than_d3, "horizontal_m"),
        )
        for name, error, fragment in cases:
            self.assert_rejected(
                make_walk(name, return_error=error),
                fragment,
                "%s.json" % name,
            )
            with self.assertRaises(InputRejected) as caught:
                self.series_of(
                    [
                        make_walk("%s-1" % name, return_error=error),
                        make_walk("%s-2" % name, return_error=error),
                        make_walk("%s-3" % name, return_error=error),
                    ],
                    name,
                )
            text = str(caught.exception)
            self.assertIn(fragment, text)
            self.assertIn("not <", text)
            self.assertIn("run-0.json", text)
            self.assertIn("consistency tolerance does not apply", text)
        # A null axis is not zero, so it does not invent a combined length.
        with self.assertRaises(InputRejected) as null_case:
            self.load(make_walk("h-null-check", return_error=null_vertical), "h-null-check.json")
        self.assertNotIn("sqrt", str(null_case.exception))
        with self.assertRaises(InputRejected) as null_other:
            self.load(make_walk("v-null-check", return_error=null_horizontal), "v-null-check.json")
        self.assertNotIn("sqrt", str(null_other.exception))
        # 0.08 and 0.06 are each under the gate. Only the combined length fails.
        with self.assertRaises(InputRejected) as norm_case:
            self.load(make_walk("norm-check", return_error=combined), "norm-check.json")
        norm_text = str(norm_case.exception)
        self.assertIn("0.100000", norm_text)
        self.assertNotIn("return_error.horizontal_m", norm_text)
        self.assertNotIn("return_error.vertical_m", norm_text)

        from spatio.series.report import counts_toward_series, status_note

        honest_error = on_horizontal
        honest = make_walk(
            "honest-gate",
            status="FAIL",
            reasons=["horizontal return is not under 0.10 m"],
            return_error=honest_error,
        )
        loaded = self.load(honest, "honest-gate.json")
        self.assertFalse(loaded.w1_supports_pass())
        self.assertFalse(counts_toward_series(loaded))
        self.assertIsNone(status_note(loaded))
        report = self.series_of(
            [
                make_walk(
                    "f1",
                    status="FAIL",
                    reasons=["combined return is not under 0.10 m"],
                    return_error=combined,
                ),
                make_walk(
                    "f2",
                    status="FAIL",
                    reasons=["combined return is not under 0.10 m"],
                    return_error=combined,
                ),
                make_walk(
                    "f3",
                    status="FAIL",
                    reasons=["combined return is not under 0.10 m"],
                    return_error=combined,
                ),
            ],
            "honest-gate",
        )
        self.assertEqual(report["verdict"], "FAIL")
        self.assertEqual(report["counted_run_ids"], [])
        self.assertIsNone(report["runs"][0]["status_note"])

        under = {"d3_m": 0.0995, "horizontal_m": 0.0995, "vertical_m": 0.0}
        passed = self.series_of(
            [
                make_walk("u1", return_error=under),
                make_walk("u2", return_error=under),
                make_walk("u3", return_error=under),
            ],
            "just-under",
        )
        self.assertEqual(passed["verdict"], "PASS")
        self.assertEqual(passed["counted_run_ids"], ["u1", "u2", "u3"])
        null_under = self.load(
            make_walk(
                "null-under",
                return_error={"d3_m": 0.0995, "horizontal_m": 0.0995, "vertical_m": None},
            ),
            "null-under.json",
        )
        self.assertTrue(null_under.w1_supports_pass())
        self.assertTrue(counts_toward_series(null_under))

    def test_zero_duration_moving_run_cannot_pass(self):
        # Review: change only duration_s on a passing fixture. Path stays 42 m
        # and the endpoint windows still span 210 s.
        self.assert_rejected(
            make_walk("zero-duration", duration_s=0),
            "duration_s is 0",
            "zero-duration.json",
        )
        with self.assertRaises(InputRejected) as caught:
            self.series_of(
                [
                    make_walk("d1", duration_s=0),
                    make_walk("d2", duration_s=0),
                    make_walk("d3", duration_s=0),
                ],
                "zero-duration",
            )
        text = str(caught.exception)
        self.assertIn("duration_s is 0", text)
        self.assertIn("validity.moving is true", text)
        self.assertIn("path_length_m is 42.0", text)
        self.assertIn("span 210.000000 s", text)
        self.assertIn("run-0.json", text)
        # A claimed failure does not make the same contradiction legal.
        self.assert_rejected(
            make_walk(
                "fail-duration",
                status="FAIL",
                reasons=["duration was zero"],
                duration_s=0,
            ),
            "duration_s is 0",
            "fail-duration.json",
        )
        # Each claim is enough on its own.
        self.assert_rejected(
            make_walk(
                "moving-only",
                duration_s=0,
                path_length_m=None,
                drift_pct_of_path=None,
                return_error={"start_window_s": None, "end_window_s": None},
            ),
            "validity.moving is true",
            "moving-only.json",
        )
        with self.assertRaises(InputRejected) as moving_only:
            self.load(
                make_walk(
                    "moving-only-check",
                    duration_s=0,
                    path_length_m=None,
                    drift_pct_of_path=None,
                    return_error={"start_window_s": None, "end_window_s": None},
                ),
                "moving-only-check.json",
            )
        moving_text = str(moving_only.exception)
        self.assertNotIn("path_length_m", moving_text)
        self.assertNotIn("endpoint windows", moving_text)
        self.assert_rejected(
            make_walk(
                "windows-only",
                status="FAIL",
                reasons=["duration does not cover the endpoint windows"],
                duration_s=0,
                path_length_m=0.0,
                drift_pct_of_path=None,
                validity={"moving": False},
                return_error={"d3_m": 0.0, "horizontal_m": 0.0, "vertical_m": 0.0},
            ),
            "endpoint windows",
            "windows-only.json",
        )
        with self.assertRaises(InputRejected) as windows_only:
            self.load(
                make_walk(
                    "windows-only-check",
                    status="FAIL",
                    reasons=["duration does not cover the endpoint windows"],
                    duration_s=0,
                    path_length_m=0.0,
                    drift_pct_of_path=None,
                    validity={"moving": False},
                    return_error={"d3_m": 0.0, "horizontal_m": 0.0, "vertical_m": 0.0},
                ),
                "windows-only-check.json",
            )
        windows_text = str(windows_only.exception)
        self.assertNotIn("validity.moving is true", windows_text)
        self.assertNotIn("path_length_m", windows_text)
        self.assert_rejected(
            make_walk(
                "travel-only",
                status="FAIL",
                reasons=["duration was zero"],
                duration_s=0,
                path_length_m=42.0,
                validity={"moving": False},
                return_error={
                    "d3_m": 0.0,
                    "horizontal_m": 0.0,
                    "vertical_m": 0.0,
                    "start_window_s": None,
                    "end_window_s": None,
                },
            ),
            "path_length_m is 42.0",
            "travel-only.json",
        )
        # A stationary measured zero and a null duration still load.
        # A one-second duration beside the default windows does not: those
        # windows span 210 s. That case used to be asserted as a pass.
        still = self.load(
            make_walk(
                "still-duration",
                status="FAIL",
                reasons=["the cart did not move"],
                duration_s=0,
                path_length_m=0.0,
                drift_pct_of_path=None,
                validity={"moving": False},
                return_error={
                    "d3_m": 0.0,
                    "horizontal_m": 0.0,
                    "vertical_m": 0.0,
                    "start_window_s": None,
                    "end_window_s": None,
                },
            ),
            "still-duration.json",
        )
        self.assertEqual(still.duration_s, 0.0)
        self.assertIs(still.validity.moving, False)
        self.assertFalse(still.w1_supports_pass())
        from spatio.series.report import counts_toward_series

        unmeasured = self.load(make_walk("null-duration", duration_s=None), "null-duration.json")
        self.assertIsNone(unmeasured.duration_s)
        self.assertTrue(counts_toward_series(unmeasured))
        self.assert_rejected(
            make_walk("one-second", duration_s=1.0),
            "span 210.000000 s",
            "one-second.json",
        )

    def test_duration_shorter_than_window_span_cannot_pass(self):
        from spatio.series.report import counts_toward_series

        # Review: change only duration_s to 1 on a passing fixture. The
        # default windows stay [0, 10] and [200, 210], a 210 s span.
        with self.assertRaises(InputRejected) as caught:
            self.series_of(
                [
                    make_walk("s1", duration_s=1),
                    make_walk("s2", duration_s=1),
                    make_walk("s3", duration_s=1),
                ],
                "short-span",
            )
        text = str(caught.exception)
        self.assertIn("duration_s is 1.0", text)
        self.assertIn("span 210.000000 s", text)
        self.assertIn("run-0.json", text)
        # 209 s is the same contradiction. The cutoff is the span, not 1.
        self.assert_rejected(
            make_walk("almost", duration_s=209),
            "span 210.000000 s",
            "almost.json",
        )
        # A different pair of windows, not starting at zero, uses the same rule.
        other_windows = {
            "start_window_s": [1000.0, 1010.0],
            "end_window_s": [1200.0, 1210.0],
        }
        self.assert_rejected(
            make_walk("other-clock", duration_s=1.0, return_error=other_windows),
            "span 210.000000 s",
            "other-clock.json",
        )
        # Status does not make a short duration legal.
        self.assert_rejected(
            make_walk(
                "fail-short",
                status="FAIL",
                reasons=["duration does not cover the endpoint windows"],
                duration_s=1.0,
            ),
            "span 210.000000 s",
            "fail-short.json",
        )
        # Equal to the span still fits. No minimum duration is applied.
        exact = self.series_of(
            [
                make_walk("e1", duration_s=210.0),
                make_walk("e2", duration_s=210.0),
                make_walk("e3", duration_s=210.0),
            ],
            "exact-span",
        )
        self.assertEqual(exact["verdict"], "PASS")
        self.assertEqual(exact["counted_run_ids"], ["e1", "e2", "e3"])
        fitted_windows = {
            "start_window_s": [0.0, 0.4],
            "end_window_s": [0.5, 1.0],
        }
        fitted = self.load(
            make_walk("fitted", duration_s=1.0, return_error=fitted_windows),
            "fitted.json",
        )
        self.assertEqual(fitted.duration_s, 1.0)
        self.assertTrue(fitted.w1_supports_pass())
        self.assertTrue(counts_toward_series(fitted))
        # A missing end window does not excuse the start window. The default
        # start window is [0, 10], ten seconds, which cannot fit in one second.
        # This used to be asserted as a pass. That assertion was wrong.
        self.assert_rejected(
            make_walk(
                "open-ended",
                duration_s=1.0,
                return_error={"end_window_s": None},
            ),
            "return_error.start_window_s covers 10.000000 s",
            "open-ended.json",
        )
        # A duration longer than a span that does not start at zero still loads.
        longer = self.load(
            make_walk("longer", duration_s=220.0, return_error=other_windows),
            "longer.json",
        )
        self.assertTrue(counts_toward_series(longer))

    def test_single_window_longer_than_duration_cannot_pass(self):
        from spatio.series.report import counts_toward_series

        # Review: duration 1 s, start window [0, 10], end window null.
        # Three otherwise-valid runs must not come out PASS.
        with self.assertRaises(InputRejected) as null_end:
            self.series_of(
                [
                    make_walk("e1", duration_s=1.0, return_error={"end_window_s": None}),
                    make_walk("e2", duration_s=1.0, return_error={"end_window_s": None}),
                    make_walk("e3", duration_s=1.0, return_error={"end_window_s": None}),
                ],
                "null-end-short",
            )
        null_end_text = str(null_end.exception)
        self.assertIn("run-0.json", null_end_text)
        self.assertIn("duration_s is 1.0", null_end_text)
        self.assertIn("return_error.start_window_s covers 10.000000 s", null_end_text)
        self.assertNotIn("span ", null_end_text)
        # Swapped: start window null, default end window [200, 210].
        with self.assertRaises(InputRejected) as null_start:
            self.series_of(
                [
                    make_walk("s1", duration_s=1.0, return_error={"start_window_s": None}),
                    make_walk("s2", duration_s=1.0, return_error={"start_window_s": None}),
                    make_walk("s3", duration_s=1.0, return_error={"start_window_s": None}),
                ],
                "null-start-short",
            )
        null_start_text = str(null_start.exception)
        self.assertIn("run-0.json", null_start_text)
        self.assertIn("duration_s is 1.0", null_start_text)
        self.assertIn("return_error.end_window_s covers 10.000000 s", null_start_text)
        self.assertNotIn("span ", null_start_text)
        # Status does not make one long window legal.
        self.assert_rejected(
            make_walk(
                "fail-null-end",
                status="FAIL",
                reasons=["duration does not cover the start window"],
                duration_s=1.0,
                return_error={"end_window_s": None},
            ),
            "return_error.start_window_s covers 10.000000 s",
            "fail-null-end.json",
        )
        self.assert_rejected(
            make_walk(
                "fail-null-start",
                status="FAIL",
                reasons=["duration does not cover the end window"],
                duration_s=1.0,
                return_error={"start_window_s": None},
            ),
            "return_error.end_window_s covers 10.000000 s",
            "fail-null-start.json",
        )
        # A window whose own length equals the duration still fits.
        # The other window stays null, so there is no paired span.
        null_end_fit = self.series_of(
            [
                make_walk("f1", duration_s=10.0, return_error={"end_window_s": None}),
                make_walk("f2", duration_s=10.0, return_error={"end_window_s": None}),
                make_walk("f3", duration_s=10.0, return_error={"end_window_s": None}),
            ],
            "null-end-fits",
        )
        self.assertEqual(null_end_fit["verdict"], "PASS")
        self.assertEqual(null_end_fit["counted_run_ids"], ["f1", "f2", "f3"])
        null_start_fit = self.load(
            make_walk("fit-end", duration_s=10.0, return_error={"start_window_s": None}),
            "fit-end.json",
        )
        self.assertIsNone(null_start_fit.return_error.start_window_s)
        self.assertEqual(null_start_fit.return_error.end_window_s, [200.0, 210.0])
        self.assertTrue(counts_toward_series(null_start_fit))
        # A shorter window on a clock that does not start at zero still loads.
        shorter = self.load(
            make_walk(
                "shorter-window",
                duration_s=10.0,
                return_error={"start_window_s": [1000.0, 1004.0], "end_window_s": None},
            ),
            "shorter-window.json",
        )
        self.assertTrue(counts_toward_series(shorter))
        # The same ten-second length is rejected when the clock does not start at zero.
        self.assert_rejected(
            make_walk(
                "late-window",
                duration_s=1.0,
                return_error={"start_window_s": None, "end_window_s": [1000.0, 1010.0]},
            ),
            "return_error.end_window_s covers 10.000000 s",
            "late-window.json",
        )
        # Each window can fit while the pair does not. The paired-span check stays.
        with self.assertRaises(InputRejected) as pair_still:
            self.load(make_walk("pair-still", duration_s=20.0), "pair-still.json")
        pair_text = str(pair_still.exception)
        self.assertIn("span 210.000000 s", pair_text)
        self.assertNotIn("covers 10.000000 s", pair_text)

    def test_null_rendered_not_measured_not_zero(self):
        data = make_walk(
            "missing-horizontal",
            return_error={"horizontal_m": None},
            drift_pct_of_path=None,
        )
        text = explain_text(self.load(data, "missing-horizontal.json"))
        self.assertIn("Horizontal return error is not measured", text)
        self.assertNotIn("Horizontal return error is 0", text)
        self.assertIn("drift is not measured", text)
        self.assertNotIn("drift is 0", text)
        self.assertGreaterEqual(len(_sentence_lines(text)), 5)
        self.assertLessEqual(len(_sentence_lines(text)), 10)
        report = self.series_of([data], "nulls")
        stored = report["runs"][0]
        self.assertIsNone(stored["return_error"]["horizontal_m"])
        self.assertIsNone(stored["drift_pct_of_path"])
        self.assertNotEqual(stored["return_error"]["horizontal_m"], 0)
        self.assertIn("not measured", render_markdown(report))

    def test_units_cm_in_text_m_in_json(self):
        data = make_walk("units")
        text = explain_text(self.load(data, "units.json"))
        self.assertIn("5.0 cm", text)
        self.assertIn("3.0 cm", text)
        self.assertIn("4.0 cm", text)
        self.assertNotIn("0.05 cm", text)
        report = self.series_of([data, make_walk("units-b"), make_walk("units-c")], "units")
        stored = report["runs"][0]["return_error"]
        self.assertEqual(stored["d3_m"], 0.05)
        self.assertEqual(stored["horizontal_m"], 0.03)
        self.assertEqual(stored["vertical_m"], 0.04)
        self.assertEqual(report["runs"][0]["path_length_m"], 42.0)
        self.assertLess(stored["d3_m"], 1.0)
        markdown = render_markdown(report)
        self.assertIn("return 3D (cm)", markdown)
        self.assertIn("5.0", markdown)
        self.assertNotIn("0.05 cm", markdown)

    def test_proposed_wall_gate_labelled(self):
        proposed = explain_text(self.load(make_walk("proposed"), "proposed.json"))
        self.assertIn("wall gate of 5.0 cm is provisional", proposed)
        self.assertIn("placement tolerance is 1.0 cm (provisional)", proposed)
        self.assertNotIn("not yet approved", proposed)
        approved = explain_text(
            self.load(
                make_walk("approved", walls={"gate_state": "approved"}),
                "approved.json",
            )
        )
        self.assertIn("wall gate of 5.0 cm is provisional", approved)
        self.assertNotIn("not yet approved", approved)
        other = explain_text(
            self.load(
                make_walk("other", walls={"gate_m": 0.08, "gate_state": "proposed"}),
                "other.json",
            )
        )
        self.assertIn("proposed, not yet approved by Trevor", other)
        self.assertNotIn("8.0 cm is provisional", other)

    def test_explain_golden_text(self):
        text = explain_text(self.load(golden_walks()[0], "golden.json"))
        expected = (GOLDEN / "explain_pass.txt").read_text(encoding="utf-8")
        self.assertEqual(text, expected)
        self.assertGreaterEqual(len(_sentence_lines(text)), 5)
        self.assertLessEqual(len(_sentence_lines(text)), 10)

    def test_compare_markdown_golden(self):
        report = self.series_of(golden_walks(), "golden-pass")
        markdown = render_markdown(report)
        expected = (GOLDEN / "series_pass.md").read_text(encoding="utf-8")
        self.assertEqual(markdown, expected)
        self.assertIn("Verdict: PASS", markdown)
        self.assertIn("g-pass-1", markdown)
        self.assertIn(CONFIG_CHANGE, markdown)
        self.assertIn("0.10 m", markdown)

    def test_precedence_order(self):
        # Stronger verdicts are checked first so a weaker one cannot hide them.
        mixed_replay = self.series_of(
            [make_replay("a"), make_replay("b", config_sha256=OTHER_CONFIG)]
        )
        self.assertEqual(mixed_replay["verdict"], "NOT_EVALUABLE")

        one_replay = self.series_of([make_replay("only")])
        self.assertEqual(one_replay["verdict"], "REPLAY_ONLY")

        failed_replays = self.series_of(
            [
                make_replay(
                    "f1",
                    status="FAIL",
                    reasons=["3D return error is not under 0.10 m"],
                    return_error={"d3_m": 0.13, "horizontal_m": 0.12, "vertical_m": 0.05},
                ),
                make_replay(
                    "f2",
                    status="FAIL",
                    reasons=["3D return error is not under 0.10 m"],
                    return_error={"d3_m": 0.13, "horizontal_m": 0.12, "vertical_m": 0.05},
                ),
                make_replay(
                    "f3",
                    status="FAIL",
                    reasons=["3D return error is not under 0.10 m"],
                    return_error={"d3_m": 0.13, "horizontal_m": 0.12, "vertical_m": 0.05},
                ),
            ]
        )
        self.assertEqual(failed_replays["verdict"], "REPLAY_ONLY")

        config_blocks_pass = self.series_of(
            [make_walk("a"), make_walk("b"), make_walk("c", config_sha256=OTHER_CONFIG)]
        )
        self.assertEqual(config_blocks_pass["verdict"], "NOT_EVALUABLE")

        pass_beats_fail = self.series_of(
            [make_walk("a"), make_walk("b"), make_walk("c"), make_return_fail("d")]
        )
        self.assertEqual(pass_beats_fail["verdict"], "PASS")

        incomplete_beats_fail = self.series_of([make_walk("a"), make_walk("b")])
        self.assertEqual(incomplete_beats_fail["verdict"], "INCOMPLETE")

        finished_fail = self.series_of(
            [make_walk("a"), make_return_fail("b"), make_walk("c")]
        )
        self.assertEqual(finished_fail["verdict"], "FAIL")

    def test_cli_exit_codes(self):
        good = self._write("good.json", make_walk("cli-a"))
        second = self._write("second.json", make_walk("cli-b"))
        third = self._write("third.json", make_walk("cli-c"))
        bad = self._write(
            "bad.json",
            make_walk("bad", return_error={"d3_m": 0.15, "horizontal_m": 0.09, "vertical_m": 0.12}),
        )
        garbage = self._write_text("garbage.json", "{not json")
        missing = os.path.join(self.tmp.name, "missing.json")
        failed = self._write("failed.json", make_return_fail("cli-fail"))

        explain_ok = self._cli(["explain", good])
        self.assertEqual(explain_ok.returncode, 0)
        self.assertIn("cli-a is PASS", explain_ok.stdout)

        out_file = os.path.join(self.tmp.name, "explain.txt")
        explain_file = self._cli(["explain", good, "--out", out_file])
        self.assertEqual(explain_file.returncode, 0)
        self.assertEqual(explain_file.stdout, Path(out_file).read_text(encoding="utf-8"))

        explain_bad = self._cli(["explain", bad])
        self.assertEqual(explain_bad.returncode, 1)
        self.assertIn("bad.json", explain_bad.stderr)
        self.assertIn("contradicts", explain_bad.stderr)

        explain_missing = self._cli(["explain", missing])
        self.assertEqual(explain_missing.returncode, 2)
        self.assertIn("missing.json", explain_missing.stderr)

        explain_garbage = self._cli(["explain", garbage])
        self.assertEqual(explain_garbage.returncode, 2)

        out_dir = os.path.join(self.tmp.name, "out-pass")
        compare_ok = self._cli(
            ["compare", good, second, third, "--out", out_dir, "--series-id", "cli"]
        )
        self.assertEqual(compare_ok.returncode, 0)
        payload = json.loads(Path(out_dir, "series.json").read_text(encoding="utf-8"))
        self.assertEqual(payload["schema"], "spatio.series/1")
        self.assertEqual(payload["verdict"], "PASS")
        self.assertTrue(os.path.isfile(os.path.join(out_dir, "series.md")))

        out_fail = os.path.join(self.tmp.name, "out-fail")
        compare_fail = self._cli(
            ["compare", good, failed, third, "--out", out_fail, "--series-id", "cli-fail"]
        )
        self.assertEqual(compare_fail.returncode, 0)
        fail_payload = json.loads(Path(out_fail, "series.json").read_text(encoding="utf-8"))
        self.assertEqual(fail_payload["verdict"], "FAIL")

        out_rejected = os.path.join(self.tmp.name, "out-rejected")
        compare_bad = self._cli(["compare", bad, "--out", out_rejected])
        self.assertEqual(compare_bad.returncode, 1)
        self.assertFalse(os.path.exists(os.path.join(out_rejected, "series.md")))

        out_missing = os.path.join(self.tmp.name, "out-missing")
        compare_missing = self._cli(["compare", missing, "--out", out_missing])
        self.assertEqual(compare_missing.returncode, 2)
        self.assertFalse(os.path.exists(os.path.join(out_missing, "series.md")))

    def test_rejects_historical_with_physical_reference(self):
        data = make_walk("hist")
        data["physical_accuracy_evaluated"] = False
        data["accuracy_label"] = REPLAY
        self.assert_rejected(data, "physical_reference must be null", "hist.json")

    def test_rejects_wrong_replay_label(self):
        data = make_replay("hist")
        data["accuracy_label"] = "replay only"
        self.assert_rejected(data, REPLAY, "label.json")
        missing = make_replay("hist2")
        del missing["accuracy_label"]
        self.assert_rejected(missing, "accuracy_label", "missing-label.json")

    def test_downgraded_fail_is_not_upgraded(self):
        data = make_walk("downgraded", status="FAIL", reasons=["operator stopped early"])
        loaded = self.load(data, "downgraded.json")
        self.assertTrue(loaded.w1_supports_pass())
        report = self.series_of([data], "one-fail")
        self.assertEqual(report["verdict"], "INCOMPLETE")
        self.assertFalse(report["runs"][0]["counts_toward_series"])
        self.assertIn("does not upgrade status", " ".join(report["reasons"]))
        self.assertIn("does not upgrade status", report["runs"][0]["status_note"])

    def test_rejects_bad_shape(self):
        missing_key = make_walk("shape")
        del missing_key["run_id"]
        self.assert_rejected(missing_key, "run_id", "shape.json")
        bad_schema = make_walk("schema")
        bad_schema["schema"] = "spatio.walk_result/0"
        self.assert_rejected(bad_schema, "schema", "schema.json")
        upper = make_walk("upper", config_sha256=CONFIG.upper())
        self.assert_rejected(upper, "config_sha256", "upper.json")
        loose_tol = make_walk(
            "tol",
            physical_reference={"placement_tolerance_m": 0.05},
        )
        self.assert_rejected(loose_tol, "0.01", "tol.json")
        negative = make_walk(
            "negative",
            physical_reference={"start_placement_residual_m": -0.001},
        )
        self.assert_rejected(negative, ">= 0", "negative.json")
        first = self.load(make_walk("same"), "same-a.json")
        second = self.load(make_walk("same"), "same-b.json")
        with self.assertRaises(InputRejected) as caught:
            build_series([first, second], "dup")
        self.assertIn("same", str(caught.exception))

    def _write(self, name, data):
        path = os.path.join(self.tmp.name, name)
        with open(path, "w", encoding="utf-8") as handle:
            json.dump(data, handle)
        return path

    def _write_text(self, name, text):
        path = os.path.join(self.tmp.name, name)
        with open(path, "w", encoding="utf-8") as handle:
            handle.write(text)
        return path

    def _cli(self, args):
        return subprocess.run(
            [sys.executable, "-m", "spatio.series"] + args,
            cwd=str(ROOT),
            capture_output=True,
            text=True,
        )


if __name__ == "__main__":
    unittest.main()
