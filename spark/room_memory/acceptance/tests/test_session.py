"""Operator script. A fake clock and a scripted reader; no terminal and no docker."""

import pytest

from acceptance.session import parse_label, resolve_durations, run_session
from acceptance.tests.support import ns


class Clock:
    def __init__(self):
        self.now = 0

    def monotonic_ns(self):
        return self.now


class Script:
    def __init__(self, clock, steps):
        self.clock = clock
        self.steps = list(steps)

    def read_line(self, deadline_ns):
        if not self.steps:
            raise AssertionError("session read past the script")
        kind = self.steps.pop(0)
        if kind[0] == "timeout":
            assert deadline_ns is not None
            self.clock.now = deadline_ns
            return None
        if kind[0] == "line":
            at_s, text = kind[1], kind[2]
            at = ns(at_s)
            if deadline_ns is not None and at > deadline_ns:
                self.clock.now = deadline_ns
                return None
            self.clock.now = at
            return text
        raise AssertionError(kind)


def _run(steps, **kwargs):
    clock = Clock()
    script = Script(clock, steps)
    out = []
    marks = run_session(clock=clock, read_line=script.read_line, write=out.append, **kwargs)
    assert script.steps == []
    assert any("reference" in line.lower() for line in out)
    return marks


def test_verified_change_records_completion_and_clear_separately():
    marks = _run([
        ("line", 1, "verified"),
        ("line", 2, "new_object reference_object other_reference_object"),
        ("line", 10, ""),
        ("line", 25, ""),
        ("timeout",),
        ("line", 146, "region 1 0 0 0 1 1 1"),
    ], scenario_ids=["box_placed"])
    assert marks["reference_setup"]["operator_confirmed"] is True
    assert marks["reference_setup"]["placement_is_removal"] is False
    row = next(item for item in marks["scenarios"] if item["id"] == "box_placed")
    assert row["change_done_mono_ns"] == ns(10)
    assert row["clear_mono_ns"] == ns(25)
    assert row["change_done_mono_ns"] != row["clear_mono_ns"]
    assert row["completion"] == "complete"
    assert row["protocol_duration_s"] == 120
    assert row["actual_duration_s"] == 120
    assert row["targets"][0]["sensor"] == 1
    assert row["targets"][0]["aabb"] == [[0.0, 0.0, 0.0], [1.0, 1.0, 1.0]]


def test_early_enter_shortens_and_does_not_claim_the_protocol():
    marks = _run([
        ("line", 1, "verified"),
        ("line", 2, "new_object reference_object other_reference_object"),
        ("line", 5, ""),
        ("line", 6, "skip"),
    ], scenario_ids=["baseline_idle"])
    row = next(item for item in marks["scenarios"] if item["id"] == "baseline_idle")
    assert row["completion"] == "shortened"
    assert row["actual_duration_s"] < row["protocol_duration_s"]
    assert row["protocol_duration_s"] == 60


def test_incompatible_setup_still_runs_idle():
    marks = _run([
        ("line", 1, "incompatible"),
        ("line", 5, ""),
        ("timeout",),
    ], scenario_ids=["box_placed", "baseline_idle"])
    placed = next(item for item in marks["scenarios"] if item["id"] == "box_placed")
    idle = next(item for item in marks["scenarios"] if item["id"] == "baseline_idle")
    assert placed["completion"] == "blocked"
    assert idle["completion"] == "complete"
    assert idle["actual_duration_s"] == 60
    assert marks["session"]["end_reason"] == "complete"


def test_abort_skips_the_rest():
    marks = _run([
        ("line", 1, "verified"),
        ("line", 2, "new_object reference_object other_reference_object"),
        ("line", 3, "abort"),
    ], scenario_ids=["baseline_idle", "hour_idle"])
    idle = next(item for item in marks["scenarios"] if item["id"] == "baseline_idle")
    hour = next(item for item in marks["scenarios"] if item["id"] == "hour_idle")
    assert idle["completion"] == "aborted"
    assert hour["completion"] == "skipped"
    assert marks["session"]["end_reason"] == "aborted"


def test_sit_override_keeps_the_protocol_duration():
    marks = _run([
        ("line", 1, "verified"),
        ("line", 2, "new_object reference_object other_reference_object"),
        ("line", 5, "skip"),
        ("timeout",),
    ], scenario_ids=["person_sit"], durations={"person_sit": 30})
    row = next(item for item in marks["scenarios"] if item["id"] == "person_sit")
    assert row["protocol_duration_s"] == 120
    assert row["planned_duration_s"] == 30
    assert row["actual_duration_s"] == 30
    assert row["completion"] == "shortened"


def test_resolve_durations_does_not_let_watch_s_shrink_the_hour():
    class Args:
        watch_s = 10
        sit_s = None
        hour_s = None
        walk_s = None

    durations = resolve_durations(Args())
    assert durations["change"] == 10
    assert durations["baseline_idle"] == 10
    assert durations["hour_idle"] == 3600
    assert durations["person_sit"] == 120
    assert durations["person_walk"] == 105


def test_parse_label_move_source():
    targets, associations = parse_label("region 1 5 0 0 6 1 1 from 2.5 0.5 0.5")
    assert targets[0]["from_centroid"] == [2.5, 0.5, 0.5]
    assert associations == []
    _, associated = parse_label("event 4 7")
    assert associated[0]["event_id"] == 7
    assert associated[0]["publisher_generation"] == 4


def test_bad_label_is_rejected():
    with pytest.raises(ValueError):
        parse_label("region only-three-numbers")
