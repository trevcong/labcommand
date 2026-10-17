"""Offline acceptance scores. No socket, no docker, no Spark."""

import json
import math

from acceptance.score import score
from acceptance.tests.support import (
    FAR,
    MOVE_DEST,
    MOVE_FROM,
    MOVE_SOURCE,
    PERSON,
    PLACE,
    REMOVE,
    event,
    latched_sample,
    marks,
    scenario,
    stream,
    target,
    verified_setup,
)

# Grid helpers: a scenario [t0, t1] needs samples at every second of that closed interval.


def _score(messages, mark, threshold=1.0):
    return score(messages, mark, max_false_alarms_per_hour=threshold)


def _row(report, scenario_id):
    return next(row for row in report["scenarios"] if row["id"] == scenario_id)


def _dispositions(report, name):
    return [item for item in report["candidates"] if item["disposition"] == name]


def test_true_detection_uses_region_and_receipt_delay():
    messages = stream(9, 131, [event(60, "appeared", 1, PLACE)], stamp_offset_s=5000)
    mark = marks([scenario(
        "box_placed", "change", 10, 130, kind="appeared", protocol=120, planned=120, actual=120,
        change_done=10, clear=20, targets=[target(1, PLACE)], prerequisites=["reference_setup"],
    )])
    report = _score(messages, mark)
    row = _row(report, "box_placed")
    assert row["detection"] == "true_detection"
    assert row["receipt_delay_s"] == 50
    assert row["clear_to_event_s"] == 40
    assert row["clear_to_event_basis"] == "not_persistence_gate"
    assert report["true_detections"] == 1
    assert report["false_alarms"] == 0
    assert report["detection_delay_basis"] == "receipt_mono"
    assert "persistence_gate_start" not in json.dumps(report)


def test_unrelated_same_kind_is_not_the_detection_or_corroboration():
    messages = stream(9, 131, [
        event(60, "appeared", 1, PLACE),
        event(70, "appeared", 2, FAR),
    ])
    mark = marks([scenario(
        "box_placed", "change", 10, 130, kind="appeared", protocol=120,
        change_done=10, clear=20, targets=[target(1, PLACE)],
    )])
    report = _score(messages, mark)
    assert _row(report, "box_placed")["detection"] == "true_detection"
    assert report["true_detections"] == 1
    assert report["false_alarms"] == 1
    assert _dispositions(report, "corroboration") == []
    far = next(item for item in report["candidates"] if item["event_id"] == 2)
    assert far["disposition"] == "false_alarm"


def test_kind_and_time_without_a_label_is_a_temporal_match():
    messages = stream(9, 131, [
        event(60, "appeared", 1, PLACE),
        event(70, "appeared", 2, FAR),
    ])
    mark = marks([scenario(
        "box_placed", "change", 10, 130, kind="appeared", protocol=120, change_done=10, clear=20,
    )])
    report = _score(messages, mark)
    assert _row(report, "box_placed")["detection"] == "temporal_match"
    assert report["true_detections"] == 0
    assert report["temporal_matches"] == 2
    assert report["false_alarms"] == 0
    assert report["misses"] == 0
    assert report["g5_acceptance"] == "denied"


def test_one_sensor_corroborates_a_target_once():
    cycles = [event(60, "appeared", 1, PLACE, sensor=1)]
    for index in range(5):
        event_id = 10 + index
        cycles.append(event(70 + index, "appeared", event_id, PLACE, sensor=2))
        cycles.append(event(70.4 + index, "appeared", event_id, PLACE, sensor=2, state="expired"))
    messages = stream(9, 131, cycles)
    mark = marks([scenario(
        "box_placed", "change", 10, 130, kind="appeared", protocol=120,
        change_done=10, clear=20, targets=[target(1, PLACE), target(2, PLACE)],
    )])
    report = _score(messages, mark)
    assert report["true_detections"] == 1
    assert [item["event_id"] for item in _dispositions(report, "corroboration")] == [10]
    assert sorted(item["event_id"] for item in _dispositions(report, "false_alarm")) == [11, 12, 13, 14]
    assert report["false_alarms"] == 4

    session_cycles = []
    for index in range(5):
        event_id = 30 + index
        session_cycles.append(event(160 + index * 2, "appeared", event_id, PLACE, sensor=2))
        session_cycles.append(event(160.4 + index * 2, "appeared", event_id, PLACE, sensor=2, state="expired"))
    session = _score(*_perfect(
        extra_events=session_cycles,
        place_targets=[target(1, PLACE), target(2, PLACE)],
    ))
    assert session["true_detections"] == 3
    assert [item["event_id"] for item in _dispositions(session, "corroboration")] == [30]
    assert session["false_alarms"] == 4
    assert session["g5_acceptance"] != "full_candidate"
    assert session["exit_code"] == 1


def test_other_sensor_is_not_automatic_corroboration():
    messages = stream(9, 131, [
        event(60, "appeared", 1, PLACE, sensor=1),
        event(61, "appeared", 2, PLACE, sensor=2),
    ])
    mark = marks([scenario(
        "box_placed", "change", 10, 130, kind="appeared", protocol=120,
        change_done=10, clear=20, targets=[target(1, PLACE)],
    )])
    report = _score(messages, mark)
    assert report["true_detections"] == 1
    assert report["false_alarms"] == 1
    assert _dispositions(report, "corroboration") == []

    both = marks([scenario(
        "box_placed", "change", 10, 130, kind="appeared", protocol=120,
        change_done=10, clear=20, targets=[target(1, PLACE), target(2, PLACE)],
    )])
    corroborated = _score(messages, both)
    assert corroborated["true_detections"] == 1
    assert len(_dispositions(corroborated, "corroboration")) == 1
    assert corroborated["false_alarms"] == 0


def test_quiet_miss_is_not_change_as_activity():
    messages = stream(9, 131)
    mark = marks([scenario(
        "box_placed", "change", 10, 130, kind="appeared", protocol=120,
        change_done=10, clear=20, targets=[target(1, PLACE)],
    )])
    report = _score(messages, mark)
    row = _row(report, "box_placed")
    assert row["detection"] == "miss"
    assert report["misses"] == 1
    assert report["miss_with_activity_observed"] == 0
    assert report["change_as_activity"] == 0


def test_activity_away_from_the_target_is_only_miss_with_activity():
    messages = stream(9, 131, active=lambda t: t >= 20)
    # Default activity box is PLACE, so move the target away from it.
    mark = marks([scenario(
        "box_placed", "change", 10, 130, kind="appeared", protocol=120,
        change_done=10, clear=20, targets=[target(1, FAR)],
    )])
    report = _score(messages, mark)
    assert _row(report, "box_placed")["detection"] == "miss"
    assert report["miss_with_activity_observed"] == 1
    assert report["change_as_activity"] == 0


def test_activity_on_the_target_after_clear_is_change_as_activity():
    messages = stream(9, 131, active=lambda t: t >= 20)
    mark = marks([scenario(
        "box_placed", "change", 10, 130, kind="appeared", protocol=120,
        change_done=10, clear=20, targets=[target(1, PLACE)],
    )])
    report = _score(messages, mark)
    assert report["misses"] == 1
    assert report["miss_with_activity_observed"] == 1
    assert report["change_as_activity"] == 1


def test_placement_motion_before_clear_is_not_change_as_activity():
    messages = stream(9, 131, active=lambda t: 10 <= t < 20)
    mark = marks([scenario(
        "box_placed", "change", 10, 130, kind="appeared", protocol=120,
        change_done=10, clear=20, targets=[target(1, PLACE)],
    )])
    report = _score(messages, mark)
    assert report["miss_with_activity_observed"] == 1
    assert report["change_as_activity"] == 0


def test_split_move_needs_the_source_and_does_not_consume_strangers():
    pair = [
        event(60, "appeared", 1, MOVE_DEST),
        event(61, "disappeared", 2, MOVE_SOURCE),
    ]
    messages = stream(9, 131, pair)
    unlabeled = marks([scenario(
        "box_moved", "change", 10, 130, kind="moved", protocol=120, change_done=10, clear=20,
    )])
    raw = _score(messages, unlabeled)
    assert raw["split_move"] == 0
    assert raw["misses"] == 0
    assert _row(raw, "box_moved")["detection"] == "unattributed"
    assert raw["false_alarms"] == 2

    labeled = marks([scenario(
        "box_moved", "change", 10, 130, kind="moved", protocol=120, change_done=10, clear=20,
        targets=[target(1, MOVE_DEST, from_centroid=MOVE_FROM, from_aabb=MOVE_SOURCE)],
    )])
    scored = _score(messages, labeled)
    assert _row(scored, "box_moved")["detection"] == "miss"
    assert scored["split_move"] == 1
    assert scored["false_alarms"] == 0
    assert scored["misses"] == 1

    stranger = stream(9, 131, pair + [event(62, "appeared", 3, FAR)])
    extra = _score(stranger, labeled)
    assert extra["split_move"] == 1
    assert extra["false_alarms"] == 1


def test_moved_hit_uses_from_centroid_and_stores_it():
    moved = event(60, "moved", 7, MOVE_DEST, from_centroid=MOVE_FROM)
    messages = stream(9, 131, [moved])
    mark = marks([scenario(
        "box_moved", "change", 10, 130, kind="moved", protocol=120, change_done=10, clear=20,
        targets=[target(1, MOVE_DEST, from_centroid=MOVE_FROM)],
    )])
    report = _score(messages, mark)
    assert _row(report, "box_moved")["detection"] == "true_detection"
    assert report["candidates"][0]["from_centroid"] == MOVE_FROM

    missed = stream(9, 131, [event(60, "moved", 7, MOVE_DEST, from_centroid=[8.0, 0.5, 0.5])])
    assert _row(_score(missed, mark), "box_moved")["detection"] == "miss"


def test_operator_association_is_a_true_detection():
    messages = stream(9, 131, [event(60, "appeared", 7, FAR)])
    mark = marks([scenario(
        "box_placed", "change", 10, 130, kind="appeared", protocol=120, change_done=10, clear=20,
        associations=[{"publisher_generation": 4, "event_id": 7, "role": "target", "sensor": 1}],
    )])
    report = _score(messages, mark)
    assert _row(report, "box_placed")["detection"] == "true_detection"
    assert report["false_alarms"] == 0


def test_person_without_activity_is_unverified():
    messages = stream(9, 50)
    mark = marks([scenario("person_walk", "activity", 10, 40, protocol=105, planned=105, actual=30,
                           completion="shortened")])
    report = _score(messages, mark)
    row = _row(report, "person_walk")
    assert row["discrimination"] == "person_unverified"
    assert report["person_unverified"] == 1
    assert report["g5_acceptance"] == "denied"
    assert report["candidate_result"] == "inconclusive"


def test_walk_with_activity_and_no_candidate_is_observed_discrimination():
    messages = stream(9, 50, active=lambda t: 10 <= t < 25)
    mark = marks([scenario("person_walk", "activity", 10, 40, protocol=105, actual=30, planned=105,
                           completion="shortened")])
    report = _score(messages, mark)
    assert _row(report, "person_walk")["discrimination"] == "activity_without_candidate"
    assert report["activity_as_change"] == 0
    assert report["g5_acceptance"] == "denied"


def test_candidate_on_a_person_target_is_activity_as_change():
    messages = stream(9, 50, [event(20, "appeared", 1, PERSON)], active=lambda t: True)
    mark = marks([scenario(
        "person_sit", "activity", 10, 40, protocol=120, actual=30, planned=120, completion="shortened",
        targets=[target(1, PERSON)],
    )])
    report = _score(messages, mark)
    assert _row(report, "person_sit")["discrimination"] == "activity_as_change"
    assert report["activity_as_change"] == 1
    assert report["false_alarms"] == 1


def test_candidate_during_a_person_scenario_without_a_region_is_not_a_proven_confusion():
    messages = stream(9, 50, [event(20, "appeared", 1, FAR)], active=lambda t: True)
    mark = marks([scenario("person_sit", "activity", 10, 40, protocol=120, actual=30, completion="shortened")])
    report = _score(messages, mark)
    assert _row(report, "person_sit")["discrimination"] == "candidate_during_activity"
    assert report["activity_as_change"] == 0
    assert report["candidate_during_activity"] == 1
    assert report["false_alarms"] == 1


def test_expired_and_confirmed_are_not_second_detections():
    messages = stream(9, 131, [
        event(60, "appeared", 4, PLACE, state="candidate"),
        event(80, "appeared", 4, PLACE, state="expired"),
        event(90, "appeared", 4, PLACE, state="confirmed"),
    ])
    mark = marks([scenario(
        "box_placed", "change", 10, 130, kind="appeared", protocol=120,
        change_done=10, clear=20, targets=[target(1, PLACE)],
    )])
    report = _score(messages, mark)
    assert report["true_detections"] == 1
    assert report["false_alarms"] == 0
    assert report["confirmed_count"] == 1
    assert report["ignored_transitions"] >= 2
    assert report["confirmed_product_bar"] == "not_evaluated"
    assert report["g5_acceptance"] != "full_product"


def test_late_candidate_is_a_miss_and_an_idle_alarm_once():
    messages = stream(9, 160, [event(145, "appeared", 1, PLACE)])
    mark = marks([
        scenario("box_placed", "change", 10, 130, kind="appeared", protocol=120,
                 change_done=10, clear=20, targets=[target(1, PLACE)]),
        scenario("baseline_idle", "idle", 131, 150, protocol=60, actual=19, planned=60, completion="shortened"),
    ])
    report = _score(messages, mark)
    assert _row(report, "box_placed")["detection"] == "miss"
    assert report["false_alarms"] == 1
    assert _row(report, "baseline_idle")["false_alarms"] == 1
    assert sum(row["false_alarms"] for row in report["scenarios"]) == 1


def test_incompatible_reference_blocks_changes_without_calling_them_misses():
    messages = stream(9, 131, [event(60, "disappeared", 1, PLACE)])
    setup = verified_setup(placement_is_removal=True, verified=False, operator_confirmed=False)
    mark = marks([scenario(
        "box_removed", "change", 10, 130, kind="disappeared", protocol=120,
        change_done=10, clear=20, targets=[target(1, PLACE)],
    )], setup=setup)
    report = _score(messages, mark)
    assert _row(report, "box_removed")["detection"] == "blocked"
    assert report["misses"] == 0
    assert report["true_detections"] == 0
    assert report["reference_compatible"] is False
    assert report["g5_acceptance"] == "denied"


def test_tracking_alone_does_not_verify_the_reference():
    messages = stream(9, 40)
    setup = verified_setup(operator_confirmed=False, verified=False, tracking_seen=True)
    mark = marks([scenario("box_placed", "change", 10, 30, kind="appeared", protocol=120,
                           actual=20, completion="shortened", targets=[target(1, PLACE)])], setup=setup)
    report = _score(messages, mark)
    assert report["reference_compatible"] is False
    assert _row(report, "box_placed")["detection"] == "blocked"


def test_stale_latched_status_is_not_exposure():
    receipt = 2_000_000_000
    messages = [
        {"schema": "room-memory-acceptance-log-v1", "op": "ready", "receipt_mono_ns": 1_000_000_000,
         "callback_mono_ns": 1_000_000_000, "topics": ["/room_memory/status", "/room_memory/events", "/room_memory/activity"],
         "matched": True},
        latched_sample(2, stamp_s=10),
        {"schema": "room-memory-acceptance-log-v1", "op": "session_end", "receipt_mono_ns": 5_000_000_000,
         "reason": "complete", "detail": ""},
    ]
    assert messages[1]["receipt_mono_ns"] == receipt
    mark = marks([scenario("hour_idle", "idle", 2, 3602, protocol=3600, planned=3600, actual=3600)])
    report = _score(messages, mark)
    assert _row(report, "hour_idle")["coverage"] == "unverifiable"
    assert "stale_or_unfresh" in _row(report, "hour_idle")["coverage_reasons"] or report["credited_idle_exposure_s"] == 0
    assert report["credited_idle_exposure_s"] == 0
    assert report["idle_false_candidate_alarms_per_hour"] is None
    assert report["exit_code"] == 2
    assert report["clock"]["synchronized"] is False
    assert report["g5_acceptance"] == "denied"


def test_missing_event_tail_is_not_a_clean_hour():
    messages = stream(0, 30, [event(10, "appeared", 1, FAR), event(20, "appeared", 2, FAR, omit=True)])
    mark = marks([scenario("hour_idle", "idle", 1, 29, protocol=3600, planned=3600, actual=28)])
    report = _score(messages, mark)
    assert _row(report, "hour_idle")["coverage"] == "unverifiable"
    assert "events_seq_gap" in _row(report, "hour_idle")["coverage_reasons"]
    assert report["credited_idle_exposure_s"] == 0
    assert report["idle_false_candidate_alarms_per_hour"] is None
    assert report["exit_code"] != 0


def test_clock_offset_does_not_rewrite_the_delay():
    messages = stream(
        9, 131, [event(60, "appeared", 1, PLACE)], stamp_offset_s=5000,
        latched=latched_sample(8.5, stamp_s=8.5 - 990),
    )
    mark = marks([scenario(
        "box_placed", "change", 10, 130, kind="appeared", protocol=120,
        change_done=10, clear=20, targets=[target(1, PLACE)],
    )])
    report = _score(messages, mark)
    assert _row(report, "box_placed")["receipt_delay_s"] == 50
    wrong = (60 + 5000) - (10 + (8.5 - 990 - 8.5))
    assert wrong == 6040
    assert report["receipt_delay_s"]["p50"] == 50
    assert wrong not in report["receipt_delay_s"]["values"]
    assert report["clock"]["note"]
    assert report["detection_delay_basis"] == "receipt_mono"


def test_sensor_dropout_is_unverifiable():
    def hz_at(t):
        return (10, 0) if t == 15 else (10, 10)

    messages = stream(0, 30, hz_at=hz_at)
    mark = marks([scenario("baseline_idle", "idle", 1, 29, protocol=60, actual=28, planned=60, completion="shortened")])
    report = _score(messages, mark)
    assert _row(report, "baseline_idle")["coverage"] == "unverifiable"
    assert "sensor_dropout" in _row(report, "baseline_idle")["coverage_reasons"]
    assert report["credited_idle_exposure_s"] == 0
    assert report["idle_false_candidate_alarms_per_hour"] is None


def test_interrupted_idle_keeps_partial_exposure_and_denies_g5():
    messages = stream(0, 100, session_end_s=100, session_reason="aborted")
    mark = marks([scenario("hour_idle", "idle", 1, 100, protocol=3600, planned=3600, actual=99, completion="aborted")])
    report = _score(messages, mark)
    assert _row(report, "hour_idle")["coverage"] == "verifiable"
    assert report["credited_idle_exposure_s"] == 99
    assert report["g5_acceptance"] == "denied"
    assert report["exit_code"] == 2
    assert report["protocol_complete"] is False

    claimed = marks([scenario("hour_idle", "idle", 1, 3601, protocol=3600, planned=3600, actual=3600)])
    truncated = stream(0, 100, session_end_s=100, session_reason="aborted")
    denied = _score(truncated, claimed)
    assert _row(denied, "hour_idle")["coverage"] == "unverifiable"
    assert "log_ended_before_interval" in _row(denied, "hour_idle")["coverage_reasons"]
    assert denied["credited_idle_exposure_s"] == 0


def test_shortened_sit_is_not_full_acceptance():
    messages = stream(0, 40, active=lambda t: True, session_end_s=50)
    mark = marks([scenario("person_sit", "activity", 1, 31, protocol=120, planned=120, actual=30, completion="shortened")])
    report = _score(messages, mark)
    row = _row(report, "person_sit")
    assert row["discrimination"] == "activity_without_candidate"
    assert row["actual_duration_s"] == 30
    assert row["protocol_duration_s"] == 120
    assert report["g5_acceptance"] == "denied"
    assert report["exit_code"] == 2


def test_idle_rate_uses_only_verifiable_exposure_and_includes_one_alarm():
    one = stream(0, 3601, [event(100, "appeared", 1, FAR)])
    mark = marks([scenario("hour_idle", "idle", 1, 3601, protocol=3600, planned=3600, actual=3600)])
    report = _score(one, mark)
    assert report["false_alarms"] == 1
    assert report["credited_idle_exposure_s"] == 3600
    assert report["idle_false_candidate_alarms_per_hour"] == 1
    assert report["idle_rate_pass"] is True

    two = stream(0, 3601, [event(100, "appeared", 1, FAR), event(200, "appeared", 2, FAR)])
    failed = _score(two, mark)
    assert failed["idle_false_candidate_alarms_per_hour"] == 2
    assert failed["idle_rate_pass"] is False


def test_generation_change_blocks_the_interval_and_reused_ids():
    def generation_at(t):
        return 4 if t < 20 else 5

    messages = stream(0, 40, [
        event(10, "appeared", 1, PLACE, generation=4),
        event(30, "appeared", 1, PLACE, generation=5),
    ], generation_at=generation_at)
    mark = marks([scenario(
        "box_placed", "change", 1, 39, kind="appeared", protocol=120, actual=38, completion="shortened",
        change_done=1, clear=2, targets=[target(1, PLACE)],
    )])
    report = _score(messages, mark)
    assert _row(report, "box_placed")["coverage"] == "unverifiable"
    assert "generation_change" in _row(report, "box_placed")["coverage_reasons"]
    assert report["true_detections"] == 0
    assert len(report["candidates"]) == 2


def test_status_seq_gap_is_unverifiable():
    messages = stream(0, 20, status_seq_jump_at={10})
    mark = marks([scenario("baseline_idle", "idle", 1, 19, protocol=60, actual=18, completion="shortened")])
    report = _score(messages, mark)
    assert "status_seq_gap" in _row(report, "baseline_idle")["coverage_reasons"]
    assert report["seq_gaps"] >= 1
    assert report["credited_idle_exposure_s"] == 0


def test_not_tracking_is_inconclusive():
    messages = stream(0, 30, state="building")
    mark = marks([scenario(
        "box_placed", "change", 1, 29, kind="appeared", protocol=120, actual=28, completion="shortened",
        targets=[target(1, PLACE)],
    )])
    report = _score(messages, mark)
    assert _row(report, "box_placed")["detection"] == "inconclusive"
    assert report["misses"] == 0
    assert report["exit_code"] == 2
    assert report["idle_false_candidate_alarms_per_hour"] is None


def test_dropped_windows_during_the_hour_are_not_exposure():
    # Positive Hz and continuous status/activity, but every window of the hour is dropped.
    messages, mark = _perfect(
        state=lambda t: "degraded" if t >= 780 else "tracking",
        dropped_at=lambda t: max(0, t - 779),
    )
    report = _score(messages, mark)
    hour = _row(report, "hour_idle")
    assert hour["coverage"] == "unverifiable"
    assert hour["coverage_reasons"] == ["dropped_windows"]
    assert hour["dropped_windows"] == 3600
    assert report["dropped_windows"] == 3600
    assert report["lost_window_scenarios"] == ["hour_idle"]
    assert report["credited_idle_exposure_s"] == 60
    assert report["protocol_complete"] is False
    assert report["g5_acceptance"] == "denied"
    assert report["exit_code"] == 2
    assert "lost_window_scenarios: hour_idle" in report["markdown"]


def test_dropped_windows_make_a_change_inconclusive_not_a_miss():
    messages, mark = _perfect(drop_place=True, dropped_at=lambda t: 0 if t < 150 else 1)
    report = _score(messages, mark)
    placed = _row(report, "box_placed")
    assert placed["detection"] == "inconclusive"
    assert "dropped_windows" in placed["coverage_reasons"]
    assert report["misses"] == 0
    assert report["lost_window_scenarios"] == ["box_placed"]
    assert report["g5_acceptance"] == "denied"
    assert report["exit_code"] == 2


def test_dropped_windows_outside_the_protocol_scenarios_still_deny():
    messages, mark = _perfect(dropped_at=lambda t: 0 if t < 740 else 1)
    report = _score(messages, mark)
    assert _row(report, "restore_reference")["coverage_reasons"] == ["dropped_windows"]
    assert _row(report, "hour_idle")["coverage"] == "verifiable"
    assert report["protocol_complete"] is True
    assert report["lost_window_scenarios"] == ["restore_reference"]
    assert report["g5_acceptance"] == "denied"
    assert report["exit_code"] == 2


def test_a_restart_inside_an_interval_does_not_hide_dropped_windows():
    # The restart breaks the status chain, so restore_reference is unverifiable for
    # its generation change. The new process still drops ten windows after it.
    messages, mark = _perfect(
        generation_at=lambda t: 4 if t < 740 else 5,
        dropped_at=lambda t: 7 if t < 740 else max(0, min(10, t - 740)),
    )
    report = _score(messages, mark)
    restore = _row(report, "restore_reference")
    assert "generation_change" in restore["coverage_reasons"]
    assert "dropped_windows" in restore["coverage_reasons"]
    assert restore["dropped_windows"] == 10
    assert _row(report, "hour_idle")["coverage"] == "verifiable"
    assert report["dropped_windows"] == 10
    assert report["lost_window_scenarios"] == ["restore_reference"]
    assert report["g5_acceptance"] == "denied"
    assert report["exit_code"] == 2

    # Drops before the restart, in a generation with no closing sample, count too.
    before = _score(*_perfect(
        generation_at=lambda t: 4 if t < 740 else 5,
        dropped_at=lambda t: (7 if t < 715 else 9) if t < 740 else 0,
    ))
    assert _row(before, "restore_reference")["dropped_windows"] == 2
    assert before["lost_window_scenarios"] == ["restore_reference"]
    assert before["g5_acceptance"] == "denied"

    # A new process that drops before its first status still counts from 0.
    first = _score(*_perfect(
        generation_at=lambda t: 4 if t < 740 else 5,
        dropped_at=lambda t: 7 if t < 740 else 3,
    ))
    assert _row(first, "restore_reference")["dropped_windows"] == 3
    assert first["g5_acceptance"] == "denied"

    # A clean restart inside the interval is unverifiable but loses no windows.
    clean = _score(*_perfect(
        generation_at=lambda t: 4 if t < 740 else 5,
        dropped_at=lambda t: 7 if t < 740 else 0,
    ))
    assert _row(clean, "restore_reference")["dropped_windows"] == 0
    assert clean["lost_window_scenarios"] == []
    assert clean["g5_acceptance"] == "full_candidate"


def test_an_unchanged_historical_dropped_count_keeps_coverage():
    steady = _score(*_perfect(dropped_at=lambda t: 7))
    assert steady["g5_acceptance"] == "full_candidate"
    assert steady["credited_idle_exposure_s"] == 3660
    assert steady["dropped_windows"] == 0
    assert steady["session_dropped_windows"] == 0
    assert steady["lost_window_scenarios"] == []

    # A restart between intervals resets the counter in a new generation.
    restarted = _score(*_perfect(
        generation_at=lambda t: 4 if t < 775 else 5,
        dropped_at=lambda t: 7 if t < 775 else 0,
    ))
    assert _row(restarted, "hour_idle")["coverage"] == "verifiable"
    assert restarted["g5_acceptance"] == "full_candidate"
    assert restarted["session_dropped_windows"] == 0
    assert restarted["credited_idle_exposure_s"] == 3660


def test_a_drop_in_the_gap_between_scenarios_is_not_full_acceptance():
    # t=705 is after person_sit (ends 700) and before restore_reference (starts 710).
    # Each interval's baseline is the last sample at or before its own t0, so neither
    # interval sees the step. Exposure stays. The session total does not.
    report = _score(*_perfect(dropped_at=lambda t: 0 if t < 705 else 1))
    assert report["g5_acceptance"] != "full_candidate"
    assert report["g5_acceptance"] == "denied"
    assert report["exit_code"] == 2
    assert report["candidate_result"] == "inconclusive"
    assert report["protocol_complete"] is True
    assert report["session_dropped_windows"] == 1
    assert report["session_window_reasons"] == ["dropped_windows"]
    assert report["dropped_windows"] == 0
    assert report["lost_window_scenarios"] == []
    assert report["credited_idle_exposure_s"] == 3660
    assert _row(report, "person_sit")["coverage"] == "verifiable"
    assert _row(report, "person_sit")["dropped_windows"] == 0
    assert _row(report, "restore_reference")["coverage"] == "verifiable"
    assert _row(report, "restore_reference")["dropped_windows"] == 0
    assert "session_dropped_windows: 1" in report["markdown"]


def test_a_drop_during_setup_is_not_full_acceptance():
    # ready at t=7, first watch at t=10. The counter steps at t=9, while the
    # operator is still confirming the reference. baseline_idle's baseline is the
    # sample at t=10, which already includes the drop.
    report = _score(*_perfect(dropped_at=lambda t: 0 if t < 9 else 2))
    assert report["g5_acceptance"] != "full_candidate"
    assert report["exit_code"] == 2
    assert report["session_dropped_windows"] == 2
    assert report["dropped_windows"] == 0
    assert report["lost_window_scenarios"] == []
    assert report["credited_idle_exposure_s"] == 3660
    assert report["protocol_complete"] is True
    assert _row(report, "baseline_idle")["coverage"] == "verifiable"
    assert _row(report, "baseline_idle")["dropped_windows"] == 0
    assert "dropped_windows" not in _row(report, "baseline_idle")["coverage_reasons"]


def test_interval_without_a_status_at_or_before_t0_does_not_hide_an_earlier_drop():
    # The first status is inside the interval and already reports 4 dropped windows.
    # There is no sample at or before t0, so that 4 is not a historical baseline.
    missed = _score(
        stream(5, 30, dropped_at=lambda t: 4, ready_s=0, session_end_s=31),
        marks([scenario("baseline_idle", "idle", 1, 20, protocol=60, actual=19, completion="shortened")]),
    )
    row = _row(missed, "baseline_idle")
    assert row["dropped_windows"] == 4
    assert "dropped_windows" in row["coverage_reasons"]
    assert row["coverage"] == "unverifiable"
    assert missed["g5_acceptance"] != "full_candidate"

    # The same count is history when a sample at or before t0 already shows it.
    held = _score(
        stream(0, 30, dropped_at=lambda t: 4),
        marks([scenario("baseline_idle", "idle", 1, 20, protocol=60, actual=19, completion="shortened")]),
    )
    assert _row(held, "baseline_idle")["dropped_windows"] == 0
    assert "dropped_windows" not in _row(held, "baseline_idle")["coverage_reasons"]


def test_a_dropped_count_that_regresses_or_is_missing_is_unverifiable():
    mark = marks([scenario("baseline_idle", "idle", 1, 29, protocol=60, actual=28, completion="shortened")])
    regressed = _score(stream(0, 30, dropped_at=lambda t: 5 if t < 15 else 0), mark)
    assert _row(regressed, "baseline_idle")["coverage_reasons"] == ["dropped_windows_regressed"]
    assert regressed["credited_idle_exposure_s"] == 0
    missing = _score(stream(0, 30, dropped_at=lambda t: None), mark)
    assert _row(missing, "baseline_idle")["coverage_reasons"] == ["dropped_windows_unknown"]
    assert missing["credited_idle_exposure_s"] == 0


def test_static_confirmations_count_without_a_captured_candidate():
    outside = [
        event(131, "appeared", 1, PLACE, state="confirmed"),
        event(261, "disappeared", 2, REMOVE, state="confirmed"),
        event(391, "moved", 3, MOVE_DEST, from_centroid=MOVE_FROM, state="confirmed"),
    ]
    labeled = _score(*_perfect(extra_events=outside))
    assert labeled["confirmed_count"] == 3
    assert labeled["false_alarms"] == 0
    assert labeled["confirmed_product_bar"] == "pass"
    assert labeled["g5_acceptance"] == "full_candidate"
    assert labeled["exit_code"] == 0

    # The candidates were published before the volatile subscription. Only the
    # later CONFIRMED lines are in the log, during the idle hour.
    one = _score(*_perfect(extra_events=[event(1000, "appeared", 50, FAR, state="confirmed")]))
    assert one["false_alarms"] == 0
    assert one["confirmed_count"] == 1
    assert one["confirmed_product_bar"] == "pass"
    assert one["idle_false_candidate_alarms_per_hour"] == 0

    two = _score(*_perfect(extra_events=[
        event(1000, "appeared", 50, FAR, state="confirmed"),
        event(2000, "appeared", 51, FAR, state="confirmed"),
    ]))
    assert two["false_alarms"] == 0
    assert two["confirmed_count"] == 2
    assert two["confirmed_product_bar"] == "fail"
    assert two["g5_acceptance"] == "full_candidate"
    assert "before the subscription" in two["confirmed_product_bar_note"]


def test_perfect_session_passes_candidates_and_does_not_pass_confirmed_bar():
    report = _score(*_perfect())
    assert report["exit_code"] == 0
    assert report["g5_acceptance"] == "full_candidate"
    assert report["candidate_result"] == "pass"
    assert report["session_dropped_windows"] == 0
    assert report["session_window_reasons"] == []
    assert report["confirmed_product_bar"] == "not_evaluated"
    assert report["confirmed_count"] == 0
    assert report["true_detections"] == 3
    assert report["false_alarms"] == 0
    assert report["misses"] == 0
    assert report["credited_idle_exposure_s"] == 3660
    assert report["idle_false_candidate_alarms_per_hour"] == 0
    assert _row(report, "person_walk")["discrimination"] == "activity_without_candidate"
    assert _row(report, "person_sit")["discrimination"] == "activity_without_candidate"
    assert report["receipt_delay_s"]["p50"] == 50
    assert report["receipt_delay_s"]["max"] == 50
    text = report["markdown"]
    assert "CANDIDATE" in text
    assert "not_evaluated" in text
    assert "true_detections: 3" in text
    assert "false_alarms: 0" in text
    assert "persistence_gate_start" not in text


def test_full_session_miss_fails_closed():
    messages, mark = _perfect(drop_place=True)
    report = _score(messages, mark)
    assert report["protocol_complete"] is True
    assert report["misses"] >= 1
    assert report["candidate_result"] == "fail"
    assert report["g5_acceptance"] == "denied"
    assert report["exit_code"] == 1


def test_one_idle_alarm_on_a_full_session_meets_the_candidate_threshold():
    messages, mark = _perfect(extra_idle=[event(1000, "appeared", 9, FAR)])
    report = _score(messages, mark)
    assert report["false_alarms"] == 1
    assert report["idle_false_candidate_alarms_per_hour"] == pytest_rate(1, 3660)
    assert report["idle_rate_pass"] is True
    assert report["exit_code"] == 0

    louder, louder_marks = _perfect(extra_idle=[event(1000, "appeared", 9, FAR), event(1100, "appeared", 10, FAR)])
    failed = _score(louder, louder_marks)
    assert failed["idle_rate_pass"] is False
    assert failed["exit_code"] == 1


def test_decode_roundtrip_scores_as_a_hit():
    from acceptance.decode import cdr_to_record
    from room_memory.messages import ChangeEvent, EventKind, EventState, Header, Time

    header = Header("lab", 1, 0, Time(1_700_000_000, 0))
    change = ChangeEvent(
        header, 7, EventKind.APPEARED, EventState.CANDIDATE, (0.5, 0.5, 0.5),
        (0.0, 0.0, 0.0), (1.0, 1.0, 1.0), None, 40, 0.04, None, None, 1,
        Time(1_700_000_000, 0), Time(1_700_000_000, 0), None, 4,
    )
    record = cdr_to_record(
        "/room_memory/events", change.to_cdr(), receipt_mono_ns=60 * 1_000_000_000,
        callback_mono_ns=60 * 1_000_000_000,
    )
    assert record["message"]["kind"] == "appeared"
    assert record["message"]["event_id"] == 7
    assert record["publisher_generation"] == 4
    assert record["stamp_ns"] != record["receipt_mono_ns"]
    messages = stream(9, 131)
    for message in messages:
        if message.get("topic") == "/room_memory/status" and message["receipt_mono_ns"] >= 60 * 1_000_000_000:
            message["message"]["events_seq"] = record["seq"]
    messages.append(record)
    mark = marks([scenario(
        "box_placed", "change", 10, 130, kind="appeared", protocol=120,
        change_done=10, clear=20, targets=[target(1, PLACE)],
    )])
    report = _score(messages, mark)
    assert _row(report, "box_placed")["detection"] == "true_detection"
    assert report["candidates"][0]["aabb"] == PLACE or report["candidates"][0]["aabb"] == [[0.0, 0.0, 0.0], [1.0, 1.0, 1.0]]


def pytest_rate(count, exposure_s):
    return count * 3600 / exposure_s


def _perfect(drop_place=False, extra_idle=(), extra_events=(), place_targets=None, **stream_options):
    events = []
    if not drop_place:
        events.append(event(130, "appeared", 1, PLACE))
    events.append(event(260, "disappeared", 2, REMOVE))
    events.append(event(390, "moved", 3, MOVE_DEST, from_centroid=MOVE_FROM))
    events.extend(extra_events)
    events.extend(extra_idle)
    def active(t):
        return (80 <= t < 90) or (470 <= t < 490) or (580 <= t <= 700)

    messages = stream(8, 4381, events, active=active, ready_s=7, session_end_s=4382, **stream_options)
    rows = [
        scenario("baseline_idle", "idle", 10, 70, protocol=60),
        scenario("box_placed", "change", 80, 200, kind="appeared", protocol=120, change_done=80, clear=90,
                 targets=[target(1, PLACE)] if place_targets is None else place_targets,
                 prerequisites=["reference_setup"]),
        scenario("box_removed", "change", 210, 330, kind="disappeared", protocol=120, change_done=210, clear=220,
                 targets=[target(1, REMOVE)], prerequisites=["reference_setup"]),
        scenario("box_moved", "change", 340, 460, kind="moved", protocol=120, change_done=340, clear=350,
                 targets=[target(1, MOVE_DEST, from_centroid=MOVE_FROM)], prerequisites=["reference_setup"]),
        scenario("person_walk", "activity", 470, 575, protocol=105),
        scenario("person_sit", "activity", 580, 700, protocol=120),
        scenario("restore_reference", "restoration", 710, 770, protocol=60),
        scenario("hour_idle", "idle", 780, 4380, protocol=3600),
    ]
    return messages, marks(rows)


def test_rates_are_finite():
    report = _score(*_perfect())
    assert math.isfinite(report["idle_false_candidate_alarms_per_hour"])
