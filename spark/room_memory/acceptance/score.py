"""Pure acceptance score. Stdlib only. No ROS, no clock conversion of the marks.

Detection delay is the recorder monotonic receipt of the event minus the operator's
physical-change-completion mark. Publisher stamps are not subtracted from that mark.
A latched status is not a synchronization sample: a sample is fresh only when a
neighbor about one second away has a stamp delta that matches the receipt delta.
"""

import math

from acceptance.report import render_markdown

MIN_GAP_NS = 400_000_000
MAX_GAP_NS = 2_500_000_000
STAMP_TOL_NS = 500_000_000
SYNC_BOUND_NS = 500_000_000
IOU_MIN = 0.05
PAD_M = 0.10
FROM_MATCH_M = 0.50

STATUS = "/room_memory/status"
EVENTS = "/room_memory/events"
ACTIVITY = "/room_memory/activity"
TOPICS = (STATUS, EVENTS, ACTIVITY)

REQUIRED_S = {
    "baseline_idle": 60.0,
    "box_placed": 120.0,
    "box_removed": 120.0,
    "box_moved": 120.0,
    "person_walk": 105.0,
    "person_sit": 120.0,
    "hour_idle": 3600.0,
}
IDLE_IDS = {"baseline_idle", "hour_idle"}
CHANGE_IDS = ("box_placed", "box_removed", "box_moved")
PERSON_IDS = ("person_walk", "person_sit")
SENSOR_HZ = {1: "mid360_hz", 2: "jt128_hz"}
LOST_WINDOW_REASONS = ("dropped_windows", "dropped_windows_regressed", "dropped_windows_unknown")

CONFIRMED_NOTE = (
    "No CONFIRMED message was published. false_candidate_alarms_per_hour is not the "
    "Q3 product bar, which counts false CONFIRMED events per hour."
)
CONFIRMED_PARTIAL_NOTE = (
    "CONFIRMED messages were seen, but the static intervals are not a full verifiable "
    "baseline plus hour, so the CONFIRMED product bar is not evaluated."
)
CONFIRMED_RATE_NOTE = (
    "Each CONFIRMED message received during a verifiable static interval counts, "
    "including when its candidate was published before the subscription."
)


def score(messages, marks, max_false_alarms_per_hour=1.0):
    messages = sorted(messages, key=lambda item: (item.get("receipt_mono_ns") or 0, item.get("seq") or 0))
    setup = marks.get("reference_setup") or {}
    compatible = reference_compatible(setup)
    expected = list(marks.get("expected_sensors") or [1, 2])
    meta = _meta(messages)
    fresh = _fresh(messages)
    scenarios = list(marks.get("scenarios") or [])
    coverage = [_coverage(item, messages, fresh, meta, expected) for item in scenarios]
    events, ignored, confirmed_count = _first_candidates(messages)
    rows, assigned = _score_scenarios(scenarios, coverage, events, messages, compatible)
    _assign_rest(events, assigned, rows, scenarios, coverage)
    return _finish(rows, events, assigned, messages, fresh, compatible, meta, ignored, confirmed_count,
                   max_false_alarms_per_hour)


def reference_compatible(setup):
    if not isinstance(setup, dict):
        return False
    return bool(
        setup.get("operator_confirmed")
        and setup.get("verified")
        and setup.get("placement") == "new_object"
        and setup.get("removal") == "reference_object"
        and setup.get("move_source") == "other_reference_object"
        and not setup.get("placement_is_removal")
        and not setup.get("removal_is_move_source")
    )


def _meta(messages):
    ready = None
    end = None
    for message in messages:
        if message.get("op") == "ready" and ready is None:
            ready = message
        if message.get("op") == "session_end":
            end = message
    return {
        "ready": ready,
        "end_ns": None if end is None else end.get("receipt_mono_ns"),
        "end_reason": None if end is None else end.get("reason"),
    }


def _live_pair(prev, cur):
    receipt_dt = cur["receipt_mono_ns"] - prev["receipt_mono_ns"]
    stamp_dt = cur["stamp_ns"] - prev["stamp_ns"]
    if not MIN_GAP_NS <= receipt_dt <= MAX_GAP_NS:
        return False
    if stamp_dt <= 0:
        return False
    return abs(stamp_dt - receipt_dt) <= STAMP_TOL_NS


def _fresh(messages):
    groups = {}
    for index, message in enumerate(messages):
        if message.get("topic") not in (STATUS, ACTIVITY):
            continue
        key = (message.get("topic"), message.get("publisher_generation"))
        groups.setdefault(key, []).append(index)
    fresh = set()
    for indexes in groups.values():
        indexes.sort(key=lambda index: messages[index]["receipt_mono_ns"])
        for prev, cur in zip(indexes, indexes[1:]):
            if _live_pair(messages[prev], messages[cur]):
                fresh.add(prev)
                fresh.add(cur)
    return fresh


def _coverage(scenario, messages, fresh, meta, expected):
    if scenario.get("class") == "setup" or scenario.get("completion") in ("blocked", "skipped"):
        return {"ok": False, "coverage": "not_applicable", "reasons": [scenario.get("completion") or "setup"], "gaps": 0,
                "dropped": 0}
    reasons = []
    gaps = 0
    dropped = 0
    t0 = scenario["t0_mono_ns"]
    t1 = scenario["t1_mono_ns"]
    if meta["end_ns"] is None:
        reasons.append("missing_session_end")
    elif meta["end_ns"] < t1:
        reasons.append("log_ended_before_interval")
    ready = meta["ready"]
    topics = set(ready.get("topics") or []) if ready else set()
    if not ready or not ready.get("matched") or ready.get("receipt_mono_ns", t0 + 1) > t0 or any(topic not in topics for topic in TOPICS):
        reasons.append("subscription_not_ready")
    generations = set()
    for message in messages:
        if message.get("topic") in TOPICS and t0 <= message.get("receipt_mono_ns", -1) <= t1:
            generations.add(message.get("publisher_generation"))
    if len(generations) > 1:
        reasons.append("generation_change")
    for message in messages:
        receipt = message.get("receipt_mono_ns")
        errored = message.get("op") == "error" or ("error" in message and message.get("op") not in ("ready", "session_end"))
        if errored and receipt is not None and t0 <= receipt <= t1:
            reasons.append("decode_error")
            break
    status_chain, status_reasons, status_gaps = _chain(messages, fresh, STATUS, "status", t0, t1)
    activity_chain, activity_reasons, activity_gaps = _chain(messages, fresh, ACTIVITY, "activity", t0, t1)
    reasons.extend(status_reasons)
    reasons.extend(activity_reasons)
    gaps += status_gaps + activity_gaps
    if status_chain:
        for sample in status_chain:
            state = (sample.get("message") or {}).get("state")
            if state not in ("tracking", "degraded"):
                reasons.append("not_tracking")
                break
        for sample in status_chain:
            body = sample.get("message") or {}
            for sensor in expected:
                hz = body.get(SENSOR_HZ.get(sensor, ""))
                if not isinstance(hz, int) or hz <= 0:
                    reasons.append("sensor_dropout")
                    break
            else:
                continue
            break
        _event_coverage(messages, status_chain[0], status_chain[-1], t0, t1, reasons)
    dropped = _dropped_windows(messages, t0, t1, reasons)
    if not status_chain and not any(reason.startswith("no_fresh") or reason == "stale_or_unfresh" for reason in reasons):
        reasons.append("stale_or_unfresh")
    unique = []
    for reason in reasons:
        if reason not in unique:
            unique.append(reason)
    return {"ok": not unique, "coverage": "verifiable" if not unique else "unverifiable", "reasons": unique, "gaps": gaps,
            "dropped": dropped}


def _session_dropped_windows(messages, meta, reasons):
    """Change in ``dropped_windows`` from ready through ``session_end``.

    Per publisher generation. A drop during setup or between scenarios sits in
    neither interval, so the per-interval total stays zero and that interval
    keeps its exposure. Full acceptance still requires this session change to
    be zero. The subscriber writes ready before the latched status arrives, so
    a generation already running often has no sample at or before ready. Its
    baseline is then its first status at or after ready: a lifetime count
    already stored there is not a drop during the session. The last status at
    or before ready is the baseline when one exists. The counter is measured
    from 0 only for a generation that first appears after an earlier generation
    was already seen in the span, which is a restart during the session. The
    span stops at ``session_end`` and does not take a sample from after it.
    """
    ready = meta.get("ready") if isinstance(meta.get("ready"), dict) else None
    ready_ns = None if ready is None else ready.get("receipt_mono_ns")
    end_ns = meta.get("end_ns")
    if _bad_stamp(ready_ns) or _bad_stamp(end_ns) or end_ns < ready_ns:
        reasons.append("dropped_windows_unknown")
        return 0
    return _dropped_windows(messages, ready_ns, end_ns, reasons, close_after=False, session_span=True)


def _bad_stamp(value):
    return not isinstance(value, int) or isinstance(value, bool)


def _dropped_windows(messages, t0, t1, reasons, close_after=True, session_span=False):
    """Windows the node dropped from ``t0`` to ``t1``, per publisher generation.

    ``dropped_windows`` counts since node start, so only an increase is new. The
    baseline is the last status sample at or before ``t0``. For an interval, if
    there is no such sample, the first sample inside it is not the baseline: the
    counter is measured from 0, so a drop already stored on that sample is counted.
    A later generation starts at 0, so a restart's reset is not a decrease.

    ``session_span`` is the ready-to-``session_end`` read. The subscriber writes
    ready before the latched status arrives. A generation with no sample at or
    before ready then uses its first status at or after ready as the baseline,
    unless it first appears after an earlier generation was already seen in the
    span. That later generation is a restart during the session and is measured
    from 0, so drops before its first status still count. A sample at or before
    ready is still that generation's baseline when one exists.

    This does not depend on a coverage chain. Positive Hz does not cover it:
    both publishers count rejected frames toward Hz.
    """
    samples = [
        message for message in messages
        if message.get("topic") == STATUS and isinstance(message.get("message"), dict)
        and message.get("receipt_mono_ns") is not None
    ]
    samples.sort(key=lambda item: item["receipt_mono_ns"])
    generations = []
    for sample in samples:
        generation = sample.get("publisher_generation")
        if t0 <= sample["receipt_mono_ns"] <= t1 and generation not in generations:
            generations.append(generation)
    total = 0
    unknown = False
    regressed = False
    seen_earlier = False
    for generation in generations:
        own = [sample for sample in samples if sample.get("publisher_generation") == generation]
        before = [sample for sample in own if sample["receipt_mono_ns"] <= t0]
        if close_after:
            after = [sample for sample in own if sample["receipt_mono_ns"] >= t1]
            hi = after[0]["receipt_mono_ns"] if after else t1
        else:
            hi = t1
        lo = before[-1]["receipt_mono_ns"] if before else t0
        run = [sample for sample in own if lo <= sample["receipt_mono_ns"] <= hi]
        # Interval: no sample at or before t0 means measure from 0.
        # Session: the first generation's first in-span status is the baseline
        # (ready is recorded before the latch). A generation that shows up
        # after another was already seen in the span is a restart and starts at 0.
        if before or (session_span and not seen_earlier):
            previous = None
        else:
            previous = 0
        seen_earlier = True
        for sample in run:
            value = sample["message"].get("dropped_windows")
            if not isinstance(value, int) or isinstance(value, bool) or value < 0:
                unknown = True
                break
            if previous is not None:
                if value < previous:
                    regressed = True
                elif value > previous:
                    total += value - previous
            previous = value
    if unknown:
        reasons.append("dropped_windows_unknown")
    if regressed:
        reasons.append("dropped_windows_regressed")
    if total:
        reasons.append("dropped_windows")
    return total


def _chain(messages, fresh, topic, label, t0, t1):
    samples = [messages[index] for index in sorted(fresh) if messages[index].get("topic") == topic]
    samples.sort(key=lambda item: item["receipt_mono_ns"])
    reasons = []
    gaps = 0
    if not samples:
        return None, [f"no_fresh_{label}", "stale_or_unfresh"], 0
    start = None
    for sample in samples:
        if sample["receipt_mono_ns"] <= t0:
            start = sample
    if start is None or t0 - start["receipt_mono_ns"] > MAX_GAP_NS:
        return None, [f"no_fresh_{label}", "stale_or_unfresh"], 0
    closing = None
    for sample in samples:
        if sample["receipt_mono_ns"] >= t1 and sample.get("publisher_generation") == start.get("publisher_generation"):
            closing = sample
            break
    if closing is None or closing["receipt_mono_ns"] - t1 > MAX_GAP_NS:
        return None, ["missing_tail"], 0
    chain = [
        sample for sample in samples
        if start["receipt_mono_ns"] <= sample["receipt_mono_ns"] <= closing["receipt_mono_ns"]
        and sample.get("publisher_generation") == start.get("publisher_generation")
    ]
    if not chain or chain[-1] is not closing:
        return None, ["missing_tail"], 0
    previous = None
    for sample in chain:
        if previous is not None:
            if sample["receipt_mono_ns"] - previous["receipt_mono_ns"] > MAX_GAP_NS:
                reasons.append(f"{label}_gap")
            if sample["seq"] != previous["seq"] + 1:
                reasons.append(f"{label}_seq_gap")
                gaps += 1
        previous = sample
    if reasons:
        return chain, reasons, gaps
    return chain, [], gaps


def _event_coverage(messages, start, closing, t0, t1, reasons):
    generation = start.get("publisher_generation")
    start_seq = (start.get("message") or {}).get("events_seq")
    tail_seq = (closing.get("message") or {}).get("events_seq")
    events = [
        message for message in messages
        if message.get("topic") == EVENTS and message.get("publisher_generation") == generation
    ]
    if tail_seq is None and start_seq is None:
        if any(t0 <= message["receipt_mono_ns"] <= t1 for message in events):
            reasons.append("events_without_seq")
        return
    if tail_seq is None or (start_seq is not None and tail_seq < start_seq):
        reasons.append("events_seq_regressed")
        return
    begin = 0 if start_seq is None else start_seq + 1
    required = set(range(begin, int(tail_seq) + 1))
    seen = {}
    for message in events:
        seen[message["seq"]] = seen.get(message["seq"], 0) + 1
    if any(seen.get(seq, 0) != 1 for seq in required):
        reasons.append("events_seq_gap")
    if any(message["receipt_mono_ns"] <= t1 and message["seq"] > tail_seq for message in events):
        reasons.append("missing_tail")


def interval_tail_ready(messages, t0_mono_ns, t1_mono_ns):
    """True when this interval already has the samples the scorer needs after ``t1``.

    The closing status and the closing activity are the first fresh samples at or
    after ``t1`` and within the coverage gap. The event tail is the ``events_seq``
    that closing status advertises. A later read cannot invent either one.
    """
    if t0_mono_ns is None or t1_mono_ns is None:
        return False
    fresh = _fresh(messages)
    status_chain, _, _ = _chain(messages, fresh, STATUS, "status", t0_mono_ns, t1_mono_ns)
    activity_chain, _, _ = _chain(messages, fresh, ACTIVITY, "activity", t0_mono_ns, t1_mono_ns)
    if not status_chain or not activity_chain:
        return False
    return _advertised_events_present(messages, status_chain[-1])


def _advertised_events_present(messages, closing_status):
    tail = (closing_status.get("message") or {}).get("events_seq")
    if tail is None:
        return True
    generation = closing_status.get("publisher_generation")
    tail = int(tail)
    for message in messages:
        if message.get("topic") != EVENTS or message.get("publisher_generation") != generation:
            continue
        if message.get("seq") == tail:
            return True
    return False


def _first_candidates(messages):
    ignored = 0
    confirmed = 0
    seen = {}
    ordered = [message for message in messages if message.get("topic") == EVENTS]
    ordered.sort(key=lambda message: (message["receipt_mono_ns"], message.get("seq") or 0))
    for message in ordered:
        state = (message.get("message") or {}).get("state")
        if state == "confirmed":
            confirmed += 1
        if state != "candidate":
            ignored += 1
            continue
        key = (message.get("publisher_generation"), (message.get("message") or {}).get("event_id"))
        if key in seen:
            ignored += 1
            continue
        seen[key] = message
    return list(seen.values()), ignored, confirmed


def _score_scenarios(scenarios, coverage, events, messages, compatible):
    rows = []
    assigned = {}
    order = sorted(range(len(scenarios)), key=lambda index: scenarios[index]["t0_mono_ns"])
    for index in order:
        scenario = scenarios[index]
        cover = coverage[index]
        row = _blank_row(scenario, cover)
        if scenario.get("class") == "setup" or scenario.get("completion") in ("blocked", "skipped"):
            if scenario.get("class") == "change":
                row["detection"] = "blocked"
            rows.append((index, row))
            continue
        if scenario.get("class") == "change":
            if not compatible:
                row["detection"] = "blocked"
            elif not cover["ok"]:
                row["detection"] = "inconclusive"
            else:
                _score_change(scenario, row, events, assigned)
        elif scenario.get("class") == "activity":
            if not cover["ok"]:
                row["discrimination"] = "inconclusive"
            else:
                _score_person(scenario, row, events, messages)
        rows.append((index, row))
    rows.sort(key=lambda item: item[0])
    return [row for _, row in rows], assigned


def _blank_row(scenario, cover):
    return {
        "id": scenario["id"],
        "class": scenario.get("class"),
        "completion": scenario.get("completion"),
        "protocol_duration_s": scenario.get("protocol_duration_s"),
        "planned_duration_s": scenario.get("planned_duration_s"),
        "actual_duration_s": scenario.get("actual_duration_s"),
        "coverage": cover["coverage"],
        "coverage_reasons": list(cover["reasons"]),
        "dropped_windows": cover["dropped"],
        "detection": "none",
        "discrimination": "none",
        "receipt_delay_s": None,
        "clear_to_event_s": None,
        "clear_to_event_basis": "not_persistence_gate",
        "false_alarms": 0,
        "activity_observed": False,
        "miss_with_activity_observed": False,
        "change_as_activity": False,
        "prerequisite_met": scenario.get("class") != "change" or scenario.get("completion") != "blocked",
        "_gaps": cover["gaps"],
        "_scenario": scenario,
    }


def _in_window(event, scenario):
    receipt = event["receipt_mono_ns"]
    return scenario["t0_mono_ns"] <= receipt <= scenario["t1_mono_ns"]


def _in_match(event, scenario):
    start = scenario.get("change_done_mono_ns")
    if start is None or scenario.get("class") != "change":
        start = scenario["t0_mono_ns"]
    return start <= event["receipt_mono_ns"] <= scenario["t1_mono_ns"]


def _score_change(scenario, row, events, assigned):
    kind = scenario.get("kind")
    targets = list(scenario.get("targets") or [])
    associations = [item for item in (scenario.get("associations") or []) if item.get("role", "target") == "target"]
    attributed = bool(targets or associations)
    window = [event for event in events if _in_window(event, scenario) and id(event) not in assigned]
    if not attributed:
        same = [event for event in window if (event.get("message") or {}).get("kind") == kind]
        for event in same:
            assigned[id(event)] = _disposition(event, "temporal_match", scenario["id"])
        for event in window:
            if id(event) not in assigned:
                assigned[id(event)] = _disposition(event, "false_alarm", scenario["id"])
        row["detection"] = "temporal_match" if same else "unattributed"
        return
    matched = [
        event for event in window
        if _in_match(event, scenario) and _matches_change(event, scenario, targets, associations)
    ]
    matched.sort(key=lambda event: event["receipt_mono_ns"])
    if matched:
        primary = matched[0]
        assigned[id(primary)] = _disposition(primary, "true_detection", scenario["id"])
        credited = {}
        primary_sensor = (primary.get("message") or {}).get("sensors")
        _credit(credited, _match_keys(primary, scenario, targets, associations), primary_sensor)
        for other in matched[1:]:
            sensor = (other.get("message") or {}).get("sensors")
            fresh = [
                key for key in _match_keys(other, scenario, targets, associations)
                if sensor not in credited.get(key, ())
            ]
            # A sensor corroborates each labeled target once. The next candidate
            # from that sensor is a separate detection, not more evidence.
            if sensor != primary_sensor and fresh:
                assigned[id(other)] = _disposition(other, "corroboration", scenario["id"])
                _credit(credited, fresh, sensor)
            else:
                assigned[id(other)] = _disposition(other, "false_alarm", scenario["id"])
        for event in window:
            if id(event) not in assigned:
                assigned[id(event)] = _disposition(event, "false_alarm", scenario["id"])
        row["detection"] = "true_detection"
        _delays(row, scenario, primary)
        return
    split = _split_pair(window, targets, associations)
    if split is not None:
        for event in split:
            assigned[id(event)] = _disposition(event, "split_move", scenario["id"])
        row["detection"] = "miss"
    else:
        row["detection"] = "miss"
    for event in window:
        if id(event) not in assigned:
            assigned[id(event)] = _disposition(event, "false_alarm", scenario["id"])


def _score_person(scenario, row, events, messages):
    activities = _activities_in(scenario, messages)
    row["activity_observed"] = any((item.get("message") or {}).get("active") for item in activities)
    window = [event for event in events if _in_window(event, scenario)]
    targets = list(scenario.get("targets") or [])
    if not row["activity_observed"]:
        row["discrimination"] = "person_unverified"
        return
    if any(_overlaps_person(event, targets) for event in window):
        row["discrimination"] = "activity_as_change"
        return
    if window:
        row["discrimination"] = "candidate_during_activity"
        return
    row["discrimination"] = "activity_without_candidate"


def _assign_rest(events, assigned, rows, scenarios, coverage):
    by_id = {row["id"]: (row, scenarios[index], coverage[index]) for index, row in enumerate(rows)}
    for event in events:
        if id(event) in assigned:
            continue
        claim = None
        for row, scenario, cover in by_id.values():
            if not _in_window(event, scenario):
                continue
            if scenario.get("class") == "setup" or scenario.get("completion") in ("blocked", "skipped") or row["detection"] == "blocked":
                claim = ("blocked_interval", scenario["id"], False)
                continue
            if not cover["ok"]:
                claim = ("inconclusive", scenario["id"], False)
                continue
            if scenario.get("class") == "activity" and row["discrimination"] == "activity_as_change" and _overlaps_person(event, scenario.get("targets") or []):
                claim = ("activity_as_change", scenario["id"], True)
                break
            claim = ("false_alarm", scenario["id"], True)
            break
        if claim is None:
            assigned[id(event)] = _disposition(event, "false_alarm", None)
        else:
            assigned[id(event)] = _disposition(event, claim[0], claim[1])


def _activities_in(scenario, messages):
    found = []
    for message in messages:
        if message.get("topic") != ACTIVITY:
            continue
        if scenario["t0_mono_ns"] <= message["receipt_mono_ns"] <= scenario["t1_mono_ns"]:
            found.append(message)
    return found


def _credit(credited, keys, sensor):
    for key in keys:
        credited.setdefault(key, set()).add(sensor)


def _match_keys(event, scenario, targets, associations):
    body = event.get("message") or {}
    kind = scenario.get("kind")
    if body.get("kind") != kind:
        return []
    keys = []
    for index, association in enumerate(associations):
        if _matches_assoc(event, association, kind):
            keys.append(("association", index))
    for index, target in enumerate(targets):
        if _matches_target(body, target, kind):
            keys.append(("target", index))
    return keys


def _matches_change(event, scenario, targets, associations):
    return bool(_match_keys(event, scenario, targets, associations))


def _matches_assoc(event, association, kind):
    body = event.get("message") or {}
    if association.get("event_id") != body.get("event_id"):
        return False
    if association.get("publisher_generation", event.get("publisher_generation")) != event.get("publisher_generation"):
        return False
    if "sensor" in association and association["sensor"] != body.get("sensors"):
        return False
    return body.get("kind") == kind


def _matches_target(body, target, kind):
    if body.get("sensors") != target.get("sensor"):
        return False
    if _iou(body.get("aabb"), target.get("aabb")) < IOU_MIN:
        return False
    if kind != "moved":
        return True
    source = target.get("from_centroid")
    if source is None and target.get("from_aabb"):
        source = _center(target["from_aabb"])
    if source is None or body.get("from_centroid") is None:
        return False
    return _dist(body["from_centroid"], source) <= FROM_MATCH_M


def _split_pair(window, targets, associations):
    appeared = None
    disappeared = None
    for event in window:
        body = event.get("message") or {}
        role = None
        for association in associations:
            if association.get("event_id") == body.get("event_id") and association.get("role") in ("split_appeared", "split_disappeared"):
                if association.get("publisher_generation", event.get("publisher_generation")) == event.get("publisher_generation"):
                    role = association["role"]
        if role == "split_appeared" or (body.get("kind") == "appeared" and any(
            body.get("sensors") == target.get("sensor") and _iou(body.get("aabb"), target.get("aabb")) >= IOU_MIN
            for target in targets
        )):
            appeared = event
        if role == "split_disappeared" or (body.get("kind") == "disappeared" and any(_is_source(body, target) for target in targets)):
            disappeared = event
    if appeared is not None and disappeared is not None and appeared is not disappeared:
        return (appeared, disappeared)
    return None


def _is_source(body, target):
    if body.get("sensors") != target.get("sensor"):
        return False
    if target.get("from_aabb") and _iou(body.get("aabb"), target["from_aabb"]) >= IOU_MIN:
        return True
    source = target.get("from_centroid")
    centroid = body.get("centroid")
    return bool(source and centroid and _dist(centroid, source) <= FROM_MATCH_M)


def _overlaps_person(event, targets):
    body = event.get("message") or {}
    for target in targets:
        if body.get("sensors") == target.get("sensor") and _iou(body.get("aabb"), target.get("aabb")) > 0:
            return True
    return False


def _delays(row, scenario, event):
    done = scenario.get("change_done_mono_ns")
    clear = scenario.get("clear_mono_ns")
    receipt = event["receipt_mono_ns"]
    if done is not None:
        row["receipt_delay_s"] = (receipt - done) / 1e9
    if clear is not None:
        row["clear_to_event_s"] = (receipt - clear) / 1e9


def _disposition(event, name, scenario_id):
    body = event.get("message") or {}
    return {
        "publisher_generation": event.get("publisher_generation"),
        "event_id": body.get("event_id"),
        "kind": body.get("kind"),
        "sensors": body.get("sensors"),
        "seq": event.get("seq"),
        "receipt_mono_ns": event.get("receipt_mono_ns"),
        "stamp_ns": event.get("stamp_ns"),
        "from_centroid": body.get("from_centroid"),
        "aabb": body.get("aabb"),
        "centroid": body.get("centroid"),
        "disposition": name,
        "scenario": scenario_id,
    }


def _finish(rows, events, assigned, messages, fresh, compatible, meta, ignored, confirmed_count, threshold):
    exposure = 0.0
    static_intervals = []
    for row in rows:
        scenario = row.pop("_scenario")
        row.pop("_gaps", None)
        row["_change_done"] = scenario.get("change_done_mono_ns")
        span = (scenario["t1_mono_ns"] - scenario["t0_mono_ns"]) / 1e9
        if row["detection"] == "miss" and row["coverage"] == "verifiable":
            _apply_confusion(row, scenario, messages)
        if row["id"] in IDLE_IDS and row["coverage"] == "verifiable":
            exposure += span
            static_intervals.append((scenario["t0_mono_ns"], scenario["t1_mono_ns"]))
        alarms = [
            item for item in assigned.values()
            if item["scenario"] == row["id"] and item["disposition"] in ("false_alarm", "activity_as_change")
        ]
        row["false_alarms"] = len(alarms)
    candidates = list(assigned.values())
    seen = {(item["publisher_generation"], item["event_id"]) for item in candidates}
    for event in events:
        body = event.get("message") or {}
        key = (event.get("publisher_generation"), body.get("event_id"))
        if key not in seen:
            candidates.append(_disposition(event, "false_alarm", None))
            seen.add(key)

    def count(name):
        return sum(1 for item in candidates if item["disposition"] == name)

    true_detections = count("true_detection")
    temporal = count("temporal_match")
    misses = sum(1 for row in rows if row["detection"] == "miss")
    split_move = sum(1 for row in rows if row["detection"] == "miss" and any(
        item["scenario"] == row["id"] and item["disposition"] == "split_move" for item in candidates
    ))
    false_alarms = count("false_alarm") + count("activity_as_change")
    activity_as_change = sum(1 for row in rows if row["discrimination"] == "activity_as_change")
    during = sum(1 for row in rows if row["discrimination"] == "candidate_during_activity")
    unverified = sum(1 for row in rows if row["discrimination"] == "person_unverified")
    with_activity = sum(1 for row in rows if row["miss_with_activity_observed"])
    as_activity = sum(1 for row in rows if row["change_as_activity"])
    idle_alarms = sum(
        1 for item in candidates
        if item["scenario"] in IDLE_IDS and item["disposition"] in ("false_alarm", "activity_as_change")
    )
    if exposure <= 0:
        rate = None
        rate_pass = None
    else:
        rate = idle_alarms * 3600.0 / exposure
        rate_pass = rate <= threshold
    non_idle = sum(
        1 for item in candidates
        if item["disposition"] in ("false_alarm", "activity_as_change") and item["scenario"] not in IDLE_IDS
    )
    delays = [row["receipt_delay_s"] for row in rows if row["receipt_delay_s"] is not None]
    protocol = _protocol_complete(rows, compatible, meta)
    for row in rows:
        row.pop("_change_done", None)
    lost = [row["id"] for row in rows if any(reason in LOST_WINDOW_REASONS for reason in row["coverage_reasons"])]
    session_reasons = []
    session_dropped = _session_dropped_windows(messages, meta, session_reasons)
    result, exit_code, acceptance = _decide(
        protocol, rows, misses, non_idle, rate_pass, activity_as_change, split_move, lost, session_reasons,
    )
    product, product_note = _confirmed_bar(
        confirmed_count, exposure, protocol, threshold, messages, static_intervals,
    )
    report = {
        "schema": "room-memory-acceptance-report-v1",
        "exit_code": exit_code,
        "g5_acceptance": acceptance,
        "candidate_result": result,
        "protocol_complete": protocol,
        "confirmed_product_bar": product,
        "confirmed_product_bar_note": product_note,
        "scored_metric": "false_candidate_alarms_per_hour",
        "product_metric": "false_confirmed_alarms_per_hour",
        "max_false_alarms_per_hour": threshold,
        "reference_compatible": compatible,
        "true_detections": true_detections,
        "temporal_matches": temporal,
        "misses": misses,
        "false_alarms": false_alarms,
        "idle_false_candidate_alarms_per_hour": rate,
        "idle_rate_pass": rate_pass,
        "credited_idle_exposure_s": exposure,
        "confirmed_count": confirmed_count,
        "ignored_transitions": ignored,
        "change_as_activity": as_activity,
        "miss_with_activity_observed": with_activity,
        "activity_as_change": activity_as_change,
        "candidate_during_activity": during,
        "person_unverified": unverified,
        "split_move": split_move,
        "dropped_windows": sum(row["dropped_windows"] for row in rows),
        "session_dropped_windows": session_dropped,
        "session_window_reasons": session_reasons,
        "lost_window_scenarios": lost,
        "seq_gaps": sum(1 for row in rows for reason in row["coverage_reasons"] if reason.endswith("_seq_gap")),
        "detection_delay_basis": "receipt_mono",
        "receipt_delay_s": {
            "p50": _percentile(delays, 0.50),
            "p95": _percentile(delays, 0.95),
            "max": None if not delays else max(delays),
            "values": delays,
        },
        "clock": _clock(messages, fresh),
        "scenarios": rows,
        "candidates": candidates,
    }
    report["markdown"] = render_markdown(report)
    return report


def _apply_confusion(row, scenario, messages):
    activities = _activities_in(scenario, messages)
    active = [item for item in activities if (item.get("message") or {}).get("active")]
    row["activity_observed"] = bool(active)
    if not active:
        return
    row["miss_with_activity_observed"] = True
    clear = scenario.get("clear_mono_ns")
    targets = list(scenario.get("targets") or [])
    if clear is None or not targets:
        return
    for item in active:
        if item["receipt_mono_ns"] < clear:
            continue
        body = item.get("message") or {}
        for target in targets:
            if body.get("largest_sensor") != target.get("sensor"):
                continue
            if _iou(body.get("aabb"), target.get("aabb")) > 0:
                row["change_as_activity"] = True
                return


def _protocol_complete(rows, compatible, meta):
    if not compatible or meta.get("end_reason") != "complete":
        return False
    by_id = {row["id"]: row for row in rows}
    for scenario_id, duration in REQUIRED_S.items():
        row = by_id.get(scenario_id)
        if row is None:
            return False
        if row["completion"] != "complete" or row["coverage"] != "verifiable":
            return False
        if row["actual_duration_s"] is None or row["actual_duration_s"] + 1e-6 < duration:
            return False
        if row["planned_duration_s"] is None or row["planned_duration_s"] + 1e-6 < duration:
            return False
        if row["protocol_duration_s"] is None or row["protocol_duration_s"] + 1e-6 < duration:
            return False
        if scenario_id in CHANGE_IDS and row.get("_change_done") is None:
            return False
    return True


def _decide(protocol, rows, misses, non_idle, rate_pass, activity_as_change, split_move, lost, session_reasons):
    hard = bool(misses or non_idle or activity_as_change or split_move or rate_pass is False)
    if not protocol:
        return ("fail" if hard else "inconclusive"), 2, "denied"
    unverified = False
    failed = hard
    for row in rows:
        if row["id"] in CHANGE_IDS and row["detection"] != "true_detection":
            if row["detection"] == "miss":
                failed = True
            else:
                unverified = True
        if row["id"] in PERSON_IDS and row["discrimination"] != "activity_without_candidate":
            if row["discrimination"] in ("activity_as_change", "candidate_during_activity"):
                failed = True
            else:
                unverified = True
    if failed:
        return "fail", 1, "denied"
    # Lost windows deny full acceptance even when every protocol scenario is
    # verifiable: a drop inside restore_reference, or any nonzero change from
    # the session baseline through session_end (during setup, in a gap between
    # scenarios, or on a generation that restarted during the session).
    session_lost = any(reason in LOST_WINDOW_REASONS for reason in session_reasons)
    if unverified or lost or session_lost or rate_pass is not True:
        return "inconclusive", 2, "denied"
    return "pass", 0, "full_candidate"


def _confirmed_bar(confirmed_count, exposure, protocol, threshold, messages, static_intervals):
    if confirmed_count == 0:
        return "not_evaluated", CONFIRMED_NOTE
    if not protocol or exposure <= 0 or not static_intervals:
        return "not_evaluated", CONFIRMED_PARTIAL_NOTE
    false_confirmed = 0
    for message in messages:
        if message.get("topic") != EVENTS or (message.get("message") or {}).get("state") != "confirmed":
            continue
        receipt = message.get("receipt_mono_ns")
        if receipt is None:
            continue
        if any(start <= receipt <= end for start, end in static_intervals):
            false_confirmed += 1
    rate = false_confirmed * 3600.0 / exposure
    if rate <= threshold:
        return "pass", (
            "CONFIRMED false-alarm rate is within the proposed static-room bar. " + CONFIRMED_RATE_NOTE
        )
    return "fail", (
        "CONFIRMED false-alarm rate is above the proposed static-room bar. " + CONFIRMED_RATE_NOTE
    )


def _clock(messages, fresh):
    offsets = []
    for index, message in enumerate(messages):
        if index in fresh and message.get("topic") == STATUS:
            offsets.append(message["stamp_ns"] - message["receipt_mono_ns"])
    if len(offsets) < 2:
        return {
            "synchronized": False,
            "offset_ns": None,
            "uncertainty_ns": None,
            "note": "Fewer than two fresh status samples. A latched status is not a sync sample. Offset is not applied to marks.",
        }
    uncertainty = max(offsets) - min(offsets)
    ordered = sorted(offsets)
    return {
        "synchronized": uncertainty <= SYNC_BOUND_NS,
        "offset_ns": ordered[len(ordered) // 2],
        "uncertainty_ns": uncertainty,
        "note": "Diagnostic only. Detection delay uses receipt time and does not apply this offset.",
    }


def _percentile(values, fraction):
    if not values:
        return None
    ordered = sorted(values)
    if len(ordered) == 1:
        return ordered[0]
    rank = (len(ordered) - 1) * fraction
    low = math.floor(rank)
    high = math.ceil(rank)
    if low == high:
        return ordered[int(rank)]
    weight = rank - low
    return ordered[low] * (1.0 - weight) + ordered[high] * weight


def _iou(left, right):
    if not left or not right:
        return 0.0
    left_lo = [left[0][i] - PAD_M for i in range(3)]
    left_hi = [left[1][i] + PAD_M for i in range(3)]
    right_lo = [right[0][i] - PAD_M for i in range(3)]
    right_hi = [right[1][i] + PAD_M for i in range(3)]
    inter = 1.0
    for index in range(3):
        lo = max(left_lo[index], right_lo[index])
        hi = min(left_hi[index], right_hi[index])
        if hi <= lo:
            return 0.0
        inter *= hi - lo
    union = _volume(left_lo, left_hi) + _volume(right_lo, right_hi) - inter
    if union <= 0:
        return 0.0
    return inter / union


def _volume(lo, hi):
    value = 1.0
    for index in range(3):
        value *= hi[index] - lo[index]
    return value


def _dist(left, right):
    return math.sqrt(sum((left[index] - right[index]) ** 2 for index in range(3)))


def _center(aabb):
    return [(aabb[0][index] + aabb[1][index]) / 2 for index in range(3)]
