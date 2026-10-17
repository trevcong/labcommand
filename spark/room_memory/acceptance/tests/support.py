"""Synthetic room-memory logs for the acceptance scorer.

Times in these builders are seconds on the recorder's monotonic clock. Publisher
stamps are that clock plus ``stamp_offset_s`` unless a case overrides them.
"""

import json

NS = 1_000_000_000
SCHEMA = "room-memory-acceptance-log-v1"
MARKS = "room-memory-acceptance-marks-v1"
TOPICS = ("/room_memory/status", "/room_memory/events", "/room_memory/activity")

PLACE = [[0.0, 0.0, 0.0], [1.0, 1.0, 1.0]]
REMOVE = [[3.0, 0.0, 0.0], [4.0, 1.0, 1.0]]
MOVE_DEST = [[5.0, 0.0, 0.0], [6.0, 1.0, 1.0]]
MOVE_SOURCE = [[2.0, 0.0, 0.0], [3.0, 1.0, 1.0]]
MOVE_FROM = [2.5, 0.5, 0.5]
FAR = [[10.0, 10.0, 0.0], [11.0, 11.0, 1.0]]
PERSON = [[0.0, 2.0, 0.0], [0.6, 2.6, 1.8]]


def ns(seconds):
    return int(round(float(seconds) * NS))


def verified_setup(**overrides):
    setup = {
        "verified": True,
        "operator_confirmed": True,
        "tracking_seen": True,
        "placement": "new_object",
        "removal": "reference_object",
        "move_source": "other_reference_object",
        "placement_is_removal": False,
        "removal_is_move_source": False,
    }
    setup.update(overrides)
    return setup


def target(sensor, aabb, from_centroid=None, from_aabb=None):
    return {
        "sensor": sensor,
        "aabb": aabb,
        "from_centroid": from_centroid,
        "from_aabb": from_aabb,
    }


def scenario(scenario_id, kind_class, t0, t1, kind=None, protocol=None, planned=None, actual=None,
             completion="complete", prerequisites=None, change_done=None, clear=None, targets=None,
             associations=None):
    duration = t1 - t0
    row = {
        "id": scenario_id,
        "class": kind_class,
        "kind": kind,
        "protocol_duration_s": duration if protocol is None else protocol,
        "planned_duration_s": duration if planned is None else planned,
        "actual_duration_s": duration if actual is None else actual,
        "completion": completion,
        "prerequisites": list(prerequisites or []),
        "t0_mono_ns": ns(t0),
        "t1_mono_ns": ns(t1),
        "change_done_mono_ns": None if change_done is None else ns(change_done),
        "clear_mono_ns": None if clear is None else ns(clear),
        "targets": list(targets or []),
        "associations": list(associations or []),
    }
    return row


def marks(scenarios, setup=None, expected_sensors=None):
    return {
        "schema": MARKS,
        "clock": "receipt_mono",
        "expected_sensors": [1, 2] if expected_sensors is None else list(expected_sensors),
        "reference_setup": verified_setup() if setup is None else setup,
        "scenarios": scenarios,
    }


def event(t, kind, event_id, aabb, sensor=1, state="candidate", from_centroid=None, omit=False,
          generation=4, centroid=None):
    if centroid is None:
        centroid = [(aabb[0][i] + aabb[1][i]) / 2 for i in range(3)]
    return {
        "t": t,
        "kind": kind,
        "event_id": event_id,
        "sensors": sensor,
        "aabb": aabb,
        "centroid": centroid,
        "from_centroid": from_centroid,
        "state": state,
        "voxel_count": 40,
        "omit": omit,
        "generation": generation,
    }


def _hz(value):
    if value is None:
        return None
    return int(value)


def stream(t_first, t_last, events=(), active=None, hz_at=None, stamp_offset_s=0.0, generation=4,
           generation_at=None, ready_s=None, session_end_s=None, session_reason="complete",
           matched=True, status_seq_jump_at=(), latched=None, state="tracking", drop_ready=False):
    """One status and one activity at each integer second from ``t_first`` to ``t_last``.

    ``events`` are published in time order. ``omit`` keeps the seq in ``events_seq`` but
    drops the line, which is a missed event. A caller who wants a scenario ``[t0, t1]``
    covered passes a grid that starts at or before ``t0`` and ends at or after ``t1``.
    """
    published = [dict(item) for item in events]
    published.sort(key=lambda item: (item["t"], item["event_id"]))
    for seq, item in enumerate(published):
        item["seq"] = seq
    messages = []
    if not drop_ready:
        ready_at = t_first - 1 if ready_s is None else ready_s
        messages.append({
            "schema": SCHEMA,
            "op": "ready",
            "receipt_mono_ns": ns(ready_at),
            "callback_mono_ns": ns(ready_at),
            "topics": list(TOPICS),
            "matched": matched,
        })
    if latched is not None:
        messages.append(latched)
    status_seq = 0
    activity_seq = 0
    t = int(t_first)
    last = int(t_last)
    while t <= last:
        gen = generation if generation_at is None else generation_at(t)
        offset = stamp_offset_s(t) if callable(stamp_offset_s) else stamp_offset_s
        receipt = ns(t)
        visible = [item for item in published if item["t"] <= t and item["generation"] == gen]
        events_seq = None if not visible else visible[-1]["seq"]
        if t in status_seq_jump_at:
            status_seq += 1
        hz = (10, 10) if hz_at is None else hz_at(t)
        messages.append({
            "schema": SCHEMA,
            "topic": "/room_memory/status",
            "receipt_mono_ns": receipt,
            "callback_mono_ns": receipt,
            "stamp_ns": receipt + ns(offset),
            "seq": status_seq,
            "publisher_generation": gen,
            "initial": False,
            "message": {
                "state": state,
                "events_seq": events_seq,
                "mid360_hz": _hz(hz[0]),
                "jt128_hz": _hz(hz[1]),
                "dropped_windows": 0,
                "n_open_events": 0,
                "pose_residual_m": 0.01,
            },
        })
        status_seq += 1
        active_now = bool(active(t)) if active is not None else False
        activity_receipt = receipt + 1_000_000
        payload = {
            "active": active_now,
            "since_s": 0.0 if active_now else None,
            "sensors": 1 if active_now else 0,
            "largest_sensor": 1 if active_now else 0,
            "n_regions": 1 if active_now else 0,
            "voxel_count": 8 if active_now else 0,
            "centroid": [0.5, 0.5, 0.5] if active_now else None,
            "aabb": [[0.0, 0.0, 0.0], [1.0, 1.0, 1.0]] if active_now else None,
        }
        messages.append({
            "schema": SCHEMA,
            "topic": "/room_memory/activity",
            "receipt_mono_ns": activity_receipt,
            "callback_mono_ns": activity_receipt,
            "stamp_ns": activity_receipt + ns(offset),
            "seq": activity_seq,
            "publisher_generation": gen,
            "initial": False,
            "message": payload,
        })
        activity_seq += 1
        t += 1
    for item in published:
        if item["omit"]:
            continue
        receipt = ns(item["t"])
        offset = stamp_offset_s(item["t"]) if callable(stamp_offset_s) else stamp_offset_s
        gen = item["generation"]
        messages.append({
            "schema": SCHEMA,
            "topic": "/room_memory/events",
            "receipt_mono_ns": receipt,
            "callback_mono_ns": receipt,
            "stamp_ns": receipt + ns(offset),
            "seq": item["seq"],
            "publisher_generation": gen,
            "initial": False,
            "message": {
                "state": item["state"],
                "kind": item["kind"],
                "event_id": item["event_id"],
                "sensors": item["sensors"],
                "centroid": item["centroid"],
                "aabb": item["aabb"],
                "from_centroid": item["from_centroid"],
                "voxel_count": item["voxel_count"],
            },
        })
    end_at = t_last + 1 if session_end_s is None else session_end_s
    messages.append({
        "schema": SCHEMA,
        "op": "session_end",
        "receipt_mono_ns": ns(end_at),
        "reason": session_reason,
        "detail": "",
    })
    messages.sort(key=lambda item: (item["receipt_mono_ns"], 0 if item.get("op") == "ready" else 1, item.get("topic") or ""))
    return messages


def latched_sample(t, stamp_s, generation=4):
    receipt = ns(t)
    return {
        "schema": SCHEMA,
        "topic": "/room_memory/status",
        "receipt_mono_ns": receipt,
        "callback_mono_ns": receipt,
        "stamp_ns": ns(stamp_s),
        "seq": 0,
        "publisher_generation": generation,
        "initial": True,
        "message": {
            "state": "tracking",
            "events_seq": None,
            "mid360_hz": 10,
            "jt128_hz": 10,
            "dropped_windows": 0,
            "n_open_events": 0,
            "pose_residual_m": 0.01,
        },
    }


def dump_jsonl(path, messages):
    path.write_text("".join(json.dumps(item, allow_nan=False, sort_keys=True) + "\n" for item in messages))


def load_jsonl(path):
    rows = []
    for line in path.read_text().splitlines():
        if line.strip():
            rows.append(json.loads(line))
    return rows


def roundtrip(messages):
    return json.loads(json.dumps(messages))
