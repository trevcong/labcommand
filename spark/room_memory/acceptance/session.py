"""Operator prompts. The clock and the input reader are injected so tests never touch the terminal."""

from acceptance.scenarios import PROTOCOL, REFERENCE_PROMPT, TOKEN_LINE, resolve_durations

MARKS = "room-memory-acceptance-marks-v1"


class _Abort(Exception):
    pass


def parse_label(text):
    """One line: ``skip``, ``region SENSOR x0 y0 z0 x1 y1 z1 [from x y z]``, or ``event GEN ID``.

    Several clauses are separated by semicolons. Coordinates are in that sensor's frame.
    """
    text = (text or "").strip()
    if text == "" or text.lower() == "skip":
        return [], []
    targets = []
    associations = []
    for part in text.split(";"):
        bits = part.split()
        if not bits:
            continue
        if bits[0] == "region":
            if len(bits) < 8:
                raise ValueError("region needs a sensor and six coordinates")
            sensor = int(bits[1])
            nums = [float(item) for item in bits[2:8]]
            item = {
                "sensor": sensor,
                "aabb": [nums[0:3], nums[3:6]],
                "from_centroid": None,
                "from_aabb": None,
            }
            if len(bits) > 8:
                if bits[8] != "from" or len(bits) < 12:
                    raise ValueError("from needs three coordinates")
                item["from_centroid"] = [float(value) for value in bits[9:12]]
            targets.append(item)
        elif bits[0] == "event":
            if len(bits) != 3:
                raise ValueError("event needs a publisher generation and an event id")
            associations.append({
                "publisher_generation": int(bits[1]),
                "event_id": int(bits[2]),
                "role": "target",
            })
        else:
            raise ValueError("a label is region, event, or skip")
    return targets, associations


def run_session(clock, read_line, write, scenario_ids=None, durations=None, candidates_fn=None, on_progress=None):
    """Run the script. ``read_line(deadline_ns)`` returns a line or None on timeout.

    Enter during a watch records ``shortened``. A planned watch below the protocol
    duration is also ``shortened``. ``abort`` skips the scenarios that have not started.
    A bad target label is asked again. Ctrl-C returns the scenarios already finished,
    with the one in progress marked ``aborted`` and ``session.end_reason`` ``interrupted``.
    ``on_progress(marks)`` runs after the setup and after each scenario so a caller can
    store the capture before the function returns.
    """
    selected = None if scenario_ids is None else set(scenario_ids)
    durations = dict(durations or {})
    rows = []
    setup = _unverified()
    current = None

    def say(text):
        write(text if text.endswith("\n") else text + "\n")

    def ask(deadline=None):
        line = read_line(deadline)
        if line is None:
            return None
        line = line.strip()
        if line.lower() == "abort":
            raise _Abort()
        return line

    def publish(reason, detail=""):
        snapshot = _marks(setup, rows, reason, detail)
        if on_progress is not None:
            on_progress(snapshot)
        return snapshot

    try:
        say(REFERENCE_PROMPT)
        try:
            verdict = ask()
        except _Abort:
            return publish("aborted")
        confirmed = (verdict or "").strip().lower() == "verified"
        if confirmed:
            say("Type this line if it is true, or anything else if it is not: " + TOKEN_LINE)
            try:
                tokens = ask()
            except _Abort:
                return publish("aborted")
            if (tokens or "").split() == TOKEN_LINE.split():
                setup = _verified()
            else:
                confirmed = False
                say("That setup is incompatible. Change scenarios are blocked. The node is left as it is.")
        else:
            say("Reference setup is not confirmed. Change scenarios are blocked. The node is left as it is.")
        publish("running")

        aborted = False
        for spec in PROTOCOL:
            if selected is not None and spec["id"] not in selected:
                continue
            planned = _planned(spec, durations)
            if aborted:
                rows.append(_shell(spec, "skipped", clock.monotonic_ns(), planned))
                publish("aborted")
                continue
            if spec["class"] == "change" and not _compatible(setup):
                rows.append(_shell(spec, "blocked", clock.monotonic_ns(), planned))
                publish("running")
                continue
            current = spec
            try:
                rows.append(_conduct(spec, planned, clock, ask, say, candidates_fn))
            except _Abort:
                rows.append(_shell(spec, "aborted", clock.monotonic_ns(), planned))
                aborted = True
            current = None
            publish("aborted" if aborted else "running")
        return publish("aborted" if aborted else "complete")
    except KeyboardInterrupt:
        if current is not None:
            rows.append(_shell(current, "aborted", clock.monotonic_ns(), _planned(current, durations)))
        return publish("interrupted", "keyboard interrupt")


def _planned(spec, durations):
    if spec["id"] in durations:
        return durations[spec["id"]]
    if spec["class"] == "change" and "change" in durations:
        return durations["change"]
    return spec["protocol_s"]


def _conduct(spec, planned, clock, ask, say, candidates_fn):
    say(spec["prompt"])
    kind = spec["class"]
    targets = []
    associations = []
    if kind == "change":
        say("Press Enter when the physical change is complete. This is not the clear mark.")
        ask()
        change_done = clock.monotonic_ns()
        say("Step at least 1 m away. A person within 1 m restarts the settle count. Press Enter when you are clear and the object is still.")
        ask()
        clear = clock.monotonic_ns()
        start = clear
    elif kind == "restoration":
        say("Press Enter to start the restoration watch, or type skip.")
        line = ask()
        if (line or "").strip().lower() == "skip":
            now = clock.monotonic_ns()
            row = _shell(spec, "skipped", now, planned)
            return row
        change_done = None
        clear = None
        start = clock.monotonic_ns()
    else:
        change_done = None
        clear = None
        if kind == "activity":
            say("Type skip, or a region line for the person, and press Enter to start the watch.")
            targets, associations = _read_label(ask, say)
        else:
            say("Press Enter to start the watch.")
            ask()
            targets = []
            associations = []
        start = clock.monotonic_ns()
    say(
        "Watching for %g s. Protocol for this step is %g s. "
        "Enter ends the watch early and withholds full G5 acceptance."
        % (planned, spec["protocol_s"])
    )
    deadline = start + int(round(planned * 1_000_000_000))
    line = ask(deadline)
    end = clock.monotonic_ns()
    actual = (end - start) / 1e9
    if line is not None or planned + 1e-9 < spec["protocol_s"] or actual + 1e-6 < spec["protocol_s"]:
        completion = "shortened"
    else:
        completion = "complete"
    if kind == "change":
        if candidates_fn is not None:
            for item in candidates_fn(change_done, end):
                say(
                    "candidate event_id=%s sensor=%s kind=%s centroid=%s aabb=%s"
                    % (item.get("event_id"), item.get("sensors"), item.get("kind"), item.get("centroid"), item.get("aabb"))
                )
        say("Label the target: region SENSOR x0 y0 z0 x1 y1 z1 [from x y z] | event GEN ID | skip")
        targets, associations = _read_label(ask, say)
    return {
        "id": spec["id"],
        "class": spec["class"],
        "kind": spec["kind"],
        "protocol_duration_s": spec["protocol_s"],
        "planned_duration_s": planned,
        "actual_duration_s": actual,
        "completion": completion,
        "prerequisites": ["reference_setup"] if kind == "change" else [],
        "t0_mono_ns": start if kind != "change" else change_done,
        "t1_mono_ns": end,
        "change_done_mono_ns": change_done if kind == "change" else None,
        "clear_mono_ns": clear if kind == "change" else None,
        "targets": targets,
        "associations": associations,
    }


def _read_label(ask, say):
    """Ask until the line parses. Enter and ``skip`` are empty labels, not errors."""
    while True:
        try:
            return parse_label(ask())
        except ValueError as exc:
            say("%s. Enter region, event, or skip." % exc)


def _shell(spec, completion, now, planned):
    return {
        "id": spec["id"],
        "class": spec["class"],
        "kind": spec["kind"],
        "protocol_duration_s": spec["protocol_s"],
        "planned_duration_s": planned,
        "actual_duration_s": 0,
        "completion": completion,
        "prerequisites": ["reference_setup"] if spec["class"] == "change" else [],
        "t0_mono_ns": now,
        "t1_mono_ns": now,
        "change_done_mono_ns": None,
        "clear_mono_ns": None,
        "targets": [],
        "associations": [],
    }


def _verified():
    return {
        "verified": True,
        "operator_confirmed": True,
        "tracking_seen": False,
        "placement": "new_object",
        "removal": "reference_object",
        "move_source": "other_reference_object",
        "placement_is_removal": False,
        "removal_is_move_source": False,
    }


def _unverified():
    return {
        "verified": False,
        "operator_confirmed": False,
        "tracking_seen": False,
        "placement": None,
        "removal": None,
        "move_source": None,
        "placement_is_removal": False,
        "removal_is_move_source": False,
    }


def _compatible(setup):
    return bool(setup.get("operator_confirmed") and setup.get("verified") and not setup.get("placement_is_removal"))


def empty_marks(reason, detail=""):
    return _marks(_unverified(), [], reason, detail)


def _marks(setup, rows, end_reason, detail=""):
    return {
        "schema": MARKS,
        "clock": "receipt_mono",
        "expected_sensors": [1, 2],
        "reference_setup": setup,
        "scenarios": rows,
        "session": {"end_reason": end_reason, "detail": detail},
    }
