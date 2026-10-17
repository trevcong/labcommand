"""Runner shutdown. No docker and no socket: the subscriber is a fake pipe."""

import io
import json
import select
import threading
import time
from types import SimpleNamespace

import pytest

from acceptance.__main__ import _run
from acceptance.__main__ import wait_for_closing
from acceptance.log import load_jsonl
from acceptance.score import score
from acceptance.tests.support import event
from acceptance.tests.support import marks
from acceptance.tests.support import ns
from acceptance.tests.support import scenario
from acceptance.tests.support import stream
from acceptance.tests.test_score import _perfect

TOPICS = ["/room_memory/status", "/room_memory/events", "/room_memory/activity"]


class FastClock:
    """Monotonic clock that jumps on sleep so a tail bound does not wait on the wall."""

    def __init__(self):
        self.now = 0

    def monotonic_ns(self):
        return self.now

    def monotonic(self):
        return self.now / 1e9

    def sleep(self, seconds):
        self.now += _forward_ns(seconds)
        time.sleep(0)


class Proc:
    def __init__(self, lines):
        self.stdout = iter(lines)
        self.stderr = io.StringIO("")
        self.stdin = io.StringIO()
        self.returncode = None

    def poll(self):
        return self.returncode

    def terminate(self):
        self.returncode = -15

    def kill(self):
        self.returncode = -9

    def wait(self, timeout=None):
        return self.returncode


def _args(tmp_path, scenarios="baseline_idle"):
    return SimpleNamespace(
        out=str(tmp_path),
        container="room-memory",
        scenarios=scenarios,
        watch_s=None,
        walk_s=None,
        sit_s=None,
        hour_s=None,
        max_false_alarms_per_hour=1.0,
        transport="ros2",
    )


def _ready_line():
    return json.dumps({
        "op": "ready",
        "mono_ns": 1,
        "matched": True,
        "topics": TOPICS,
    }) + "\n"


def _popen(proc):
    def popen(command, **kwargs):
        assert command[:3] == ["docker", "exec", "-i"]
        assert kwargs.get("text") is True
        return proc
    return popen


def _session_end(path):
    rows = load_jsonl(path)
    assert rows[-1]["op"] == "session_end"
    return rows[-1]


def test_wait_returns_immediately_when_the_tail_is_already_in_the_log():
    messages, mark = _perfect()
    records = [item for item in messages if item.get("op") != "session_end"]

    def sleep(seconds):
        raise AssertionError("tail was already recorded")

    wait_for_closing(records, threading.Lock(), mark, lambda: messages[-1]["receipt_mono_ns"], sleep)


def test_wait_accepts_the_fixture_samples_that_arrive_after_the_watch():
    messages, mark = _perfect()
    hour = mark["scenarios"][-1]
    hour["t0_mono_ns"] += ns(0.25)
    hour["t1_mono_ns"] += ns(0.25)
    t1 = hour["t1_mono_ns"]
    cutoff = [item for item in messages if item.get("op") == "session_end" or item["receipt_mono_ns"] <= t1]
    denied = score(cutoff, mark)
    assert denied["g5_acceptance"] == "denied"
    assert "missing_tail" in denied["scenarios"][-1]["coverage_reasons"]

    early = [item for item in messages if item.get("op") != "session_end" and item["receipt_mono_ns"] <= t1]
    late = [item for item in messages if item.get("op") != "session_end" and item["receipt_mono_ns"] > t1]
    assert late
    records = list(early)
    added = {"done": False}
    clock = {"now": t1}

    def sleep(seconds):
        assert added["done"] is False
        records.extend(late)
        added["done"] = True
        clock["now"] = max(item["receipt_mono_ns"] for item in late)

    wait_for_closing(records, threading.Lock(), mark, lambda: clock["now"], sleep)
    assert added["done"] is True
    end = next(item for item in messages if item.get("op") == "session_end")
    report = score(records + [end], mark)
    assert report["g5_acceptance"] == "full_candidate"
    assert report["scenarios"][-1]["coverage_reasons"] == []


def test_wait_holds_until_the_advertised_event_arrives():
    messages = stream(0, 2, [event(1.9, "appeared", 1, [[0.0, 0.0, 0.0], [1.0, 1.0, 1.0]])], session_end_s=4)
    t1 = ns(1.25)
    early = [item for item in messages if item.get("op") != "session_end" and item["receipt_mono_ns"] <= t1]
    closing = [
        item for item in messages
        if item.get("topic") in ("/room_memory/status", "/room_memory/activity") and item["receipt_mono_ns"] >= ns(2)
    ]
    events = [item for item in messages if item.get("topic") == "/room_memory/events"]
    status = next(item for item in closing if item["topic"] == "/room_memory/status")
    assert status["message"]["events_seq"] == events[0]["seq"]
    records = list(early)
    mark = marks([scenario(
        "baseline_idle", "idle", 0.25, 1.25, protocol=60, actual=1, planned=60, completion="shortened",
    )])
    clock = {"now": t1}
    phase = {"n": 0}

    def sleep(seconds):
        phase["n"] += 1
        if phase["n"] == 1:
            records.extend(closing)
            clock["now"] = ns(2)
            return
        assert phase["n"] == 2
        records.extend(events)
        clock["now"] = ns(2.1)

    wait_for_closing(records, threading.Lock(), mark, lambda: clock["now"], sleep)
    assert phase["n"] == 2
    assert events[0] in records


def test_wait_stops_at_the_bound_when_the_tail_never_arrives():
    mark = marks([scenario(
        "hour_idle", "idle", 0, 10, protocol=3600, actual=10, planned=3600, completion="complete",
    )])
    clock = {"now": ns(10)}
    sleeps = {"n": 0}

    def sleep(seconds):
        sleeps["n"] += 1
        assert sleeps["n"] < 200
        clock["now"] += _forward_ns(seconds)

    wait_for_closing([], threading.Lock(), mark, lambda: clock["now"], sleep)
    assert sleeps["n"] >= 1
    assert clock["now"] >= ns(10) + 3_500_000_000


def test_runner_keeps_the_subscriber_until_the_fixture_tail_arrives(tmp_path, monkeypatch, capsys):
    messages, mark = _perfect()
    hour = mark["scenarios"][-1]
    hour["t0_mono_ns"] += ns(0.25)
    hour["t1_mono_ns"] += ns(0.25)
    t1 = hour["t1_mono_ns"]
    early = [item for item in messages if item.get("op") != "session_end" and item["receipt_mono_ns"] <= t1]
    late = [item for item in messages if item.get("op") != "session_end" and item["receipt_mono_ns"] > t1]
    assert late

    clock = StampClock()

    class TailProc(Proc):
        def __init__(self):
            super().__init__([])
            self.stdout = self._lines()
            self.saw_late = False

        def _lines(self):
            for item in early:
                clock.now = item["receipt_mono_ns"]
                yield json.dumps(item) + "\n"
            clock.early_done.set()
            clock.late_request.wait(30)
            if clock.stop.is_set():
                return
            for item in late:
                clock.now = item["receipt_mono_ns"]
                yield json.dumps(item) + "\n"
            clock.late_done.set()
            clock.stop.wait(30)

        def terminate(self):
            self.saw_late = clock.late_done.is_set()
            self.returncode = -15
            clock.stop.set()
            clock.late_request.set()

    proc = TailProc()

    def fake_session(**kwargs):
        assert clock.early_done.wait(30), "subscriber did not deliver the in-watch samples"
        clock.now = t1
        clock.session_done = True
        return mark

    monkeypatch.setattr("acceptance.__main__.run_session", fake_session)
    code = _run(_args(tmp_path), popen=_popen(proc), clock=clock)
    assert proc.saw_late is True
    assert code == 0
    report = json.loads((tmp_path / "report.json").read_text())
    assert report["g5_acceptance"] == "full_candidate"
    assert report["scenarios"][-1]["coverage_reasons"] == []
    saved = load_jsonl(tmp_path / "topics.jsonl")
    assert any(item.get("receipt_mono_ns", 0) > t1 and item.get("topic") == "/room_memory/status" for item in saved)
    assert "Holding the recorder" in capsys.readouterr().out


class StampClock:
    """Receipt stamps follow the fixture. The ready wait uses a separate wall."""

    def __init__(self):
        self.now = 0
        self._wall = 0.0
        self.session_done = False
        self.released = False
        self.early_done = threading.Event()
        self.late_request = threading.Event()
        self.late_done = threading.Event()
        self.stop = threading.Event()

    def monotonic_ns(self):
        return self.now

    def monotonic(self):
        if not self.session_done:
            return self._wall
        return self.now / 1e9

    def sleep(self, seconds):
        if self.session_done and not self.released:
            self.released = True
            self.late_request.set()
            assert self.late_done.wait(30), "closing samples were not delivered"
            return
        if not self.session_done:
            self._wall += 0.05
            time.sleep(0.001)
            return
        self.now += _forward_ns(seconds)


def _forward_ns(seconds):
    step = int(max(0.0, seconds) * 1e9)
    if seconds > 0 and step < 1:
        return 1
    return step


def test_bad_label_still_leaves_a_scoreable_capture(tmp_path, monkeypatch, capsys):
    monkeypatch.setattr(select, "select", lambda *args, **kwargs: ([], [], []))
    lines = [
        "verified\n",
        "new_object reference_object other_reference_object\n",
        "\n",
        "\n",
        "region nope\n",
        "region 1 0 0 0 1 1 1\n",
    ]
    monkeypatch.setattr("acceptance.__main__.sys.stdin", Stdin(lines))
    code = _run(
        _args(tmp_path, "box_placed"),
        popen=_popen(Proc([_ready_line()])),
        clock=FastClock(),
    )
    assert code == 2
    saved = json.loads((tmp_path / "marks.json").read_text())
    row = next(item for item in saved["scenarios"] if item["id"] == "box_placed")
    assert row["targets"][0]["aabb"] == [[0.0, 0.0, 0.0], [1.0, 1.0, 1.0]]
    assert (tmp_path / "topics.jsonl").is_file()
    assert (tmp_path / "report.md").is_file()
    assert "region needs a sensor" in capsys.readouterr().out
    assert _session_end(tmp_path / "topics.jsonl")["reason"] == "complete"


def test_checkpoint_survives_an_interrupt(tmp_path, monkeypatch):
    monkeypatch.setattr(select, "select", lambda *args, **kwargs: ([], [], []))
    lines = [
        "verified\n",
        "new_object reference_object other_reference_object\n",
        "\n",
    ]

    class Asking(Stdin):
        def readline(self):
            if self.lines:
                return self.lines.pop(0)
            checkpoint = json.loads((tmp_path / "marks.json").read_text())
            assert any(item["id"] == "baseline_idle" for item in checkpoint["scenarios"])
            assert "ready" in (tmp_path / "topics.jsonl").read_text()
            raise KeyboardInterrupt()

    monkeypatch.setattr("acceptance.__main__.sys.stdin", Asking(lines))
    code = _run(
        _args(tmp_path, "baseline_idle,box_placed"),
        popen=_popen(Proc([_ready_line()])),
        clock=FastClock(),
    )
    assert code == 2
    saved = json.loads((tmp_path / "marks.json").read_text())
    by_id = {item["id"]: item for item in saved["scenarios"]}
    assert by_id["baseline_idle"]["completion"] == "shortened"
    assert by_id["box_placed"]["completion"] == "aborted"
    assert saved["session"]["end_reason"] == "interrupted"
    end = _session_end(tmp_path / "topics.jsonl")
    assert end["reason"] == "interrupted"
    assert (tmp_path / "report.json").is_file()
    assert (tmp_path / "report.md").is_file()


def test_value_error_from_the_session_still_writes_the_capture(tmp_path, monkeypatch, capsys):
    def boom(**kwargs):
        raise ValueError("region needs a sensor and six coordinates")

    monkeypatch.setattr("acceptance.__main__.run_session", boom)
    code = _run(_args(tmp_path), popen=_popen(Proc([_ready_line()])), clock=FastClock())
    assert code == 2
    assert "region needs a sensor" in capsys.readouterr().err
    saved = json.loads((tmp_path / "marks.json").read_text())
    assert saved["session"]["end_reason"] == "error"
    assert "region needs a sensor" in saved["session"]["detail"]
    end = _session_end(tmp_path / "topics.jsonl")
    assert end["reason"] == "error"
    assert (tmp_path / "report.json").is_file()
    assert (tmp_path / "report.md").is_file()


def test_keyboard_interrupt_outside_the_session_still_writes_the_capture(tmp_path, monkeypatch):
    def boom(**kwargs):
        raise KeyboardInterrupt()

    monkeypatch.setattr("acceptance.__main__.run_session", boom)
    code = _run(_args(tmp_path), popen=_popen(Proc([_ready_line()])), clock=FastClock())
    assert code == 2
    saved = json.loads((tmp_path / "marks.json").read_text())
    assert saved["session"]["end_reason"] == "interrupted"
    assert _session_end(tmp_path / "topics.jsonl")["reason"] == "interrupted"
    assert (tmp_path / "report.md").is_file()


def test_unexpected_error_still_leaves_the_capture(tmp_path, monkeypatch):
    def boom(**kwargs):
        raise RuntimeError("subscriber vanished")

    monkeypatch.setattr("acceptance.__main__.run_session", boom)
    with pytest.raises(RuntimeError, match="subscriber vanished"):
        _run(_args(tmp_path), popen=_popen(Proc([_ready_line()])), clock=FastClock())
    saved = json.loads((tmp_path / "marks.json").read_text())
    assert saved["session"]["end_reason"] == "error"
    assert "subscriber vanished" in saved["session"]["detail"]
    assert _session_end(tmp_path / "topics.jsonl")["reason"] == "error"
    assert (tmp_path / "report.json").is_file()


class Stdin:
    def __init__(self, lines):
        self.lines = list(lines)

    def readline(self):
        if not self.lines:
            raise AssertionError("session read past the script")
        return self.lines.pop(0)
