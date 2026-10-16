"""Rejected-frame throttle, and the type-name/message pair RoomMemory keeps for it.

Does not import ``room_memory.node`` (that pulls in rclpy).
"""

import gc
import weakref

import pytest

import scenes
from room_memory.memory import MemoryConfig, RoomMemory
from room_memory.messages import MemoryState, Time
from room_memory.rejectlog import INTERVAL_S, RejectionLog

TRUNCATED = "PointCloud2 CDR: truncated header.stamp.sec at byte 4 (need 4, have 0)"
BAD = b"\x00\x01\x00\x00"


def open_log():
    """A ``RejectionLog`` on the default interval whose clock is ``t[0]``."""
    t = [0.0]
    return RejectionLog(clock=lambda: t[0]), t


def room():
    return RoomMemory(MemoryConfig(state_dir=None), Time(0, 0), 1)


def test_first_line():
    log, _t = open_log()
    assert INTERVAL_S == 10.0
    assert log.observe("mid360", "PointCloudError", "truncated header") == (
        "mid360 rejected frame: PointCloudError: truncated header"
    )


def test_inside_the_window():
    log, t = open_log()
    log.observe("mid360", "PointCloudError", "truncated header")
    t[0] = 9.999
    assert log.observe("mid360", "PointCloudError", "truncated header") is None
    t[0] = 10.0
    assert log.observe("mid360", "PointCloudError", "truncated header") == (
        "mid360 rejected frame: PointCloudError: truncated header (1 suppressed)"
    )


def test_boundary():
    log, t = open_log()
    log.observe("mid360", "PointCloudError", "truncated header")
    t[0] = 1.0
    assert log.observe("mid360", "PointCloudError", "truncated header") is None
    t[0] = 10.0
    assert log.observe("mid360", "PointCloudError", "truncated header") == (
        "mid360 rejected frame: PointCloudError: truncated header (1 suppressed)"
    )


def test_count_resets():
    log, t = open_log()
    log.observe("mid360", "PointCloudError", "truncated header")
    t[0] = 1.0
    assert log.observe("mid360", "PointCloudError", "truncated header") is None
    t[0] = 10.0
    assert log.observe("mid360", "PointCloudError", "truncated header").endswith("(1 suppressed)")
    t[0] = 11.0
    assert log.observe("mid360", "PointCloudError", "truncated header") is None
    t[0] = 20.0
    line = log.observe("mid360", "PointCloudError", "truncated header")
    assert line == "mid360 rejected frame: PointCloudError: truncated header (1 suppressed)"
    assert "(2 suppressed)" not in line


def test_same_reading():
    log, t = open_log()
    assert log.observe("mid360", "PointCloudError", "truncated header") == (
        "mid360 rejected frame: PointCloudError: truncated header"
    )
    assert log.observe("mid360", "PointCloudError", "truncated header") is None
    t[0] = 10.0
    assert log.observe("mid360", "PointCloudError", "truncated header") == (
        "mid360 rejected frame: PointCloudError: truncated header (1 suppressed)"
    )


def test_per_sensor():
    log, t = open_log()
    assert log.observe("mid360", "PointCloudError", "truncated header") == (
        "mid360 rejected frame: PointCloudError: truncated header"
    )
    t[0] = 1.0
    assert log.observe("jt128", "PointCloudError", "truncated header") == (
        "jt128 rejected frame: PointCloudError: truncated header"
    )


def test_type_is_not_the_key():
    log, t = open_log()
    log.observe("mid360", "PointCloudError", "truncated header")
    t[0] = 1.0
    assert log.observe("mid360", "MemoryError", "decoder ran out of memory") is None
    t[0] = 10.0
    assert log.observe("mid360", "MemoryError", "decoder ran out of memory") == (
        "mid360 rejected frame: MemoryError: decoder ran out of memory (1 suppressed)"
    )


def test_backwards_clock():
    log, t = open_log()
    t[0] = 5.0
    log.observe("mid360", "PointCloudError", "truncated header")
    t[0] = 4.0
    assert log.observe("mid360", "PointCloudError", "truncated header") is None
    assert log._last_emit["mid360"] == 5.0
    t[0] = 15.0
    assert log.observe("mid360", "PointCloudError", "truncated header") == (
        "mid360 rejected frame: PointCloudError: truncated header (1 suppressed)"
    )


def test_interval_restarts_at_the_last_emission():
    """A grid aligned to t = 0 would emit at 20. A window extended by the suppressed call would still be shut at 22."""
    log, t = open_log()
    assert log.observe("mid360", "PointCloudError", "truncated header") == (
        "mid360 rejected frame: PointCloudError: truncated header"
    )
    t[0] = 12.0
    assert log.observe("mid360", "PointCloudError", "truncated header") == (
        "mid360 rejected frame: PointCloudError: truncated header"
    )
    t[0] = 20.0
    assert log.observe("mid360", "PointCloudError", "still truncated") is None
    t[0] = 22.0
    assert log.observe("mid360", "PointCloudError", "still truncated") == (
        "mid360 rejected frame: PointCloudError: still truncated (1 suppressed)"
    )


def test_emitting_one_sensor_does_not_reset_the_other():
    log, t = open_log()
    assert log.observe("mid360", "PointCloudError", "bad mid") == (
        "mid360 rejected frame: PointCloudError: bad mid"
    )
    assert log.observe("jt128", "PointCloudError", "bad jt") == (
        "jt128 rejected frame: PointCloudError: bad jt"
    )
    t[0] = 1.0
    assert log.observe("mid360", "PointCloudError", "bad mid") is None
    t[0] = 2.0
    assert log.observe("mid360", "PointCloudError", "bad mid") is None
    t[0] = 4.0
    assert log.observe("jt128", "PointCloudError", "bad jt") is None
    t[0] = 10.0
    assert log.observe("mid360", "PointCloudError", "bad mid") == (
        "mid360 rejected frame: PointCloudError: bad mid (2 suppressed)"
    )
    assert log.observe("jt128", "PointCloudError", "bad jt") == (
        "jt128 rejected frame: PointCloudError: bad jt (1 suppressed)"
    )
    t[0] = 11.0
    assert log.observe("mid360", "PointCloudError", "bad mid") is None
    t[0] = 20.0
    # jt128 emits first. That emit must not clear mid360's suppressed call from t = 11.
    assert log.observe("jt128", "PointCloudError", "bad jt") == (
        "jt128 rejected frame: PointCloudError: bad jt"
    )
    assert log.observe("mid360", "PointCloudError", "bad mid") == (
        "mid360 rejected frame: PointCloudError: bad mid (1 suppressed)"
    )


def test_rejected_cdr_stores_type_name_and_message():
    memory = room()
    assert memory.channels["mid360"].last_error is None
    assert memory.add_cdr("mid360", BAD) is False
    channel = memory.channels["mid360"]
    assert (channel.received, channel.failed) == (1, 1)
    assert channel.last_error == ("PointCloudError", TRUNCATED)
    assert type(channel.last_error) is tuple
    assert [type(part) for part in channel.last_error] == [str, str]
    assert not isinstance(channel.last_error, BaseException)
    assert memory.channels["jt128"].last_error is None
    log, _t = open_log()
    assert log.observe("mid360", *channel.last_error) == f"mid360 rejected frame: PointCloudError: {TRUNCATED}"


def test_close_window_keeps_the_diagnostic_and_drops():
    memory = room()
    assert memory.add_cdr("mid360", BAD) is False
    diagnostic = memory.channels["mid360"].last_error
    memory.close_window(Time(0, 0))
    status = memory.status(Time(1, 0), 1.0)
    assert memory.dropped_windows == 1
    assert status.state == MemoryState.DEGRADED and status.dropped_windows == 1
    channel = memory.channels["mid360"]
    assert channel.failed == 0
    assert channel.last_error is diagnostic


def test_accepted_frame_clears_last_error_without_clearing_failed():
    memory = room()
    assert memory.add_cdr("mid360", BAD) is False
    good = scenes.encode_pointcloud2([(0.0, 0.0, 0.0)])
    assert memory.add_cdr("mid360", good) is True
    channel = memory.channels["mid360"]
    assert channel.last_error is None
    # add_cdr does not clear failed. close_window does that.
    assert (channel.received, channel.failed) == (2, 1)


def test_memory_error_stores_strings_and_releases_the_frame(monkeypatch):
    import room_memory.memory as memory_module

    class Payload:
        pass

    held = []

    def explode(data):
        payload = Payload()
        held.append(weakref.ref(payload))
        try:
            raise ValueError("inner")
        except ValueError as inner:
            raise MemoryError("decoder ran out of memory") from inner

    monkeypatch.setattr(memory_module, "decode_pointcloud2", explode)
    memory = room()
    # A bare bytearray cannot be weak-referenced. The subclass is still the frame buffer.
    class Frame(bytearray):
        pass

    frame = Frame(BAD)
    frame_ref = weakref.ref(frame)
    assert memory.add_cdr("mid360", frame) is False
    del frame
    gc.collect()
    channel = memory.channels["mid360"]
    assert (channel.received, channel.failed) == (1, 1)
    assert channel.last_error == ("MemoryError", "decoder ran out of memory")
    assert type(channel.last_error) is tuple
    assert [type(part) for part in channel.last_error] == [str, str]
    assert held[0]() is None
    assert frame_ref() is None
    assert not any(isinstance(value, BaseException) for value in vars(channel).values())


def test_keyboard_interrupt_is_not_a_rejected_frame(monkeypatch):
    import room_memory.memory as memory_module

    def explode(data):
        raise KeyboardInterrupt

    monkeypatch.setattr(memory_module, "decode_pointcloud2", explode)
    memory = room()
    with pytest.raises(KeyboardInterrupt):
        memory.add_cdr("mid360", BAD)
    channel = memory.channels["mid360"]
    assert (channel.received, channel.failed) == (0, 0)
    assert channel.last_error is None
