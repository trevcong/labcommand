"""Codec tests against the Phase 0 hand goldens (values from ``make_room_memory_goldens.py``)."""

import hashlib
import math
import pathlib
import re
import struct
import zlib
from dataclasses import replace

import pytest

from room_memory import messages as m

HAND = pathlib.Path(__file__).resolve().parent / "fixtures" / "hand"
# The Swift suite's copy of the hand goldens (absent in the image).
SWIFT_HAND = pathlib.Path(__file__).resolve().parents[3] / "Tests" / "Fixtures" / "room" / "golden" / "hand"
T10 = m.Time(1760400010, 250000000)
# The one publisher generation of every v2 golden (a start time in ms).
G = 1760400000000
TOKEN_A = bytes.fromhex("5a1e7c3d9b024f618e33c4d5e6f70819")
TOKEN_B = bytes.fromhex("c10ffee012344567a9876543210fedcb")
TOKEN_C = bytes.fromhex("0e8a3b5c77214d90b4065913c2ae6f01")


def hexfile(name, directory=HAND):
    text = (directory / name).read_text()
    return bytes.fromhex(" ".join(line.split("#", 1)[0] for line in text.splitlines()))


def head(room, epoch, seq, stamp=T10):
    return m.Header(room, epoch, seq, stamp)


def mask(first, last=0):
    return bytes([first] + [0] * 62 + [last])


DELTA_BRICKS = (m.Brick((-1, 2, -3), mask(0x01, 0x80), mask(0x03, 0x80)),)


def delta(compression=m.Compression.NONE):
    return m.MapBricks(head("lab", 43, 12), compression, m.MapKind.DELTA, 42, 0.05, DELTA_BRICKS, (0, 5))


def snapshot_zlib():
    bricks = DELTA_BRICKS + (m.Brick((0, 0, 1), bytes(64), b"\xff" * 64),)
    return m.MapBricks(head("lab", 43, 13), m.Compression.ZLIB, m.MapKind.SNAPSHOT, 0, 0.05, bricks)


def memory_status():
    return m.MemoryStatus(
        head("lab-a", 42, 7, m.Time(1760400000, 500000000)), m.MemoryState.TRACKING, 3200, 2, 0.75, None, 1.0, 10, None, 3, 12,
    )


def epoch():
    return m.Epoch(head("lab", 4294967298, 1000, m.Time(1760400005, 0)), 5, 1, 16909060, None, m.MemoryState.RIG_MOVED)


def moved_event():
    region = m.MapBricks(
        head("lab", 43, 0), m.Compression.NONE, m.MapKind.SNAPSHOT, 0, 0.05,
        (m.Brick((3, -1, 0), mask(0x01), mask(0x01)),),
    )
    return m.ChangeEvent(
        head("lab", 43, 3), 9, m.EventKind.MOVED, m.EventState.CONFIRMED, (1.5, -0.25, 0.125),
        (1.25, -0.5, 0.0), (1.75, 0.0, 0.25), (0.5, -0.25, 0.125), 125, 0.015625, 0.875, None,
        m.SENSOR_MID360 | m.SENSOR_JT128, m.Time(1760400000, 0), T10, region,
    )


def appeared_event():
    return m.ChangeEvent(
        head("lab", 43, 4), 10, m.EventKind.APPEARED, m.EventState.CANDIDATE, (2.0, 1.0, 0.5),
        (1.875, 0.875, 0.375), (2.125, 1.125, 0.625), None, 8, 0.001, None, 0.05, m.SENSOR_MID360,
        m.Time(1760400010, 0), T10,
    )


def activity():
    return m.Activity(head("lab", 43, 21), True, m.SENSOR_MID360 | m.SENSOR_JT128, m.SENSOR_JT128, 3, 16909060,
                      (0.5, 1.25, 0.0625), (0.25, 1.0, 0.0), (0.75, 1.5, 0.125), 2.5)


def activity_idle():
    return m.Activity(head("lab", 1, 0), False, 0, 0, 0, 0, None, None, None, None)


def rig_pose():
    return m.RigPose(head("lab", 43, 22), m.RigOutcome.REALIGNED, m.CAUSE_IMU_SHOCK | m.CAUSE_PERIODIC,
                     m.SENSOR_MID360 | m.SENSOR_JT128, (m.RigChannel.MOVED, m.RigChannel.IN_PLACE), 40, 16909060,
                     (0.03125, -0.0625, 0.0, 0.5, 0.5, 0.5, 0.5), (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0),
                     (0.03125, None), (0.5, None), (0.75, 0.875), (0.0078125, 0.00390625), (0.25, 0.125), 0.25, True,
                     m.Time(1760400005, 500000000))


def imu():
    return m.ImuSample(T10, "livox_frame", (0.015625, -0.03125, 0.5), (0.0078125, -0.0, -1.0))


class Imu:
    """``decode_imu`` in the ``from_cdr`` shape of the message classes."""

    from_cdr = staticmethod(m.decode_imu)


def request_map():
    return m.RequestSync(head("lab", 43, 2), (43, 12), m.SyncTopic.MAP_BRICKS)


def request_events_none():
    return m.RequestSync(
        head("lab-a", m.SINCE_NONE, 0, m.Time(1760400000, 500000000)), None, m.SyncTopic.EVENTS,
    )


def moved_event_v2():
    event = moved_event()
    return replace(event, mask=replace(event.mask, publisher_generation=G), publisher_generation=G)


def request_map_v2():
    return m.RequestSync(head("lab", 43, 2), (43, 10), m.SyncTopic.MAP_BRICKS, G, TOKEN_C)


def sync_end_events_empty():
    return m.SyncEnd(head("lab", 43, 0), G, m.ZERO_TOKEN, m.SyncTopic.EVENTS, None)


def sync_end_events_fallback():
    return m.SyncEnd(head("lab", 43, 1), G, TOKEN_B, m.SyncTopic.EVENTS, 120, ((10, 10), (21, 120)), ((11, 20),))


def sync_end_map_replay():
    return m.SyncEnd(head("lab", 45, 2), G, TOKEN_C, m.SyncTopic.MAP_BRICKS, 13, ((11, 13),))


# name: (type, value, encoder reproduces the bytes)
GOLDENS = {
    "memory_status_le.hex": (m.MemoryStatus, memory_status, True),
    "epoch_le.hex": (m.Epoch, epoch, True),
    "epoch_be.hex": (m.Epoch, epoch, False),
    "map_bricks_delta_le.hex": (m.MapBricks, delta, True),
    "map_bricks_delta_be.hex": (m.MapBricks, delta, False),
    "map_bricks_delta_lzfse_le.hex": (m.MapBricks, lambda: delta(m.Compression.LZFSE), False),
    "map_bricks_snapshot_zlib_le.hex": (m.MapBricks, snapshot_zlib, False),
    "change_event_moved_mask_le.hex": (m.ChangeEvent, moved_event, True),
    "change_event_appeared_le.hex": (m.ChangeEvent, appeared_event, True),
    "request_sync_map_le.hex": (m.RequestSync, request_map, True),
    "request_sync_map_be.hex": (m.RequestSync, request_map, False),
    "request_sync_events_none_le.hex": (m.RequestSync, request_events_none, True),
    "activity_le.hex": (m.Activity, activity, True),
    "activity_idle_le.hex": (m.Activity, activity_idle, True),
    "memory_status_v2_le.hex": (m.MemoryStatus, lambda: replace(memory_status(), publisher_generation=G), True),
    "map_bricks_delta_v2_le.hex": (m.MapBricks, lambda: replace(delta(), publisher_generation=G), True),
    "map_bricks_delta_v2_be.hex": (m.MapBricks, lambda: replace(delta(), publisher_generation=G), False),
    "change_event_appeared_v2_le.hex": (m.ChangeEvent, lambda: replace(appeared_event(), publisher_generation=G), True),
    "change_event_moved_mask_v2_le.hex": (m.ChangeEvent, moved_event_v2, True),
    "request_sync_events_v2_le.hex": (
        m.RequestSync, lambda: replace(request_events_none(), expected_generation=0, token=TOKEN_A), True),
    "request_sync_map_v2_be.hex": (m.RequestSync, request_map_v2, False),
    "sync_end_events_empty_le.hex": (m.SyncEnd, sync_end_events_empty, True),
    "sync_end_events_fallback_le.hex": (m.SyncEnd, sync_end_events_fallback, True),
    "sync_end_map_replay_be.hex": (m.SyncEnd, sync_end_map_replay, False),
    "epoch_v2_le.hex": (m.Epoch, lambda: replace(epoch(), publisher_generation=G), True),
    "epoch_v2_be.hex": (m.Epoch, lambda: replace(epoch(), publisher_generation=G), False),
    "activity_v2_le.hex": (m.Activity, lambda: replace(activity(), publisher_generation=G), True),
    "rig_pose_le.hex": (m.RigPose, rig_pose, True),
    "imu_le.hex": (Imu, imu, True),
}
# v2 goldens whose cut at the legacy length (+ up to 3 bytes) is a valid legacy message.
V2_EXTENDS_LEGACY = {name for name in GOLDENS if "_v2_" in name}
# Derived on rig-guard; the Swift suite decodes neither RigPose nor Imu, so it has no copy (PROVENANCE.md).
NOT_IN_SWIFT = {"rig_pose_le.hex", "imu_le.hex"}


def test_provenance_hashes():
    table = dict(re.findall(r"`([\w.]+\.hex)` \| `([0-9a-f]{64})`", (HAND / "PROVENANCE.md").read_text()))
    assert sorted(table) == sorted(path.name for path in HAND.glob("*.hex")) == sorted(GOLDENS)
    for name, digest in table.items():
        assert hashlib.sha256((HAND / name).read_bytes()).hexdigest() == digest, name


@pytest.mark.skipif(not SWIFT_HAND.is_dir(), reason=f"Swift hand goldens not found at {SWIFT_HAND}")
def test_hand_goldens_match_the_swift_copies():
    shared = sorted(set(GOLDENS) - NOT_IN_SWIFT)
    assert sorted(path.name for path in SWIFT_HAND.glob("*.hex")) == shared
    for name in shared:
        assert hexfile(name, SWIFT_HAND) == hexfile(name), name
    assert memory_status().to_cdr() == hexfile("memory_status_le.hex", SWIFT_HAND)


@pytest.mark.parametrize("name", sorted(GOLDENS))
def test_golden_decodes_to_value(name):
    kind, build, _ = GOLDENS[name]
    assert kind.from_cdr(hexfile(name)) == build()


@pytest.mark.parametrize("name", sorted(name for name, (_, _, exact) in GOLDENS.items() if exact))
def test_encoder_reproduces_golden(name):
    _, build, _ = GOLDENS[name]
    assert build().to_cdr() == hexfile(name)


def test_zlib_golden_header_and_payload():
    golden = hexfile("map_bricks_snapshot_zlib_le.hex")
    encoded = snapshot_zlib().to_cdr()
    assert encoded[:4 + 64] == golden[:4 + 64]
    length = struct.unpack_from("<I", encoded, 4 + 64)[0]
    assert len(encoded) == 4 + 68 + length
    assert zlib.decompress(encoded[4 + 68:], -15) == golden[4 + 73:4 + 341]
    assert not encoded[4 + 68:].startswith(b"\x78")
    assert m.MapBricks.from_cdr(encoded) == snapshot_zlib()


def test_lzfse_compressed_block_is_unsupported_and_not_encodable():
    golden = bytearray(hexfile("map_bricks_delta_lzfse_le.hex"))
    golden[4 + 68:4 + 72] = b"bvx2"
    with pytest.raises(m.Unsupported):
        m.MapBricks.from_cdr(bytes(golden))
    with pytest.raises(m.Unsupported):
        delta(m.Compression.LZFSE).to_cdr()


@pytest.mark.parametrize("name", sorted(GOLDENS))
def test_every_truncation_throws(name):
    kind, build, _ = GOLDENS[name]
    data = hexfile(name)
    legacy_cuts = 0
    for length in range(len(data)):
        try:
            value = kind.from_cdr(data[:length])
        except m.CDRError:
            continue
        # A v2 message cut where its legacy layout ends is that legacy message, never the golden.
        assert name in V2_EXTENDS_LEGACY, (name, length)
        generation = value.expected_generation if kind is m.RequestSync else value.publisher_generation
        assert generation is None and value != build(), (name, length)
        legacy_cuts += 1
    assert legacy_cuts <= 4


@pytest.mark.parametrize("name", sorted(GOLDENS))
def test_trailing_padding(name):
    kind, build, _ = GOLDENS[name]
    data = hexfile(name)
    assert kind.from_cdr(data + b"\x00" * 3) == build()
    with pytest.raises(m.Invalid):
        kind.from_cdr(data + b"\x00" * 4)


def test_activity_rejects_inconsistent_fields():
    data = bytearray(hexfile("activity_le.hex"))
    data[4 + 32] = 2  # active is a bool
    with pytest.raises(m.Invalid):
        m.Activity.from_cdr(bytes(data))
    data = bytearray(hexfile("activity_le.hex"))
    data[4 + 46:4 + 48] = b"\xc0\x7f"  # centroid.x NaN, the rest finite
    with pytest.raises(m.Invalid):
        m.Activity.from_cdr(bytes(data))
    data = bytearray(hexfile("activity_le.hex"))
    data[4 + 34] = 1  # largest_sensor MID360 is set in sensors: still valid
    assert m.Activity.from_cdr(bytes(data)).largest_sensor == m.SENSOR_MID360
    data[4 + 33] = 2  # but not when sensors is JT128 only
    with pytest.raises(m.Invalid):
        m.Activity.from_cdr(bytes(data))
    data = bytearray(hexfile("activity_idle_le.hex"))
    data[4 + 36] = 1  # an inactive message with a region
    with pytest.raises(m.Invalid):
        m.Activity.from_cdr(bytes(data))
    with pytest.raises(m.Invalid):
        replace(activity_idle(), since_s=-1.0).to_cdr()
    with pytest.raises(m.Invalid):
        replace(activity(), since_s=None).to_cdr()


def test_rig_pose_is_168_bytes_for_lab():
    assert len(rig_pose().to_cdr()) == 168


@pytest.mark.parametrize("offset, value", [
    (32, 6),  # outcome past INCONCLUSIVE
    (33, 64),  # an undefined cause bit
    (34, 4),  # an undefined sensor bit
    (35, 5),  # channel past UNAVAILABLE
    (152, 2),  # imu_ok is a bool
])
def test_rig_pose_rejects_bad_codes(offset, value):
    data = bytearray(hexfile("rig_pose_le.hex"))
    data[4 + offset] = value
    with pytest.raises(m.Invalid):
        m.RigPose.from_cdr(bytes(data))


def test_rig_pose_rejects_inconsistent_fields():
    data = bytearray(hexfile("rig_pose_le.hex"))
    data[4 + 34] = m.SENSOR_JT128  # the Mid-360 is outside sensors but has a MOVED result
    with pytest.raises(m.Invalid):
        m.RigPose.from_cdr(bytes(data))
    data[4 + 35] = m.RigChannel.UNAVAILABLE  # UNAVAILABLE is allowed for it
    assert m.RigPose.from_cdr(bytes(data)).channel == (m.RigChannel.UNAVAILABLE, m.RigChannel.IN_PLACE)
    data = bytearray(hexfile("rig_pose_le.hex"))
    data[4 + 52:4 + 56] = b"\x00\x00\xc0\x7f"  # mid360 tx NaN, the rest finite
    with pytest.raises(m.Invalid):
        m.RigPose.from_cdr(bytes(data))
    data = bytearray(hexfile("rig_pose_le.hex"))
    data[4 + 52:4 + 80] = b"\x00\x00\xc0\x7f" * 7  # NaN x 7: not configured
    assert m.RigPose.from_cdr(bytes(data)).mid360_pose is None
    data = bytearray(hexfile("rig_pose_le.hex"))
    data[4 + 124:4 + 128] = struct.pack("<f", math.inf)  # inlier_frac[0] infinite
    with pytest.raises(m.Invalid):
        m.RigPose.from_cdr(bytes(data))
    for change in ({"mid360_pose": (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.01)}, {"inlier_frac": (1.5, None)},
                   {"tilt_deg": -1.0}, {"delta_m": (math.inf, None)}, {"channel": (1,)}, {"imu_ok": 1}):
        with pytest.raises(m.Invalid):
            replace(rig_pose(), **change).to_cdr()


def test_decode_imu_rejects_non_finite_values():
    data = bytearray(hexfile("imu_le.hex"))
    data[4 + 128:4 + 136] = struct.pack("<d", math.nan)
    with pytest.raises(m.Invalid):
        m.decode_imu(bytes(data))


def test_encapsulation_and_strings():
    data = bytearray(hexfile("epoch_le.hex"))
    data[1] = 0x07
    with pytest.raises(m.Unsupported):
        m.Epoch.from_cdr(bytes(data))
    data = bytearray(hexfile("epoch_le.hex"))
    data[4 + 7] = 0x41
    with pytest.raises(m.Invalid, match="NUL-terminated"):
        m.Epoch.from_cdr(bytes(data))


def test_payload_size_and_stream_checks():
    raw = bytearray(hexfile("map_bricks_delta_le.hex"))
    raw[4 + 60] = 3
    with pytest.raises(m.CDRError):
        m.MapBricks.from_cdr(bytes(raw))

    def with_payload(payload):
        golden = hexfile("map_bricks_snapshot_zlib_le.hex")
        return golden[:4 + 64] + struct.pack("<I", len(payload)) + payload

    raw_payload = hexfile("map_bricks_snapshot_zlib_le.hex")[4 + 73:4 + 341]
    wrapped = zlib.compress(raw_payload)
    with pytest.raises(m.Invalid):
        m.MapBricks.from_cdr(with_payload(wrapped))
    stream = zlib.compressobj(6, zlib.DEFLATED, -15)
    good = stream.compress(raw_payload) + stream.flush()
    assert m.MapBricks.from_cdr(with_payload(good)) == snapshot_zlib()
    with pytest.raises(m.Invalid):
        m.MapBricks.from_cdr(with_payload(good + b"\x00"))
    short = zlib.compressobj(6, zlib.DEFLATED, -15)
    with pytest.raises(m.Invalid):
        m.MapBricks.from_cdr(with_payload(short.compress(raw_payload[:-1]) + short.flush()))
    with pytest.raises(m.Invalid):
        m.MapBricks.from_cdr(with_payload(b""))


def test_empty_zlib_payload_is_two_bytes():
    message = m.MapBricks(head("lab", 1, 0), m.Compression.ZLIB, m.MapKind.SNAPSHOT, 0, 0.05, ())
    encoded = message.to_cdr()
    assert encoded[-2:] == b"\x03\x00"
    assert m.MapBricks.from_cdr(encoded) == message


@pytest.mark.parametrize("change", [
    lambda b: replace(b, voxel_size=float("nan")),
    lambda b: replace(b, voxel_size=0.0),
    lambda b: replace(b, base_epoch=43),
    lambda b: replace(b, kind=m.MapKind.SNAPSHOT),
    lambda b: replace(b, removed=(5, 5)),
    lambda b: replace(b, bricks=DELTA_BRICKS * 2),
    lambda b: replace(b, bricks=(m.Brick((-1, 2, -3), mask(0x01), bytes(64)),)),
    lambda b: replace(b, bricks=(m.Brick((-1, 2, -3), bytes(64), bytes(64)),)),
    lambda b: replace(b, bricks=(m.Brick((40000, 0, 0), mask(1), mask(1)),)),
    lambda b: replace(b, header=head("", 43, 12)),
    lambda b: replace(b, header=head("a\x00b", 43, 12)),
    lambda b: replace(b, header=head("lab", 43, 12, m.Time(0, 1_000_000_000))),
])
def test_map_bricks_rejections(change):
    with pytest.raises(m.Invalid):
        change(delta()).to_cdr()


@pytest.mark.parametrize("change", [
    lambda e: replace(e, from_centroid=None),
    lambda e: replace(e, kind=m.EventKind.APPEARED),
    lambda e: replace(e, centroid=(math.nan, 0.0, 0.0)),
    lambda e: replace(e, aabb_min=(2.0, -0.5, 0.0)),
    lambda e: replace(e, volume_m3=-1.0),
    lambda e: replace(e, evidence=1.5),
    lambda e: replace(e, alpha=math.inf),
    lambda e: replace(e, sensors=4),
    lambda e: replace(e, first_seen=m.Time(1760400011, 0)),
    lambda e: replace(e, mask=replace(e.mask, header=head("lab", 43, 1))),
    lambda e: replace(e, mask=replace(e.mask, header=head("lab", 44, 0))),
    lambda e: replace(e, mask=replace(e.mask, header=head("other", 43, 0))),
    lambda e: replace(e, mask=replace(e.mask, header=head("lab", 43, 0, m.Time(0, 0)))),
])
def test_change_event_rejections(change):
    with pytest.raises(m.Invalid):
        change(moved_event()).to_cdr()


def test_change_event_partial_nan_from_centroid_is_invalid():
    data = bytearray(hexfile("change_event_appeared_le.hex"))
    data[4 + 80:4 + 84] = struct.pack("<f", 1.0)
    with pytest.raises(m.Invalid, match="partly NaN"):
        m.ChangeEvent.from_cdr(bytes(data))


@pytest.mark.parametrize("change", [
    lambda s: replace(s, coverage_known_frac=1.25),
    lambda s: replace(s, pose_residual_m=-0.1),
    lambda s: replace(s, window_s=0.0),
    lambda s: replace(s, mid360_hz=m.HZ_UNAVAILABLE),
    lambda s: replace(s, state=9),
    lambda s: replace(s, events_seq=m.EVENTS_SEQ_NONE),
])
def test_memory_status_rejections(change):
    with pytest.raises(m.Invalid):
        change(memory_status()).to_cdr()


def test_memory_status_events_seq_none():
    data = bytearray(hexfile("memory_status_le.hex"))
    assert len(data) == 4 + 80
    data[4 + 72:4 + 80] = b"\xff" * 8
    expected = replace(memory_status(), events_seq=None)
    assert m.MemoryStatus.from_cdr(bytes(data)) == expected
    assert expected.to_cdr() == bytes(data)


def test_request_sync_rules():
    with pytest.raises(m.Invalid):
        replace(request_map(), header=head("lab", 42, 2)).to_cdr()
    with pytest.raises(m.Invalid):
        replace(request_map(), since=(m.SINCE_NONE, 12)).to_cdr()
    data = bytearray(hexfile("request_sync_map_le.hex"))
    data[4 + 40:4 + 48] = b"\xff" * 8
    with pytest.raises(m.Invalid, match="both be SINCE_NONE"):
        m.RequestSync.from_cdr(bytes(data))
    data = bytearray(hexfile("request_sync_map_le.hex"))
    data[4 + 48] = 2
    with pytest.raises(m.Invalid):
        m.RequestSync.from_cdr(bytes(data))


def test_float_fields_hold_binary32():
    status = replace(memory_status(), coverage_known_frac=0.1)
    assert status.coverage_known_frac == struct.unpack("<f", struct.pack("<f", 0.1))[0]
    assert m.MemoryStatus.from_cdr(status.to_cdr()) == status


# MARK: Protocol v2


def test_layout_detection_from_trailing_bytes():
    legacy = hexfile("memory_status_le.hex")
    assert m.MemoryStatus.from_cdr(legacy).publisher_generation is None
    for extra in range(4, 8):
        with pytest.raises(m.Invalid, match="neither padding"):
            m.MemoryStatus.from_cdr(legacy + bytes(extra))
    with pytest.raises(m.Invalid, match="GENERATION_UNKNOWN"):
        m.MemoryStatus.from_cdr(legacy + bytes(8))  # generation 0 rejects 8 appended zero bytes
    assert m.MemoryStatus.from_cdr(legacy + struct.pack("<Q", 7) + bytes(3)).publisher_generation == 7
    with pytest.raises(m.Invalid):
        m.MemoryStatus.from_cdr(legacy + struct.pack("<Q", 7) + bytes(4))
    # MapBricks: the pad before the generation depends on the payload length.
    v2 = hexfile("map_bricks_delta_v2_le.hex")
    legacy = hexfile("map_bricks_delta_le.hex")
    assert v2[:len(legacy)] == legacy and len(v2) == len(legacy) + 6 + 8
    with pytest.raises(m.Invalid):
        m.MapBricks.from_cdr(legacy + bytes(6) + struct.pack("<Q", G)[:7])
    # RequestSync: 24 v2 bytes after the pad.
    request = hexfile("request_sync_events_none_le.hex")
    assert m.RequestSync.from_cdr(request).token is None
    with pytest.raises(m.Invalid):
        m.RequestSync.from_cdr(request + bytes(7 + 8))
    with pytest.raises(m.Invalid, match="all-zero"):
        m.RequestSync.from_cdr(request + bytes(7 + 24))
    # Epoch ends on a uint8 (7 pad bytes before the generation), Activity on a float32 (4).
    legacy = hexfile("epoch_le.hex")
    assert legacy + bytes(7) + struct.pack("<Q", G) == hexfile("epoch_v2_le.hex")
    with pytest.raises(m.Invalid, match="GENERATION_UNKNOWN"):
        m.Epoch.from_cdr(legacy + bytes(7 + 8))
    with pytest.raises(m.Invalid, match="neither padding"):
        m.Epoch.from_cdr(legacy + bytes(3) + struct.pack("<Q", G))
    legacy = hexfile("activity_le.hex")
    assert legacy + bytes(4) + struct.pack("<Q", G) == hexfile("activity_v2_le.hex")
    with pytest.raises(m.Invalid, match="GENERATION_UNKNOWN"):
        m.Activity.from_cdr(legacy + bytes(4 + 8))


def test_v2_mask_follows_the_parent():
    event = moved_event_v2()
    with pytest.raises(m.Invalid, match="mask publisher_generation"):
        replace(event, mask=replace(event.mask, publisher_generation=None)).to_cdr()
    with pytest.raises(m.Invalid, match="mask publisher_generation"):
        replace(event, publisher_generation=None).to_cdr()
    with pytest.raises(m.Invalid, match="mask publisher_generation"):
        replace(event, mask=replace(event.mask, publisher_generation=G + 1)).to_cdr()
    data = bytearray(hexfile("change_event_moved_mask_v2_le.hex"))
    data[4 + 344:4 + 352] = struct.pack("<Q", G + 1)
    with pytest.raises(m.Invalid, match="mask publisher_generation"):
        m.ChangeEvent.from_cdr(bytes(data))
    # Only the parent's generation after a v2 mask is a bad length, not a legacy parent.
    data = hexfile("change_event_moved_mask_v2_le.hex")
    with pytest.raises(m.Invalid, match="neither padding"):
        m.ChangeEvent.from_cdr(data[:-8])
    assert m.ChangeEvent.from_cdr(hexfile("change_event_moved_mask_le.hex")) == moved_event()


@pytest.mark.parametrize("change", [
    lambda r: replace(r, token=None),
    lambda r: replace(r, expected_generation=None),
    lambda r: replace(r, token=m.ZERO_TOKEN),
    lambda r: replace(r, token=TOKEN_A[:15]),
    lambda r: replace(r, expected_generation=-1),
])
def test_request_sync_v2_rejections(change):
    with pytest.raises(m.Invalid):
        change(request_map_v2()).to_cdr()


def test_request_sync_v2_round_trip():
    request = request_map_v2()
    assert m.RequestSync.from_cdr(request.to_cdr()) == request
    assert len(request.to_cdr()) == 84
    assert hexfile("request_sync_map_v2_be.hex")[4 + 64:] == TOKEN_C  # uint8[16] is never reversed


def test_request_sync_ignores_padding_bytes():
    # The node's RequestSync service re-serializes the request with rmw, which leaves alignment
    # padding uninitialized (D39): pad bytes are never checked, so the value is unchanged.
    request = request_map_v2()
    data = bytearray(request.to_cdr())
    assert data[4 + 48:4 + 56] == bytes([request.topic]) + bytes(7)  # topic, then 7 pad bytes
    data[4 + 49:4 + 56] = b"\xa5" * 7
    assert m.RequestSync.from_cdr(bytes(data)) == request


@pytest.mark.parametrize("change", [
    lambda s: replace(s, publisher_generation=0),
    lambda s: replace(s, token=TOKEN_A[:8]),
    lambda s: replace(s, topic=2),
    lambda s: replace(s, bound=m.SEQ_NONE),
    lambda s: replace(s, bound=119),
    lambda s: replace(s, bound=None),
    lambda s: replace(s, delivered=((21, 120), (10, 10))),
    lambda s: replace(s, delivered=((10, 20), (21, 120)), unavailable=()),
    lambda s: replace(s, delivered=((10, 10), (10, 120))),
    lambda s: replace(s, delivered=((11, 10),)),
    lambda s: replace(s, unavailable=((11, 21),)),
    lambda s: replace(s, topic=m.SyncTopic.MAP_BRICKS),
])
def test_sync_end_rejections(change):
    with pytest.raises(m.Invalid):
        change(sync_end_events_fallback()).to_cdr()


def test_sync_end_sizes_and_patches():
    assert len(sync_end_events_empty().to_cdr()) == 84
    assert len(replace(sync_end_events_empty(), bound=5, delivered=((0, 5),)).to_cdr()) == 88 + 16
    assert len(replace(sync_end_events_empty(), bound=5, unavailable=((0, 5),)).to_cdr()) == 84 + 16
    assert len(sync_end_events_fallback().to_cdr()) == 92 + 32 + 16
    data = bytearray(hexfile("sync_end_events_fallback_le.hex"))
    data[4 + 72] = 3  # three delivered ranges: truncated
    with pytest.raises(m.CDRError):
        m.SyncEnd.from_cdr(bytes(data))
    data = bytearray(hexfile("sync_end_events_fallback_le.hex"))
    data[4 + 72:4 + 76] = struct.pack("<I", 0xFFFFFFFF)  # a hostile count is refused before allocating
    with pytest.raises(m.Truncated):
        m.SyncEnd.from_cdr(bytes(data))
    data = bytearray(hexfile("sync_end_events_empty_le.hex"))
    data[4 + 32:4 + 40] = bytes(8)
    with pytest.raises(m.Invalid, match="GENERATION_UNKNOWN"):
        m.SyncEnd.from_cdr(bytes(data))
