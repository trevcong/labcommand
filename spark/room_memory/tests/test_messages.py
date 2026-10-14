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
T10 = m.Time(1760400010, 250000000)


def hexfile(name):
    text = (HAND / name).read_text()
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
        head("lab-a", 42, 7, m.Time(1760400000, 500000000)), m.MemoryState.TRACKING, 3200, 2, 0.75, None, 1.0, 10, None, 3,
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


def request_map():
    return m.RequestSync(head("lab", 43, 2), (43, 12), m.SyncTopic.MAP_BRICKS)


def request_events_none():
    return m.RequestSync(
        head("lab-a", m.SINCE_NONE, 0, m.Time(1760400000, 500000000)), None, m.SyncTopic.EVENTS,
    )


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
}


def test_provenance_hashes():
    table = dict(re.findall(r"`([\w.]+\.hex)` \| `([0-9a-f]{64})`", (HAND / "PROVENANCE.md").read_text()))
    assert sorted(table) == sorted(path.name for path in HAND.glob("*.hex")) == sorted(GOLDENS)
    for name, digest in table.items():
        assert hashlib.sha256((HAND / name).read_bytes()).hexdigest() == digest, name


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
    kind, _, _ = GOLDENS[name]
    data = hexfile(name)
    for length in range(len(data)):
        with pytest.raises(m.CDRError):
            kind.from_cdr(data[:length])


@pytest.mark.parametrize("name", sorted(GOLDENS))
def test_trailing_padding(name):
    kind, build, _ = GOLDENS[name]
    data = hexfile(name)
    assert kind.from_cdr(data + b"\x00" * 3) == build()
    with pytest.raises(m.Invalid):
        kind.from_cdr(data + b"\x00" * 4)


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
])
def test_memory_status_rejections(change):
    with pytest.raises(m.Invalid):
        change(memory_status()).to_cdr()


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
