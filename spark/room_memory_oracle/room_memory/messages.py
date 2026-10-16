"""CDR codecs for the five ``labcommand_msgs`` room-memory messages, with no ROS imports.

The contract is ``docs/ROOM-MEMORY-PROTOCOL.md`` (Phase 0 branch). Classic CDR (XCDR1):
4-byte encapsulation header (``00 01`` little endian, ``00 00`` big endian), every primitive
aligned to its own size counted from the byte after the header, strings as a ``uint32``
length that includes the NUL, ``T[]`` as a ``uint32`` count, nested messages without
alignment of their own, at most 3 trailing padding bytes. Decoders read either byte order;
encoders write little endian. Validation is symmetric (protocol D16): ``to_cdr`` refuses
exactly what ``from_cdr`` rejects. Float fields hold binary32 values; a missing float
measurement is ``None`` here and NaN on the wire.
"""

import math
import struct
import zlib
from dataclasses import dataclass
from enum import IntEnum
from typing import Optional, Tuple

import numpy as np

BRICK_DIM = 8
MASK_BYTES = 64
COORDINATE_BYTES = 6
BYTES_PER_BRICK = COORDINATE_BYTES + 2 * MASK_BYTES
MAX_PAYLOAD_BYTES = 16 << 20
MAX_TRAILING_PADDING = 3
HZ_UNAVAILABLE = 0xFFFF
SINCE_NONE = (1 << 64) - 1
EVENTS_SEQ_NONE = (1 << 64) - 1
SENSOR_MID360 = 1
SENSOR_JT128 = 2
SENSORS_ALL = SENSOR_MID360 | SENSOR_JT128
_NAN_BITS = 0x7FC00000


class CDRError(ValueError):
    """Base class for codec failures."""


class Truncated(CDRError):
    """The buffer ends before a field does."""


class Invalid(CDRError):
    """A value breaks the protocol contract."""


class Unsupported(CDRError):
    """Well-formed input this codec does not handle (encapsulation, ``brick_dim``, LZFSE blocks)."""


class Compression(IntEnum):
    """``MapBricks.compression``: how the payload is encoded."""

    NONE = 0
    ZLIB = 1
    LZFSE = 2


class MapKind(IntEnum):
    """``MapBricks.kind``."""

    SNAPSHOT = 0
    DELTA = 1


class EventKind(IntEnum):
    """``ChangeEvent.kind``."""

    APPEARED = 0
    DISAPPEARED = 1
    MOVED = 2


class EventState(IntEnum):
    """``ChangeEvent.state``."""

    CANDIDATE = 0
    CONFIRMED = 1
    ACCEPTED = 2
    REVERTED = 3
    EXPIRED = 4


class MemoryState(IntEnum):
    """``MemoryStatus.state`` and ``Epoch.state``."""

    BUILDING = 0
    TRACKING = 1
    RIG_MOVED = 2
    DEGRADED = 3
    PAUSED = 4


class SyncTopic(IntEnum):
    """``RequestSync.topic``."""

    MAP_BRICKS = 0
    EVENTS = 1


def f32(value):
    """Rounds a number to binary32 and returns it as a Python float (overflow gives ±inf)."""
    with np.errstate(over="ignore"):
        return float(np.float32(value))


def _f32_fields(instance, names):
    for name in names:
        value = getattr(instance, name)
        if value is None:
            continue
        if isinstance(value, tuple):
            object.__setattr__(instance, name, tuple(f32(item) for item in value))
        else:
            object.__setattr__(instance, name, f32(value))


def _enum(kind, value, what):
    try:
        return kind(value)
    except ValueError:
        raise Invalid(f"{what} {value} is not a {kind.__name__} value") from None


def _require_int(value, low, high, what):
    if not isinstance(value, int) or not low <= value <= high:
        raise Invalid(f"{what} {value} is outside {low}...{high}")


def _require_finite(values, what):
    if not all(math.isfinite(value) for value in values):
        raise Invalid(f"{what} {values} is not finite")


def _require_fraction(value, what):
    if value is not None and not 0 <= value <= 1:
        raise Invalid(f"{what} {value} is outside 0...1")


# MARK: Common types


@dataclass(frozen=True)
class Time:
    """``builtin_interfaces/Time``: seconds and nanoseconds since the Unix epoch."""

    sec: int
    nanosec: int

    def validate(self, what):
        _require_int(self.sec, -(1 << 31), (1 << 31) - 1, f"{what}.sec")
        _require_int(self.nanosec, 0, (1 << 32) - 1, f"{what}.nanosec")
        if self.nanosec >= 1_000_000_000:
            raise Invalid(f"{what}.nanosec {self.nanosec} is not below 1e9")


@dataclass(frozen=True)
class Header:
    """The four common fields that start every ``labcommand_msgs`` message."""

    room_id: str
    map_epoch: int
    seq: int
    stamp: Time

    def validate(self, context):
        if not self.room_id:
            raise Invalid(f"{context}: room_id is empty")
        if "\x00" in self.room_id:
            raise Invalid(f"{context}: room_id contains a NUL")
        _require_int(self.map_epoch, 0, SINCE_NONE, f"{context}: map_epoch")
        _require_int(self.seq, 0, SINCE_NONE, f"{context}: seq")
        self.stamp.validate(f"{context}: stamp")


@dataclass(frozen=True)
class Brick:
    """One 8 x 8 x 8 brick. Voxel ``(x, y, z)`` is bit ``x + 8y + 64z``, LSB first."""

    coordinate: Tuple[int, int, int]
    occupied: bytes
    known: bytes


# MARK: Wire


class _Writer:
    def __init__(self):
        self.data = bytearray(b"\x00\x01\x00\x00")

    def align(self, size):
        while (len(self.data) - 4) % size:
            self.data.append(0)

    def u8(self, value):
        self.data.append(value)

    def _put(self, fmt, size, value):
        self.align(size)
        self.data.extend(struct.pack(fmt, value))

    def u16(self, value):
        self._put("<H", 2, value)

    def i16(self, value):
        self._put("<h", 2, value)

    def u32(self, value):
        self._put("<I", 4, value)

    def u64(self, value):
        self._put("<Q", 8, value)

    def f32(self, value):
        self.u32(_NAN_BITS if value is None or math.isnan(value) else struct.unpack("<I", struct.pack("<f", value))[0])

    def floats(self, values):
        for value in values:
            self.f32(value)

    def string(self, text):
        body = text.encode("utf-8")
        self.u32(len(body) + 1)
        self.data.extend(body + b"\x00")

    def time(self, value):
        self.u32(value.sec & 0xFFFFFFFF)
        self.u32(value.nanosec)

    def header(self, value):
        self.string(value.room_id)
        self.u64(value.map_epoch)
        self.u64(value.seq)
        self.time(value.stamp)


class _Reader:
    def __init__(self, data, message):
        self.data = memoryview(data).cast("B")
        self.message = message
        if len(self.data) < 4:
            raise Truncated(f"{message} CDR: truncated encapsulation header ({len(self.data)} of 4 bytes)")
        if self.data[0] == 0x00 and self.data[1] == 0x01:
            self.order = "<"
        elif self.data[0] == 0x00 and self.data[1] == 0x00:
            self.order = ">"
        else:
            raise Unsupported(
                f"{message} CDR: unsupported encapsulation 0x{self.data[0]:02x} 0x{self.data[1]:02x} "
                "(expected CDR_LE 0x00 0x01 or CDR_BE 0x00 0x00)"
            )
        self.offset = 4

    def take(self, length, what):
        if length > len(self.data) - self.offset:
            have = max(0, len(self.data) - self.offset)
            raise Truncated(f"{self.message} CDR: truncated {what} at byte {self.offset} (need {length}, have {have})")
        start = self.offset
        self.offset += length
        return start

    def align(self, size):
        misalignment = (self.offset - 4) % size
        if misalignment:
            self.offset += size - misalignment

    def _get(self, fmt, size, what):
        self.align(size)
        start = self.take(size, what)
        return struct.unpack_from(self.order + fmt, self.data, start)[0]

    def u8(self, what):
        return self.data[self.take(1, what)]

    def u16(self, what):
        return self._get("H", 2, what)

    def i16(self, what):
        return self._get("h", 2, what)

    def u32(self, what):
        return self._get("I", 4, what)

    def i32(self, what):
        return self._get("i", 4, what)

    def u64(self, what):
        return self._get("Q", 8, what)

    def f32(self, what):
        return self._get("f", 4, what)

    def floats(self, what, count):
        return tuple(self.f32(f"{what}[{index}]") for index in range(count))

    def string(self, what):
        length = self.u32(what + " length")
        start = self.take(length, what)
        raw = bytes(self.data[start:start + length])
        if length < 1 or raw[-1] != 0:
            raise Invalid(f"{self.message} CDR: {what} is not NUL-terminated")
        try:
            if b"\x00" in raw[:-1]:
                raise UnicodeError
            return raw[:-1].decode("utf-8")
        except UnicodeError:
            raise Invalid(f"{self.message} CDR: {what} is not NUL-free UTF-8") from None

    def time(self, what):
        return Time(self.i32(what + ".sec"), self.u32(what + ".nanosec"))

    def header(self, path):
        return Header(
            room_id=self.string(path + "room_id"),
            map_epoch=self.u64(path + "map_epoch"),
            seq=self.u64(path + "seq"),
            stamp=self.time(path + "stamp"),
        )

    def byte_sequence(self, what):
        count = self.u32(what + " length")
        available = max(0, len(self.data) - self.offset)
        if count > available:
            raise Truncated(f"{self.message} CDR: truncated {what} at byte {self.offset} ({count} bytes, have {available})")
        start = self.take(count, what)
        return bytes(self.data[start:start + count])

    def finish(self):
        trailing = len(self.data) - self.offset
        if trailing > MAX_TRAILING_PADDING:
            raise Invalid(
                f"{self.message} CDR: {trailing} bytes after the message (at most {MAX_TRAILING_PADDING} of padding)"
            )


def _decode(data, message, body):
    reader = _Reader(data, message)
    value = body(reader)
    reader.finish()
    return value


# MARK: MapBricks payload


def payload_size(bricks, removed):
    """Uncompressed payload bytes: 134 per brick plus 4 per removed index."""
    return bricks * BYTES_PER_BRICK + removed * 4


def _pack_payload(bricks, removed):
    parts = [struct.pack("<3h", *brick.coordinate) for brick in bricks]
    parts += [brick.occupied for brick in bricks]
    parts += [brick.known for brick in bricks]
    parts.append(struct.pack(f"<{len(removed)}I", *removed))
    return b"".join(parts)


def _unpack_payload(raw, brick_count, removed_count):
    occupied_start = brick_count * COORDINATE_BYTES
    known_start = occupied_start + brick_count * MASK_BYTES
    removed_start = known_start + brick_count * MASK_BYTES
    bricks = []
    for index in range(brick_count):
        mask = index * MASK_BYTES
        bricks.append(Brick(
            coordinate=struct.unpack_from("<3h", raw, index * COORDINATE_BYTES),
            occupied=raw[occupied_start + mask:occupied_start + mask + MASK_BYTES],
            known=raw[known_start + mask:known_start + mask + MASK_BYTES],
        ))
    removed = struct.unpack_from(f"<{removed_count}I", raw, removed_start)
    return tuple(bricks), tuple(removed)


def _compress(raw, compression):
    if compression == Compression.NONE:
        return raw
    if compression == Compression.ZLIB:
        encoder = zlib.compressobj(6, zlib.DEFLATED, -15)
        return encoder.compress(raw) + encoder.flush()
    raise Unsupported("MapBricks: this encoder writes COMPRESSION_NONE or COMPRESSION_ZLIB; the Spark sends zlib")


def _inflate_lzfse(payload, expected, what):
    out = bytearray()
    offset = 0
    while True:
        magic = payload[offset:offset + 4]
        if magic == b"bvx$":
            offset += 4
            break
        if magic in (b"bvx1", b"bvx2", b"bvxn"):
            raise Unsupported(f"{what}: compressed LZFSE blocks need Apple's Compression framework")
        if magic != b"bvx-" or offset + 8 > len(payload):
            raise Invalid(f"{what}: {len(payload)} LZFSE bytes are not one complete stream of {expected} bytes")
        count = struct.unpack_from("<I", payload, offset + 4)[0]
        if len(out) + count > expected or offset + 8 + count > len(payload):
            raise Invalid(f"{what}: {len(payload)} LZFSE bytes are not one complete stream of {expected} bytes")
        out += payload[offset + 8:offset + 8 + count]
        offset += 8 + count
    if offset != len(payload) or len(out) != expected:
        raise Invalid(f"{what}: {len(payload)} LZFSE bytes are not one complete stream of {expected} bytes")
    return bytes(out)


def _decompress(payload, compression, expected, what):
    if compression == Compression.NONE:
        if len(payload) != expected:
            raise Invalid(f"{what} holds {len(payload)} bytes, expected {expected}")
        return payload
    if not payload:
        raise Invalid(f"{what}: an empty {compression.name} payload is not a stream")
    if compression == Compression.LZFSE:
        return _inflate_lzfse(payload, expected, what)
    decoder = zlib.decompressobj(-15)
    try:
        raw = decoder.decompress(payload, expected + 1)
    except zlib.error as error:
        raise Invalid(f"{what}: {len(payload)} ZLIB bytes do not inflate: {error}") from None
    if not decoder.eof or len(raw) != expected or decoder.unused_data or decoder.unconsumed_tail:
        raise Invalid(f"{what}: {len(payload)} ZLIB bytes are not one complete stream of {expected} bytes")
    return raw


# MARK: Messages


@dataclass(frozen=True)
class MapBricks:
    """Snapshot or delta of a room's voxel memory (protocol §3.1)."""

    header: Header
    compression: Compression
    kind: MapKind
    base_epoch: int
    voxel_size: float
    bricks: Tuple[Brick, ...]
    removed: Tuple[int, ...] = ()

    def __post_init__(self):
        _f32_fields(self, ("voxel_size",))

    def validate(self, context="MapBricks"):
        self.header.validate(context)
        _enum(Compression, self.compression, f"{context}: compression")
        _enum(MapKind, self.kind, f"{context}: kind")
        _require_int(self.base_epoch, 0, SINCE_NONE, f"{context}: base_epoch")
        if not (math.isfinite(self.voxel_size) and self.voxel_size > 0):
            raise Invalid(f"{context}: voxel_size {self.voxel_size} must be finite and > 0 m")
        if self.kind == MapKind.DELTA and self.header.map_epoch <= self.base_epoch:
            raise Invalid(f"{context}: delta map_epoch {self.header.map_epoch} is not after base_epoch {self.base_epoch}")
        if self.kind == MapKind.SNAPSHOT and self.removed:
            raise Invalid(f"{context}: a snapshot carries {len(self.removed)} removed_bricks_idx")
        for index, value in enumerate(self.removed):
            _require_int(value, 0, (1 << 32) - 1, f"{context}: removed_bricks_idx[{index}]")
            if index and self.removed[index - 1] >= value:
                raise Invalid(f"{context}: removed_bricks_idx not strictly increasing at {index}")
        size = payload_size(len(self.bricks), len(self.removed))
        if size > MAX_PAYLOAD_BYTES:
            raise Invalid(f"{context}: {len(self.bricks)} bricks need a {size}-byte payload; the limit is {MAX_PAYLOAD_BYTES}")
        for index, brick in enumerate(self.bricks):
            if len(brick.occupied) != MASK_BYTES or len(brick.known) != MASK_BYTES:
                raise Invalid(
                    f"{context}: brick {index} masks are {len(brick.occupied)}/{len(brick.known)} bytes, not {MASK_BYTES}"
                )
            for axis, value in zip("ijk", brick.coordinate):
                _require_int(value, -(1 << 15), (1 << 15) - 1, f"{context}: brick {index} {axis}")
            if index and tuple(self.bricks[index - 1].coordinate) >= tuple(brick.coordinate):
                raise Invalid(f"{context}: brick {index} is not after brick {index - 1} in (i, j, k) order")
            if any(occ & ~known & 0xFF for occ, known in zip(brick.occupied, brick.known)):
                raise Invalid(f"{context}: brick {index} marks a voxel occupied that is not known")
            if not any(brick.known):
                raise Invalid(f"{context}: brick {index} has no known voxel; an all-unknown brick is omitted")

    def _write(self, writer):
        writer.header(self.header)
        writer.u8(self.compression)
        writer.u8(self.kind)
        writer.u64(self.base_epoch)
        writer.f32(self.voxel_size)
        writer.u8(BRICK_DIM)
        writer.u32(len(self.bricks))
        writer.u32(len(self.removed))
        payload = _compress(_pack_payload(self.bricks, self.removed), self.compression)
        writer.u32(len(payload))
        writer.data.extend(payload)

    @classmethod
    def _read(cls, reader, path):
        header = reader.header(path)
        compression = _enum(Compression, reader.u8(path + "compression"), f"MapBricks: {path}compression")
        kind = _enum(MapKind, reader.u8(path + "kind"), f"MapBricks: {path}kind")
        base_epoch = reader.u64(path + "base_epoch")
        voxel_size = reader.f32(path + "voxel_size")
        brick_dim = reader.u8(path + "brick_dim")
        if brick_dim != BRICK_DIM:
            raise Unsupported(f"MapBricks: {path}brick_dim {brick_dim}; v1 handles only {BRICK_DIM}")
        brick_count = reader.u32(path + "n_bricks")
        removed_count = reader.u32(path + "n_removed")
        size = payload_size(brick_count, removed_count)
        if size > MAX_PAYLOAD_BYTES:
            raise Invalid(
                f"MapBricks: {path}n_bricks {brick_count} and n_removed {removed_count} need a {size}-byte payload; "
                f"the limit is {MAX_PAYLOAD_BYTES}"
            )
        payload = reader.byte_sequence(path + "payload")
        raw = _decompress(payload, compression, size, f"MapBricks: {path}payload")
        bricks, removed = _unpack_payload(raw, brick_count, removed_count)
        return cls(header, compression, kind, base_epoch, voxel_size, bricks, removed)

    def to_cdr(self):
        """CDR little endian, encapsulation header included. Raises ``Invalid``/``Unsupported``."""
        self.validate()
        writer = _Writer()
        self._write(writer)
        return bytes(writer.data)

    @classmethod
    def from_cdr(cls, data):
        """Decodes either byte order. Raises ``Truncated``, ``Invalid`` or ``Unsupported``."""
        def body(reader):
            message = cls._read(reader, "")
            message.validate()
            return message
        return _decode(data, "MapBricks", body)


@dataclass(frozen=True)
class ChangeEvent:
    """One lifecycle transition of one change (protocol §3.2). Coordinates in meters."""

    header: Header
    event_id: int
    kind: EventKind
    state: EventState
    centroid: Tuple[float, float, float]
    aabb_min: Tuple[float, float, float]
    aabb_max: Tuple[float, float, float]
    from_centroid: Optional[Tuple[float, float, float]]
    voxel_count: int
    volume_m3: float
    evidence: Optional[float]
    alpha: Optional[float]
    sensors: int
    first_seen: Time
    last_seen: Time
    mask: Optional[MapBricks] = None

    def __post_init__(self):
        _f32_fields(self, ("centroid", "aabb_min", "aabb_max", "from_centroid", "volume_m3", "evidence", "alpha"))

    def validate(self):
        context = "ChangeEvent"
        self.header.validate(context)
        _require_int(self.event_id, 0, SINCE_NONE, f"{context}: event_id")
        kind = _enum(EventKind, self.kind, f"{context}: kind")
        _enum(EventState, self.state, f"{context}: state")
        _require_finite(self.centroid, f"{context}: centroid")
        _require_finite(self.aabb_min, f"{context}: aabb min")
        _require_finite(self.aabb_max, f"{context}: aabb max")
        if any(low > high for low, high in zip(self.aabb_min, self.aabb_max)):
            raise Invalid(f"{context}: aabb min {self.aabb_min} exceeds max {self.aabb_max}")
        if kind == EventKind.MOVED and self.from_centroid is None:
            raise Invalid(f"{context}: MOVED needs from_centroid")
        if kind != EventKind.MOVED and self.from_centroid is not None:
            raise Invalid(f"{context}: from_centroid is only for MOVED, not {kind.name}")
        if self.from_centroid is not None:
            _require_finite(self.from_centroid, f"{context}: from_centroid")
        _require_int(self.voxel_count, 0, (1 << 32) - 1, f"{context}: voxel_count")
        if not (math.isfinite(self.volume_m3) and self.volume_m3 >= 0):
            raise Invalid(f"{context}: volume_m3 {self.volume_m3} must be finite and >= 0")
        _require_fraction(self.evidence, f"{context}: evidence")
        _require_fraction(self.alpha, f"{context}: alpha")
        _require_int(self.sensors, 0, 0xFF, f"{context}: sensors")
        if self.sensors & ~SENSORS_ALL:
            raise Invalid(f"{context}: sensors 0x{self.sensors:x} sets an undefined bit")
        self.first_seen.validate(f"{context}: first_seen")
        self.last_seen.validate(f"{context}: last_seen")
        if (self.first_seen.sec, self.first_seen.nanosec) > (self.last_seen.sec, self.last_seen.nanosec):
            raise Invalid(f"{context}: last_seen is before first_seen")
        if self.mask is not None:
            if self.mask.kind != MapKind.SNAPSHOT:
                raise Invalid(f"{context}: mask must be KIND_SNAPSHOT")
            self.mask.validate(f"{context} mask[0]")
            if self.mask.header.room_id != self.header.room_id:
                raise Invalid(f"{context}: mask room_id '{self.mask.header.room_id}' is not the event's '{self.header.room_id}'")
            if self.mask.header.map_epoch != self.header.map_epoch:
                raise Invalid(f"{context}: mask map_epoch {self.mask.header.map_epoch} is not the event's {self.header.map_epoch}")
            if self.mask.header.stamp != self.header.stamp:
                raise Invalid(f"{context}: mask stamp is not the event's stamp")
            if self.mask.header.seq != 0:
                raise Invalid(f"{context}: mask seq {self.mask.header.seq} must be 0")
            if self.mask.base_epoch != 0:
                raise Invalid(f"{context}: mask base_epoch {self.mask.base_epoch} must be 0")

    def to_cdr(self):
        """CDR little endian, encapsulation header included. Raises ``Invalid``/``Unsupported``."""
        self.validate()
        writer = _Writer()
        writer.header(self.header)
        writer.u64(self.event_id)
        writer.u8(self.kind)
        writer.u8(self.state)
        writer.floats(self.centroid)
        writer.floats(self.aabb_min)
        writer.floats(self.aabb_max)
        writer.floats(self.from_centroid or (None, None, None))
        writer.u32(self.voxel_count)
        writer.f32(self.volume_m3)
        writer.f32(self.evidence)
        writer.f32(self.alpha)
        writer.u8(self.sensors)
        writer.time(self.first_seen)
        writer.time(self.last_seen)
        writer.u32(0 if self.mask is None else 1)
        if self.mask is not None:
            self.mask._write(writer)
        return bytes(writer.data)

    @classmethod
    def from_cdr(cls, data):
        """Decodes either byte order. Raises ``Truncated``, ``Invalid`` or ``Unsupported``."""
        def body(reader):
            header = reader.header("")
            event_id = reader.u64("event_id")
            kind = _enum(EventKind, reader.u8("kind"), "ChangeEvent: kind")
            state = _enum(EventState, reader.u8("state"), "ChangeEvent: state")
            centroid = reader.floats("centroid", 3)
            aabb_min = reader.floats("aabb", 3)
            aabb_max = reader.floats("aabb", 3)
            from_centroid = reader.floats("from_centroid", 3)
            nans = [math.isnan(value) for value in from_centroid]
            if any(nans) and not all(nans):
                raise Invalid(f"ChangeEvent: from_centroid {from_centroid} is partly NaN")
            voxel_count = reader.u32("voxel_count")
            volume = reader.f32("volume_m3")
            evidence = reader.f32("evidence")
            alpha = reader.f32("alpha")
            sensors = reader.u8("sensors")
            first_seen = reader.time("first_seen")
            last_seen = reader.time("last_seen")
            mask_count = reader.u32("mask length")
            if mask_count > 1:
                raise Invalid(f"ChangeEvent: mask holds {mask_count} maps; the bound is 1")
            mask = MapBricks._read(reader, "mask[0].") if mask_count else None
            event = cls(
                header, event_id, kind, state, centroid, aabb_min, aabb_max, None if all(nans) else from_centroid,
                voxel_count, volume, None if math.isnan(evidence) else evidence, None if math.isnan(alpha) else alpha,
                sensors, first_seen, last_seen, mask,
            )
            event.validate()
            return event
        return _decode(data, "ChangeEvent", body)


@dataclass(frozen=True)
class MemoryStatus:
    """Room-memory health, 1 Hz (protocol §3.3). ``None`` is "not available"."""

    header: Header
    state: MemoryState
    n_bricks: int
    n_open_events: int
    coverage_known_frac: Optional[float]
    pose_residual_m: Optional[float]
    window_s: Optional[float]
    mid360_hz: Optional[int]
    jt128_hz: Optional[int]
    dropped_windows: int
    events_seq: Optional[int]  # seq of the newest live events message; None before the first

    def __post_init__(self):
        _f32_fields(self, ("coverage_known_frac", "pose_residual_m", "window_s"))

    def validate(self):
        context = "MemoryStatus"
        self.header.validate(context)
        _enum(MemoryState, self.state, f"{context}: state")
        _require_int(self.n_bricks, 0, (1 << 32) - 1, f"{context}: n_bricks")
        _require_int(self.n_open_events, 0, (1 << 32) - 1, f"{context}: n_open_events")
        _require_fraction(self.coverage_known_frac, f"{context}: coverage_known_frac")
        if self.pose_residual_m is not None and not (math.isfinite(self.pose_residual_m) and self.pose_residual_m >= 0):
            raise Invalid(f"{context}: pose_residual_m {self.pose_residual_m} must be finite and >= 0")
        if self.window_s is not None and not (math.isfinite(self.window_s) and self.window_s > 0):
            raise Invalid(f"{context}: window_s {self.window_s} must be finite and > 0")
        for name in ("mid360_hz", "jt128_hz"):
            value = getattr(self, name)
            if value is not None:
                _require_int(value, 0, HZ_UNAVAILABLE - 1, f"{context}: {name} (65535 is the unavailable sentinel; use None)")
        _require_int(self.dropped_windows, 0, (1 << 32) - 1, f"{context}: dropped_windows")
        if self.events_seq is not None:
            _require_int(self.events_seq, 0, EVENTS_SEQ_NONE - 1,
                         f"{context}: events_seq ({EVENTS_SEQ_NONE} is EVENTS_SEQ_NONE; use None)")

    def to_cdr(self):
        """CDR little endian, encapsulation header included. Raises ``Invalid``."""
        self.validate()
        writer = _Writer()
        writer.header(self.header)
        writer.u8(self.state)
        writer.u32(self.n_bricks)
        writer.u32(self.n_open_events)
        writer.f32(self.coverage_known_frac)
        writer.f32(self.pose_residual_m)
        writer.f32(self.window_s)
        writer.u16(HZ_UNAVAILABLE if self.mid360_hz is None else self.mid360_hz)
        writer.u16(HZ_UNAVAILABLE if self.jt128_hz is None else self.jt128_hz)
        writer.u32(self.dropped_windows)
        writer.u64(EVENTS_SEQ_NONE if self.events_seq is None else self.events_seq)
        return bytes(writer.data)

    @classmethod
    def from_cdr(cls, data):
        """Decodes either byte order. Raises ``Truncated`` or ``Invalid``."""
        def body(reader):
            header = reader.header("")
            state = _enum(MemoryState, reader.u8("state"), "MemoryStatus: state")
            n_bricks = reader.u32("n_bricks")
            n_open = reader.u32("n_open_events")
            optional = [reader.f32(name) for name in ("coverage_known_frac", "pose_residual_m", "window_s")]
            hz = [reader.u16(f"per_sensor_hz[{index}]") for index in range(2)]
            dropped = reader.u32("dropped_windows")
            events_seq = reader.u64("events_seq")
            status = cls(
                header, state, n_bricks, n_open, *(None if math.isnan(value) else value for value in optional),
                *(None if value == HZ_UNAVAILABLE else value for value in hz), dropped,
                None if events_seq == EVENTS_SEQ_NONE else events_seq,
            )
            status.validate()
            return status
        return _decode(data, "MemoryStatus", body)


@dataclass(frozen=True)
class Epoch:
    """Timeline tick, 0.2 Hz (protocol §3.4). Counts are cumulative since the room was created."""

    header: Header
    n_appeared: int
    n_disappeared: int
    n_moved: int
    coverage_known_frac: Optional[float]
    state: MemoryState

    def __post_init__(self):
        _f32_fields(self, ("coverage_known_frac",))

    def validate(self):
        self.header.validate("Epoch")
        for name in ("n_appeared", "n_disappeared", "n_moved"):
            _require_int(getattr(self, name), 0, (1 << 32) - 1, f"Epoch: {name}")
        _require_fraction(self.coverage_known_frac, "Epoch: coverage_known_frac")
        _enum(MemoryState, self.state, "Epoch: state")

    def to_cdr(self):
        """CDR little endian, encapsulation header included. Raises ``Invalid``."""
        self.validate()
        writer = _Writer()
        writer.header(self.header)
        writer.u32(self.n_appeared)
        writer.u32(self.n_disappeared)
        writer.u32(self.n_moved)
        writer.f32(self.coverage_known_frac)
        writer.u8(self.state)
        return bytes(writer.data)

    @classmethod
    def from_cdr(cls, data):
        """Decodes either byte order. Raises ``Truncated`` or ``Invalid``."""
        def body(reader):
            header = reader.header("")
            counts = [reader.u32(name) for name in ("n_appeared", "n_disappeared", "n_moved")]
            coverage = reader.f32("coverage_known_frac")
            state = _enum(MemoryState, reader.u8("state"), "Epoch: state")
            epoch = cls(header, *counts, None if math.isnan(coverage) else coverage, state)
            epoch.validate()
            return epoch
        return _decode(data, "Epoch", body)


@dataclass(frozen=True)
class Activity:
    """Transient motion next to the rig, 1 Hz (protocol §3.7). Never a change: activity does not
    become a ``ChangeEvent``. ``active`` when a tracking window since the previous message had
    activity; the counts, sensor mask and largest region describe the newest such window, the
    largest region in the frame of ``largest_sensor``. ``since_s`` counts from that newest
    activity window, ``None`` (NaN) before the first."""

    header: Header
    active: bool
    sensors: int
    largest_sensor: int
    n_regions: int
    voxel_count: int
    centroid: Optional[Tuple[float, float, float]]
    aabb_min: Optional[Tuple[float, float, float]]
    aabb_max: Optional[Tuple[float, float, float]]
    since_s: Optional[float]

    def __post_init__(self):
        _f32_fields(self, ("centroid", "aabb_min", "aabb_max", "since_s"))

    def validate(self):
        context = "Activity"
        self.header.validate(context)
        if not isinstance(self.active, bool):
            raise Invalid(f"{context}: active {self.active!r} is not a bool")
        _require_int(self.sensors, 0, 0xFF, f"{context}: sensors")
        if self.sensors & ~SENSORS_ALL:
            raise Invalid(f"{context}: sensors 0x{self.sensors:x} sets an undefined bit")
        _require_int(self.largest_sensor, 0, 0xFF, f"{context}: largest_sensor")
        _require_int(self.n_regions, 0, (1 << 32) - 1, f"{context}: n_regions")
        _require_int(self.voxel_count, 0, (1 << 32) - 1, f"{context}: voxel_count")
        geometry = (self.centroid, self.aabb_min, self.aabb_max)
        if self.active:
            if self.n_regions < 1 or self.voxel_count < 1:
                raise Invalid(f"{context}: active needs at least one region and voxel")
            if self.largest_sensor not in (SENSOR_MID360, SENSOR_JT128) or not self.sensors & self.largest_sensor:
                raise Invalid(f"{context}: largest_sensor {self.largest_sensor} is not one bit of sensors {self.sensors}")
            if any(value is None for value in geometry):
                raise Invalid(f"{context}: active needs the largest region's centroid and aabb")
            for value, what in zip(geometry, ("centroid", "aabb min", "aabb max")):
                _require_finite(value, f"{context}: {what}")
            if any(low > high for low, high in zip(self.aabb_min, self.aabb_max)):
                raise Invalid(f"{context}: aabb min {self.aabb_min} exceeds max {self.aabb_max}")
            if self.since_s is None:
                raise Invalid(f"{context}: active needs since_s")
        else:
            if self.n_regions or self.voxel_count or self.sensors or self.largest_sensor:
                raise Invalid(f"{context}: inactive must have zero counts and sensor bits")
            if any(value is not None for value in geometry):
                raise Invalid(f"{context}: inactive must have NaN geometry")
        if self.since_s is not None and not (math.isfinite(self.since_s) and self.since_s >= 0):
            raise Invalid(f"{context}: since_s {self.since_s} must be finite and >= 0, or NaN")

    def to_cdr(self):
        """CDR little endian, encapsulation header included. Raises ``Invalid``."""
        self.validate()
        writer = _Writer()
        writer.header(self.header)
        writer.u8(1 if self.active else 0)
        writer.u8(self.sensors)
        writer.u8(self.largest_sensor)
        writer.u32(self.n_regions)
        writer.u32(self.voxel_count)
        writer.floats(self.centroid or (None, None, None))
        writer.floats(self.aabb_min or (None, None, None))
        writer.floats(self.aabb_max or (None, None, None))
        writer.f32(self.since_s)
        return bytes(writer.data)

    @classmethod
    def from_cdr(cls, data):
        """Decodes either byte order. Raises ``Truncated`` or ``Invalid``."""
        def body(reader):
            header = reader.header("")
            active = reader.u8("active")
            if active > 1:
                raise Invalid(f"Activity: active {active} is not a bool")
            sensors = reader.u8("sensors")
            largest_sensor = reader.u8("largest_sensor")
            n_regions = reader.u32("n_regions")
            voxel_count = reader.u32("voxel_count")
            geometry = []
            for what, count in (("centroid", 3), ("aabb", 3), ("aabb", 3)):
                values = reader.floats(what, count)
                nans = [math.isnan(value) for value in values]
                if any(nans) and not all(nans):
                    raise Invalid(f"Activity: {what} {values} is partly NaN")
                geometry.append(None if all(nans) else values)
            since = reader.f32("since_s")
            activity = cls(header, bool(active), sensors, largest_sensor, n_regions, voxel_count, *geometry,
                           None if math.isnan(since) else since)
            activity.validate()
            return activity
        return _decode(data, "Activity", body)


@dataclass(frozen=True)
class RequestSync:
    """Mac → Spark resync request (protocol §3.5). ``since`` is ``(epoch, seq)`` or ``None``."""

    header: Header
    since: Optional[Tuple[int, int]]
    topic: SyncTopic

    def validate(self):
        context = "RequestSync"
        self.header.validate(context)
        _enum(SyncTopic, self.topic, f"{context}: topic")
        if self.since is not None:
            for value in self.since:
                _require_int(value, 0, SINCE_NONE - 1, f"{context}: since ({SINCE_NONE} is SINCE_NONE; use None)")
        since_epoch = SINCE_NONE if self.since is None else self.since[0]
        if self.header.map_epoch != since_epoch:
            raise Invalid(f"{context}: map_epoch {self.header.map_epoch} must repeat since_epoch {since_epoch}")

    def to_cdr(self):
        """CDR little endian, encapsulation header included. Raises ``Invalid``."""
        self.validate()
        writer = _Writer()
        writer.header(self.header)
        writer.u64(SINCE_NONE if self.since is None else self.since[0])
        writer.u64(SINCE_NONE if self.since is None else self.since[1])
        writer.u8(self.topic)
        return bytes(writer.data)

    @classmethod
    def from_cdr(cls, data):
        """Decodes either byte order. Raises ``Truncated`` or ``Invalid``."""
        def body(reader):
            header = reader.header("")
            since_epoch = reader.u64("since_epoch")
            since_seq = reader.u64("since_seq")
            topic = _enum(SyncTopic, reader.u8("topic"), "RequestSync: topic")
            if (since_epoch == SINCE_NONE) != (since_seq == SINCE_NONE):
                raise Invalid("RequestSync: since_epoch and since_seq must both be SINCE_NONE or neither")
            since = None if since_epoch == SINCE_NONE else (since_epoch, since_seq)
            request = cls(header, since, topic)
            request.validate()
            return request
        return _decode(data, "RequestSync", body)

_RHYTHM_INT16 = (-(1 << 15), (1 << 15) - 1)
_RHYTHM_REGIONS = 65535


@dataclass(frozen=True)
class RhythmMinute:
    """One minute of per-lidar region occupancy on ``/room_memory/rhythm``.

    ``sensor`` is the file-header code (1 Mid-360, 2 JT128) and is the first field after
    ``room_id``. Region arrays are strictly increasing in signed ``(ix, iy, iz)``. A loss
    marker carries ``sensor`` too, with zero counts and flag bits 1 and 4.
    """

    room_id: str
    sensor: int
    map_epoch: int
    seq: int
    stamp: Time
    windows_closed: int
    windows_with_points: int
    windows_unavailable: int
    clipped: int
    regions_dropped: int
    pose_ok: int
    flags: int
    ix: Tuple[int, ...]
    iy: Tuple[int, ...]
    iz: Tuple[int, ...]
    hit_windows: Tuple[int, ...]
    mean_voxels: Tuple[int, ...]

    def validate(self):
        context = "RhythmMinute"
        if not self.room_id:
            raise Invalid(f"{context}: room_id is empty")
        if "\x00" in self.room_id:
            raise Invalid(f"{context}: room_id contains a NUL")
        if self.sensor not in (SENSOR_MID360, SENSOR_JT128):
            raise Invalid(f"{context}: sensor {self.sensor} is not 1 (Mid-360) or 2 (JT128)")
        _require_int(self.map_epoch, 0, SINCE_NONE, f"{context}: map_epoch")
        _require_int(self.seq, 0, SINCE_NONE, f"{context}: seq")
        self.stamp.validate(f"{context}: stamp")
        for name in ("windows_closed", "windows_with_points", "windows_unavailable"):
            _require_int(getattr(self, name), 0, 0xFFFF, f"{context}: {name}")
        _require_int(self.clipped, 0, (1 << 32) - 1, f"{context}: clipped")
        _require_int(self.regions_dropped, 0, (1 << 32) - 1, f"{context}: regions_dropped")
        if self.pose_ok not in (0, 1):
            raise Invalid(f"{context}: pose_ok {self.pose_ok} is not 0 or 1")
        _require_int(self.flags, 0, 255, f"{context}: flags")
        arrays = (self.ix, self.iy, self.iz, self.hit_windows, self.mean_voxels)
        if any(len(values) != len(self.ix) for values in arrays):
            raise Invalid(f"{context}: region array lengths disagree")
        if len(self.ix) > _RHYTHM_REGIONS:
            raise Invalid(f"{context}: n_regions {len(self.ix)} does not fit in uint16")
        if self.windows_with_points == 0 and len(self.ix) != 0:
            raise Invalid(f"{context}: a minute with no point-windows stores no regions")
        previous = None
        for index, region in enumerate(zip(self.ix, self.iy, self.iz, self.hit_windows, self.mean_voxels)):
            ix, iy, iz, hits, mean = region
            for name, value in (("ix", ix), ("iy", iy), ("iz", iz)):
                _require_int(value, _RHYTHM_INT16[0], _RHYTHM_INT16[1], f"{context}: {name}[{index}]")
            _require_int(hits, 1, 0xFFFF, f"{context}: hit_windows[{index}]")
            _require_int(mean, 0, 0xFFFF, f"{context}: mean_voxels[{index}]")
            if self.windows_with_points > 0 and hits > self.windows_with_points:
                raise Invalid(f"{context}: hit_windows[{index}] {hits} exceeds windows_with_points")
            current = (ix, iy, iz)
            if previous is not None and current <= previous:
                raise Invalid(f"{context}: regions are not strictly increasing at {index}")
            previous = current

    def to_cdr(self):
        """CDR little endian, encapsulation header included. Raises ``Invalid``."""
        self.validate()
        writer = _Writer()
        writer.string(self.room_id)
        writer.u8(self.sensor)
        writer.u64(self.map_epoch)
        writer.u64(self.seq)
        writer.time(self.stamp)
        writer.u16(self.windows_closed)
        writer.u16(self.windows_with_points)
        writer.u16(self.windows_unavailable)
        writer.u32(self.clipped)
        writer.u32(self.regions_dropped)
        writer.u8(self.pose_ok)
        writer.u8(self.flags)
        writer.u16(len(self.ix))
        for name, values, put in (
            ("ix", self.ix, writer.i16), ("iy", self.iy, writer.i16), ("iz", self.iz, writer.i16),
            ("hit_windows", self.hit_windows, writer.u16), ("mean_voxels", self.mean_voxels, writer.u16),
        ):
            writer.u32(len(values))
            for value in values:
                put(value)
        return bytes(writer.data)

    @classmethod
    def from_cdr(cls, data):
        """Decodes either byte order. Raises ``Truncated`` or ``Invalid``."""
        def body(reader):
            room_id = reader.string("room_id")
            sensor = reader.u8("sensor")
            map_epoch = reader.u64("map_epoch")
            seq = reader.u64("seq")
            stamp = reader.time("stamp")
            windows_closed = reader.u16("windows_closed")
            windows_with_points = reader.u16("windows_with_points")
            windows_unavailable = reader.u16("windows_unavailable")
            clipped = reader.u32("clipped")
            regions_dropped = reader.u32("regions_dropped")
            pose_ok = reader.u8("pose_ok")
            flags = reader.u8("flags")
            declared = reader.u16("n_regions")
            if declared > _RHYTHM_REGIONS:
                raise Invalid(f"RhythmMinute: n_regions {declared} does not fit in uint16")

            def series(name, read):
                count = reader.u32(name + " length")
                if count > _RHYTHM_REGIONS:
                    raise Invalid(f"RhythmMinute: {name} length {count} is above {_RHYTHM_REGIONS}")
                return tuple(read(f"{name}[{index}]") for index in range(count))

            ix = series("ix", reader.i16)
            iy = series("iy", reader.i16)
            iz = series("iz", reader.i16)
            hits = series("hit_windows", reader.u16)
            means = series("mean_voxels", reader.u16)
            if declared != len(ix):
                raise Invalid(f"RhythmMinute: n_regions {declared} does not match ix length {len(ix)}")
            message = cls(
                room_id, sensor, map_epoch, seq, stamp, windows_closed, windows_with_points,
                windows_unavailable, clipped, regions_dropped, pose_ok, flags, ix, iy, iz, hits, means,
            )
            message.validate()
            return message
        return _decode(data, "RhythmMinute", body)
