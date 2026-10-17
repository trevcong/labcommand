"""Per-lidar occupancy rhythm: fixed grid, minute records, one writer thread.

The accumulator only counts occupied voxel keys ``close_window`` already built. It does not
cast rays, diff, or pack bricks. Disk and the optional ``/room_memory/rhythm`` publish run
on one writer thread. The window callback copies a finished record into a fixed queue and
returns. ``pose_ok`` is 1 only when the caller passes a tracking residual at or below
``learn_max_offset_m``. The ``.rhy`` record does not carry ``publisher_generation``; the
``RhythmMinute`` message does, when this writer was given one.
"""

import datetime
import errno
import logging
import os
import pathlib
import re
import struct
import threading
import time
import zlib
from dataclasses import dataclass

import numpy as np

from .core.change_detection import splitmix64, unpack
from .messages import SENSOR_JT128, SENSOR_MID360

log = logging.getLogger("room_memory.rhythm")

REGION_SLOTS = 4096
QUEUE_SLOTS = 8
REGION_BYTES = 10
ENVELOPE_BYTES = 40
CRC_BYTES = 4
MAX_RECORD_BYTES = ENVELOPE_BYTES + REGION_SLOTS * REGION_BYTES + CRC_BYTES  # 41004
RECOVERY_BYTES = 32 * 1024 * 1024
SHUTDOWN_JOIN_S = 5.0
GRACE_S = 2
INT16_MIN = -32768
INT16_MAX = 32767
U16_MAX = 65535
U32_MAX = 0xFFFFFFFF

FLAG_PARTIAL = 1 << 0
FLAG_DISCONTINUITY = 1 << 1
FLAG_UNAVAILABLE = 1 << 2
FLAG_OVERFLOW = 1 << 3
FLAG_LOST = 1 << 4

SENSOR_CODE = {"mid360": SENSOR_MID360, "jt128": SENSOR_JT128}
SENSOR_NAME = {SENSOR_MID360: "mid360", SENSOR_JT128: "jt128"}
_RECORD = struct.Struct("<IqQHHHHIIBBH")
_REGION = struct.Struct("<hhhHH")
# Same little-endian row as ``_REGION``. The minute seal writes this in one block.
_REGION_DTYPE = np.dtype([("ix", "<i2"), ("iy", "<i2"), ("iz", "<i2"), ("hits", "<u2"), ("mean", "<u2")])
_NAME = re.compile(r"^rhythm-(mid360|jt128)-(\d{8})(?:-(\d+))?\.rhy$")
_QUARANTINE = re.compile(r"^rhythm-(mid360|jt128)-(\d{8})(?:-(\d+))?\.rhy\.(superseded|bad)-(\d+)$")
_ROOM_ID = re.compile(r"[A-Za-z0-9][A-Za-z0-9._-]*")


def _is_int(value):
    return isinstance(value, int) and not isinstance(value, bool)


def _sat_u32(total, extra):
    return min(U32_MAX, total + extra)


def voxels_per_cell(cell_m, voxel_size):
    """Binary32 ``cell_m / voxel_size``. An integer in 1…64, or ``ValueError``.

    The division stays in float32. Promoting the operands to float64 is a different
    operation and is not used here.
    """
    cell = np.float32(cell_m)
    voxel = np.float32(voxel_size)
    if not np.isfinite(cell) or not np.isfinite(voxel) or not (cell > np.float32(0)) or not (voxel > np.float32(0)):
        raise ValueError("rhythm_cell_m and voxel_size must be finite and > 0")
    with np.errstate(divide="ignore", invalid="ignore", over="ignore"):
        ratio = np.float32(cell / voxel)
    if ratio.dtype != np.dtype(np.float32) or not np.isfinite(ratio):
        raise ValueError("rhythm_cell_m / voxel_size is not a finite float32")
    nearest = np.float32(np.rint(ratio))
    if ratio != nearest or not 1 <= int(nearest) <= 64:
        raise ValueError(f"rhythm_cell_m / voxel_size must be an integer in 1...64 as float32, got {float(ratio)}")
    return int(nearest)


def windows_per_period(period_s, window_s):
    """``period_s / window_s`` as an integer in 1…3600, or ``ValueError``."""
    if not _is_int(period_s) or not 1 <= period_s <= 65535:
        raise ValueError("rhythm_period_s must be an integer in 1...65535")
    if not (isinstance(window_s, (int, float)) and not isinstance(window_s, bool)
            and np.isfinite(window_s) and window_s > 0):
        raise ValueError("window_s must be a finite number > 0 s")
    quotient = period_s / float(window_s)
    windows = int(round(quotient))
    if not 1 <= windows <= 3600 or abs(quotient - windows) > 1e-9:
        raise ValueError("rhythm_period_s / window_s must be an integer in 1...3600")
    return windows


def validate_rhythm(config):
    """Reject a bad rhythm parameter. No clamping. Safe to call before any subscription.

    ``rhythm_cell_m / voxel_size`` and ``rhythm_period_s / window_s`` are checked only when
    ``rhythm_enable`` is true. A disabled logger does not refuse a detection grid. The rhythm
    period and cell still have to be in range on their own.
    """
    if not isinstance(config.rhythm_enable, bool):
        raise ValueError("rhythm_enable must be a bool")
    if not isinstance(config.rhythm_publish, bool):
        raise ValueError("rhythm_publish must be a bool")
    if config.rhythm_dims not in (2, 3):
        raise ValueError("rhythm_dims must be 2 or 3")
    if config.rhythm_enable:
        voxels_per_cell(config.rhythm_cell_m, config.voxel_size)
        windows_per_period(config.rhythm_period_s, config.window_s)
    else:
        if not _is_int(config.rhythm_period_s) or not 1 <= config.rhythm_period_s <= 65535:
            raise ValueError("rhythm_period_s must be an integer in 1...65535")
        cell = np.float32(config.rhythm_cell_m)
        if not np.isfinite(cell) or not (cell > np.float32(0)):
            raise ValueError("rhythm_cell_m must be finite and > 0")
    if not isinstance(config.rhythm_dir, str):
        raise ValueError("rhythm_dir must be a string")
    if not _is_int(config.rhythm_keep_days) or config.rhythm_keep_days < 0:
        raise ValueError("rhythm_keep_days must be an integer >= 0 (0: no age cap)")
    if not _is_int(config.rhythm_max_bytes) or config.rhythm_max_bytes < 0:
        raise ValueError("rhythm_max_bytes must be an integer >= 0 (0: no byte cap)")
    file_cap = config.rhythm_max_file_bytes
    if not _is_int(file_cap) or file_cap < 0:
        raise ValueError("rhythm_max_file_bytes must be an integer >= 0")
    if file_cap != 0 and file_cap < 65536:
        raise ValueError("rhythm_max_file_bytes must be 0 or at least 65536")
    if config.rhythm_max_bytes > 0 and file_cap > config.rhythm_max_bytes:
        raise ValueError("rhythm_max_file_bytes must be <= rhythm_max_bytes")
    if not _is_int(config.rhythm_fsync_s) or config.rhythm_fsync_s < 0:
        raise ValueError("rhythm_fsync_s must be an integer >= 0 (0: no periodic fsync)")
    if (config.rhythm_dir or config.state_dir) and not _ROOM_ID.fullmatch(config.room_id):
        raise ValueError(f"room_id '{config.room_id}' is not a plain file name ([A-Za-z0-9._-], no leading dot)")


def rhythm_directory(config):
    """``{rhythm_dir or state_dir}/{room_id}/rhythm``, with ``~`` expanded once. ``None`` if unset."""
    base = config.rhythm_dir or config.state_dir
    if not base:
        return None
    if not _ROOM_ID.fullmatch(config.room_id):
        raise ValueError(f"room_id '{config.room_id}' is not a plain file name ([A-Za-z0-9._-], no leading dot)")
    return pathlib.Path(base).expanduser() / config.room_id / "rhythm"


def rhythm_active(config):
    """False when rhythm is off, or when there is nowhere to put a bin."""
    if not config.rhythm_enable:
        return False
    return rhythm_directory(config) is not None or config.rhythm_publish


def history_depth(sensor_count):
    """Keep-last depth of ``/room_memory/rhythm``: one stored minute per configured sensor.

    Depth 1 keeps only the newest message. Both lidars publish on this one topic, so a
    subscriber that joins late would otherwise see a single sensor's minute.
    """
    if isinstance(sensor_count, bool) or not isinstance(sensor_count, int) or sensor_count < 1:
        raise ValueError("rhythm history depth needs a sensor count >= 1")
    return sensor_count


def _f32_bits(value):
    return struct.pack("<f", np.float32(value))


def _crc(data):
    return zlib.crc32(data) & 0xFFFFFFFF


# MARK: bytes


def encode_header(sensor, dims, voxels, voxel_size, cell_m, period_s, room_id):
    """File header, little endian, zlib CRC of the bytes before the CRC."""
    room = room_id.encode("utf-8")
    if len(room) > U16_MAX:
        raise ValueError("room_id is longer than uint16")
    prefix = 26 + len(room)
    padded = (prefix + 3) & ~3
    header_bytes = padded + 4
    buf = bytearray(header_bytes)
    buf[0:4] = b"RHY1"
    struct.pack_into("<H", buf, 4, 1)
    struct.pack_into("<H", buf, 6, header_bytes)
    buf[8] = sensor
    buf[9] = dims
    struct.pack_into("<H", buf, 10, voxels)
    buf[12:16] = _f32_bits(voxel_size)
    buf[16:20] = _f32_bits(cell_m)
    struct.pack_into("<I", buf, 20, period_s)
    struct.pack_into("<H", buf, 24, len(room))
    buf[26:26 + len(room)] = room
    struct.pack_into("<I", buf, padded, _crc(buf[:padded]))
    return bytes(buf)


def header_identity(data):
    """Parsed identity, or ``None`` when the magic or header CRC is bad."""
    if len(data) < 28 or data[:4] != b"RHY1":
        return None
    version, header_bytes = struct.unpack_from("<HH", data, 4)
    if version != 1 or header_bytes < 28 or header_bytes % 4 or header_bytes > len(data):
        return None
    if _crc(data[:header_bytes - 4]) != struct.unpack_from("<I", data, header_bytes - 4)[0]:
        return None
    id_len = struct.unpack_from("<H", data, 24)[0]
    if 26 + id_len > header_bytes - 4:
        return None
    return {
        "header_bytes": header_bytes,
        "sensor": data[8],
        "dims": data[9],
        "voxels_per_cell": struct.unpack_from("<H", data, 10)[0],
        "voxel_bits": bytes(data[12:16]),
        "cell_bits": bytes(data[16:20]),
        "period_s": struct.unpack_from("<I", data, 20)[0],
        "room_id": bytes(data[26:26 + id_len]).decode("utf-8", "replace"),
    }


def identity_matches(parsed, expected):
    return (
        parsed["room_id"] == expected["room_id"]
        and parsed["sensor"] == expected["sensor"]
        and parsed["dims"] == expected["dims"]
        and parsed["voxels_per_cell"] == expected["voxels_per_cell"]
        and parsed["voxel_bits"] == expected["voxel_bits"]
        and parsed["cell_bits"] == expected["cell_bits"]
        and parsed["period_s"] == expected["period_s"]
    )


def encode_record(minute, map_epoch, closed, with_points, unavailable, clipped, dropped, pose_ok, flags,
                  regions=(), packed_regions=None):
    """One record. ``regions`` is ``(ix, iy, iz, hits, mean)`` in strictly increasing order.

    ``packed_regions`` is that same payload already packed little-endian, one row per
    ``_REGION``. The minute seal uses it so a full table does not build a Python tuple
    per region on the callback.
    """
    if packed_regions is None:
        count = len(regions)
        if count > U16_MAX:
            raise ValueError("n_regions does not fit in uint16")
        body = 40 + count * REGION_BYTES
        buf = bytearray(4 + body)
        _RECORD.pack_into(
            buf, 0, body, int(minute), int(map_epoch), int(closed), int(with_points), int(unavailable), 0,
            int(clipped), int(dropped), int(pose_ok), int(flags), count,
        )
        offset = ENVELOPE_BYTES
        previous = None
        for region in regions:
            ix, iy, iz, hits, mean = region
            current = (int(ix), int(iy), int(iz))
            if previous is not None and current <= previous:
                raise ValueError("regions must be strictly increasing")
            previous = current
            _REGION.pack_into(buf, offset, current[0], current[1], current[2], int(hits), int(mean))
            offset += REGION_BYTES
        struct.pack_into("<I", buf, offset, _crc(buf[4:offset]))
        return bytes(buf)
    if len(packed_regions) % REGION_BYTES:
        raise ValueError("packed regions are not a whole number of region rows")
    count = len(packed_regions) // REGION_BYTES
    if count > U16_MAX:
        raise ValueError("n_regions does not fit in uint16")
    body = 40 + count * REGION_BYTES
    buf = bytearray(4 + body)
    _RECORD.pack_into(
        buf, 0, body, int(minute), int(map_epoch), int(closed), int(with_points), int(unavailable), 0,
        int(clipped), int(dropped), int(pose_ok), int(flags), count,
    )
    if count:
        buf[ENVELOPE_BYTES:ENVELOPE_BYTES + len(packed_regions)] = packed_regions
    end = ENVELOPE_BYTES + len(packed_regions)
    struct.pack_into("<I", buf, end, _crc(buf[4:end]))
    return bytes(buf)


def encode_marker(minute, map_epoch):
    """Loss marker: same minute and epoch, zero counts, bits 1 and 4. No partial bit."""
    return encode_record(minute, map_epoch, 0, 0, 0, 0, 0, 0, FLAG_DISCONTINUITY | FLAG_LOST, ())


def parse_record(data):
    """One record, or ``None`` when the length or CRC does not match."""
    if len(data) < 4 + 40:
        return None
    body = struct.unpack_from("<I", data, 0)[0]
    if body < 40 or 4 + body != len(data) or (body - 40) % REGION_BYTES:
        return None
    if _crc(data[4:-4]) != struct.unpack_from("<I", data, len(data) - 4)[0]:
        return None
    fields = _RECORD.unpack_from(data, 0)
    # ``<IqQHHHHIIBBH`` is 12 fields. ``n_regions`` is the last one.
    count = fields[11]
    if fields[6] != 0 or 40 + count * REGION_BYTES != body:
        return None
    regions = []
    offset = ENVELOPE_BYTES
    previous = None
    for _ in range(count):
        region = _REGION.unpack_from(data, offset)
        current = region[:3]
        if region[3] == 0 or (previous is not None and current <= previous):
            return None
        previous = current
        regions.append(region)
        offset += REGION_BYTES
    return {
        "minute": fields[1],
        "map_epoch": fields[2],
        "closed": fields[3],
        "with_points": fields[4],
        "unavailable": fields[5],
        "pad": fields[6],
        "clipped": fields[7],
        "regions_dropped": fields[8],
        "pose_ok": fields[9],
        "flags": fields[10],
        "regions": tuple(regions),
    }


def scan_records(data):
    """Byte offset of the end of the last good record, and that record's minute (or ``None``)."""
    parsed = header_identity(data)
    if parsed is None:
        return None
    offset = parsed["header_bytes"]
    last = None
    while offset + 4 <= len(data):
        body = struct.unpack_from("<I", data, offset)[0]
        total = 4 + body
        if body < 40 or offset + total > len(data):
            break
        record = parse_record(data[offset:offset + total])
        if record is None:
            break
        last = record["minute"]
        offset += total
    return offset, last


def day_string(minute_unix):
    return time.strftime("%Y%m%d", time.gmtime(int(minute_unix)))


def segment_name(sensor_name, minute_unix, index=1):
    day = day_string(minute_unix)
    if index <= 1:
        return f"rhythm-{sensor_name}-{day}.rhy"
    return f"rhythm-{sensor_name}-{day}-{index}.rhy"


# MARK: table


class _Table:
    """4096 open-addressed slots. Allocated once. ``clear`` does not allocate."""

    def __init__(self, slots=REGION_SLOTS):
        if slots < 1 or slots & (slots - 1):
            raise ValueError("rhythm table length must be a power of two")
        self.slots = slots
        self.mask = slots - 1
        self.used = np.zeros(slots, dtype=np.uint8)
        self.ix = np.zeros(slots, dtype=np.int16)
        self.iy = np.zeros(slots, dtype=np.int16)
        self.iz = np.zeros(slots, dtype=np.int16)
        self.hits = np.zeros(slots, dtype=np.uint32)
        self.sums = np.zeros(slots, dtype=np.uint64)
        self.count = 0
        # Slot cache for a repeated cell list. Probing from Python is the callback cost;
        # a steady room presents the same cells every window. Cleared with the table.
        self._cached = 0
        self._cix = np.empty(slots, dtype=np.int16)
        self._ciy = np.empty(slots, dtype=np.int16)
        self._ciz = np.empty(slots, dtype=np.int16)
        self._cslot = np.empty(slots, dtype=np.int32)

    def clear(self):
        """Drop occupancy and counts. A complete layout cache stays.

        A minute seal calls this. The next window of the same room writes those slots
        back and does not probe. The cache is dropped when it is not the whole table:
        restoring a subset would leave a key past an empty slot, and the next probe
        would miss it.
        """
        if self._cached != self.count:
            self._cached = 0
        self.used.fill(0)
        self.hits.fill(0)
        self.sums.fill(0)
        self.count = 0

    def insert_window(self, ix, iy, iz, counts):
        """Insert unique cells in signed order. The first miss returns how many were not stored.

        A miss stops the window: later regions, including ones already in the table, are
        counted and not updated. The cells that fit stay cached in insertion order. The
        next window reuses those slots when it is that list, or a longer list with that
        prefix once the table is exactly those cells. A longer tail is then a miss
        without a probe: the cells are unique, so none of it is already stored, and a
        miss in a full table would walk every slot. ``clear`` keeps this cache when it
        is the whole table. The update writes each slot from one row, and a repeated
        index does not accumulate, so the cell list has to be unique. ``_cells`` has
        already summed each cell into one row.
        """
        total = len(ix)
        if total == 0:
            return 0
        begin = 0
        cached = self._cached
        if cached and total >= cached and self._prefix_matches(ix, iy, iz, cached):
            if self.count == 0:
                self._apply_cached(cached, counts, fresh=True)
            elif self.count == cached:
                self._apply_cached(cached, counts, fresh=False)
            else:
                cached = 0
            if cached:
                if total == cached:
                    return 0
                # The prefix is the entire table and it is full. Every later cell misses.
                if self.count >= self.slots:
                    return total - cached
                begin = cached
        packed = ((ix.astype(np.int64) & 0xFFFF) << 32) | ((iy.astype(np.int64) & 0xFFFF) << 16) | (iz.astype(np.int64) & 0xFFFF)
        starts = np.asarray(splitmix64(packed), dtype=np.uint64) & np.uint64(self.mask)
        for n in range(begin, total):
            # Unique cells, and every occupied slot is one of this window's first n.
            # The next cell is absent. Searching for it would scan the whole table.
            if self.count >= self.slots and n == self.count:
                self._remember(ix, iy, iz, n)
                return total - n
            slot = self._slot(int(ix[n]), int(iy[n]), int(iz[n]), int(starts[n]))
            if slot is None:
                if n > 0:
                    self._remember(ix, iy, iz, n)
                return total - n
            if n < self.slots:
                self._cslot[n] = slot
            self.hits[slot] = min(U16_MAX, int(self.hits[slot]) + 1)
            self.sums[slot] = np.uint64(int(self.sums[slot]) + int(counts[n]))
        if total <= self.slots:
            self._remember(ix, iy, iz, total)
        else:
            self._cached = 0
        return 0

    def _prefix_matches(self, ix, iy, iz, cached):
        return (
            np.array_equal(ix[:cached], self._cix[:cached])
            and np.array_equal(iy[:cached], self._ciy[:cached])
            and np.array_equal(iz[:cached], self._ciz[:cached])
        )

    def _apply_cached(self, n, counts, fresh):
        slots = self._cslot[:n]
        taken = np.asarray(counts[:n], dtype=np.uint64)
        if fresh:
            self.used[slots] = 1
            self.ix[slots] = self._cix[:n]
            self.iy[slots] = self._ciy[:n]
            self.iz[slots] = self._ciz[:n]
            self.hits[slots] = np.uint32(1)
            self.sums[slots] = taken
            self.count = n
            return
        updated = self.hits[slots] + np.uint32(1)
        np.minimum(updated, np.uint32(U16_MAX), out=updated)
        self.hits[slots] = updated
        self.sums[slots] += taken

    def _remember(self, ix, iy, iz, n):
        if n <= 0 or n > self.slots:
            self._cached = 0
            return
        self._cix[:n] = ix[:n]
        self._ciy[:n] = iy[:n]
        self._ciz[:n] = iz[:n]
        self._cached = n

    def _slot(self, ix, iy, iz, start):
        for step in range(self.slots):
            slot = (start + step) & self.mask
            if not self.used[slot]:
                if self.count >= self.slots:
                    return None
                self.used[slot] = 1
                self.ix[slot] = ix
                self.iy[slot] = iy
                self.iz[slot] = iz
                self.hits[slot] = 0
                self.sums[slot] = 0
                self.count += 1
                return slot
            if int(self.ix[slot]) == ix and int(self.iy[slot]) == iy and int(self.iz[slot]) == iz:
                return slot
        return None

    def _occupied(self, with_points):
        """Occupied rows with a hit, sorted by signed ``(ix, iy, iz)``. ``None`` when there are none."""
        if with_points <= 0:
            return None
        taken = np.flatnonzero(self.used)
        if len(taken) == 0:
            return None
        ix = self.ix[taken].astype(np.int32)
        iy = self.iy[taken].astype(np.int32)
        iz = self.iz[taken].astype(np.int32)
        hits = self.hits[taken]
        sums = self.sums[taken]
        keep = hits > 0
        if not np.any(keep):
            return None
        ix, iy, iz, hits, sums = ix[keep], iy[keep], iz[keep], hits[keep], sums[keep]
        order = np.lexsort((iz, iy, ix))
        # uint64 sum. Casting through int64 would invert a sum above 2^63.
        means = (sums[order] + np.uint64(with_points // 2)) // np.uint64(with_points)
        np.minimum(means, np.uint64(U16_MAX), out=means)
        return ix[order], iy[order], iz[order], hits[order], means

    def regions(self, with_points):
        """Occupied cells with a hit, sorted by signed ``(ix, iy, iz)``. Mean saturates at 65535."""
        rows = self._occupied(with_points)
        if rows is None:
            return ()
        ix, iy, iz, hits, means = rows
        return tuple(
            (int(ix[n]), int(iy[n]), int(iz[n]), int(hits[n]), int(means[n]))
            for n in range(len(ix))
        )

    def pack_regions(self, with_points):
        """Sorted region payload, little endian. Empty when nothing was hit."""
        rows = self._occupied(with_points)
        if rows is None:
            return b""
        ix, iy, iz, hits, means = rows
        packed = np.empty(len(ix), dtype=_REGION_DTYPE)
        packed["ix"] = ix
        packed["iy"] = iy
        packed["iz"] = iz
        packed["hits"] = hits
        packed["mean"] = means
        return packed.tobytes()


class _Bin:
    def __init__(self, minute, map_epoch, discontinuity):
        self.minute = minute
        self.map_epoch = map_epoch
        self.closed = 0
        self.with_points = 0
        self.unavailable = 0
        self.clipped = 0
        self.regions_dropped = 0
        self.pose_bad = False
        self.discontinuity = bool(discontinuity)
        self.overflow = False


@dataclass(frozen=True)
class Fragment:
    """One encoded record ready for the queue."""

    sensor: str
    minute: int
    map_epoch: int
    flags: int
    data: bytes

    @property
    def lost(self):
        return bool(self.flags & FLAG_LOST)


class RhythmAccumulator:
    """Per-sensor minute bins. No files. The region tables are allocated in ``__init__``."""

    def __init__(self, sensors, voxels, dims, period_s, windows_per, slots=REGION_SLOTS):
        self.voxels = voxels
        self.dims = dims
        self.period_s = period_s
        self.windows_per = windows_per
        self._sensors = {
            name: {"bin": None, "table": _Table(slots), "loss": None, "force_disc": False}
            for name in sensors
        }
        self.finished = []
        self.hook_errors = 0

    def add(self, sensor, sec, keys, map_epoch, tracking=False, residual=None, learn_max_offset_m=None):
        """One evaluated non-empty window. ``keys`` are that window's occupied voxel keys."""
        opened = self._advance(sensor, sec, map_epoch)
        if opened is None:
            return
        pose_ok = (
            tracking and residual is not None and learn_max_offset_m is not None and residual <= learn_max_offset_m
        )
        if not pose_ok:
            opened.pose_bad = True
        opened.closed = min(U16_MAX, opened.closed + 1)
        opened.with_points = min(U16_MAX, opened.with_points + 1)
        clipped, ix, iy, iz, counts = self._cells(keys)
        opened.clipped = _sat_u32(opened.clipped, clipped)
        missed = self._sensors[sensor]["table"].insert_window(ix, iy, iz, counts)
        if missed:
            opened.overflow = True
            opened.regions_dropped = _sat_u32(opened.regions_dropped, missed)

    def add_empty(self, sensor, sec, map_epoch):
        """One evaluated window with no occupied voxel."""
        opened = self._advance(sensor, sec, map_epoch)
        if opened is None:
            return
        opened.closed = min(U16_MAX, opened.closed + 1)

    def note_unavailable(self, sensor, sec, map_epoch):
        """BUILDING skip: occupancy was not evaluated. Not an empty room."""
        opened = self._advance(sensor, sec, map_epoch)
        if opened is None:
            return
        opened.unavailable = min(U16_MAX, opened.unavailable + 1)

    def discard(self, sensor, sec, map_epoch):
        """Drop the open bin after a hook error and remember a loss marker."""
        state = self._sensors[sensor]
        minute = state["bin"].minute if state["bin"] is not None else self._minute(sec)
        epoch = state["bin"].map_epoch if state["bin"] is not None else map_epoch
        state["bin"] = None
        state["table"].clear()
        if minute is not None:
            state["loss"] = (minute, int(epoch))
        state["force_disc"] = True
        self.hook_errors += 1

    def note_epoch(self, map_epoch):
        """Open bins keep the epoch at close, including a bump after the hook returned."""
        for state in self._sensors.values():
            if state["bin"] is not None:
                state["bin"].map_epoch = map_epoch

    def take(self):
        """Finished bins since the last take. Does not flush the open minute."""
        done = self.finished
        self.finished = []
        return done

    def flush(self):
        """Seal every open bin that saw a window. Shutdown and tests."""
        for sensor in self._sensors:
            self._seal(sensor)
        return self.take()

    def _minute(self, sec):
        if sec is None or sec < 0:
            return None
        return int(sec) - int(sec) % self.period_s

    def _advance(self, sensor, sec, map_epoch):
        minute = self._minute(sec)
        if minute is None:
            return None
        state = self._sensors[sensor]
        opened = state["bin"]
        if opened is None:
            state["bin"] = _Bin(minute, map_epoch, state["force_disc"])
            state["force_disc"] = False
            return state["bin"]
        if minute == opened.minute or (
            minute + self.period_s == opened.minute and 0 < opened.minute - int(sec) <= GRACE_S
        ):
            opened.map_epoch = map_epoch
            return opened
        gap = minute > opened.minute + self.period_s or minute < opened.minute
        self._seal(sensor)
        state["bin"] = _Bin(minute, map_epoch, gap or state["force_disc"])
        state["force_disc"] = False
        return state["bin"]

    def _seal(self, sensor):
        state = self._sensors[sensor]
        opened = state["bin"]
        if opened is None:
            return
        if opened.closed + opened.unavailable == 0:
            state["bin"] = None
            state["table"].clear()
            return
        flags = 0
        if opened.closed < self.windows_per:
            flags |= FLAG_PARTIAL
        if opened.discontinuity:
            flags |= FLAG_DISCONTINUITY
        if opened.unavailable:
            flags |= FLAG_UNAVAILABLE
        if opened.overflow or opened.regions_dropped:
            flags |= FLAG_OVERFLOW
        pose_ok = 0 if opened.with_points == 0 or opened.pose_bad else 1
        # Build the record before dropping the bin. A failure leaves it for ``discard``.
        # The packed payload stays off the tuple path: a full table is 4096 regions.
        data = encode_record(
            opened.minute, opened.map_epoch, opened.closed, opened.with_points, opened.unavailable,
            opened.clipped, opened.regions_dropped, pose_ok, flags,
            packed_regions=state["table"].pack_regions(opened.with_points),
        )
        state["bin"] = None
        state["table"].clear()
        self.finished.append(Fragment(sensor, opened.minute, opened.map_epoch, flags, data))

    def _cells(self, keys):
        """Group occupied voxel keys into unique region cells.

        ``close_window`` passes a sorted unique run, and an unsorted caller is sorted
        once. Packed ``(i, j, k)`` order is not coarse-cell order. Floor division of
        ``(0, 0, 0), (0, 10, 0), (1, 0, 0)`` by 10 voxels per cell yields A, B, A, and
        an adjacent run would insert A twice in one window. Sort the coarse
        coordinates and sum each run so the window inserts every cell once, in signed
        ``(ix, iy, iz)`` order. Clipped cells leave the run. The 4096 slots are not
        grown here.
        """
        keys = np.asarray(keys, dtype=np.int64).reshape(-1)
        if len(keys) == 0:
            empty = np.zeros(0, dtype=np.int16)
            return 0, empty, empty, empty, empty
        if len(keys) > 1 and bool(np.any(keys[1:] < keys[:-1])):
            keys = np.sort(keys)
        i, j, k = unpack(keys)
        vpc = np.int64(self.voxels)
        ix = np.floor_divide(i, vpc)
        iy = np.floor_divide(j, vpc)
        iz = np.zeros_like(ix) if self.dims == 2 else np.floor_divide(k, vpc)
        ok = (
            (ix >= INT16_MIN) & (ix <= INT16_MAX)
            & (iy >= INT16_MIN) & (iy <= INT16_MAX)
            & (iz >= INT16_MIN) & (iz <= INT16_MAX)
        )
        clipped = int(np.count_nonzero(~ok))
        if not bool(np.all(ok)):
            ix, iy, iz = ix[ok], iy[ok], iz[ok]
        count = len(ix)
        if count == 0:
            empty = np.zeros(0, dtype=np.int16)
            return clipped, empty, empty, empty, empty
        if count > 1:
            order = np.lexsort((iz, iy, ix))
            ix, iy, iz = ix[order], iy[order], iz[order]
        if count == 1:
            one = np.ones(1, dtype=np.int64)
            return (
                clipped, ix.astype(np.int16, copy=False), iy.astype(np.int16, copy=False),
                iz.astype(np.int16, copy=False), one,
            )
        change = np.empty(count, dtype=bool)
        change[0] = True
        change[1:] = (ix[1:] != ix[:-1]) | (iy[1:] != iy[:-1]) | (iz[1:] != iz[:-1])
        starts = np.flatnonzero(change)
        counts = np.diff(np.append(starts, count))
        return (
            clipped, ix[starts].astype(np.int16, copy=False), iy[starts].astype(np.int16, copy=False),
            iz[starts].astype(np.int16, copy=False), counts,
        )


# MARK: writer


class _Store:
    """Filesystem calls the writer thread is allowed to block on."""

    def write(self, fd, data):
        return os.write(fd, data)

    def fsync(self, fd):
        os.fsync(fd)

    def truncate(self, fd, length):
        os.ftruncate(fd, length)


class _OpenSegment:
    def __init__(self, path, fd, day, index, minute):
        self.path = path
        self.fd = fd
        self.day = day
        self.index = index
        self.last_minute = minute
        self.abandoned = False


class RhythmWriter:
    """One thread. Append, fsync, rotate, and prune. The callback only ``try_enqueue``."""

    def __init__(self, config, directory, store=None, publish=None, publish_would_block=None, log_fn=None,
                 publisher_generation=None):
        self.config = config
        self.directory = directory
        self.publisher_generation = publisher_generation
        self.store = store or _Store()
        self.publish = publish
        self.publish_would_block = publish_would_block or (lambda: False)
        self.log_fn = log_fn or (lambda message: log.warning("%s", message))
        self.voxels = voxels_per_cell(config.rhythm_cell_m, config.voxel_size)
        self.queue_dropped = 0
        self.budget_dropped = 0
        self.io_errors = 0
        self.publish_dropped = 0
        self._logged = False
        self._seq = 0
        self._lock = threading.Lock()
        self._wake = threading.Event()
        self._stop = threading.Event()
        self.gate = threading.Event()
        self.gate.set()
        self._pub_lock = threading.Lock()
        self._slots = [bytearray(MAX_RECORD_BYTES) for _ in range(QUEUE_SLOTS)]
        self._lengths = [0] * QUEUE_SLOTS
        self._codes = [0] * QUEUE_SLOTS
        self._head = 0
        self._tail = 0
        self._count = 0
        self._busy = False
        self._open = {}
        self._resume = {}
        self._force_disc = {name: False for name in config.sensors}
        self._last_minute = {name: None for name in config.sensors}
        self._last_fsync = time.monotonic()
        self._last_age_day = None
        self._shutting_down = False
        self._thread = None
        if directory is not None:
            self._prepare_directory()

    def start(self):
        if self._thread is not None:
            return
        self._thread = threading.Thread(target=self._run, name="rhythm-writer", daemon=True)
        self._thread.start()

    def set_publish(self, publish, would_block=None):
        self.publish = publish
        if would_block is not None:
            self.publish_would_block = would_block

    def try_enqueue(self, sensor, data):
        """Copy ``data`` into a free slot. Never touches the filesystem. False drops the record."""
        if len(data) > MAX_RECORD_BYTES:
            with self._lock:
                self.queue_dropped += 1
            return False
        code = SENSOR_CODE[sensor]
        with self._lock:
            if self._count >= QUEUE_SLOTS:
                self.queue_dropped += 1
                return False
            slot = self._slots[self._tail]
            slot[:len(data)] = data
            self._lengths[self._tail] = len(data)
            self._codes[self._tail] = code
            self._tail = (self._tail + 1) % QUEUE_SLOTS
            self._count += 1
        self._wake.set()
        return True

    def note_discontinuity(self, sensor):
        self._force_disc[sensor] = True

    def close(self):
        """Flush is the caller's job. Drain, one sync, join at most 5 s. No prune."""
        self._shutting_down = True
        # Stop before opening the gate, so a paused writer cannot age-prune on the way out.
        self._stop.set()
        self.gate.set()
        self._wake.set()
        thread = self._thread
        if thread is not None and thread is not threading.current_thread():
            thread.join(SHUTDOWN_JOIN_S)
        self._thread = None

    def wait_idle(self, timeout=5.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            with self._lock:
                idle = self._count == 0 and not self._busy
            if idle:
                return True
            time.sleep(0.005)
        return False

    def _log_once(self, message):
        if self._logged:
            return
        self._logged = True
        self.log_fn(message)

    def _prepare_directory(self):
        try:
            self.directory.mkdir(parents=True, exist_ok=True)
            self._recover_open_files()
            self._age_prune()
            self._last_age_day = time.strftime("%Y%m%d", time.gmtime())
        except OSError as error:
            self.io_errors += 1
            self._log_once(f"rhythm directory {self.directory} is unusable ({error}); detection will still start")

    def _identity(self, sensor_name):
        return {
            "room_id": self.config.room_id,
            "sensor": SENSOR_CODE[sensor_name],
            "dims": self.config.rhythm_dims,
            "voxels_per_cell": self.voxels,
            "voxel_bits": _f32_bits(self.config.voxel_size),
            "cell_bits": _f32_bits(self.config.rhythm_cell_m),
            "period_s": self.config.rhythm_period_s,
        }

    def _recover_open_files(self):
        """Scan only the append file for today, at most 32 MiB, before the thread writes."""
        today = time.strftime("%Y%m%d", time.gmtime())
        for sensor_name in self.config.sensors:
            path, index = self._latest(sensor_name, today)
            if path is None:
                continue
            try:
                size = path.stat().st_size
            except OSError as error:
                self.io_errors += 1
                self._log_once(f"rhythm recovery skipped {path} ({error})")
                self._force_disc[sensor_name] = True
                continue
            if size > RECOVERY_BYTES:
                self._quarantine(path, "superseded")
                self._force_disc[sensor_name] = True
                continue
            try:
                data = path.read_bytes()
            except OSError as error:
                self.io_errors += 1
                self._log_once(f"rhythm recovery skipped {path} ({error})")
                self._force_disc[sensor_name] = True
                continue
            parsed = header_identity(data)
            if parsed is None:
                self._quarantine(path, "bad")
                self._force_disc[sensor_name] = True
                continue
            if not identity_matches(parsed, self._identity(sensor_name)):
                self._quarantine(path, "superseded")
                self._force_disc[sensor_name] = True
                continue
            scanned = scan_records(data)
            if scanned is None:
                self._quarantine(path, "bad")
                self._force_disc[sensor_name] = True
                continue
            good, last = scanned
            if good < len(data):
                try:
                    os.truncate(path, good)
                except OSError as error:
                    self.io_errors += 1
                    self._log_once(f"rhythm could not truncate {path} ({error})")
                    self._force_disc[sensor_name] = True
                    continue
                self._force_disc[sensor_name] = True
            self._last_minute[sensor_name] = last
            self._resume[sensor_name] = (path, index or 1, today)

    def _latest(self, sensor_name, day):
        best = None
        best_index = 0
        try:
            names = list(self.directory.iterdir())
        except OSError:
            return None, 1
        for path in names:
            match = _NAME.fullmatch(path.name)
            if match is None or match.group(1) != sensor_name or match.group(2) != day:
                continue
            index = int(match.group(3) or 1)
            if index >= best_index:
                best = path
                best_index = index
        return best, best_index

    def _quarantine(self, path, kind):
        stamp = int(time.time())
        target = path.with_name(f"{path.name}.{kind}-{stamp}")
        step = 0
        while target.exists():
            step += 1
            target = path.with_name(f"{path.name}.{kind}-{stamp}-{step}")
        os.replace(path, target)

    def _run(self):
        try:
            while not self._stop.is_set() or self._queued():
                self._wake.wait(self._idle_timeout())
                self._wake.clear()
                if not self.gate.is_set():
                    continue
                self._drain(prune=not self._stop.is_set())
                if self._stop.is_set():
                    continue
                self._maybe_fsync()
                self._maybe_age_prune()
            self._drain(prune=False)
            self._fsync_open()
        except Exception as error:
            self.io_errors += 1
            self._log_once(f"rhythm writer stopped ({error})")

    def _idle_timeout(self):
        """Sleep until an enqueue or shutdown. Polling here stalls the window callback.

        Periodic fsync and the daily age prune are the only reasons to wake otherwise.
        """
        interval = self.config.rhythm_fsync_s
        if interval <= 0:
            return 30.0
        remaining = interval - (time.monotonic() - self._last_fsync)
        if remaining < 0.05:
            return 0.05
        return min(30.0, remaining)

    def _queued(self):
        with self._lock:
            return self._count > 0

    def _pop(self):
        with self._lock:
            if self._count == 0:
                return None
            length = self._lengths[self._head]
            data = bytes(self._slots[self._head][:length])
            code = self._codes[self._head]
            self._head = (self._head + 1) % QUEUE_SLOTS
            self._count -= 1
            # Set before releasing the lock so a waiter cannot observe an empty, idle queue
            # while this record is still being written.
            self._busy = True
            return code, data

    def _drain(self, prune):
        while True:
            item = self._pop()
            if item is None:
                return
            code, data = item
            try:
                self._append(SENSOR_NAME[code], data, prune=prune)
            finally:
                with self._lock:
                    self._busy = False

    def _append(self, sensor_name, data, prune):
        record = parse_record(data)
        if record is None:
            self.io_errors += 1
            return
        if self.directory is None:
            self._publish(sensor_name, record, data)
            return
        segment = self._open.get(sensor_name)
        if segment is not None and segment.abandoned:
            return
        last = self._last_minute.get(sensor_name)
        if last is not None and record["minute"] == last:
            self._force_disc[sensor_name] = True
        if self._force_disc.get(sensor_name) and not (record["flags"] & FLAG_DISCONTINUITY):
            record["flags"] |= FLAG_DISCONTINUITY
            data = encode_record(
                record["minute"], record["map_epoch"], record["closed"], record["with_points"],
                record["unavailable"], record["clipped"], record["regions_dropped"], record["pose_ok"],
                record["flags"], record["regions"],
            )
            record = parse_record(data)
        if not self._ready(sensor_name, record, len(data), prune):
            return
        segment = self._open[sensor_name]
        if not self._write_record(segment, data, record):
            return
        self._force_disc[sensor_name] = False
        self._last_minute[sensor_name] = record["minute"]
        segment.last_minute = record["minute"]
        self._publish(sensor_name, record, data)

    def _ready(self, sensor_name, record, size, prune):
        day = day_string(record["minute"])
        segment = self._open.get(sensor_name)
        if segment is None:
            self._take_resume(sensor_name, day, size)
            segment = self._open.get(sensor_name)
        rotate = segment is not None and segment.day != day
        if segment is not None and not rotate and self.config.rhythm_max_file_bytes:
            try:
                used = os.lseek(segment.fd, 0, os.SEEK_END)
            except OSError:
                used = 0
            if used + size > self.config.rhythm_max_file_bytes:
                rotate = True
        if rotate and segment is not None:
            self._sync_fd(segment.fd)
            self._close_segment(sensor_name, abandon=False)
            segment = None
        header = 0 if segment is not None else self._header_size(sensor_name)
        if not self._fits(size + header, prune):
            self.budget_dropped += 1
            return False
        if segment is None:
            if not self._open_new(sensor_name, record["minute"], day):
                return False
        return True

    def _take_resume(self, sensor_name, day, size):
        """Continue today's recovered append file, unless the next record belongs in a new segment."""
        resumed = self._resume.get(sensor_name)
        if resumed is None:
            return
        path, index, resume_day = resumed
        if resume_day != day:
            self._resume.pop(sensor_name, None)
            return
        try:
            used = path.stat().st_size
        except OSError:
            used = 0
        if self.config.rhythm_max_file_bytes and used + size > self.config.rhythm_max_file_bytes:
            self._resume.pop(sensor_name, None)
            return
        self._resume.pop(sensor_name, None)
        try:
            fd = os.open(path, os.O_RDWR)
            os.lseek(fd, 0, os.SEEK_END)
        except OSError as error:
            self.io_errors += 1
            self._log_once(f"rhythm cannot resume {path} ({error})")
            self._force_disc[sensor_name] = True
            return
        self._open[sensor_name] = _OpenSegment(path, fd, day, index, self._last_minute.get(sensor_name))

    def _header_size(self, sensor_name):
        return len(encode_header(
            SENSOR_CODE[sensor_name], self.config.rhythm_dims, self.voxels, self.config.voxel_size,
            self.config.rhythm_cell_m, self.config.rhythm_period_s, self.config.room_id,
        ))

    def _open_new(self, sensor_name, minute, day):
        try:
            self.directory.mkdir(parents=True, exist_ok=True)
        except OSError as error:
            self.io_errors += 1
            self._log_once(f"rhythm cannot create {self.directory} ({error})")
            return False
        _, index = self._latest(sensor_name, day)
        next_index = index + 1 if index else 1
        path = self.directory / segment_name(sensor_name, minute, next_index)
        if path.exists():
            next_index = max(index, 1) + 1
            path = self.directory / segment_name(sensor_name, minute, next_index)
        try:
            fd = os.open(path, os.O_CREAT | os.O_EXCL | os.O_RDWR, 0o644)
        except OSError as error:
            self.io_errors += 1
            self._log_once(f"rhythm cannot open {path} ({error})")
            return False
        header = encode_header(
            SENSOR_CODE[sensor_name], self.config.rhythm_dims, self.voxels, self.config.voxel_size,
            self.config.rhythm_cell_m, self.config.rhythm_period_s, self.config.room_id,
        )
        try:
            written = self._write_all(fd, header)
            if written != len(header):
                raise OSError(errno.EIO, "short header")
            self.store.fsync(fd)
        except OSError as error:
            self.io_errors += 1
            self._log_once(f"rhythm header failed for {path} ({error})")
            os.close(fd)
            try:
                os.unlink(path)
            except OSError:
                pass
            return False
        self._open[sensor_name] = _OpenSegment(path, fd, day, next_index, None)
        return True

    def _write_record(self, segment, data, record):
        """Append ``data`` at the current end. The offset is saved before the write.

        A short write or ``ENOSPC``/``EDQUOT`` is retried once at that offset. If the retry
        fails, the file is truncated back there and the loss marker is written in its place.
        The marker carries bits 1 and 4, so the following fragment is not flagged again.
        When the marker cannot be written, the next fragment gets bit 1. A failed truncate
        abandons the segment: nothing more is appended, and no replacement file is opened.
        """
        try:
            offset = os.lseek(segment.fd, 0, os.SEEK_END)
        except OSError as error:
            self._abandon(segment, error)
            return False
        status = self._attempt(segment, data, offset)
        if status == "ok":
            return True
        if status != "retry":
            return False
        status = self._attempt(segment, data, offset)
        if status == "ok":
            return True
        if status != "retry":
            return False
        marker = encode_marker(record["minute"], record["map_epoch"])
        status = self._attempt(segment, marker, offset)
        sensor_name = SENSOR_NAME[self._code_of(segment)]
        if status == "ok":
            self._last_minute[sensor_name] = record["minute"]
            segment.last_minute = record["minute"]
            self._force_disc[sensor_name] = False
            self._publish(sensor_name, parse_record(marker), marker)
            self.io_errors += 1
            return False
        if status == "retry" and not segment.abandoned:
            # The marker did not land at the restored offset. The next fragment carries bit 1.
            self._force_disc[sensor_name] = True
            self.io_errors += 1
        return False

    def _attempt(self, segment, data, offset):
        """Write ``data`` once. ``ok``, ``retry`` (file is back at ``offset``), or ``stop``."""
        try:
            written = self._write_all(segment.fd, data)
        except OSError as error:
            if error.errno in (errno.ENOSPC, errno.EDQUOT):
                written = 0
            else:
                self._log_once(f"rhythm write failed on {segment.path} ({error})")
                if not self._restore(segment, offset):
                    return "stop"
                self.io_errors += 1
                self._force_disc[SENSOR_NAME[self._code_of(segment)]] = True
                return "stop"
        if written == len(data):
            return "ok"
        if not self._restore(segment, offset):
            return "stop"
        return "retry"

    def _code_of(self, segment):
        for name, open_segment in self._open.items():
            if open_segment is segment:
                return SENSOR_CODE[name]
        return SENSOR_MID360

    def _restore(self, segment, offset):
        try:
            self.store.truncate(segment.fd, offset)
            os.lseek(segment.fd, offset, os.SEEK_SET)
            return True
        except OSError as error:
            self._abandon(segment, error)
            return False

    def _abandon(self, segment, error):
        segment.abandoned = True
        self.io_errors += 1
        self._log_once(f"rhythm abandoned {segment.path} ({error})")
        for name, open_segment in self._open.items():
            if open_segment is segment:
                self._force_disc[name] = True
        try:
            os.close(segment.fd)
        except OSError:
            pass
        segment.fd = -1

    def _write_all(self, fd, data):
        """One store write. A short count is the caller's retry signal, not a loop."""
        return self.store.write(fd, data)

    def _fits(self, incoming, prune):
        if self.directory is None or self.config.rhythm_max_bytes == 0:
            return True
        try:
            files = self._budget_files()
        except OSError as error:
            self.io_errors += 1
            self._log_once(f"rhythm cannot list {self.directory} ({error})")
            return False
        total = sum(size for _, size, _, _ in files)
        if total + incoming <= self.config.rhythm_max_bytes:
            return True
        if not prune or self._shutting_down:
            return False
        open_paths = {segment.path for segment in self._open.values() if not segment.abandoned}
        quarantine = [item for item in files if item[2] == "quarantine" and item[0] not in open_paths]
        quarantine.sort(key=lambda item: item[3])
        closed = [item for item in files if item[2] == "segment" and item[0] not in open_paths]
        closed.sort(key=lambda item: _segment_sort_key(item[0].name))
        for group in (quarantine, closed):
            for path, size, _, _ in group:
                if total + incoming <= self.config.rhythm_max_bytes:
                    break
                try:
                    path.unlink()
                except OSError as error:
                    self._log_once(f"rhythm could not delete {path} ({error})")
                    continue
                total -= size
            if total + incoming <= self.config.rhythm_max_bytes:
                return True
        return False

    def _budget_files(self):
        found = []
        for path in self.directory.iterdir():
            if not path.is_file():
                continue
            name = path.name
            kind = None
            if _NAME.fullmatch(name):
                kind = "segment"
            elif _QUARANTINE.fullmatch(name) or ".superseded-" in name or ".bad-" in name:
                kind = "quarantine"
            if kind is None:
                continue
            try:
                stat = path.stat()
            except OSError:
                continue
            found.append((path, stat.st_size, kind, stat.st_mtime))
        return found

    def _age_prune(self):
        if self.directory is None or self.config.rhythm_keep_days == 0:
            return
        today = time.gmtime()
        today_ordinal = today.tm_year * 10000 + today.tm_mon * 100 + today.tm_mday
        # Compare UTC dates as YYYYMMDD integers. ``keep_days`` 90 drops a file 91 days old.
        limit = _shift_yyyymmdd(today_ordinal, self.config.rhythm_keep_days)
        open_paths = {segment.path for segment in self._open.values()}
        try:
            names = list(self.directory.iterdir())
        except OSError as error:
            self._log_once(f"rhythm age prune skipped ({error})")
            return
        for path in names:
            if path in open_paths or not path.is_file():
                continue
            match = _NAME.fullmatch(path.name) or _QUARANTINE.fullmatch(path.name)
            if match is None:
                continue
            if int(match.group(2)) < limit:
                try:
                    path.unlink()
                except OSError as error:
                    self._log_once(f"rhythm could not delete {path} ({error})")

    def _maybe_age_prune(self):
        if self.config.rhythm_keep_days == 0:
            return
        day = time.strftime("%Y%m%d", time.gmtime())
        if day == self._last_age_day:
            return
        self._last_age_day = day
        self._age_prune()

    def _maybe_fsync(self):
        interval = self.config.rhythm_fsync_s
        if interval == 0:
            return
        now = time.monotonic()
        if now - self._last_fsync < interval:
            return
        self._fsync_open()

    def _fsync_open(self):
        for segment in self._open.values():
            self._sync_fd(segment.fd)
        self._last_fsync = time.monotonic()

    def _sync_fd(self, fd):
        if fd is None or fd < 0:
            return
        try:
            self.store.fsync(fd)
        except OSError as error:
            self.io_errors += 1
            self._log_once(f"rhythm fsync failed ({error})")

    def _close_segment(self, sensor_name, abandon):
        segment = self._open.get(sensor_name)
        if segment is None:
            return
        if segment.fd is not None and segment.fd >= 0:
            try:
                os.close(segment.fd)
            except OSError:
                pass
            segment.fd = -1
        if abandon:
            segment.abandoned = True
        else:
            self._open.pop(sensor_name, None)

    def _publish(self, sensor_name, record, data):
        if not self.config.rhythm_publish or self.publish is None or record is None:
            return
        if self.publish_would_block() or not self._pub_lock.acquire(blocking=False):
            self.publish_dropped += 1
            return
        try:
            from .messages import RhythmMinute, Time
            message = RhythmMinute(
                self.config.room_id, SENSOR_CODE[sensor_name], record["map_epoch"], self._seq,
                Time(int(record["minute"]), 0), record["closed"], record["with_points"], record["unavailable"],
                record["clipped"], record["regions_dropped"], record["pose_ok"], record["flags"],
                tuple(region[0] for region in record["regions"]),
                tuple(region[1] for region in record["regions"]),
                tuple(region[2] for region in record["regions"]),
                tuple(region[3] for region in record["regions"]),
                tuple(region[4] for region in record["regions"]),
                self.publisher_generation,
            )
            self.publish(message.to_cdr())
            self._seq += 1
        except Exception:
            self.publish_dropped += 1
        finally:
            self._pub_lock.release()


def _segment_sort_key(name):
    match = _NAME.fullmatch(name)
    if match is None:
        return (name, 0)
    return (match.group(2), int(match.group(3) or 1))


def _shift_yyyymmdd(yyyymmdd, days):
    """UTC date ``days`` before ``yyyymmdd``. A file dated strictly earlier is past the age cap."""
    year, month, day = yyyymmdd // 10000, (yyyymmdd // 100) % 100, yyyymmdd % 100
    shifted = datetime.date(year, month, day) - datetime.timedelta(days=days)
    return shifted.year * 10000 + shifted.month * 100 + shifted.day


class Rhythm:
    """Accumulator plus writer. ``enqueue`` is the only callback-side output."""

    def __init__(self, config, store=None, publish=None, publish_would_block=None, publisher_generation=None):
        validate_rhythm(config)
        self.config = config
        self.hook_errors = 0
        self._last_hook_log = 0.0
        directory = rhythm_directory(config)
        self.accumulator = RhythmAccumulator(
            config.sensors, voxels_per_cell(config.rhythm_cell_m, config.voxel_size), config.rhythm_dims,
            config.rhythm_period_s, windows_per_period(config.rhythm_period_s, config.window_s),
        )
        self.writer = RhythmWriter(
            config, directory, store=store, publish=publish, publish_would_block=publish_would_block,
            publisher_generation=publisher_generation,
        )
        for sensor, flag in self.writer._force_disc.items():
            if flag:
                self.accumulator._sensors[sensor]["force_disc"] = True
        self.writer.start()

    @property
    def rhythm_budget_dropped(self):
        return self.writer.budget_dropped

    def add(self, sensor, sec, keys, map_epoch, tracking=False, residual=None, learn_max_offset_m=None):
        self.accumulator.add(sensor, sec, keys, map_epoch, tracking, residual, learn_max_offset_m)

    def add_empty(self, sensor, sec, map_epoch):
        self.accumulator.add_empty(sensor, sec, map_epoch)

    def note_unavailable(self, sensor, sec, map_epoch):
        self.accumulator.note_unavailable(sensor, sec, map_epoch)

    def discard(self, sensor, sec, map_epoch):
        self.accumulator.discard(sensor, sec, map_epoch)
        self.hook_errors = self.accumulator.hook_errors
        now = time.monotonic()
        if now - self._last_hook_log >= 60:
            self._last_hook_log = now
            log.warning("rhythm hook failed for %s; dropped its open bin", sensor)

    def note_epoch(self, map_epoch):
        self.accumulator.note_epoch(map_epoch)

    def enqueue(self):
        """Move finished records onto the writer queue. No filesystem I/O."""
        try:
            for fragment in self.accumulator.take():
                self._push(fragment)
        except Exception as error:
            log.warning("rhythm enqueue failed (%s)", error)

    def close(self):
        try:
            for fragment in self.accumulator.flush():
                self._push(fragment)
            for sensor, state in self.accumulator._sensors.items():
                loss = state["loss"]
                if loss is None:
                    continue
                state["loss"] = None
                self.writer.try_enqueue(sensor, encode_marker(*loss))
        finally:
            self.writer.close()

    def _push(self, fragment):
        state = self.accumulator._sensors[fragment.sensor]
        loss = state["loss"]
        if loss is not None and not fragment.lost:
            if not self.writer.try_enqueue(fragment.sensor, encode_marker(*loss)):
                state["loss"] = (fragment.minute, fragment.map_epoch)
                self.writer.note_discontinuity(fragment.sensor)
                return
            state["loss"] = None
        if self.writer.try_enqueue(fragment.sensor, fragment.data):
            return
        state["loss"] = (fragment.minute, fragment.map_epoch)
        self.writer.note_discontinuity(fragment.sensor)
