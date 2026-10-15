"""Three-state voxel memory and the window engine the ROS node drives. No ROS imports.

A ``VoxelLayer`` holds two sorted key arrays, ``occupied ⊆ known``: free is known minus
occupied, and every other voxel is UNKNOWN. A window's free space comes from the core's
spec DDA from the sensor origin. Each sensor keeps its own reference and recent layers, in
its own frame, until the JT128↔Mid-360 extrinsic exists (brief §5 R1). The reference is the
settled part of per-voxel window counts (``VoxelStats``); one sensor's reference is the map the
node publishes as ``MapBricks`` snapshots and deltas.
"""

import math
import os
import pathlib
import re
from collections import deque
from dataclasses import dataclass, field, replace
from typing import Optional, Tuple

import numpy as np

from .core import Config, OccupancyMap, cast_rays, decode_pointcloud2, diff, occupancy_map
from .core.change_detection import contains, in_range, pack, unpack, validate
from .messages import (
    BRICK_DIM,
    Brick,
    ChangeEvent,
    Compression,
    EventKind,
    EventState,
    HZ_UNAVAILABLE,
    SINCE_NONE,
    Header,
    MapBricks,
    MapKind,
    MemoryState,
    MemoryStatus,
    SENSOR_JT128,
    SENSOR_MID360,
    SyncTopic,
    Time,
)

_INT16 = (-(1 << 15), (1 << 15) - 1)
_NO_POINTS = np.zeros((0, 3), dtype=np.float32)
_KINDS = {"appeared": EventKind.APPEARED, "disappeared": EventKind.DISAPPEARED, "moved": EventKind.MOVED}
_ROOM_ID = re.compile(r"[A-Za-z0-9][A-Za-z0-9._-]*")
_EMPTY_KEYS = np.zeros(0, dtype=np.int64)
_EMPTY_COUNTS = np.zeros(0, dtype=np.uint32)

# The protocol's replay depths (§5).
MAP_LOG_DEPTH = 10
EVENT_LOG_DEPTH = 100
# pose_residual_m: the registration search reaches this many voxels along each axis.
POSE_SEARCH_VOXELS = 4


@dataclass(frozen=True)
class Sensor:
    """One lidar: its topic, its ``ChangeEvent.sensors`` bit, and its ``per_sensor_hz`` slot."""

    name: str
    topic: str
    bit: int
    hz_index: int


SENSORS = {
    "mid360": Sensor("mid360", "/livox/lidar", SENSOR_MID360, 0),
    "jt128": Sensor("jt128", "/lidar_points", SENSOR_JT128, 1),
}


# MARK: Layers


@dataclass
class VoxelLayer:
    """Occupied/free/unknown evidence as two sorted unique ``int64`` key arrays, ``occupied ⊆ known``."""

    occupied: np.ndarray
    known: np.ndarray
    _brick_count: Optional[int] = field(default=None, repr=False, compare=False)

    @classmethod
    def empty(cls):
        """A layer with every voxel UNKNOWN."""
        return cls(np.zeros(0, dtype=np.int64), np.zeros(0, dtype=np.int64))

    @classmethod
    def observe(cls, occupancy, config):
        """One window: the occupied voxels, plus every voxel an after-ray marks as free.

        ``occupancy`` must be built with evidence on, so it carries ray endpoints.
        """
        free = cast_rays(occupancy, config)
        return cls(occupancy.occupied, np.union1d(occupancy.occupied, free))

    def merge(self, other):
        """Union of evidence. A voxel occupied in either layer stays occupied."""
        return VoxelLayer(np.union1d(self.occupied, other.occupied), np.union1d(self.known, other.known))

    def as_occupancy(self, voxel_size):
        """The occupied set as a core ``OccupancyMap`` (the ``before`` side of a diff)."""
        return OccupancyMap(np.float32(voxel_size), self.occupied, 0, 0)

    def _brick_ids(self):
        i, j, k = unpack(self.known)
        bricks = pack(i >> 3, j >> 3, k >> 3)
        return i, j, k, bricks

    @property
    def brick_count(self):
        """Bricks with at least one known voxel (the ``n_bricks`` a snapshot would carry)."""
        if self._brick_count is None:
            self._brick_count = int(len(np.unique(self._brick_ids()[3]))) if len(self.known) else 0
        return self._brick_count

    def to_bricks(self):
        """Canonical bricks: strictly increasing ``(i, j, k)``, all-free bricks kept, all-unknown omitted.

        Raises:
            ValueError: When a brick index does not fit ``int16`` (beyond about 13 km at 5 cm).
        """
        if len(self.known) == 0:
            return ()
        i, j, k, brick_keys = self._brick_ids()
        unique, slot = np.unique(brick_keys, return_inverse=True)
        bi, bj, bk = unpack(unique)
        if min(bi.min(), bj.min(), bk.min()) < _INT16[0] or max(bi.max(), bj.max(), bk.max()) > _INT16[1]:
            raise ValueError("a voxel lies outside the int16 brick range")
        bit = (i & 7) + 8 * (j & 7) + 64 * (k & 7)
        known = np.zeros((len(unique), BRICK_DIM ** 3 // 8), dtype=np.uint8)
        np.bitwise_or.at(known, (slot, bit >> 3), (1 << (bit & 7)).astype(np.uint8))
        occupied = np.zeros_like(known)
        in_occupied = contains(self.occupied, self.known)
        np.bitwise_or.at(
            occupied, (slot[in_occupied], bit[in_occupied] >> 3), (1 << (bit[in_occupied] & 7)).astype(np.uint8),
        )
        return tuple(
            Brick((int(bi[n]), int(bj[n]), int(bk[n])), occupied[n].tobytes(), known[n].tobytes())
            for n in range(len(unique))
        )

    @classmethod
    def from_bricks(cls, bricks):
        """Inverse of ``to_bricks``."""
        if not bricks:
            return cls.empty()
        coordinates = np.array([brick.coordinate for brick in bricks], dtype=np.int64)

        def keys(masks):
            bits = np.unpackbits(np.frombuffer(b"".join(masks), dtype=np.uint8).reshape(len(bricks), -1), axis=1,
                                 bitorder="little")
            brick, n = np.nonzero(bits)
            base = coordinates[brick] * BRICK_DIM
            return np.unique(pack(base[:, 0] + (n & 7), base[:, 1] + ((n >> 3) & 7), base[:, 2] + (n >> 6)))

        return cls(keys([brick.occupied for brick in bricks]), keys([brick.known for brick in bricks]))


def _nanoseconds(time):
    return time.sec * 1_000_000_000 + time.nanosec


# MARK: Window counts


@dataclass(frozen=True)
class SettleRule:
    """Thresholds that turn per-voxel window counts into reference evidence."""

    occupied_min_hits: int = 1
    occupied_min_ratio: float = 0.0
    free_min_seen: int = 1
    free_max_ratio: float = 0.0


class VoxelStats:
    """Per-voxel window counts: ``seen`` counts the windows that observed the voxel (occupied or
    ray-free), ``hits`` the windows in which it was occupied. Sorted unique keys, parallel counts."""

    def __init__(self):
        self.keys, self.hits, self.seen = _EMPTY_KEYS, _EMPTY_COUNTS, _EMPTY_COUNTS

    def add(self, occupied, known):
        """Counts one window: every voxel of ``known`` once in seen, those of ``occupied`` also in hits."""
        keys = np.union1d(self.keys, known)
        hits = np.zeros(len(keys), dtype=np.uint32)
        seen = np.zeros(len(keys), dtype=np.uint32)
        old = np.searchsorted(keys, self.keys)
        hits[old], seen[old] = self.hits, self.seen
        new = np.searchsorted(keys, known)
        hits[new] += contains(occupied, known).astype(np.uint32)
        seen[new] += np.uint32(1)
        self.keys, self.hits, self.seen = keys, hits, seen

    def settled(self, rule):
        """The voxels the rule settles, as a layer: occupied when ``hits >= occupied_min_hits`` and
        ``hits >= occupied_min_ratio * seen``; else free when ``seen >= free_min_seen`` and
        ``hits <= free_max_ratio * seen``. Every other counted voxel stays UNKNOWN."""
        h, n = self.hits.astype(np.float64), self.seen.astype(np.float64)
        occupied = (self.hits.astype(np.int64) >= rule.occupied_min_hits) & (h >= rule.occupied_min_ratio * n)
        free = ~occupied & (self.seen.astype(np.int64) >= rule.free_min_seen) & (h <= rule.free_max_ratio * n)
        return VoxelLayer(self.keys[occupied], self.keys[occupied | free])

    def erase(self, drop):
        """Drops the counts of the sorted keys ``drop``."""
        keep = ~contains(drop, self.keys)
        self.keys, self.hits, self.seen = self.keys[keep], self.hits[keep], self.seen[keep]


def map_delta(before, after):
    """What turns the canonical brick tuple ``before`` into ``after``: ``(bricks, removed)``, the new
    or changed bricks and the indices into ``before`` of the bricks absent from ``after``."""
    old = {brick.coordinate: brick for brick in before}
    new = {brick.coordinate for brick in after}
    bricks = tuple(brick for brick in after if old.get(brick.coordinate) != brick)
    removed = tuple(index for index, brick in enumerate(before) if brick.coordinate not in new)
    return bricks, removed


def _coverage(reference):
    """Share of the voxels in the index box of ``reference.occupied`` that the reference observed."""
    if not len(reference.occupied):
        return None
    lo = [axis.min() for axis in unpack(reference.occupied)]
    hi = [axis.max() for axis in unpack(reference.occupied)]
    i, j, k = unpack(reference.known)
    inside = (i >= lo[0]) & (i <= hi[0]) & (j >= lo[1]) & (j <= hi[1]) & (k >= lo[2]) & (k <= hi[2])
    cells = int(hi[0] - lo[0] + 1) * int(hi[1] - lo[1] + 1) * int(hi[2] - lo[2] + 1)
    return int(np.count_nonzero(inside)) / cells


def pose_offset(recent, reference_occupied, voxel_size):
    """The translation (meters) that best registers ``recent`` (occupied keys) onto the reference
    occupancy, as one number: per axis, the shift of -POSE_SEARCH_VOXELS .. POSE_SEARCH_VOXELS
    voxels that maximizes the count of recent voxels landing on reference-occupied ones, refined
    by the parabola through that count and its two neighbours (not at the ends of the range);
    the norm of the three per-axis shifts. None when no shift overlaps at all."""
    r = POSE_SEARCH_VOXELS
    lanes = unpack(recent)
    squared = 0.0
    for axis in range(3):
        counts = []
        for shift in range(-r, r + 1):
            moved = [lane + shift if n == axis else lane for n, lane in enumerate(lanes)]
            ok = in_range(*moved)
            counts.append(int(contains(reference_occupied, pack(*(lane[ok] for lane in moved))).sum()))
        # The peak; on ties the shift nearest zero, the negative one first.
        best = 0
        for step in range(1, r + 1):
            for shift in (-step, step):
                if counts[shift + r] > counts[best + r]:
                    best = shift
        if counts[best + r] == 0:
            return None
        shift = float(best)
        if -r < best < r:
            low, mid, high = (float(counts[best + r + d]) for d in (-1, 0, 1))
            curvature = low - 2.0 * mid + high
            if curvature < 0.0:
                shift += (low - high) / (2.0 * curvature)
        meters = shift * voxel_size
        squared += meters * meters
    return math.sqrt(squared)


# MARK: Persistence


def reference_path(state_dir, room_id, sensor):
    """``<state_dir>/<room_id>/reference-<sensor>.cdr``. ``room_id`` must be a plain name."""
    if not _ROOM_ID.fullmatch(room_id):
        raise ValueError(f"room_id '{room_id}' is not a plain file name ([A-Za-z0-9._-], no leading dot)")
    return pathlib.Path(state_dir).expanduser() / room_id / f"reference-{sensor}.cdr"


def save_reference(path, layer, header, voxel_size):
    """Writes ``layer`` as one zlib ``MapBricks`` snapshot, atomically (temp file, then rename)."""
    message = MapBricks(header, Compression.ZLIB, MapKind.SNAPSHOT, 0, voxel_size, layer.to_bricks())
    data = message.to_cdr()
    path = pathlib.Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    scratch = path.with_suffix(path.suffix + ".tmp")
    with open(scratch, "wb") as handle:
        handle.write(data)
        handle.flush()
        os.fsync(handle.fileno())
    os.replace(scratch, path)


def load_reference(path):
    """Reads a saved reference: ``(MapBricks message, VoxelLayer)``. Raises ``messages.CDRError``."""
    message = MapBricks.from_cdr(pathlib.Path(path).read_bytes())
    return message, VoxelLayer.from_bricks(message.bricks)


# MARK: Candidates


@dataclass
class Candidate:
    """One change region followed across windows: pending until it has persisted, then an open
    candidate (CANDIDATE sent, no terminal transition yet)."""

    event_id: int  # 0 while pending
    kind: EventKind
    keys: np.ndarray
    region: object
    first_seen: Time
    last_seen: Time
    windows: int = 1  # windows with fresh support in its unbroken run of matched windows
    missed: int = 0  # consecutive windows without a match


def region_keys(region):
    """Every voxel of a region, both poses for ``moved``, sorted."""
    if region.from_keys is None:
        return region.keys
    return np.union1d(region.keys, region.from_keys)


class CandidateTracker:
    """Transition-only bookkeeping for one sensor's change regions (PR 2.3 adds CONFIRMED).

    A region matches the first open, else the first pending, region of the same kind sharing a
    voxel that no other region of this window matched; otherwise it starts a pending one. A
    region is *fresh* when this window itself supports it (``update``'s ``fresh``); a stale one
    keeps a match alive but neither counts toward promotion nor starts a pending region. An
    appeared or disappeared region that matches no open entry of its kind but shares a voxel
    with the same side of an open moved candidate (its new pose, its old pose) is a half of that
    move and starts nothing; when a window has both halves, they are the move itself and count
    as its match. A pending region becomes a CANDIDATE (and gets its event id) once
    ``candidate_windows`` fresh windows have matched it without a break, and is dropped silently
    the first window it is not matched. An open candidate goes EXPIRED after ``expire_windows``
    consecutive windows without a match. A matched candidate sends nothing: the protocol has one
    message per transition.
    """

    def __init__(self, candidate_windows=1, expire_windows=1):
        self.candidate_windows = candidate_windows
        self.expire_windows = expire_windows
        self.open = []
        self.pending = []

    def update(self, regions, stamp, next_id, fresh=None):
        """Returns ``[(state, candidate)]``: new candidates in region order, then expiries by id.

        ``fresh`` holds one flag per region, None when every region is fresh.
        """
        transitions = []
        open_matched, pending_matched = set(), set()
        promoted, started = [], []

        def first(entries, matched, kind, keys):
            for index, entry in enumerate(entries):
                if index not in matched and entry.kind == kind and len(np.intersect1d(entry.keys, keys, assume_unique=True)):
                    matched.add(index)
                    return entry
            return None

        def half_of(kind, keys):
            """The first open move whose pose on this side (new for appeared, old for disappeared)
            shares a voxel with ``keys``."""
            for index, entry in enumerate(self.open):
                if entry.kind != EventKind.MOVED:
                    continue
                pose = entry.region.keys if kind == EventKind.APPEARED else entry.region.from_keys
                if len(np.intersect1d(pose, keys, assume_unique=True)):
                    return index
            return None

        def promote(candidate):
            candidate = replace(candidate, event_id=next_id())
            transitions.append((EventState.CANDIDATE, candidate))
            promoted.append(candidate)

        # First pass: matches with open candidates of the same kind, and the halves of open moves.
        unmatched, halves = [], {EventKind.APPEARED: set(), EventKind.DISAPPEARED: set()}
        for index, region in enumerate(regions):
            kind = _KINDS.get(region.kind)
            if kind is None:
                continue
            keys = region_keys(region)
            match = first(self.open, open_matched, kind, keys)
            if match is not None:
                match.keys, match.region, match.last_seen, match.missed = keys, region, stamp, 0
                continue
            move = half_of(kind, keys) if kind != EventKind.MOVED else None
            if move is not None:
                halves[kind].add(move)
            unmatched.append((index, region, kind, keys, move))
        # A move reported as its two halves (the diff's pairing gates flicker) is still that move.
        # One half alone is not: it keeps nothing alive, so a move whose other half has gone
        # (removed from its new pose, or its old pose filled again) expires and frees the half.
        absorbed = halves[EventKind.APPEARED] & halves[EventKind.DISAPPEARED]
        for move in absorbed:
            self.open[move].last_seen, self.open[move].missed = stamp, 0
        # Second pass, in region order: the rest match or start pending regions. A half starts
        # nothing while its move is open, whether or not the move was seen whole.
        promoted_pending = set()
        for index, region, kind, keys, move in unmatched:
            if move is not None:
                continue
            supported = fresh is None or fresh[index]
            waiting = first(self.pending, pending_matched, kind, keys)
            if waiting is not None:
                waiting.keys, waiting.region, waiting.last_seen = keys, region, stamp
                waiting.windows += 1 if supported else 0
                if waiting.windows >= self.candidate_windows:
                    promoted_pending.add(id(waiting))
                    promote(waiting)
            elif not supported:
                continue
            elif self.candidate_windows <= 1:
                promote(Candidate(0, kind, keys, region, stamp, stamp))
            else:
                started.append(Candidate(0, kind, keys, region, stamp, stamp))
        expired = set()
        for index in sorted(range(len(self.open)), key=lambda n: self.open[n].event_id):
            if index in open_matched or index in absorbed:
                continue
            self.open[index].missed += 1
            if self.open[index].missed >= self.expire_windows:
                expired.add(index)
                transitions.append((EventState.EXPIRED, self.open[index]))
        self.open = [c for n, c in enumerate(self.open) if n not in expired] + promoted
        self.pending = [c for n, c in enumerate(self.pending)
                        if n in pending_matched and id(c) not in promoted_pending] + started
        return transitions


# MARK: Engine


@dataclass(frozen=True)
class MemoryConfig:
    """Node parameters. Distances in meters, ``window_s`` in seconds."""

    room_id: str = "lab"
    sensors: Tuple[str, ...] = ("mid360", "jt128")
    voxel_size: float = 0.05
    max_range: Optional[float] = 15.0
    tolerance: int = 1
    min_voxels: int = 6
    min_extent: int = 2
    window_s: float = 1.0
    # Defaults measured on the static recording (room_memory_cpp/PLAN.md, "False change events").
    reference_windows: int = 30
    recent_windows: int = 3
    state_dir: Optional[str] = "~/room-memory"
    # Reference evidence from per-voxel window counts (``VoxelStats.settled``).
    occupied_min_hits: int = 2
    occupied_min_ratio: float = 0.3
    free_min_seen: int = 2
    free_max_ratio: float = 0.0
    # Change gate: windows a region must persist before its CANDIDATE, and windows an open
    # candidate may go unmatched before it EXPIRES.
    candidate_windows: int = 3
    expire_windows: int = 3
    # A tracking window adds to the counts of unsettled voxels only while its pose offset is at
    # most this (meters), so a moved rig never teaches the reference its new view.
    learn_max_offset_m: float = 0.02
    # Map stream: the sensor whose reference is published ("" = jt128 when configured, else the
    # first configured), the
    # minimum time between map messages, the maximum time between snapshots, and the share of
    # the last snapshot's size the deltas since it may reach before a snapshot replaces the next delta.
    map_sensor: str = ""
    delta_period_s: float = 5.0
    keyframe_period_s: float = 60.0
    keyframe_delta_ratio: float = 0.5

    def change_config(self):
        """The core config: evidence on, sensor origin (0, 0, 0) in each sensor's frame."""
        return Config(
            voxel_size=self.voxel_size, max_range=self.max_range, tolerance=self.tolerance,
            min_voxels=self.min_voxels, min_extent=self.min_extent, evidence_of_absence=True,
        )

    def settle_rule(self):
        return SettleRule(self.occupied_min_hits, self.occupied_min_ratio, self.free_min_seen, self.free_max_ratio)


@dataclass
class Logged:
    """One published message kept for ``RequestSync`` replays."""

    seq: int
    map_epoch: int
    data: bytes


class _Channel:
    def __init__(self, sensor, config):
        self.sensor = sensor
        self.frames = []
        self.received = 0
        self.failed = 0
        self.last_error = None  # None, or (type name, message). Never the exception or the frame.
        self.reference = VoxelLayer.empty()
        self.stats = VoxelStats()  # every voxel while BUILDING; the not yet settled ones while TRACKING
        self.reference_windows = 0  # windows with usable points counted into the reference
        self.recent = deque(maxlen=config.recent_windows)  # one entry per closed window, empty when it had none
        self.tracker = CandidateTracker(config.candidate_windows, config.expire_windows)
        self.hz = None
        self.residual = None  # pose_offset of the last tracking window with points


class RoomMemory:
    """Accumulation windows, reference/recent layers, candidates, and status for one room.

    The caller feeds frames with ``add_cdr`` or ``add_points``, calls ``close_window`` every
    ``window_s`` and ``status`` at 1 Hz, and publishes the returned messages.
    """

    def __init__(self, config, start):
        unknown = [name for name in config.sensors if name not in SENSORS]
        if unknown or not config.sensors:
            raise ValueError(f"sensors must be a non-empty subset of {sorted(SENSORS)}, got {list(config.sensors)}")
        if config.reference_windows < 1 or config.recent_windows < 1:
            raise ValueError("reference_windows and recent_windows must be >= 1")
        if not (math.isfinite(config.window_s) and config.window_s > 0):
            raise ValueError("window_s must be a finite number > 0 s")
        if (config.occupied_min_hits < 1 or config.free_min_seen < 1 or not 0 <= config.occupied_min_ratio <= 1
                or not 0 <= config.free_max_ratio <= 1):
            raise ValueError("occupied_min_hits and free_min_seen must be >= 1, the ratios in [0, 1]")
        if config.candidate_windows < 1 or config.expire_windows < 1:
            raise ValueError("candidate_windows and expire_windows must be >= 1")
        if not 0 <= config.learn_max_offset_m < 1e9:
            raise ValueError("learn_max_offset_m must be in [0, 1e9) m")
        if not (0 < config.delta_period_s < 1e9 and 0 < config.keyframe_period_s < 1e9
                and 0 <= config.keyframe_delta_ratio < 1e9):
            raise ValueError("delta_period_s and keyframe_period_s must be in (0, 1e9) s, keyframe_delta_ratio in [0, 1e9)")
        names = list(dict.fromkeys(config.sensors))
        if config.map_sensor and config.map_sensor not in names:
            raise ValueError(f"map_sensor '{config.map_sensor}' is not one of the sensors {list(config.sensors)}")
        self.config = config
        self.change = config.change_config()
        validate(self.change)
        self.channels = {name: _Channel(SENSORS[name], config) for name in names}
        self.map_sensor = config.map_sensor or ("jt128" if "jt128" in names else names[0])
        self.state = MemoryState.BUILDING
        self.map_epoch = 0
        self.event_seq = 0
        self.status_seq = 0
        self.dropped_windows = 0
        self.last_window_dropped = False
        self._next_event_id = ((start.sec * 1000 + start.nanosec // 1_000_000) << 20) & ((1 << 64) - 1)
        self._coverage = None
        # Map stream: the bricks and epoch of the last live map message, its seq counter and
        # timing, and the bytes sent since the last snapshot.
        self._map_started = False
        self._map_bricks = ()
        self._map_epoch_sent = 0
        self._map_seq = 0
        self._last_map_ns = 0
        self._last_keyframe_ns = 0
        self._keyframe_bytes = 0
        self._delta_bytes = 0
        self._map_log = deque(maxlen=MAP_LOG_DEPTH)
        self._event_log = deque(maxlen=EVENT_LOG_DEPTH)
        self._open_latest = {}  # event_id -> latest Logged message, open events only
        self._load(start)

    def _paths(self):
        return {name: reference_path(self.config.state_dir, self.config.room_id, name) for name in self.channels}

    def _load(self, start):
        if self.config.state_dir is None:
            return
        paths = self._paths()
        present = {name: path for name, path in paths.items() if path.exists()}
        if not present:
            return
        if len(present) != len(paths):
            missing = sorted(set(paths) - set(present))
            raise ValueError(f"saved references are incomplete for room '{self.config.room_id}': missing {missing}")
        epochs = set()
        for name, path in present.items():
            message, layer = load_reference(path)
            if message.header.room_id != self.config.room_id:
                raise ValueError(f"{path} holds room '{message.header.room_id}', not '{self.config.room_id}'")
            if message.voxel_size != float(np.float32(self.config.voxel_size)):
                raise ValueError(f"{path} uses voxel_size {message.voxel_size} m, not {self.config.voxel_size} m")
            epochs.add(message.header.map_epoch)
            channel = self.channels[name]
            self._set_reference(channel, layer)
            channel.reference_windows = self.config.reference_windows
        if len(epochs) != 1:
            raise ValueError(f"saved references disagree on map_epoch: {sorted(epochs)}")
        # seq restarts at 0, so the restart takes a new epoch: no (map_epoch, seq) sync cursor from
        # before it names a message after it. Saved at once, so the next restart moves on again.
        self.map_epoch = epochs.pop() + 1
        self.save(start)
        self.state = MemoryState.TRACKING

    def save(self, stamp):
        """Writes every sensor's reference with the current ``map_epoch``. No-op without ``state_dir``."""
        if self.config.state_dir is None:
            return
        header = Header(self.config.room_id, self.map_epoch, 0, stamp)
        for name, path in self._paths().items():
            save_reference(path, self.channels[name].reference, header, self.config.voxel_size)

    def add_points(self, sensor, positions):
        """Queues decoded ``(n, 3)`` positions (meters, sensor frame) for the current window."""
        channel = self.channels[sensor]
        channel.received += 1
        channel.frames.append(np.asarray(positions, dtype=np.float32).reshape(-1, 3))

    def add_cdr(self, sensor, data):
        """Decodes one CDR PointCloud2 and queues it. Returns False when the frame is rejected.

        On rejection, ``channel.last_error`` is ``(type name, message)``. The exception object
        is not kept. An accepted frame sets ``last_error`` to ``None``.
        """
        try:
            positions = decode_pointcloud2(data).positions
        except Exception as error:  # rejected frame, not a crash; BaseException still propagates
            channel = self.channels[sensor]
            channel.received += 1
            channel.failed += 1
            # Type name and message only. The traceback would keep this frame and decoder locals.
            type_name = type(error).__name__
            message = str(error)
            channel.last_error = (type_name, message)
            return False
        self.add_points(sensor, positions)
        self.channels[sensor].last_error = None
        return True

    def _next_id(self):
        value = self._next_event_id
        self._next_event_id = (value + 1) & ((1 << 64) - 1)
        return value

    def _set_reference(self, channel, reference):
        channel.reference = reference
        if channel.sensor.name == self.map_sensor:
            self._coverage = _coverage(reference)

    def _reference_ready(self, channel):
        """A built reference: its quota of windows and some settled occupancy, which tracking
        needs (``pose_offset`` registers on it, and without it no window may teach the reference)."""
        return channel.reference_windows >= self.config.reference_windows and len(channel.reference.occupied) > 0

    def close_window(self, stamp):
        """Integrates the queued frames. Returns the ``ChangeEvent`` messages to publish, in order."""
        events = []
        dropped = False
        for channel in self.channels.values():
            frames, channel.frames = channel.frames, []
            if channel.failed and not frames:
                dropped = True
            channel.failed = 0
            if self.state == MemoryState.BUILDING:
                # A sensor's reference freezes once it is ready while the others catch up, and
                # only windows with at least one occupied voxel count toward it.
                if frames and not self._reference_ready(channel):
                    window = occupancy_map(np.concatenate(frames), self.change)
                    if len(window.occupied):
                        observed = VoxelLayer.observe(window, self.change)
                        channel.stats.add(observed.occupied, observed.known)
                        channel.reference_windows += 1
                        self._set_reference(channel, channel.stats.settled(self.config.settle_rule()))
                continue
            points = np.concatenate(frames) if frames else _NO_POINTS
            window = occupancy_map(points, self.change) if frames else None
            # A window without usable points still ages the recent layer, but sends nothing.
            if window is None or not len(window.occupied):
                channel.recent.append(_NO_POINTS)
                continue
            channel.recent.append(points)
            reference = channel.reference
            recent = window if len(channel.recent) == 1 else occupancy_map(np.concatenate(channel.recent), self.change)
            # The rig-pose estimate uses this window alone, so a move shows in full at once.
            channel.residual = pose_offset(window.occupied, reference.occupied, self.config.voxel_size)
            # Voxels the reference has not settled keep counting, so new coverage joins it, unless
            # the rig may have moved.
            if channel.residual is not None and channel.residual <= self.config.learn_max_offset_m:
                observed = VoxelLayer.observe(window, self.change)
                channel.stats.add(np.setdiff1d(observed.occupied, reference.known, assume_unique=True),
                                  np.setdiff1d(observed.known, reference.known, assume_unique=True))
            report = diff(reference.as_occupancy(self.config.voxel_size), recent, self.change, reference.known)
            # The recent layer carries a return and its ray for recent_windows windows, so only
            # this window's own evidence counts toward the persistence gate: an appeared or moved
            # region is fresh when this window occupies one of its (new-pose) voxels, a disappeared
            # one when this window's rays traverse one of its voxels.
            gone = [region.keys for region in report.regions if region.kind == "disappeared"]
            free = cast_rays(window, self.change, np.unique(np.concatenate(gone))) if gone else _EMPTY_KEYS
            fresh = [bool(contains(free if region.kind == "disappeared" else window.occupied, region.keys).any())
                     for region in report.regions]
            for state, candidate in channel.tracker.update(report.regions, stamp, self._next_id, fresh):
                events.append(self._event(channel, state, candidate, stamp))
                self._log_event(events[-1])
        if dropped:
            self.dropped_windows += 1
        self.last_window_dropped = dropped
        if self.state == MemoryState.BUILDING and all(self._reference_ready(c) for c in self.channels.values()):
            self.state = MemoryState.TRACKING
            # From here on only the voxels the reference has not settled keep their counts.
            for channel in self.channels.values():
                channel.stats.erase(channel.reference.known)
            self.map_epoch += 1
            self.save(stamp)
        return events

    def _log_event(self, event):
        entry = Logged(event.header.seq, event.header.map_epoch, event.to_cdr())
        if event.state in (EventState.CANDIDATE, EventState.CONFIRMED):
            self._open_latest[event.event_id] = entry
        else:
            self._open_latest.pop(event.event_id, None)
        self._event_log.append(entry)

    def _map_message(self, kind, stamp, bricks, base_epoch=0, removed=()):
        header = Header(self.config.room_id, self.map_epoch, self._map_seq, stamp)
        return MapBricks(header, Compression.ZLIB, kind, base_epoch, self.config.voxel_size, bricks, removed)

    def _publish_map(self, message):
        data = message.to_cdr()
        ns = _nanoseconds(message.header.stamp)
        if message.kind == MapKind.SNAPSHOT:
            self._last_keyframe_ns, self._keyframe_bytes, self._delta_bytes = ns, len(data), 0
        else:
            self._delta_bytes += len(data)
        self._last_map_ns = ns
        self._map_epoch_sent = message.header.map_epoch
        self._map_log.append(Logged(self._map_seq, message.header.map_epoch, data))
        self._map_seq += 1
        return data

    def map_update(self, stamp):
        """The live ``MapBricks`` CDR due at ``stamp``, or None (call once per window, after
        ``close_window``). Nothing while BUILDING; the first message is a snapshot. Afterwards,
        once ``delta_period_s`` has passed since the last message, voxels that settled since join
        the references (``map_epoch`` + 1, references saved) and go out as a delta of the changed
        bricks; a snapshot instead when ``keyframe_period_s`` has passed since the last one, or
        when the deltas since it would exceed ``keyframe_delta_ratio`` of its size."""
        if self.state == MemoryState.BUILDING:
            return None
        ns = _nanoseconds(stamp)
        if not self._map_started:
            self._map_started = True
            self._map_bricks = self.channels[self.map_sensor].reference.to_bricks()
            return self._publish_map(self._map_message(MapKind.SNAPSHOT, stamp, self._map_bricks))
        keyframe_due = ns - self._last_keyframe_ns >= int(self.config.keyframe_period_s * 1e9)
        if not keyframe_due and ns - self._last_map_ns < int(self.config.delta_period_s * 1e9):
            return None
        changed = False
        for channel in self.channels.values():
            fresh = channel.stats.settled(self.config.settle_rule())
            if not len(fresh.known):
                continue
            channel.stats.erase(fresh.known)
            self._set_reference(channel, channel.reference.merge(fresh))
            changed = True
        if not changed and not keyframe_due:
            return None
        if changed:
            self.map_epoch += 1
            self.save(stamp)
        bricks = self.channels[self.map_sensor].reference.to_bricks() if changed else self._map_bricks
        if not keyframe_due:
            upserts, removed = map_delta(self._map_bricks, bricks)
            delta = self._map_message(MapKind.DELTA, stamp, upserts, self._map_epoch_sent, removed)
            if self._delta_bytes + len(delta.to_cdr()) <= self.config.keyframe_delta_ratio * self._keyframe_bytes:
                self._map_bricks = bricks
                return self._publish_map(delta)
        self._map_bricks = bricks
        return self._publish_map(self._map_message(MapKind.SNAPSHOT, stamp, bricks))

    def sync(self, request, stamp):
        """The answer to one ``RequestSync`` (protocol §5): CDR messages to publish in order on the
        sync topic of ``request.topic``. Empty for another ``room_id``, and for a map request
        before the first live map message."""
        if request.header.room_id != self.config.room_id:
            return []

        def since_index(log):
            if request.since is None:
                return None
            return next((n for n, entry in enumerate(log) if (entry.map_epoch, entry.seq) == request.since), None)

        if request.topic == SyncTopic.MAP_BRICKS:
            if not self._map_started:
                return []
            log = list(self._map_log)
            at = since_index(log)
            if at is not None and at + 1 < len(log):
                return [entry.data for entry in log[at + 1:]]
            # A fresh snapshot of the last live state, under the latest live seq.
            header = Header(self.config.room_id, self.map_epoch, self._map_seq - 1, stamp)
            return [MapBricks(header, Compression.ZLIB, MapKind.SNAPSHOT, 0, self.config.voxel_size,
                              self._map_bricks).to_cdr()]
        log = list(self._event_log)
        at = since_index(log)
        if at is not None:
            return [entry.data for entry in log[at + 1:]]
        # Bounded fallback: open events whose latest message is older than the kept ones, then
        # every kept message.
        oldest = log[0].seq if log else SINCE_NONE
        older = sorted((entry for entry in self._open_latest.values() if entry.seq < oldest), key=lambda e: e.seq)
        return [entry.data for entry in older] + [entry.data for entry in log]

    def _event(self, channel, state, candidate, stamp):
        region = candidate.region
        header = Header(self.config.room_id, self.map_epoch, self.event_seq, stamp)
        self.event_seq += 1
        keys = candidate.keys
        evidence = float(np.mean(contains(channel.reference.known, keys)))
        mask = MapBricks(
            Header(self.config.room_id, self.map_epoch, 0, stamp), Compression.ZLIB, MapKind.SNAPSHOT, 0,
            self.config.voxel_size, VoxelLayer(keys, keys).to_bricks(),
        )
        return ChangeEvent(
            header=header,
            event_id=candidate.event_id,
            kind=candidate.kind,
            state=state,
            centroid=tuple(region.centroid.tolist()),
            aabb_min=tuple(region.bounds_min.tolist()),
            aabb_max=tuple(region.bounds_max.tolist()),
            from_centroid=None if region.from_centroid is None else tuple(region.from_centroid.tolist()),
            voxel_count=region.voxel_count,
            volume_m3=float(region.volume),
            evidence=evidence,
            alpha=None,
            sensors=channel.sensor.bit,
            first_seen=candidate.first_seen,
            last_seen=candidate.last_seen,
            mask=mask,
        )

    def status(self, stamp, period_s):
        """``MemoryStatus`` for the last ``period_s`` seconds; resets the per-sensor frame counts.

        DEGRADED while a configured sensor measured 0 Hz or the last window was dropped.
        ``n_bricks`` and ``coverage_known_frac`` describe the map sensor's reference;
        ``pose_residual_m`` is the largest per-sensor residual of the last tracking windows;
        ``events_seq`` is the seq of the newest event message, None before the first.
        """
        hz = [None, None]
        for channel in self.channels.values():
            channel.hz = min(int(channel.received / period_s + 0.5), HZ_UNAVAILABLE - 1) if period_s > 0 else None
            channel.received = 0
            hz[channel.sensor.hz_index] = channel.hz
        state = self.state
        residuals = [c.residual for c in self.channels.values() if c.residual is not None]
        residual = max(residuals) if residuals and self.state != MemoryState.BUILDING else None
        if self.last_window_dropped or any(channel.hz == 0 for channel in self.channels.values()):
            state = MemoryState.DEGRADED
        message = MemoryStatus(
            header=Header(self.config.room_id, self.map_epoch, self.status_seq, stamp),
            state=state,
            n_bricks=self.channels[self.map_sensor].reference.brick_count,
            n_open_events=sum(len(channel.tracker.open) for channel in self.channels.values()),
            coverage_known_frac=self._coverage,
            pose_residual_m=residual,
            window_s=self.config.window_s,
            mid360_hz=hz[0],
            jt128_hz=hz[1],
            dropped_windows=self.dropped_windows,
            events_seq=self.event_seq - 1 if self.event_seq else None,
        )
        self.status_seq += 1
        return message
