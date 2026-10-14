"""Three-state voxel memory and the window engine the ROS node drives. No ROS imports.

A ``VoxelLayer`` holds two sorted key arrays, ``occupied ⊆ known``: free is known minus
occupied, and every other voxel is UNKNOWN. A window's free space comes from the core's
spec DDA from the sensor origin. Each sensor keeps its own reference and recent layers, in
its own frame, until the JT128↔Mid-360 extrinsic exists (brief §5 R1).
"""

import math
import os
import pathlib
import re
from collections import deque
from dataclasses import dataclass, field
from typing import Optional, Tuple

import numpy as np

from .core import Config, OccupancyMap, cast_rays, decode_pointcloud2, diff, occupancy_map
from .core.change_detection import contains, pack, unpack, validate
from .messages import (
    BRICK_DIM,
    Brick,
    ChangeEvent,
    Compression,
    EventKind,
    EventState,
    HZ_UNAVAILABLE,
    Header,
    MapBricks,
    MapKind,
    MemoryState,
    MemoryStatus,
    SENSOR_JT128,
    SENSOR_MID360,
    Time,
)

_INT16 = (-(1 << 15), (1 << 15) - 1)
_NO_POINTS = np.zeros((0, 3), dtype=np.float32)
_KINDS = {"appeared": EventKind.APPEARED, "disappeared": EventKind.DISAPPEARED, "moved": EventKind.MOVED}
_ROOM_ID = re.compile(r"[A-Za-z0-9][A-Za-z0-9._-]*")


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
    """One open change: CANDIDATE sent, no terminal transition yet."""

    event_id: int
    kind: EventKind
    keys: np.ndarray
    region: object
    first_seen: Time
    last_seen: Time


def region_keys(region):
    """Every voxel of a region, both poses for ``moved``, sorted."""
    if region.from_keys is None:
        return region.keys
    return np.union1d(region.keys, region.from_keys)


class CandidateTracker:
    """Transition-only bookkeeping for one sensor's change regions (PR 2.3 adds CONFIRMED).

    A region without an open candidate of the same kind sharing a voxel starts a CANDIDATE.
    A candidate that no region of the next window matches goes EXPIRED. A matched
    candidate sends nothing: the protocol has one message per lifecycle transition.
    """

    def __init__(self):
        self.open = []

    def update(self, regions, stamp, next_id):
        """Returns ``[(state, candidate)]`` in a fixed order: new candidates, then expiries."""
        transitions = []
        matched = set()
        for region in regions:
            kind = _KINDS.get(region.kind)
            if kind is None:
                continue
            keys = region_keys(region)
            match = next(
                (c for c in self.open if c.event_id not in matched and c.kind == kind
                 and len(np.intersect1d(c.keys, keys, assume_unique=True))),
                None,
            )
            if match is None:
                match = Candidate(next_id(), kind, keys, region, stamp, stamp)
                self.open.append(match)
                transitions.append((EventState.CANDIDATE, match))
            else:
                match.keys, match.region, match.last_seen = keys, region, stamp
            matched.add(match.event_id)
        for candidate in sorted(self.open, key=lambda c: c.event_id):
            if candidate.event_id not in matched:
                transitions.append((EventState.EXPIRED, candidate))
        self.open = [c for c in self.open if c.event_id in matched]
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
    reference_windows: int = 5
    recent_windows: int = 1
    state_dir: Optional[str] = "~/room-memory"

    def change_config(self):
        """The core config: evidence on, sensor origin (0, 0, 0) in each sensor's frame."""
        return Config(
            voxel_size=self.voxel_size, max_range=self.max_range, tolerance=self.tolerance,
            min_voxels=self.min_voxels, min_extent=self.min_extent, evidence_of_absence=True,
        )


class _Channel:
    def __init__(self, sensor, recent_windows):
        self.sensor = sensor
        self.frames = []
        self.received = 0
        self.failed = 0
        self.reference = VoxelLayer.empty()
        self.reference_windows = 0  # windows with usable points merged into the reference
        self.recent = deque(maxlen=recent_windows)  # one entry per closed window, empty when it had none
        self.tracker = CandidateTracker()
        self.hz = None


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
        self.config = config
        self.change = config.change_config()
        validate(self.change)
        self.channels = {name: _Channel(SENSORS[name], config.recent_windows) for name in config.sensors}
        self.state = MemoryState.BUILDING
        self.map_epoch = 0
        self.event_seq = 0
        self.status_seq = 0
        self.dropped_windows = 0
        self.last_window_dropped = False
        self._next_event_id = ((start.sec * 1000 + start.nanosec // 1_000_000) << 20) & ((1 << 64) - 1)
        self._load()

    def _paths(self):
        return {name: reference_path(self.config.state_dir, self.config.room_id, name) for name in self.channels}

    def _load(self):
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
            channel.reference = layer
            channel.reference_windows = self.config.reference_windows
        if len(epochs) != 1:
            raise ValueError(f"saved references disagree on map_epoch: {sorted(epochs)}")
        self.map_epoch = epochs.pop()
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
        """Decodes one CDR PointCloud2 and queues it. Returns False when the frame is rejected."""
        try:
            positions = decode_pointcloud2(data).positions
        except Exception:  # any decode failure is a rejected frame, never a crash (PointCloudError and beyond)
            channel = self.channels[sensor]
            channel.received += 1
            channel.failed += 1
            return False
        self.add_points(sensor, positions)
        return True

    def _next_id(self):
        value = self._next_event_id
        self._next_event_id = (value + 1) & ((1 << 64) - 1)
        return value

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
                # A sensor's reference freezes at its own quota while the others catch up, and
                # only windows with at least one occupied voxel count toward it.
                if frames and channel.reference_windows < self.config.reference_windows:
                    window = occupancy_map(np.concatenate(frames), self.change)
                    if len(window.occupied):
                        channel.reference = channel.reference.merge(VoxelLayer.observe(window, self.change))
                        channel.reference_windows += 1
                continue
            points = np.concatenate(frames) if frames else _NO_POINTS
            window = occupancy_map(points, self.change) if frames else None
            # A window without usable points still ages the recent layer, but sends nothing.
            if window is None or not len(window.occupied):
                channel.recent.append(_NO_POINTS)
                continue
            channel.recent.append(points)
            recent = window if len(channel.recent) == 1 else occupancy_map(np.concatenate(channel.recent), self.change)
            reference = channel.reference
            report = diff(reference.as_occupancy(self.config.voxel_size), recent, self.change, reference.known)
            for state, candidate in channel.tracker.update(report.regions, stamp, self._next_id):
                events.append(self._event(channel, state, candidate, stamp))
        if dropped:
            self.dropped_windows += 1
        self.last_window_dropped = dropped
        if self.state == MemoryState.BUILDING and all(
            channel.reference_windows >= self.config.reference_windows for channel in self.channels.values()
        ):
            self.state = MemoryState.TRACKING
            self.map_epoch += 1
            self.save(stamp)
        return events

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
        ``pose_residual_m`` and ``coverage_known_frac`` are NaN: no rig-pose guard and no
        configured room volume yet.
        """
        hz = [None, None]
        for channel in self.channels.values():
            channel.hz = min(int(channel.received / period_s + 0.5), HZ_UNAVAILABLE - 1) if period_s > 0 else None
            channel.received = 0
            hz[channel.sensor.hz_index] = channel.hz
        state = self.state
        if self.last_window_dropped or any(channel.hz == 0 for channel in self.channels.values()):
            state = MemoryState.DEGRADED
        message = MemoryStatus(
            header=Header(self.config.room_id, self.map_epoch, self.status_seq, stamp),
            state=state,
            n_bricks=sum(channel.reference.brick_count for channel in self.channels.values()),
            n_open_events=sum(len(channel.tracker.open) for channel in self.channels.values()),
            coverage_known_frac=None,
            pose_residual_m=None,
            window_s=self.config.window_s,
            mid360_hz=hz[0],
            jt128_hz=hz[1],
            dropped_windows=self.dropped_windows,
        )
        self.status_seq += 1
        return message
