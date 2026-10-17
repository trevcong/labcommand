"""Operator actions on the room memory (protocol §3.6, D46): ``RoomMemory.operate``, the held
ACCEPTED, the window hooks for suppressions and confirmed candidates, and their durable writes.
The C++ twin is ``room_memory_cpp/src/operator.cpp``; both write the same bytes.

Every durable effect is written before it is committed in memory: a write that installs nothing
(``StorageError``) changes nothing and answers STORAGE_FAILED. A write that installed the file but
could not sync its directory is committed, because a restart reads the new file. Everything here
runs on the node's single-threaded executor, like ``close_window`` and ``sync``.
"""

import os
import pathlib
from dataclasses import dataclass, replace
from typing import Optional

import numpy as np

from .core import cast_rays
from .core.change_detection import contains
from .memory import _EMPTY_KEYS, _room_dir, _same_shape, VoxelLayer, reference_path, region_keys
from .messages import (
    Compression, EventKind, EventState, Header, MapBricks, MapKind, MemoryState, OperatorActionKind, OperatorOutcome,
)

STORED_ACTIONS = 256  # repeated action ids answered from memory; a restart forgets them
_HEADER = "room_memory suppressed 1"
_KINDS = {EventKind.APPEARED: "appeared", EventKind.DISAPPEARED: "disappeared", EventKind.MOVED: "moved"}
_REGION_KINDS = {"appeared": EventKind.APPEARED, "disappeared": EventKind.DISAPPEARED, "moved": EventKind.MOVED}

# Tests only: called with "write", "sync", "rename" or "directory" before that step of
# ``write_staged``; returning True makes the step fail.
WRITE_FAULT = None


class StorageError(OSError):
    """A durable write that installed nothing: the old file (or none) is what a reader sees."""


@dataclass
class Suppression:
    """A dismissed change: regions of its kind (and, for a move, its halves) of the same shape over
    these voxels are dropped before the tracker sees them. ``cleared`` counts the windows that saw
    the voxels back at the reference state since a matching region was last seen."""

    event_id: int
    kind: EventKind
    keys: np.ndarray  # the new pose for a move
    from_keys: np.ndarray  # the old pose; empty unless moved
    cleared: int = 0


@dataclass
class HeldAccept:
    """An operator accept waiting for the map message that carries it (protocol §4.2)."""

    channel: str
    candidate: object
    epoch: Optional[int] = None


def _injected(stage):
    return WRITE_FAULT is not None and WRITE_FAULT(stage)


def write_staged(path, data):
    """Temp file, fsync, rename, then fsync the directory, like ``_write_atomic``, but it says how far
    it got. Raises ``StorageError`` when it fails before the rename (the temp file is removed).
    Returns False when the file is installed but the directory sync failed: every reader, a
    restart included, sees the new file. Creates missing parents."""
    path = pathlib.Path(path)
    try:
        path.parent.mkdir(parents=True, exist_ok=True)
    except OSError as error:
        raise StorageError(f"cannot create {path.parent}: {error}") from None
    scratch = path.with_name(path.name + ".tmp")

    def fail(what):
        scratch.unlink(missing_ok=True)
        raise StorageError(what)

    try:
        if _injected("write"):
            raise OSError("injected")
        with open(scratch, "wb") as handle:
            handle.write(data)
            handle.flush()
            if _injected("sync"):
                raise OSError("injected")
            os.fsync(handle.fileno())
    except OSError:
        fail(f"cannot write or sync {scratch}")
    try:
        if _injected("rename"):
            raise OSError("injected")
        os.replace(scratch, path)
    except OSError:
        fail(f"cannot rename {scratch}")
    # Installed: what every reader, a restart included, sees from here on.
    if _injected("directory"):
        return False
    try:
        directory = os.open(path.parent, os.O_RDONLY)
    except OSError:
        return False
    try:
        os.fsync(directory)
        return True
    except OSError:
        return False
    finally:
        os.close(directory)


def suppressions_path(state_dir, room_id, sensor):
    """``<state_dir>/<room_id>/suppressed-<sensor>``: one file per sensor, like its reference."""
    return _room_dir(state_dir, room_id) / f"suppressed-{sensor}"


def format_suppressions(entries):
    """The file: the header line, then ``<event_id> <kind> <n> <key>... <m> <key>...`` per
    suppression in event id order (keys as decimal packed int64; m is 0 unless moved)."""
    lines = [_HEADER]
    for s in entries:
        words = [str(s.event_id), _KINDS[s.kind]]
        for pose in (s.keys, s.from_keys):
            words.append(str(len(pose)))
            words += [str(int(key)) for key in pose]
        lines.append(" ".join(words))
    return "".join(line + "\n" for line in lines)


def parse_suppressions(text):
    """Raises ``ValueError`` unless ``text`` is exactly what ``format_suppressions`` writes for some
    list (event ids increasing, keys sorted and unique, a pose for each kind)."""
    def malformed(why):
        return ValueError(f"the suppressions file is malformed: {why}")

    if not text.startswith(_HEADER + "\n"):
        raise malformed("bad header")
    entries = []
    for line in text[len(_HEADER) + 1:-1].split("\n") if len(text) > len(_HEADER) + 1 else []:
        words = line.split(" ")
        if "" in words:
            raise malformed(f"line '{line}' is not single-spaced")
        at = 0

        def number(low, high):
            nonlocal at
            if at >= len(words):
                raise malformed(f"line '{line}' is short")
            word = words[at]
            at += 1
            try:
                value = int(word)
            except ValueError:
                raise malformed(f"'{word}' is not a number") from None
            if str(value) != word or not low <= value <= high:
                raise malformed(f"'{word}' is not a number")
            return value

        event_id = number(0, (1 << 64) - 1)
        if at >= len(words) or words[at] not in _REGION_KINDS:
            raise malformed(f"kind in '{line}'")
        kind = _REGION_KINDS[words[at]]
        at += 1
        poses = []
        for _ in range(2):
            count = number(0, (1 << 64) - 1)
            if count > len(words):
                raise malformed(f"line '{line}' is short")
            keys = [number(-(1 << 63), (1 << 63) - 1) for _ in range(count)]
            if any(b <= a for a, b in zip(keys, keys[1:])):
                raise malformed("keys are not sorted and unique")
            poses.append(np.array(keys, dtype=np.int64))
        if at != len(words):
            raise malformed(f"line '{line}' is long")
        if not len(poses[0]) or (kind == EventKind.MOVED) == (not len(poses[1])):
            raise malformed(f"event {event_id} has the wrong poses for {_KINDS[kind]}")
        if entries and event_id <= entries[-1].event_id:
            raise malformed("event ids are not increasing")
        entries.append(Suppression(event_id, kind, poses[0], poses[1]))
    if not text.endswith("\n"):
        raise malformed("no final newline")
    return entries


def _shown(candidate):
    """The candidate as its published geometry describes it: what an operator event carries."""
    return replace(candidate, region=candidate.published, keys=region_keys(candidate.published))


def _from_keys(region):
    return _EMPTY_KEYS if region.from_keys is None else region.from_keys


def _absorbed(reference, change):
    """The reference with a published change absorbed: an appearance (or a move's new pose)
    becomes occupied, a disappearance (or the rest of a move's old pose) known free."""
    occupy, clear = _EMPTY_KEYS, _EMPTY_KEYS
    if change.kind == "disappeared":
        clear = change.keys
    else:
        occupy = change.keys
        if change.kind == "moved":
            clear = np.setdiff1d(change.from_keys, change.keys, assume_unique=True)
    occupied = np.union1d(np.setdiff1d(reference.occupied, clear, assume_unique=True), occupy).astype(np.int64)
    known = np.union1d(reference.known, np.union1d(occupy, clear)).astype(np.int64)
    return VoxelLayer(occupied, known)


def _intersects(a, b):
    return bool(len(a) and len(b) and len(np.intersect1d(a, b, assume_unique=True)))


def _match(s, region, tolerance, fraction):
    """(touches, same): the region shares a voxel with the matching pose of the suppression, and
    keeps its shape. A move's suppression also takes its halves (an appeared region on its new
    pose, a disappeared one on its old pose)."""
    kind = _REGION_KINDS.get(region.kind)
    if kind is None:
        return False, False

    def pose(keys, suppressed):
        return _intersects(keys, suppressed), _same_shape(keys, suppressed, tolerance, fraction)

    if kind == s.kind and s.kind != EventKind.MOVED:
        return pose(region.keys, s.keys)
    if kind == s.kind:
        touches = _intersects(region.keys, s.keys) or _intersects(_from_keys(region), s.from_keys)
        return touches, bool(touches and _same_shape(region.keys, s.keys, tolerance, fraction)
                             and _same_shape(_from_keys(region), s.from_keys, tolerance, fraction))
    if s.kind != EventKind.MOVED:
        return False, False
    if kind == EventKind.APPEARED:
        return pose(region.keys, s.keys)
    if kind == EventKind.DISAPPEARED:
        return pose(region.keys, s.from_keys)
    return False, False


def _supported_by(memory, evidence, keys):
    count = int(contains(evidence, keys).sum())
    return count >= memory.config.fresh_min_voxels and count >= memory.config.fresh_min_fraction * len(keys)


def _agrees(memory, kind, keys, from_keys, window):
    """This window's own evidence shows the region back at the reference state: an appearance's
    voxels (a move's new pose) traversed by its rays, a disappearance's voxels (a move's old pose)
    occupied."""
    def free_over(target):
        return cast_rays(window, memory.change, target)
    if kind == EventKind.APPEARED:
        return _supported_by(memory, free_over(keys), keys)
    if kind == EventKind.DISAPPEARED:
        return _supported_by(memory, window.occupied, keys)
    return _supported_by(memory, free_over(keys), keys) and _supported_by(memory, window.occupied, from_keys)


def load_suppressions(memory, references):
    """Reads each sensor's file when the references loaded; without them (BUILDING) removes a stale
    one, so a new reference starts without the old room's dismissals."""
    if memory.config.state_dir is None:
        return
    for name in memory.channels:
        path = suppressions_path(memory.config.state_dir, memory.config.room_id, name)
        if not path.exists():
            continue
        if not references:
            path.unlink()
            continue
        try:
            text = path.read_bytes().decode("ascii")
        except UnicodeDecodeError:
            raise ValueError("the suppressions file is malformed: not ASCII") from None
        memory._suppressions[name] = parse_suppressions(text)


def _write_suppressions(memory, name, entries):
    """Raises ``StorageError``; records an uncertain or cleared ``storage_error``."""
    durable = write_staged(suppressions_path(memory.config.state_dir, memory.config.room_id, name),
                           format_suppressions(entries).encode("ascii"))
    memory.storage_error = None if durable else f"suppressions of {name} written, directory not synced"


def persist_suppressions(memory):
    """At the end of each window: writes every list a lift changed, retried every window until it lands."""
    for name in memory.channels:
        if not memory._suppressions_dirty[name]:
            continue
        if memory.config.state_dir is None:
            memory._suppressions_dirty[name] = False
            continue
        try:
            _write_suppressions(memory, name, memory._suppressions[name])
            memory._suppressions_dirty[name] = False
        except StorageError as error:
            memory.storage_error = str(error)


def screen_suppressed(memory, channel, regions, fresh, window):
    """Before the tracker: drops suppressed regions with their fresh flags, then counts or lifts each
    suppression. Returns ``(regions, fresh)``."""
    name = channel.sensor.name
    entries = memory._suppressions[name]
    if not entries:
        return regions, fresh
    config = memory.config
    seen = [False] * len(entries)
    kept, kept_fresh = [], []
    for index, region in enumerate(regions):
        drop = False
        for n, s in enumerate(entries):
            touches, same = _match(s, region, config.tolerance, config.stable_min_fraction)
            seen[n] = seen[n] or touches
            drop = drop or (touches and same)
        if not drop:
            kept.append(region)
            kept_fresh.append(fresh is None or fresh[index])
    # A matching region keeps a suppression alive; a window that sees its voxels back at the
    # reference state counts toward lifting it; any other window leaves the count alone.
    still = []
    for n, s in enumerate(entries):
        if seen[n]:
            s.cleared = 0
        elif _agrees(memory, s.kind, s.keys, s.from_keys, window):
            s.cleared += 1
            if s.cleared >= config.suppress_clear_windows:
                memory._suppressions_dirty[name] = True
                continue
        still.append(s)
    memory._suppressions[name] = still
    return kept, kept_fresh


def restore_confirmed(memory, channel, window, stamp, events):
    """After the tracker: confirmed candidates unmatched for ``expire_windows`` whose voxels this
    window sees back at the reference state go REVERTED, in event id order."""
    back = sorted(c.event_id for c in channel.tracker.open
                  if c.confirmed and c.missed >= memory.config.expire_windows
                  and _agrees(memory, _REGION_KINDS[c.published.kind], c.published.keys, _from_keys(c.published), window))
    for event_id in back:
        candidate = _release(channel.tracker, event_id)
        events.append(memory._event(channel, EventState.REVERTED, _shown(candidate), stamp))
        memory._log_event(events[-1])


def _release(tracker, event_id):
    for index, candidate in enumerate(tracker.open):
        if candidate.event_id == event_id:
            return tracker.open.pop(index)
    return None


def operate(memory, action, stamp):
    """``(outcome, events, repeat, repeat_differs)`` for one decoded ``OperatorAction``. Never raises; a
    refused action changes nothing."""
    if action.header.room_id != memory.config.room_id:
        return OperatorOutcome.WRONG_ROOM, [], False, False
    if action.expected_generation != memory.publisher_generation:
        return OperatorOutcome.STALE_GENERATION, [], False, False
    for stored, outcome in memory._actions:
        if stored.action_id == action.action_id:
            return outcome, [], True, stored != action
    events = []
    if action.action in (OperatorActionKind.ACKNOWLEDGE, OperatorActionKind.CONFIRM, OperatorActionKind.DISMISS,
                         OperatorActionKind.ACCEPT):
        outcome = _event_action(memory, action, stamp, events)
    else:
        outcome = OperatorOutcome.UNSUPPORTED
    # A failed write may succeed on a retry with the same id.
    if outcome != OperatorOutcome.STORAGE_FAILED:
        memory._actions.append((action, outcome))
    return outcome, events, False, False


def _event_action(memory, action, stamp, events):
    config = memory.config
    if memory.state != MemoryState.TRACKING:
        return OperatorOutcome.NOT_TRACKING
    event_id = action.event_id
    # A held accept is ACCEPTED here, though the wire still says CONFIRMED.
    if any(held.candidate.event_id == event_id for held in memory._held):
        return OperatorOutcome.APPLIED if action.action == OperatorActionKind.ACKNOWLEDGE else OperatorOutcome.STATE_MISMATCH
    channel, candidate = None, None
    for c in memory.channels.values():
        candidate = next((entry for entry in c.tracker.open if entry.event_id == event_id), None)
        if candidate is not None:
            channel = c
            break
    if candidate is None:
        return OperatorOutcome.UNKNOWN_EVENT
    if action.action == OperatorActionKind.ACKNOWLEDGE:
        return OperatorOutcome.APPLIED
    state = EventState.CONFIRMED if candidate.confirmed else EventState.CANDIDATE
    if action.expected_state != state:
        return OperatorOutcome.STATE_MISMATCH
    # What the operator saw, still supported: matched in the channel's last window, which had
    # points, and still the shape of its published transition.
    fresh = (candidate.missed == 0 and len(channel.recent) > 0 and len(channel.recent[-1]) > 0
             and _same_shape(region_keys(candidate.region), region_keys(candidate.published), config.tolerance,
                             config.stable_min_fraction))

    if action.action == OperatorActionKind.CONFIRM:
        if not fresh:
            return OperatorOutcome.TARGET_CHANGED
        candidate.confirmed = True
        candidate.published = candidate.region
        events.append(memory._event(channel, EventState.CONFIRMED, _shown(candidate), stamp))
        memory._log_event(events[-1])
        return OperatorOutcome.APPLIED

    name = channel.sensor.name
    if action.action == OperatorActionKind.DISMISS:
        added = Suppression(event_id, candidate.kind, candidate.published.keys,
                            _from_keys(candidate.published) if candidate.kind == EventKind.MOVED else _EMPTY_KEYS)
        entries = sorted(memory._suppressions[name] + [added], key=lambda s: s.event_id)
        if config.state_dir is not None:
            try:
                _write_suppressions(memory, name, entries)
            except StorageError as error:
                memory.storage_error = str(error)
                return OperatorOutcome.STORAGE_FAILED
        memory._suppressions[name] = entries
        memory._suppressions_dirty[name] = False
        released = _release(channel.tracker, event_id)
        events.append(memory._event(channel, EventState.REVERTED, _shown(released), stamp))
        memory._log_event(events[-1])
        return OperatorOutcome.APPLIED

    # ACCEPT: compare-and-set on the published map, and only one accept waits for it at a time.
    if not memory._map_started or action.header.map_epoch != memory._map_epoch_sent or memory._held:
        return OperatorOutcome.STALE_EPOCH
    if not fresh:
        return OperatorOutcome.TARGET_CHANGED
    if channel.residual is None or channel.residual > config.learn_max_offset_m:
        return OperatorOutcome.RIG_UNSETTLED
    layer = _absorbed(channel.reference, candidate.published)
    if config.state_dir is not None:
        try:
            header = Header(config.room_id, memory.map_epoch, 0, stamp)
            data = MapBricks(header, Compression.ZLIB, MapKind.SNAPSHOT, 0, config.voxel_size, layer.to_bricks()).to_cdr()
            durable = write_staged(reference_path(config.state_dir, config.room_id, name), data)
            memory.storage_error = None if durable else f"reference of {name} written, directory not synced"
        except (StorageError, ValueError) as error:  # or a layer the codec cannot carry
            memory.storage_error = str(error)
            return OperatorOutcome.STORAGE_FAILED
    released = _release(channel.tracker, event_id)
    channel.stats.erase(region_keys(released.published))
    memory._set_reference(channel, layer)
    memory._held.append(HeldAccept(name, released))
    return OperatorOutcome.APPLIED


def accepted_after_map(memory, stamp):
    """The ACCEPTED transitions whose map message ``map_update`` just published, with its ``map_epoch``."""
    events, still = [], []
    for held in memory._held:
        if held.epoch is None:
            still.append(held)
            continue
        event = memory._event(memory.channels[held.channel], EventState.ACCEPTED, _shown(held.candidate), stamp)
        event = replace(event, header=replace(event.header, map_epoch=held.epoch))
        memory._log_event(event)
        events.append(event)
    memory._held = still
    return events

