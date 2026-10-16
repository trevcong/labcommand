"""Durable publisher generation (protocol §1.4). No ROS imports.

Every node start takes ``g = max(stored + 1, now_ms)``, writes it to
``<state_dir>/<room_id>/publisher_generation`` (decimal ASCII and a newline, next to ``map_epoch``)
with a temp file, fsync, rename and a directory fsync, and only then publishes. The wall-clock
floor keeps generations rising if the file is lost; the residual is a lost file *and* a clock
that went back past the old value. The allocator fails closed: it never creates ``state_dir``,
never resets a malformed file, and holds ``publisher.lock`` (``flock``) for the life of the
process, so one process publishes per room.
"""

import fcntl
import pathlib
import re
from dataclasses import dataclass
from typing import BinaryIO

from .memory import _room_dir, _write_atomic
from .messages import UINT64_MAX

GENERATION_FILE = "publisher_generation"
LOCK_FILE = "publisher.lock"


class GenerationError(RuntimeError):
    """The node must not start: no durable, exclusive generation could be allocated."""


@dataclass
class Allocation:
    """An allocated generation and the open lock file that keeps it exclusive. Keep it for the life
    of the process: closing ``lock`` (or dropping the last reference to it) releases the ``flock``."""

    generation: int
    path: pathlib.Path
    lock: BinaryIO

    def release(self):
        self.lock.close()


def generation_path(state_dir, room_id):
    """``<state_dir>/<room_id>/publisher_generation``. Same ``room_id`` rule as ``reference_path``."""
    return _room_dir(state_dir, room_id) / GENERATION_FILE


def load_generation(path):
    """The stored generation, 0 if the file does not exist. Raises ``GenerationError`` unless it
    holds one uint64 as digits and a newline."""
    path = pathlib.Path(path)
    if not path.exists():
        return 0
    data = path.read_bytes()
    if not re.fullmatch(rb"[0-9]{1,20}\n", data) or int(data) > UINT64_MAX:
        raise GenerationError(f"{path} does not hold one publisher_generation; refusing to reset it")
    return int(data)


def allocate_generation(state_dir, room_id, now_ms):
    """Allocates this process's generation: ``max(stored + 1, now_ms)``, durable before it returns.
    ``now_ms`` is the wall clock in ms since the Unix epoch (a parameter, for tests). Raises
    ``GenerationError`` when ``state_dir`` is empty or missing, the room name is not a plain file
    name, another process holds the lock, the file is malformed, the value would overflow, or the
    write or fsync fails."""
    if not state_dir:
        raise GenerationError("state_dir is empty: a publisher generation needs durable storage")
    base = pathlib.Path(state_dir).expanduser()
    if not base.is_dir():
        raise GenerationError(f"state_dir {base} does not exist (is the volume mounted?); it is never created here")
    try:
        room = _room_dir(state_dir, room_id)
    except ValueError as error:
        raise GenerationError(str(error)) from None
    try:
        room.mkdir(exist_ok=True)
        lock = open(room / LOCK_FILE, "ab")
    except OSError as error:
        raise GenerationError(f"cannot open {room / LOCK_FILE}: {error}") from None
    try:
        try:
            fcntl.flock(lock.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError as error:
            raise GenerationError(f"another process holds {room / LOCK_FILE} ({error.strerror})") from None
        path = room / GENERATION_FILE
        stored = load_generation(path)
        if stored >= UINT64_MAX:
            raise GenerationError(f"{path} holds {stored}: the next generation would overflow uint64")
        generation = max(stored + 1, int(now_ms))
        if generation > UINT64_MAX:
            raise GenerationError(f"now_ms {now_ms} does not fit a uint64 generation")
        try:
            _write_atomic(path, f"{generation}\n".encode("ascii"), make_parents=False)
        except OSError as error:
            raise GenerationError(f"cannot write {path} durably: {error}") from None
    except BaseException:
        lock.close()
        raise
    return Allocation(generation, path, lock)
