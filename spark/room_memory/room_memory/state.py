"""The rig guard's files in ``<state_dir>/<room_id>/``, next to the references
(PLAN-RIGGUARD.md, Design 6). No ROS imports; the C++ twin is ``room_memory_cpp/src/state.cpp``.

- ``registration-<sensor>.bin``: the sensor's registration cloud. Magic ``LCRG``, u32 version 1, the
  sensor name (u32 length, bytes), u64 ``frame_epoch``, u32 count, ``count`` x 3 float32, all little
  endian, then the zlib CRC-32 of everything before it.
- ``rig_state``: ASCII, one item per line, every sensor's pose and baseline, the gravity baseline and
  the name, size and CRC-32 of each registration file, then the CRC-32 of the lines above. It is
  written last, so a crash between files leaves the old ``rig_state`` naming the old files, and a
  file that does not match it fails the load.
- ``prev/<seq>-<name>/``: archived copies, the newest ARCHIVE_KEEP kept.

Each file is written atomically (temp file, fsync, rename). The set is bound by ``rig_state``'s
sizes and CRCs; it is not one transaction (PLAN-RIGGUARD.md, "Later": a transactional manifest).
"""

import math
import os
import pathlib
import re
import shutil
import struct
import zlib
from dataclasses import dataclass
from typing import Dict, Optional, Tuple

import numpy as np

from .guard import Pose

REGISTRATION_MAGIC = b"LCRG"
REGISTRATION_VERSION = 1
STATE_HEADER = "lcrm-rig-state 1"
ARCHIVE_KEEP = 3


def registration_name(sensor):
    return f"registration-{sensor}.bin"


def registration_bytes(sensor, frame_epoch, points):
    name = sensor.encode("utf-8")
    points = np.ascontiguousarray(points, dtype="<f4").reshape(-1, 3)
    body = (REGISTRATION_MAGIC + struct.pack("<I", REGISTRATION_VERSION) + struct.pack("<I", len(name)) + name
            + struct.pack("<QI", frame_epoch, len(points)) + points.tobytes())
    return body + struct.pack("<I", zlib.crc32(body))


def parse_registration(data, what):
    """(sensor, frame_epoch, points). Raises ``ValueError`` on any malformed byte."""
    def bad(reason):
        return ValueError(f"{what}: {reason}")
    if len(data) < 4 + 4 + 4 + 8 + 4 + 4 or data[:4] != REGISTRATION_MAGIC:
        raise bad("not a registration cloud")
    if struct.unpack_from("<I", data, len(data) - 4)[0] != zlib.crc32(data[:-4]):
        raise bad("CRC mismatch")
    if struct.unpack_from("<I", data, 4)[0] != REGISTRATION_VERSION:
        raise bad("unsupported version")
    length = struct.unpack_from("<I", data, 8)[0]
    at = 12 + length
    if at + 12 + 4 > len(data):
        raise bad("truncated")
    try:
        sensor = data[12:at].decode("utf-8")
    except UnicodeError:
        raise bad("sensor name is not UTF-8") from None
    frame_epoch, count = struct.unpack_from("<QI", data, at)
    at += 12
    if at + 12 * count + 4 != len(data):
        raise bad("point count does not match the size")
    points = np.frombuffer(data, dtype="<f4", count=3 * count, offset=at).reshape(-1, 3).astype(np.float32)
    return sensor, frame_epoch, points


@dataclass
class SensorState:
    pose: Pose
    baseline: Optional[float]
    registration: Tuple[str, int, int]  # file name, size, CRC-32


@dataclass
class RigState:
    room_id: str
    voxel_size: float
    frame_epoch: int
    n_realigned: int
    g_base: Optional[Tuple[float, float, float]]
    sensors: Dict[str, SensorState]


def _number(value):
    return "none" if value is None else f"{value:.17g}"


def format_rig_state(state):
    lines = [STATE_HEADER, f"room {state.room_id}", f"voxel_size {state.voxel_size:.17g}",
             f"frame_epoch {state.frame_epoch}", f"n_realigned {state.n_realigned}",
             "g_base " + ("none" if state.g_base is None else " ".join(f"{v:.17g}" for v in state.g_base))]
    for name, sensor in state.sensors.items():
        file, size, crc = sensor.registration
        lines.append(f"sensor {name} pose {sensor.pose.text()} baseline {_number(sensor.baseline)} "
                     f"registration {file} {size} {crc:08x}")
    body = "".join(line + "\n" for line in lines)
    return (body + f"crc {zlib.crc32(body.encode('ascii')):08x}\n").encode("ascii")


def parse_rig_state(data, what):
    """A ``RigState``. Raises ``ValueError`` on a CRC mismatch or any malformed line."""
    def bad(reason):
        return ValueError(f"{what}: {reason}")
    try:
        text = data.decode("ascii")
    except UnicodeError:
        raise bad("not ASCII") from None
    match = re.fullmatch(r"(.*\n)crc ([0-9a-f]{8})\n", text, re.S)
    if not match or int(match[2], 16) != zlib.crc32(match[1].encode("ascii")):
        raise bad("CRC mismatch")
    lines = match[1].splitlines()
    if not lines or lines[0] != STATE_HEADER:
        raise bad("not a rig state")
    fields = {}
    sensors = {}
    try:
        for line in lines[1:]:
            words = line.split(" ")
            if words[0] == "sensor":
                if len(words) != 21 or words[2] != "pose" or words[15] != "baseline" or words[17] != "registration":
                    raise bad(f"malformed sensor line {line!r}")
                baseline = None if words[16] == "none" else float(words[16])
                sensors[words[1]] = SensorState(Pose.from_values(words[3:15]), baseline,
                                                (words[18], int(words[19]), int(words[20], 16)))
            else:
                fields[words[0]] = words[1:]
        g_base = None if fields["g_base"] == ["none"] else tuple(float(v) for v in fields["g_base"])
        state = RigState(fields["room"][0], float(fields["voxel_size"][0]), int(fields["frame_epoch"][0]),
                         int(fields["n_realigned"][0]), g_base, sensors)
    except (KeyError, IndexError, ValueError) as error:
        raise bad(f"malformed ({error})") from None
    if g_base is not None and (len(g_base) != 3 or not all(math.isfinite(v) for v in g_base)):
        raise bad("g_base is not three finite numbers")
    return state


def write_atomic(path, data, make_parents=True):
    """Temp file, fsync, rename, then fsync the directory, so the rename survives a crash."""
    path = pathlib.Path(path)
    if make_parents:
        path.parent.mkdir(parents=True, exist_ok=True)
    scratch = path.with_suffix(path.suffix + ".tmp")
    with open(scratch, "wb") as handle:
        handle.write(data)
        handle.flush()
        os.fsync(handle.fileno())
    os.replace(scratch, path)
    directory = os.open(path.parent, os.O_RDONLY)
    try:
        os.fsync(directory)
    finally:
        os.close(directory)


def archive(room, name, files, remove=()):
    """Copies ``files`` (those that exist) into ``room/prev/<seq>-<name>/``, ``seq`` one past the highest
    archive's (four digits), then deletes ``remove``; keeps the ARCHIVE_KEEP newest archives. Returns
    the archive directory."""
    room = pathlib.Path(room)
    prev = room / "prev"
    prev.mkdir(parents=True, exist_ok=True)
    existing = sorted(path for path in prev.iterdir() if path.is_dir() and re.fullmatch(r"[0-9]{4}-.*", path.name))
    seq = int(existing[-1].name[:4]) + 1 if existing else 1
    target = prev / f"{seq:04d}-{name}"
    target.mkdir()
    try:
        for file in files:
            source = room / file
            if source.exists():
                shutil.copyfile(source, target / file)
                with open(target / file, "rb") as handle:
                    os.fsync(handle.fileno())
    except BaseException:
        # A partial copy is no archive: left behind, every retry would add one and pruning would keep
        # those over complete archives.
        shutil.rmtree(target, ignore_errors=True)
        raise
    for file in remove:
        (room / file).unlink(missing_ok=True)
    for old in (existing + [target])[:-ARCHIVE_KEEP]:
        shutil.rmtree(old)
    return target
