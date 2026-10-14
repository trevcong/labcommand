"""The Python oracle side of ``room_memory_replay``: runs the same script through
``room_memory.memory.RoomMemory`` and prints the same lines (see tools/replay.cpp)."""

import pathlib

import numpy as np

from room_memory.memory import MemoryConfig, RoomMemory
from room_memory.messages import Time

_FLOATS = {"voxel_size", "window_s"}
_INTS = {"tolerance", "min_voxels", "min_extent", "reference_windows", "recent_windows"}


def _config(fields, pair):
    key, value = pair.split("=", 1)
    if key == "room_id":
        fields[key] = value
    elif key == "sensors":
        fields[key] = tuple(name for name in value.split(",") if name)
    elif key == "max_range":
        fields[key] = None if value == "none" else float(value)
    elif key == "state_dir":
        fields[key] = None if value == "none" else value
    elif key in _FLOATS:
        fields[key] = float(value)
    elif key in _INTS:
        fields[key] = int(value)
    else:
        raise ValueError(f"unknown config key {key}")


def run(script):
    """Returns the replay output for ``script`` (text), one line per output."""
    fields = {}
    memory = None
    out = []
    for line in script.splitlines():
        words = line.split()
        if not words or words[0].startswith("#"):
            continue
        command, args = words[0], words[1:]
        if command == "config":
            for pair in args:
                _config(fields, pair)
        elif command == "start":
            try:
                memory = RoomMemory(MemoryConfig(**fields), Time(int(args[0]), int(args[1])))
                out.append(f"init {memory.state.name} {memory.map_epoch}")
            except ValueError:
                out.append("init error")
        elif command == "points":
            memory.add_points(args[0], np.fromfile(args[1], dtype="<f4").reshape(-1, 3))
        elif command in ("cdr", "hex"):
            data = pathlib.Path(args[1]).read_bytes() if command == "cdr" else bytes.fromhex(args[1])
            out.append(f"cdr {1 if memory.add_cdr(args[0], data) else 0}")
        elif command == "close":
            events = memory.close_window(Time(int(args[0]), int(args[1])))
            out.append(f"close {len(events)} {memory.state.name} {memory.map_epoch}")
            out += [f"event {event.to_cdr().hex()}" for event in events]
        elif command == "status":
            status = memory.status(Time(int(args[0]), int(args[1])), float(args[2]))
            out.append(f"status {status.to_cdr().hex()}")
        else:
            raise ValueError(f"unknown command {command}")
    return "".join(line + "\n" for line in out)
