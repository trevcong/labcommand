"""The Python oracle side of ``room_memory_replay``: runs the same script through
``room_memory.memory.RoomMemory`` and prints the same lines (see tools/replay.cpp).

``start <sec> <nanosec> [generation]``: the generation defaults to the start time in ms (at least
1), the allocator's wall-clock floor, so successive starts take rising generations. ``sync``
prints ``sync <n>``, the answer's messages as ``data`` lines, then ``end <SyncEnd hex>`` (``end
none`` for another room)."""

import dataclasses
import pathlib

import numpy as np

from room_memory.memory import MemoryConfig, RoomMemory
from room_memory.messages import CDRError, RequestSync, Time

_FLOATS = {"voxel_size", "window_s", "occupied_min_ratio", "free_max_ratio", "learn_max_offset_m",
           "fresh_min_fraction", "stable_min_fraction", "activity_radius_m", "delta_period_s",
           "keyframe_period_s", "keyframe_delta_ratio", "imu_shock_frac", "imu_gyro_dps", "imu_quiet_s",
           "tilt_trigger_deg", "pose_trigger_m", "guard_check_period_s", "check_max_s", "move_min_m", "move_min_deg",
           "align_max_m", "align_max_deg", "align_inlier_ratio", "align_max_rms_m", "observe_min_eig",
           "gross_inlier_ratio", "relearn_after_s", "reg_voxel_m", "src_voxel_m", "icp_max_range_m"}
_INTS = {"tolerance", "min_voxels", "min_extent", "reference_windows", "recent_windows", "occupied_min_hits",
         "free_min_seen", "candidate_windows", "expire_windows", "learn_windows", "fresh_min_voxels",
         "align_stable_windows", "reg_max_points", "src_max_points"}
_BOOLS = {"rig_guard", "auto_relearn"}


def _config(fields, pair):
    key, value = pair.split("=", 1)
    if key in ("room_id", "map_sensor", "imu_topic"):
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
    elif key in _BOOLS:
        fields[key] = {"true": True, "false": False}[value]
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
            sec, nanosec = int(args[0]), int(args[1])
            try:
                generation = int(args[2]) if len(args) > 2 else max(1, sec * 1000 + nanosec // 1_000_000)
                memory = RoomMemory(MemoryConfig(**fields), Time(sec, nanosec), generation)
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
        elif command == "defaults":
            for field in dataclasses.fields(MemoryConfig):
                value = getattr(MemoryConfig(), field.name)
                text = (",".join(value) if isinstance(value, tuple) else "none" if value is None
                        else str(value).lower() if isinstance(value, bool) else str(value))
                out.append(f"default {field.name} {text}")
        elif command == "activity":
            out.append(f"activity {memory.activity(Time(int(args[0]), int(args[1]))).to_cdr().hex()}")
        elif command == "map":
            message = memory.map_update(Time(int(args[0]), int(args[1])))
            out.append(f"map {'none' if message is None else message.hex()}")
        elif command == "sync":
            try:
                answer = memory.sync(RequestSync.from_cdr(bytes.fromhex(args[2])), Time(int(args[0]), int(args[1])))
            except CDRError:
                out.append("sync invalid")
            else:
                messages, end = answer
                out.append(f"sync {len(messages)}")
                out += [f"data {message.hex()}" for message in messages]
                out.append(f"end {'none' if end is None else end.to_cdr().hex()}")
        elif command == "imu":
            ok = memory.add_imu(bytes.fromhex(args[2]), Time(int(args[0]), int(args[1])))
            out.append(f"imu {1 if ok else 0}")
        elif command == "rig":
            message = memory.rig_pose(Time(int(args[0]), int(args[1])))
            out.append(f"rig {'none' if message is None else message.to_cdr().hex()}")
        else:
            raise ValueError(f"unknown command {command}")
    return "".join(line + "\n" for line in out)
