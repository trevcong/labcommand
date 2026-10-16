"""Backend API and version-1 framed-stdio subprocess client (see README)."""
from abc import ABC, abstractmethod
import json
import os
from pathlib import Path
import select
import struct
import subprocess
import sys
import time

import numpy as np
from .generator import HZ, SENSORS

MEMORY_CONFIG = {"sensors": list(SENSORS), "voxel_size": .05, "max_range": 15.,
                 "tolerance": 1, "min_voxels": 6, "min_extent": 2, "window_s": 1.,
                 "reference_windows": 5, "recent_windows": 1, "state_dir": None}


class Backend(ABC):
    pid = None

    @abstractmethod
    def tick(self, tick):
        """Consume one two-sensor tick and return normalized detection dictionaries."""

    @abstractmethod
    def close(self):
        """Release resources; errors must propagate to the runner."""


class PythonBackend(Backend):
    def __init__(self):
        # Resolve this checkout explicitly; no editable installation or ROS required.
        sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "room_memory"))
        from room_memory.memory import MemoryConfig, RoomMemory
        from room_memory.messages import Time
        self.Time = Time
        self.memory = RoomMemory(MemoryConfig(**MEMORY_CONFIG), Time(0, 0), 1)

    def tick(self, tick):
        for sensor in SENSORS:
            self.memory.add_points(sensor, tick.clouds[sensor])
        if not tick.close_window:
            return []
        stamp = self.Time((tick.index + 1) // HZ, 0)
        events = self.memory.close_window(stamp)
        self.memory.status(stamp, 1.)
        return [{"time_s": float(stamp.sec), "sensor": "mid360" if e.sensors == 1 else "jt128",
                 "event_id": e.event_id, "state": e.state.name.lower(), "type": e.kind.name.lower(),
                 "bbox": [list(e.aabb_min), list(e.aabb_max)],
                 "from_centroid": None if e.from_centroid is None else list(e.from_centroid)}
                for e in events]

    def close(self):
        self.memory = None


class BackendUnavailable(Exception):
    """An optional executable is absent; the runner records a clean skip."""


class SubprocessBackend(Backend):
    def __init__(self, binary, stderr_path, timeout=120.):
        self.timeout = timeout
        self.buffer = bytearray()
        path = Path(binary).expanduser().resolve()
        if not path.is_file():
            raise BackendUnavailable(f"C++ binary is missing: {path}")
        if not os.access(path, os.X_OK):
            raise ValueError(f"C++ binary is not executable: {path}")
        self.stderr = open(stderr_path, "wb")
        self.process = None
        try:
            self.process = subprocess.Popen([str(path), "--room-memory-bench-stdio-v1"],
                                            stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                            stderr=self.stderr)
            self.pid = self.process.pid
            os.set_blocking(self.process.stdin.fileno(), False)
            os.set_blocking(self.process.stdout.fileno(), False)
            reply = self._exchange({"op": "init", "version": 1, "config": MEMORY_CONFIG})
            if reply != {"op": "ready", "version": 1}:
                raise ValueError(f"invalid backend handshake: {reply!r}")
        except BaseException:
            self._cleanup()
            raise

    def _exchange(self, header, payload=b""):
        raw = json.dumps(header, allow_nan=False, separators=(",", ":")).encode()
        packet = memoryview(struct.pack("<I", len(raw)) + raw + payload)
        deadline = time.monotonic() + self.timeout
        read_fd, write_fd = self.process.stdout.fileno(), self.process.stdin.fileno()
        while packet:
            remaining = deadline - time.monotonic()
            if remaining <= 0 or not select.select([], [write_fd], [], max(0, remaining))[1]:
                raise TimeoutError("backend request write timed out")
            try:
                count = os.write(write_fd, packet[:65536])
                packet = packet[count:]
            except BlockingIOError:
                continue
        while b"\n" not in self.buffer:
            remaining = deadline - time.monotonic()
            if remaining <= 0 or not select.select([read_fd], [], [], max(0, remaining))[0]:
                raise TimeoutError("backend reply timed out")
            chunk = os.read(read_fd, 65536)
            if not chunk:
                raise RuntimeError("backend closed stdout before replying; inspect backend.stderr.log")
            self.buffer.extend(chunk)
            if len(self.buffer) > 1024 * 1024:
                raise ValueError("backend reply exceeds 1 MiB")
        line, _, rest = self.buffer.partition(b"\n")
        self.buffer = bytearray(rest)
        reply = json.loads(line)
        if not isinstance(reply, dict) or "error" in reply:
            raise ValueError(f"backend error or invalid response: {reply!r}")
        return reply

    def tick(self, tick):
        chunks, frames = [], []
        for sensor in SENSORS:
            points = np.asarray(tick.clouds[sensor], dtype="<f4", order="C")
            chunks.append(points.tobytes())
            frames.append({"sensor": sensor, "points": len(points), "bytes": points.nbytes})
        reply = self._exchange({"op": "tick", "index": tick.index, "time_s": tick.time_s,
                                "end_s": (tick.index + 1) / HZ,
                                "close_window": tick.close_window, "frames": frames}, b"".join(chunks))
        if reply.get("op") != "events" or reply.get("index") != tick.index or not isinstance(reply.get("events"), list):
            raise ValueError("backend tick response has wrong op/index/events")
        events = reply["events"]
        for event in events:
            validate_detection(event, (tick.index + 1) / HZ)
        return events

    def _cleanup(self):
        if self.process is not None:
            if self.process.poll() is None:
                self.process.kill()
            self.process.wait()
            self.process.stdin.close()
            self.process.stdout.close()
        self.stderr.close()

    def close(self):
        try:
            reply = self._exchange({"op": "shutdown"})
            if reply != {"op": "bye"}:
                raise ValueError("invalid shutdown response")
            if self.process.wait(timeout=self.timeout) != 0:
                raise RuntimeError("backend exited unsuccessfully")
        finally:
            self._cleanup()

    def abort(self):
        self._cleanup()


def validate_detection(event, end_s):
    if not isinstance(event, dict):
        raise ValueError("event must be an object")
    if event.get("sensor") not in SENSORS or event.get("type") not in ("appeared", "disappeared", "moved"):
        raise ValueError("event sensor/type is invalid")
    if event.get("state") not in ("candidate", "confirmed", "accepted", "reverted", "expired"):
        raise ValueError("event state is invalid")
    if not isinstance(event.get("event_id"), int) or event["event_id"] < 0:
        raise ValueError("event_id must be a nonnegative integer")
    if event.get("time_s") != end_s:
        raise ValueError("event time_s must equal the emitting tick end_s")
    box = np.asarray(event.get("bbox"), dtype=float)
    if box.shape != (2, 3) or not np.isfinite(box).all() or (box[0] > box[1]).any():
        raise ValueError("event bbox is invalid")
    if event.get("from_centroid") is not None:
        source = np.asarray(event["from_centroid"], dtype=float)
        if source.shape != (3,) or not np.isfinite(source).all():
            raise ValueError("event from_centroid is invalid")
