"""Streams a rosbag2 mcap recording of the two lidar topics as room_memory_bag records (see
tools/bag.cpp), optionally with synthetic boxes injected into one sensor's frames:

    python bag_stream.py <bag dir or .mcap> [--inject SPEC]... | room_memory_bag key=value ...

Records come in file order, which for rosbag2 is receive (log_time) order; the mcap is read
sequentially, so no index is needed. A frame without an injection active passes through as its
CDR bytes; a frame with one is decoded, edited and sent as float32 x, y, z.

SPEC is sensor:kind:t:x,y,z:side[:x2,y2,z2], times in seconds from the first record and
coordinates in meters in that sensor's frame (cube center, edge length):
  appear     the cube is present from t on;
  disappear  the cube is present until t;
  move       the cube is at x,y,z until t, then at x2,y2,z2.
A present cube is drawn by ray casting: every point whose ray from the sensor origin enters
the cube before its own return is moved to the entry point, plus N(0, 1 cm) range noise. So
the cube occludes what is behind it and is sampled at the sensor's own density, and once it
is gone the real returns behind it come back as free-space evidence.

--loop N plays the recording N times back to back, each pass shifted by the length of the
ones before it plus one frame period, for long runs of a static recording (injection times
count from the first record of the first pass).
"""

import argparse
import os
import pathlib
import re
import struct
import sys

import numpy as np

# The oracle's PointCloud2 decoder: ROOM_MEMORY_PYTHON_PACKAGE when set (CTest sets it; the image keeps
# the oracle in /oracle/room_memory), else spark/room_memory next to this package.
ORACLE = pathlib.Path(__file__).resolve().parents[2] / "room_memory"
sys.path.insert(0, os.environ.get("ROOM_MEMORY_PYTHON_PACKAGE", str(ORACLE)))
from room_memory.core.pointcloud2 import decode_pointcloud2  # noqa: E402

SENSORS = {"/livox/lidar": 0, "/lidar_points": 1}
NAMES = {"mid360": 0, "jt128": 1}
RANGE_NOISE_M = 0.01


class Injection:
    def __init__(self, spec):
        parts = spec.split(":")
        if len(parts) not in (5, 6) or parts[0] not in NAMES or parts[1] not in ("appear", "disappear", "move"):
            raise ValueError(f"bad --inject {spec!r}")
        self.sensor, self.kind, self.t = NAMES[parts[0]], parts[1], float(parts[2])
        self.center = np.array([float(v) for v in parts[3].split(",")], dtype=np.float64)
        self.side = float(parts[4])
        self.moved_to = np.array([float(v) for v in parts[5].split(",")]) if len(parts) == 6 else None
        if (self.kind == "move") != (self.moved_to is not None):
            raise ValueError(f"--inject {spec!r}: move takes a second center, and only move does")

    def center_at(self, t):
        """The cube's center at ``t`` seconds, or None when it is absent."""
        if self.kind == "appear":
            return self.center if t >= self.t else None
        if self.kind == "disappear":
            return self.center if t < self.t else None
        return self.center if t < self.t else self.moved_to


def draw_cube(points, center, side, rng):
    """Moves every return whose ray from the origin enters the cube first to its entry point."""
    p = points.astype(np.float64)
    lo, hi = center - side / 2, center + side / 2
    with np.errstate(divide="ignore", invalid="ignore"):
        t1, t2 = lo / p, hi / p
    near = np.nanmax(np.where(np.isnan(t1), -np.inf, np.minimum(t1, t2)), axis=1)
    far = np.nanmin(np.where(np.isnan(t1), np.inf, np.maximum(t1, t2)), axis=1)
    finite = np.isfinite(p).all(axis=1)
    hit = finite & (near <= far) & (near > 0) & (near < 1)
    # The entry point at `near` of the original range, plus the noise as a share of that range.
    scale = near[hit] + rng.normal(0, RANGE_NOISE_M, hit.sum()) / np.linalg.norm(p[hit], axis=1)
    out = points.copy()
    out[hit] = (p[hit] * scale[:, None]).astype(np.float32)
    return out


def messages(files):
    """(topic, log_time, data) of every message, in file order."""
    from mcap.records import Channel, Message
    from mcap.stream_reader import StreamReader

    for file in files:
        topics = {}
        with open(file, "rb") as handle:
            for record in StreamReader(handle).records:
                if isinstance(record, Channel):
                    topics[record.id] = record.topic
                elif isinstance(record, Message):
                    yield topics[record.channel_id], record.log_time, record.data


def segments(path):
    """The mcap files of a bag directory in recording order (rosbag2 splits number them
    ``<name>_<n>.mcap``, so ``_10`` comes after ``_9``), or the one file given."""
    path = pathlib.Path(path)
    if not path.is_dir():
        return [path]
    index = lambda file: int(match[1]) if (match := re.search(r"_(\d+)\.mcap$", file.name)) else -1
    return sorted(path.glob("*.mcap"), key=lambda file: (index(file), file.name))


def passes(files, loop):
    """(topic, log_time, data) of the sensor messages of ``loop`` passes over the recording, each
    pass shifted to start one frame period (0.1 s) after the previous one ended."""
    shift, end, last = 0, None, None
    for _ in range(loop):
        for topic, log_time, data in messages(files):
            if topic not in SENSORS:
                continue
            if end is not None:
                shift, end = end + 100_000_000 - log_time, None
            yield topic, log_time + shift, data
            last = log_time + shift
        end = last


def records(path, injections, seed=0, loop=1):
    files = segments(path)
    rng = np.random.default_rng(seed)
    first = None
    for topic, log_time, data in passes(files, loop):
        sensor = SENSORS[topic]
        first = log_time if first is None else first
        t = (log_time - first) / 1e9
        active = [(j, j.center_at(t)) for j in injections if j.sensor == sensor]
        active = [(j, c) for j, c in active if c is not None]
        if not active:
            yield 0, sensor, log_time, data
            continue
        points = decode_pointcloud2(data).positions
        for injection, center in active:
            points = draw_cube(points, center, injection.side, rng)
        yield 1, sensor, log_time, np.ascontiguousarray(points, dtype="<f4").tobytes()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("bag")
    parser.add_argument("--inject", action="append", default=[], type=Injection)
    parser.add_argument("--loop", type=int, default=1, help="passes over the recording (default 1)")
    args = parser.parse_args(argv)
    out = sys.stdout.buffer
    try:
        for kind, sensor, stamp, data in records(args.bag, args.inject, loop=args.loop):
            out.write(struct.pack("<BBQI", kind, sensor, stamp, len(data)))
            out.write(data)
    except BrokenPipeError:
        return 1
    out.flush()
    return 0


if __name__ == "__main__":
    sys.exit(main())
