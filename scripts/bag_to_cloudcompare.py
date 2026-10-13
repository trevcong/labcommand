#!/usr/bin/env python3
"""Merge /cloud_registered from a LabCommand mcap into a PLY CloudCompare can open."""
from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
from mcap_ros2.reader import read_ros2_messages

# sensor_msgs/PointField datatype
_FLOAT32 = 7
_FLOAT64 = 8


def _xyz_from_cloud(msg) -> np.ndarray:
    fields = {f.name: f for f in msg.fields}
    if not all(k in fields for k in ("x", "y", "z")):
        return np.zeros((0, 3), dtype=np.float32)
    step = int(msg.point_step)
    n = len(msg.data) // step if step else 0
    if n == 0:
        return np.zeros((0, 3), dtype=np.float32)
    blob = np.frombuffer(msg.data, dtype=np.uint8, count=n * step).reshape(n, step)
    # FAST-LIO /cloud_registered: x y z float32 at 0,4,8
    xoff, yoff, zoff = (int(fields[a].offset) for a in ("x", "y", "z"))
    if (
        xoff == 0
        and yoff == 4
        and zoff == 8
        and all(int(fields[a].datatype) == _FLOAT32 for a in ("x", "y", "z"))
    ):
        return blob[:, 0:12].view("<f4").reshape(n, 3).copy()
    out = np.empty((n, 3), dtype=np.float32)
    for i, axis in enumerate(("x", "y", "z")):
        f = fields[axis]
        off = int(f.offset)
        dt = "<f8" if int(f.datatype) == _FLOAT64 else "<f4"
        out[:, i] = np.frombuffer(blob[:, off : off + np.dtype(dt).itemsize].tobytes(), dtype=dt)
    return out


def voxel_downsample(pts: np.ndarray, voxel: float) -> np.ndarray:
    if pts.size == 0 or voxel <= 0:
        return pts
    keys = np.floor(pts / voxel).astype(np.int64)
    # unique rows
    _, idx = np.unique(keys, axis=0, return_index=True)
    return pts[np.sort(idx)]


def write_ply(path: Path, pts: np.ndarray) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    header = (
        "ply\nformat binary_little_endian 1.0\n"
        f"element vertex {len(pts)}\n"
        "property float x\nproperty float y\nproperty float z\n"
        "end_header\n"
    )
    with path.open("wb") as fh:
        fh.write(header.encode("ascii"))
        fh.write(np.ascontiguousarray(pts, dtype=np.float32).tobytes())


def convert(mcap: Path, out: Path, topic: str, voxel: float) -> dict:
    chunks = []
    frames = 0
    for msg in read_ros2_messages(str(mcap), topics=[topic]):
        xyz = _xyz_from_cloud(msg.ros_msg)
        if len(xyz):
            chunks.append(xyz)
            frames += 1
    if not chunks:
        raise SystemExit(f"no points on {topic} in {mcap}")
    pts = np.concatenate(chunks, axis=0)
    raw = int(len(pts))
    pts = pts[np.isfinite(pts).all(axis=1)]
    if voxel > 0:
        pts = voxel_downsample(pts, voxel)
    write_ply(out, pts)
    return {"frames": frames, "raw_points": raw, "kept": int(len(pts)), "out": str(out)}


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("mcap", type=Path)
    ap.add_argument("-o", "--out", type=Path)
    ap.add_argument("--topic", default="/cloud_registered")
    ap.add_argument("--voxel", type=float, default=0.02, help="metres; 0 = keep all")
    args = ap.parse_args()
    out = args.out
    if out is None:
        out = args.mcap.parent.parent / f"{args.mcap.parent.parent.name}.ply"
    info = convert(args.mcap, out, args.topic, args.voxel)
    print(info)


if __name__ == "__main__":
    main()
