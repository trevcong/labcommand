"""Stream a PCD v0.7 cloud.

Binary payloads are little-endian and are read with np.fromfile in chunks.
A float32 memory map is not used: the data offset is not always a multiple
of 4. The JT128 map's payload starts at byte 174.
"""

from __future__ import annotations

import hashlib
import os
from typing import Dict, Iterator, List

import numpy as np

from spatio.wallpatch.errors import WallpatchError
from spatio.wallpatch.numbers import (
    parse_field_count,
    parse_pcd_size,
    parse_point_count,
    parse_point_product,
    parse_real,
    reject_xyz,
)

# One chunk is about 2.4 MB of xyz float32. Large enough to be vectorized,
# small enough that several agents can read maps at once.
DEFAULT_CHUNK = 200000


def sha256_file(path):
    """Sha256 of the file bytes, lowercase hex. Streamed, not loaded whole."""
    digest = hashlib.sha256()
    with open(path, "rb") as handle:
        while True:
            block = handle.read(1 << 20)
            if not block:
                break
            digest.update(block)
    return digest.hexdigest()


def read_xyz(path):
    """Load every point. For tests and small clouds. Large maps should iterate."""
    chunks = list(iter_xyz(path))
    if not chunks:
        return np.zeros((0, 3), dtype=np.float64)
    return np.concatenate(chunks, axis=0)


def iter_xyz(path, chunk=DEFAULT_CHUNK):
    # type: (str, int) -> Iterator[np.ndarray]
    """Yield (N, 3) float64 chunks of x, y, z."""
    with open(path, "rb") as handle:
        header = _read_header(handle)
        data = header["data"]
        if data == "ascii":
            for batch in _iter_ascii(handle, header, chunk):
                yield batch
        elif data == "binary":
            _check_binary_size(path, handle.tell(), header)
            for batch in _iter_binary(handle, header, chunk):
                yield batch
        else:
            raise WallpatchError(
                "unsupported PCD DATA type %r (binary_compressed is not read)"
                % data
            )


def write_binary_pcd(path, points):
    """Write a PCD 0.7 binary xyz float32 file. Tests and small fixtures only."""
    pts = np.asarray(points, dtype=np.float64)
    if pts.ndim != 2 or pts.shape[1] != 3:
        raise WallpatchError("points must have shape (N, 3)")
    _write_header_and_payload(path, pts, binary=True)


def write_ascii_pcd(path, points):
    """Write a PCD 0.7 ascii xyz file. Tests only."""
    pts = np.asarray(points, dtype=np.float64)
    if pts.ndim != 2 or pts.shape[1] != 3:
        raise WallpatchError("points must have shape (N, 3)")
    _write_header_and_payload(path, pts, binary=False)


def _write_header_and_payload(path, pts, binary):
    n_points = int(pts.shape[0])
    data_line = "DATA binary\n" if binary else "DATA ascii\n"
    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\n"
        "FIELDS x y z\n"
        "SIZE 4 4 4\n"
        "TYPE F F F\n"
        "COUNT 1 1 1\n"
        "WIDTH %d\n"
        "HEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        "POINTS %d\n"
        "%s"
    ) % (n_points, n_points, data_line)
    parent = os.path.dirname(os.path.abspath(path))
    if parent and not os.path.isdir(parent):
        os.makedirs(parent)
    with open(path, "wb") as handle:
        handle.write(header.encode("ascii"))
        if binary:
            handle.write(np.ascontiguousarray(pts, dtype="<f4").tobytes())
        else:
            for row in pts:
                line = "%.9g %.9g %.9g\n" % (row[0], row[1], row[2])
                handle.write(line.encode("ascii"))


def _read_header(handle):
    # type: (...) -> Dict
    fields = None  # type: List[str]
    sizes = None  # type: List[int]
    types = None  # type: List[str]
    counts = None  # type: List[int]
    points = None  # type: int
    width = None  # type: int
    height = None  # type: int
    data = None  # type: str
    while True:
        line = handle.readline()
        if not line:
            raise WallpatchError("PCD header ended before DATA")
        stripped = line.strip()
        if not stripped or stripped.startswith(b"#"):
            continue
        try:
            text = stripped.decode("ascii")
        except UnicodeError:
            raise WallpatchError("PCD header is not ASCII")
        parts = text.split()
        key = parts[0].upper()
        if key == "FIELDS":
            fields = parts[1:]
        elif key == "SIZE":
            sizes = [parse_pcd_size(item, "SIZE") for item in parts[1:]]
        elif key == "TYPE":
            types = [item.upper() for item in parts[1:]]
        elif key == "COUNT":
            counts = [parse_field_count(item, "COUNT") for item in parts[1:]]
        elif key == "WIDTH":
            width = parse_point_count(parts[1], "WIDTH")
        elif key == "HEIGHT":
            height = parse_point_count(parts[1], "HEIGHT")
        elif key == "POINTS":
            points = parse_point_count(parts[1], "POINTS")
        elif key == "DATA":
            if len(parts) < 2:
                raise WallpatchError("PCD DATA line has no type")
            data = parts[1].lower()
            break
    if fields is None or sizes is None or types is None:
        raise WallpatchError("PCD header is missing FIELDS, SIZE, or TYPE")
    if counts is None:
        counts = [1] * len(fields)
    if not (len(fields) == len(sizes) == len(types) == len(counts)):
        raise WallpatchError("PCD FIELDS, SIZE, TYPE, and COUNT disagree")
    if points is None:
        if width is None or height is None:
            raise WallpatchError("PCD header has no POINTS count")
        points = parse_point_product(width, height)
    for name in ("x", "y", "z"):
        if name not in fields:
            raise WallpatchError("PCD is missing field %s" % name)
    offsets = []  # type: List[int]
    cursor = 0
    for size, count in zip(sizes, counts):
        offsets.append(cursor)
        cursor += int(size) * int(count)
    return {
        "fields": fields,
        "sizes": sizes,
        "types": types,
        "counts": counts,
        "offsets": offsets,
        "stride": cursor,
        "points": int(points),
        "data": data,
    }


def _xyz_index(header, name):
    index = header["fields"].index(name)
    if header["types"][index] != "F" or header["sizes"][index] not in (4, 8):
        raise WallpatchError("PCD field %s must be float32 or float64" % name)
    if header["counts"][index] != 1:
        raise WallpatchError("PCD field %s must have COUNT 1" % name)
    return index


def _check_binary_size(path, offset, header):
    payload = os.path.getsize(path) - offset
    expected = header["points"] * header["stride"]
    if payload < expected:
        raise WallpatchError(
            "PCD binary payload is shorter than the header POINTS "
            "(%d bytes at offset %d, need %d)" % (payload, offset, expected)
        )


def _fast_xyz(header):
    """True when each point is packed little-endian float32 x, y, z and nothing else."""
    return (
        header["fields"] == ["x", "y", "z"]
        and header["sizes"] == [4, 4, 4]
        and header["types"] == ["F", "F", "F"]
        and header["counts"] == [1, 1, 1]
        and header["stride"] == 12
    )


def _iter_binary(handle, header, chunk):
    n_points = header["points"]
    if n_points == 0:
        return
    if _fast_xyz(header):
        remaining = n_points
        while remaining:
            take = min(chunk, remaining)
            buf = np.fromfile(handle, dtype="<f4", count=take * 3)
            if buf.size != take * 3:
                raise WallpatchError(
                    "PCD binary payload is shorter than the header POINTS"
                )
            xyz = buf.reshape(take, 3).astype(np.float64, copy=False)
            reject_xyz(xyz)
            yield xyz
            remaining -= take
        return
    xi = _xyz_index(header, "x")
    yi = _xyz_index(header, "y")
    zi = _xyz_index(header, "z")
    stride = header["stride"]
    remaining = n_points
    while remaining:
        take = min(chunk, remaining)
        buf = np.fromfile(handle, dtype=np.uint8, count=take * stride)
        if buf.size != take * stride:
            raise WallpatchError(
                "PCD binary payload is shorter than the header POINTS"
            )
        raw = buf.reshape(take, stride)
        columns = []
        for index in (xi, yi, zi):
            size = header["sizes"][index]
            start = header["offsets"][index]
            piece = np.ascontiguousarray(raw[:, start : start + size])
            dtype = "<f4" if size == 4 else "<f8"
            columns.append(np.frombuffer(piece.tobytes(), dtype=dtype))
        xyz = np.column_stack(columns).astype(np.float64, copy=False)
        reject_xyz(xyz)
        yield xyz
        remaining -= take


def _ascii_token_offsets(header):
    """Column of each field in an ascii row. COUNT, not the field index."""
    offsets = []
    cursor = 0
    for count in header["counts"]:
        offsets.append(cursor)
        cursor += int(count)
    return offsets


def _iter_ascii(handle, header, chunk):
    token_at = _ascii_token_offsets(header)
    xi = token_at[_xyz_index(header, "x")]
    yi = token_at[_xyz_index(header, "y")]
    zi = token_at[_xyz_index(header, "z")]
    batch = []  # type: List
    seen = 0
    limit = header["points"]
    for line in handle:
        if seen >= limit:
            break
        stripped = line.strip()
        if not stripped or stripped.startswith(b"#"):
            continue
        parts = stripped.split()
        try:
            batch.append(
                (
                    parse_real(parts[xi], "x"),
                    parse_real(parts[yi], "y"),
                    parse_real(parts[zi], "z"),
                )
            )
        except IndexError:
            raise WallpatchError("bad ascii PCD row")
        seen += 1
        if len(batch) >= chunk:
            yield np.asarray(batch, dtype=np.float64)
            batch = []
    if seen != limit:
        raise WallpatchError(
            "PCD ascii payload has %d points, header says %d" % (seen, limit)
        )
    if batch:
        yield np.asarray(batch, dtype=np.float64)
