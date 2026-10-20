"""Read x, y, z from a PCD v0.7 file.

Ascii and little-endian binary are supported. PCL binary_compressed is not:
a compressed cloud is reported as unreadable rather than guessed.
"""

from __future__ import annotations

import io
from pathlib import Path

import numpy as np

_DTYPES = {
    ("F", 4): "<f4",
    ("F", 8): "<f8",
    ("U", 1): "|u1",
    ("U", 2): "<u2",
    ("U", 4): "<u4",
    ("U", 8): "<u8",
    ("I", 1): "|i1",
    ("I", 2): "<i2",
    ("I", 4): "<i4",
    ("I", 8): "<i8",
}


class PcdError(ValueError):
    """The file is not a PCD this checker can read."""


def read_pcd_xyz(path: Path) -> np.ndarray:
    """Return an (N, 3) float64 array of x, y, z. Extra fields are ignored."""
    raw = Path(path).read_bytes()
    lines, mode, offset = _split_header(raw)
    header = _parse_header(lines)
    count = header["points"]
    names = header["names"]
    for axis in ("x", "y", "z"):
        if axis not in names:
            raise PcdError("PCD has no x y z fields")
    if count == 0:
        return np.zeros((0, 3), dtype=np.float64)
    if mode == "ascii":
        xyz = _read_ascii(raw[offset:], names, header["counts"], count)
    elif mode == "binary":
        xyz = _read_binary(raw[offset:], header, count)
    else:
        raise PcdError(
            f"unsupported PCD data mode {mode!r}; binary_compressed is not read"
        )
    if xyz.shape != (count, 3):
        raise PcdError(
            f"PCD POINTS says {count} but {xyz.shape[0]} positions were read"
        )
    return xyz


def _split_header(raw: bytes) -> tuple[list[str], str, int]:
    index = 0
    lines: list[str] = []
    while index < len(raw):
        newline = raw.find(b"\n", index)
        if newline < 0:
            line_bytes = raw[index:]
            next_index = len(raw)
        else:
            line_bytes = raw[index:newline]
            next_index = newline + 1
        line = line_bytes.rstrip(b"\r").decode("ascii", errors="replace").strip()
        if line.upper().startswith("DATA"):
            parts = line.split()
            if len(parts) < 2:
                raise PcdError("DATA line has no mode")
            return lines, parts[1].lower(), next_index
        if line and not line.startswith("#"):
            lines.append(line)
        index = next_index
        if newline < 0:
            break
    raise PcdError("PCD header has no DATA line")


def _parse_header(lines: list[str]) -> dict:
    fields: dict[str, list[str]] = {}
    width = None
    height = None
    points = None
    for line in lines:
        parts = line.split()
        key = parts[0].upper()
        values = parts[1:]
        if key in {"FIELDS", "SIZE", "TYPE", "COUNT"}:
            fields[key] = values
        elif key == "WIDTH" and values:
            width = int(values[0])
        elif key == "HEIGHT" and values:
            height = int(values[0])
        elif key == "POINTS" and values:
            points = int(values[0])
    names = fields.get("FIELDS")
    sizes = fields.get("SIZE")
    types = fields.get("TYPE")
    counts = fields.get("COUNT")
    if not names or not sizes or not types or not counts:
        raise PcdError("PCD header is missing FIELDS, SIZE, TYPE, or COUNT")
    if not (len(names) == len(sizes) == len(types) == len(counts)):
        raise PcdError("PCD FIELDS, SIZE, TYPE, and COUNT differ in length")
    if points is None:
        if width is None or height is None:
            raise PcdError("PCD header has no POINTS, WIDTH, or HEIGHT")
        points = width * height
    return {
        "names": names,
        "sizes": [int(v) for v in sizes],
        "types": types,
        "counts": [int(v) for v in counts],
        "points": points,
    }


def _xyz_columns(names: list[str], counts: list[int]) -> list[int]:
    """Column of each coordinate. COUNT is how many columns a field occupies.

    A field index is not a column index. FIELDS `descriptor x y z` with
    COUNT `3 1 1 1` puts x at column 3, not column 1. x, y, and z themselves
    are one column each. A wider coordinate is rejected rather than guessed.
    """
    cursor = 0
    start: dict[str, int] = {}
    for name, field_count in zip(names, counts):
        if field_count < 1:
            raise PcdError(f"PCD field {name} has COUNT {field_count}")
        if name in ("x", "y", "z") and name not in start:
            if field_count != 1:
                raise PcdError(
                    f"PCD field {name} has COUNT {field_count}; only 1 is supported"
                )
            start[name] = cursor
        cursor += field_count
    missing = [axis for axis in ("x", "y", "z") if axis not in start]
    if missing:
        raise PcdError("PCD has no x y z fields")
    return [start[axis] for axis in ("x", "y", "z")]


def _read_ascii(blob: bytes, names: list[str], counts: list[int], count: int) -> np.ndarray:
    cols = _xyz_columns(names, counts)
    if not blob.strip():
        raise PcdError("ascii PCD has a header count but no data")
    try:
        loaded = np.loadtxt(io.BytesIO(blob), dtype=np.float64, usecols=cols, comments="#")
    except ValueError as exc:
        raise PcdError(f"ascii PCD data could not be read: {exc}") from exc
    if loaded.size == 0:
        return np.zeros((0, 3), dtype=np.float64)
    if loaded.ndim == 1:
        loaded = loaded.reshape(1, 3)
    return np.asarray(loaded, dtype=np.float64)


def _read_binary(blob: bytes, header: dict, count: int) -> np.ndarray:
    dtype_fields = []
    for name, size, typ, field_count in zip(
        header["names"], header["sizes"], header["types"], header["counts"]
    ):
        if field_count != 1:
            raise PcdError(f"PCD field {name} has COUNT {field_count}; only 1 is supported")
        kind = _DTYPES.get((typ.upper(), size))
        if kind is None:
            raise PcdError(f"PCD field {name} has unsupported TYPE {typ} SIZE {size}")
        dtype_fields.append((name, kind))
    dtype = np.dtype(dtype_fields)
    need = count * dtype.itemsize
    if len(blob) < need:
        raise PcdError(f"binary PCD is short: need {need} bytes, have {len(blob)}")
    records = np.frombuffer(blob, dtype=dtype, count=count)
    xyz = np.empty((count, 3), dtype=np.float64)
    for index, axis in enumerate(("x", "y", "z")):
        xyz[:, index] = records[axis]
    return xyz
