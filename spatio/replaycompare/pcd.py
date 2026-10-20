"""PCD v0.7 ascii and little-endian binary readers.

Binary payloads are memory-mapped as bytes so the DATA offset does not need to
be aligned. Only ``x y z`` are returned; extra fields are skipped.
``binary_compressed`` is rejected.
"""

from __future__ import annotations

from pathlib import Path
from typing import Dict, List, NamedTuple, Optional, Sequence, Union

import numpy as np

PathLike = Union[str, Path]


class _Field(NamedTuple):
    name: str
    size: int
    typ: str
    count: int
    offset: int
    column: int


def read_pcd(path: PathLike) -> np.ndarray:
    """Return xyz as float64 with shape ``(N, 3)``."""
    path = Path(path)
    header, data_offset, kind = _read_header(path)
    layout, stride = _layout(header, path)
    xyz = {field.name: field for field in layout if field.name in ("x", "y", "z")}
    missing = [name for name in ("x", "y", "z") if name not in xyz]
    if missing:
        raise ValueError(f"{path}: PCD is missing fields {missing}")
    for name in ("x", "y", "z"):
        field = xyz[name]
        if field.count != 1:
            raise ValueError(f"{path}: field {name} has COUNT {field.count}, expected 1")
    count = _point_count(header, path)
    if kind == "ascii":
        columns = {name: xyz[name].column for name in ("x", "y", "z")}
        return _read_ascii(path, data_offset, columns, count)
    if kind == "binary":
        for name in ("x", "y", "z"):
            _require_float_field(path, xyz[name])
        return _read_binary(path, data_offset, xyz, stride, count)
    raise ValueError(f"{path}: DATA {kind} is not supported (ascii and binary only)")


def write_pcd(path: PathLike, points: np.ndarray, binary: bool = True, double: bool = False) -> None:
    """Write an xyz PCD. ``points`` has shape ``(N, 3)``.

    Binary files are little-endian float32, or float64 when ``double`` is set.
    """
    dtype = np.float64 if double else np.float32
    cloud = np.ascontiguousarray(np.asarray(points, dtype=dtype).reshape(-1, 3))
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    kind = "binary" if binary else "ascii"
    header = _header_text(len(cloud), kind, double=double)
    if binary:
        with path.open("wb") as handle:
            handle.write(header.encode("ascii"))
            handle.write(cloud.tobytes(order="C"))
        return
    lines = [header.rstrip("\n")]
    for row in cloud:
        lines.append(" ".join(format(float(value), ".17g" if double else ".9g") for value in row))
    path.write_text("\n".join(lines) + "\n", encoding="ascii")


def voxel_downsample(points: np.ndarray, voxel: float) -> np.ndarray:
    """Return one centroid per voxel, in deterministic voxel-key order.

    The voxel index is ``floor(coord / voxel)``. Points are sorted by that
    index and then by x, y, z before the sum, so the same set of points always
    produces the same centroids. Non-finite points are dropped.
    """
    if voxel <= 0:
        raise ValueError(f"voxel must be > 0, got {voxel}")
    cloud = np.asarray(points, dtype=np.float64)
    if cloud.size == 0:
        return np.zeros((0, 3), dtype=np.float64)
    cloud = np.ascontiguousarray(cloud.reshape(-1, 3))
    cloud = cloud[np.isfinite(cloud).all(axis=1)]
    if len(cloud) == 0:
        return np.zeros((0, 3), dtype=np.float64)
    keys = np.floor(cloud / float(voxel)).astype(np.int64)
    order = np.lexsort(
        (
            cloud[:, 2],
            cloud[:, 1],
            cloud[:, 0],
            keys[:, 2],
            keys[:, 1],
            keys[:, 0],
        )
    )
    cloud = cloud[order]
    keys = keys[order]
    if len(cloud) == 1:
        return cloud.copy()
    changed = np.any(keys[1:] != keys[:-1], axis=1)
    starts = np.concatenate(
        (np.array([0], dtype=np.int64), np.flatnonzero(changed) + 1)
    )
    sums = np.add.reduceat(cloud, starts, axis=0)
    counts = np.diff(np.append(starts, len(cloud))).astype(np.float64)
    return sums / counts[:, None]


def _read_header(path: Path) -> tuple:
    fields: Dict[str, List[str]] = {}
    with path.open("rb") as handle:
        while True:
            line = handle.readline()
            if not line:
                raise ValueError(f"{path}: missing DATA line")
            text = line.decode("ascii", errors="replace").strip()
            if not text or text.startswith("#"):
                continue
            parts = text.split()
            if parts[0] == "DATA":
                if len(parts) != 2:
                    raise ValueError(f"{path}: DATA line must be 'DATA ascii' or 'DATA binary'")
                return fields, handle.tell(), parts[1].lower()
            fields[parts[0]] = parts[1:]
    raise ValueError(f"{path}: missing DATA line")


def _expand(values: Optional[Sequence[str]], count: int, label: str, path: Path, default: Optional[str] = None) -> List[str]:
    if not values:
        if default is None:
            raise ValueError(f"{path}: PCD has no {label} line")
        return [default] * count
    if len(values) == 1 and count != 1:
        return list(values) * count
    if len(values) != count:
        raise ValueError(f"{path}: {label} has {len(values)} values for {count} fields")
    return list(values)


def _layout(header: Dict[str, List[str]], path: Path) -> tuple:
    names = header.get("FIELDS") or []
    if not names:
        raise ValueError(f"{path}: PCD has no FIELDS line")
    sizes = _expand(header.get("SIZE"), len(names), "SIZE", path)
    types = _expand(header.get("TYPE"), len(names), "TYPE", path)
    counts = _expand(header.get("COUNT"), len(names), "COUNT", path, default="1")
    layout: List[_Field] = []
    offset = 0
    column = 0
    for name, size_text, typ, count_text in zip(names, sizes, types, counts):
        size = int(size_text)
        count = int(count_text)
        if size <= 0 or count <= 0:
            raise ValueError(f"{path}: field {name} has SIZE {size} COUNT {count}")
        layout.append(_Field(name, size, typ, count, offset, column))
        offset += size * count
        column += count
    return layout, offset


def _point_count(header: Dict[str, List[str]], path: Path) -> int:
    if header.get("POINTS"):
        return int(header["POINTS"][0])
    if header.get("WIDTH"):
        width = int(header["WIDTH"][0])
        height = int(header["HEIGHT"][0]) if header.get("HEIGHT") else 1
        return width * height
    raise ValueError(f"{path}: PCD has no POINTS or WIDTH")


def _require_float_field(path: Path, field: _Field) -> None:
    if field.typ.upper() != "F" or field.size not in (4, 8):
        raise ValueError(
            f"{path}: field {field.name} must be float32 or float64, "
            f"found TYPE {field.typ} SIZE {field.size}"
        )


def _read_ascii(path: Path, data_offset: int, columns: Dict[str, int], count: int) -> np.ndarray:
    if count == 0:
        return np.zeros((0, 3), dtype=np.float64)
    out = np.empty((count, 3), dtype=np.float64)
    seen = 0
    with path.open("rb") as handle:
        handle.seek(data_offset)
        for line in handle:
            text = line.decode("ascii", errors="replace").strip()
            if not text or text.startswith("#"):
                continue
            parts = text.split()
            try:
                out[seen, 0] = float(parts[columns["x"]])
                out[seen, 1] = float(parts[columns["y"]])
                out[seen, 2] = float(parts[columns["z"]])
            except (IndexError, ValueError) as exc:
                raise ValueError(f"{path}: bad ascii point at index {seen}: {exc}") from exc
            seen += 1
            if seen == count:
                break
    if seen != count:
        raise ValueError(f"{path}: expected {count} ascii points, read {seen}")
    return out


def _read_binary(
    path: Path,
    data_offset: int,
    xyz: Dict[str, _Field],
    stride: int,
    count: int,
) -> np.ndarray:
    if count == 0:
        return np.zeros((0, 3), dtype=np.float64)
    nbytes = count * stride
    file_size = path.stat().st_size
    have = file_size - data_offset
    if have < nbytes:
        raise ValueError(f"{path}: binary payload is {have} bytes, expected {nbytes}")
    mapped = np.memmap(path, dtype=np.uint8, mode="r", offset=data_offset, shape=(nbytes,))
    try:
        raw = np.array(mapped, dtype=np.uint8, copy=True).reshape(count, stride)
    finally:
        view = getattr(mapped, "_mmap", None)
        if view is not None:
            view.close()
        del mapped
    out = np.empty((count, 3), dtype=np.float64)
    for axis, name in enumerate(("x", "y", "z")):
        field = xyz[name]
        width = field.size
        column = np.ascontiguousarray(raw[:, field.offset : field.offset + width])
        dtype = "<f4" if width == 4 else "<f8"
        out[:, axis] = column.view(dtype).reshape(count)
    return out


def _header_text(count: int, kind: str, double: bool = False) -> str:
    size = "8 8 8" if double else "4 4 4"
    return (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\n"
        "FIELDS x y z\n"
        f"SIZE {size}\n"
        "TYPE F F F\n"
        "COUNT 1 1 1\n"
        f"WIDTH {count}\n"
        "HEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {count}\n"
        f"DATA {kind}\n"
    )
