"""PCD v0.7 reader and writer for ascii, binary, and binary_compressed.

Binary clouds are returned as views of a read-only memory map. Compressed
clouds follow PCL: a little-endian uint32 compressed size, a uint32
uncompressed size, then an liblzf payload laid out field-by-field. Fields
named ``_`` or starting with ``_`` are padding. They occupy bytes in ascii
and binary records and are omitted from the compressed payload.
"""

from __future__ import annotations

import math
import os
import struct
from dataclasses import dataclass
from typing import Dict, List, Sequence, Tuple

import numpy as np

_DATA_KINDS = ("ascii", "binary", "binary_compressed")
_DTYPE = {
    ("F", 4): "<f4",
    ("F", 8): "<f8",
    ("I", 1): "<i1",
    ("I", 2): "<i2",
    ("I", 4): "<i4",
    ("I", 8): "<i8",
    ("U", 1): "<u1",
    ("U", 2): "<u2",
    ("U", 4): "<u4",
    ("U", 8): "<u8",
}
_FROM_NUMPY = {
    np.dtype("<f4"): (4, "F"),
    np.dtype("<f8"): (8, "F"),
    np.dtype("<i1"): (1, "I"),
    np.dtype("<i2"): (2, "I"),
    np.dtype("<i4"): (4, "I"),
    np.dtype("<i8"): (8, "I"),
    np.dtype("<u1"): (1, "U"),
    np.dtype("<u2"): (2, "U"),
    np.dtype("<u4"): (4, "U"),
    np.dtype("<u8"): (8, "U"),
}


class PcdFormatError(ValueError):
    """The PCD header or body does not match PCD v0.7."""


@dataclass(frozen=True)
class PcdField:
    """One column of a PCD header, in file order."""

    name: str
    size: int
    type: str
    count: int
    offset: int
    dtype: str

    @property
    def nbytes(self) -> int:
        return self.size * self.count

    @property
    def padding(self) -> bool:
        return self.name == "_" or self.name.startswith("_")


@dataclass(frozen=True)
class PcdHeader:
    """The ASCII header of a PCD v0.7 file.

    ``data_offset`` is the byte index of the first payload byte.
    ``point_step`` is the interleaved record size, including padding.
    ``viewpoint`` is ``tx ty tz qw qx qy qz`` as stored in the file.
    """

    version: str
    fields: Tuple[str, ...]
    size: Tuple[int, ...]
    type: Tuple[str, ...]
    count: Tuple[int, ...]
    width: int
    height: int
    viewpoint: Tuple[float, float, float, float, float, float, float]
    points: int
    data: str
    data_offset: int
    point_step: int


def read_pcd_header(path: str) -> PcdHeader:
    """Read only the header. The point payload is not loaded."""

    fields = None
    sizes = None
    types = None
    counts = None
    width = None
    height = None
    points = None
    version = ""
    viewpoint = (0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0)
    data = None
    offset = 0
    with open(path, "rb") as handle:
        while True:
            raw = handle.readline()
            if raw == b"":
                raise PcdFormatError("missing DATA line in %s" % path)
            offset += len(raw)
            try:
                text = raw.decode("ascii")
            except UnicodeDecodeError as exc:
                raise PcdFormatError("PCD header is not ASCII in %s" % path) from exc
            # Keep the byte count above; parsing uses the stripped text.
            text = text.strip()
            if not text or text.startswith("#"):
                continue
            parts = text.split()
            key = parts[0].upper()
            if key == "VERSION":
                if len(parts) != 2:
                    raise PcdFormatError("VERSION needs one value")
                version = parts[1]
            elif key in ("FIELDS", "COLUMNS"):
                if len(parts) < 2:
                    raise PcdFormatError("FIELDS has no names")
                fields = tuple(parts[1:])
            elif key == "SIZE":
                sizes = tuple(_positive_ints(parts[1:], "SIZE"))
            elif key == "TYPE":
                types = tuple(_type_codes(parts[1:]))
            elif key == "COUNT":
                counts = tuple(_positive_ints(parts[1:], "COUNT"))
            elif key == "WIDTH":
                width = _one_nonneg(parts, "WIDTH")
            elif key == "HEIGHT":
                height = _one_nonneg(parts, "HEIGHT")
            elif key == "VIEWPOINT":
                if len(parts) != 8:
                    raise PcdFormatError("VIEWPOINT needs tx ty tz qw qx qy qz")
                try:
                    viewpoint = tuple(float(p) for p in parts[1:])  # type: ignore[assignment]
                except ValueError as exc:
                    raise PcdFormatError("VIEWPOINT is not numeric") from exc
            elif key == "POINTS":
                points = _one_nonneg(parts, "POINTS")
            elif key == "DATA":
                if len(parts) != 2 or parts[1] not in _DATA_KINDS:
                    raise PcdFormatError(
                        "unknown DATA type %r" % (" ".join(parts[1:]),)
                    )
                data = parts[1]
                break
            else:
                raise PcdFormatError("unknown header line %r" % parts[0])
    if fields is None or sizes is None or types is None:
        raise PcdFormatError("header is missing FIELDS, SIZE, or TYPE")
    if counts is None:
        counts = tuple(1 for _ in fields)
    if width is None or height is None or points is None or data is None:
        raise PcdFormatError("header is missing WIDTH, HEIGHT, POINTS, or DATA")
    if not (len(fields) == len(sizes) == len(types) == len(counts)):
        raise PcdFormatError(
            "FIELDS/SIZE/TYPE/COUNT lengths differ: %d %d %d %d"
            % (len(fields), len(sizes), len(types), len(counts))
        )
    for size, type_code in zip(sizes, types):
        if (type_code, size) not in _DTYPE:
            raise PcdFormatError("unsupported TYPE/SIZE %s/%d" % (type_code, size))
    if width * height != points:
        raise PcdFormatError(
            "WIDTH (%d) x HEIGHT (%d) != POINTS (%d)" % (width, height, points)
        )
    point_step = 0
    for size, count in zip(sizes, counts):
        point_step += size * count
    return PcdHeader(
        version=version,
        fields=fields,
        size=sizes,
        type=types,
        count=counts,
        width=width,
        height=height,
        viewpoint=viewpoint,  # type: ignore[arg-type]
        points=points,
        data=data,
        data_offset=offset,
        point_step=point_step,
    )


def read_pcd(path: str, fields: Sequence[str] = ("x", "y", "z")) -> Dict[str, np.ndarray]:
    """Read the requested fields.

    Binary results are views onto a read-only memory map of ``path`` (no
    full copy of the point records). Ascii and compressed results are
    newly allocated arrays. A field with COUNT > 1 has shape (N, COUNT).
    """

    header = read_pcd_header(path)
    layout = _layout(header)
    wanted = tuple(fields)
    by_name = {}
    for field in layout:
        # The first field of a repeated name wins. Padding names are included.
        if field.name not in by_name:
            by_name[field.name] = field
    missing = [name for name in wanted if name not in by_name]
    if missing:
        raise PcdFormatError("fields not in %s: %s" % (path, ", ".join(missing)))
    if header.data == "ascii":
        columns = _read_ascii(path, header, layout)
    elif header.data == "binary":
        columns = _read_binary(path, header, layout)
    elif header.data == "binary_compressed":
        columns = _read_compressed(path, header, layout)
    else:
        raise PcdFormatError("unknown DATA type %r" % header.data)
    return {name: columns[name] for name in wanted}


def write_pcd(path: str, arrays: Dict[str, np.ndarray], data: str = "ascii") -> None:
    """Write ``arrays`` in field order. Values are little-endian.

    ``data`` is ``ascii``, ``binary``, or ``binary_compressed``. The
    compressed encoder emits literal LZF runs only. A 1-D array has COUNT 1.
    A 2-D array of shape (N, C) has COUNT C. The header is padded so the
    payload starts on an 8-byte boundary.
    """

    if data not in _DATA_KINDS:
        raise ValueError("data must be ascii, binary, or binary_compressed")
    if not arrays:
        raise ValueError("arrays must contain at least one field")
    prepared = []
    n = None
    for name, value in arrays.items():
        arr = np.asarray(value)
        if arr.ndim == 1:
            count = 1
        elif arr.ndim == 2:
            count = int(arr.shape[1])
            if count < 1:
                raise ValueError("field %s has no columns" % name)
        else:
            raise ValueError("field %s must be 1-D or 2-D" % name)
        if n is None:
            n = int(arr.shape[0])
        elif int(arr.shape[0]) != n:
            raise ValueError("field %s has length %d, expected %d" % (name, arr.shape[0], n))
        base = np.dtype(arr.dtype).newbyteorder("<")
        # newbyteorder keeps the kind; look up the little-endian form.
        le = np.dtype(base.str if base.byteorder != "=" else "<" + base.char)
        # np.dtype('<f4').str is '<f4'. Normalize via kind/itemsize.
        key = (le.kind.upper() if le.kind != "f" else "F", le.itemsize)
        # kind is 'f', 'i', 'u'. Map to PCD letters.
        kind_map = {"f": "F", "i": "I", "u": "U"}
        if le.kind not in kind_map:
            raise ValueError("field %s has unsupported dtype %s" % (name, arr.dtype))
        type_code = kind_map[le.kind]
        size = int(le.itemsize)
        if (type_code, size) not in _DTYPE:
            raise ValueError("field %s has unsupported dtype %s" % (name, arr.dtype))
        le_dtype = np.dtype(_DTYPE[(type_code, size)])
        cast = np.ascontiguousarray(arr, dtype=le_dtype)
        prepared.append((str(name), size, type_code, count, cast))
    assert n is not None
    names = [item[0] for item in prepared]
    sizes = [item[1] for item in prepared]
    types = [item[2] for item in prepared]
    counts = [item[3] for item in prepared]
    header = _header_bytes(names, sizes, types, counts, n, data)
    payload = _payload_bytes(prepared, n, data)
    with open(path, "wb") as handle:
        handle.write(header)
        handle.write(payload)


def lzf_decompress(data: bytes, out_len: int) -> bytes:
    """Decompress one liblzf stream into exactly ``out_len`` bytes.

    This is the liblzf decoder: a control byte below 32 introduces that
    many-plus-one literal bytes; any other control byte is a back-reference.
    Overlapping copies are applied one byte at a time. The encoder in this
    module writes literals only, but readers of real PCL files need the
    back-references.
    """

    if out_len < 0:
        raise PcdFormatError("negative uncompressed size")
    src = bytes(data)
    out = bytearray(out_len)
    ip = 0
    op = 0
    n = len(src)
    try:
        while ip < n:
            ctrl = src[ip]
            ip += 1
            if ctrl < 32:
                length = ctrl + 1
                if op + length > out_len or ip + length > n:
                    raise PcdFormatError("truncated LZF literal")
                out[op : op + length] = src[ip : ip + length]
                op += length
                ip += length
            else:
                length = ctrl >> 5
                if ip >= n:
                    raise PcdFormatError("truncated LZF back-reference")
                if length == 7:
                    length += src[ip]
                    ip += 1
                    if ip >= n:
                        raise PcdFormatError("truncated LZF back-reference")
                ref = op - ((ctrl & 0x1F) << 8) - 1 - src[ip]
                ip += 1
                length += 2
                if ref < 0 or op + length > out_len:
                    raise PcdFormatError("LZF back-reference out of range")
                for _ in range(length):
                    out[op] = out[ref]
                    op += 1
                    ref += 1
    except IndexError as exc:
        raise PcdFormatError("truncated LZF stream") from exc
    if op != out_len:
        raise PcdFormatError(
            "LZF decompressed %d bytes, header says %d" % (op, out_len)
        )
    return bytes(out)


def lzf_compress_literals(data: bytes) -> bytes:
    """Encode ``data`` as literal-only LZF. The result is a valid stream.

    Each run is at most 32 bytes. The compressed form is slightly larger
    than the input and needs no hash table.
    """

    src = bytes(data)
    out = bytearray()
    ip = 0
    n = len(src)
    while ip < n:
        run = min(32, n - ip)
        out.append(run - 1)
        out.extend(src[ip : ip + run])
        ip += run
    return bytes(out)


def _layout(header: PcdHeader) -> Tuple[PcdField, ...]:
    fields = []
    offset = 0
    for name, size, type_code, count in zip(
        header.fields, header.size, header.type, header.count
    ):
        fields.append(
            PcdField(
                name=name,
                size=size,
                type=type_code,
                count=count,
                offset=offset,
                dtype=_DTYPE[(type_code, size)],
            )
        )
        offset += size * count
    return tuple(fields)


def _read_ascii(
    path: str, header: PcdHeader, layout: Tuple[PcdField, ...]
) -> Dict[str, np.ndarray]:
    columns = _empty_columns(layout, header.points)
    tokens_per_point = sum(field.count for field in layout)
    found = 0
    with open(path, "rb") as handle:
        handle.seek(header.data_offset)
        for raw in handle:
            if found >= header.points:
                break
            try:
                text = raw.decode("ascii")
            except UnicodeDecodeError as exc:
                raise PcdFormatError("ASCII point is not ASCII") from exc
            text = text.strip()
            if not text or text.startswith("#"):
                continue
            parts = text.split()
            if len(parts) != tokens_per_point:
                raise PcdFormatError(
                    "ASCII point %d has %d values, expected %d"
                    % (found + 1, len(parts), tokens_per_point)
                )
            cursor = 0
            for index, field in enumerate(layout):
                chunk = parts[cursor : cursor + field.count]
                cursor += field.count
                try:
                    values = [float(tok) if field.type == "F" else int(tok) for tok in chunk]
                except ValueError as exc:
                    raise PcdFormatError(
                        "ASCII point %d field %s is not numeric" % (found + 1, field.name)
                    ) from exc
                _store(columns, index, field, found, values)
            found += 1
    if found != header.points:
        raise PcdFormatError(
            "truncated ASCII cloud: read %d of %d points" % (found, header.points)
        )
    return _finalize(columns, layout)


def _read_binary(
    path: str, header: PcdHeader, layout: Tuple[PcdField, ...]
) -> Dict[str, np.ndarray]:
    expected = header.data_offset + header.points * header.point_step
    file_size = os.path.getsize(path)
    if file_size < expected:
        raise PcdFormatError(
            "truncated binary cloud: file has %d bytes, need %d" % (file_size, expected)
        )
    # Map the whole file. The structured view is offset into that map, so
    # the returned columns keep the memmap alive and do not copy the records.
    mapped = np.memmap(path, dtype=np.uint8, mode="r")
    if header.points == 0:
        return _finalize(_empty_columns(layout, 0), layout)
    dtype = np.dtype(
        {
            "names": ["f%d" % i for i in range(len(layout))],
            "formats": [
                field.dtype if field.count == 1 else (field.dtype, (field.count,))
                for field in layout
            ],
            "offsets": [field.offset for field in layout],
            "itemsize": header.point_step,
        }
    )
    cloud = np.ndarray(
        shape=(header.points,), dtype=dtype, buffer=mapped, offset=header.data_offset
    )
    columns = {}
    for index, field in enumerate(layout):
        # A field view, not a copy. Its base chain reaches the memmap.
        columns[index] = cloud["f%d" % index]
    return _finalize(columns, layout)


def _read_compressed(
    path: str, header: PcdHeader, layout: Tuple[PcdField, ...]
) -> Dict[str, np.ndarray]:
    stored = [field for field in layout if not field.padding]
    uncompressed_need = header.points * sum(field.nbytes for field in stored)
    file_size = os.path.getsize(path)
    if file_size < header.data_offset + 8:
        raise PcdFormatError("truncated binary_compressed header")
    with open(path, "rb") as handle:
        handle.seek(header.data_offset)
        prefix = handle.read(8)
        if len(prefix) != 8:
            raise PcdFormatError("truncated binary_compressed size header")
        compressed_size, uncompressed_size = struct.unpack("<II", prefix)
        if header.data_offset + 8 + compressed_size > file_size:
            raise PcdFormatError(
                "truncated binary_compressed payload: need %d bytes, file has %d"
                % (header.data_offset + 8 + compressed_size, file_size)
            )
        blob = handle.read(compressed_size)
    if len(blob) != compressed_size:
        raise PcdFormatError("truncated binary_compressed payload")
    if uncompressed_size != uncompressed_need:
        raise PcdFormatError(
            "compressed uncompressed size %d != field payload %d"
            % (uncompressed_size, uncompressed_need)
        )
    if header.points == 0:
        return _finalize(_empty_columns(layout, 0), layout)
    raw = lzf_decompress(blob, uncompressed_size)
    columns = _empty_columns(layout, header.points)
    cursor = 0
    for index, field in enumerate(layout):
        if field.padding:
            # PCL does not store padding in the compressed payload.
            columns[index].fill(0)
            continue
        nbytes = field.nbytes * header.points
        chunk = raw[cursor : cursor + nbytes]
        cursor += nbytes
        values = np.frombuffer(chunk, dtype=np.dtype(field.dtype))
        if field.count == 1:
            columns[index][:] = values
        else:
            columns[index][:] = values.reshape(header.points, field.count)
    return _finalize(columns, layout)


def _empty_columns(layout: Tuple[PcdField, ...], n: int) -> Dict[int, np.ndarray]:
    columns = {}
    for index, field in enumerate(layout):
        shape = (n,) if field.count == 1 else (n, field.count)
        columns[index] = np.empty(shape, dtype=np.dtype(field.dtype))
    return columns


def _store(
    columns: Dict[int, np.ndarray], index: int, field: PcdField, row: int, values: List[float]
) -> None:
    if field.count == 1:
        columns[index][row] = values[0]
    else:
        columns[index][row, :] = values


def _finalize(
    columns: Dict[int, np.ndarray], layout: Tuple[PcdField, ...]
) -> Dict[str, np.ndarray]:
    out = {}
    for index, field in enumerate(layout):
        if field.name not in out:
            out[field.name] = columns[index]
    return out


def _header_bytes(
    names: Sequence[str],
    sizes: Sequence[int],
    types: Sequence[str],
    counts: Sequence[int],
    n: int,
    data: str,
) -> bytes:
    lines = [
        "# .PCD v0.7 - Point Cloud Data file format",
        "VERSION 0.7",
        "FIELDS " + " ".join(names),
        "SIZE " + " ".join(str(v) for v in sizes),
        "TYPE " + " ".join(types),
        "COUNT " + " ".join(str(v) for v in counts),
        "WIDTH %d" % n,
        "HEIGHT 1",
        "VIEWPOINT 0 0 0 1 0 0 0",
        "POINTS %d" % n,
    ]
    body = ("\n".join(lines) + "\n").encode("ascii")
    data_line = ("DATA %s\n" % data).encode("ascii")
    # Pad with a comment so the payload is 8-byte aligned. PCL skips comments.
    base = len(body) + len(data_line)
    need = (8 - (base % 8)) % 8
    if need == 0:
        pad = b""
    else:
        plen = need if need >= 2 else need + 8
        pad = b"#" + b" " * (plen - 2) + b"\n"
    return body + pad + data_line


def _payload_bytes(prepared, n: int, data: str) -> bytes:
    if data == "ascii":
        return _ascii_payload(prepared, n)
    if data == "binary":
        return _binary_payload(prepared, n)
    return _compressed_payload(prepared, n)


def _ascii_payload(prepared, n: int) -> bytes:
    lines = []
    for row in range(n):
        tokens = []
        for name, size, type_code, count, arr in prepared:
            values = arr[row] if count > 1 else [arr[row]]
            for value in np.reshape(values, (-1,)):
                tokens.append(_format_ascii(value, type_code, int(arr.dtype.itemsize)))
        lines.append(" ".join(tokens))
    if not lines:
        return b""
    return ("\n".join(lines) + "\n").encode("ascii")


def _format_ascii(value, type_code: str, itemsize: int) -> str:
    if type_code == "F":
        number = float(value)
        if not math.isfinite(number):
            raise ValueError("cannot write a non-finite ASCII point")
        # Nine significant digits round-trip float32. Seventeen round-trip float64.
        spec = "%.17g" if itemsize >= 8 else "%.9g"
        return spec % number
    return str(int(value))


def _binary_payload(prepared, n: int) -> bytes:
    step = sum(size * count for _, size, _, count, _ in prepared)
    buf = np.zeros((n, step), dtype=np.uint8)
    offset = 0
    for _, size, _, count, arr in prepared:
        width = size * count
        raw = np.ascontiguousarray(arr)
        if count == 1:
            raw = raw.reshape(n, 1)
        view = raw.view(np.uint8).reshape(n, width)
        buf[:, offset : offset + width] = view
        offset += width
    return buf.tobytes()


def _compressed_payload(prepared, n: int) -> bytes:
    parts = []
    for name, size, _, count, arr in prepared:
        if name == "_" or name.startswith("_"):
            continue
        raw = np.ascontiguousarray(arr)
        if count == 1:
            raw = raw.reshape(n, 1)
        parts.append(raw.view(np.uint8).reshape(-1).tobytes())
    raw_bytes = b"".join(parts)
    if n == 0:
        return struct.pack("<II", 0, 0)
    compressed = lzf_compress_literals(raw_bytes)
    return struct.pack("<II", len(compressed), len(raw_bytes)) + compressed


def _positive_ints(parts: Sequence[str], label: str) -> List[int]:
    if not parts:
        raise PcdFormatError("%s has no values" % label)
    values = []
    for part in parts:
        try:
            value = int(part)
        except ValueError as exc:
            raise PcdFormatError("%s value %r is not an integer" % (label, part)) from exc
        if value < 1:
            raise PcdFormatError("%s value %d must be >= 1" % (label, value))
        values.append(value)
    return values


def _type_codes(parts: Sequence[str]) -> List[str]:
    if not parts:
        raise PcdFormatError("TYPE has no values")
    codes = []
    for part in parts:
        if part not in ("F", "I", "U"):
            raise PcdFormatError("unknown TYPE %r" % part)
        codes.append(part)
    return codes


def _one_nonneg(parts: Sequence[str], label: str) -> int:
    if len(parts) != 2:
        raise PcdFormatError("%s needs one value" % label)
    try:
        value = int(parts[1])
    except ValueError as exc:
        raise PcdFormatError("%s is not an integer" % label) from exc
    if value < 0:
        raise PcdFormatError("%s must be >= 0" % label)
    return value
