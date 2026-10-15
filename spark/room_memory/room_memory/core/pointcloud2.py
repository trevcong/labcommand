"""CDR ``sensor_msgs/PointCloud2`` decoding, with the rules of ``PointCloud2CDR.swift``.

Wire layout, alignment counted from the byte after the 4-byte encapsulation header:
header (stamp int32 sec, uint32 nanosec, frame_id string), uint32 height, uint32 width,
PointField[] (string name, uint32 offset, uint8 datatype, uint32 count), uint8 is_bigendian,
uint32 point_step, uint32 row_step, uint8[] data, uint8 is_dense.
"""

from dataclasses import dataclass

import numpy as np

FLOAT32 = 7
FLOAT64 = 8
UINT8 = 2
UINT16 = 4

_POSITION_TYPES = {FLOAT32: 4, FLOAT64: 8}
_INTENSITY_TYPES = {FLOAT32: 4, FLOAT64: 8, UINT8: 1, UINT16: 2}


class PointCloudError(ValueError):
    """Truncated, malformed, or unsupported PointCloud2 bytes."""


@dataclass
class PointCloud:
    """Finite positions of one decoded frame.

    ``positions`` is ``float32`` with shape ``(n, 3)``, in meters, in the message's frame.
    Points with a non-finite coordinate are skipped, as the Swift decoder does.
    """

    positions: np.ndarray
    frame_id: str
    stamp_sec: int
    stamp_nanosec: int


class _Reader:
    def __init__(self, data):
        if len(data) < 4:
            raise PointCloudError(f"PointCloud2 CDR: truncated encapsulation header ({len(data)} of 4 bytes)")
        if data[0] == 0x00 and data[1] == 0x01:
            self.order = "<"
        elif data[0] == 0x00 and data[1] == 0x00:
            self.order = ">"
        else:
            raise PointCloudError(
                f"PointCloud2 CDR: unsupported encapsulation 0x{data[0]:02x} 0x{data[1]:02x} "
                "(expected CDR_LE 0x00 0x01 or CDR_BE 0x00 0x00)"
            )
        self.data = data
        self.offset = 4

    def take(self, length, what):
        if length > len(self.data) - self.offset:
            have = max(0, len(self.data) - self.offset)
            raise PointCloudError(f"PointCloud2 CDR: truncated {what} at byte {self.offset} (need {length}, have {have})")
        start = self.offset
        self.offset += length
        return start

    def u8(self, what):
        return self.data[self.take(1, what)]

    def u32(self, what):
        misalignment = (self.offset - 4) % 4
        if misalignment:
            self.offset += 4 - misalignment
        start = self.take(4, what)
        return int.from_bytes(self.data[start:start + 4], "little" if self.order == "<" else "big")

    def i32(self, what):
        value = self.u32(what)
        return value - (1 << 32) if value >= 1 << 31 else value

    def string(self, what):
        length = self.u32(what + " length")
        start = self.take(length, what)
        body = bytes(self.data[start:start + length])
        end = body.find(b"\x00")
        return (body if end < 0 else body[:end]).decode("utf-8", errors="replace")


def decode_pointcloud2(data):
    """Decodes a CDR PointCloud2 message, including the encapsulation header.

    Fields named ``x``, ``y``, ``z`` (case-insensitive, first occurrence) must be FLOAT32 or
    FLOAT64; FLOAT64 values round to binary32. An ``intensity`` field is checked for bounds
    like the Swift decoder but not returned. ``row_step`` 0 means tightly packed rows.

    Args:
        data: ``bytes``, ``bytearray`` or ``memoryview`` of the whole message.

    Returns:
        A ``PointCloud`` of the finite points, in input order.

    Raises:
        PointCloudError: On truncated, malformed, or unsupported input.
    """
    buffer = memoryview(data).cast("B")
    reader = _Reader(buffer)
    stamp_sec = reader.i32("header.stamp.sec")
    stamp_nanosec = reader.u32("header.stamp.nanosec")
    frame_id = reader.string("header.frame_id")
    height = reader.u32("height")
    width = reader.u32("width")

    fields = {}
    count = reader.u32("fields length")
    for index in range(count):
        name = reader.string(f"fields[{index}].name").lower()
        offset = reader.u32(f"fields[{index}].offset")
        datatype = reader.u8(f"fields[{index}].datatype")
        reader.u32(f"fields[{index}].count")
        if name in ("x", "y", "z", "intensity") and name not in fields:
            fields[name] = (offset, datatype)

    big_endian = reader.u8("is_bigendian") != 0
    point_step = reader.u32("point_step")
    declared_row_step = reader.u32("row_step")
    data_length = reader.u32("data length")
    data_start = reader.take(data_length, "data")
    reader.u8("is_dense")

    total = width * height
    empty = PointCloud(np.zeros((0, 3), dtype=np.float32), frame_id, stamp_sec, stamp_nanosec)
    if total == 0:
        return empty
    if point_step == 0:
        raise PointCloudError(f"PointCloud2 CDR: point_step is 0 for a {width}x{height} cloud")
    packed_row = point_step * width
    row_step = declared_row_step if declared_row_step > 0 else packed_row
    if packed_row > row_step:
        raise PointCloudError(
            f"PointCloud2 CDR: point_step ({point_step}) x width ({width}) exceeds row_step ({row_step})"
        )
    if row_step * height > data_length:
        raise PointCloudError(
            f"PointCloud2 CDR: data holds {data_length} bytes but row_step ({row_step}) x height ({height}) needs more"
        )

    readers = []
    for name in ("x", "y", "z"):
        if name not in fields:
            raise PointCloudError(f"PointCloud2 CDR: missing required field '{name}'")
        offset, datatype = fields[name]
        if datatype not in _POSITION_TYPES:
            raise PointCloudError(
                f"PointCloud2 CDR: field '{name}' has datatype {datatype}; only FLOAT32 (7) and FLOAT64 (8) are supported"
            )
        readers.append(_bounded(name, offset, _POSITION_TYPES[datatype], point_step))
    if "intensity" in fields and fields["intensity"][1] in _INTENSITY_TYPES:
        offset, datatype = fields["intensity"]
        _bounded("intensity", offset, _INTENSITY_TYPES[datatype], point_step)

    rows = np.frombuffer(buffer, dtype=np.uint8, count=row_step * height, offset=data_start)
    records = rows.reshape(height, row_step)[:, :packed_row].reshape(total, point_step)
    order = ">" if big_endian else "<"
    columns = []
    for offset, size in readers:
        raw = np.ascontiguousarray(records[:, offset:offset + size]).view(f"{order}f{size}").reshape(total)
        columns.append(raw.astype(np.float32))
    positions = np.stack(columns, axis=1)
    finite = np.isfinite(positions).all(axis=1)
    return PointCloud(np.ascontiguousarray(positions[finite]), frame_id, stamp_sec, stamp_nanosec)


def _bounded(name, offset, size, point_step):
    if offset > point_step - size:
        raise PointCloudError(
            f"PointCloud2 CDR: field '{name}' (offset {offset}, {size} bytes) extends past point_step ({point_step})"
        )
    return offset, size
