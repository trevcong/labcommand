"""PCD v0.7 ascii, binary, and binary_compressed."""

from __future__ import annotations

import struct
import tempfile
import unittest
from pathlib import Path

import numpy as np

from spatio.io.pcd import (
    PcdFormatError,
    lzf_decompress,
    read_pcd,
    read_pcd_header,
    write_pcd,
)

FIXTURES = Path(__file__).resolve().parents[1] / "fixtures"


def _header(kind, fields, size, type_code, count, points):
    return "\n".join(
        [
            "# .PCD v0.7 - Point Cloud Data file format",
            "VERSION 0.7",
            "FIELDS " + fields,
            "SIZE " + size,
            "TYPE " + type_code,
            "COUNT " + count,
            "WIDTH %d" % points,
            "HEIGHT 1",
            "VIEWPOINT 0 0 0 1 0 0 0",
            "POINTS %d" % points,
            "DATA " + kind,
            "",
        ]
    )


def _thousand():
    index = np.arange(1000, dtype=np.float32)
    return {
        "x": index * np.float32(0.125),
        "y": (index - np.float32(500.0)) * np.float32(0.25),
        "z": np.full(1000, np.float32(-1.5), dtype=np.float32),
    }


def _memmap_of(array):
    current = array
    seen = set()
    while current is not None and id(current) not in seen:
        if isinstance(current, np.memmap):
            return current
        seen.add(id(current))
        current = getattr(current, "base", None)
    return None


class PcdTests(unittest.TestCase):
    def test_pcd_ascii_binary_compressed_identical(self):
        # Two points, field-major compressed bytes written by hand.
        raw = struct.pack("<ffffff", 1.0, 4.0, 2.0, 5.0, 3.0, 6.0)
        compressed = bytes([len(raw) - 1]) + raw
        payload = struct.pack("<II", len(compressed), len(raw)) + compressed
        ascii_body = _header("ascii", "x y z", "4 4 4", "F F F", "1 1 1", 2) + "1 2 3\n4 5 6\n"
        binary_body = _header("binary", "x y z", "4 4 4", "F F F", "1 1 1", 2).encode(
            "ascii"
        ) + struct.pack("<ffffff", 1, 2, 3, 4, 5, 6)
        compressed_body = _header(
            "binary_compressed", "x y z", "4 4 4", "F F F", "1 1 1", 2
        ).encode("ascii") + payload
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            (root / "a.pcd").write_text(ascii_body, encoding="ascii")
            (root / "b.pcd").write_bytes(binary_body)
            (root / "c.pcd").write_bytes(compressed_body)
            loaded = [read_pcd(str(root / name)) for name in ("a.pcd", "b.pcd", "c.pcd")]
        for cloud in loaded:
            np.testing.assert_array_equal(cloud["x"], np.array([1.0, 4.0], dtype=np.float32))
            np.testing.assert_array_equal(cloud["y"], np.array([2.0, 5.0], dtype=np.float32))
            np.testing.assert_array_equal(cloud["z"], np.array([3.0, 6.0], dtype=np.float32))
        rng = np.random.default_rng(0)
        points = rng.standard_normal((50, 3)).astype(np.float32)
        arrays = {"x": points[:, 0], "y": points[:, 1], "z": points[:, 2]}
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            for kind in ("ascii", "binary", "binary_compressed"):
                write_pcd(str(root / (kind + ".pcd")), arrays, data=kind)
            back = [read_pcd(str(root / (kind + ".pcd"))) for kind in (
                "ascii", "binary", "binary_compressed"
            )]
        for cloud in back:
            np.testing.assert_array_equal(cloud["x"], arrays["x"])
            np.testing.assert_array_equal(cloud["y"], arrays["y"])
            np.testing.assert_array_equal(cloud["z"], arrays["z"])
            self.assertEqual(cloud["x"].dtype, np.dtype("<f4"))
        expected = _thousand()
        folder = FIXTURES / "pcd_formats"
        names = ("points_ascii.pcd", "points_binary.pcd", "points_binary_compressed.pcd")
        clouds = [read_pcd(str(folder / name)) for name in names]
        map_cloud = read_pcd(str(folder / "map.pcd"))
        for cloud in clouds + [map_cloud]:
            np.testing.assert_array_equal(cloud["x"], expected["x"])
            np.testing.assert_array_equal(cloud["y"], expected["y"])
            np.testing.assert_array_equal(cloud["z"], expected["z"])
        self.assertEqual(
            (folder / "map.pcd").read_bytes(),
            (folder / "points_binary.pcd").read_bytes(),
        )
        # Literal-only encoding still decodes, and real back-references do too.
        self.assertEqual(lzf_decompress(bytes([2, 97, 98, 99, 32, 2]), 6), b"abcabc")
        self.assertEqual(lzf_decompress(bytes([0, ord("a"), 64, 0]), 5), b"aaaaa")
        repeated = bytes([8]) + b"012345678" + bytes([224, 0, 8])
        self.assertEqual(lzf_decompress(repeated, 18), b"012345678" * 2)

    def test_pcd_extra_fields_and_padding(self):
        pad = struct.pack("<I", 0xAABBCCDD)
        rec0 = struct.pack("<fff", 1, 2, 3) + pad + struct.pack("<f", 7.5)
        rec1 = struct.pack("<fff", 4, 5, 6) + pad + struct.pack("<f", 8.5)
        blob = _header(
            "binary", "x y z _ intensity", "4 4 4 4 4", "F F F U F", "1 1 1 1 1", 2
        ).encode("ascii") + rec0 + rec1
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "pad.pcd"
            path.write_bytes(blob)
            cloud = read_pcd(str(path), fields=("x", "y", "z", "_", "intensity"))
        np.testing.assert_array_equal(cloud["x"], np.array([1, 4], dtype=np.float32))
        np.testing.assert_array_equal(cloud["z"], np.array([3, 6], dtype=np.float32))
        np.testing.assert_array_equal(cloud["_"], np.array([0xAABBCCDD, 0xAABBCCDD], dtype=np.uint32))
        np.testing.assert_array_equal(cloud["intensity"], np.array([7.5, 8.5], dtype=np.float32))
        normal = np.array(
            [[1, 0, 0], [0, 1, 0], [0, 0, 1], [0, 0, 1]], dtype=np.float32
        )
        intensity = np.array([1.5, 2.5, 3.5, 4.5], dtype=np.float32)
        padding = np.array([9, 8, 7, 6], dtype=np.uint8)
        arrays = {
            "x": np.arange(4, dtype=np.float32),
            "y": np.arange(4, dtype=np.float32) + np.float32(10),
            "z": np.full(4, np.float32(-2), dtype=np.float32),
            "normal": normal,
            "_": padding,
            "intensity": intensity,
        }
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            loaded = {}
            for kind in ("ascii", "binary", "binary_compressed"):
                dest = root / (kind + ".pcd")
                write_pcd(str(dest), arrays, data=kind)
                loaded[kind] = read_pcd(
                    str(dest), fields=("x", "normal", "_", "intensity")
                )
        for kind in ("ascii", "binary"):
            np.testing.assert_array_equal(loaded[kind]["normal"], normal)
            np.testing.assert_array_equal(loaded[kind]["_"], padding)
            np.testing.assert_array_equal(loaded[kind]["intensity"], intensity)
            self.assertEqual(loaded[kind]["normal"].shape, (4, 3))
        np.testing.assert_array_equal(loaded["binary_compressed"]["normal"], normal)
        np.testing.assert_array_equal(loaded["binary_compressed"]["intensity"], intensity)
        np.testing.assert_array_equal(
            loaded["binary_compressed"]["_"], np.zeros(4, dtype=np.uint8)
        )

    def test_pcd_width_height_points_mismatch_errors(self):
        mismatch = "\n".join(
            [
                "VERSION 0.7",
                "FIELDS x y z",
                "SIZE 4 4 4",
                "TYPE F F F",
                "COUNT 1 1 1",
                "WIDTH 4",
                "HEIGHT 4",
                "VIEWPOINT 0 0 0 1 0 0 0",
                "POINTS 10",
                "DATA ascii",
                "",
            ]
        )
        unknown = mismatch.replace("WIDTH 4", "WIDTH 10").replace("DATA ascii", "DATA banana")
        short_fields = "\n".join(
            [
                "VERSION 0.7",
                "FIELDS x y z",
                "SIZE 4 4",
                "TYPE F F F",
                "COUNT 1 1 1",
                "WIDTH 1",
                "HEIGHT 1",
                "POINTS 1",
                "DATA ascii",
                "",
            ]
        )
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            (root / "bad.pcd").write_text(mismatch, encoding="ascii")
            (root / "kind.pcd").write_text(unknown, encoding="ascii")
            (root / "fields.pcd").write_text(short_fields, encoding="ascii")
            with self.assertRaises(PcdFormatError) as width_error:
                read_pcd_header(str(root / "bad.pcd"))
            with self.assertRaises(PcdFormatError) as kind_error:
                read_pcd_header(str(root / "kind.pcd"))
            with self.assertRaises(PcdFormatError):
                read_pcd_header(str(root / "fields.pcd"))
        self.assertIn("WIDTH", str(width_error.exception))
        self.assertIn("unknown DATA", str(kind_error.exception))

    def test_pcd_truncated_binary_errors(self):
        header = _header("binary", "x y z", "4 4 4", "F F F", "1 1 1", 3).encode("ascii")
        one_point = struct.pack("<fff", 1, 2, 3)
        compressed_header = _header(
            "binary_compressed", "x y z", "4 4 4", "F F F", "1 1 1", 2
        ).encode("ascii")
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            (root / "short.pcd").write_bytes(header + one_point)
            (root / "cut.pcd").write_bytes(compressed_header + b"\x01\x00\x00")
            with self.assertRaises(PcdFormatError) as binary_error:
                read_pcd(str(root / "short.pcd"))
            with self.assertRaises(PcdFormatError) as compressed_error:
                read_pcd(str(root / "cut.pcd"))
        self.assertIn("truncated", str(binary_error.exception))
        self.assertIn("truncated", str(compressed_error.exception))

    def test_pcd_binary_is_memory_mapped(self):
        n = 2000000
        x = np.arange(n, dtype=np.float32)
        y = np.zeros(n, dtype=np.float32)
        z = np.ones(n, dtype=np.float32)
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "large.pcd"
            write_pcd(str(path), {"x": x, "y": y, "z": z}, data="binary")
            cloud = read_pcd(str(path))
            column = cloud["x"]
            mapped = _memmap_of(column)
            self.assertIsNotNone(mapped)
            self.assertFalse(column.flags.owndata)
            self.assertEqual(int(column.shape[0]), n)
            self.assertEqual(float(column[0]), 0.0)
            self.assertEqual(float(column[n // 2]), float(np.float32(n // 2)))
            self.assertEqual(float(column[-1]), float(np.float32(n - 1)))
            self.assertEqual(float(cloud["y"][10]), 0.0)
            self.assertEqual(float(cloud["z"][-1]), 1.0)
            self.assertEqual(Path(mapped.filename).resolve(), path.resolve())
            del column, mapped, cloud


if __name__ == "__main__":
    unittest.main()
