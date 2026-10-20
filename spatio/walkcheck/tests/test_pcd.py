from pathlib import Path

import numpy as np
import pytest

from spatio.walkcheck.pcd import PcdError, read_pcd_xyz
from spatio.walkcheck.synthetic import write_pcd_ascii, write_pcd_binary


def test_ascii_roundtrip(tmp_path: Path):
    points = np.array([[0.0, 1.0, -2.5], [3.25, -4.5, 0.125]], dtype=np.float64)
    path = tmp_path / "cloud.pcd"
    write_pcd_ascii(path, points)
    got = read_pcd_xyz(path)
    assert got.shape == (2, 3)
    assert got == pytest.approx(points, abs=1e-5)


def test_binary_roundtrip(tmp_path: Path):
    rng = np.random.default_rng(2)
    points = rng.normal(size=(30, 3))
    path = tmp_path / "cloud.pcd"
    write_pcd_binary(path, points)
    got = read_pcd_xyz(path)
    assert got.shape == (30, 3)
    assert got == pytest.approx(points, abs=1e-6)


def test_ascii_ignores_intensity(tmp_path: Path):
    path = tmp_path / "extra.pcd"
    path.write_text(
        "\n".join(
            [
                "# .PCD v0.7",
                "VERSION 0.7",
                "FIELDS x y z intensity",
                "SIZE 4 4 4 4",
                "TYPE F F F F",
                "COUNT 1 1 1 1",
                "WIDTH 2",
                "HEIGHT 1",
                "POINTS 2",
                "DATA ascii",
                "1.0 2.0 3.0 9.0",
                "4.0 5.0 6.0 8.0",
                "",
            ]
        ),
        encoding="utf-8",
    )
    got = read_pcd_xyz(path)
    assert got == pytest.approx(np.array([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]]))


def test_binary_compressed_is_rejected(tmp_path: Path):
    path = tmp_path / "compressed.pcd"
    path.write_bytes(
        b"VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n"
        b"WIDTH 1\nHEIGHT 1\nPOINTS 1\nDATA binary_compressed\nxxxx"
    )
    with pytest.raises(PcdError, match="binary_compressed"):
        read_pcd_xyz(path)


def test_missing_xyz_is_rejected(tmp_path: Path):
    path = tmp_path / "no-xyz.pcd"
    path.write_text(
        "FIELDS a b c\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH 1\nHEIGHT 1\nPOINTS 1\nDATA ascii\n1 2 3\n",
        encoding="utf-8",
    )
    with pytest.raises(PcdError, match="x y z"):
        read_pcd_xyz(path)
