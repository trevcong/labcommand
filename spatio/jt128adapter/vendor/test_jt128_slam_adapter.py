#!/usr/bin/env python3
"""Tests for the shipped JT128 → FAST-LIO adapter.

Drives scripts/jt128_slam_adapter.py conversion on real JT128-shaped bytes
(from the pcap fixture via jt128_decode) plus the Hesai manual gyro example.
No mocked converter, no hardcoded room XYZ.
"""

from __future__ import annotations

import math
import struct
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SCRIPTS = ROOT / "scripts"
sys.path.insert(0, str(SCRIPTS))

import jt128_decode  # noqa: E402
import jt128_slam_adapter as adapter  # noqa: E402

PCAP = ROOT / "tests" / "fixtures" / "jt128_one_frame.pcap"
ANGLE = SCRIPTS / "JT128_default_angle.csv"


def _pack_hesai_cloud(points: list[tuple[float, float, float, float, int, float]]) -> bytes:
    """Pack (x,y,z,intensity,ring,timestamp) into the Hesai PointCloud2 layout."""
    buf = bytearray(len(points) * 26)
    for i, (x, y, z, inten, ring, ts) in enumerate(points):
        off = i * 26
        struct.pack_into("<ffff", buf, off, x, y, z, inten)
        struct.pack_into("<H", buf, off + 16, ring)
        struct.pack_into("<d", buf, off + 18, ts)
    return bytes(buf)


HESAI_OFFS = {
    "x": (0, 7),
    "y": (4, 7),
    "z": (8, 7),
    "intensity": (12, 7),
    "ring": (16, 4),
    "timestamp": (18, 8),
}


class TestJt128SlamAdapter(unittest.TestCase):
    def test_gyro_manual_example_is_deg_s_not_rad_s(self):
        # Manual: field 5 → 5 × 30.49 mdps = 152.45 mdps = 0.15245 deg/s.
        gx, gy, gz = adapter.gyro_deg_s_to_rad_s(0.0, 0.0, 0.15245)
        self.assertEqual(adapter.gyro_deg_s_to_rad_s.__module__, "jt128_slam_adapter")
        self.assertAlmostEqual(gz, 0.15245 * math.pi / 180.0, places=12)
        self.assertLess(abs(gz), 0.01)  # must not leave 0.152 as rad/s (~8.7 deg/s)

    def test_cloud_time_is_seconds_from_scan_start(self):
        pts = [
            (1.0, 0.0, 0.5, 10.0, 0, 1000.00),
            (1.1, 0.2, 0.5, 11.0, 1, 1000.04),
            (1.2, 0.4, 0.6, 12.0, 2, 1000.10),
        ]
        payload, n, dt = adapter.hesai_cloud_to_velodyne(
            _pack_hesai_cloud(pts), 3, 1, 26, HESAI_OFFS, stride=1
        )
        self.assertEqual(n, 3)
        self.assertAlmostEqual(dt, 0.10, places=5)
        t0 = struct.unpack_from("<f", payload, 16)[0]
        t1 = struct.unpack_from("<f", payload, 24 + 16)[0]
        t2 = struct.unpack_from("<f", payload, 48 + 16)[0]
        self.assertAlmostEqual(t0, 0.0, places=5)
        self.assertAlmostEqual(t1, 0.04, places=5)
        self.assertAlmostEqual(t2, 0.10, places=5)
        x0, y0, z0 = struct.unpack_from("<fff", payload, 0)
        self.assertAlmostEqual(x0, 1.0)
        self.assertAlmostEqual(y0, 0.0)
        self.assertAlmostEqual(z0, 0.5)

    def test_adapter_on_real_jt128_pcap_points(self):
        self.assertTrue(PCAP.is_file())
        corr = jt128_decode.load_angle_correction(ANGLE)
        decoded = []
        t = 10.0
        n_pkt = 0
        for payload in jt128_decode.iter_pcap_udp_payloads(PCAP):
            pts = jt128_decode.decode_packet(payload, corr)
            # Keep a few points per packet so the test stays small.
            for p in pts[::16]:
                decoded.append((p.x, p.y, p.z, float(p.intensity), p.channel, t))
            t += 1.0 / 4490.0
            n_pkt += 1
            if n_pkt >= 250:
                break
        self.assertGreater(n_pkt, 100)
        self.assertGreater(len(decoded), 200)
        blob = _pack_hesai_cloud(decoded)
        out, n, dt = adapter.hesai_cloud_to_velodyne(
            blob, len(decoded), 1, 26, HESAI_OFFS, stride=1
        )
        self.assertEqual(n, len(decoded))
        self.assertGreater(dt, 0.01)
        self.assertLess(dt, 2.0)
        last_t = struct.unpack_from("<f", out, (n - 1) * 24 + 16)[0]
        self.assertGreater(last_t, 0.01)
        xs = [struct.unpack_from("<f", out, i * 24)[0] for i in range(0, n, 20)]
        zs = [struct.unpack_from("<f", out, i * 24 + 8)[0] for i in range(0, n, 20)]
        self.assertGreater(max(xs) - min(xs), 0.5)
        self.assertGreater(max(zs) - min(zs), 0.2)


if __name__ == "__main__":
    unittest.main()
