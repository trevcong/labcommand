import json
import os
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import session_table  # noqa: E402
import tum  # noqa: E402

IDENTITY = (0.0, 0.0, 0.0, 1.0)


def write_session(d, walkcheck=None):
    poses = [tum.make_pose(float(i), (float(i), 0.0, 0.0), IDENTITY) for i in range(11)]
    tum.write_tum(os.path.join(d, "trajectory.tum"), poses)
    if walkcheck is not None:
        with open(os.path.join(d, "walkcheck.json"), "w") as f:
            json.dump(walkcheck, f)


class SessionTableTests(unittest.TestCase):
    def test_row_without_walkcheck(self):
        with tempfile.TemporaryDirectory() as d:
            write_session(d)
            cells = [c.strip() for c in session_table.row(d).split("|")[1:-1]]
        self.assertEqual(cells[1:7], ["11", "10.0", "10.0", "10.000", "6.000", "not run"])
        self.assertEqual(cells[7:], ["—", "—"])

    def test_row_with_walkcheck(self):
        wc = {"result": "NOT-EVALUABLE", "reasons": ["timestamp_gap"],
              "return_to_start": None, "walls": {"p95_point_to_plane_cm": 5.2154}}
        with tempfile.TemporaryDirectory() as d:
            write_session(d, wc)
            cells = [c.strip() for c in session_table.row(d).split("|")[1:-1]]
        self.assertEqual(cells[6:], ["NOT-EVALUABLE (timestamp_gap)", "—", "5.22"])

    def test_header_and_row_have_same_column_count(self):
        with tempfile.TemporaryDirectory() as d:
            write_session(d, {"result": "FAIL", "reasons": ["return_error"],
                              "return_to_start": {"error_3d_m": 1.8}, "walls": {}})
            r = session_table.row(d)
        self.assertEqual(session_table.HEADER.splitlines()[0].count("|"), r.count("|"))
        self.assertIn("| 1.800 |", r)


if __name__ == "__main__":
    unittest.main()
