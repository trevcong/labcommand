import math
import os
import sys
import unittest
from types import SimpleNamespace as NS

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import bag_to_tum  # noqa: E402  (ROS imports are inside functions)
import tum  # noqa: E402


def ros_pose(x, y, z, qx, qy, qz, qw):
    return NS(position=NS(x=x, y=y, z=z), orientation=NS(x=qx, y=qy, z=qz, w=qw))


class PoseFromStampedTests(unittest.TestCase):
    def test_fields_map_to_tum_order(self):
        h = math.sqrt(0.5)
        p = bag_to_tum.pose_from_stamped(NS(sec=1786223549, nanosec=564407110),
                                         ros_pose(1.0, -2.0, 0.5, 0.0, 0.0, h, h))
        self.assertEqual((p.x, p.y, p.z), (1.0, -2.0, 0.5))
        self.assertEqual((p.qx, p.qy, p.qz, p.qw), (0.0, 0.0, h, h))
        self.assertAlmostEqual(p.t, 1786223549.564407110, places=6)

    def test_w_lands_in_qw_not_qx(self):
        p = bag_to_tum.pose_from_stamped(NS(sec=0, nanosec=0), ros_pose(0, 0, 0, 0, 0, 0, 1.0))
        self.assertEqual(tum.format_line(p).split()[4:], ["0.000000000"] * 3 + ["1.000000000"])

    def test_sensor_uptime_stamps_pass_through(self):
        # JT128 headers are sensor-uptime seconds; they must not be rewritten.
        p = bag_to_tum.pose_from_stamped(NS(sec=731, nanosec=5600000), ros_pose(0, 0, 0, 0, 0, 0, 1))
        self.assertAlmostEqual(p.t, 731.0056, places=9)

    def test_non_unit_orientation_rejected(self):
        with self.assertRaises(tum.TumError):
            bag_to_tum.pose_from_stamped(NS(sec=0, nanosec=0), ros_pose(0, 0, 0, 0, 0, 0, 0))


if __name__ == "__main__":
    unittest.main()
