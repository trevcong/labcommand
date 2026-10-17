"""The rig guard's IMU detector (Mid-360 ``sensor_msgs/Imu``): shocks, quiet, gravity and the gyro
rotation over a shock. No ROS imports; the C++ twin is ``room_memory_cpp/src/imu.cpp``.

Units are not assumed. At stream start, and after every gap of more than GAP_S, the detector
re-estimates the at-rest magnitude ``g0`` (median |a|) and the gyro bias (mean ω) over ``quiet_s`` of
samples and cannot report a shock until then, so a stream restart is never a shock. A shock is
``||a| / g0 - 1| > shock_frac`` on one sample, or ``|ω - bias| > gyro_dps`` on GYRO_RUN consecutive
samples; any sample above either threshold disturbs, and the IMU is quiet ``quiet_s`` after the last
disturbance. Gravity is the mean acceleration direction of the undisturbed samples of a window. From a
shock until quiet the bias-corrected rates are integrated (header stamps) into the rotation of the new
sensor frame in the old one: the increment ``D`` of guard.py's conventions. Times are receive times in
nanoseconds; samples arrive in order.
"""

import math
from typing import Optional

from .guard import _IDENTITY, Pose, rotation_vector

GAP_S = 1.0
GYRO_RUN = 3
# The longest gap between two header stamps the gyro integral bridges (a dropped sample or two).
MAX_STEP_S = 0.05


def _stamp_ns(sample):
    return sample.stamp.sec * 1_000_000_000 + sample.stamp.nanosec


class ImuDetector:
    def __init__(self, shock_frac, gyro_dps, quiet_s):
        self.shock_frac = shock_frac
        self.gyro_rad = math.radians(gyro_dps)
        self.quiet_ns = int(quiet_s * 1e9)
        self.g0 = None
        self.bias = None
        self.last_received = None
        self._calibration = []  # (|a|, ω) since the stream (re)started, until calibrated
        self._calibration_start = None
        self._run = 0
        self._last_disturbance = None
        self._shock = False  # a shock since the last take_shock
        self._integrating = False
        self._rotation = _IDENTITY
        self._last_stamp = None
        self._finished = None  # the rotation of the last completed shock, until taken
        self._gravity = [0.0, 0.0, 0.0]
        self._gravity_count = 0

    @property
    def calibrated(self):
        return self.g0 is not None

    def add(self, sample, received_ns):
        a = sample.linear_acceleration
        w = sample.angular_velocity
        if self.last_received is not None and received_ns - self.last_received > int(GAP_S * 1e9):
            self._restart()
        self.last_received = received_ns
        magnitude = math.sqrt((a[0] * a[0] + a[1] * a[1]) + a[2] * a[2])
        stamp = _stamp_ns(sample)
        if not self.calibrated:
            if self._calibration_start is None:
                self._calibration_start = received_ns
            self._calibration.append((magnitude, w))
            if received_ns - self._calibration_start >= self.quiet_ns:
                self._calibrate()
            self._last_stamp = stamp
            return
        rate = (w[0] - self.bias[0], w[1] - self.bias[1], w[2] - self.bias[2])
        spin = math.sqrt((rate[0] * rate[0] + rate[1] * rate[1]) + rate[2] * rate[2])
        jolt = abs(magnitude / self.g0 - 1.0) > self.shock_frac
        turning = spin > self.gyro_rad
        self._run = self._run + 1 if turning else 0
        if jolt or turning:
            self._last_disturbance = received_ns
        if (jolt or self._run >= GYRO_RUN) and not self._integrating:
            self._shock = True
            self._integrating = True
            self._rotation = _IDENTITY
            self._finished = None
        if self._integrating and self._last_stamp is not None:
            step = (stamp - self._last_stamp) / 1e9
            if 0.0 < step <= MAX_STEP_S:
                increment = rotation_vector(rate[0] * step, rate[1] * step, rate[2] * step)
                self._rotation = Pose(self._rotation).compose(Pose(increment)).R
        if not (jolt or turning):
            for n in range(3):
                self._gravity[n] += a[n]
            self._gravity_count += 1
        self._last_stamp = stamp

    def _calibrate(self):
        magnitudes = sorted(m for m, _ in self._calibration)
        self.g0 = magnitudes[len(magnitudes) // 2]
        count = len(self._calibration)
        self.bias = tuple(sum(w[n] for _, w in self._calibration) / count for n in range(3))
        self._calibration = []
        if not self.g0 > 0.0:
            self.g0 = None
            self._calibration_start = None

    def _restart(self):
        self.g0 = None
        self.bias = None
        self._calibration = []
        self._calibration_start = None
        self._run = 0
        self._integrating = False
        self._rotation = _IDENTITY
        self._last_stamp = None

    def ok(self, now_ns):
        """Samples within the last second, and calibrated."""
        return self.calibrated and self.last_received is not None and now_ns - self.last_received <= 1_000_000_000

    def quiet(self, now_ns):
        """Calibrated and no disturbance within ``quiet_s`` before ``now_ns``."""
        return self.calibrated and (self._last_disturbance is None or now_ns - self._last_disturbance >= self.quiet_ns)

    def take_shock(self):
        """Whether a shock started since the last call."""
        shock, self._shock = self._shock, False
        return shock

    def take_rotation(self, now_ns):
        """The gyro rotation of the last shock (the increment ``D``'s rotation, sensor frame), once
        the IMU is quiet after it; None while it is still moving or without one. Taken once."""
        if self._integrating and self.quiet(now_ns):
            self._integrating = False
            self._finished = self._rotation
        finished, self._finished = self._finished, None
        return finished

    def take_gravity(self):
        """The mean acceleration of the undisturbed samples since the last call, or None."""
        if not self._gravity_count:
            return None
        count = self._gravity_count
        gravity = tuple(value / count for value in self._gravity)
        self._gravity = [0.0, 0.0, 0.0]
        self._gravity_count = 0
        return gravity if any(gravity) else None
