"""The rig guard's IMU detector (room_memory/imu.py) on synthetic 200 Hz streams."""

import math

import numpy as np
import pytest

from room_memory import guard as g
from room_memory.imu import ImuDetector
from room_memory.messages import ImuSample, Time

RATE = 200
NS = 1_000_000_000
START = 1_760_400_000 * NS


def sample(t_ns, a, w=(0.0, 0.0, 0.0)):
    return ImuSample(Time(t_ns // NS, t_ns % NS), "livox_frame", tuple(map(float, w)), tuple(map(float, a)))


def stream(seconds, g0=1.0, noise=0.002, gyro_noise=0.001, seed=0, start_s=0.0, gravity=(0.0, 0.0, -1.0),
           events=()):
    """(received ns, sample) at RATE Hz. ``events`` holds (t0, t1, accel offset, gyro rad/s) bursts."""
    rng = np.random.default_rng(seed)
    out = []
    for n in range(int(seconds * RATE)):
        t = start_s + n / RATE
        a = np.array(gravity) * g0 + rng.normal(0.0, noise * g0, 3)
        w = rng.normal(0.0, gyro_noise, 3) + np.array([0.001, -0.002, 0.0005])  # a constant bias
        for t0, t1, accel, rate in events:
            if t0 <= t < t1:
                a = a + np.array(accel) * g0
                w = w + np.array(rate)
        ns = START + int(round(t * NS))
        out.append((ns, sample(ns, a, w)))
    return out


def feed(detector, samples):
    for ns, s in samples:
        detector.add(s, ns)
    return samples[-1][0]


@pytest.mark.parametrize("g0", [1.0, 9.80665])
def test_calibrates_in_either_unit_and_finds_the_bias(g0):
    detector = ImuDetector(0.05, 2.0, 2.0)
    feed(detector, stream(1.5, g0=g0))
    assert not detector.calibrated  # still within quiet_s of the stream start
    end = feed(detector, stream(1.0, g0=g0, start_s=1.5))
    assert detector.g0 == pytest.approx(g0, rel=0.01)
    assert detector.bias == pytest.approx((0.001, -0.002, 0.0005), abs=3e-4)
    assert detector.ok(end) and not detector.take_shock()


def test_a_50_ms_shock_triggers_and_noise_does_not():
    detector = ImuDetector(0.05, 2.0, 2.0)
    end = feed(detector, stream(600.0, noise=0.006, gyro_noise=0.006))  # 3x the rest noise, 10 minutes
    assert not detector.take_shock() and detector.quiet(end)
    detector = ImuDetector(0.05, 2.0, 2.0)
    feed(detector, stream(5.0))
    end = feed(detector, stream(0.05, start_s=5.0, events=[(5.0, 5.05, (0.4, 0.0, 0.0), (0.0, 0.0, 0.0))]))
    assert detector.take_shock() and not detector.take_shock()
    assert not detector.quiet(end)
    end = feed(detector, stream(2.5, start_s=5.05, seed=1))
    assert detector.quiet(end)


def test_a_stream_gap_recalibrates_without_a_shock():
    detector = ImuDetector(0.05, 2.0, 2.0)
    end = feed(detector, stream(5.0))
    assert detector.ok(end) and not detector.ok(end + 2 * NS)
    # The stream returns 30 s later in m/s²: a new g0, no shock.
    end = feed(detector, stream(5.0, g0=9.80665, start_s=35.0))
    assert detector.g0 == pytest.approx(9.80665, rel=0.01)
    assert not detector.take_shock() and detector.ok(end)


def test_the_gyro_integral_gives_the_rotation_of_a_shock():
    detector = ImuDetector(0.05, 2.0, 2.0)
    feed(detector, stream(5.0, gyro_noise=0.0005))
    # A 2 degree yaw over 0.5 s, then rest.
    rate = math.radians(2.0) / 0.5
    feed(detector, stream(0.5, start_s=5.0, gyro_noise=0.0005, events=[(5.0, 5.5, (0.0, 0.0, 0.0), (0.0, 0.0, rate))]))
    assert detector.take_shock()
    end = feed(detector, stream(1.0, start_s=5.5, gyro_noise=0.0005))
    assert detector.take_rotation(end) is None  # not quiet yet
    end = feed(detector, stream(1.5, start_s=6.5, gyro_noise=0.0005))
    rotation = g.Pose(detector.take_rotation(end))
    assert rotation.angle_deg == pytest.approx(2.0, abs=0.05)
    assert rotation.R[3] > 0  # about +z
    assert detector.take_rotation(end) is None


def test_gravity_is_the_mean_direction_of_undisturbed_samples():
    detector = ImuDetector(0.05, 2.0, 2.0)
    feed(detector, stream(3.0))
    detector.take_gravity()
    tilted = (math.sin(math.radians(0.3)), 0.0, -math.cos(math.radians(0.3)))
    feed(detector, stream(1.0, start_s=3.0, gravity=tilted, noise=0.0005))
    assert g.angle_between_deg(detector.take_gravity(), (0.0, 0.0, -1.0)) == pytest.approx(0.3, abs=0.02)
    assert detector.take_gravity() is None
