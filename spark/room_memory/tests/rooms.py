"""Ray-cast lidar scans of small synthetic rooms for the rig-guard tests: a sensor at a pose casts a
fixed fan of rays, and each return is the first surface it hits, with range noise. Boxes are solid
(the sensor sees their outside); the room is the inside of its box. Points come back in the
sensor's own frame, binary32, so a scan from a moved pose is what a moved rig would report.

The C++ tests (``room_memory_cpp/tests/rooms.cpp``) build the same scenes with the same rays.
"""

import math

import numpy as np

from room_memory.guard import Pose, rotation_about

# Interior of a 8 x 6 x 3 m room with the sensor 1.2 m above the floor, and solid furniture.
ROOM = ((-3.0, -2.0, -1.2), (5.0, 4.0, 1.8))
FURNITURE = (
    ((1.5, 1.0, -1.2), (2.3, 1.8, -0.4)),
    ((-2.5, -1.8, -1.2), (-1.7, -0.6, 0.6)),
    ((3.5, -1.5, -1.2), (4.2, -0.5, 0.3)),
    ((0.5, -2.0, -0.2), (1.5, -1.6, 0.5)),
    ((-1.0, 3.0, -1.2), (0.0, 3.6, -0.1)),
    ((2.8, 2.6, -1.2), (3.4, 3.2, 1.1)),
)
# A corridor (two parallel walls and a floor, open along x) and a lone floor: degenerate views.
CORRIDOR = ((-100.0, -1.0, -1.2), (100.0, 1.0, 100.0))
FLOOR_ONLY = ((-100.0, -100.0, -1.2), (100.0, 100.0, 100.0))


def fan(rows=32, columns=720, low_deg=-35.0, high_deg=30.0, seed=None):
    """Unit ray directions: ``rows`` elevations from ``low_deg`` to ``high_deg`` x ``columns`` azimuths.
    With ``seed`` the whole fan is offset by a random fraction of a step on each axis, as a spinning
    lidar's frames are, so frames accumulate into dense coverage."""
    shift = np.random.default_rng(seed).random(2) if seed is not None else np.zeros(2)
    row_step = (high_deg - low_deg) / (rows - 1)
    elevation = np.radians(np.linspace(low_deg, high_deg, rows) + (shift[0] - 0.5) * row_step)
    azimuth = np.radians((np.arange(columns) + shift[1]) * (360.0 / columns))
    e, a = np.meshgrid(elevation, azimuth, indexing="ij")
    return np.stack([np.cos(e) * np.cos(a), np.cos(e) * np.sin(a), np.sin(e)], axis=-1).reshape(-1, 3)


def _slabs(origin, directions, low, high):
    with np.errstate(divide="ignore", invalid="ignore"):
        t1 = (np.asarray(low) - origin) / directions
        t2 = (np.asarray(high) - origin) / directions
    near = np.nanmax(np.minimum(t1, t2), axis=1)
    far = np.nanmin(np.maximum(t1, t2), axis=1)
    return near, far


def scan(pose=Pose(), room=ROOM, boxes=FURNITURE, directions=None, noise_m=0.01, seed=0, max_range_m=30.0,
         columns=720):
    """The returns of a sensor at ``pose`` (sensor -> room frame): binary32 ``(n, 3)`` in the sensor
    frame. A ray without a return within ``max_range_m`` gives no point. Without ``directions`` the
    rays are ``fan(columns=columns, seed=seed)``."""
    directions = fan(columns=columns, seed=seed) if directions is None else directions
    R = np.array(pose.R).reshape(3, 3)
    origin = np.array(pose.t)
    world = np.stack([(directions[:, 0] * R[n, 0] + directions[:, 1] * R[n, 1]) + directions[:, 2] * R[n, 2]
                      for n in range(3)], axis=1)
    _, distance = _slabs(origin, world, *room)  # the room is seen from inside: its far face
    for low, high in boxes:
        near, far = _slabs(origin, world, low, high)
        hit = (near <= far) & (near > 0)
        distance = np.where(hit & (near < distance), near, distance)
    rng = np.random.default_rng(seed)
    distance = distance + rng.normal(0.0, noise_m, len(distance))
    keep = np.isfinite(distance) & (distance > 0.1) & (distance < max_range_m)
    return (directions[keep] * distance[keep, None]).astype(np.float32)


def rig(dx=0.0, dy=0.0, dz=0.0, yaw_deg=0.0, roll_deg=0.0, pitch_deg=0.0):
    """A rig pose in the room frame: yaw about z, then pitch about y, then roll about x."""
    R = rotation_about((0.0, 0.0, 1.0), yaw_deg)
    if pitch_deg:
        R = Pose(R).compose(Pose(rotation_about((0.0, 1.0, 0.0), pitch_deg))).R
    if roll_deg:
        R = Pose(R).compose(Pose(rotation_about((1.0, 0.0, 0.0), roll_deg))).R
    return Pose(R, (dx, dy, dz))


def pose_error(estimate, truth):
    """(meters, degrees) between two poses."""
    difference = truth.inverse().compose(estimate)
    return difference.distance_m, math.degrees(math.acos(difference.cos_angle))
