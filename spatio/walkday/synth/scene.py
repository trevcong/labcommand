"""Synthetic marked loop: a cart pushed around a box room, seen by a
Livox-style lidar with a co-located IMU. numpy only (no ROS).

Ground truth is exact: positions, yaw, IMU specific force and yaw rate are
computed analytically from one smooth path, so a correct estimator returns
the true end offset. Nothing here copies a real recording: room size, rates,
topics, frames and the time base are all parameters (see SceneParams).

World frame: z up, floor at z = 0. The sensor rides at `sensor_height_m`.
The cart only yaws (flat floor), so roll = pitch = 0.
"""

from __future__ import annotations

import math
from dataclasses import asdict, dataclass, field
from typing import List, Tuple

import numpy as np

G = 9.80665


@dataclass
class SceneParams:
    # Time base (s since epoch). Arbitrary; not taken from any recording.
    t0: float = 1800000000.0
    hold_start_s: float = 12.0         # parked on the marks (protocol: >= 10 s; walkcheck needs 8 s of detected stillness)
    hold_end_s: float = 12.0           # parked at the end
    ramp_s: float = 3.0                # speed-up / slow-down time
    loop_s: float = 50.0               # time spent moving (incl. ramps)
    # Loop: an ellipse around the room centre (metres).
    centre_xy: Tuple[float, float] = (4.0, 2.5)
    semi_axes: Tuple[float, float] = (3.2, 1.6)
    # Where the cart really stops relative to where it started (map axes at
    # the start: x = initial heading). PASS case 0.05 m, FAIL case 0.15 m.
    end_offset_m: Tuple[float, float, float] = (0.05, 0.0, 0.0)
    sensor_height_m: float = 1.0
    # Room (axis-aligned box) and clutter boxes [(min_xyz, max_xyz)]. The room
    # is centred on the start position: in an asymmetric box room FAST-LIO's
    # registration was measured to slide several cm toward the far wall while
    # parked (see SPATIO-m1-integration.md), which would swamp the 5 cm case.
    room_min: Tuple[float, float, float] = (-1.5, -3.6, 0.0)
    room_max: Tuple[float, float, float] = (9.5, 5.4, 3.2)
    boxes: List[Tuple[Tuple[float, float, float], Tuple[float, float, float]]] = field(
        default_factory=lambda: [
            ((3.8, 2.2, 0.0), (4.3, 2.7, 3.2)),     # pillar inside the loop
            ((-1.5, 3.4, 0.0), (-0.9, 5.4, 1.8)),   # cabinet on the -x wall
            ((8.6, -3.6, 0.0), (9.5, -2.1, 1.0)),   # low box in a corner
            ((2.0, 4.8, 0.0), (5.0, 5.4, 0.9)),     # bench on the +y wall
            ((6.5, 2.3, 0.0), (7.1, 2.9, 1.2)),     # crate inside the loop
            ((0.5, -3.6, 1.2), (2.5, -3.3, 1.6)),   # shelf on the -y wall
            ((-1.5, 0.5, 2.2), (9.5, 0.8, 2.5)),    # ceiling beam
        ]
    )
    # Extra seeded furniture (floor items, wall shelves, ceiling fixtures),
    # kept at least `furniture_clearance_m` from the loop. 0 = only `boxes`.
    furniture: int = 0
    furniture_clearance_m: float = 0.8
    lidar_rate_hz: float = 10.0
    points_per_scan: int = 12000
    elevation_deg: Tuple[float, float] = (-7.0, 52.0)   # Livox Mid-360 data-sheet FOV
    range_noise_m: float = 0.01
    imu_rate_hz: float = 250.0
    acc_noise_m_s2: float = 0.02
    gyro_noise_rad_s: float = 0.002
    seed: int = 7

    def as_json(self):
        d = asdict(self)
        d["boxes"] = [[list(a), list(b)] for a, b in self.boxes]
        return d


class Path:
    """Ellipse traversed once, plus a linear drift that ends at end_offset_m.

    theta(t) goes 0 -> 2*pi with a cosine speed ramp at both ends; the cart
    is exactly still during the holds.
    """

    def __init__(self, p: SceneParams):
        self.p = p
        a, b = p.semi_axes
        self.a, self.b = a, b
        # theta' has a plateau w and cosine ramps of length r:
        # integral = w * (loop_s - r) must equal 2*pi.
        self.r = p.ramp_s
        self.w = 2.0 * math.pi / (p.loop_s - p.ramp_s)
        self.t_move0 = p.t0 + p.hold_start_s
        self.t_move1 = self.t_move0 + p.loop_s
        self.t_end = self.t_move1 + p.hold_end_s
        # Start at the bottom of the ellipse, heading +x.
        self.base0 = self._ellipse(0.0)[0]
        # Rotate the world so the start heading is +x and the start is the origin
        # of the "start frame" (used for ground truth only).
        self.off = np.asarray(p.end_offset_m, dtype=float)

    # -- theta(t) and derivatives -------------------------------------------
    def theta(self, t):
        t = np.asarray(t, dtype=float)
        u = np.clip(t - self.t_move0, 0.0, self.p.loop_s)
        r, w, L = self.r, self.w, self.p.loop_s
        # ramp up: w/2 (1 - cos(pi u / r)) for u<r
        up = w / 2.0 * (u - r / math.pi * np.sin(math.pi * np.minimum(u, r) / r))
        up = np.where(u < r, up, w / 2.0 * r + w * (u - r))
        # ramp down over the last r seconds (mirror image)
        v = np.clip(u - (L - r), 0.0, r)
        down_loss = w / 2.0 * (v - r / math.pi * np.sin(math.pi * v / r))
        th = up - np.where(u > L - r, down_loss, 0.0)
        return th

    def theta_dot(self, t):
        t = np.asarray(t, dtype=float)
        u = t - self.t_move0
        r, w, L = self.r, self.w, self.p.loop_s
        d = np.where((u > 0) & (u < L), w, 0.0)
        d = np.where((u > 0) & (u < r), w / 2.0 * (1 - np.cos(math.pi * u / r)), d)
        v = u - (L - r)
        d = np.where((v > 0) & (v < r), w / 2.0 * (1 + np.cos(math.pi * v / r)), d)
        return d

    def theta_ddot(self, t):
        t = np.asarray(t, dtype=float)
        u = t - self.t_move0
        r, w, L = self.r, self.w, self.p.loop_s
        dd = np.zeros_like(u)
        dd = np.where((u > 0) & (u < r), w / 2.0 * math.pi / r * np.sin(math.pi * u / r), dd)
        v = u - (L - r)
        dd = np.where((v > 0) & (v < r), -w / 2.0 * math.pi / r * np.sin(math.pi * v / r), dd)
        return dd

    # -- geometry ------------------------------------------------------------
    def _ellipse(self, th):
        th = np.asarray(th, dtype=float)
        cx, cy = self.p.centre_xy
        ang = th - math.pi / 2.0
        p = np.stack([cx + self.a * np.cos(ang), cy + self.b * np.sin(ang)], axis=-1)
        d1 = np.stack([-self.a * np.sin(ang), self.b * np.cos(ang)], axis=-1)
        d2 = np.stack([-self.a * np.cos(ang), -self.b * np.sin(ang)], axis=-1)
        return p, d1, d2

    def _offset_xy(self):
        # end_offset_m is given in the start frame (x = initial heading = world +x
        # here, because the ellipse starts heading +x), so it maps directly.
        return self.off[:2]

    def state(self, t):
        """position (N,3), yaw (N,), world accel (N,3), yaw rate (N,)."""
        t = np.atleast_1d(np.asarray(t, dtype=float))
        th, thd, thdd = self.theta(t), self.theta_dot(t), self.theta_ddot(t)
        p, d1, d2 = self._ellipse(th)
        k = self._offset_xy() / (2.0 * math.pi)       # drift per radian
        p = p + th[:, None] * k
        d1 = d1 + k
        pos = np.column_stack([p, np.full(len(t), self.p.sensor_height_m)])
        pos[:, 2] += th * self.off[2] / (2.0 * math.pi)
        yaw = np.arctan2(d1[:, 1], d1[:, 0])
        yaw_dtheta = (d1[:, 0] * d2[:, 1] - d1[:, 1] * d2[:, 0]) / (d1[:, 0] ** 2 + d1[:, 1] ** 2)
        yaw_rate = yaw_dtheta * thd
        acc_xy = d2 * (thd ** 2)[:, None] + d1 * thdd[:, None]
        acc_z = np.full(len(t), 0.0)  # z drift is linear in theta; ignore its tiny accel
        acc_z += thdd * self.off[2] / (2.0 * math.pi)
        acc = np.column_stack([acc_xy, acc_z])
        return pos, yaw, acc, yaw_rate


def rotz(yaw):
    c, s = np.cos(yaw), np.sin(yaw)
    z, o = np.zeros_like(c), np.ones_like(c)
    return np.stack([np.stack([c, -s, z], -1), np.stack([s, c, z], -1), np.stack([z, z, o], -1)], -2)


def imu_samples(path: Path, rng: np.random.Generator):
    """Times, specific force in g (Livox convention) and gyro in rad/s, body frame."""
    p = path.p
    n = int(math.floor((path.t_end - p.t0 + 1.0) * p.imu_rate_hz))
    t = p.t0 - 0.5 + np.arange(n) / p.imu_rate_hz
    _, yaw, acc, yaw_rate = path.state(t)
    R = rotz(yaw)                                       # body -> world
    f_world = acc + np.array([0.0, 0.0, G])             # specific force
    f_body = np.einsum("nji,nj->ni", R, f_world)        # R^T f
    f_body += rng.normal(0, p.acc_noise_m_s2, f_body.shape)
    gyro = np.column_stack([np.zeros(n), np.zeros(n), yaw_rate])
    gyro += rng.normal(0, p.gyro_noise_rad_s, gyro.shape)
    return t, f_body / G, gyro


def _ray_box_exit(o, d, lo, hi):
    """Distance to leave an axis-aligned box from inside (rays start inside)."""
    with np.errstate(divide="ignore", invalid="ignore"):
        t1 = (lo - o) / d
        t2 = (hi - o) / d
    tmax = np.where(d > 0, t2, np.where(d < 0, t1, np.inf))
    return tmax.min(axis=1)


def _ray_box_entry(o, d, lo, hi):
    with np.errstate(divide="ignore", invalid="ignore"):
        t1 = (lo - o) / d
        t2 = (hi - o) / d
    tmin = np.minimum(t1, t2)
    tmax = np.maximum(t1, t2)
    # Axis-parallel rays: inside the slab -> unconstrained, outside -> miss.
    par = d == 0
    inside = (o >= lo) & (o <= hi)
    tmin = np.where(par, np.where(inside, -np.inf, np.inf), tmin)
    tmax = np.where(par, np.where(inside, np.inf, -np.inf), tmax)
    enter = tmin.max(axis=1)
    leave = tmax.min(axis=1)
    hit = (enter <= leave) & (enter > 1e-6)
    return np.where(hit, enter, np.inf)


_FURNITURE_CACHE: dict = {}


def all_boxes(p: SceneParams):
    """Fixed boxes plus `furniture` seeded random ones clear of the loop."""
    key = (p.furniture, p.seed, p.furniture_clearance_m, p.room_min, p.room_max,
           p.centre_xy, p.semi_axes, tuple(map(tuple, (b[0] + b[1] for b in p.boxes))))
    if key in _FURNITURE_CACHE:
        return _FURNITURE_CACHE[key]
    boxes = [(tuple(a), tuple(b)) for a, b in p.boxes]
    rng = np.random.default_rng(10_000 + p.seed)
    lo, hi = np.asarray(p.room_min, float), np.asarray(p.room_max, float)
    path = Path(p)
    loop = path._ellipse(np.linspace(0, 2 * math.pi, 400))[0]
    tries = 0
    while len(boxes) < len(p.boxes) + p.furniture and tries < 20000:
        tries += 1
        kind = rng.choice(["floor", "shelf", "ceiling"], p=[0.6, 0.25, 0.15])
        size = rng.uniform([0.3, 0.3, 0.4], [1.4, 1.4, 2.0])
        c = rng.uniform(lo[:2] + size[:2] / 2, hi[:2] - size[:2] / 2)
        if kind == "floor":
            z0, z1 = lo[2], lo[2] + size[2]
        elif kind == "shelf":
            # snap to the nearest wall, mounted 0.8-2.0 m up, 0.2-0.4 m deep
            dist = np.array([c[0] - lo[0], hi[0] - c[0], c[1] - lo[1], hi[1] - c[1]])
            k = int(np.argmin(dist))
            depth = rng.uniform(0.2, 0.4)
            if k < 2:
                size[0] = depth
                c[0] = lo[0] + depth / 2 if k == 0 else hi[0] - depth / 2
            else:
                size[1] = depth
                c[1] = lo[1] + depth / 2 if k == 2 else hi[1] - depth / 2
            z0 = rng.uniform(0.8, 2.0)
            z1 = z0 + rng.uniform(0.05, 0.5)
        else:
            z1 = hi[2]
            z0 = hi[2] - rng.uniform(0.1, 0.6)
        b0 = np.array([c[0] - size[0] / 2, c[1] - size[1] / 2, z0])
        b1 = np.array([c[0] + size[0] / 2, c[1] + size[1] / 2, z1])
        # clearance from the loop (in the horizontal plane) for anything the cart could hit
        nearest = np.clip(loop, b0[:2], b1[:2])
        if kind != "ceiling" and np.min(np.linalg.norm(loop - nearest, axis=1)) < p.furniture_clearance_m:
            continue
        boxes.append((tuple(b0), tuple(b1)))
    _FURNITURE_CACHE[key] = boxes
    return boxes


def raycast(p: SceneParams, origins, dirs):
    lo, hi = np.asarray(p.room_min, float), np.asarray(p.room_max, float)
    r = _ray_box_exit(origins, dirs, lo, hi)
    for blo, bhi in all_boxes(p):
        r = np.minimum(r, _ray_box_entry(origins, dirs, np.asarray(blo, float), np.asarray(bhi, float)))
    return r


def scan(path: Path, k: int, rng: np.random.Generator):
    """Scan k: header time (s), per-point offset (ns), sensor-frame xyz (f4)."""
    p = path.p
    period = 1.0 / p.lidar_rate_hz
    t_scan = p.t0 + k * period
    n = p.points_per_scan
    offs = np.sort(rng.uniform(0.0, period, n))
    az = rng.uniform(-math.pi, math.pi, n)
    lo, hi = np.radians(p.elevation_deg)
    el = np.arcsin(rng.uniform(math.sin(lo), math.sin(hi), n))
    d_s = np.column_stack([np.cos(el) * np.cos(az), np.cos(el) * np.sin(az), np.sin(el)])
    pos, yaw, _, _ = path.state(t_scan + offs)
    R = rotz(yaw)
    d_w = np.einsum("nij,nj->ni", R, d_s)
    rng_m = raycast(p, pos, d_w)
    rng_m = rng_m + rng.normal(0, p.range_noise_m, n)
    xyz = (d_s * rng_m[:, None]).astype(np.float32)
    off_ns = np.round(offs * 1e9).astype(np.uint32)
    return t_scan, off_ns, xyz


def scan_count(path: Path) -> int:
    p = path.p
    return int(math.floor((path.t_end - p.t0) * p.lidar_rate_hz))


def ground_truth(path: Path) -> dict:
    """True start/end pose of the sensor and the true return error."""
    p = path.p
    ts = np.array([p.t0, path.t_end])
    pos, yaw, _, _ = path.state(ts)
    d = pos[1] - pos[0]
    return {
        "start_xyz_world_m": pos[0].tolist(),
        "end_xyz_world_m": pos[1].tolist(),
        "start_yaw_deg": math.degrees(yaw[0]),
        "end_yaw_deg": math.degrees(yaw[1]),
        "true_return_error_3d_m": float(np.linalg.norm(d)),
        "true_return_error_horizontal_m": float(np.linalg.norm(d[:2])),
        "true_heading_change_deg": math.degrees(yaw[1] - yaw[0]),
        "moving_window_s": [path.t_move0, path.t_move1],
        "duration_s": path.t_end - p.t0,
    }
