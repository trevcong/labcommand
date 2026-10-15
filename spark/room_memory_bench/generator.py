"""Seeded, bounded-memory ray/box replay; all coordinates are meters."""
from dataclasses import dataclass
import numpy as np

SENSORS = ("jt128", "mid360")
HZ = 10
ROOM = ((-4., -4., -1.5), (4., 4., 1.5))
BOXES = {
    "appeared": ((1.0, -.8, -.6), (1.6, -.2, 0.)),
    "disappeared": ((-1.6, .4, -.6), (-1., 1., 0.)),
    "moved": ((.2, 1.2, -.6), (.8, 1.8, 0.)),
    "destination": ((1.2, 1.2, -.6), (1.8, 1.8, 0.)),
}


@dataclass(frozen=True)
class ReplayConfig:
    duration: int = 3600
    points: int = 80_000
    seed: int = 7
    noise_m: float = .01

    def __post_init__(self):
        if self.duration < 1 or self.points < 1 or self.seed < 0:
            raise ValueError("duration/points must be positive and seed nonnegative")
        if not np.isfinite(self.noise_m) or self.noise_m < 0:
            raise ValueError("noise_m must be finite and nonnegative")


@dataclass
class Tick:
    index: int
    time_s: float
    clouds: dict
    close_window: bool


def ground_truth(config):
    """Transitions away from the frozen reference; restoration resets are separate."""
    events = []
    for cycle in range((config.duration + 59) // 60):
        for offset, kind in ((10, "appeared"), (25, "disappeared"), (40, "moved")):
            stamp = cycle * 60 + offset
            if stamp >= config.duration:
                continue
            event = {"id": f"{cycle}:{kind}", "time_s": stamp, "type": kind,
                     "bbox": BOXES["destination" if kind == "moved" else kind],
                     "end_s": min((cycle + 1) * 60, config.duration)}
            if kind == "moved":
                event["from_bbox"] = BOXES["moved"]
            events.append(event)
    return {"schema": 1, "events": events,
            "reference_build_s": 5,
            "restorations_s": list(range(60, config.duration, 60)),
            "room_bbox": ROOM, "baseline_boxes": [BOXES["disappeared"], BOXES["moved"]]}


def active_boxes(time_s):
    phase = time_s % 60
    boxes = []
    if phase >= 10:
        boxes.append(BOXES["appeared"])
    if phase < 25:
        boxes.append(BOXES["disappeared"])
    boxes.append(BOXES["destination" if phase >= 40 else "moved"])
    return boxes


def _cloud(config, index, sensor_index):
    # Per-frame streams permit random access and do not depend on iteration order.
    rng = np.random.default_rng(np.random.SeedSequence([config.seed, index, sensor_index]))
    n = config.points
    if sensor_index == 0:
        # 128 spinning elevation channels, staggered azimuth on each revolution.
        elevation = np.linspace(-.65, .65, 128)[np.arange(n) % 128]
        azimuth = (np.arange(n) // 128) * (2 * np.pi / max(1, (n + 127) // 128))
        azimuth += (index * .037) % (2 * np.pi)
    else:
        # Broad nonrepeating solid-angle sample, simplified Mid-360-like pattern.
        azimuth = rng.uniform(-np.pi, np.pi, n)
        elevation = np.arcsin(rng.uniform(np.sin(-.65), np.sin(.9), n))
    directions = np.column_stack((np.cos(elevation) * np.cos(azimuth),
                                  np.cos(elevation) * np.sin(azimuth), np.sin(elevation)))
    with np.errstate(divide="ignore", invalid="ignore"):
        inv = 1. / directions
        exits = np.maximum(np.asarray(ROOM[0]) * inv, np.asarray(ROOM[1]) * inv)
        distance = exits.min(axis=1)
        for low, high in active_boxes(index / HZ):
            a, b = np.asarray(low) * inv, np.asarray(high) * inv
            near, far = np.minimum(a, b).max(axis=1), np.maximum(a, b).min(axis=1)
            distance = np.where((near > 0) & (far >= near), np.minimum(distance, near), distance)
    distance += rng.normal(0, config.noise_m, n)
    return np.ascontiguousarray(directions * distance[:, None], dtype=np.float32)


def replay(config):
    """Hold only the current pair of clouds; consumers may retain their own window."""
    for index in range(config.duration * HZ):
        yield Tick(index, index / HZ,
                   {sensor: _cloud(config, index, i) for i, sensor in enumerate(SENSORS)},
                   (index + 1) % HZ == 0)
