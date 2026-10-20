"""Strict JSON contracts. Geometry is in metres in a shared map frame."""

from dataclasses import dataclass
import json
import math
from pathlib import Path

import numpy as np


def keys(value, expected, where):
    if not isinstance(value, dict) or set(value) != set(expected):
        raise ValueError(f"{where}: expected exactly {sorted(expected)}")


def number(value, where):
    if type(value) not in (int, float) or not math.isfinite(value):
        raise ValueError(f"{where}: expected a finite number (not boolean)")
    return float(value)


def vector(value, where):
    if not isinstance(value, list) or len(value) != 3:
        raise ValueError(f"{where}: expected three numbers")
    return np.array([number(x, where) for x in value])


def text(value, where):
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"{where}: expected nonempty text")
    return value


def load_json(path):
    def pairs(items):
        result = {}
        for key, value in items:
            if key in result:
                raise ValueError(f"duplicate JSON key: {key}")
            result[key] = value
        return result

    def constant(value):
        raise ValueError(f"nonfinite JSON constant: {value}")

    return json.loads(Path(path).read_text(), object_pairs_hook=pairs,
                      parse_constant=constant)


def header(data, member):
    keys(data, ("schema_version", "frame_id", member), "document")
    if type(data["schema_version"]) is not int or data["schema_version"] != 1:
        raise ValueError("schema_version must be integer 1")
    frame = text(data["frame_id"], "frame_id")
    if not isinstance(data[member], list):
        raise ValueError(f"{member}: expected a list")
    return frame


@dataclass(frozen=True)
class Zone:
    id: str
    shape: str
    rule_id: str
    provision: str
    low: np.ndarray
    high: np.ndarray
    axis: int
    direction: int

    @property
    def depth(self):
        return float(self.high[self.axis] - self.low[self.axis])

    @property
    def reference(self):
        return float(self.low[self.axis] if self.direction == 1
                     else self.high[self.axis])


def parse_zones(data):
    frame = header(data, "zones")
    zones, ids = [], set()
    if not data["zones"]:
        raise ValueError("zones must not be empty")
    for item in data["zones"]:
        keys(item, ("id", "shape", "rule_id", "provision", "required_volume"), "zone")
        zid = text(item["id"], "id")
        if zid in ids:
            raise ValueError(f"duplicate zone id: {zid}")
        ids.add(zid)
        rule = text(item["rule_id"], "rule_id")
        provision = text(item["provision"], "provision")
        shape, volume = item["shape"], item["required_volume"]
        if shape == "box":
            keys(volume, ("min", "max", "clearance_axis", "clearance_direction"),
                 "box required_volume")
            low, high = vector(volume["min"], "min"), vector(volume["max"], "max")
            axis, direction = volume["clearance_axis"], volume["clearance_direction"]
            if type(axis) is not int or axis not in (0, 1, 2):
                raise ValueError("clearance_axis must be integer 0, 1 or 2")
            if type(direction) is not int or direction not in (-1, 1):
                raise ValueError("clearance_direction must be integer -1 or 1")
        elif shape == "plane-offset":
            keys(volume, ("plane_point", "normal", "u_min", "u_max",
                          "v_min", "v_max", "depth_m"), "plane-offset required_volume")
            point = vector(volume["plane_point"], "plane_point")
            normal = vector(volume["normal"], "normal")
            if np.count_nonzero(normal) != 1 or np.max(np.abs(normal)) != 1:
                raise ValueError("normal must be a signed coordinate unit axis")
            axis = int(np.flatnonzero(normal)[0])
            direction = int(normal[axis])
            depth = number(volume["depth_m"], "depth_m")
            if depth <= 0:
                raise ValueError("depth_m must be positive")
            low, high = point.copy(), point.copy()
            low[axis] += min(0, direction * depth)
            high[axis] += max(0, direction * depth)
            # Tangent coordinates are offsets from plane_point, in axis order.
            for tangent, label in zip([a for a in range(3) if a != axis], ("u", "v")):
                low[tangent] += number(volume[f"{label}_min"], f"{label}_min")
                high[tangent] += number(volume[f"{label}_max"], f"{label}_max")
        else:
            raise ValueError("shape must be box or plane-offset")
        if not np.all(np.isfinite([low, high])) or not np.all(high > low):
            raise ValueError("required volume must have finite, positive extents")
        zones.append(Zone(zid, shape, rule, provision, low, high, axis, direction))
    return frame, zones


@dataclass(frozen=True)
class Scan:
    timestamp: float
    origin: np.ndarray
    hits: np.ndarray


def parse_scans(data):
    frame = header(data, "scans")
    scans, previous = [], -math.inf
    for item in data["scans"]:
        keys(item, ("timestamp", "origin", "hits"), "scan")
        timestamp = number(item["timestamp"], "timestamp")
        if timestamp <= previous:
            raise ValueError("scan timestamps must be strictly increasing")
        previous = timestamp
        origin = vector(item["origin"], "origin")
        if not isinstance(item["hits"], list):
            raise ValueError("hits must be a list of first-return map-frame points")
        hits = np.array([vector(p, "hit") for p in item["hits"]]).reshape(-1, 3)
        if len(hits) and np.any(np.all(hits == origin, axis=1)):
            raise ValueError("a hit cannot equal its sensor origin")
        scans.append(Scan(timestamp, origin, hits))
    return frame, scans


def load_zones(path):
    return parse_zones(load_json(path))


def load_scans(path):
    return parse_scans(load_json(path))
