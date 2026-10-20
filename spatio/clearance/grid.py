"""Sparse evidence, with independent free and occupied histories.

Free rays are zero-width segments: no beam widening or filling between rays.
Cells containing the sensor origin are never marked free by that ray.
"""

from dataclasses import dataclass, field
from itertools import product
import math

import numpy as np

from .model import number, vector


@dataclass
class Cell:
    free_times: set = field(default_factory=set)
    occupied_times: set = field(default_factory=set)
    hits: list = field(default_factory=list)


class EvidenceGrid:
    def __init__(self, voxel_size, origin=(0, 0, 0), max_cells=1_000_000):
        self.size = number(voxel_size, "voxel_size")
        if self.size <= 0:
            raise ValueError("voxel_size must be positive")
        self.origin = vector(list(origin), "grid origin")
        if type(max_cells) is not int or max_cells <= 0:
            raise ValueError("max_cells must be a positive integer")
        self.max_cells = max_cells
        self.cells = {}
        self.timestamps = []

    def coordinates(self, point):
        with np.errstate(over="ignore", invalid="ignore"):
            result = (np.asarray(point) - self.origin) / self.size
        # Bound integer precision, not scene position or sensor configuration.
        if not np.all(np.isfinite(result)) or np.any(np.abs(result) > 2**50):
            raise ValueError("coordinates exceed supported voxel precision")
        nearest = np.rint(result)
        tolerance = 8 * np.finfo(float).eps * np.maximum(1, np.abs(result))
        result = np.where(np.abs(result - nearest) <= tolerance, nearest, result)
        return result

    def index(self, point):
        return tuple(int(v) for v in np.floor(self.coordinates(point)))

    def bounds(self, index):
        low = self.origin + np.array(index) * self.size
        return low, low + self.size

    def cell(self, index):
        if index not in self.cells:
            if len(self.cells) >= self.max_cells:
                raise ValueError("grid exceeds max_cells; use a smaller scene")
            self.cells[index] = Cell()
        return self.cells[index]

    def ray_cells(self, start, end):
        """Positive-length interior intersections, excluding origin and hit cells."""
        a, b = self.coordinates(start), self.coordinates(end)
        delta = b - a
        # A ray lying exactly on a voxel face proves neither neighbouring
        # interior free. Do not choose a side by floating-point convention.
        if any(delta[i] == 0 and a[i] == math.floor(a[i]) for i in range(3)):
            return set()
        crossings = [0.0, 1.0]
        for axis in range(3):
            if delta[axis] == 0:
                continue
            lo, hi = sorted((a[axis], b[axis]))
            first, last = math.floor(lo) + 1, math.ceil(hi)
            if last - first > self.max_cells:
                raise ValueError("ray exceeds max_cells")
            for boundary in range(first, last):
                t = (boundary - a[axis]) / delta[axis]
                if 0 < t < 1:
                    crossings.append(t)
        crossings = sorted(set(crossings))
        traversed = set()
        for left, right in zip(crossings, crossings[1:]):
            midpoint = a + ((left + right) / 2) * delta
            traversed.add(tuple(int(v) for v in np.floor(midpoint)))
        traversed.discard(self.index(start))
        traversed.discard(self.index(end))
        return traversed

    def add_scan(self, scan):
        if self.timestamps and scan.timestamp <= self.timestamps[-1]:
            raise ValueError("scan timestamps must be strictly increasing")
        # Accumulation is monotonic: a later ray never erases a prior hit.
        for hit in scan.hits:
            for index in self.ray_cells(scan.origin, hit):
                self.cell(index).free_times.add(scan.timestamp)
            cell = self.cell(self.index(hit))
            cell.occupied_times.add(scan.timestamp)
            cell.hits.append((scan.timestamp, hit.copy()))
        self.timestamps.append(scan.timestamp)

    def volume_cells(self, zone):
        low = self.coordinates(zone.low)
        high = self.coordinates(zone.high)
        starts = [math.floor(v) for v in low]
        stops = [math.ceil(v) for v in high]
        count = math.prod(b - a for a, b in zip(starts, stops))
        if count > self.max_cells:
            raise ValueError("zone exceeds max_cells")
        return product(*(range(a, b) for a, b in zip(starts, stops)))
