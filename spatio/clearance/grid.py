"""Sparse evidence, with independent free and occupied histories.

Free rays are zero-width segments: no beam widening or filling between rays.
Cells containing the sensor origin are never marked free by that ray.
"""

from dataclasses import dataclass, field
from itertools import product
import math

import numpy as np

from .model import number, vector

# Chord endpoints are compared as offsets inside their cell, in voxel units.
# Collinear rays of different range or origin reach the same chord only up to
# floating-point error (well under CHORD_TOLERANCE for rays bounded by
# max_cells); CHORD_QUANTUM buckets them. Merging is the conservative side.
CHORD_QUANTUM = 1e-6
CHORD_TOLERANCE = 1e-8


@dataclass
class Cell:
    free_times: set = field(default_factory=set)
    free_ray_count: int = 0
    free_chords: set = field(default_factory=set)
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
        return set(self.ray_chords(start, end))

    def ray_chords(self, start, end):
        """Map each traversed cell to the chord the ray cuts through it."""
        a, b = self.coordinates(start), self.coordinates(end)
        delta = b - a
        # A ray lying exactly on a voxel face proves neither neighbouring
        # interior free. Do not choose a side by floating-point convention.
        if any(delta[i] == 0 and a[i] == math.floor(a[i]) for i in range(3)):
            return {}
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
        spans = {}
        for left, right in zip(crossings, crossings[1:]):
            midpoint = a + ((left + right) / 2) * delta
            index = tuple(int(v) for v in np.floor(midpoint))
            low, high = spans.get(index, (left, right))
            spans[index] = (min(low, left), max(high, right))
        spans.pop(self.index(start), None)
        spans.pop(self.index(end), None)
        # Every remaining cell is crossed face to face, so its chord depends
        # only on the line, not on the ray's range or where it started.
        return {index: tuple((a + t * delta - index).tolist() for t in span)
                for index, span in spans.items()}

    @staticmethod
    def chord_keys(chord):
        """Quantized keys a chord within CHORD_TOLERANCE of this one may hold."""
        options = []
        for value in chord[0] + chord[1]:
            scaled = value / CHORD_QUANTUM
            key = math.floor(scaled)
            near = CHORD_TOLERANCE / CHORD_QUANTUM
            options.append((key, key - 1) if scaled - key < near else
                           (key, key + 1) if scaled - key > 1 - near else (key,))
        return product(*options)

    def count_free_chord(self, cell, chord):
        """Count a chord once per cell, in either direction."""
        for candidate in (chord, chord[::-1]):
            if any(key in cell.free_chords for key in self.chord_keys(candidate)):
                return
        cell.free_chords.add(next(iter(self.chord_keys(chord))))
        cell.free_ray_count += 1

    def add_scan(self, scan):
        if self.timestamps and scan.timestamp <= self.timestamps[-1]:
            raise ValueError("scan timestamps must be strictly increasing")
        # Accumulation is monotonic: a later ray never erases a prior hit.
        for hit in scan.hits:
            # A ray retracing a chord already seen in a cell (repeated, a
            # collinear return at another range, or reversed) samples the same
            # line again, not more of the voxel: it counts once toward the
            # free-ray minimum. Its hit is still recorded below, so occupancy
            # evidence is never dropped.
            for index, chord in self.ray_chords(scan.origin, hit).items():
                cell = self.cell(index)
                cell.free_times.add(scan.timestamp)
                self.count_free_chord(cell, chord)
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
