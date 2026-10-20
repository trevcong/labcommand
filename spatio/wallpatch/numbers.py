"""One rule for numbers from maps, patch files, and the command line.

The limits are provisional safety bounds. They are not the 5 cm wall gates.
A coordinate past 10 km, a non-finite value, or a count this tool will not
store is rejected here. Callers do not repeat the comparison.
"""

from __future__ import annotations

import math

import numpy as np

from spatio.wallpatch.errors import WallpatchError

# Provisional. Positions within 10 km are the delegate's bound.
MAX_ABS_METRES = 10000.0
# Above the 3,544,568-point Mid-360 map. Provisional.
MAX_COUNT = 50000000
# COUNT on one PCD field. An ascii descriptor of length 3 must fit.
MAX_FIELD_COUNT = 16
# How many patches suggest may be asked to write. Provisional.
MAX_SUGGEST_N = 1000
# Search voxel. The default is 0.08 m. Provisional.
MIN_VOXEL_M = 0.001
MAX_VOXEL_M = 10.0
_PCD_SIZES = (1, 2, 4, 8)

_RANGE = "outside the accepted range"


def parse_real(value, name, limit=MAX_ABS_METRES):
    """Finite float with absolute value <= limit. Rejects bools and junk text."""
    number = _coerce_real(value, name)
    if not math.isfinite(number) or abs(number) > float(limit):
        raise WallpatchError(
            "%s is %s (provisional limit %s)" % (name, _RANGE, _format_limit(limit))
        )
    return number


def parse_positive_metres(value, name):
    """A length in metres: finite, positive, and within the provisional cap."""
    number = parse_real(value, name, MAX_ABS_METRES)
    if number <= 0.0:
        raise WallpatchError("%s must be positive" % name)
    return number


def parse_count(value, name, lower=0, upper=MAX_COUNT):
    """Integer in [lower, upper]. Floats and scientific tokens are rejected."""
    number = _coerce_int(value, name)
    if number < int(lower) or number > int(upper):
        raise WallpatchError(
            "%s is %s (provisional limits %d to %d)"
            % (name, _RANGE, int(lower), int(upper))
        )
    return number


def parse_suggest_n(value):
    return parse_count(value, "--n", lower=1, upper=MAX_SUGGEST_N)


def parse_voxel_m(value):
    number = parse_real(value, "voxel size", limit=MAX_VOXEL_M)
    if number < MIN_VOXEL_M:
        raise WallpatchError(
            "voxel size is %s (provisional limits %s m to %s m)"
            % (_RANGE, MIN_VOXEL_M, MAX_VOXEL_M)
        )
    return number


def parse_pcd_size(value, name="SIZE"):
    number = parse_count(value, name, lower=1, upper=8)
    if number not in _PCD_SIZES:
        raise WallpatchError("%s is %s" % (name, _RANGE))
    return number


def parse_field_count(value, name="COUNT"):
    return parse_count(value, name, lower=1, upper=MAX_FIELD_COUNT)


def parse_point_count(value, name="POINTS"):
    return parse_count(value, name, lower=0, upper=MAX_COUNT)


def parse_point_product(width, height):
    """WIDTH times HEIGHT. Rejected when the product exceeds the point cap."""
    product = int(width) * int(height)
    if product > MAX_COUNT:
        raise WallpatchError(
            "POINTS is %s (provisional limits 0 to %d)" % (_RANGE, MAX_COUNT)
        )
    return product


def reject_xyz(points):
    """Raise if any coordinate fails parse_real's provisional metre limit.

    This is the vectorized form used for binary chunks. The limit and the
    error text match parse_real.
    """
    arr = np.asarray(points, dtype=np.float64)
    if arr.size == 0:
        return
    finite = np.isfinite(arr).all()
    if not finite or float(np.max(np.abs(arr))) > MAX_ABS_METRES:
        raise WallpatchError(
            "PCD coordinate is %s (provisional limit %s m)"
            % (_RANGE, _format_limit(MAX_ABS_METRES))
        )


def _coerce_real(value, name):
    if isinstance(value, (bool, np.bool_)):
        raise WallpatchError("%s is not a real number" % name)
    if isinstance(value, (bytes, bytearray)):
        try:
            value = value.decode("ascii")
        except UnicodeError:
            raise WallpatchError("%s is not a real number" % name)
    if isinstance(value, str):
        try:
            return float(value.strip())
        except ValueError:
            raise WallpatchError("%s is not a real number" % name)
    if isinstance(value, (int, float, np.floating, np.integer)):
        return float(value)
    raise WallpatchError("%s is not a real number" % name)


def _coerce_int(value, name):
    if isinstance(value, (bool, np.bool_)):
        raise WallpatchError("%s is not an integer" % name)
    if isinstance(value, (bytes, bytearray)):
        try:
            value = value.decode("ascii")
        except UnicodeError:
            raise WallpatchError("%s is not an integer" % name)
    if isinstance(value, str):
        text = value.strip()
        try:
            return int(text, 10)
        except ValueError:
            raise WallpatchError("%s is not an integer" % name)
    if isinstance(value, (float, np.floating)):
        raise WallpatchError("%s is not an integer" % name)
    if isinstance(value, (int, np.integer)):
        return int(value)
    raise WallpatchError("%s is not an integer" % name)


def _format_limit(limit):
    if float(limit) == int(limit):
        return "%d" % int(limit)
    return "%s" % (limit,)
