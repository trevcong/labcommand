"""One parser for room-scale metres and seconds.

Gates, windows, durations, half-extents, and tape residuals all use
parse_quantity. Callers do not keep their own finite or magnitude checks.
"""

from __future__ import annotations

import math

from spatio.expspec.constants import (
    ABSURD_MAX_M,
    ABSURD_MAX_S,
    ABSURD_MIN_POSITIVE,
)

_MAX_BY_UNIT = {
    "m": ABSURD_MAX_M,
    "s": ABSURD_MAX_S,
}


def parse_quantity(value, unit):
    """Return (number, None) or (None, reason).

    reason is "not_a_number", "not_finite", or "absurd".
    Zero is allowed. Negative values are allowed here; the caller decides
    whether the sign is legal. A non-zero magnitude below ABSURD_MIN_POSITIVE,
    or a magnitude above the unit cap, is absurd.
    """
    if unit not in _MAX_BY_UNIT:
        raise ValueError("unknown unit")
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None, "not_a_number"
    number = float(value)
    if not math.isfinite(number):
        return None, "not_finite"
    magnitude = abs(number)
    if magnitude > _MAX_BY_UNIT[unit]:
        return None, "absurd"
    if number != 0.0 and magnitude < ABSURD_MIN_POSITIVE:
        return None, "absurd"
    return number, None


def windows_are_disjoint(start_s, end_s, duration_low_s):
    """True when the two endpoint windows share no time.

    The start window is the first start_s seconds. The end window is the
    last end_s seconds. On a run of duration D they overlap or touch when
    start_s + end_s >= D. The shortest run in the duration band is the
    tightest case. Touching at one instant is not disjoint.
    """
    return float(start_s) + float(end_s) < float(duration_low_s) - 1e-6


def format_quantity(number):
    """Text that round-trips and does not collapse a gate to two decimals.

    0.10 stays 0.10. 0.096 stays 0.096. 0.006 stays 0.006.
    """
    number = float(number)
    two = "{0:.2f}".format(number)
    if float(two) == number:
        return two
    text = format(number, ".16g")
    if float(text) != number:
        text = repr(number)
    return text
