"""The one home for the walk-day acceptance gates.

Every stage that applies these gates imports them from here: walkcheck,
wallpatch measure, spatio.wall_scoring, spatio.series and spatio.expspec.
walkday.conf's WALL_GATE_M reaches walkcheck, wallpatch measure and
spatio.walkday.result through --wall-gate-m; WALL_GATE_M here is the default
when no gate is passed. No other module may hold its own copy of a number.

This module imports nothing from spatio, so any module can import it.
"""

import math

# Provisional (approved for now, revisit after real data): Trevor's delegate
# approved the 5 cm wall gate. Whole-map p95, patch p95 and patch separation
# all use it.
WALL_GATE_M = 0.05
WALL_GATE_CM = round(WALL_GATE_M * 100.0, 9)
WALL_GATE_STATE = "approved"


# Provisional (Trevor, 2026-10-20): guard band under the wall gate. On a
# patch that would otherwise pass, p95 and both 80% bands must be at most this.
# Above it and up to the gate the patch is not checkable (never a pass); p95
# or separation over the gate still fails. Revisit after real walks.
GUARD_BAND_M = 0.040
GUARD_BAND_LABEL = "provisional guard band (Trevor 2026-10-20, revisit after real walks)"

# The milestone return-error gate (workplan rule): a 3D return error must be
# strictly under this. It is fixed by the milestone and has no setting.
RETURN_GATE_M = 0.10


def wall_gate(value):
    """Parse a stricter-only wall gate, including CLI/settings strings."""
    if isinstance(value, bool):
        raise ValueError("wall gate must be a number")
    gate = float(value)
    if not math.isfinite(gate) or not 0 < gate <= WALL_GATE_M:
        raise ValueError(f"wall gate must be finite and > 0 and <= {WALL_GATE_M:g} m (stricter-only)")
    return gate
