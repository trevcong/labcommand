"""Checker defaults.

The 10 cm return rule is the milestone rule.

Trevor's delegate approved these as provisional (approved for now, revisit
after real data): the 5 cm wall gate, the 5 s window, the 1 s gap, and the
must-move minimums (2 m path, 0.5 m excursion).

The endpoint spread and speed limits are a separate choice. A window that
is still moving can average back to the start and look closed. They were
not in that approved list.

Measured on 2026-10-19, before these two limits were set, from the Mid-360
replay sessions on disk (not taped holds): endpoint spreads 0.61 cm,
2.19 cm, and 2.44 cm, and speeds up to 4.74 cm/s. The case Codex reproduced
travels from -30 cm to +30 cm inside the end window (spread 30 cm, about
12 cm/s).
"""

STATIONARY_WINDOW_S = 5.0
MAX_GAP_S = 1.0
RETURN_PASS_M = 0.10
MIN_PATH_LENGTH_M = 2.0
MIN_EXCURSION_M = 0.5
MIN_WINDOW_SAMPLES = 3
MIN_POSES = 6
# Largest distance from a sample to its window mean. Over this, the mean
# is not a parked position. 5 cm is above the measured replay jitter and
# well below the 30 cm cancellation.
MAX_ENDPOINT_SPREAD_M = 0.05
# Path length inside the window divided by that window's own duration.
# 10 cm/s is above the measured replay speeds and below the 12 cm/s case.
MAX_ENDPOINT_SPEED_M_S = 0.10

WALL_VOXEL_M = 0.30
WALL_PLANARITY_MIN = 0.55
WALL_MAX_ABS_NZ = 0.35
WALL_MIN_CELL_POINTS = 25
WALL_GATE_CM = 5.0
WALL_DOUBLE_MIN_M = 0.05
# Kept as one term of the normal-fit distance below. It is not a cap on
# doubled sheets. Capping them at 40 cm threw away a gap the checker had
# already measured, and the wall gate passed.
WALL_DOUBLE_MAX_M = 0.40
# When a wall runs along the voxel diagonal, one cell is about as deep as
# it is wide, so its own normal is noise and the two sheets are dropped
# before pairing. The second look fits the normal on points out to this
# radius: one cell plus the old 40 cm figure. Pairing looks one cell past
# that. A stacked pair inside the search and over 5 cm is a double.
# Farther than the search, the checker is not looking, and two sharp
# sheets can still be one wall seen twice. Provisional, like the 5 cm
# gate. Not fitted to a recording.
WALL_NORMAL_SUPPORT_M = WALL_VOXEL_M + WALL_DOUBLE_MAX_M
WALL_LATERAL_MAX_M = 0.20
WALL_LAYER_P95_MAX_M = 0.02
WALL_LAYER_MIN_FRACTION = 0.20
WALL_PARALLEL_DEG = 12.0

PROVISIONAL_LABEL = "provisional (approved for now, revisit after real data)"
# Same words as PROVISIONAL_LABEL. The old name stays so wall reports and
# tests keep one label field.
PROPOSED_WALL_LABEL = PROVISIONAL_LABEL
ENDPOINT_STATIONARITY_LABEL = (
    "Provisional, and not part of the delegate's approved list. "
    "Revisit after a taped hold."
)
HISTORICAL_LABEL = "replay demonstrated; physical accuracy not evaluated"
SCHEMA = "spatio.walkcheck.v1"
