"""Checker defaults.

The 10 cm return rule is the milestone rule. The gap limit and the minimum
motion are checker rules so a broken or parked trajectory cannot pass. Trevor
has not approved those two, or the wall gate.
"""

STATIONARY_WINDOW_S = 5.0
MAX_GAP_S = 1.0
RETURN_PASS_M = 0.10
MIN_PATH_LENGTH_M = 2.0
MIN_EXCURSION_M = 0.5
MIN_WINDOW_SAMPLES = 3
MIN_POSES = 6

WALL_VOXEL_M = 0.30
WALL_PLANARITY_MIN = 0.55
WALL_MAX_ABS_NZ = 0.35
WALL_MIN_CELL_POINTS = 25
WALL_GATE_CM = 5.0
WALL_DOUBLE_MIN_M = 0.05
WALL_DOUBLE_MAX_M = 0.40
WALL_LATERAL_MAX_M = 0.20
WALL_LAYER_P95_MAX_M = 0.02
WALL_LAYER_MIN_FRACTION = 0.20
WALL_PARALLEL_DEG = 12.0

PROPOSED_WALL_LABEL = "proposed, not yet approved by Trevor"
HISTORICAL_LABEL = "replay demonstrated; physical accuracy not evaluated"
SCHEMA = "spatio.walkcheck.v1"
