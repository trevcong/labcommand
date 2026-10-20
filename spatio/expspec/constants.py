"""Fixed names and proposed numbers for the milestone-1 experiment spec."""

from __future__ import annotations

SPEC_SCHEMA = "spatio.experiment_spec/1"
REFERENCE_SCHEMA = "spatio.physical_reference/1"

# Workplan step 1 and step 6. A spec may differ; validate() then warns.
# The 0.10 m return error is the milestone rule.
# Trevor's delegate approved the rest as provisional (approved for now,
# revisit after real data): the 1 cm placement tolerance and the 5 cm
# wall gates. The 5 s window minimum below is provisional on the same terms.
PROPOSED_THRESHOLDS = {
    "return_error_threshold_m": 0.10,
    "placement_tolerance_m": 0.01,
    "wall_p95_gate_m": 0.05,
    "revisit_separation_gate_m": 0.05,
}

PROVISIONAL_LABEL = "provisional (approved for now, revisit after real data)"

# One magnitude cap, used only by numbers.parse_quantity.
# Smaller than this (and not zero) is below a micrometre or a microsecond.
ABSURD_MIN_POSITIVE = 1e-6
# A kilometre, or a hold longer than a day, is not a room-walk quantity.
ABSURD_MAX_M = 1000.0
ABSURD_MAX_S = 100000.0

# Exact list items. A longer sentence that only contains the phrase does not count.
REQUIRED_FORBIDDEN = (
    "end-onto-start alignment",
    "post-hoc loop correction",
    "smoothing away failures",
)

# Extra default. Not required, so a spec with only the three phrases still freezes.
EXTRA_FORBIDDEN = (
    "sliding the stationary window to shrink the error",
)

# Provisional: approved for now, revisit after real data.
MIN_STATIONARY_S = 5.0
MIN_RUNS = 3
MIN_STOPS = 2
MIN_PATCHES = 2
MAX_RENDER_LINES = 70

PROPOSED_BANNER = "PROPOSED — not yet approved by Trevor"
APPROVED_BANNER = "APPROVED by Trevor."

ACCEPTANCE_STATES = ("proposed", "approved_by_trevor")
HEADING_VALUES = ("ok", "off", "not_checked")

# Return-error sentence. Tests look for these clauses.
RETURN_ERROR_SENTENCE = (
    "The return error is measured from the fixed stationary windows, "
    "with no end-onto-start alignment, and heading is reported separately."
)

TOP_LEVEL_KEYS = (
    "schema",
    "spec_id",
    "sensor",
    "fastlio_revision",
    "config_file",
    "config_sha256",
    "route",
    "reference_point",
    "start_marks",
    "frames",
    "wall_patches",
    "stationary_windows_s",
    "acceptance",
    "forbidden",
)

OPTIONAL_TOP_LEVEL_KEYS = ("notes",)

ROUTE_KEYS = ("description", "duration_target_s", "intermediate_stops")
FRAME_KEYS = ("map", "body", "lidar_to_imu", "cart_mount")
STATIONARY_KEYS = ("start", "end")
ACCEPTANCE_KEYS = (
    "return_error_threshold_m",
    "placement_tolerance_m",
    "wall_p95_gate_m",
    "revisit_separation_gate_m",
    "runs_required",
    "state",
)
PATCH_KEYS = ("name", "description", "box_map_frame")
BOX_KEYS = ("center_m", "normal_hint", "half_extent_m")

REFERENCE_KEYS = (
    "schema",
    "run_id",
    "spec_id",
    "spec_sha256",
    "operator",
    "measured_at",
    "start_placement_residual_m",
    "end_placement_residual_m",
    "heading_check",
    "placement_tolerance_m",
    "method",
    "notes",
)
