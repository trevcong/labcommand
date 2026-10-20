"""Blank experiment spec. Every S5 key is present. Unknowns are TODO(...)."""

from __future__ import annotations

from spatio.expspec.constants import (
    ACCEPTANCE_STATES,
    EXTRA_FORBIDDEN,
    PROPOSED_THRESHOLDS,
    REQUIRED_FORBIDDEN,
    SPEC_SCHEMA,
)


def build_template():
    """Return a new spec object.

    Proposed numbers from the workplan are filled in, because they are not
    physical facts about the room. Anything Trevor or Claude still has to
    decide is the string TODO(who), which validate() rejects.
    """
    return {
        "schema": SPEC_SCHEMA,
        "spec_id": "TODO(Trevor)",
        "sensor": "TODO(Trevor)",
        "fastlio_revision": "TODO(Claude)",
        "config_file": "TODO(Claude)",
        "config_sha256": "TODO(Claude)",
        "route": {
            "description": "TODO(Trevor)",
            "duration_target_s": [180, 300],
            "intermediate_stops": ["TODO(Trevor): stop A", "TODO(Trevor): stop B"],
        },
        "reference_point": "TODO(Trevor)",
        "start_marks": "TODO(Trevor)",
        "frames": {
            "map": "camera_init",
            "body": "body",
            "lidar_to_imu": "TODO(Claude)",
            "cart_mount": "TODO(Trevor)",
        },
        "wall_patches": [
            {
                "name": "TODO(Trevor): patch A",
                "description": "TODO(Trevor)",
                "box_map_frame": None,
            },
            {
                "name": "TODO(Trevor): patch B",
                "description": "TODO(Trevor)",
                "box_map_frame": None,
            },
        ],
        "stationary_windows_s": {"start": 10, "end": 10},
        "acceptance": {
            "return_error_threshold_m": PROPOSED_THRESHOLDS["return_error_threshold_m"],
            "placement_tolerance_m": PROPOSED_THRESHOLDS["placement_tolerance_m"],
            "wall_p95_gate_m": PROPOSED_THRESHOLDS["wall_p95_gate_m"],
            "revisit_separation_gate_m": PROPOSED_THRESHOLDS["revisit_separation_gate_m"],
            "runs_required": 3,
            "state": ACCEPTANCE_STATES[0],
        },
        "forbidden": list(REQUIRED_FORBIDDEN) + list(EXTRA_FORBIDDEN),
    }
