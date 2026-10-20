"""Turn validated walk results into a plain-English note and a series verdict.

Precedence, strongest first: input error, NOT_EVALUABLE, REPLAY_ONLY, PASS,
INCOMPLETE, FAIL. A weaker verdict must not hide a stronger problem.
"""

from __future__ import annotations

from decimal import Decimal, ROUND_HALF_UP
from typing import Dict, List, Optional, Sequence, Tuple

from spatio.series.validate import (
    PLACEMENT_TOLERANCE_M,
    REPLAY_LABEL,
    RETURN_THRESHOLD_M,
    WALL_GATE_M,
    InputRejected,
    WalkResult,
)

CONFIG_CHANGE_REASON = "a changed configuration starts a new series"
TREVOR_LINE = (
    "Other failed attempts do not void this series (workplan rule; Trevor to confirm)"
)
SERIES_SCHEMA = "spatio.series/1"

_TABLE_HEADERS = (
    "attempt",
    "run_id",
    "status",
    "return 3D (cm)",
    "horizontal (cm)",
    "vertical (cm)",
    "heading (°)",
    "path (m)",
    "drift (% of path)",
    "wall p95 (cm)",
    "doubled wall?",
    "placement start/end (cm)",
    "physical accuracy evaluated?",
    "counts toward series?",
    "reasons",
)

_RULES_MD = """## Rules

This report checks the producer's walk_result files again. It does not trust a PASS that the numbers do not support. Attempts are in the order given on the command line.

Precedence is input error (exit 1), then NOT_EVALUABLE, then REPLAY_ONLY, then PASS, then INCOMPLETE, then FAIL. A weaker verdict must not hide a stronger problem.

- R0. Validate each file against schema S4 and rules W1–W4. Reject a PASS that the numbers do not support. Reject a non-PASS with no reasons. The error names the file.
- R1. Every run must share config_sha256 and fastlio_revision. If not, the verdict is NOT_EVALUABLE because a changed configuration starts a new series. The table still lists every run.
- R2. A run counts only when its status is PASS, physical accuracy was evaluated, both placement residuals are measured and at or under 0.01 m, and the heading check is ok.
- R3. The series is PASS only when three consecutive attempts all count. The earliest three are named. Other non-counting attempts stay in the table and under Retained failed attempts.
- R4. If every run has physical accuracy not evaluated, the verdict is REPLAY_ONLY and the label is "replay demonstrated; physical accuracy not evaluated".
- R5. Fewer than 3 attempts, once the stronger verdicts above do not apply, is INCOMPLETE. A retained failure does not rule the series out before three attempts exist. Three or more attempts with no counting triple is FAIL.
- Boundary. A 3D return error of exactly 0.10 m is not under 0.10 m, so that run fails. Each measured component, and the combined length of the two components, uses that same strict comparison. The 1 mm consistency tolerance does not move this boundary.
- Null. A null measurement is "not measured". It is never read as zero.
- Gates. The return threshold is 0.10 m and the placement tolerance is 0.01 m (provisional). A looser number in the file is rejected. Wall p95 is shown and is not an acceptance gate. W1 uses the doubled-wall flag only. The 5 cm wall gate is provisional.
"""


def explain_text(walk: WalkResult) -> str:
    """Nine short sentences. Centimetres in the prose, null shown as not measured."""
    lines = [
        "Run %s is %s." % (walk.run_id, walk.status),
        _return_sentence(walk),
        "Horizontal return error is %s and vertical return error is %s."
        % (
            _cm_or_not(walk.return_error.horizontal_m),
            _cm_or_not(walk.return_error.vertical_m),
        ),
        "Heading error is %s." % _deg_or_not(walk.return_error.heading_deg),
        _duration_sentence(walk),
        _validity_sentence(walk),
        _wall_sentence(walk),
        _physical_sentence(walk),
        _counts_sentence(walk),
    ]
    if not 5 <= len(lines) <= 10:
        raise RuntimeError("explain produced %s sentences" % len(lines))
    return "\n".join(lines) + "\n"


def build_series(walks: Sequence[WalkResult], series_id: str) -> dict:
    """Judge the runs in the order given. Raises InputRejected on duplicate ids."""
    if not walks:
        raise InputRejected(series_id, "compare needs at least one walk_result")
    if not isinstance(series_id, str) or not series_id.strip():
        raise InputRejected(walks[0].source_path, "series_id must be a non-empty string")
    _reject_duplicate_ids(walks)
    verdict, counted, reasons = _judge(walks)
    retained = [walk.run_id for walk in walks if not counts_toward_series(walk)]
    return {
        "schema": SERIES_SCHEMA,
        "series_id": series_id,
        "verdict": verdict,
        "counted_run_ids": counted,
        "retained_failed": retained,
        "reasons": reasons,
        "runs": [_run_dict(walk, index) for index, walk in enumerate(walks, start=1)],
    }


def render_markdown(report: dict) -> str:
    """Human report. Rules are included so the verdict can be checked without the code."""
    lines = [
        "# Spatio series %s" % report["series_id"],
        "",
        "Verdict: %s" % report["verdict"],
        "",
    ]
    counted = report["counted_run_ids"]
    if counted:
        lines.append("Counted runs: %s" % ", ".join(counted))
    else:
        lines.append("Counted runs: none")
    lines.append("")
    if report["verdict"] == "REPLAY_ONLY":
        lines.append("Label: %s" % REPLAY_LABEL)
        lines.append("")
    lines.append(_RULES_MD.rstrip())
    lines.append("")
    lines.append("## Comparison")
    lines.append("")
    lines.append(_table_row(_TABLE_HEADERS))
    lines.append(_table_row("---" for _ in _TABLE_HEADERS))
    for run in report["runs"]:
        lines.append(_table_row(_table_cells(run)))
    lines.append("")
    lines.append("## Retained failed attempts")
    lines.append("")
    retained = report["retained_failed"]
    if not retained:
        lines.append("None.")
    else:
        by_id = {run["run_id"]: run for run in report["runs"]}
        for run_id in retained:
            run = by_id[run_id]
            why = "; ".join(run["count_blockers"]) or "does not count"
            lines.append("- attempt %s, %s: %s" % (run["attempt"], run_id, why))
    lines.append("")
    if TREVOR_LINE in report["reasons"]:
        lines.append(TREVOR_LINE)
        lines.append("")
    if _any_provisional_wall_gate(report):
        lines.append("The 5 cm wall gate is provisional.")
        lines.append("")
    elif _any_proposed_gate(report):
        lines.append("The wall gate is proposed, not yet approved by Trevor.")
        lines.append("")
    elif report["runs"] and _all_approved_gate(report):
        lines.append("The wall gate is approved by Trevor.")
        lines.append("")
    lines.append("## Reasons")
    lines.append("")
    if report["reasons"]:
        for reason in report["reasons"]:
            lines.append("- %s" % reason)
    else:
        lines.append("None.")
    lines.append("")
    return "\n".join(lines)


def default_series_id(walks: Sequence[WalkResult]) -> str:
    first = walks[0].run_id
    last = walks[-1].run_id
    if first == last:
        return first
    return "%s--%s" % (first, last)


def counts_toward_series(walk: WalkResult) -> bool:
    return not count_blockers(walk)


def count_blockers(walk: WalkResult) -> List[str]:
    """Why this run does not count. Empty means it counts (rule R2)."""
    blockers = []
    if walk.status != "PASS":
        blockers.append("status is %s" % walk.status)
    if not walk.physical_accuracy_evaluated:
        blockers.append("physical accuracy was not evaluated")
        return blockers
    reference = walk.physical_reference
    if reference is None:
        blockers.append("physical reference is missing")
        return blockers
    tolerance = min(reference.placement_tolerance_m, PLACEMENT_TOLERANCE_M)
    start = reference.start_placement_residual_m
    end = reference.end_placement_residual_m
    if start is None:
        blockers.append("start placement residual was not measured")
    elif start > tolerance:
        blockers.append(
            "start placement residual is %s, over the %s tolerance"
            % (_format_cm(start), _format_cm(tolerance))
        )
    if end is None:
        blockers.append("end placement residual was not measured")
    elif end > tolerance:
        blockers.append(
            "end placement residual is %s, over the %s tolerance"
            % (_format_cm(end), _format_cm(tolerance))
        )
    if reference.heading_check != "ok":
        blockers.append("heading check is %s" % reference.heading_check)
    return blockers


def status_note(walk: WalkResult) -> Optional[str]:
    """Visible when a producer marked FAIL even though W1 would allow PASS.

    The series does not upgrade that status. The note stops the downgrade
    from being silent.
    """
    if walk.status != "PASS" and walk.w1_supports_pass():
        return (
            "Status is %s, but the return error, validity flags, and doubled-wall "
            "flag would allow PASS. This tool does not upgrade status." % walk.status
        )
    return None


def _judge(walks: Sequence[WalkResult]) -> Tuple[str, List[str], List[str]]:
    configs = {walk.config_sha256 for walk in walks}
    revisions = {walk.fastlio_revision for walk in walks}
    if len(configs) > 1 or len(revisions) > 1:
        reasons = [CONFIG_CHANGE_REASON]
        if len(configs) > 1:
            reasons.append(
                "config_sha256 values differ: %s." % ", ".join(sorted(configs))
            )
        if len(revisions) > 1:
            reasons.append(
                "fastlio_revision values differ: %s." % ", ".join(sorted(revisions))
            )
        reasons.append(
            "Per-run counting flags are still shown. They do not make this series a PASS."
        )
        return "NOT_EVALUABLE", [], reasons

    if all(not walk.physical_accuracy_evaluated for walk in walks):
        return "REPLAY_ONLY", [], [REPLAY_LABEL]

    triple = _earliest_triple(walks)
    if triple is not None:
        counted = [walks[index].run_id for index in triple]
        reasons = ["Three consecutive attempts count: %s." % ", ".join(counted)]
        outside = [
            walk.run_id
            for walk in walks
            if counts_toward_series(walk) and walk.run_id not in counted
        ]
        if outside:
            reasons.append(
                "Counting attempts outside that triple stay in the table: %s."
                % ", ".join(outside)
            )
        retained = [walk.run_id for walk in walks if not counts_toward_series(walk)]
        if retained:
            reasons.append(TREVOR_LINE)
        return "PASS", counted, reasons

    if len(walks) < 3:
        if len(walks) == 1:
            lead = "Only 1 attempt was given."
        else:
            lead = "Only %s attempts were given." % len(walks)
        reasons = [
            lead
            + " A series needs 3 consecutive counting runs, and nothing here rules the series out."
        ]
        reasons.extend(_blocker_lines(walks))
        return "INCOMPLETE", [], reasons

    reasons = ["No three consecutive attempts count toward the series."]
    reasons.extend(_blocker_lines(walks))
    return "FAIL", [], reasons


def _earliest_triple(walks: Sequence[WalkResult]) -> Optional[Tuple[int, int, int]]:
    flags = [counts_toward_series(walk) for walk in walks]
    last_start = len(flags) - 2
    for index in range(last_start):
        if flags[index] and flags[index + 1] and flags[index + 2]:
            return (index, index + 1, index + 2)
    return None


def _blocker_lines(walks: Sequence[WalkResult]) -> List[str]:
    lines = []
    for walk in walks:
        blockers = count_blockers(walk)
        if blockers:
            lines.append(
                "%s does not count: %s." % (walk.run_id, "; ".join(blockers))
            )
        note = status_note(walk)
        if note:
            lines.append(note)
    return lines


def _reject_duplicate_ids(walks: Sequence[WalkResult]) -> None:
    seen: Dict[str, str] = {}
    for walk in walks:
        previous = seen.get(walk.run_id)
        if previous is not None:
            raise InputRejected(
                walk.source_path,
                "run_id %s is also used by %s" % (walk.run_id, previous),
            )
        seen[walk.run_id] = walk.source_path


def _run_dict(walk: WalkResult, attempt: int) -> dict:
    reference = walk.physical_reference
    if reference is None:
        reference_json = None
    else:
        reference_json = {
            "start_placement_residual_m": reference.start_placement_residual_m,
            "end_placement_residual_m": reference.end_placement_residual_m,
            "heading_check": reference.heading_check,
            "placement_tolerance_m": reference.placement_tolerance_m,
        }
    error = walk.return_error
    return {
        "attempt": attempt,
        "run_id": walk.run_id,
        "status": walk.status,
        "reasons": list(walk.reasons),
        "physical_accuracy_evaluated": walk.physical_accuracy_evaluated,
        "accuracy_label": walk.accuracy_label,
        "fastlio_revision": walk.fastlio_revision,
        "config_sha256": walk.config_sha256,
        "duration_s": walk.duration_s,
        "path_length_m": walk.path_length_m,
        "return_error": {
            "d3_m": error.d3_m,
            "horizontal_m": error.horizontal_m,
            "vertical_m": error.vertical_m,
            "heading_deg": error.heading_deg,
            "threshold_m": error.threshold_m,
            "start_window_s": error.start_window_s,
            "end_window_s": error.end_window_s,
        },
        "drift_pct_of_path": walk.drift_pct_of_path,
        "validity": {
            "timestamps_monotonic": walk.validity.timestamps_monotonic,
            "resets": walk.validity.resets,
            "max_gap_s": walk.validity.max_gap_s,
            "moving": walk.validity.moving,
        },
        "walls": {
            "p95_point_to_plane_m": walk.walls.p95_point_to_plane_m,
            "doubled_wall_suspected": walk.walls.doubled_wall_suspected,
            "gate_m": walk.walls.gate_m,
            "gate_state": walk.walls.gate_state,
        },
        "physical_reference": reference_json,
        "counts_toward_series": counts_toward_series(walk),
        "count_blockers": count_blockers(walk),
        "status_note": status_note(walk),
        "source_file": walk.source_path,
    }


def _return_sentence(walk: WalkResult) -> str:
    d3 = walk.return_error.d3_m
    threshold = walk.return_error.threshold_m
    d3_text = _cm_or_not(d3)
    threshold_text = _cm_or_not(threshold)
    if d3 is None or threshold is None:
        return "The 3D return error is %s and the threshold is %s." % (
            d3_text,
            threshold_text,
        )
    # Compare against the milestone gate. Validation already rejected other thresholds.
    if d3 < RETURN_THRESHOLD_M:
        relation = "under"
    else:
        relation = "not under"
    return "The 3D return error is %s, which is %s the %s threshold." % (
        d3_text,
        relation,
        threshold_text,
    )


def _duration_sentence(walk: WalkResult) -> str:
    if walk.duration_s is None:
        duration = "not measured"
    else:
        duration = "%s seconds" % _format_fixed(walk.duration_s, 1)
    if walk.path_length_m is None:
        path = "not measured"
    else:
        path = "%s m" % _format_fixed(walk.path_length_m, 1)
    if walk.drift_pct_of_path is None:
        drift = "drift is not measured"
    else:
        drift = "drift is %s%% of the path" % _format_fixed(walk.drift_pct_of_path, 2)
    return "Duration is %s, path length is %s, and %s." % (duration, path, drift)


def _validity_sentence(walk: WalkResult) -> str:
    validity = walk.validity
    if validity.timestamps_monotonic is True:
        monotonic = "Timestamps are monotonic"
    elif validity.timestamps_monotonic is False:
        monotonic = "Timestamps are not monotonic"
    else:
        monotonic = "Timestamps were not measured"
    if validity.resets is None:
        resets = "resets were not measured"
    else:
        resets = "resets are %s" % validity.resets
    if validity.max_gap_s is None:
        gap = "the largest gap is not measured"
    else:
        gap = "the largest gap is %s seconds" % _format_fixed(validity.max_gap_s, 2)
    if validity.moving is True:
        moving = "the cart was moving"
    elif validity.moving is False:
        moving = "the cart was not moving"
    else:
        moving = "whether the cart was moving was not measured"
    return "%s, %s, %s, and %s." % (monotonic, resets, gap, moving)


def _wall_sentence(walk: WalkResult) -> str:
    walls = walk.walls
    if walls.p95_point_to_plane_m is None:
        p95 = "The wall p95 point-to-plane residual was not measured"
    else:
        p95 = "The wall p95 point-to-plane residual is %s" % _format_cm(
            walls.p95_point_to_plane_m
        )
    if walls.doubled_wall_suspected is True:
        doubled = "a doubled wall is suspected"
    elif walls.doubled_wall_suspected is False:
        doubled = "a doubled wall is not suspected"
    else:
        doubled = "a doubled wall was not measured"
    gate = "the wall gate of %s is %s" % (
        _format_cm(walls.gate_m),
        _gate_phrase(walls.gate_m, walls.gate_state),
    )
    return "%s, %s, and %s." % (p95, doubled, gate)


def _gate_phrase(gate_m: float, gate_state: str) -> str:
    if abs(gate_m - WALL_GATE_M) <= 1e-9:
        return "provisional"
    if gate_state == "proposed":
        return "proposed, not yet approved by Trevor"
    return "approved by Trevor"


def _physical_sentence(walk: WalkResult) -> str:
    if not walk.physical_accuracy_evaluated:
        return (
            'Physical accuracy was not evaluated, and the label is "%s".' % walk.accuracy_label
        )
    reference = walk.physical_reference
    if reference is None:
        return "Physical accuracy was evaluated, and the physical reference is missing."
    return (
        "Physical accuracy was evaluated: the start placement residual is %s, "
        "the end placement residual is %s, the placement tolerance is %s (provisional), "
        "and the heading check is %s."
        % (
            _cm_or_not(reference.start_placement_residual_m),
            _cm_or_not(reference.end_placement_residual_m),
            _format_cm(min(reference.placement_tolerance_m, PLACEMENT_TOLERANCE_M)),
            reference.heading_check,
        )
    )


def _counts_sentence(walk: WalkResult) -> str:
    blockers = count_blockers(walk)
    if not blockers:
        if walk.reasons:
            return (
                "This run counts toward a three-run series; recorded reasons: %s."
                % "; ".join(walk.reasons)
            )
        return "This run counts toward a three-run series."
    detail = []
    note = status_note(walk)
    for blocker in blockers:
        if note and blocker.startswith("status is "):
            continue
        detail.append(blocker)
    if note:
        detail.append(
            "the return error, validity flags, and doubled-wall flag would allow PASS, "
            "and this tool does not upgrade status"
        )
    if walk.reasons:
        detail.append("recorded reasons: %s" % "; ".join(walk.reasons))
    return "This run does not count toward a three-run series: %s." % "; ".join(detail)


def _table_cells(run: dict) -> List[str]:
    error = run["return_error"]
    walls = run["walls"]
    return [
        str(run["attempt"]),
        _cell(run["run_id"]),
        run["status"],
        _cm_cell(error["d3_m"]),
        _cm_cell(error["horizontal_m"]),
        _cm_cell(error["vertical_m"]),
        _deg_cell(error["heading_deg"]),
        _metre_cell(run["path_length_m"]),
        _drift_cell(run["drift_pct_of_path"]),
        _cm_cell(walls["p95_point_to_plane_m"]),
        _yes_no_null(walls["doubled_wall_suspected"]),
        _placement_cell(run["physical_reference"]),
        "yes" if run["physical_accuracy_evaluated"] else "no",
        "yes" if run["counts_toward_series"] else "no",
        _reasons_cell(run),
    ]


def _reasons_cell(run: dict) -> str:
    parts: List[str] = []
    parts.extend(run["count_blockers"])
    if run["status_note"]:
        parts.append(run["status_note"])
    recorded = run["reasons"]
    if recorded:
        parts.append("recorded: %s" % "; ".join(recorded))
    if not parts:
        return "-"
    return _cell("; ".join(parts))


def _placement_cell(reference: Optional[dict]) -> str:
    if reference is None:
        return "not measured / not measured"
    return "%s / %s" % (
        _cm_cell(reference["start_placement_residual_m"]),
        _cm_cell(reference["end_placement_residual_m"]),
    )


def _yes_no_null(value: Optional[bool]) -> str:
    if value is True:
        return "yes"
    if value is False:
        return "no"
    return "not measured"


def _cm_cell(metres: Optional[float]) -> str:
    if metres is None:
        return "not measured"
    return _format_fixed(Decimal(str(metres)) * Decimal("100"), 1)


def _deg_cell(degrees: Optional[float]) -> str:
    if degrees is None:
        return "not measured"
    return _format_fixed(degrees, 1)


def _metre_cell(metres: Optional[float]) -> str:
    if metres is None:
        return "not measured"
    return _format_fixed(metres, 1)


def _drift_cell(value: Optional[float]) -> str:
    if value is None:
        return "not measured"
    return _format_fixed(value, 2)


def _cm_or_not(metres: Optional[float]) -> str:
    if metres is None:
        return "not measured"
    return _format_cm(metres)


def _deg_or_not(degrees: Optional[float]) -> str:
    if degrees is None:
        return "not measured"
    return "%s degrees" % _format_fixed(degrees, 1)


def _format_cm(metres: float) -> str:
    return "%s cm" % _format_fixed(Decimal(str(metres)) * Decimal("100"), 1)


def _format_fixed(value, decimals: int) -> str:
    quantum = Decimal("1").scaleb(-decimals)
    quantized = Decimal(str(value)).quantize(quantum, rounding=ROUND_HALF_UP)
    return format(quantized, ".%df" % decimals)


def _cell(value: str) -> str:
    return value.replace("|", "/").replace("\n", " ").replace("\r", " ")


def _table_row(cells: Sequence[str]) -> str:
    return "| " + " | ".join(cells) + " |"


def _any_provisional_wall_gate(report: dict) -> bool:
    for run in report["runs"]:
        gate = run["walls"]["gate_m"]
        if isinstance(gate, (int, float)) and not isinstance(gate, bool):
            if abs(float(gate) - WALL_GATE_M) <= 1e-9:
                return True
    return False


def _any_proposed_gate(report: dict) -> bool:
    return any(run["walls"]["gate_state"] == "proposed" for run in report["runs"])


def _all_approved_gate(report: dict) -> bool:
    return all(run["walls"]["gate_state"] == "approved" for run in report["runs"])
