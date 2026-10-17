"""Render the score dict. The markdown is generated from the numbers, not hand-edited."""

import json
from pathlib import Path


def render_markdown(report):
    lines = [
        "# Room-memory acceptance",
        "",
        "CANDIDATE score. This is not the CONFIRMED product bar "
        "(false CONFIRMED events per hour in a static room).",
        "",
        f"g5_acceptance: {report['g5_acceptance']}",
        f"candidate_result: {report['candidate_result']}",
        f"confirmed_product_bar: {report['confirmed_product_bar']}",
        f"exit_code: {report['exit_code']}",
        f"true_detections: {report['true_detections']}",
        f"temporal_matches: {report['temporal_matches']}",
        f"misses: {report['misses']}",
        f"false_alarms: {report['false_alarms']}",
        f"idle_false_candidate_alarms_per_hour: {report['idle_false_candidate_alarms_per_hour']}",
        f"credited_idle_exposure_s: {report['credited_idle_exposure_s']}",
        f"change_as_activity: {report['change_as_activity']}",
        f"miss_with_activity_observed: {report['miss_with_activity_observed']}",
        f"activity_as_change: {report['activity_as_change']}",
        f"person_unverified: {report['person_unverified']}",
        f"split_move: {report['split_move']}",
        f"dropped_windows: {report['dropped_windows']}",
        f"session_dropped_windows: {report['session_dropped_windows']}",
        f"session_window_reasons: {', '.join(report['session_window_reasons']) or 'none'}",
        f"lost_window_scenarios: {', '.join(report['lost_window_scenarios']) or 'none'}",
        f"confirmed_count: {report['confirmed_count']}",
        f"detection_delay_basis: {report['detection_delay_basis']}",
        f"receipt_delay_p50_s: {report['receipt_delay_s']['p50']}",
        f"receipt_delay_p95_s: {report['receipt_delay_s']['p95']}",
        f"receipt_delay_max_s: {report['receipt_delay_s']['max']}",
        "",
        report["confirmed_product_bar_note"],
        "",
        "| scenario | completion | actual_s | protocol_s | coverage | detection | discrimination |",
        "| --- | --- | --- | --- | --- | --- | --- |",
    ]
    for row in report["scenarios"]:
        lines.append(
            "| {id} | {completion} | {actual_duration_s} | {protocol_duration_s} | {coverage} | {detection} | {discrimination} |".format(**row)
        )
    lines.append("")
    return "\n".join(lines)


def write_report(report, directory):
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    body = dict(report)
    markdown = body.pop("markdown")
    (directory / "report.json").write_text(json.dumps(body, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    (directory / "report.md").write_text(markdown, encoding="utf-8")
    return None
