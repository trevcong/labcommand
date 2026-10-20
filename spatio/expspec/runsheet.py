"""Printable sheet for the walks. Residuals are filled in centimetres."""

from __future__ import annotations

from pathlib import Path

from spatio.expspec.constants import PROPOSED_BANNER, PROVISIONAL_LABEL
from spatio.expspec.freeze import FrozenSpecError, load_frozen
from spatio.expspec.io_util import write_bytes_atomic
from spatio.expspec.numbers import format_quantity, parse_quantity


class RunsheetError(Exception):
    pass


def _one_line(value):
    return " ".join(str(value).split())


def render_runsheet(spec, spec_sha256, runs):
    if not isinstance(runs, int) or isinstance(runs, bool) or runs < 1:
        raise RunsheetError("runs must be an integer >= 1")
    acceptance = spec["acceptance"]
    windows = spec["stationary_windows_s"]
    parsed_tolerance, reason = parse_quantity(acceptance["placement_tolerance_m"], "m")
    if reason is not None or parsed_tolerance <= 0:
        raise RunsheetError("placement tolerance is not a positive room-scale number")
    tolerance_m = parsed_tolerance
    lines = []
    lines.append("# Operator run sheet: {0}".format(spec["spec_id"]))
    lines.append("")
    if acceptance.get("state") != "approved_by_trevor":
        lines.append("{0}. Do not treat these walks as scored acceptance.".format(PROPOSED_BANNER))
        lines.append("")
    lines.append("Spec sha256: {0}".format(spec_sha256))
    lines.append("Sensor: {0}. Config: {1}.".format(spec["sensor"], spec["config_file"]))
    lines.append("Route: {0}".format(_one_line(spec["route"]["description"])))
    lines.append("Reference point: {0}".format(_one_line(spec["reference_point"])))
    lines.append("Start marks: {0}".format(_one_line(spec["start_marks"])))
    lines.append("Placement tolerance: {0} cm ({1} m).".format(
        format_quantity(tolerance_m * 100.0),
        format_quantity(tolerance_m),
    ))
    lines.append("Write residuals in centimetres. The record command takes metres (divide cm by 100).")
    lines.append("Leave the residual blank if you did not measure. Blank means null, and null is not zero.")
    lines.append("Heading must be ok. off or not_checked is not within tolerance.")
    lines.append("Do not align the end pose onto the start. Heading is reported separately from the return error.")
    lines.append("Held-still checkboxes: held still {0:g} s at start / held still {1:g} s at end.".format(
        windows["start"], windows["end"]
    ))
    lines.append(
        "The 5 s window minimum, the wall gates, and the placement tolerance are {0}.".format(
            PROVISIONAL_LABEL
        )
    )
    lines.append("")
    stops = spec["route"]["intermediate_stops"]
    for number in range(1, runs + 1):
        lines.append("## Run {0} of {1}".format(number, runs))
        lines.append("")
        lines.append("Run id: ____________________")
        lines.append("Operator: ____________________")
        lines.append("Start clock: ____________________")
        lines.append("End clock: ____________________")
        lines.append("[ ] held still {0:g} s at start".format(windows["start"]))
        lines.append("[ ] held still {0:g} s at end".format(windows["end"]))
        lines.append("Start placement residual (cm): __________")
        lines.append("End placement residual (cm): __________")
        lines.append("Heading mark check (ok / off / not_checked): __________")
        lines.append("Method (what you measured with): ____________________")
        lines.append("Intermediate stops reached:")
        for stop in stops:
            lines.append("[ ] {0}".format(_one_line(stop)))
        lines.append("Notes: ____________________")
        lines.append("")
    return "\n".join(lines).rstrip("\n") + "\n"


def write_runsheet(spec_path, runs, out_path):
    """Read a frozen spec, render the sheet, and write it. Return the text."""
    try:
        spec, digest = load_frozen(spec_path)
    except FrozenSpecError as exc:
        raise RunsheetError(str(exc))
    text = render_runsheet(spec, digest, runs)
    write_bytes_atomic(Path(out_path), text.encode("utf-8"))
    return text, spec
