"""Walk-day series gate: the acceptance rules spatio.series does not know.

    python3 -m spatio.walkday.seriesgate STAGED_DIR OUT_DIR WALK_RESULT...

STAGED_DIR holds spatio.series compare's own series.json for the same files
(series.sh keeps it as <series>/spatio_series/). This writes the final
OUT_DIR/series.{md,json}. Every spatio.series key is kept; the final verdict
replaces `verdict` and `counted_run_ids`, spatio.series' own verdict stays in
`spatio_series_verdict`, and each run gains its walk-day fields.

Rules, applied on top of spatio.series R0-R5. They never upgrade a verdict:
  G1. Every run carries a valid walk-day settings identity, and all runs share
      it, the same driver_config_sha256 (null = none given) and the same frozen
      spec, wall_patches.spec_sha256 (null = none given). Otherwise the
      verdict is NOT_EVALUABLE: a changed setting or experiment starts a new
      series (as R1 does for a changed FAST-LIO config).
  G2. A run counts only if spatio.series counts it AND its wall_patches show
      both (all, at least two) of the frozen spec's preselected patches
      verified (spatio.wall_scoring.acceptance_blockers). When spatio.series
      said PASS, the earliest three consecutive counting attempts are found
      again; if none remain the verdict is FAIL (R5: three or more attempts and
      no counting triple).

Exit codes are spatio.series': 0 PASS, 3 FAIL, 4 INCOMPLETE, 5 NOT_EVALUABLE,
6 REPLAY_ONLY, 1 input rejected, 2 unreadable.
"""

from __future__ import annotations

import json
import os
import sys

from spatio.series.cli import VERDICT_EXIT
from spatio.series.report import order_walks, render_markdown
from spatio.series.validate import InputRejected, Unreadable, load_walk_result
from spatio.walkday.provenance import check_identity
from spatio.wall_scoring import acceptance_blockers

SETTINGS_CHANGE_REASON = ("a changed walk-day setting, driver config or frozen spec starts a new series "
                          "(walk-day rule G1)")
RULES_MD = """## Walk-day checks (spatio.walkday.seriesgate)

These rules come after spatio.series' R0-R5 above and can only make the verdict stricter.

- G1. Every run must carry the same walk-day settings identity (the effective walkday.conf values: hold, endpoint, gap, path and wall limits, playback rate, map voxel, ...), the same driver config sha256 and the same frozen spec (wall_patches.spec_sha256). If not, the verdict is NOT_EVALUABLE.
- G2. A run counts only when spatio.series counts it and every preselected wall patch in the frozen spec (at least two) was measured, not doubled, and within both gates. Runs scored on auto-suggested patches never count. When spatio.series said PASS, the triple is found again under G2. If no triple remains, the verdict is FAIL.
"""


def _read(path):
    try:
        with open(path, encoding="utf-8") as f:
            return json.load(f)
    except (OSError, ValueError) as exc:
        raise Unreadable(path, str(exc))


def gate(staged: dict, paths):
    walks = order_walks([load_walk_result(p) for p in paths])
    raw = {w.run_id: _read(w.source_path) for w in walks}
    runs = staged["runs"]
    if [r["run_id"] for r in runs] != [w.run_id for w in walks]:
        raise InputRejected(paths[0], "staged spatio.series report does not list these runs in recorded order")
    report = dict(staged)
    base = staged["verdict"]

    settings = {i: (raw[i].get("walkday_settings") if check_identity(raw[i].get("walkday_settings")) else None)
                for i in raw}
    drivers = {i: raw[i].get("driver_config_sha256") for i in raw}
    specs = {i: (raw[i]["wall_patches"].get("spec_sha256") if isinstance(raw[i].get("wall_patches"), dict) else None)
             for i in raw}
    specs = {i: None if s is None else str(s) for i, s in specs.items()}   # null = no frozen spec
    mixed = []
    unknown = [i for i, s in settings.items() if s is None]
    if unknown:
        mixed.append("walk-day settings identity missing or invalid for: %s." % ", ".join(unknown))
    elif len({s["sha256"] for s in settings.values()}) > 1:
        keys = sorted(k for k in next(iter(settings.values()))["values"]
                      if len({json.dumps(s["values"].get(k)) for s in settings.values()}) > 1)
        mixed.append("walk-day settings differ (%s): %s." % (
            ", ".join(keys), ", ".join("%s %s" % (i, s["sha256"][:12]) for i, s in settings.items())))
    if len(set(drivers.values())) > 1:
        mixed.append("driver_config_sha256 values differ: %s." % ", ".join(
            "%s %s" % (i, d) for i, d in drivers.items()))
    if len(set(specs.values())) > 1:
        mixed.append("wall_patches.spec_sha256 values differ (frozen experiment spec): %s." % ", ".join(
            "%s %s" % (i, d) for i, d in specs.items()))

    new_runs = []
    for run in runs:
        run = dict(run)
        why = acceptance_blockers(raw[run["run_id"]].get("wall_patches"))
        s = settings[run["run_id"]]
        run["walkday_settings_sha256"] = None if s is None else s["sha256"]
        run["driver_config_sha256"] = drivers[run["run_id"]]
        run["spec_sha256"] = specs[run["run_id"]]
        run["wall_patch_blockers"] = why
        if why:
            run["counts_toward_series"] = False
            run["count_blockers"] = list(run["count_blockers"]) + why
        new_runs.append(run)
    flags = [r["counts_toward_series"] for r in new_runs]

    verdict, counted, reasons = base, list(staged["counted_run_ids"]), list(staged["reasons"])
    if mixed:
        verdict, counted = "NOT_EVALUABLE", []
        keep = [r for r in reasons if base == "NOT_EVALUABLE" and not r.startswith("Per-run counting flags")]
        reasons = keep + [SETTINGS_CHANGE_REASON] + mixed + [
            "Per-run counting flags are still shown. They do not make this series a PASS."]
    elif base == "PASS":
        triple = next((i for i in range(len(flags) - 2) if all(flags[i:i + 3])), None)
        if triple is None:
            verdict, counted = "FAIL", []
            reasons = ["No three consecutive attempts count toward the series once walk-day rule G2 "
                       "(both preselected wall patches verified) is applied; spatio.series alone said PASS."]
            reasons += ["%s does not count: %s." % (r["run_id"], "; ".join(r["count_blockers"]))
                        for r in new_runs if not r["counts_toward_series"]]
        else:
            ids = [r["run_id"] for r in new_runs[triple:triple + 3]]
            if ids != counted:
                counted = ids
                reasons = ["Three consecutive attempts count under walk-day rule G2: %s." % ", ".join(ids)]
    report.update({
        "verdict": verdict,
        "counted_run_ids": counted,
        "retained_failed": [r["run_id"] for r in new_runs if not r["counts_toward_series"]],
        "reasons": reasons,
        "runs": new_runs,
        "spatio_series_verdict": base,
        "walkday_gate": {"rules": ["G1", "G2"], "mixed": mixed,
                         "settings_sha256": sorted({s["sha256"] for s in settings.values() if s}),
                         "driver_config_sha256": sorted({str(d) for d in drivers.values()}),
                         "spec_sha256": sorted({str(d) for d in specs.values()})},
    })
    return report


def render(report: dict) -> str:
    md = render_markdown(report)
    v, base = report["verdict"], report["spatio_series_verdict"]
    head = "Verdict: %s\n" % v
    note = ("(walk-day verdict; spatio.series alone said %s, see Walk-day checks)\n" % base) if v != base else ""
    md = md.replace(head, head + note, 1)
    lines = [md.rstrip("\n"), "", RULES_MD.rstrip(), ""]
    lines.append("spatio.series alone: %s. After G1 and G2: %s." % (base, v))
    for m in report["walkday_gate"]["mixed"]:
        lines.append("- G1: %s" % m)
    for r in report["runs"]:
        lines.append("- %s: settings %s, driver config %s, spec %s, wall patches: %s" % (
            r["run_id"], (r["walkday_settings_sha256"] or "MISSING")[:12], r["driver_config_sha256"],
            (r["spec_sha256"] or "none")[:12],
            "; ".join(r["wall_patch_blockers"]) or "both preselected patches verified"))
    return "\n".join(lines) + "\n"


def main(argv=None) -> int:
    argv = sys.argv[1:] if argv is None else argv
    if len(argv) < 3:
        print(__doc__, file=sys.stderr)
        return 2
    staged_dir, out_dir, paths = argv[0], argv[1], argv[2:]
    try:
        report = gate(_read(os.path.join(staged_dir, "series.json")), paths)
    except Unreadable as exc:
        print("unreadable: %s" % exc, file=sys.stderr)
        return 2
    except InputRejected as exc:
        print("rejected: %s" % exc, file=sys.stderr)
        return 1
    os.makedirs(out_dir, exist_ok=True)
    with open(os.path.join(out_dir, "series.md"), "w", encoding="utf-8") as f:
        f.write(render(report))
    with open(os.path.join(out_dir, "series.json"), "w", encoding="utf-8") as f:
        json.dump(report, f, indent=2, ensure_ascii=False)
        f.write("\n")
    print("%s %s" % (report["verdict"], os.path.join(out_dir, "series.md")))
    return VERDICT_EXIT[report["verdict"]]


if __name__ == "__main__":
    sys.exit(main())
