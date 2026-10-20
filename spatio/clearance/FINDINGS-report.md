# F7a / F8a findings

Built the one-walk output stage in `spatio/clearance/report.py` and exposed it
as the `report` subcommand of `python3 -m spatio.clearance`. It reads existing
clearance results and the matching zone definitions, plus an explicit walk ID.
It writes compact Markdown and JSON with schema `spatio.clearance_report.v1`,
settings, all four status counts, all FAIL items, and a numbered list of every
NOT_MEASURED / INDETERMINATE zone. Full upstream zone records are retained.

Run from the repository root with a new output directory:

```sh
python3 -m spatio.clearance report \
  --results /path/to/clearance-results/report.json \
  --zones /path/to/zones.json --walk-id fresh-walk-id \
  --output /private/tmp/clearance-walk-summary
PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -q -p no:cacheprovider spatio/clearance
```

Design decisions: positions are the centres of required volumes in the input
map frame, explicitly not obstacle or viewing poses. Both output lists sort
by zone ID; rechecks are numbered from 1. No planner or classifier changes.
Measured hit clearance, required minimum and occupied-voxel lower bound are
distinct. Null measured clearance stays unmeasured. Unsupported PASS summaries
are rejected instead of silently changed. Empty lists give no conclusions.
No runtime clocks or output paths enter the report; output is UTF-8 with LF.

Measured local validation: baseline **136 passed, 0 failed, 0 skipped**.
Final full clearance suite: **171 passed, 0 failed, 0 skipped**, including
**35 new report cases**. Existing tests were not changed. Tests cover all four
statuses, exact upstream-record fidelity, empty lists, recheck ordering,
translated/scaled geometry, malformed records, evidence-free PASS rejection,
literal Markdown text, CLI failure before writing, and overwrite refusal.
Two separate CLI processes with hash seeds 1 and 789 produced byte-identical
JSON and Markdown. Input list/key reordering also produced identical outputs.
`git diff --check` passed; status confirmed all changes are in `spatio/clearance`
on branch `spatio-f1-report`.

Synthetic results: four zones yielded PASS=1, FAIL=1, NOT_MEASURED=1,
INDETERMINATE=1. The FAIL reported 0.8 m measured hit clearance against
1.0 m required minimum at zone centre [0.5, 2.05, 0.05] m, with evidence time
[101, 101]. Rechecks were 1: a-near, 2: b-unseen. A second input used twice
the scale, translation [10, -4, 6] m and times starting at 9000; it retained
the four statuses and reported approximately 1.6 m against 2.0 m at
[11, 0.1, 6.1] m. A partial-cell obstruction test retained FAIL with no
measured hit clearance and a separately labelled 0 m voxel lower bound.
These are synthetic software results, not real-walk or physical measurements.

Limits: summary validation cannot independently verify rays or physical
clearance. Matching zone geometry is the caller's responsibility: upstream
results lack geometry/hash provenance, so translation cannot be detected from
IDs/rules/depth alone. Markdown is a compact one-page-style report, not fixed
pagination; large exception lists grow instead of omitting items. Latency,
regulatory compliance and physical accuracy were not evaluated.

No Swift files changed; Swift tests and app build were not run because this
is an isolated Python output stage. No recordings, Spark, containers, apps,
other workstreams, network operations or Git writes were used. Trevor still
needs to approve real zone/rule applicability and operational settings before
physical use; no new approval is needed for this local output-stage work.

Changes are intentionally uncommitted for the orchestrator.
