PLAN: Widen the sideways uncertainty check, require a configurable minimum
free-ray count per required cell (default 2), add focused regressions, run the
whole clearance suite, and commit only if Git metadata is writable.

# F1 clearance follow-up findings

Implemented the two P2 follow-ups only in `spatio/clearance` on branch
`spatio-f1-evidence`.

The provisional uncertainty now expands both sideways bounds for hit checks.
With sufficient free evidence, a hit in that expanded footprint at positive
clearance up to required depth plus uncertainty gives INDETERMINATE. This
includes a hit 5 mm outside the footprint at 0.50 m depth for a 1 m zone and
0.02 m uncertainty. Regressions cover both sideways axes, both sides, and
all clearance axes/directions. Zero uncertainty disables the band.
`min_clearance_m` now includes hits in the expanded footprint; README documents
that change. Required cells and occupied-cell decision precedence are unchanged.

PASS now requires `--min-free-rays` traversals per required cell (default 2,
positive integer). Multiple rays in one scan count individually; timestamps
remain separate evidence history. Under-supported unoccupied cells produce
NOT_MEASURED. Existing free/unknown counts retain their meaning; a new
`insufficient_evidence_cells` count explains abstentions even at 100% sampled
free coverage. JSON and Markdown expose the provisional minimum.

Run from the repository root, choosing a new output directory:

```sh
python3 -m spatio.clearance --synthetic --voxel-size 0.1 \
  --uncertainty-m 0.02 --min-free-rays 2 \
  --output /private/tmp/clearance-followup-example
python3 -m pytest -q spatio/clearance
```

For new inputs, replace `--synthetic` with `--zones /path/to/zones.json
--scans /path/to/scans.json`; supply voxel size, grid origin and uncertainty.
No recording paths, topics, sensor settings or old-data tuning were added.

Measured local validation: baseline **82 passed**; final full package suite
**136 passed, 0 failed, 0 skipped**. All existing assertions remain; the two
existing coverage/uncertainty fixtures now supply two distinct rays to meet
the stronger default. The 54 added cases include count thresholds, a single
under-supported cell, invalid arguments, occupied precedence and CLI runs
with translated/scaled geometry and different timestamps. An intermediate
run had 24 fixture-type failures; correcting NumPy scalars to JSON-style
numbers resolved them without changing parser rules or assertions.
No Swift files changed; Swift tests and app build were not run.

The synthetic CLI smoke run (0.10 m voxels, 0.02 m uncertainty, minimum 2)
returned FAIL / NOT_MEASURED / NOT_MEASURED for panel / occluded / gap.
Minimum hit clearances were 0.80 m / none / 1.00 m; below-minimum unoccupied
cell counts were 159 / 80 / 100. Three scans, zero conflicts.
`git diff --check` passed.

Limits: this is synthetic voxel evidence, not physical clearance validation.
Repeated or duplicate rays count; independence, spatial diversity and evidence
age are not enforced. Two rays can still miss thin obstacles within a cell.
The uncertainty and count defaults are provisional, not calibrated.
Trevor's approval is still needed for operational acceptance settings and real
rule applicability; no new approval is needed for these local changes.
No Spark, real recordings, containers, apps or other workstreams were touched.


Commit status: committed by the orchestrator (Codex cannot write Git metadata).
