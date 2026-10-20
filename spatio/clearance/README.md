# Synthetic clearance evidence

This is a small offline prototype using Python, NumPy and the standard library.
Tests also require pytest. It does not read bags, merged maps or live sensors.
No physical accuracy or regulatory compliance has been established.

Run from the repository root, choosing a new output directory:

```sh
python3 -m spatio.clearance --synthetic \
  --voxel-size 0.1 --uncertainty-m 0.02 --min-free-rays 2 \
  --output /private/tmp/clearance-example
python3 -m pytest -q spatio/clearance
```

The CLI writes `report.json` and `report.md`, and prints the Markdown.
Synthetic mode also writes `zones.json` and `scans.json` so the inputs are
inspectable and reusable. It refuses an existing output directory.

For independent inputs:

```sh
python3 -m spatio.clearance --zones /path/to/zones.json \
  --scans /path/to/scans.json --voxel-size 0.1 \
  --grid-origin 0 0 0 --uncertainty-m 0.02 \
  --output /private/tmp/clearance-new-input
```

The numbers above are example parameters, not recommended operating settings.
No sensor-specific names, timing rates, calibration or recording paths are used.

## Provenance binding (B2)

Optional inputs bind a result to where its rays came from:

```sh
python3 -m spatio.clearance --zones zones.json --scans scans.json \
  --voxel-size 0.1 --uncertainty-m 0.02 \
  --recording /path/to/bag-or-file --extrinsics extrinsics.yaml \
  --trajectory trajectory.txt --time-basis "sensor header stamp, seconds" \
  --output /private/tmp/clearance-bound
```

`report.json` gains `provenance` (`schema: "spatio.clearance_provenance.v1"`):
`recording_sha256`, `extrinsics_sha256`, `trajectory_sha256` (sha256 of the
file bytes; a recording directory hashes its sorted relative names and file
hashes), `time_basis` (supplied text), `scans_sha256` (canonical JSON of the
scan document, i.e. the ray evidence) and `limitations`. Only hashes are
written, never paths. Each zone record gains `zone_geometry_sha256` (frame,
shape and parsed min/max/axis/direction; not ID, rule or provision) and
`provenance_limitations`, next to its classification.

A missing recording, extrinsics, trajectory or time basis gives a
`limitations` key (`no_recording_identity`, `no_extrinsics_identity`,
`no_trajectory_identity`, `no_time_basis`). The same keys are on every zone
record and in both Markdown reports, so a PASS is never silent about them.
They never change a status. Hashes detect changed inputs; they are not
signatures, and the classifier still decides from the rays alone.

## One-walk exception report and recheck list (F7a / F8a)

Present an existing clearance result using its matching zone definitions:

```sh
python3 -m spatio.clearance report \
  --results /path/to/clearance-results/report.json \
  --zones /path/to/zones.json --walk-id fresh-walk-id \
  --output /private/tmp/clearance-walk-summary
```

This subcommand writes `report.md` and `report.json` to a new directory and
prints the Markdown. The output directory must not already exist, including
the input results directory. Supplied `--trajectory`, `--extrinsics`,
`--recording` and `--scans` files are only hashed; no maps, sensors or network
services are opened at this stage.

Reuse is refused (exit 2, nothing written) when any binding fails:

- A zone's geometry hash differs from the supplied zone definition.
- The results are bound to a trajectory or extrinsics that is not resupplied,
  or the resupplied file hashes differently.
- A trajectory or extrinsics is supplied for results bound to none.
- A supplied recording or scans file hashes differently from the results.
- The results have no `provenance` block (pre-B2 results), or its limitations
  disagree with its null identities.

An unsupplied recording or scans file is listed in `reuse_limitations`
(`recording_not_reverified`, `scans_not_reverified`). The output copies
`provenance` and lists all limitations in the Markdown.

JSON has `schema: "spatio.clearance_report.v1"`, `walk_id`, `frame_id`,
`settings`, `scan_count`, `scan_time_range`, `provenance`, `reuse_limitations`,
`counts` (all four status keys),
`zone_count`, `zones`, `failures`, and `rechecks`. `physical_accuracy_evaluated`
remains false. Each zone item contains the complete unchanged upstream record
under `evidence` plus `map_position_m`. Failures contain only FAIL items;
rechecks contain every NOT_MEASURED and INDETERMINATE item with a consecutive
`number` starting at 1. Lists sort by case-sensitive Unicode zone ID, not
severity, spatial proximity or travel order. There is no planner.

Positions are **required-volume centres in the supplied frame**, derived from
box or plane-offset geometry. They are references for finding the zone, not
measured obstacle locations or recommended viewpoints. The caller must supply
the original zone document. The join checks IDs, frame, shape, rule,
provision and depth, and each zone's geometry hash, so a translated zone with
the same attributes is refused.

The compact one-page-style Markdown includes settings, status counts, all
FAIL measurements against required minimum depth, positions, evidence times,
supplied provisions, and numbered recheck reasons. A missing hit clearance
stays “not measured”; the occupied-voxel lower bound is labelled separately.
Neither a voxel bound nor a measured hit proves full-volume coverage.
Long supplied text or many exceptions can exceed one printed page: Markdown
has no fixed page geometry, and no exception is truncated to force a page fit.
The JSON retains all evidence and full numeric precision.

Input statuses are never upgraded or recomputed. Missing fields, mismatched
zone sets, unknown statuses and inconsistent evidence summaries fail explicitly.
A PASS requires positive required-cell count, all cells free, zero insufficient
or conflicting cells, and evidence times within a nonempty scan range.
This validates upstream summary consistency, not the original rays or physical
truth. The classifier remains responsible for the actual decision.
Empty result and definition zone lists produce zero counts and an explicit
“no clearance conclusions” note; the classifier's nonempty-zone rule is unchanged.

Output uses sorted JSON keys, UTF-8, LF endings and no generated clock or path
metadata. Identical inputs and walk ID produce identical bytes. Observed time
ranges are copied in the input's time basis; they are not labelled UTC.
Processing latency and recording-end-to-report latency are not measured here.
Synthetic tests cover fidelity and deterministic formatting, not physical accuracy.

## Zone contract, version 1

The document has exactly `schema_version` (integer 1), a nonempty `frame_id`,
and a nonempty `zones` array. Each zone has exactly:

- `id`: unique, nonempty text.
- `shape`: `box` or `plane-offset`.
- `rule_id`: nonempty rule identifier.
- `provision`: nonempty supplied provision/applicability text.
- `required_volume`: geometry as described below.

All coordinates and distances are in metres in the named shared frame.
All numbers must be finite; booleans are not numbers. Unknown fields, missing
fields, duplicate JSON keys, duplicate zone IDs and degenerate volumes fail.
The parser validates structure, not the truth or approval of a provision.
The checked-in fixture is explicitly **EXAMPLE, NOT APPROVED** and contains
invented dimensions with no regulatory authority.

A `box` required volume has exactly:

```json
{
  "min": [0, 0, 0],
  "max": [1, 0.4, 0.4],
  "clearance_axis": 0,
  "clearance_direction": 1
}
```

Each `max` coordinate must exceed its corresponding `min`. Axis 0/1/2
means x/y/z. Direction is integer +1 or -1. Clearance is measured along
that direction from the minimum face (+1) or maximum face (-1).
Required depth is the box extent on that axis.

A `plane-offset` required volume has exactly:

```json
{
  "plane_point": [2, 0, 0],
  "normal": [-1, 0, 0],
  "u_min": 0.1, "u_max": 0.5,
  "v_min": 0.1, "v_max": 0.5,
  "depth_m": 1
}
```

Only signed coordinate-axis normals are supported; oblique planes fail
explicitly. The two remaining coordinate axes, in ascending axis order, are
u and v. Their bounds are offsets from `plane_point`, not absolute positions.
Depth must be positive. The example spans x=[1,2], y=[0.1,0.5], z=[0.1,0.5],
with clearance measured toward -x from x=2.

## Scan contract, version 1

```json
{
  "schema_version": 1,
  "frame_id": "example_map",
  "scans": [
    {
      "timestamp": 123.25,
      "origin": [-0.15, 0.05, 0.05],
      "hits": [[1.25, 0.05, 0.05]]
    }
  ]
}
```

Exactly these fields are allowed. Times are finite seconds and strictly
increasing. An empty scan list or empty hit list provides no spatial evidence.
Each hit is a finite three-vector different from its origin. The frame must
match the zone document.

**Input responsibility:** hits must be first returns, with origins and points
already transformed into the same frame at the correct acquisition time.
The parser cannot verify occlusion or pose accuracy from this data alone.
No-return/missed beams provide no evidence. Use separately timestamped groups
when sensor motion changes the origin; deskew and trajectory interpolation
are outside this package.

## Evidence and decisions

The grid uses half-open cells `[origin+i*size, origin+(i+1)*size)`.
Required cells are every cell with positive-volume intersection with the zone,
including partial boundary cells. Decimal grid coordinates within eight
floating-point epsilons of an integer are snapped to that boundary.
This avoids phantom cells from arithmetic such as 0.3/0.1.

Each ray is split at voxel boundaries. Positive-length interior traversal is
free evidence; the endpoint cell is occupied. The sensor-origin cell is not
marked free by that ray. Rays lying exactly on a voxel face do not establish
free evidence on either side. There is no widening, filling between rays,
interpolation through occluders or inference from missing map points.

Each cell retains independent sets of free and occupied timestamps, plus
occupied hit positions and a free-ray count. A ray contributes once to each
traversed cell; several distinct rays in one scan count separately. An
identical repeated ray (same origin and hit, in one scan or several) counts
once toward the free-ray minimum, since it samples the same line again; its
hit and time are still recorded as occupancy. Either observation
order produces the same conflict.
All conflicts are reported by index, both globally and within each zone;
an occupied cell never becomes free-only. Unknown is implicit.

Decision precedence:

1. An occupied required cell gives FAIL, regardless of unobserved cells.
   The explicit near-limit exception is INDETERMINATE when the absolute
   clearance-minus-required-depth margin is within the supplied uncertainty.
2. Otherwise any required cell with fewer than `--min-free-rays` free
   traversals gives NOT_MEASURED. The default is **2**, a provisional positive
   integer that can be set explicitly; it is not fitted to a recording.
3. Otherwise every required cell is free-only with sufficient ray evidence.
   A hit inside the sideways-expanded footprint and at a positive clearance
   no greater than required depth plus uncertainty gives INDETERMINATE;
   the remaining cases give PASS.

The uncertainty band is inclusive; zero disables it. The CLI requires an
explicit value and always labels it provisional. It is not a calibrated
sensor, localization, voxel or combined error bound. Both lateral bounds are
widened by that amount for hit checks. For example, a hit 5 mm outside a
side face at 0.5 m depth blocks PASS with 0.02 m uncertainty and 1 m required
depth. The nominal required cells and occupied-cell precedence do not change.

`min_clearance_m` is the minimum **observed hit** distance strictly in front
of the reference face and within the uncertainty-expanded lateral footprint,
even beyond the required depth. With zero uncertainty it uses the nominal
half-open footprint. Reference-face hits are not clearance measurements.
Null means no qualifying hit; a number does not establish full-volume
coverage. `decision_margin_m` uses hits inside occupied required cells.
For a partially intersecting occupied cell whose hits lie outside the
volume, it conservatively uses the cell's near face. With no occupied required
cells, the margin uses `min_clearance_m` minus required depth; a sideways hit
can therefore yield a large negative margin and INDETERMINATE. The separately reported
`occupied_voxel_clearance_lower_bound_m` always uses voxel near faces.
These fields deliberately distinguish sampled geometry from cell bounds.
The evidence time range covers required-cell observations and qualifying
clearance hits. Scan time range also includes scans with no returns.
`free_only_cells` and `free_coverage` still count cells with at least one free
ray and no occupancy, and `unknown_cells` counts cells with no evidence.
`insufficient_evidence_cells` counts unoccupied required cells below the
configured minimum, including unknown cells. A 100% free coverage alone no
longer suffices for PASS. JSON and Markdown report the provisional minimum.

## Limits

PASS means observed free **at the chosen voxel resolution**, not proof that
every physical point is empty. Thin objects, sparse beams, bad poses, moving
objects and incorrect first-return inputs can defeat that interpretation.
Conflicts are retained indefinitely within this one input session; there is
no aging or moving-object filter. The minimum-ray count is only a provisional
sampling gate: identical repeated rays count once, but near-duplicate rays
still count, independence and spatial diversity are not established, and even
two rays can miss most of a voxel. A single ray clipping a voxel corner counts
as that voxel's free evidence when `--min-free-rays 1` is chosen. It is not an operational clearance guarantee.

Boundary cells can cause conservative false alarms, including a reference
surface sharing a cell with the required volume. A panel at x=2 facing -x
in the synthetic fixture lands outside its half-open required volume.
Rotated zones, arbitrary plane fitting, uncertainty propagation, beam models,
scan ingestion and physical validation remain future work.

The implementation is for small synthetic scenes: at most one million
observed or required cells, with bounded per-axis ray traversal. It stores
hit history in memory and is not a streaming large-walk implementation.
Trevor must approve the zone definitions, real rule provisions/applicability,
and calibrated acceptance criteria before any operational use.
