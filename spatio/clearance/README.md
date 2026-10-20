# Synthetic clearance evidence

This is a small offline prototype using Python, NumPy and the standard library.
Tests also require pytest. It does not read bags, merged maps or live sensors.
No physical accuracy or regulatory compliance has been established.

Run from the repository root, choosing a new output directory:

```sh
python3 -m spatio.clearance --synthetic \
  --voxel-size 0.1 --uncertainty-m 0.02 \
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
occupied hit positions. Either observation order produces the same conflict.
All conflicts are reported by index, both globally and within each zone;
an occupied cell never becomes free-only. Unknown is implicit.

Decision precedence:

1. An occupied required cell gives FAIL, regardless of unobserved cells.
   The explicit near-limit exception is INDETERMINATE when the absolute
   clearance-minus-required-depth margin is within the supplied uncertainty.
2. Otherwise any unknown required cell gives NOT_MEASURED.
3. Otherwise every required cell is free-only. A measured obstruction within
   the uncertainty band gives INDETERMINATE; the remaining cases give PASS.

The uncertainty band is inclusive; zero disables it. The CLI requires an
explicit value and always labels it provisional. It is not a calibrated
sensor, localization, voxel or combined error bound.

`min_clearance_m` is the minimum **observed hit** distance strictly in front
of the reference face and within the zone's lateral footprint, even beyond
the required depth. Reference-face hits are not clearance measurements.
Null means no qualifying hit; a number does not establish full-volume
coverage. `decision_margin_m` uses hits inside occupied required cells.
For a partially intersecting occupied cell whose hits lie outside the
volume, it conservatively uses the cell's near face. The separately reported
`occupied_voxel_clearance_lower_bound_m` always uses voxel near faces.
These fields deliberately distinguish sampled geometry from cell bounds.
The evidence time range covers required-cell observations and qualifying
clearance hits. Scan time range also includes scans with no returns.

## Limits

PASS means observed free **at the chosen voxel resolution**, not proof that
every physical point is empty. Thin objects, sparse beams, bad poses, moving
objects and incorrect first-return inputs can defeat that interpretation.
Conflicts are retained indefinitely within this one input session; there is
no aging or moving-object filter.

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
