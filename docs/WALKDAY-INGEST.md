# Walk-day evidence ingest

`spatio.memory.ingest.ingest_walkday(store, session_dir, recording_dir,
code_revision, clearance=None)` imports one scored walk into the V1
`EvidenceStore`. It implements the mapping in `lc-tasks/plans/PLAN-V2.md`,
following [MEMORY-V2-DESIGN.md](MEMORY-V2-DESIGN.md) §2.3, §2.6 and the
approved §5 decisions. It writes no interpretations.

```sh
python3 -m spatio.memory.ingest /data/walk-001 \
  --recording-dir /data/recording-001 \
  --code-revision <40-lowercase-hex-commit-id> \
  --database /data/evidence.sqlite \
  --clearance /data/clearance/report.json /data/zones.json < /dev/null
```

Omit `--clearance` when no clearance result is available. Its first file is
the classifier's `schema_version: 1` report, not the presentation report.
Success prints an `IngestResult` as JSON; refusal exits 2. The result includes
native session/revision/observation IDs, newly inserted observation IDs,
skipped items and the new transaction ID (`null` on a no-op).

## Input binding

The recording must be a nonempty directory without `SYNTHETIC.json`.
Its complete tree hash uses the clearance `sha256_path` definition.
`manifest.json` must bind the recording's metadata hash, bag names, sizes
and any non-null bag hashes. The manifest builder's recording layout helpers
are reused. Null bag hashes remain permitted, matching walk-day output;
in that case the original manifest binds bag names and sizes, not bag bytes.
The importer still hashes all current recording bytes for the session identity.

`walk_result.json` must parse as `spatio.walk_result/1`, have status `PASS`
or `FAIL`, and match the manifest's run ID, FAST-LIO revision and configuration
hash. Missing/unscored and `NOT_EVALUABLE` runs are refused. The actual map and
trajectory hashes must match the manifest. Session times come from the first
and last parsed TUM stamps, with at least two strictly increasing stamps.
The wallpatch report must bind the actual map hash; its `map_path` is ignored.

Optional clearance requires matching recording and trajectory hashes, matching
zone geometry, no provenance limitations, and scan times inside the TUM span.
The existing clearance validator checks counts, coverage and PASS evidence.
The two-file API validates the recorded scans/extrinsics hashes structurally;
it cannot independently rehash those source files. `walk_id` and `frame_id`
are not database IDs. Interpreting the clearance boxes in R1 assumes the
clearance frame is the bound trajectory's FAST-LIO map frame; the hash binding
alone does not prove that frame convention.

All JSON must have unique keys and finite numbers. Symlinks, special files,
unreadable recording subtrees and database/input aliases are refused. The CLI
checks database placement before opening it. Python callers must open their
store outside the input trees. Inputs must remain quiescent during ingest;
the importer does not snapshot concurrent filesystem writers.

## Evidence mapping

The session ID and map ID are the run ID. T1 and R1 use revision 1; T1 has no
parent because it was rebuilt from the recording. Both retain FAST-LIO and
configuration provenance. R1 names T1, and every observation names R1.

- Each `MEASURED` patch becomes a lidar detection. Its AABB encloses all eight
  corners of the oriented patch box, using the writer's along/across/vertical
  axes. Its `sigma_m` is `residuals.p95_abs_m`.
- Each `PASS` clearance zone becomes a lidar absence with the parsed zone AABB,
  `free_coverage` and `uncertainty_m`. FAIL, INDETERMINATE and NOT_MEASURED
  zones are skipped, as are NOT_EVALUABLE patches, and listed in the result.
- Series directories and series files supplied as clearance are refused:
  aggregate verdicts carry no measurement geometry.

Following the plan's proposed conventions, acquisition time is the last TUM
stamp and measurement confidence is 1.0. This is not a calibrated probability.
The extraction `code_revision` argument is recorded as `method_version`; it is
operator-supplied because the writers do not record it. Wallpatch gates and
clearance settings receive canonical parameter hashes.

## Persistence and history

All new rows use V1's public write methods inside one transaction. The ingest
plan and conflict checks run before opening it. A no-op writes nothing,
including no transaction row. Late clearance appends its absences in one new
transaction and reuses the session, T1 and R1.

The `derived_from` key is
`recording:<sha>|source:<wallpatch|clearance>|item:<name>|record:<sha>`.
The record hash covers one item, excluding `figure`; report creation time and
paths are not part of it. Regenerating presentation output does not duplicate
evidence. Changed item content, stored values, session identity or revision
provenance is a conflict. A retracted session is refused: its evidence is
retracted and needs an operator. Re-processing the same recording under a new
run ID is also refused, as a conflicting `derived_from`, because the key names
the recording, not the session. Any failure rolls back the complete ingest.

V1 queries see these observations with their normal retraction, supersession
and `known_at` semantics. Object queries require later interpretations; ingest
creates none. This module provides no alignments, migration, re-extraction or
live-robot validation.

## Validation

`spatio/memory/tests/test_v2_ingest.py` exercises the public V1 ingest path with
real manifest, wallpatch and clearance writers, explicit refusals, item-level
idempotency, rollback, enclosing geometry, and V1 history/comparability checks.
Required local gates are
`lc-tasks/gate-full.sh` and `python3 -m pytest -q spatio .github/scripts`.
These are local software checks, not physical accuracy or deployment proof.
