# Spatio memory v2: time, revisions and alignment (design only)

Status: design only, written 2026-10-20 for job B3 of the solidity plan. No code, schema or test changes come with it. Schema v2 needs Trevor's approval before anyone builds it (DESIGN §3, northstar M3).

Tags, as in the northstar: **[measured]** is read in the code cited; **[proposal]** is my recommendation, not decided; **[provisional]** marks a number Trevor sets before it is used.

## 0. What this is grounded in

- **Unmerged branch evidence: the v1 store is not on main.** `spatio/memory/` (`store.py`, `api_stub.py`, `DESIGN.md`, `tests/`) is absent from the checked `origin/main` revision 3434f0e. Every `store.py:N`, `api_stub.py:N` and `DESIGN §N` below refers to Mac-only commit `90ae03e` on the unmerged `spatio-memory` branch; that commit is not an ancestor of 3434f0e. These are design inputs from unmerged code, not evidence of a shipped store or merged tests. The northstar's "implementation is Grok's task" is stale: the store and its invariant tests are written, just not merged.
- **Merged room-memory evidence.** Citations (`spark/room_memory`, `spark/room_memory_cpp`, `docs/ROOM-MEMORY-PROTOCOL.md`) refer to 4de8ec7, an ancestor of the checked `origin/main` revision 3434f0e.
- Other inputs: `SPATIO-northstar.md` (layers 2–3, M3, M4) and `SPATIO-drift-investigation.md` (what a "trajectory error" looks like on real walks) in `lc-tasks/plans`.

v2 keeps every v1 rule: measurements and guesses in separate tables and fields; `CURRENT` only from a fresh measured sighting with a valid pose; not looking is not absence; queries take `now` and never read a clock. It adds what v1 cannot express:

1. acquisition time vs knowledge time;
2. retractions that do not rewrite the past;
3. map and trajectory revisions;
4. validated cross-session transforms;
5. immutable evidence, reconciled with retention;
6. unaligned sessions kept separate.

## 1. What v1 does today, and where it stops

| Topic | v1 as built [measured] | Consequence |
|---|---|---|
| Time axes | Queries have one time argument, `now`, in sensor time. `_newest_sighting_id` filters only `acquired_at <= now` (`store.py:1697-1712`); absences likewise (`:1777-1810`). Every row has a wall-time `created_at`, but no query reads it | A row inserted late changes the answer for every past `now`. "What did we know at T" cannot be asked |
| Retraction | `interpretations.retracted_at` is set once (`:1446-1465`, trigger `:394-414`). The `sightings` view filters `retracted_at IS NULL` with no time bound (`:472-481`) | A retraction is retroactive: yesterday's report, recomputed today, silently changes |
| Retraction scope | Only interpretations can be retracted. Observations are immutable (`:368-372`) and can only be deleted by the purge (`:374-379`) | A sensor fault found later has no honest record: the row can neither be withdrawn nor deleted |
| Maps | `sessions.map_id` is `UNIQUE` (`:166`); `add_session` refuses a second session per map (`:1152-1153`). A map is `map_id` + `map_pcd_sha256` (`:154-162`) with no revision | One session per map. Any re-run of the map is a new map |
| Trajectory | One `trajectory_sha256` per session (`:169`). Pose and centroid are stored in the map frame on the observation (`:193-274`) and are immutable | A corrected trajectory cannot be applied. The old positions stay baked in |
| Cross-session | Absences count only with `map_id = s.map_id` (`:1789`) | Correctly conservative; it is also why nothing compares walks (DESIGN §3) |
| Look-again lifecycle | A request row is updated in place: `open` → `fulfilled` or `cancelled`, with `closed_at` (trigger `:443-466`; `fulfil_look_again` and `cancel_look_again`, `:1578-1628`). The purge then nulls `fulfilled_by_observation_id` (`ON DELETE SET NULL`, `:325`, and the third transition at `:461-465`). `object_status` lists `state = 'open'` as it is now (`:1638-1646`) | A closure changes every past answer's `open_look_again`. The purge erases which observation fulfilled a request |
| Deletion | `purge_expired` deletes camera rows and cascades their interpretations (`:1825` on). It leaves only a counter, `objects.purged_sightings` (`:189`) | Evidence a past report used can vanish. Only the count survives |
| Schema identity | Thresholds are written into the view SQL as literals (`:142-149`, `:472-481`). `thumbnail_gc` is created lazily by the first purge without bumping `user_version` (`:364-366`, `:1814-1823`) | A reproducible answer must name the constants, not only the schema version. Two v1 files at `user_version` 1 can differ by one table, so a migration must accept both |
| Clocks | Sensor time is checked to be UNIX UTC (`api_stub.py:35-36`, `ClockError`). DESIGN §2 lists exactly two cross-clock comparisons (retention; look-again fulfilment) | Kept. v2 adds a third axis (§2.1) and says which comparisons use it |

Room memory already solves the same problems for a stationary rig, in a different form [measured]:

| Room memory | What it does | What v2 takes from it |
|---|---|---|
| `map_epoch` | A revision counter, bumped on every reference change (`memory.py:1372`, `:1947`). The reference file is overwritten in place, atomically (`save_reference`, `memory.py:342-347`). Old revisions are kept only when a relearn archives the frame (`archive_due_ = "frame-<epoch>"`, `memory.cpp:1713`, `:1874`) | A revision identifier. v2 keeps every revision; it does not overwrite |
| `frame_epoch` and relearn | A relearn starts a new room frame (`memory.py:1786`, `:1513-1518`). `Registry::apply_frame_change` retires every anchor of another frame and keeps its geometry; it never transforms it (`objects.cpp:736-760`) | Positions are never moved into a new frame by assumption. Old-frame geometry stays in its frame, marked as such |
| Re-alignment (D40) | A pose maps a sensor's current frame into its reference frame, and "re-alignment changes it and nothing else" (protocol §3.9). An estimate counts only with inlier fraction ≥ 0.9 × baseline, RMS ≤ 0.02 m and λ_min/Σw ≥ 0.02 (`memory.hpp:304-327`) | A transform is its own record, gated by recorded fit metrics. Applying it changes no evidence |
| Publisher generation (D34) | `g = max(stored + 1, now_ms)`, written durably before use (protocol §5.1) | A knowledge sequence that only rises, floored by the wall clock (§2.1) |
| Honest-state rule 2 | "Absence of returns ≠ absence of object" (protocol §6) | Kept unchanged (v1 already follows it) |

Wall scoring has the input-binding pattern v2 reuses for evidence: `_check_map` refuses a measurement whose `map_sha256` is not the run map's bytes (`spatio/wall_scoring.py:297-314`).

Nothing in this design changes room memory or wall scoring.

## 2. Design

### 2.1 Acquisition time and knowledge time

There are three axes. Each comparison uses one of them, and the doc says which.

| Axis | Columns | Source | Compared with |
|---|---|---|---|
| Acquisition (sensor) time | `acquired_at`, session `t_start`/`t_end` | the sensor clock, UNIX UTC, as v1 | the query's `now` |
| Knowledge sequence | `recorded_txn` on every row; `retracted_txn` on every retraction | the store: one integer per write transaction, strictly increasing | the query's `known_at` |
| Wall time | `transactions.recorded_at` | injected `wallclock`, as v1 | only retention (as v1), and resolving a wall-time `known_at` to a transaction |

**Transactions** [proposal]. Every public write opens one row in a new `transactions` table: `txn_id INTEGER PRIMARY KEY AUTOINCREMENT`, `recorded_at REAL`, `writer TEXT`, `operation TEXT`. Every row the write inserts carries that `txn_id` as `recorded_txn`. `recorded_at = max(previous recorded_at, wallclock())`, the same floor as the publisher generation (protocol §5.1), so the wall column never goes backwards even if the clock does. Ordering always uses `txn_id`, never wall time.

**Queries.** Every read takes `known_at`, a `txn_id`. It is optional only in the sense that `None` means "the newest committed transaction"; the answer always reports the `txn_id` it used, so it can be run again. `known_at_wall(T)` is a pure helper that returns the newest `txn_id` with `recorded_at ≤ T`.

**Visibility rule.** For a query at `(now, known_at)`, a row is visible when `recorded_txn ≤ known_at` and it is not invalid at `known_at`: neither it nor any row it depends on has a retraction with `retracted_txn ≤ known_at` (§2.2). Then v1's rules run unchanged on the visible rows that have a position in a current frame (§2.3), or a `body` position under the active trajectory revision, with `acquired_at ≤ now` as before. A visible row in a superseded frame stays readable as history and counts as neither a sighting nor an absence. A late observation is visible only to queries whose `known_at` is at or after its ingest; it never changes an answer given before it was recorded.

**Look-again fulfilment** keeps v1's rule: the fulfilling observation's `acquired_at` must be ≥ the request's creation time. v2 states the request's creation time as its transaction's `recorded_at`, so this stays v1's single sensor-vs-wall comparison, with the same caveat (unverified without a disciplined sensor clock, DESIGN §2). An observation acquired before the request but ingested after it does not fulfil it.

**Look-again lifecycle** [proposal]. v1 closes a request by updating its row (§1). v2 makes the lifecycle append-only, like retractions:

- A request row carries `recorded_txn` and is never updated. v1's `state`, `closed_at` and `fulfilled_by_observation_id` columns move out of it.
- `look_again_closures(request_id UNIQUE, outcome, fulfilled_by_observation_id, recorded_txn)`, with `outcome` either `fulfilled` or `cancelled`. A request closes at most once, and a closure is never retracted. To ask again, file a new request.
- `fulfilled_by_observation_id` is a plain id with no foreign key. It is set exactly when `outcome = 'fulfilled'`. The purge does not touch it. If that observation is purged, the id resolves through `purged_observations` (§2.6). v1's third transition, which nulls the id at purge time, does not exist in v2.
- Fulfilment needs the observation to be visible at the fulfilling transaction (§2.1), plus v1's acquisition rule above.
- A later retraction of the fulfilling observation, or of a session or revision it depends on (§2.2), does not reopen the request. A closure records an action that was taken, not evidence. It stays closed, and an operator who wants another look files a new request.
- The open requests at `known_at` are those with `recorded_txn ≤ known_at` and no closure with `recorded_txn ≤ known_at`. As in v1, `now` does not filter them.

### 2.2 Retractions

[proposal] One append-only table replaces v1's in-row `retracted_at`:

`retractions(retraction_id, target_kind, target_id, recorded_txn, reason, retracted_by)`, with `target_kind` one of `interpretation`, `observation`, `alignment`, `trajectory_revision`, `session`.

- A target can be retracted once (`UNIQUE (target_kind, target_id)`). A retraction cannot itself be retracted. To undo one, add the content again as a new row. This keeps the history one-directional, as v1's trigger does.
- Retracting an **observation** is new. It is for a measurement later found bad (a sensor fault, a clock offset, the wrong session). The row stays, immutable; it stops counting from `retracted_txn` on. It is not a deletion and not an absence.
- Retracting a **session** is new too. It is for a whole walk later found bad (the wrong building, a corrupt recording, a clock offset across the walk).
- A **correction** is one transaction: retract the old row and insert its replacement. Both carry the same `recorded_txn`, so no `known_at` can see neither or both. A session is corrected the same way: retract it and ingest the replacement as a new session.
- `retracted_by` is an operator id or `model:<name>@<version>`. A model may retract only interpretations it wrote [proposal]. A model can never retract an observation, alignment, trajectory revision or session ("guesses never flow down", northstar §2).

**Retraction is transitive** [proposal]. Retracting a session or a trajectory revision invalidates all evidence derived from it, and all evidence derived from that. A row is *invalid* at `known_at` when it, or any row it depends on, has a retraction with `retracted_txn ≤ known_at`. An invalid row counts as retracted: it is not visible (§2.1), supports no sighting, refutes nothing and enables no comparison. The dependencies are immutable columns:

| Row | Depends on |
|---|---|
| interpretation | its observation |
| observation | its session; a `map`-frame one also its map revision |
| trajectory revision | its session and its parent revision |
| map revision | the trajectory revision it was built from |
| alignment | its `from_frame` and `to_frame` map revisions |

A `body` observation's position depends on its session's active trajectory revision (§2.3). That is checked per revision, not stored as a column.

- Invalidity is computed at read time from these columns and `retractions`. A retraction writes one row, for its target, and none for what depends on it. Cascade rows would collide with `UNIQUE (target_kind, target_id)` when a dependent row is later retracted on its own, and would make what is stored depend on the graph at write time. Since nothing else is written, every answer at a `known_at` before the retraction is unchanged (V2-3).
- A **trajectory correction** (a new revision, nothing retracted) supersedes instead. The rows derived from the old revision are not invalid, and answers before the new revision's `recorded_txn` keep them. From that transaction on, they take part in no conclusion: their frame is no longer current (§2.3, §2.5). They remain immutable and readable as superseded history; excluding them from a current conclusion never deletes them or their interpretation links. Supersession is computed at `known_at`, not written back to the evidence rows.
- Either way, every conclusion that depended on the evidence is recomputed without it. An absence from a retracted session cannot refute a sighting from another session, and a "moved" claim or absence that used it is superseded (§2.5). Only a look-again closure stays as it is, because it records an action, not evidence (§2.1).

### 2.3 Map and trajectory revisions

A walk produces a trajectory, and the map is built from it. The two are revised together.

- `trajectory_revisions(trajectory_revision_id, session_id, revision, tum_sha256, parent_revision_id, method, method_version, params_sha256, reason, recorded_txn)`. Revision 1 is the session's FAST-LIO output. A loop-closure back end, an offline bundle adjustment or a timing fix adds revision 2, with its parent.
- `map_revisions(map_id, revision, map_pcd_sha256, trajectory_revision_id, built_by, recorded_txn)`. A map revision names the trajectory revision it was built from. Every revision is kept. This is room memory's `map_epoch`, without the overwrite.
- A **frame** is `(map_id, revision)`. Every stored position names its frame, as v1's positions name their `map_id`.
- The session's **active** trajectory revision at `known_at` is its newest revision visible at `known_at` (§2.1). A retracted revision falls back to its parent. Retracting a revision also invalidates its descendants (§2.2), so the fallback is the newest revision with no retracted ancestor. If there is none, the session's `body` observations have no position (`source_retracted`). A revision rebuilt from the raw recording, not from a parent's poses, is recorded with `parent_revision_id = NULL` [proposal].
- A frame is **current** at `known_at` when its map revision is valid (§2.2) and was built from its session's active trajectory revision. Only current frames take part in conclusions (§2.5).

**Observation geometry** comes in two kinds [proposal]:

| `geometry_frame` | Stored | Position in a map frame | After a trajectory correction |
|---|---|---|---|
| `body` | the measurement relative to the rig at `acquired_at` (a camera detection, a per-scan lidar cluster) | derived: the active trajectory's pose at `acquired_at` (v1's `pose_from_trajectory`) applied to the body geometry | re-derived exactly from the new revision. No row changes |
| `map` | the measurement in one map revision's frame (a cluster or free-space region computed from the accumulated `map.pcd`) | stored, bound to that revision | **not transformed**. It stays in its revision. The pipeline re-extracts it from the new map revision as a new observation with `derived_from` pointing at the same raw recording |

A trajectory correction is generally not rigid: the drift investigation measured local inconsistency of up to 8 cm / 1.7° along one walk (M14b). So a `map`-frame observation cannot be moved by one transform. This is room memory's relearn rule (`objects.cpp:736-760`): old-frame geometry is retired with its frame, not carried over. Pose fields on a `body` observation (v1's `pose_*`, `pose_bracket_s`, `pose_quality`) are derived per trajectory revision. The pose bracket check runs per revision, so a detection can be `valid` under one revision and `invalid` under another.

### 2.4 Validated cross-session transforms

[proposal] Sessions are never written into another session's map. v1's `UNIQUE map_id` stays true in spirit: a session's own observations stay in its own frames. "Many sessions per map" (northstar M3) comes from **alignment records** into a reference map revision:

`alignments(alignment_id, from_frame, to_frame, transform (tx ty tz qx qy qz qw), method, method_version, params_sha256, inlier_frac, rms_m, observe_min_eig, basin_margin, holdout_residual_m, holdout_n, verdict, recorded_txn)`

- `from_frame` is a session's map revision; `to_frame` is a revision of a designated reference map.
- `verdict` is `validated` or `rejected`. As with v1's `pose_quality`, a row claiming `validated` whose metrics fail a gate raises `ValidationError`; it is not stored as `rejected` instead. A failed attempt is stored as `rejected`, so failures stay on record (the workplan's "failed attempts retained").
- **Gates.** All are needed:
  - fit: `inlier_frac`, `rms_m` and `observe_min_eig` within limits. Room memory's rig values (0.9 × baseline, 0.02 m, 0.02; `memory.hpp:304-327`) are placeholders only: they were set for a stationary rig against its own cloud, not for a cart against a building map [provisional];
  - **basin uniqueness:** `basin_margin`, the score gap between the best solution and the best distinct solution from a seed grid. The drift investigation found the identity seed converged to a wrong basin (M1: 1.37 m / +22°, against 1.81 m / −15.2°). Only the 600-seed grid exposed it (M16). One seed is not evidence. Repeating rack aisles (northstar §6) make this the main failure mode;
  - **held-out check:** residual at `holdout_n` points or landmarks not used in the fit (the Codex suggestion in the drift investigation).
- **Star topology, no chaining** [proposal]. A session aligns only directly into a reference revision. Transforms are never composed through other sessions, so errors do not accumulate unseen. A new reference revision needs fresh alignments; old ones stay valid only for the revision they name.
- An alignment can be retracted (§2.2), for example when a later walk shows it locked onto the wrong aisle. Comparisons that used it then become `not_comparable` for every `known_at` from the retraction on. It also stops counting, with no retraction row of its own, while its `from_frame` or `to_frame` is invalid (`source_retracted`) or not current (`superseded_revision`) (§2.2, §2.3).

### 2.5 Comparability: the rule behind "moved" and absence

v2 compares positions only through one function [proposal]:

`compare_positions(obs_a, obs_b, known_at) -> Comparison`, with fields `comparable`, `reason`, `frame`, `distance_m`, `sigma_m`, `revisions_used`.

**Resolved frame.** At `known_at`, each observation resolves to at most one map revision:

- a `map` observation resolves to the revision it is bound to, while that frame is current (§2.3). Otherwise it has no resolved frame, and it is neither a sighting nor an absence for any status at that `known_at`;
- a `body` observation resolves to its session's *active map revision*. That is the newest visible map revision built from the session's active trajectory revision (§2.3). If no such revision is visible yet, it has no resolved frame. A `body` observation whose pose is `invalid` under the active trajectory revision (§2.3) has no position at all, and is not comparable (`invalid_pose`).

Two observations are comparable at `known_at` only when one of these holds:

1. both are `body` observations of the **same session**, compared under that session's active trajectory revision, whether or not a map revision has been built from it yet; or
2. both resolve to the **same map revision**, whatever their `geometry_frame`. A `body` and a `map` observation are comparable when the `map` one is bound to the `body` one's active map revision. This is how a same-session absence, which is always `map`-frame (below), rechecks a `body`-frame sighting; or
3. each resolved frame has a visible `validated` alignment into the **same reference revision**, and the positions are compared there.

Under every rule, both observations must be valid at `known_at` (§2.2); otherwise the reason is `source_retracted`. Under rules 2 and 3, every frame used must be current (§2.3); otherwise the reason is `superseded_revision`.

Anything else returns `comparable = False` with a reason (`different_frames`, `no_validated_alignment`, `alignment_retracted`, `superseded_revision`, `no_map_revision`, `invalid_pose`, `source_retracted`). It never returns a distance. In particular, a position from trajectory revision 1 is never compared with one from revision 2. A `body` observation under T2 against a `map` observation bound to a revision built from T1 is `superseded_revision`. If T2 has no map revision yet, the reason is `no_map_revision`.

`sigma_m` is `sqrt(σa² + σb²)`, plus each alignment's `rms_m` when case 3 applies. The pose leg of a `body` observation adds nothing: v1 records no pose σ, only the bracket check (`pose_quality`), and that check is the only gate on it [provisional].

Uses:
- **Absence.** v1's step 3 (`map_id = s.map_id`, DESIGN §6) becomes "comparable with s", tested in the absence's own frame (below). An absence that is invalid, or whose frame or alignment is no longer current, refutes nothing. The status is recomputed without it.
- **Moved.** v2 does not detect changes (§4). A future detector may call something "moved" only when `compare_positions` is comparable and `distance_m > MOVED_SIGMA_K × sigma_m` [provisional: `MOVED_SIGMA_K = 3`], and the claim stores the `revisions_used` and `known_at`. When a later trajectory revision, a retracted alignment, or a retraction of anything either observation depends on (§2.2) makes that comparison non-comparable, the claim is superseded, never silently kept.

**Absence across frames** [proposal]. An absence region and its one coverage number are exact only in the frame where the region was measured, its *native* frame. v2 never re-boxes that region in another frame. Re-boxing a rotated region adds corners nobody measured, and v1's tightness test can still accept a footprint sitting in one of them (V2-5e: the 45° re-box of `[-0.03, 0.03]²` passes `_region_is_tight` for a footprint at `[0.033, 0.039]²`, which lies wholly outside the measured square). Instead, s's footprint is brought into the absence's native frame and v1's step 3 runs there:

1. **Map the footprint.** Take the 8 corners of s's footprint (DESIGN §6: its AABB, else its centroid) into the absence's native frame. For two frames with alignments, the path is s's validated alignment into reference revision B, then the inverse of the absence frame's validated alignment into the same B. These are two star edges into one reference, not a chain (§2.4). Between two `map` observations in the same revision, it is the identity. For a `body` sighting, the corners start in its body frame, and the first leg is its pose under the active trajectory revision: a rigid transform, rotation included. They are never projected into the map and re-boxed first. That is the re-boxing trap above, with no alignment involved: the box would hold corners nobody measured and would loosen the margin in step 3. The pose leg adds nothing to the growth in step 2 (see `sigma_m` above).
2. **Containment.** Grow the AABB of the mapped corners on every side by the sum of the alignments' `rms_m`. That box must lie inside the region (inclusive). It is a superset of the true footprint, so passing it implies the real footprint is inside the measured region.
3. **Margin.** Map the region's 8 corners the other way, into s's frame (for a `body` sighting, its body frame). They must lie inside s's footprint grown by `ABSENCE_MAX_MARGIN_M` per side. This is measured against the true footprint, not the grown box from step 2, so the margin never loosens.
4. **Coverage** is the absence's recorded number, for its region as recorded. It is never rescaled or reused for a derived region.

With the identity transform and `rms_m = 0`, steps 2 and 3 are exactly v1's `_region_is_tight`. The original design author reported no disagreement on 20,000 random box pairs against `90ae03e` (unmerged branch evidence, §0); this is not a merged acceptance test. Under a rotation, a region counts only if it holds the whole rotated footprint and still fits within footprint + margin. Otherwise it does not count, and s stays `LAST_SEEN`.

Absences are `map`-frame observations only [proposal]. A per-scan (`body`-frame) free-space region would turn with the pose, which is the same re-boxing problem with no alignment involved. Ingest refuses one with `ValidationError`, until it is designed with this test run per trajectory revision.

### 2.6 Immutable evidence

- **Observation rows stay immutable**, as v1 (`store.py:368-372`). v2 adds nothing that updates them. Derived values (pose, map-frame position) are computed per revision, not stored back.
- **Raw inputs are bound by hash** [proposal]: sessions gain `source_recording_sha256`, alongside v1's `trajectory_sha256` and `map_pcd_sha256`. Ingest refuses an observation whose `derived_from` names bytes that do not hash to the recorded value, the same pattern as `_check_map` (`wall_scoring.py:297-314`).
- **Derivations name their code**: `method`, `method_version`, `params_sha256` on trajectory revisions, map revisions, alignments and re-extracted observations.
- **The code contradicts strict immutability in one place:** the purge deletes camera rows under the 30-day retention policy (DESIGN §8, `store.py:1825` on). v2 keeps the purge, because the retention policy is Trevor's privacy rule, and records each deletion instead, in two tombstone tables:
  - `purged_observations(observation_id, kind, session_id, sensor_kind, acquired_at, recorded_txn, purged_txn, row_sha256)`, one row per deleted observation;
  - `purged_associations(interpretation_id, observation_id, object_id, recorded_txn, purged_txn)`, one row per deleted interpretation, retracted or not. This is the part v1 loses: its cascade deletes the interpretation, and `objects.purged_sightings` keeps only a per-object count at purge time, so it cannot say which object an observation supported at an earlier `known_at`.

  Neither table holds image data, a label, a confidence or geometry. Rows in `retractions` (§2.2) are never purged. Their targets may be deleted, but the retraction stays, so an association's retracted state at any `known_at` can still be worked out.

  **When an answer is incomplete.** `object_status(X, now, known_at)` returns `complete = False` and lists in `purged_ids` every purged observation D with a tombstoned association A where all of these hold:
  - `A.object_id = X`;
  - `A.recorded_txn ≤ known_at`, and A has no retraction with `retracted_txn ≤ known_at`;
  - `D.recorded_txn ≤ known_at`, and D is not invalid at `known_at`: neither D nor its session has a retraction with `retracted_txn ≤ known_at` (§2.2). The tombstone keeps `session_id` but not D's trajectory revision, so a trajectory retraction cannot be checked here and D stays listed. That over-reports, as below;
  - `D.acquired_at ≤ now`.

  `purged_txn` is not compared. A purge is not a retraction: the evidence was known and is now missing, both before and after the purge. The tombstone does not keep the inputs to v1's sighting gates (confidences, pose validity). Pose validity depends on the trajectory revision anyway (§2.3). So the flag means "this answer **may** have depended on purged evidence". It can over-report and never under-reports. In the other direction, the status reason uses the same set: X with no retained sighting and a non-empty `purged_ids` is `UNKNOWN / no_retained_sightings` (v1 DESIGN §6 step 2), never `never_seen`.

  A historical query never quietly gives a different answer.

### 2.7 Unaligned sessions stay separate

- A session with no visible `validated` alignment lives only in its own frames. That is exactly v1's behaviour: its absences cannot refute another session's sightings, and its places are never compared with another session's.
- A `rejected` or retracted alignment counts as none, and so does one that is invalid or not current (§2.4).
- `object_status` reports which frame its answer is in. A sighting in an unaligned session can still be `CURRENT` or `LAST_SEEN` in that session's frame. It is never reported in the reference frame.
- `describe()` keeps v1's wording and appends `; not aligned to the building map` when the sighting's frame has no validated alignment [proposal].

### 2.8 Query shape

`object_status(object_id, now, known_at=None, frame=None) -> ObjectStatus` [proposal]:

- `frame=None` means "the resolved frame of the newest sighting" (§2.5). A `body` sighting with no resolved frame reports its session and active trajectory revision instead, and only rule 1 of §2.5 applies to it. A reference frame may be named. Only aligned evidence takes part then.
- The return adds `known_at` (the `txn_id` used), `frame`, `revisions_used`, `complete`, `purged_ids` and `purged_unattributed` (§2.9) to v1's fields. v1's `purged_sightings` becomes `len(purged_ids) + purged_unattributed` at `known_at`. `open_look_again` is X's requests that are open at `known_at` (§2.1), not their state at the time of the query. `measured` and `hypothesis` stay separate sub-records (v1 invariant I16).
- A **report** is canonical JSON of statuses plus a fingerprint: `schema_version`, `constants_sha256` (a hash of every constant in DESIGN §5 and v2's new ones), `known_at`, `now`. Regenerating a report on a store whose `constants_sha256` differs raises `ReproducibilityError`. It never recomputes the answer under new thresholds.

### 2.9 Migration from v1

[proposal]
- v1 refuses `user_version = 2` (invariant I23, `store.py:1026-1030`). So the migration copies into a new file and never upgrades in place.
- Each v1 map becomes `map_revisions` revision 1. Each session's `trajectory_sha256` becomes trajectory revision 1. Every v1 observation becomes a `map`-frame observation bound to revision 1, because v1 stored map-frame poses.
- **The first trajectory correction preserves migrated evidence.** Adding T2 (parent T1) and its map revision R2 does not delete, rewrite, rebind or retract any migrated observation, interpretation or revision-1 geometry. R1 and T1 remain stored. At `known_at` from T2's transaction on, the migrated observations remain readable in their original R1 frame as history with the derived reason `superseded_revision`; they support no current sighting, absence or moved claim. History lookup by observation id must not apply the current-frame filter from §2.5. At a `known_at` before T2, they still support the same historical answers. A correction must not run a retention purge or create purge tombstones for these rows; the independent camera-retention policy (§2.6) is the only deletion exception.
- Re-extraction from R2 appends new observations with new ids and their own interpretations and derivation provenance (§2.3, §2.6). It never replaces the migrated rows or copies their geometry into R2. If re-extraction is unavailable, current status in R2 is `UNKNOWN` when no other eligible evidence exists, while the old evidence remains inspectable as superseded history. A migration cannot invent body-frame measurements to make old rows usable under T2.
- Transactions are synthesized in `created_at` order (ties by table, then id). v1 `retracted_at` becomes a retraction row ordered the same way. Every synthesized transaction is flagged `knowledge_basis = 'migrated'`, because v1 wall times were never a knowledge record: `known_at` before the migration point is approximate.
- Each closed v1 request gets a `look_again_closures` row in a transaction synthesized at its `closed_at`, ordered and flagged like the rest. A v1 fulfilment whose observation was already purged has a NULL `fulfilled_by_observation_id`. That is the only case where v2 allows NULL on a `fulfilled` closure, and it applies only to `migrated` transactions.
- `thumbnail_gc` may or may not exist (§1). The migration accepts both.
- A v1 file that has already purged rows keeps only `objects.purged_sightings`; which observations and associations were lost cannot be recovered. The migration copies each non-zero count into `purged_unattributed(object_id, n, recorded_txn)`, with `recorded_txn` set to the migration transaction. For every `known_at` at or after it, that object reports `complete = False` with `purged_unattributed = n` and no ids it cannot name. Its reason stays `no_retained_sightings` when nothing is retained, as in v1.

## 3. Acceptance scenarios (future tests)

These are written to become `spatio/memory/tests/test_v2_*.py`, one test per case. They use v1's notation (DESIGN §9): `t0 = 1.76e9`, a "valid detection of X" with `measurement_confidence = 0.9` and a model interpretation linking it to X at confidence 0.9, and "absence over X" with coverage 1.0 and a region equal to X's AABB. `K1 < K2 < …` are transaction ids in write order. One session S1 spans `[t0, t0 + 200]` unless stated. S1 has trajectory revision T1 and map revision R1 built from it. Unless stated, a detection is `body`-frame with an identity pose under T1 throughout, so its footprint is X's AABB in R1. An absence is bound to R1. V2-1 is therefore a body/map recheck, compared under rule 2 of §2.5.

### V2-1. A late observation does not rewrite the past

`test_V2_1_late_absence_visible_only_after_ingest`
- Given a valid detection of X at `t0` (recorded K1), and an absence over X acquired at `t0 + 5` but ingested later (recorded K2).
- At `now = t0 + 10, known_at = K1`: `CURRENT`, `measured.acquired_at == t0`, `absence is None`.
- At `now = t0 + 10, known_at = K2`: `NOT_SEEN_ON_RECHECK`, `absence.acquired_at == t0 + 5`.
- With `known_at=None`, the answer equals `known_at = K2` and reports `known_at == K2`.
- The `known_at = K1` answer is identical (canonical JSON) before and after K2 is written.

`test_V2_1b_late_observation_does_not_fulfil_earlier_request`
- Given a look-again request for X created at wall time W.
- An observation acquired at sensor time `W − 60` and ingested after W does not fulfil it (`ValidationError`). One acquired at `W + 1` does.

### V2-2. A label correction keeps the old answer reproducible

`test_V2_2_label_correction`
- Given a valid detection D of X at `t0` with a model interpretation "chair" (conf 0.9, recorded K1).
- The operator corrects it at K2 in one transaction: retract "chair" (reason "is a stool") and add an operator interpretation "stool" on D for X.
- At `now = t0 + 10`: `known_at = K1` gives `hypothesis.label == "chair"`, `source_kind == model`; `known_at = K2` gives `"stool"`, `operator`. `measured` is the same row both times.
- No `known_at` sees both labels active, or neither.
- Retracting the retraction raises `ValidationError`.

`test_V2_2b_identity_correction`
- Given the same D, retracted at K2 with no replacement (it was object Y, not X).
- X at `now = t0 + 10, known_at = K2` is `UNKNOWN / never_seen`. At `known_at = K1` it is `CURRENT`.

`test_V2_2c_retracted_observation`
- Given D retracted as an observation at K2 (sensor fault).
- At `known_at = K2`, X is `UNKNOWN`. D still exists in `observations`, and an `UPDATE` on it still raises (I18).

### V2-3. A historical report is reproducible ("as known at T")

`test_V2_3_report_as_known_at_T`
- Given objects X and Y, three sessions' worth of detections and absences, one validated alignment, recorded through K5. Report R = `report(now = t0 + 500, known_at = K5)`.
- Then write K6–K9: a late absence, a label correction, a new trajectory revision for one session, and a retraction of the alignment.
- `report(now = t0 + 500, known_at = K5)` is byte-identical to R, fingerprint included.
- `report(now = t0 + 500, known_at = K9)` differs, and its `known_at` is K9.

`test_V2_3b_purge_is_never_silent`
- Given a camera detection of X used in R (recorded K1), then `purge_expired` at its `expires_at` (K2).
- `report(..., known_at = K1)` returns X with `complete = False` and `purged_ids == [that id]`. It does not return X as `UNKNOWN / never_seen` without that flag.

`test_V2_3c_constants_change_refuses`
- Given R from a store whose constants hash is H. Opening a store with any changed constant and regenerating R raises `ReproducibilityError`.

`test_V2_3d_purge_after_identity_correction`
- Given a camera detection D at `t0`, attributed to X by a model interpretation (K1). At K2, one transaction retracts it and adds an operator interpretation linking D to Y. At K3, `purge_expired` deletes D. X and Y have no other evidence. Every query below runs on the store after K3, with `now = t0 + 10`.

  | Query | Expected |
  |---|---|
  | X at K1 | `UNKNOWN / no_retained_sightings`, `complete = False`, `purged_ids == [D]` |
  | X at K2, K3 | `UNKNOWN / never_seen`, `complete = True`, `purged_ids == []` |
  | Y at K1 | `UNKNOWN / never_seen`, `complete = True` |
  | Y at K2, K3 | `UNKNOWN / no_retained_sightings`, `complete = False`, `purged_ids == [D]` |

- Control, on a copy of the store taken before K3: X at K1 and Y at K2 are both `CURRENT`, with `measured.observation_id == D`.
- The column set of `purged_observations` and `purged_associations` contains no label, confidence, thumbnail path or geometry column.
- The retraction written at K2 still exists after K3.

`test_V2_3e_purge_flag_respects_now_and_observation_retraction`
- D as in V2-3d, attributed to X only, purged at K3. X at `now = t0 − 1` has `complete = True` (D was not yet acquired).
- A second camera detection E of X at `t0 + 5`, retracted as an observation at K4 and purged at K5. X at `known_at ≥ K4` does not list E in `purged_ids`.

`test_V2_3f_request_closure_keeps_old_report`
- Given a lidar detection D of X at `t0` (K1), which the purge never deletes. Look-again requests A and B for X are created at injected wall times `t0 + 20` (K2) and `t0 + 30` (K3). A camera detection F of X is acquired at `t0 + 50` (K4). Every report below uses `now = t0 + 60`. R3 = `report(known_at = K3)` and R4 = `report(known_at = K4)` are saved. Both have `open_look_again == [A, B]`.
- A is fulfilled by F at K5, and B is cancelled at K6. Afterwards `report(known_at = K3)` and `report(known_at = K4)` are byte-identical to R3 and R4. At K5, `open_look_again == [B]`. At K6 it is `[]`.
- An `UPDATE` on a request or closure row raises, and so does a second closure of A.
- `purge_expired` deletes F at K7. R3 is still byte-identical, since F is not recorded at K3. At K4, K5 and K6 the open lists are still `[A, B]`, `[B]` and `[]`, and A's closure still names F. The rest of those answers changes as §2.6 says: the sighting falls back to D, 60 s old, so X becomes `LAST_SEEN` with `complete = False` and `purged_ids == [F]`.
- On a copy of the store taken before K7, F retracted as an observation at K7 leaves A closed: at K7, `open_look_again == []`.

### V2-4. A trajectory correction makes no false "moved"

The drift investigation's lesson in test form: a pose error must not read as an object moving.

`test_V2_4_trajectory_correction_body_frame`
- Given session S1 with trajectory revision T1. Body-frame detections of X at `t0 + 10` and `t0 + 100`, both σ = 0.03 m. Under T1 they project to (2.00, 0, 0.5) and (2.30, 0, 0.5). T1 drifts by +0.30 m in x after `t0 + 50`.
- Under T1 (`known_at` before T2): `compare_positions` is comparable, `distance_m ≈ 0.30`, `sigma_m ≈ 0.042`. The 3σ predicate is true, so a detector would say "moved". The claim records `revisions_used = {T1}`.
- Revision T2 is recorded (parent T1). It removes the 0.30 m drift after `t0 + 50`.
- Under T2: both project within 1 mm of (2.00, 0, 0.5). `distance_m < 0.001`, and the predicate is false.
- The T1 claim is reported superseded at `known_at ≥ T2's txn`. At `known_at` before T2 it is still reported, unchanged (V2-3).
- No observation row changed (row hashes are equal before and after).

`test_V2_4b_mixed_revisions_refuse`
- With T1 and T2 both stored, comparing the first sighting under T1 with the second under T2 returns `comparable = False, reason = "superseded_revision"`, and `distance_m is None`.

`test_V2_4c_map_frame_not_transformed`
- Given a `map`-frame detection of X bound to map revision R1 (built from T1), and T2 recorded with map revision R2.
- The R1 detection is not projected into R2. Comparing it with an R2 observation returns `comparable = False`.
- X's status in frame R2 uses only observations re-extracted from R2. With none yet, X in R2 is `UNKNOWN`. From T2's transaction on, R1 is not current (§2.3), so the R1 detection supports no status in any frame, R1 included. At `known_at` before T2, X in R1 is unchanged (V2-3).

`test_V2_4d_body_sighting_map_absence_same_revision`
- Given a body-frame detection D of X at `t0` (K1), footprint `[0.5, 0.7] × [-0.1, 0.1] × [0, 0.4]` in the body frame. T1's pose at `t0` is a 20° yaw about z with translation (2, 1, 0), valid. Its corners map into R1 with AABB about `[2.436, 2.692] × [1.077, 1.333] × [0, 0.4]`.
- An absence A in R1 at `t0 + 5` (K2) has region `[2.42, 2.71] × [1.06, 1.35] × [-0.01, 0.41]` and coverage 1.0. Its corners in D's body frame span about `[0.415, 0.787] × [-0.187, 0.185] × [-0.01, 0.41]`, within the footprint + 0.10 m.
- At `now = t0 + 10, known_at = K2`: `compare_positions(D, A)` is comparable with `frame == R1` and `revisions_used == {T1, R1}`. X is `NOT_SEEN_ON_RECHECK` with `absence.observation_id == A`.
- At `known_at = K1` X is `CURRENT` (V2-1).
- Control, the re-boxing trap: with a 45° yaw instead, D's corners map to about `[2.283, 2.566] × [1.283, 1.566] × [0, 0.4]`, and A's region is `[2.27, 2.58] × [1.27, 1.58] × [-0.01, 0.41]`. Re-boxing D into R1 and running v1's `_region_is_tight` on the re-box passes; assert that as a precondition. A's corners in D's body frame span about `[0.382, 0.820] × [-0.219, 0.219]`, about 0.02 m beyond the footprint + 0.10 m. So A does not count, and X stays `CURRENT` with `absence is None`.

`test_V2_4e_body_sighting_map_absence_mismatched_revision`
- The same D and A, then trajectory revision T2 (parent T1, same pose at `t0`) at K3, and map revision R2 built from T2 at K4.
- At `known_at = K3`: `compare_positions(D, A)` returns `comparable = False, reason = "no_map_revision"`. At `known_at = K4`: `reason = "superseded_revision"`. `distance_m is None` both times.
- At `now = t0 + 10`, X is `CURRENT` with `absence is None` for `known_at ≥ K3`, in frame R2 from K4 on. At `known_at = K2` it is still `NOT_SEEN_ON_RECHECK`, byte-identical to the answer before K3 (V2-3).
- A map-frame absence A2 re-extracted from R2 with A's region (K5) makes X `NOT_SEEN_ON_RECHECK` again for `known_at ≥ K5`.

### V2-5. Unaligned sessions stay separate

`test_V2_5_unaligned_absence_does_not_refute`
- Given a sighting of X in S1 and a later absence over the same coordinates in S2, with no alignment between S1, S2 and any reference.
- X is `LAST_SEEN` (v1 I12, unchanged). `compare_positions` between them returns `no_validated_alignment`.

`test_V2_5b_validated_alignment_enables_comparison`
- Given S1 and S2 each with a `validated` alignment into reference revision B1.
- The S2 absence, which passes the native-frame test (§2.5) against X's S1 footprint, makes X `NOT_SEEN_ON_RECHECK` in frame B1.
- Retracting S2's alignment at a later K returns X to `LAST_SEEN` for `known_at ≥ K` only.

`test_V2_5c_alignment_gates`
- A `validated` alignment with `basin_margin` below its gate, or `holdout_n = 0`, raises `ValidationError`, and nothing is written.
- The same row as `rejected` is stored and enables no comparison.

`test_V2_5d_no_chaining`
- Given S1 → B1 validated and S2 → S1 validated (`to_frame` not a reference revision): the second insert raises `ValidationError`.

`test_V2_5e_rotated_absence_unmeasured_corner`
- Given S1 → B1 validated as the identity, and S2 → B1 validated as a 45° rotation about z. Both have `rms_m = 0.002`, so the containment growth is 0.004 m.
- The S2 absence has native region `[-0.03, 0.03]² × [-0.02, 0.12]` and coverage 1.0, acquired after X's sighting.
- X's S1 sighting has footprint `[0.033, 0.039]² × [0, 0.1]`. The 45° re-box of the region into B1 passes v1's `_region_is_tight` for it; assert that as a precondition, so the test shows the trap it guards against. The footprint mapped into S2's frame spans x ∈ [0.0467, 0.0552], outside the region. So the absence does not count, and X stays `LAST_SEEN` in frame B1 with `absence is None`.
- Control: with footprint `[-0.01, 0.01]² × [0, 0.1]` instead, the mapped footprint grown by 0.004 m spans ±0.0181 m and lies inside the region. The region's corners in S1's frame stay within the footprint + 0.10 m. X is `NOT_SEEN_ON_RECHECK` in frame B1.
- With both alignments the identity, the result equals v1's `_region_is_tight` on the same boxes.

`test_V2_5f_body_frame_absence_refused`
- A `lidar_free_space` observation with `geometry_frame = 'body'` raises `ValidationError`, and nothing is written.

### V2-6. Migration keeps v1's answers

`test_V2_6_migrated_v1_equivalence`
- Given a v1 database built by the v1 invariant fixtures (I1–I29), with and without `thumbnail_gc`.
- After migration, for every object and every `now` the fixtures use, v2 `object_status(..., known_at=None)` equals v1's `ObjectStatus` field for field (v2's added fields aside). `describe()` text is identical.
- The v1 file is unchanged (byte hash).

`test_V2_6b_migrated_purge_stays_incomplete`
- Given a v1 database after I27's purge, so X has `purged_sightings = 1` and no rows.
- After migration, X at `known_at=None` is `UNKNOWN / no_retained_sightings`, `purged_sightings == 1`, `complete = False`, `purged_unattributed == 1`, `purged_ids == []`, as v1 reports it.

`test_V2_6c_first_correction_preserves_migrated_evidence`
- Given a v1 session S1 with a valid lidar detection D of X at `t0`, its interpretation I, and a lidar absence A over X at `t0 + 5`. Migrate it through transaction K1 into trajectory T1 and map R1. Neither observation is subject to the camera purge. Every query uses `now = t0 + 10`. Save the migrated row ids and hashes for D, I, A, T1 and R1, the source v1 file hash, and report H at `known_at = K1`. H reports `NOT_SEEN_ON_RECHECK` with `measured.observation_id == D` and `absence.observation_id == A`.
- At K2 append the session's first corrected trajectory T2 (parent T1) and map R2, with no re-extracted evidence yet. All saved row ids, hashes, geometry and interpretation links remain unchanged. No retraction or purge tombstone is added, and the source v1 file hash is unchanged.
- At `known_at = K2`, history still returns D and A in R1, with I still linked to D and derived reason `superseded_revision`. X's current status in R2 is `UNKNOWN` with no measured sighting or absence; the old absence refutes nothing. Save the report at K2. Regenerating H at K1 is byte-identical, fingerprint included.
- At K3 re-extract detection D2 and absence A2 from R2, retaining the original acquisition times and recording new ids, a new interpretation I2 and the raw-recording/code provenance. X in R2 becomes `NOT_SEEN_ON_RECHECK` using only D2 and A2. D, I and A remain unchanged and readable in R1. Comparing D with D2 returns `comparable = False, reason = "superseded_revision", distance_m = None`; the correction cannot create a moved claim.
- After K3, reports at K1 and K2 are byte-identical to those saved at those transactions. An `UPDATE` of D or deletion outside the retention purge still raises, as in v1. Repeat with and without v1's `thumbnail_gc` table.

### V2-7. Retracting or correcting a source invalidates what was derived from it

Unless stated, S1 and S2 are each aligned `validated` into reference revision B1, as in V2-5b. S2 spans `[t0 + 100, t0 + 300]`, with trajectory revision U1 and map revision Q1 built from it; its alignment is from Q1. D is a valid detection of X in S1 at `t0`. A is an absence over X in S2, bound to Q1, at `t0 + 150`, which passes the native-frame test (§2.5) against D. Both are recorded through K1. Every query uses `now = t0 + 200`.

`test_V2_7_retracted_session_absence_cannot_refute`
- A camera detection E of Y in S2 at `t0 + 160` is recorded at K1 and purged at K2. Y has no other evidence.
- At `known_at = K2`: X is `NOT_SEEN_ON_RECHECK` in frame B1, with `absence.observation_id == A`. Y is `UNKNOWN / no_retained_sightings` with `purged_ids == [E]`.
- At K3 the operator retracts S2 (reason "clock offset").
- At `known_at = K3`: X is `LAST_SEEN` in frame B1 with `absence is None`. `compare_positions(D, A)` returns `comparable = False, reason = "source_retracted"` and `distance_m is None`. Y is `UNKNOWN / never_seen`, `complete = True`, `purged_ids == []`.
- At `known_at = K2`, both answers are byte-identical to those before K3 (V2-3).
- K3 added exactly one row to `retractions`, targeting S2. A, U1, Q1 and S2's alignment have no retraction row of their own. Retracting A afterwards succeeds and changes no answer.
- Control: an absence A3 over X at `t0 + 180` in a third aligned session S3 (K4) makes X `NOT_SEEN_ON_RECHECK` again for `known_at ≥ K4`, with `absence.observation_id == A3`.
- A model writer retracting a session raises `ValidationError`, and nothing is written.

`test_V2_7b_retracted_trajectory_revision_is_transitive`
- At K2, U1 is retracted.
- At `known_at = K2`: Q1, S2's alignment and A are invalid, with no retraction rows of their own. X is `LAST_SEEN` with `absence is None`, and `compare_positions(D, A)` gives `source_retracted`. A `body` detection in S2 has no position, and comparing it gives `source_retracted`.
- Descendants. On a copy of the store taken before K2: U2 (parent U1), Q2 built from U2, a `validated` alignment Q2 → B1, and an absence A2 re-extracted from Q2 with A's region are recorded at K2. X is `NOT_SEEN_ON_RECHECK` with A2. Retracting U1 at K3 invalidates U2, Q2, that alignment and A2 too: X is `LAST_SEEN` for `known_at ≥ K3`.
- Rebuilt root. On the same copy, U3 with `parent_revision_id = NULL`, Q3 built from it, an alignment Q3 → B1 and a re-extracted absence A3, recorded at K4, are not invalidated by U1's retraction. X is `NOT_SEEN_ON_RECHECK` with A3 for `known_at ≥ K4`.

`test_V2_7c_trajectory_correction_supersedes_derived_evidence`
- At K2, U2 (parent U1) is recorded. Nothing is retracted.
- At `known_at = K2`: Q1 is not current. `compare_positions(D, A)` gives `superseded_revision`, and the Q1 → B1 alignment enables no comparison. X is `LAST_SEEN` with `absence is None`. A and the alignment each have no retraction and the alignment is `validated`, so this is the case rule 3 alone would let through.
- At K3: Q2 built from U2, a `validated` alignment Q2 → B1, and an absence A2 re-extracted from Q2 with A's region. X is `NOT_SEEN_ON_RECHECK` with `absence.observation_id == A2` for `known_at ≥ K3`.
- At `known_at = K1`, the answer is byte-identical to the one before K2.

`test_V2_7d_moved_claim_recomputed_after_retraction`
- Instead of A, a body-frame detection F of X in S2 at `t0 + 150` (K1). D and F are σ = 0.03 m, both alignments have `rms_m = 0.002`, and they are 0.5 m apart in B1. `compare_positions(D, F)` is comparable with `sigma_m ≈ 0.046`. The 3σ predicate is true, and a claim records `revisions_used = {T1, R1, U1, Q1}` and both alignments. X's newest sighting is F.
- S2 is retracted at K2. At `known_at = K2`: the claim is reported superseded, `compare_positions(D, F)` gives `source_retracted`, and X's status is recomputed from D alone (`measured.observation_id == D`).
- At `known_at = K1`, the claim and X's status are reported unchanged (V2-3).

## 4. Deferred, deliberately

These are out of v2 and not designed here:

- **Temporal occupancy and change detection.** Appeared, disappeared and moved regions from free-space evidence between walks (northstar layer 2, M4). v2 only fixes the rule they must obey (§2.5).
- **Recheck relevance.** Ranking or prioritising look-again requests (northstar layer 6, M7). v2 keeps v1's request fields and fulfilment rule. It changes only how a request closes, which becomes append-only (§2.1).
- **ConceptGraphs, GNN and GraphRAG.** Open-vocabulary scene graphs, learned graph models and graph-retrieval language answers (northstar layers 4–5). If they come, they write interpretations only.

Also unchanged from v1: no object merge or split, no named places, a single local writer, no camera capture.

## 5. Questions for Trevor

1. Approve schema v2 at all (DESIGN §3 requires it), and the star topology (§2.4) over chained alignments.
2. Set the alignment gates for a cart against a building map: fit, basin margin and held-out residual. The rig values are placeholders.
3. Set `MOVED_SIGMA_K` (3 proposed) for the comparability predicate, and confirm that a `body` observation's pose adds 0 to `sigma_m` and to the absence containment growth (§2.5) until a pose σ is recorded.
4. Allow observation retraction (§2.2)? I recommend yes, operator-only.
5. Retention against reproducibility: confirm that a purge may make an old report `complete = False`, rather than keeping images longer. Two parts of this conflict with the workplan's wording that "derived records are kept locally for 30 days" (v1 DESIGN §8), so they need your decision, not my assumption:
   - `purged_associations` keeps an object association, which is a derived record, past 30 days. It holds ids and transaction numbers only.
   - A retraction's free-text `reason` stays after its target is purged. That text may name what the image showed. The alternative is to blank `reason` in the purge, keeping the row.
   - A look-again closure keeps `fulfilled_by_observation_id` after that camera observation is purged (§2.1), where v1 nulls it. When the request names an object, that link is an object association kept past 30 days. It holds ids only. The alternative is v1's: drop the id at purge, which makes the closure row mutable.
6. Approve transitive invalidation (§2.2): retracting a session or trajectory revision invalidates everything derived from it, including child trajectory revisions, so a rebuild from raw data must be recorded with no parent. A trajectory correction supersedes the old revision's map frame, alignments and map-frame observations for current conclusions, while preserving them as immutable history. Two consequences follow. A new trajectory revision on the reference map's session makes B1 not current, so every alignment into it stops counting until the sessions are aligned again. A migrated session's first new trajectory revision likewise excludes its revision-1 observations from current conclusions, but keeps those observations and their interpretations readable as superseded history; historical answers before the correction remain reproducible (§2.9, V2-6c).
