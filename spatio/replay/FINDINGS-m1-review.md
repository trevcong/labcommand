Plan (before implementation):
1. Preserve child shutdown exit codes and require successful recorder/FAST-LIO shutdown for replay success.
2. Export registered points before cosmetic filtering as the scoring map, retaining a separate downsampled display map with explicit paths.
3. Add synthetic regressions for both findings, including failure propagation and multiple configurable recordings.
4. Run the complete replay test package, record exact results and limits, then commit locally if Git metadata is writable.

Built:
- P1: `stop_proc` captures `wait`'s status before logging and returns it. The
  runner captures recorder, watcher and FAST-LIO shutdown statuses, logs them,
  and returns the first nonzero step status even if exports succeed. It still
  rejects FAST-LIO exiting before shutdown. No nonzero shutdown code is excused.
- P2: `bag_to_map.py` streams every finite registered XYZ observation to
  `map.pcd`, including repeats. The existing copied voxel saver now writes
  `map_display.pcd`. Both paths and point counts are explicit in the map summary;
  replay provenance and the README describe their roles.
- Contract correction: `map.pcd` is now the scoring map before cosmetic
  downsampling; `--map-voxel` affects only `map_display.pcd`. This lets existing
  callers that score `map.pcd` receive the intended evidence without editing
  another workstream. Summary fields are `scoring_map`, `points_written`,
  `scoring_voxel_m` (null), `display_map`, `display_points_written`, and
  `display_voxel_m`. Historical copy mode remains unchanged.

How to run:
- Full package tests: `PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover -s spatio/replay/tests`.
- Existing replay command: `spatio/replay/replay.sh RECORDING RUN_ID --config CONFIG`.
  Score the resulting `map.pcd`; view `map_display.pcd`.
- Inside the ROS environment: `python3 spatio/replay/bag_to_map.py OUTPUT_BAG map.pcd --display-out map_display.pcd --topic REGISTERED_TOPIC --voxel 0.03 --summary map_summary.json`.
  Output names, topic and display voxel size are configurable.

Measured local results:
- Baseline at `4e3527c`: 45 tests passed.
- Final package: 57 tests passed, 0 failures, 0 errors, 0 skips (3.225 seconds).
  The 12 added tests cover both findings; existing tests were not changed.
- Both targeted regression tests were also executed against the original HEAD
  code loaded in memory: exactly 2 expected assertion failures, 0 errors.
  Original shutdown: child exit 42 became 0. Fixed shutdown: exit 42 remains 42;
  a SIGINT-ignoring child killed at the deadline returns 137; clean exit returns 0.
- Runner-tail checks cover success, premature FAST-LIO exit and each of six
  failed steps while readable partial output files exist.
- Synthetic recording A: 4 finite observations (including a repeated point)
  remain in the scoring PCD; the display PCD has 2 points at 0.1 m voxels.
  NaN/infinite XYZ entries are excluded.
- Independent synthetic recording B uses another topic/frame and coordinates:
  changing display voxels from 0.01 m to 0.5 m changes display counts from 3 to 2,
  while scoring retains the identical 4 observations. No settings were fitted
  to either historical recording.
- `bash -n spatio/replay/run_in_container.sh spatio/replay/replay.sh` and
  `git diff --check` passed.

Limits and handoff:
- Tests run locally with real child processes and real binary PCD output.
  ROS/bag interfaces and the external display saver are test stand-ins. No
  container, FAST-LIO replay, Spark access, app launch, Swift build or physical
  accuracy evaluation was performed. Only the replay folder changed.
- The scoring map retains FAST-LIO's published registered XYZ, not sensor points
  already removed inside FAST-LIO. It is streamed to disk but is larger than the
  display map; downstream memory/runtime at full walk scale is not measured.
- Existing historical copied maps retain unknown filtering provenance. They are
  not restored to pre-downsampling evidence by this change. Historical evidence
  remains "replay demonstrated; physical accuracy not evaluated".
- No Trevor approval is needed for these local fixes. Physical runs and Spark
  execution still require the separate approval described in the plans.
  `/Users/tconf/Developer/labcommand/.git/worktrees/spatio-m1-replayfix` returned

Commit status: committed by the orchestrator (Codex cannot write Git metadata).
