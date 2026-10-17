# Room rhythm log (baseline v1)

Log each lidar's occupied fraction per region once a minute, for a later fit of the daily rhythm. Bin keys `close_window` already built. Off unless `rhythm_enable`. With it on, disk follows a directory; the topic stays off unless `rhythm_publish`.

## Where

Both nodes in this checkout are the protocol v2 engine (Activity, pose residual, `publisher_generation`): `spark/room_memory` and `spark/room_memory_cpp`. Differential tests compare those two. The `.rhy` layout is unchanged. `RhythmMinute` ends with `publisher_generation` (protocol §1.4, §3.9, D44); the hand codec omits it on the legacy layout. The labcommand-spark worktree is not edited.

Mac `close_window` without the logger is ~16.5 ms per 1 s window (one thread, both sensors). No SSH. Clouds and status share that thread, so the callback only enqueues. One writer does `write`, `fsync`, rotate, and prune.

## Sample

A region is a grid cell in that lidar's frame. Sensors stay separate.

`voxels_per_cell` = float32(`cell_m` / `voxel_size`), an integer in 1…64 (0.50/0.05 = 10). Floor-div toward −∞. Dims 2 stores `iz` 0 and ignores `k`. Pins at 10: `9→0`, `10→1`, `-1→-1`, `-10→-1`, `-11→-2`. Outside int16: increment `clipped` and drop the index.

`add` / `add_empty` / `note_unavailable`: evaluated non-empty, evaluated empty, BUILDING skip. Counts: `windows_closed`, `windows_with_points`, `windows_unavailable`. No second occupancy or ray cast. Fraction = `hit_windows / windows_with_points`; omit zeros. Bit 3 clear means no hit, not free space. No point-windows stores no regions. Bit 2 is unavailable, not an empty room.

`mean_voxels` = `(sum + n/2) / n`, cap 65535. Pin: 15×10 voxels in 60 windows → hit 15, mean 3.

`minute_unix = sec - sec % period_s` (skip sec < 0). ≤2 s before an open bin, in the previous period, stays there (60, 61, 59 → minute 60). A longer forward gap sets bit 1 and writes no filler. Any other backward step flushes and opens the earlier minute with bit 1.

`pose_ok` is 1 only when every point-window was tracking and its residual was ≤ `learn_max_offset_m`. A window is observed with the state it entered in, so the window that crosses BUILDING → TRACKING is not tracking. A bin with no point-windows stores 0.

## Hot path

4096 slots, no allocation after construction. Hash `splitmix64` of the uint64 of the three uint16 lanes. Insert in signed `(ix, iy, iz)` order; the first miss sets bit 3, counts the rest of the window in `regions_dropped`, and stops.

Catch errors after that sensor's detection (`catch (...)` / `except Exception`, not `noexcept`). Drop its open bin and set its loss flag. The other sensor still detects. Event, map, and status bytes match rhythm off.

`try_enqueue` uses 8×41004-byte slots (40+4096×10+4). The callback does no filesystem I/O. A full queue drops the record and sets the loss flag. The writer appends, then publishes if `rhythm_publish` is set, and drops a publish that would block.

No table or thread when `rhythm_enable` is false, or with no directory and publish false (bench `state_dir: null`). Enabled median wall, p99 wall, and p99 CPU stay within `max(1 ms, 10% of the disabled median wall)`. Callback wall stays under 50 ms if `write`/`fsync` blocks for 500 ms.

## File

`{base}/{room_id}/rhythm/rhythm-<mid360|jt128>-<YYYYMMDD>.rhy` (`base` = `rhythm_dir` or `state_dir`, `~` once). Size rotation inserts `-2`, `-3`, … before `.rhy`. Identity mismatch (`room_id`, sensor, dims, `voxels_per_cell`, float32 bits of `voxel_size` and `cell_m`, `period_s`) → `.superseded-<unix>`. Bad magic or header CRC → `.bad-<unix>`.

Little endian, zlib `crc32`. `crc32("RHY1") = 0x046E226A`.

Header: `RHY1`, u16 ver 1, u16 `header_bytes` (×4, incl. CRC), u8 sensor (1 Mid-360, 2 JT128), u8 dims, u16 `voxels_per_cell`, f32 `voxel_size`, f32 `cell_m`, u32 `period_s` (1…65535), u16 id len, UTF-8 id, pad to 4, u32 CRC.

Record: u32 `body_bytes` (incl. CRC), i64 `minute_unix`, u64 `map_epoch`, u16 closed / with_points / unavailable, u16 pad 0, u32 `clipped` / `regions_dropped`, u8 `pose_ok` / flags, u16 `n_regions`, then n strictly increasing `(i16 ix, iy, iz, u16 hits, mean)`, CRC after `body_bytes`. Envelope 40, region 10. Dims 2 stores `iz` 0.

Flags: 0 partial (`closed < period_s/window_s`), 1 discontinuity, 2 unavailable, 3 overflow, 4 lost (clear on a callback record). Store the offset before every append. A short write or a failed retry truncates to that offset before the one marker (same minute and epoch, zeros, bits 1 and 4) or any later record. The marker is written only at the restored offset; otherwise the next fragment has bit 1. If the truncate fails, close the segment, append nothing more to it, and open no replacement. The next start truncates it to the last good CRC, and the next fragment has bit 1. Do not rewrite the queue.

Before spin, scan the open append file only (≤32 MiB) and truncate to the last good CRC. A larger file is superseded unread (next fragment has bit 1). A disk error logs once; spin still starts. `fsync` follows `rhythm_fsync_s`, plus rotation and shutdown (flush, drain, one sync, join ≤5 s, no prune).

## Topic

`labcommand_msgs/RhythmMinute` on `/room_memory/rhythm` (protocol §3.9, D44), from the writer: reliable, transient local, keep-last equal to the number of configured sensors. Depth 1 kept only the newest message, so a late subscriber saw one lidar. It skips the common header: `sensor` is the field after `room_id`. Record fields minus the pad, plus `room_id`, `sensor` (u8 immediately after `room_id`: 1 Mid-360, 2 JT128, the file-header codes), `map_epoch`, `seq`, and the minute `stamp`. Every message carries `sensor`, including a loss marker. The live publisher stamps `publisher_generation` (never 0), the same field other v2 Spark→Mac messages end with. Absent, the hand codec keeps the legacy layout so the pinned goldens stay. The `.rhy` record layout stays unchanged and does not carry the generation; the two lidars share this topic and use different frames.

## Config

`MemoryConfig` ROS parameters, copied into launch `ARGUMENTS`. A bad value fails before subscribe. No clamp. The `rhythm_cell_m / voxel_size` and `rhythm_period_s / window_s` rules apply only when `rhythm_enable` is true. With it false, a detection `voxel_size` or `window_s` the grid cannot divide still starts.

| parameter | default | rule |
|---|---|---|
| `rhythm_enable` | false | off: no logger |
| `rhythm_cell_m` | 0.5 | when enabled: float32 ÷ `voxel_size` ∈ 1…64 |
| `rhythm_dims` | 2 | 2 or 3 |
| `rhythm_period_s` | 60 | 1…65535; when enabled, ÷ `window_s` ∈ 1…3600 |
| `rhythm_publish` | false | publish the topic |
| `rhythm_dir` | `""` | empty uses `state_dir` |
| `rhythm_keep_days` | 90 | 0: no age cap |
| `rhythm_max_bytes` | 536870912 | 512 MiB; 0: no byte cap |
| `rhythm_max_file_bytes` | 33554432 | 0 off; else ≥ 65536 and ≤ the byte cap |
| `rhythm_fsync_s` | 600 | 0: no periodic fsync |

Default `~/room-memory` does not record and does not publish. `rhythm_enable` turns the logger on; an empty `rhythm_dir` then uses `state_dir`. Reference save/load still follows `state_dir` only.

## Retention

Sum `rhythm-*.rhy`, `*.superseded-*`, and `*.bad-*` (both sensors) before every append and rotation. Delete oldest quarantine by mtime, then oldest closed segments by date then index. Never delete an open append file. Drop a record that does not fit and count `rhythm_budget_dropped`. Age-prune at startup and at most once per UTC day. A short write or `ENOSPC`/`EDQUOT` retries the full record once at the restored pre-append offset. If the retry fails, truncate to that offset again before the marker or the next record. If that truncate fails, abandon the segment (no replacement file) and count a writer I/O error.

## Tests

`spark/room_memory/tests/test_rhythm.py` and `tests/test_rhythm.cpp` (one CTest) share the header + 60-window hex.

- Reject a bad float32 ratio, period, and file cap above the directory cap (0 stays legal). Floor pins, dims 2/3, the 15/3 pin, omitted zeros, empty vs unavailable, and a one-sensor BUILDING skip (no second occupancy).
- The first miss sets bit 3 and stops the window, including past 4096 regions, inside the allowance. A throw mid-insert discards the bin; the other sensor still detects.
- CRC `0x046E226A`, torn tail, superseded and bad magic. A short write whose retry fails is truncated to the pre-append offset before the marker. A failed truncate leaves nothing after the tear and opens no new file; the next start keeps the prefix, and the next fragment has bit 1. Stamps 60/61/59 stay one fragment; a gap writes no filler; an in-minute restart is two unmerged fragments. Rotations stay within the byte cap. A blocking store keeps the callback under 50 ms. A full queue drops. Rhythm on and off match on event, map, and status bytes.
- One LE `RhythmMinute` golden round-trips with `sensor` 1; the BE twin decodes; `sensor` 2 is a distinct message; any other `sensor` is rejected; old goldens stay. A stamped generation round-trips and generation 0 is invalid; the legacy CDR prefix is unchanged. `colcon test` checks the publish and skips with a message if rclpy/rclcpp is missing. Spark `.rhy` bytes match for one occupied minute and one unavailable minute (skip here). `ROOM_MEMORY_RHYTHM_BUDGET=1` checks the allowance across a rotation.

## Deploy (room-memory container only)

From this checkout, replace the C++ room-memory image only:

```sh
docker build -f spark/room_memory_cpp/Dockerfile -t labcommand/room-memory-cpp:jazzy .
docker run --rm --net=host --ipc=host \
  -v ~/room-memory:/root/room-memory \
  labcommand/room-memory-cpp:jazzy
```

Leave lidar-center, lidar-dual, and ptp-master. Keep the volume and host network. `lab` → `/root/room-memory/lab/rhythm/`. No new mount, no delete of `reference-*.cdr` or `map_epoch`, no bridge change, and no swap to this checkout's Python image.

## Later

Not in v1. See `/Users/tconf/Developer/lc-tasks/plans/rhythm.full.md`.

- FreMEn, frequency/recency, surprise, HDC, new `ChangeEvent` fields.
- Free-space rays, a second occupancy, one fused frame.
- Reader, Swift/Mac decoder, `RequestSync` replay, cross-fragment sums.
- Hash-slot pins, 64×64 probes, counter saturation, the 65536-region cut.
- Reclaim under the byte cap, a quarantine-count cap, the errno matrix.
- Other containers. Any Spark run.
