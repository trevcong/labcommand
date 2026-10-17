# Room-memory fixtures (Phase 0, PR 0.2 and 0.3)

Tests that read these files run from the repo root: `swift run LabCommandCoreTestsRunner`. A missing file fails the run; nothing skips.

## `snapshots/a`, `snapshots/b`

These are real single frames, `jt128.bin` and `mid360.bin`, copied byte for byte from `~/Developer/lc-tasks/snapshots/`. Each is a CDR little-endian `sensor_msgs/PointCloud2` as foxglove_bridge delivers it. The room was static and the two takes are about 10 s apart. `a/jt128.bin` and `b/jt128.bin` are the same length but not identical.

| file | bytes | frame_id | finite points decoded |
|---|---|---|---|
| `a/jt128.bin` | 2,995,397 | `hesai_lidar` | 115,200 |
| `a/mid360.bin` | 521,881 | `livox_frame` | 20,064 |
| `b/jt128.bin` | 2,995,397 | `hesai_lidar` | 115,200 |
| `b/mid360.bin` | 519,385 | `livox_frame` | 19,968 |

## `synthetic/`

A ray-cast 8 × 8 × 3 m furnished room. The scanner sits at (4.0, 4.0, 1.2) m and casts a 288 × 81 ray grid (1.25° steps, ±50° elevation) with ±1 cm uniform range noise. It is a repetitive pattern, like a spinning lidar. Points are in the sensor frame (`frame_id` `synthetic_lidar`), in the same PointCloud2 CDR layout that `change-detect` reads (x, y, z, intensity float32, 16 B per point). `MANIFEST.json` lists the scenes, seeds and sensor origins, and the pairs with their ground truth, with boxes given in both the room and sensor frames.

| pair | before → after | ground truth |
|---|---|---|
| `static` | `base_s1` → `base_s2` | no change (different noise seed) |
| `box_added` | `base_s1` → `box_a` | 1 APPEARED: 25 cm box 1.8 m from the sensor |
| `box_removed` | `box_a` → `base_s2` | 1 DISAPPEARED |
| `box_moved` | `box_a` → `box_b` | 1 MOVED, 1.0 m in +y |
| `person_walkthrough` | `base_s1` → `person_0` … `person_4` | no semi-static change; a 0.45 × 0.3 × 1.75 m person crossing at 0.3 m per 0.25 s frame |
| `rig_shifted` | `base_s1` → `rig_shifted` | RIG_MOVED: sensor +5 cm in x (same noise seed as `base_s1`), no events |

Regenerate:

```bash
swift run LabCommandCoreTestsRunner generate-room-fixtures
```

The scenes are deterministic: SplitMix64 noise, a fixed ray order, and Darwin libm. The test suite regenerates every file in memory and requires byte equality with the stored file.

**Deferred.** Expected `change-detect` outputs are not stored yet. The CLI exists only on the `change-detection` branch (06b5c83). The follow-up runs `swift run -c release change-detect -- <before> <after>` for each pair in `MANIFEST.json` and stores the outputs under `expected/`. The static pair must give 0 regions at the default thresholds, or the PR must justify tuned defaults (brief §3 PR 0.3). Trevor's 12 Oct box takes wait on brief §5 Q4.

## `golden/hand/`

These are hand-derived CDR bytes for the `labcommand_msgs` types except `OperatorAction` (no codec, protocol §3.6), in the legacy and the protocol v2 layouts, and for the `sensor_msgs/Imu` the rig-pose guard reads. They were written from the `.msg` files and the rules in `docs/ROOM-MEMORY-PROTOCOL.md` §1 and §3, not produced by the encoder, and every span is annotated with its stream position. There are 30: 23 little-endian files and 7 big-endian ones; the big-endian files are decode-only. `spark/room_memory/tests/fixtures/hand/` holds the same files (`scripts/check-shared-goldens.sh`), and `PROVENANCE.md` there has their hashes. The `.hex` format is whitespace-separated hex bytes, with `#` starting a comment.

DEFLATE and LZFSE output is not canonical, so `map_bricks_snapshot_zlib_le.hex` and `map_bricks_delta_lzfse_le.hex` carry hand-built uncompressed blocks: a DEFLATE stored block and an LZFSE `bvx-` block. The encoder must match them up to the payload length, and its own compressed payload must inflate to their verbatim bytes. Every other little-endian golden is reproduced byte for byte.

**Deferred.** `golden/spark/` will hold the same values serialized by `rclpy` on the Spark (`scripts/spark/make_room_memory_goldens.py`, not run yet).
