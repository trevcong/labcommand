# Room memory protocol, v1 (Spark `room_memory` → LabCommand Mac)

Status: draft for PR 0.1. Codex has not signed it off, and Trevor has not answered Q1–Q4 (end of this doc). Sources: `RESEARCH_BRIEF.md` §2.2–§2.5 (2026-10-14). The `.msg` files are in `ros/labcommand_msgs/msg/`. The Swift codecs are `Sources/LabCommandCore/RoomMemoryMessages.swift` and `RoomMemoryCDR.swift`.

The Spark does all perception and keeps the memory. The Mac mirrors the reference map, the event list and the timeline, and renders them. It never infers a change itself. Four ROS 2 topics carry everything, and they reach the Mac through the existing foxglove_bridge on :8765.

## 1. Conventions

### 1.1 Wire format

- **Encoding.** Classic CDR (XCDR1), the encoding rmw_fastrtps uses on Jazzy. foxglove_bridge passes the bytes through unchanged.
- **Encapsulation header.** 4 bytes. `00 01` is little endian and `00 00` is big endian; the 2 option bytes are ignored. The live JT128 and Mid-360 frames already carry `00 01 00 00`. Decoders reject every other header, including XCDR2 (`00 07`, `00 09`, …), as *unsupported*.
- **Alignment.** Each primitive is aligned to its own size (1, 2, 4 or 8 bytes), counted from the byte after the header. A `uint64` therefore sits on an 8-byte boundary. Nested messages add no alignment of their own.
- **Strings.** `string` is a `uint32` length that includes the NUL, then the bytes, then the NUL.
- **Arrays.** `T[]` is a `uint32` element count, then the elements. `T[N]` is N elements with no count. `T[<=1]` is a count of 0 or 1, then the elements.
- **Trailing bytes.** The rmw layer may add up to 3 bytes of padding after the message, and decoders accept that. Four or more unread bytes make the message *invalid*.
- **Encoders.** The Mac's encoders always write little endian with option bytes 0. They refuse any message the decoder would reject.
- **Not yet confirmed.** The 8-byte alignment of `uint64` is the XCDR1 rule, but no Spark-produced bytes with a 64-bit field have been checked yet. That is what the Spark goldens (§9) confirm.

### 1.2 Frames, units, time

- Coordinates are in the **room frame**, in meters. The room frame is fixed per `room_id`, and its z axis points up.
- `voxel_size` is fixed per room. The default is 0.05 m.
- A voxel's index is `floor(coordinate / voxel_size)` on each axis. This is the rule `VoxelKey.index` uses in the Swift `ChangeDetection` module on the `change-detection` branch. PR 2.1 requires the Spark to match it exactly.
- `builtin_interfaces/Time` is `int32 sec` then `uint32 nanosec`, measured from the Unix epoch. `nanosec` must be below 10⁹, or the message is *invalid*.

### 1.3 Bricks

- **Size.** A brick holds 8 × 8 × 8 voxels: 0.4 m on a side at the default voxel size.
- **Coverage.** Brick `(i, j, k)` covers voxels `8i … 8i+7` on x, and likewise `8j … 8j+7` on y and `8k … 8k+7` on z. `int16` brick indices cover about ±13 km at 0.4 m per brick.
- **Bit order.** Voxel `(x, y, z)` inside a brick is bit `x + 8y + 64z` of that brick's 64-byte mask. Bit `n` is byte `n >> 3`, bit `n & 7`, least significant bit first. So voxel (0,0,0) is byte 0 bit 0, and voxel (7,7,7) is byte 63 bit 7.
- **Per-voxel evidence.**

  | `known` bit | `occupied` bit | evidence |
  |---|---|---|
  | 0 | 0 | UNKNOWN |
  | 1 | 0 | free |
  | 1 | 1 | occupied |
  | 0 | 1 | *invalid* |

- **An absent brick is all UNKNOWN.** A brick that is all free is still sent, with `known` full and `occupied` empty. Every brick that is sent has at least one known voxel.
- **Canonical order.** Bricks are strictly increasing in `(i, j, k)`, compared on `i` first, then `j`, then `k`. Duplicate or out-of-order bricks are *invalid*.

### 1.4 Common fields

Every message starts with these four fields, in the order the brief lists them:

| field | type | unit | valid | if missing or invalid |
|---|---|---|---|---|
| `room_id` | `string` | — | non-empty UTF-8, no NUL | message rejected |
| `map_epoch` | `uint64` | epoch | bumped on every reference-map change | always present; a stale message renders "—" |
| `seq` | `uint64` | count | +1 per message on that topic, starts at 0 per publisher; `0` inside `ChangeEvent.mask` | gaps follow §5 |
| `stamp` | `builtin_interfaces/Time` | s, ns | `nanosec` < 10⁹ | `{0, 0}` renders "time —" |

A message whose `room_id` differs from the room being viewed is ignored.

### 1.5 Missing values

Every field has one of three "missing" encodings:

- **float32 measurements:** NaN means *not available*. ±Inf is *invalid*. In Swift these are Optionals, so a missing value cannot be read as 0.
- **Integers that can be unmeasured:** a named sentinel. The only one in v1 is `HZ_UNAVAILABLE = 65535` in `per_sensor_hz`. `0` is a measured zero.
- **Counters, epochs, enums, ids:** they cannot be missing inside a valid message. If the message itself is missing or stale (§5), they render "—".

Decoders reject an out-of-range enum value as *invalid*, and the whole message is dropped. A dropped message is never partly applied. The Mac counts it in a diagnostics line ("n room-memory messages rejected").

## 2. Topics

Topic names are a proposal for PR 2.4 (D18):

| topic | message | rate | QoS (proposal) |
|---|---|---|---|
| `/room_memory/map_bricks` | `MapBricks` | snapshot on (re)connect or request; deltas ≤ 0.2 Hz, only when the reference changes | reliable, keep-last 10 |
| `/room_memory/events` | `ChangeEvent` | one per lifecycle transition, typically < 1/min | reliable, keep-last 100 |
| `/room_memory/status` | `MemoryStatus` | 1 Hz | reliable, keep-last 1 |
| `/room_memory/epoch` | `Epoch` | 0.2 Hz | reliable, keep-last 1 |

## 3. Messages

The `.msg` text below is the legal ROS 2 version of §2.4. §8 (D1) lists every syntax change. Field names, types and order are as §2.4 gives them. The *missing* column says how "not available" travels on the wire, and the *rendering* column says what the Mac shows then. "Stale" is the message-level rendering of §5.

### 3.1 `MapBricks`: snapshot or delta

```
uint8 KIND_SNAPSHOT=0
uint8 KIND_DELTA=1
uint8 COMPRESSION_NONE=0
uint8 COMPRESSION_ZSTD=1
string room_id
uint64 map_epoch
uint64 seq
builtin_interfaces/Time stamp
uint8 kind
uint64 base_epoch
float32 voxel_size
uint8 brick_dim
int16[] brick_ijk
uint8[] occ_bits
uint8[] known_bits
uint32[] removed_bricks_idx
uint8 compression
```

| field | type | unit | valid | missing on the wire | rendering when missing / invalid |
|---|---|---|---|---|---|
| `kind` | `uint8` | enum | `KIND_SNAPSHOT`, `KIND_DELTA` | cannot be missing | other value: message rejected, map keeps its prior state |
| `base_epoch` | `uint64` | epoch | delta: the epoch it applies to; snapshot: 0 (ignored) | cannot be missing | delta on a different epoch: resync (§5) |
| `voxel_size` | `float32` | m | finite, > 0 | NaN is *invalid* | message rejected; map "unavailable" until a valid snapshot |
| `brick_dim` | `uint8` | voxels per edge | 8 | cannot be missing | other value: *unsupported*, map "unavailable" |
| `brick_ijk` | `int16[]` | brick index (0.4 m at 5 cm) | 3 per brick, canonical order | an absent brick | its volume renders UNKNOWN (hatched/transparent), never as empty space |
| `occ_bits` | `uint8[]` | bitmask | 64 B per brick, ⊆ `known_bits` | bit clear | voxel is free if known, else UNKNOWN |
| `known_bits` | `uint8[]` | bitmask | 64 B per brick, ≥ 1 bit set per brick | bit clear | voxel UNKNOWN, hatched |
| `removed_bricks_idx` | `uint32[]` | index | delta only; strictly increasing indices into the `base_epoch` brick list in canonical order | empty | a removed brick becomes UNKNOWN, never free |
| `compression` | `uint8` | enum | `COMPRESSION_NONE` | cannot be missing | `1` (zstd): *unsupported*, map "unavailable: unsupported compression" (D9); other: *invalid* |

**Applying a message.**
- **Snapshot.** Replaces the mirror: bricks, `map_epoch`, and the `seq` baseline.
- **Delta.** Applies only when the mirror holds exactly `base_epoch` and `seq` is the next one. It removes the bricks at `removed_bricks_idx` first (indices into the `base_epoch` list), then replaces or inserts each listed brick at its coordinate. The mirror's epoch then becomes `map_epoch`. A delta that removes and re-sends the same brick is *invalid*; `RoomMemoryStore` checks that (PR 1.1).

**Wire size.**

```
4 + C + 24 + (4 + 6n, padded to 4) + (4 + 64n) + (4 + 64n) + (4 + 4r) + 1   bytes
```

Here n is the brick count, r is the number of removed indices, and C is the common-field size `align8(4 + L) + 24`. L is the `room_id` UTF-8 length + 1; C = 32 for "lab".

### 3.2 `ChangeEvent`: one lifecycle transition

```
uint8 KIND_APPEARED=0
uint8 KIND_DISAPPEARED=1
uint8 KIND_MOVED=2
uint8 STATE_CANDIDATE=0
uint8 STATE_CONFIRMED=1
uint8 STATE_ACCEPTED=2
uint8 STATE_REVERTED=3
uint8 STATE_EXPIRED=4
uint8 SENSOR_MID360=1
uint8 SENSOR_JT128=2
string room_id
uint64 map_epoch
uint64 seq
builtin_interfaces/Time stamp
uint64 event_id
uint8 kind
uint8 state
float32[3] centroid
float32[6] aabb
float32[3] from_centroid
uint32 voxel_count
float32 volume_m3
float32 evidence
float32 alpha
uint8 sensors
builtin_interfaces/Time first_seen
builtin_interfaces/Time last_seen
MapBricks[<=1] mask
```

| field | type | unit | valid | missing on the wire | rendering when missing / invalid |
|---|---|---|---|---|---|
| `event_id` | `uint64` | id | unique per room; repeated by every transition of one change | cannot be missing | — |
| `kind` | `uint8` | enum | `KIND_*`; fixed for the life of an `event_id` | cannot be missing | other value: message rejected |
| `state` | `uint8` | enum | `STATE_*`, legal transition (§4.2) | cannot be missing | other value: message rejected |
| `centroid` | `float32[3]` | m, room frame | finite | NaN is *invalid* | message rejected |
| `aabb` | `float32[6]` | m, room frame | `min_x, min_y, min_z, max_x, max_y, max_z`; finite; min ≤ max | NaN is *invalid* | message rejected |
| `from_centroid` | `float32[3]` | m, room frame | finite exactly when `kind = MOVED` | NaN × 3 when not MOVED | no arrow. A partial NaN, a MOVED without it, or a non-MOVED with it is *invalid* |
| `voxel_count` | `uint32` | voxels | any | cannot be missing | 0 is a real zero |
| `volume_m3` | `float32` | m³ | finite, ≥ 0; `voxel_count × voxel_size³` | NaN is *invalid* | message rejected |
| `evidence` | `float32` | fraction 0–1 | share of the region's voxels with free/occupied evidence | NaN = not computed | "evidence —", never 0 % |
| `alpha` | `float32` | probability 0–1 | calibrated false-alarm level from the research layer (RT1) | NaN = uncalibrated | "uncalibrated", never "α 0" |
| `sensors` | `uint8` | bitmask | bit 0 Mid-360, bit 1 JT128; other bits *invalid* | 0 = no attribution | "sensor —" |
| `first_seen` | `builtin_interfaces/Time` | s, ns | ≤ `last_seen` | `{0, 0}` | "first seen —" |
| `last_seen` | `builtin_interfaces/Time` | s, ns | ≥ `first_seen` | `{0, 0}` | "last seen —" |
| `mask` | `MapBricks[<=1]` | — | 0 or 1 map, `KIND_SNAPSHOT`. `occupied` = voxel in the region; `known` = voxel had evidence; header copies the parent's `room_id`, `map_epoch` and `stamp`, with `seq` 0 (D14) | count 0 | the region is drawn from `aabb` only, labelled "no mask" |

**Wire size.** Without a mask, `4 + C + 100` bytes: 136 B for "lab". A mask adds its nested `MapBricks`, which starts 4-aligned, so its `map_epoch` may take 4 bytes of padding.

### 3.3 `MemoryStatus`: health, 1 Hz

```
uint8 STATE_BUILDING=0
uint8 STATE_TRACKING=1
uint8 STATE_RIG_MOVED=2
uint8 STATE_DEGRADED=3
uint8 STATE_PAUSED=4
uint16 HZ_UNAVAILABLE=65535
string room_id
uint64 map_epoch
uint64 seq
builtin_interfaces/Time stamp
uint8 state
uint32 n_bricks
uint32 n_open_events
float32 coverage_known_frac
float32 pose_residual_m
float32 window_s
uint16[2] per_sensor_hz
uint32 dropped_windows
```

`map_epoch` in §2.4's MemoryStatus is the common field and is not repeated (D2).

| field | type | unit | valid | missing on the wire | rendering when missing / invalid |
|---|---|---|---|---|---|
| `state` | `uint8` | enum | `STATE_*` (§4.1) | cannot be missing | other value: message rejected; status stale after 3 s |
| `n_bricks` | `uint32` | bricks | bricks in the reference map | cannot be missing | stale: "—" |
| `n_open_events` | `uint32` | events | events in CANDIDATE or CONFIRMED | cannot be missing | stale: "—" |
| `coverage_known_frac` | `float32` | fraction 0–1 | observed share of the room volume | NaN = room volume not yet known | "coverage —", never 0 % |
| `pose_residual_m` | `float32` | m | finite, ≥ 0 | NaN = guard has not run | "guard not run", never "0 m" |
| `window_s` | `float32` | s | finite, > 0 (0.5–1.0 expected) | NaN = not reported | "window —" |
| `per_sensor_hz` | `uint16[2]` | Hz, frames received per second, rounded | `[0]` Mid-360, `[1]` JT128 (same order as `sensors`) | `65535` = not measured | "—". `0` renders "0 Hz", a measured zero |
| `dropped_windows` | `uint32` | windows since node start | any | cannot be missing | stale: "—" |

**Wire size.** `4 + C + 32` bytes: 68 B for "lab", 76 B for "lab-a".

### 3.4 `Epoch`: timeline tick, 0.2 Hz

```
string room_id
uint64 map_epoch
uint64 seq
builtin_interfaces/Time stamp
uint32 n_appeared
uint32 n_disappeared
uint32 n_moved
float32 coverage_known_frac
uint8 state
```

§2.4 gives Epoch as prose: "epoch id, stamp, counts of appeared/disappeared/moved, coverage_known_frac, state". The epoch id is `map_epoch` (D2).

| field | type | unit | valid | missing on the wire | rendering when missing / invalid |
|---|---|---|---|---|---|
| `n_appeared` | `uint32` | events | APPEARED events that reached CONFIRMED since the room was created; never decreases (D13) | cannot be missing | stale: "—" |
| `n_disappeared` | `uint32` | events | same, DISAPPEARED | cannot be missing | stale: "—" |
| `n_moved` | `uint32` | events | same, MOVED | cannot be missing | stale: "—" |
| `coverage_known_frac` | `float32` | fraction 0–1 | as in MemoryStatus | NaN | "coverage —" |
| `state` | `uint8` | enum | MemoryStatus `STATE_*` at this tick | cannot be missing | other value: message rejected |

**Wire size.** `4 + C + 17` bytes: 53 B for "lab".

## 4. State machines

The Spark owns both machines. The Mac renders them, and it flags a transition that is not in these tables as a protocol violation. It keeps the last legal state and logs the violation; it does not render the illegal state.

### 4.1 Memory state (`MemoryStatus.state`, `Epoch.state`)

| from | to | trigger |
|---|---|---|
| node start | BUILDING | no saved reference for `room_id` |
| node start | TRACKING | saved reference loaded and the rig-pose guard passes |
| node start | RIG_MOVED | saved reference loaded and the guard fails |
| BUILDING | TRACKING | reference established (coverage threshold set in PR 2.2); `map_epoch` bumps |
| TRACKING | RIG_MOVED | guard: ICP residual or pose delta past threshold (§2.2 step 2) |
| RIG_MOVED | TRACKING | guard passes again (rig back in place) |
| RIG_MOVED | BUILDING | operator re-references the room; the new reference bumps `map_epoch` |
| BUILDING, TRACKING | DEGRADED | a sensor delivers no frames (`per_sensor_hz` 0) or windows are being dropped |
| DEGRADED | the state before DEGRADED | input restored |
| any | PAUSED | operator pause |
| PAUSED | BUILDING / TRACKING / RIG_MOVED | operator resume, re-entered through the guard check |

| state | change events | Mac rendering |
|---|---|---|
| BUILDING | none | "Building reference" with coverage; no change overlays |
| TRACKING | all transitions | normal |
| RIG_MOVED | no new CANDIDATEs; open CANDIDATEs go EXPIRED; CONFIRMED events keep their state | red banner "Rig moved: changes suspended", residual shown; no change overlays from the current window |
| DEGRADED | continue, attributed by `sensors` to the sensors still delivering (D17) | amber banner naming the silent sensor |
| PAUSED | none; open events frozen | grey banner "Paused" |

### 4.2 Event lifecycle (`ChangeEvent.state`)

| from | to | trigger | Mac rendering of the new state |
|---|---|---|---|
| — | CANDIDATE | disagreement between the reference and recent layers where both have evidence | dashed outline, "candidate"; not counted as a change |
| CANDIDATE | CONFIRMED | persists ≥ T_commit (start at 5 s; tuned in PR 2.3) | solid overlay; counted; arrow for MOVED |
| CANDIDATE | EXPIRED | gone before T_commit (walk-through), superseded by a MOVED pairing, or memory left TRACKING for RIG_MOVED | removed from the overlay; history only |
| CONFIRMED | ACCEPTED | operator accept; the reference absorbs it and `map_epoch` bumps | overlay removed; the map now shows the new state; history entry |
| CONFIRMED | REVERTED | the region returned to the reference state, or operator revert | removed; history only |

- ACCEPTED, REVERTED and EXPIRED are terminal. Only CONFIRMED renders as a change. CANDIDATE renders as a candidate, and REVERTED or EXPIRED never render as a change.
- An event's `kind` never changes. Moved pairing happens before confirmation: the paired CANDIDATEs go EXPIRED, and a new MOVED event starts at CANDIDATE (D17).
- A Mac that connects mid-session accepts any state as the first one it sees for an `event_id`.

## 5. Resync rule

From §2.4: *if the Mac sees a `seq` gap, or holds `map_epoch ≠ base_epoch` for a delta, it discards local state, renders "stale", and requests a `KIND_SNAPSHOT`. It never patches a map it cannot prove is current.* Applied per topic:

**`map_bricks`.** Any one of these triggers a resync: discard the mirror, render the map STALE (greyed, "resyncing", no change overlays drawn as current), and request a snapshot.
- `seq` ≠ previous + 1, including a reset to a lower value when the node restarts.
- A delta whose `base_epoch` ≠ the mirror's epoch.
- A delta as the first message after (re)connect.
- `MemoryStatus.map_epoch` ≠ the mirror's epoch in 2 consecutive status messages. This catches a lost delta that no later delta exposes (D20).

A snapshot is always accepted. It resets the `seq` baseline and the epoch.

**`events`.** A `seq` gap marks the event list "incomplete since \<stamp of the last good message\>". Missed transitions are not invented, and open events show "may be outdated". §2.4 defines no event-replay request; see D19.

**`status`, `epoch`.** These are full-state messages, so a gap is only counted. Each is stale when its newest message is older than 3 periods: 3 s for `status` (inside PR 3.1's "stale within 5 s") and 15 s for `epoch`.

**Snapshot request.** No §2.4 message carries a request. The mechanism is PR 2.4 / 3.1's (D19). Until it exists, the map stays STALE until the Spark sends its next snapshot.

## 6. Honest-state rendering rules

These extend the app's existing contract (missing ≠ zero, a running container is not a running stream) to the map.

1. **UNKNOWN ≠ free.** A voxel with no `known` bit, or inside an absent or removed brick, renders hatched/transparent ("not observed"). It is never drawn as empty space, and never counted as free.
2. **Absence of returns ≠ absence of object.** DISAPPEARED comes only from the Spark, on free-space evidence: rays now pass through a formerly occupied voxel. The Mac never derives a change from missing points.
3. **`alpha = NaN` ≠ 0.** NaN renders "uncalibrated". It is never "α = 0", which would claim a perfect false-alarm level, and never hidden as if it were zero. This follows the brief's recommendation in §5 Q11.
4. **Other NaN floats are "—", not 0.** `evidence`, `coverage_known_frac`, `pose_residual_m` and `window_s` render "—" or their named phrase ("guard not run"), never 0 % or 0 m.
5. **Rates.** `0` Hz is a measured zero (the sensor is silent). `65535` is "—" (not measured).
6. **Stale is not current.** A stale topic shows "STALE · \<age\>". Its last values are never presented as live.
7. **Only CONFIRMED is a change.** CANDIDATE is shown as a candidate. REVERTED and EXPIRED never render as changes. ACCEPTED is history.
8. **RIG_MOVED suppresses change overlays** from the current window. The banner gives the residual.
9. **No partial application.** A message that fails to decode, or fails the delta preconditions, changes nothing on screen except the diagnostics count and the STALE state.
10. **Minimum detectable size is stated.** The view states the smallest object size the room's `voxel_size` supports, with the figure from PR 3.2's scoring (brief R5). It does not imply that smaller changes would be seen.

## 7. Byte budget

**Budget (§2.5).** ≤ 5 KB/s mean and ≤ 2 MB per (re)connect in memory mode. A snapshot targets ≤ 500 KB, with a hard cap of 2 MB.

| stream | rate | message size (exact, "lab") | steady state |
|---|---|---|---|
| MemoryStatus | 1 Hz | 68 B | 0.07 KB/s |
| Epoch | 0.2 Hz | 53 B | 0.01 KB/s |
| ChangeEvent, no mask | per transition, < 1/min | 136 B | < 0.01 KB/s |
| ChangeEvent with a 25 cm box mask | per transition | 481 B (measured) | < 0.01 KB/s |
| MapBricks delta, 25 cm box added | ≤ 0.2 Hz, on reference change | 1,685 B (measured) | < 1 KB/s |
| MapBricks snapshot | on (re)connect | 457,285–546,933 B raw (measured, see below) | — |

**Measured** by the size test in `RoomMemoryCDRTests.swift`. The test is a full-sphere synthetic accumulation of the furnished 8 × 8 × 3 m room: 720 × 360 rays, ±1 cm range noise, seed 1, 5 cm voxels. Compression uses NSData `.zlib`, which Apple implements as raw DEFLATE (RFC 1951) at level 5 (`NSData.h`, `compression.h`), and `.lzfse`.

| message | bricks | raw | zlib | lzfse |
|---|---|---|---|---|
| snapshot, every observed brick (v1 semantics), floor at z = 0 on a brick boundary | 4,081 | 546,933 B | 58,989 B | 54,023 B |
| the same room shifted (0.2, 0.2, 0.1) m off the brick grid | 3,412 | 457,285 B | 52,827 B | 48,790 B |
| snapshot, surface bricks only (the brief's size model) | 2,211 | 296,353 B | 53,347 B | 51,110 B |
| delta: 25 cm box added | 12 | 1,685 B | 335 B | 385 B |
| APPEARED event with mask | 2 (mask) | 481 B | 144 B | 194 B |

**Reading the numbers.**
- The brief's model (1–3k surface bricks, 130–400 KB raw) holds for surface bricks: 2,211 bricks, 296 KB.
- With absent = UNKNOWN (D5), free-only bricks must be sent too. Raw size then depends on where the brick grid sits relative to the room (D6):
  - With the floor and two walls exactly on brick boundaries, ±1 cm noise splits their hits across two brick layers. That makes 4,081 bricks and **547 KB raw, over the 500 KB target**.
  - With the room off the grid, it is 3,412 bricks and **457 KB raw, under it**.
  - Both are inside the 2 MB cap.
- Compressed, the snapshot is **59 KB (zlib) or 54 KB (lzfse)**, about 9–10× under the target. Mostly-free bricks compress to almost nothing.
- v1 has no working compression (D9), so the 500 KB target is met only once compression is resolved.
- A (re)connect costs one snapshot plus the latest status and epoch, about 0.55 MB raw. That is inside the 2 MB per-(re)connect budget.

## 8. Decisions for review

Each item is a place where §2.4 was not legal ROS 2, was underspecified, or conflicted with the honest-state rules. Each states what this doc does and why. All need Codex review; D5, D9 and D19 also need Trevor.

- **D1. ROS 2 syntax.** §2.4's text was made legal ROS 2 with names, types and order unchanged:
  - Constants go one per line (`KIND_SNAPSHOT=0, KIND_DELTA=1` became two lines).
  - Fields joined with `;` go one per line.
  - ROS 1 `time` became `builtin_interfaces/Time` (`int32 sec`, `uint32 nanosec`), for `stamp` as well.
  - `per_sensor_hz[2]` became `uint16[2] per_sensor_hz`.
  - "`MapBricks mask # optional`" became `MapBricks[<=1] mask`, since ROS 2 has no optional fields.
  - Named constants were added for values §2.4 described in comments: `COMPRESSION_*`, `SENSOR_*`, `HZ_UNAVAILABLE`.
- **D2. Common fields.**
  - They are `string room_id`, `uint64 map_epoch`, `uint64 seq` and `builtin_interfaces/Time stamp`, first in every message and in the brief's order.
  - They are written inline rather than as a shared header message; the wire bytes are identical either way.
  - MemoryStatus's own `map_epoch` is the common one, and Epoch's "epoch id" is `map_epoch`.
  - `room_id` is a non-empty UTF-8 string. Its format, such as a slug, is not fixed.
- **D3. Enum values** follow §2.4's listing order from 0. Unknown values reject the message instead of mapping to a default.
- **D4. Classic CDR (XCDR1) with 8-byte `uint64` alignment.** This is an assumption, to be confirmed by the Spark goldens (§9). The ≤ 3 trailing bytes rule comes from rmw padding.
- **D5. Absent brick = UNKNOWN.** §2.4's `removed_bricks_idx` comment says bricks "that became all-unknown/all-free".
  - Removing an all-free brick would turn free evidence into UNKNOWN on the Mac, or make absence ambiguous. Either way it breaks UNKNOWN ≠ free.
  - So removal means "became all-unknown" only, and all-free bricks are sent.
  - Cost: measured 4,081 vs 2,211 bricks (§7).
- **D6. Voxel and bit layout.** Voxel index `floor(coordinate / voxel_size)`; brick `(i, j, k)` covers voxels `8i … 8i+7` (likewise for j, k); bit `x + 8y + 64z`, LSB first. §2.4 did not specify these.
  - Where the room frame's origin sits relative to the floor and walls changes snapshot bytes by about 20 % (§7).
  - A floor at exactly z = 0 straddles a brick boundary. Choosing the room-frame origin (PR 2.2) is a size decision as well as a convenience.
- **D7. Canonical order.**
  - Bricks are strictly increasing in `(i, j, k)`, so that `removed_bricks_idx` (`uint32`, as specified) has a meaning: an index into the `base_epoch` brick list in that order.
  - A delta applies removals first, then upserts.
  - Brick coordinates would be sturdier than indices, at 6 B instead of 4 B each. That is left for review rather than changed.
- **D8. Brick invariants.** `occupied ⊆ known`, and every sent brick has ≥ 1 known voxel. A snapshot carries no removals.
- **D9. `compression`.** §2.4 has "`uint8 compression # 0 none, 1 zstd (whole payload)`" as the *last* field.
  - A byte after the arrays cannot describe compression of the arrays before it. With typed CDR arrays, a zstd payload has nowhere to live.
  - Foundation has no zstd.
  - v1 codecs therefore accept only 0. They reject 1 as *unsupported* and anything else as *invalid*.
  - The measured 9–10× ratio (§7) says compression is worth having. Options for a v2 decision:
    - (a) Transport-level compression: WebSocket permessage-deflate, if the deployed foxglove_bridge supports it (not verified).
    - (b) A layout with `compression` before a single `uint8[]` blob that holds the brick arrays. Use raw DEFLATE: Foundation `.zlib` on the Mac, `zlib.compressobj(wbits=-15)` on the Spark.
- **D10. Floats.** NaN = missing for `evidence`, `alpha`, `coverage_known_frac`, `pose_residual_m` and `window_s`; ±Inf is invalid. Ranges are as in §3. `from_centroid` is NaN × 3 exactly when the event is not MOVED.
- **D11. `per_sensor_hz`.** It means sensor frames received per second, rounded to whole Hz. Index 0 is Mid-360 and index 1 is JT128, matching the `sensors` bit order. `65535` = not measured.
- **D12. `sensors`.** Bit 0 is Mid-360 and bit 1 is JT128. Other bits are invalid. 0 means no attribution.
- **D13. Epoch counts are cumulative** since the room was created, so a lost tick loses nothing. The Mac diffs consecutive ticks for the timeline.
- **D14. `mask` semantics.**
  - It is a snapshot-kind `MapBricks` with `base_epoch` 0.
  - `occupied` means the voxel is in the region, and `known` means the voxel had evidence. Region voxels must be known, since a change needs evidence in both layers.
  - Its header copies the parent's `room_id`, `map_epoch` and `stamp`, with `seq` 0.
- **D15. Times.** `{0, 0}` renders "—". `nanosec` ≥ 10⁹ is invalid. `last_seen` must not be before `first_seen`.
- **D16. Validation is symmetric.** The Swift encoder refuses exactly what the decoder rejects, so the Mac can never emit a message it would drop.
- **D17. Lifecycle details §2.2 leaves open.**
  - `kind` is fixed per `event_id`.
  - Moved pairing supersedes candidates (they go EXPIRED).
  - RIG_MOVED expires open candidates.
  - DEGRADED keeps tracking with the sensors left.
  - ACCEPTED bumps `map_epoch`.
  - Accept semantics also depend on the brief's §5 Q6.
- **D18. Topic names and QoS** (§2) are proposals for PR 2.4.
- **D19. Snapshot request and event replay.** §2.4 has no request message, and foxglove_bridge's client-publish or service paths have not been checked against the deployed bridge. Until PR 2.4 / 3.1 define one, a stale map waits for the next snapshot, and a gap in the event list cannot be filled.
- **D20. Staleness.**
  - `status` is stale after 3 s and `epoch` after 15 s.
  - A missed trailing delta is caught by `MemoryStatus.map_epoch` disagreeing with the mirror twice in a row.
- **D21. `package.xml` license** says `Proprietary` because the repo has no LICENSE file. Trevor should confirm.

## 9. Verification status

- **Hand goldens.** `Tests/Fixtures/room/golden/hand/*.hex` were written byte by byte from these rules, with every span annotated. The tests check three things:
  - The Swift decoder reads all 7 (5 little endian, 2 big endian) to the expected values.
  - The encoder reproduces the 5 little-endian ones exactly.
  - Every truncation throws.
- **No independent cross-check yet.** No offline ROS 2 CDR library (`rosbags`, `mcap_ros2`) is installed on this Mac, and none was installed for this PR. So the hand goldens have only been checked against the Swift codec, which follows the same rules.
- **Spark goldens: not produced.** `scripts/spark/make_room_memory_goldens.py` serializes the same values with `rclpy` after a `colcon build` of `labcommand_msgs` in the Jazzy container. Running it is a Spark task. The follow-up test then requires each Spark file to decode to the same value as its hand golden, which confirms D4.
- **`colcon build` of `labcommand_msgs`: not run.** There is no ROS 2 on the Mac.

## 10. Open questions for Trevor (brief §5, Q1–Q4)

These are copied from the brief and left unanswered.

1. **Is the rig static for v1?** If it will move between sessions, relocalization (and place recognition) moves from optional to required.
2. **Room and targets:** room size, smallest object that must be detected (5 cm? 10 cm?), acceptable latency (seconds vs minutes), acceptable false events per hour. These set voxel size, T_commit and the PR 3.2 bar.
3. **HDC's role.** Your own record (exp119/122/123, exp113, D-33) says a superposed memory is not a cheaper map, and HDC is parked. Should the app be HDC-free with HDC kept only as an optional RT3 comparison, or do you want to reopen HDC? This brief assumes the former.
4. **Fixtures:** may the 12 Oct box takes (`*ROBOT_box_*`, `*speedybee_box_*` on the Spark) be copied to the Mac as golden fixtures? This brief did not access the Spark.
