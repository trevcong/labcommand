# Room memory protocol, v1 (Spark `room_memory` → LabCommand Mac)

Status: draft for PR 0.1. Codex has not signed it off. Trevor decided D5, D9 and D19 on 2026-10-14 (§8); he has not answered Q1–Q4 (end of this doc). Sources: `RESEARCH_BRIEF.md` §2.2–§2.5 (2026-10-14). The `.msg` files are in `ros/labcommand_msgs/msg/`. The Swift codecs are `Sources/LabCommandCore/RoomMemoryMessages.swift` and `RoomMemoryCDR.swift`.

The Spark does all perception and keeps the memory. The Mac mirrors the reference map, the event list and the timeline, and renders them. It never infers a change itself. Four Spark → Mac topics carry the state, two sync topics carry answers to the Mac's resync requests, and one Mac → Spark topic carries those requests. All of them go through the existing foxglove_bridge on :8765.

## 1. Conventions

### 1.1 Wire format

- **Encoding.** Classic CDR (XCDR1), the encoding rmw_fastrtps uses on Jazzy. foxglove_bridge passes the bytes through unchanged.
- **Encapsulation header.** 4 bytes. `00 01` is little endian and `00 00` is big endian; the 2 option bytes are ignored. The live JT128 and Mid-360 frames already carry `00 01 00 00`. Decoders reject every other header, including XCDR2 (`00 07`, `00 09`, …), as *unsupported*.
- **Alignment.** Each primitive is aligned to its own size (1, 2, 4 or 8 bytes), counted from the byte after the header. A `uint64` therefore sits on an 8-byte boundary. Nested messages add no alignment of their own.
- **Strings.** `string` is a `uint32` length that includes the NUL, then the bytes, then the NUL.
- **Arrays.** `T[]` is a `uint32` element count, then the elements. `T[N]` is N elements with no count. `T[<=1]` is a count of 0 or 1, then the elements.
- **Trailing bytes.** The rmw layer may add up to 3 bytes of padding after the message, and decoders accept that. Four or more unread bytes make the message *invalid*.
- **Encoders.** The Mac's encoders always write little endian with option bytes 0. They refuse any message the decoder would reject.
- **One exception to the byte order.** `MapBricks.payload` is a `uint8[]` whose contents are packed little endian in either CDR byte order (§3.1).
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

- **An absent brick is all UNKNOWN.** A brick that is all free is still sent, with `known` full and `occupied` empty (D5, decided). Every brick that is sent has at least one known voxel.
- **Canonical order.** Bricks are strictly increasing in `(i, j, k)`, compared on `i` first, then `j`, then `k`. Duplicate or out-of-order bricks are *invalid*.

### 1.4 Common fields

Every message starts with these four fields, in the order the brief lists them:

| field | type | unit | valid | if missing or invalid |
|---|---|---|---|---|
| `room_id` | `string` | — | non-empty UTF-8, no NUL | message rejected |
| `map_epoch` | `uint64` | epoch | bumped on every reference-map change; in `RequestSync`, repeats `since_epoch` | always present; a stale message renders "—" |
| `seq` | `uint64` | count | +1 per message on that topic, starts at 0 per publisher; `0` inside `ChangeEvent.mask`; in `RequestSync`, +1 per request from that client | gaps follow §5 |
| `stamp` | `builtin_interfaces/Time` | s, ns | `nanosec` < 10⁹ | `{0, 0}` renders "time —" |

A message whose `room_id` differs from the room being viewed is ignored.

### 1.5 Missing values

Every field has one of three "missing" encodings:

- **float32 measurements:** NaN means *not available*. ±Inf is *invalid*. In Swift these are Optionals, so a missing value cannot be read as 0.
- **Integers that can be unmeasured or absent:** a named sentinel. v1 has two: `HZ_UNAVAILABLE = 65535` in `per_sensor_hz`, where `0` is a measured zero, and `SINCE_NONE = 2⁶⁴ − 1` in `RequestSync`, where `0` is a real epoch or seq.
- **Counters, epochs, enums, ids:** they cannot be missing inside a valid message. If the message itself is missing or stale (§5), they render "—".

Decoders reject an out-of-range enum value as *invalid*, and the whole message is dropped. A dropped message is never partly applied. The Mac counts it in a diagnostics line ("n room-memory messages rejected").

## 2. Topics

Topic names are a proposal for PR 2.4 (D18):

| topic | direction | message | rate | QoS (proposal) |
|---|---|---|---|---|
| `/room_memory/map_bricks` | Spark → Mac | `MapBricks` | snapshot when the node starts and on (re)connect; deltas ≤ 0.2 Hz, only when the reference changes | reliable, keep-last 10 |
| `/room_memory/events` | Spark → Mac | `ChangeEvent` | one per lifecycle transition, typically < 1/min | reliable, keep-last 100 |
| `/room_memory/status` | Spark → Mac | `MemoryStatus` | 1 Hz | reliable, keep-last 1 |
| `/room_memory/epoch` | Spark → Mac | `Epoch` | 0.2 Hz | reliable, keep-last 1 |
| `/room_memory/request_sync` | Mac → Spark | `RequestSync` | on (re)connect and on a gap, ≤ 1 per topic per 5 s | reliable, keep-last 10 |
| `/room_memory/sync/map_bricks` | Spark → Mac | `MapBricks` | answers to `TOPIC_MAP_BRICKS` requests | reliable, keep-last 10 |
| `/room_memory/sync/events` | Spark → Mac | `ChangeEvent` | answers to `TOPIC_EVENTS` requests | reliable, keep-last 200 |

The Mac publishes `RequestSync` through foxglove_bridge's client-publish path. That path has not been checked against the deployed bridge (D19).

## 3. Messages

The `.msg` text below is the legal ROS 2 version of §2.4. §8 (D1) lists every syntax change. Field names, types and order are as §2.4 gives them, except for `MapBricks`, whose brick arrays moved into a compressed payload (D9). §3.5 adds `RequestSync` (D19). The *missing* column says how "not available" travels on the wire, and the *rendering* column says what the Mac shows then. "Stale" is the message-level rendering of §5.

### 3.1 `MapBricks`: snapshot or delta

```
uint8 KIND_SNAPSHOT=0
uint8 KIND_DELTA=1
uint8 COMPRESSION_NONE=0
uint8 COMPRESSION_ZLIB=1
uint8 COMPRESSION_LZFSE=2
string room_id
uint64 map_epoch
uint64 seq
builtin_interfaces/Time stamp
uint8 compression
uint8 kind
uint64 base_epoch
float32 voxel_size
uint8 brick_dim
uint32 n_bricks
uint32 n_removed
uint8[] payload
```

| field | type | unit | valid | missing on the wire | rendering when missing / invalid |
|---|---|---|---|---|---|
| `compression` | `uint8` | enum | `COMPRESSION_*`; `ZLIB` by default | cannot be missing | other value: *invalid*, message rejected |
| `kind` | `uint8` | enum | `KIND_SNAPSHOT`, `KIND_DELTA` | cannot be missing | other value: message rejected, map keeps its prior state |
| `base_epoch` | `uint64` | epoch | delta: the epoch it applies to, < `map_epoch`; snapshot: 0 (ignored) | cannot be missing | delta with `map_epoch` ≤ `base_epoch`: *invalid*; delta on a different epoch: resync (§5) |
| `voxel_size` | `float32` | m | finite, > 0 | NaN is *invalid* | message rejected; map "unavailable" until a valid snapshot |
| `brick_dim` | `uint8` | voxels per edge | 8 | cannot be missing | other value: *unsupported*, map "unavailable" |
| `n_bricks` | `uint32` | bricks | bricks in `payload` | cannot be missing | a payload of another size: message rejected |
| `n_removed` | `uint32` | indices | removed indices in `payload`; 0 in a snapshot | cannot be missing | as `n_bricks` |
| `payload` | `uint8[]` | bytes | `compression` applied to the packed arrays below | — | does not decompress to exactly their size: message rejected |

The payload, before compression, is exactly `134 × n_bricks + 4 × n_removed` bytes. Its arrays are packed little endian in either CDR byte order, back to back with no padding:

| part | type | unit | valid | missing on the wire | rendering when missing / invalid |
|---|---|---|---|---|---|
| `brick_ijk` | `int16[3 × n_bricks]` | brick index (0.4 m at 5 cm) | canonical order | an absent brick | its volume renders UNKNOWN (hatched/transparent), never as empty space |
| `occ_bits` | `uint8[64 × n_bricks]` | bitmask | 64 B per brick, ⊆ `known_bits` | bit clear | voxel is free if known, else UNKNOWN |
| `known_bits` | `uint8[64 × n_bricks]` | bitmask | 64 B per brick, ≥ 1 bit set per brick | bit clear | voxel UNKNOWN, hatched |
| `removed_bricks_idx` | `uint32[n_removed]` | index | delta only; strictly increasing indices into the `base_epoch` brick list in canonical order | empty | a removed brick becomes UNKNOWN, never free |

**Compression (D9).**
- `COMPRESSION_ZLIB` (1), the default for every `MapBricks`, including deltas and event masks. The payload is a raw DEFLATE stream (RFC 1951): no zlib header and no Adler-32 trailer.
  - The Mac writes it with Foundation `NSData.compressed(using: .zlib)`, which is Apple's Compression framework at level 5.
  - Python reads and writes it like this:

    ```python
    raw = zlib.decompress(payload, -15)
    c = zlib.compressobj(6, zlib.DEFLATED, -15)
    payload = c.compress(raw) + c.flush()
    ```

  - Plain `zlib.compress(raw)` writes the zlib-wrapped format (`78 9c` … Adler-32), which the Mac rejects as *invalid*.
  - Plain `zlib.decompress(payload)` fails on the Mac's stream with "incorrect header check".
  - Both directions were checked on this Mac (Python 3.9 zlib and Apple, on a 547 KB payload; §9).
- `COMPRESSION_LZFSE` (2). The payload is an Apple LZFSE stream (Foundation `.lzfse`). Decoders accept it anywhere. Python's standard library has no LZFSE, so the Spark sends zlib.
- `COMPRESSION_NONE` (0). The packed arrays as they are. Use it for goldens and debugging only: a full snapshot is over the 500 KB target uncompressed (§7).
- Value 1 meant zstd in the earlier draft of this doc. zstd is dropped, because Foundation has none, and 1 now means zlib. Values ≥ 3 are *invalid*.
- **Empty payload.** With `n_bricks = n_removed = 0` the payload is empty under `NONE`. zlib and LZFSE still send a complete stream; for raw DEFLATE that is `03 00`. An empty compressed payload is *invalid*.

**Decoder checks.**
- Size limit:
  - `134 × n_bricks + 4 × n_removed` above 16 MiB (D22) is *invalid*, rejected before anything is inflated.
  - A compressed payload must be one complete stream that inflates to exactly that size. The decoder never inflates more than one byte past it.
  - An uncompressed payload must have exactly that length.
- What the Mac's inflater does not check. Apple's inflater checks neither the NLEN of a DEFLATE stored block nor for bytes after the end of the stream; Python's zlib checks both. Neither gap can make the Mac accept a payload of the wrong size, but the Spark must still send well-formed streams.
- Why there are no compressed byte goldens. DEFLATE and LZFSE output is not canonical: Apple's level 5 and Python's level 6 emit different bytes for the same input. So compressed messages are compared by decoded value, never by bytes (§9).

**Applying a message.**
- **Snapshot.** Replaces the mirror: bricks, `map_epoch`, and the `seq` baseline.
- **Delta.** Applies only when the mirror holds exactly `base_epoch` and `seq` is the next one. It removes the bricks at `removed_bricks_idx` first (indices into the `base_epoch` list), then replaces or inserts each listed brick at its coordinate. The mirror's epoch then becomes `map_epoch`. A delta that removes and re-sends the same brick is *invalid*; `RoomMemoryStore` checks that (PR 1.1).

**Wire size.**

```
4 + C + 36 + P   bytes
```

- P is the payload length: `134n + 4r` uncompressed, with n bricks and r removed indices.
- C is the common-field size `align8(4 + L) + 24`, where L is the `room_id` UTF-8 length + 1. C = 32 for "lab", so a message for "lab" is 72 + P bytes.

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
| `mask` | `MapBricks[<=1]` | — | 0 or 1 map, `KIND_SNAPSHOT`, any `compression`. `occupied` = voxel in the region; `known` = voxel had evidence; header copies the parent's `room_id`, `map_epoch` and `stamp`, with `seq` 0 (D14) | count 0 | the region is drawn from `aabb` only, labelled "no mask" |

**Wire size.** Without a mask, `4 + C + 100` bytes: 136 B for "lab". A mask adds its nested `MapBricks`, which starts 4-aligned, so its `map_epoch` may take 4 bytes of padding. The 25 cm box event in §7 is 253 B with its 2-brick mask zlib-compressed.

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

### 3.5 `RequestSync`: resync request, Mac → Spark

```
uint8 TOPIC_MAP_BRICKS=0
uint8 TOPIC_EVENTS=1
uint64 SINCE_NONE=18446744073709551615
string room_id
uint64 map_epoch
uint64 seq
builtin_interfaces/Time stamp
uint64 since_epoch
uint64 since_seq
uint8 topic
```

Trevor's D19 gives `RequestSync{room_id, since_epoch, since_seq, stamp}`. The common fields of §1.4 add `map_epoch` and `seq`, and `topic` says which topic's `seq` `since_seq` counts (D19).

| field | type | unit | valid | missing on the wire | rendering when missing / invalid |
|---|---|---|---|---|---|
| `map_epoch` | `uint64` | epoch | equal to `since_epoch` | cannot be missing | otherwise *invalid*; the Spark drops the request |
| `seq` | `uint64` | count | this client's request counter, +1 per request, from 0 | cannot be missing | — |
| `since_epoch` | `uint64` | epoch | `map_epoch` of the last message the Mac applied on `topic`; for `TOPIC_MAP_BRICKS` the epoch the mirror holds | `SINCE_NONE` = the Mac holds nothing for `topic` | `SINCE_NONE` in only one of `since_epoch`, `since_seq`: *invalid* |
| `since_seq` | `uint64` | count | `seq` of that message | `SINCE_NONE`, together with `since_epoch` | as above |
| `topic` | `uint8` | enum | `TOPIC_*` | cannot be missing | other value: *invalid* |

`SINCE_NONE` is 2⁶⁴ − 1, not 0, because epoch 0 and seq 0 are real values. In Swift `since` is an Optional `Position`. The Spark answers by the rules in §5.

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
- A Mac that connects mid-session accepts any state as the first one it sees for an `event_id`. The same holds for the first message about an `event_id` after a gap in `events` (§5): the transitions in between may be lost.

## 5. Resync rule

From §2.4: *if the Mac sees a `seq` gap, or holds `map_epoch ≠ base_epoch` for a delta, it discards local state, renders "stale", and requests a `KIND_SNAPSHOT`. It never patches a map it cannot prove is current.*

`RequestSync` (§3.5, D19) is that request. Its answer is either a snapshot or the deltas the Mac missed. A replayed delta is provably current, because it chains from the epoch the Mac holds and passes the same `seq` and `base_epoch` checks as a live one. So the Mac keeps its last good mirror while STALE instead of discarding it. It never shows that mirror as current. Applied per topic:

**`map_bricks`.** A delta with `seq` ≤ the last applied and `map_epoch` ≤ the mirror's epoch is a **duplicate**: the mirror already holds that state or a later one. It is ignored and counted, and it is not a gap. Epochs only grow along the chain, because a delta's `map_epoch` must exceed its `base_epoch`. A restart that resets epochs is still caught by the `MemoryStatus` rule below.

Any one of these makes the map STALE:
- `seq` ≠ previous + 1 for anything but a duplicate, including a reset to a lower value when the node restarts.
- A delta whose `base_epoch` ≠ the mirror's epoch.
- A delta as the first message after (re)connect.
- `MemoryStatus.map_epoch` ≠ the mirror's epoch in 2 consecutive status messages. This catches a lost delta that no later delta exposes (D20).

While STALE, the Mac behaves as follows.
- It renders the map greyed, labelled "resyncing", with no change overlays drawn as current.
- It sends `RequestSync{TOPIC_MAP_BRICKS, since = (mirror epoch, last applied seq)}`, or `since` = `SINCE_NONE` if it holds no mirror. After a (re)connect it first waits up to 5 s for the snapshot the Spark sends on its own (§2), and requests only if none arrives.
- It applies, from the live topic or the sync topic, only what the live rules allow:
  - A snapshot is always accepted. It resets the `seq` baseline and the epoch.
  - A delta must have `seq` = last + 1 and `base_epoch` = the mirror's epoch.
- The map is current again once the mirror has applied every `seq` up to the highest seen on the live topic, and its epoch equals the newest `MemoryStatus.map_epoch`.
- Live messages that arrive while STALE may be buffered (at most 10, the topic's depth) and applied in `seq` order once they chain. Otherwise they are dropped.
- If a request has not ended STALE within 5 s, the Mac sends it again with its current `since`.

The Spark keeps its last 10 `map_bricks` messages. It answers a `TOPIC_MAP_BRICKS` request on `/room_memory/sync/map_bricks`:
1. **Replay** when all of these hold:
   - `since` is not `SINCE_NONE`.
   - The kept messages include the one with `seq = since_seq`, and that message has `map_epoch = since_epoch`.
   - At least one message follows it.

   It republishes every message after `since_seq` unchanged (same `seq`, same bytes), in `seq` order.
2. **Snapshot** otherwise. That covers no `since`, a `since_seq` older than the kept messages, a `since_seq` newer than the latest (the node restarted), an epoch that does not match, and nothing missed. The Spark publishes a new snapshot of the current map. Its `seq` is the latest `seq` published on the live topic, so the next live delta chains from it.

**`events`.** A message with `seq` ≤ the last applied, for an `event_id` the Mac already holds in that `state`, is a duplicate: ignored and counted. Any other `seq` ≠ previous + 1 is a gap, including a reset after a node restart. A gap marks the event list "incomplete since \<stamp of the last good message\>". Open events show "may be outdated". Missed transitions are never invented. The Mac sends `RequestSync{TOPIC_EVENTS, since = (map_epoch, seq) of the last event message applied}`, or `SINCE_NONE` when it holds none. It does the same on every (re)connect, and it repeats the request after 5 s like the map request.

The Spark keeps its last 100 `ChangeEvent` messages. It answers on `/room_memory/sync/events`:
1. **Replay** when the kept messages include the one with `seq = since_seq` and `map_epoch = since_epoch`. It republishes every message after it, unchanged, in `seq` order. Once the Mac has applied them contiguously up to the highest `seq` seen on the live topic, the gap is closed and "incomplete" clears.
2. **Bounded fallback** otherwise. Unchanged, in `seq` order, the Spark publishes:
   - the latest message of each open event (CANDIDATE or CONFIRMED) that is older than the kept messages;
   - then all kept messages.

   That is at most (open events + 100) messages. The Mac applies them by `event_id`, ignoring a state it already holds and accepting the first message per `event_id` as in §4.2. Its `seq` baseline becomes the highest `seq` replayed. The history stays "incomplete since \<stamp\>", because transitions older than the kept messages are gone. Events the Mac held open that were not re-sent keep "may be outdated" until a later transition for them arrives: their closing transition may be among the lost ones.

**Sync topics are shared.** Every Mac subscribed to a sync topic receives every answer. A Mac reads a sync topic only while it has a request outstanding on that topic, and applies only what the rules above allow. An answer meant for another Mac is therefore either one it can use or one it ignores.

**Older transitions on a sync topic.** A sync message with `seq` ≤ the last applied is ignored. In a fallback replay, the Mac also ignores a state its event has already passed in §4.2's order (CANDIDATE, then CONFIRMED, then a terminal state). Neither counts as a protocol violation.

**`status`, `epoch`.** These are full-state messages, so a gap is only counted. Each is stale when its newest message is older than 3 periods: 3 s for `status` (inside PR 3.1's "stale within 5 s") and 15 s for `epoch`.

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

| stream | rate | message size ("lab", default zlib) | steady state |
|---|---|---|---|
| MemoryStatus | 1 Hz | 68 B (exact) | 0.07 KB/s |
| Epoch | 0.2 Hz | 53 B (exact) | 0.01 KB/s |
| ChangeEvent, no mask | per transition, < 1/min | 136 B (exact) | < 0.01 KB/s |
| ChangeEvent with a 25 cm box mask | per transition | 253 B (measured) | < 0.01 KB/s |
| MapBricks delta, 25 cm box added | ≤ 0.2 Hz, on reference change | 367 B (measured) | < 0.1 KB/s |
| MapBricks snapshot | node start, (re)connect, and the answer to a request that cannot be replayed | 52,863–59,024 B (measured, see below) | — |
| RequestSync (Mac → Spark) | on (re)connect and on a gap | 53 B (exact) | — |

**Measured** by the size test in `RoomMemoryCDRTests.swift`.
- The input is a full-sphere synthetic accumulation of the furnished 8 × 8 × 3 m room: 720 × 360 rays, ±1 cm range noise, seed 1, 5 cm voxels.
- Every brick with any evidence is sent, all-free ones included (D5).
- The sizes are whole encoded messages, the payload compressed as the column says. zlib is Apple's raw DEFLATE at level 5. Python's level 6 on the Spark gives sizes that differ by a few percent.

| message | bricks | none | zlib (default) | lzfse |
|---|---|---|---|---|
| snapshot, floor at z = 0 on a brick boundary | 4,081 | 546,926 B | 59,024 B | 54,091 B |
| the same room shifted (0.2, 0.2, 0.1) m off the brick grid | 3,412 | 457,280 B | 52,863 B | 48,816 B |
| snapshot, surface bricks only (the brief's size model, not v1) | 2,211 | 296,346 B | 53,375 B | 51,194 B |
| delta: 25 cm box added | 12 | 1,680 B | 367 B | 413 B |
| APPEARED event, mask compressed as the column says | 2 (mask) | 476 B | 253 B | 293 B |

**Reading the numbers.**
- The default zlib snapshot is **53–59 KB**, 8–9× under the 500 KB target and 34× under the 2 MB cap.
- Keeping all-free bricks (D5) costs little once compressed.
  - Raw, it takes the snapshot from 2,211 surface bricks to 4,081 bricks, 296 KB to 547 KB.
  - With zlib it costs 5.6 KB (53.4 KB → 59.0 KB), because all-free bricks compress to almost nothing.
- Uncompressed, raw size depends on where the brick grid sits relative to the room (D6):
  - With the floor and two walls exactly on brick boundaries, ±1 cm noise splits their hits across two brick layers: 4,081 bricks, **547 KB, over the 500 KB target**.
  - With the room off the grid it is 3,412 bricks and 457 KB.
  - So `COMPRESSION_NONE` is for goldens and debugging only.
- A (re)connect costs one snapshot plus the latest status and epoch, about 0.06 MB with zlib. The 2 MB per-(re)connect budget leaves room for the event fallback replay (§5), at most (open events + 100) × a few hundred bytes.

## 8. Decisions for review

Each item is a place where §2.4 was not legal ROS 2, was underspecified, or conflicted with the honest-state rules. Each states what this doc does and why. All need Codex review. Trevor decided D5, D9 and D19 on 2026-10-14; the parts of D19 marked below go beyond his decision and need his confirmation.

- **D1. ROS 2 syntax.** §2.4's text was made legal ROS 2. Names, types and order are unchanged except in `MapBricks`, where D9 moved `compression` ahead of the brick arrays and packed those into `payload`. The syntax changes:
  - Constants go one per line (`KIND_SNAPSHOT=0, KIND_DELTA=1` became two lines).
  - Fields joined with `;` go one per line.
  - ROS 1 `time` became `builtin_interfaces/Time` (`int32 sec`, `uint32 nanosec`), for `stamp` as well.
  - `per_sensor_hz[2]` became `uint16[2] per_sensor_hz`.
  - "`MapBricks mask # optional`" became `MapBricks[<=1] mask`, since ROS 2 has no optional fields.
  - Named constants were added for values §2.4 described in comments: `COMPRESSION_*`, `SENSOR_*`, `HZ_UNAVAILABLE`. `RequestSync` (D19) adds `TOPIC_*` and `SINCE_NONE`.
- **D2. Common fields.**
  - They are `string room_id`, `uint64 map_epoch`, `uint64 seq` and `builtin_interfaces/Time stamp`, first in every message and in the brief's order.
  - They are written inline rather than as a shared header message; the wire bytes are identical either way.
  - MemoryStatus's own `map_epoch` is the common one, and Epoch's "epoch id" is `map_epoch`.
  - `RequestSync` carries them too: `map_epoch` repeats `since_epoch` and `seq` counts the client's requests (D19).
  - `room_id` is a non-empty UTF-8 string. Its format, such as a slug, is not fixed.
- **D3. Enum values** follow §2.4's listing order from 0. Unknown values reject the message instead of mapping to a default.
- **D4. Classic CDR (XCDR1) with 8-byte `uint64` alignment.** This is an assumption, to be confirmed by the Spark goldens (§9). The ≤ 3 trailing bytes rule comes from rmw padding.
- **D5. Absent brick = UNKNOWN; all-free bricks are sent.** Decided by Trevor, 2026-10-14. §2.4's `removed_bricks_idx` comment says bricks "that became all-unknown/all-free".
  - Removing an all-free brick would turn free evidence into UNKNOWN on the Mac, or make absence ambiguous. Either way it breaks UNKNOWN ≠ free.
  - So removal means "became all-unknown" only, and all-free bricks are sent.
  - Cost, measured (§7): 4,081 instead of 2,211 bricks. That is 547 KB instead of 296 KB raw, but only 59.0 KB instead of 53.4 KB with the default zlib.
- **D6. Voxel and bit layout.** Voxel index `floor(coordinate / voxel_size)`; brick `(i, j, k)` covers voxels `8i … 8i+7` (likewise for j, k); bit `x + 8y + 64z`, LSB first. §2.4 did not specify these.
  - Where the room frame's origin sits relative to the floor and walls changes snapshot bytes by about 20 % (§7).
  - A floor at exactly z = 0 straddles a brick boundary. Choosing the room-frame origin (PR 2.2) is a size decision as well as a convenience.
- **D7. Canonical order.**
  - Bricks are strictly increasing in `(i, j, k)`, so that `removed_bricks_idx` (`uint32`, as specified) has a meaning: an index into the `base_epoch` brick list in that order.
  - A delta applies removals first, then upserts.
  - Brick coordinates would be sturdier than indices, at 6 B instead of 4 B each. That is left for review rather than changed.
- **D8. Brick invariants.** `occupied ⊆ known`, and every sent brick has ≥ 1 known voxel. A snapshot carries no removals. A delta's `map_epoch` exceeds its `base_epoch`, which §5's duplicate rule relies on.
- **D9. `compression` before the payload.** Decided by Trevor, 2026-10-14.
  - §2.4 had "`uint8 compression # 0 none, 1 zstd (whole payload)`" as the *last* field of `MapBricks`. A byte after the arrays cannot describe compression of the arrays before it, and typed CDR arrays leave a compressed stream nowhere to live.
  - So `compression` now comes right after the common fields. The four brick arrays are packed into one `uint8[] payload`, and `n_bricks` and `n_removed` stay outside it.
  - Those two counts fix the uncompressed size before anything is inflated. That gives the decoder an exact length check and a limit (D22).
  - The payload is packed little endian in either CDR byte order. It is an opaque byte array, the Spark (aarch64) and the Mac are both little endian, and Python packs it with `struct.pack('<…')`.
  - Supported values:
    - `NONE` (0).
    - `ZLIB` (1), the default for every `MapBricks`. It is raw DEFLATE, chosen because it is what Foundation's `.zlib` writes and Python reads with `zlib.decompress(payload, -15)`. The zlib-wrapped format would need a hand-written header and Adler-32 on the Mac.
    - `LZFSE` (2), Mac-native. The Spark has no stdlib encoder for it.
  - zstd is dropped, because Foundation has none, and its old value 1 is reused.
  - Decoding uses the Compression framework's stream API, so a stream that ends early, runs long, or never ends is caught exactly.
  - Apple's inflater is laxer than zlib in two ways (§3.1). The Mac checks the size, not the stream's framing.
  - Goldens: DEFLATE output is not canonical. So each compressed golden carries a hand-built stored block or LZFSE uncompressed block, and the tests check:
    - the encoder's bytes up to the payload;
    - that its own stream inflates to the hand payload;
    - a round trip (§9).
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
- **D18. Topic names and QoS** (§2) are proposals for PR 2.4. They include the request topic and the two sync topics of D19.
- **D19. `RequestSync`.** Trevor decided on 2026-10-14 that the Mac sends `RequestSync{room_id, since_epoch, since_seq, stamp}`, which the Spark answers with a snapshot or the missed deltas and events. §3.5 defines the message and §5 the answer rule. These parts go beyond his field list and need his confirmation:
  - **Common fields.** `map_epoch` and `seq` were added because §1.4 puts the four common fields first in every message. `map_epoch` repeats `since_epoch`, and both directions validate that. `seq` counts the client's requests.
  - **`topic`.** `seq` is per topic, so one `since_seq` cannot describe both `map_bricks` and `events`. The Mac sends one request per stale topic.
  - **`SINCE_NONE` = 2⁶⁴ − 1** means "holds nothing". It is not 0, because epoch 0 and seq 0 are real values (§1.5).
  - **Answers go out on separate sync topics** (`/room_memory/sync/map_bricks`, `/room_memory/sync/events`), not on the live topics. Replays keep their original `seq`.
    - On a live topic, other Macs would see `seq` go backwards. They could not tell a replay from a node restart, which §5 must treat as a gap.
    - With separate topics the live rules stay strict, and a Mac reads a sync topic only while its own request is outstanding.
  - **The answer is bounded.**
    - Maps: at most 9 replayed deltas, else one snapshot.
    - Events: replay from the last 100 messages, else the bounded fallback of §5. That fallback leaves the history marked incomplete instead of reconstructing it.
  - **`RequestSync` adds to the Spark's own snapshots; it does not replace them.** The Spark still sends a snapshot on the live topic when the node starts and on (re)connect (§2), so a Mac gets a map without sending anything.
  - **Not verified:** whether the deployed foxglove_bridge accepts client publishing (`clientPublish`) for a custom message type. PR 2.4 / 3.1 must check it, together with how the Spark notices a (re)connect through the bridge. If client publish is unavailable, the Mac cannot request. A stale map then waits for the Spark's next own snapshot, and an event gap stays open.
- **D20. Staleness.**
  - `status` is stale after 3 s and `epoch` after 15 s.
  - A missed trailing delta is caught by `MemoryStatus.map_epoch` disagreeing with the mirror twice in a row.
- **D21. `package.xml` license** says `Proprietary` because the repo has no LICENSE file. Trevor should confirm.
- **D22. Payload limit.** The uncompressed `MapBricks` payload, `134 × n_bricks + 4 × n_removed`, is at most 16 MiB (16,777,216 bytes), about 125,000 bricks or 8,000 m³ fully covered. Without a limit, a 2 MB zlib payload could declare and inflate to over 2 GB. The decoder rejects a larger declared size before inflating, and the encoder refuses to write one (D16). The measured room needs 547 KB.

## 9. Verification status

- **Hand goldens.** `Tests/Fixtures/room/golden/hand/*.hex` were written byte by byte from these rules, with every span annotated. There are 12: 9 little endian and 3 big endian, covering all five messages. The tests check:
  - The Swift decoder reads all 12 to the expected values. The big-endian `MapBricks` golden has a little-endian payload inside a big-endian message.
  - The encoder reproduces the 7 uncompressed little-endian goldens exactly.
  - For the zlib and LZFSE goldens, which hold a hand-built stored block and an uncompressed LZFSE block:
    - the encoder matches the bytes up to the payload length;
    - its own compressed stream, inflated through Foundation rather than the codec, equals the hand payload;
    - that stream decodes back to the value.
  - Every truncation throws.
- **Compression interop, checked once by hand.** The test runner has no Python, so these checks are not in the suite.
  - Python 3.9 `zlib.decompress(payload, -15)` inflates Apple's `.zlib` output of a 547 KB payload. Plain `zlib.decompress(payload)` rejects it.
  - Apple inflates Python's `zlib.compressobj(6, zlib.DEFLATED, -15)` output of the same payload.
  - The packing helpers in `scripts/spark/make_room_memory_goldens.py` reproduce the payload bytes of all 5 hand goldens that have one.
- **No independent CDR cross-check yet.** No offline ROS 2 CDR library (`rosbags`, `mcap_ros2`) is installed on this Mac, and none was installed for this PR. So the hand goldens have only been checked against the Swift codec, which follows the same rules.
- **Spark goldens: not produced.** `scripts/spark/make_room_memory_goldens.py` serializes the 9 little-endian values with `rclpy` after a `colcon build` of `labcommand_msgs` in the Jazzy container. Running it is a Spark task. The follow-up test then requires each Spark file to decode to the same value as its hand golden, which confirms D4.
- **`colcon build` of `labcommand_msgs`: not run.** There is no ROS 2 on the Mac.
- **The resync rules of §5 are a contract only.** They are not implemented yet: `RoomMemoryStore` is PR 1.1 and the Spark side is PR 2.4.

## 10. Open questions for Trevor (brief §5, Q1–Q4)

These are copied from the brief and left unanswered.

1. **Is the rig static for v1?** If it will move between sessions, relocalization (and place recognition) moves from optional to required.
2. **Room and targets:** room size, smallest object that must be detected (5 cm? 10 cm?), acceptable latency (seconds vs minutes), acceptable false events per hour. These set voxel size, T_commit and the PR 3.2 bar.
3. **HDC's role.** Your own record (exp119/122/123, exp113, D-33) says a superposed memory is not a cheaper map, and HDC is parked. Should the app be HDC-free with HDC kept only as an optional RT3 comparison, or do you want to reopen HDC? This brief assumes the former.
4. **Fixtures:** may the 12 Oct box takes (`*ROBOT_box_*`, `*speedybee_box_*` on the Spark) be copied to the Mac as golden fixtures? This brief did not access the Spark.
