# Room memory protocol, v1 (Spark `room_memory` → LabCommand Mac)

Status: draft for PR 0.1. Codex has not signed it off. Trevor decided D5, D9 and D19 on 2026-10-14 (§8); he has not answered Q1–Q5 (end of this doc). PR #13 added D23–D24 (the `seq` rule and durability), D25 (`OperatorAction`), D26 (the events high-water mark, `MemoryStatus.events_seq`) and D27 (the events fallback header, `EventsFallback`) as proposals. PR #14 (the Spark node, merged before it) added D28–D33 and §3.7 (`Activity`). Sources: `RESEARCH_BRIEF.md` §2.2–§2.5 (2026-10-14). The `.msg` files are in `ros/labcommand_msgs/msg/`. The Swift codecs are `Sources/LabCommandCore/RoomMemoryMessages.swift` and `RoomMemoryCDR.swift`.

The Spark does all perception and keeps the memory. The Mac mirrors the reference map, the event list and the timeline, and renders them. It never infers a change itself. Five Spark → Mac topics carry the state, three sync topics carry answers to the Mac's resync requests, and two Mac → Spark topics carry those requests and the operator's actions. How they travel between the Spark and the Mac is open (§2, D28, Q5): a dedicated foxglove_bridge on :8767 in the room-memory container, or the existing foxglove_bridge on :8765 with the requests relayed over the ssh-authenticated command-center API on :8766.

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
| `map_epoch` | `uint64` | epoch | bumped on every reference-map change and on every node start that loads saved references; in `RequestSync`, repeats `since_epoch` | always present; a stale message renders "—" |
| `seq` | `uint64` | count | +1 per message on that topic, starts at 0 per publisher; `0` inside `ChangeEvent.mask`; in `RequestSync` and `OperatorAction`, +1 per request or action from that client | duplicates, gaps and restarts follow §5's `seq` rule |
| `stamp` | `builtin_interfaces/Time` | s, ns | `nanosec` < 10⁹ | `{0, 0}` renders "time —" |

A message whose `room_id` differs from the room being viewed is ignored.

### 1.5 Missing values

Every field has one of three "missing" encodings:

- **float32 measurements:** NaN means *not available*. ±Inf is *invalid*. In Swift these are Optionals, so a missing value cannot be read as 0.
- **Integers that can be unmeasured or absent:** a named sentinel. v1 has five:
  - `HZ_UNAVAILABLE = 65535` in `per_sensor_hz`, where `0` is a measured zero.
  - `SINCE_NONE = 2⁶⁴ − 1` in `RequestSync`, and copied into `EventsFallback`, where `0` is a real epoch or seq.
  - `EVENTS_SEQ_NONE = 2⁶⁴ − 1` in `MemoryStatus.events_seq`, where `0` is a real seq.
  - In `OperatorAction`, `EVENT_NONE = 2⁶⁴ − 1` and `STATE_NONE = 255`, the room-level target and its expected state.
- **Counters, epochs, enums, ids:** they cannot be missing inside a valid message. If the message itself is missing or stale (§5), they render "—".

Decoders reject an out-of-range enum value as *invalid*, and the whole message is dropped. A dropped message is never partly applied. The Mac counts it in a diagnostics line ("n room-memory messages rejected").

## 2. Topics

Topic names are a proposal for PR 2.4 (D18):

| topic | direction | message | rate | QoS (proposal): reliability, durability, history |
|---|---|---|---|---|
| `/room_memory/map_bricks` | Spark → Mac | `MapBricks` | snapshot when TRACKING starts and then every `keyframe_period_s` (60 s); deltas at least `delta_period_s` (5 s) apart, only when the reference changed (D29) | reliable, **volatile**, keep-last 10 |
| `/room_memory/events` | Spark → Mac | `ChangeEvent` | one per lifecycle transition, typically < 1/min | reliable, **volatile**, keep-last 100 |
| `/room_memory/status` | Spark → Mac | `MemoryStatus` | 1 Hz | reliable, **transient_local**, keep-last 1 |
| `/room_memory/activity` | Spark → Mac | `Activity` | 1 Hz, with each status (D33) | reliable, **transient_local**, keep-last 1 |
| `/room_memory/epoch` | Spark → Mac | `Epoch` | 0.2 Hz | reliable, **transient_local**, keep-last 1 |
| `/room_memory/request_sync` | Mac → Spark | `RequestSync` | on (re)connect and on a gap, ≤ 1 per topic per 5 s | reliable, **volatile**, keep-last 10 |
| `/room_memory/operator` | Mac → Spark | `OperatorAction` | one per operator action | reliable, **volatile**, keep-last 10 |
| `/room_memory/sync/map_bricks` | Spark → Mac | `MapBricks` | answers to `TOPIC_MAP_BRICKS` requests | reliable, **volatile**, keep-last 10 |
| `/room_memory/sync/events` | Spark → Mac | `ChangeEvent` | answers to `TOPIC_EVENTS` requests | reliable, **volatile**, keep-last 200 |
| `/room_memory/sync/events_fallback` | Spark → Mac | `EventsFallback` | one header per events fallback (§5, D27) | reliable, **volatile**, keep-last 10 |

**Who publishes.** The room-memory node publishes the eight Spark → Mac topics and is the only subscriber of the two Mac → Spark topics. The Mac originates `RequestSync` and `OperatorAction`. How they reach the node is part of the open transport question (D28, Q5, below): foxglove_bridge's client-publish path, which has not been checked against the deployed bridge (D19), or an ssh-authenticated relay on the command-center API (`docs/phase3-live-integration.md`, P11). Either path carries the CDR bytes unchanged.

**Durability (D24).** Every publisher and subscriber declares durability explicitly.
- **`status`, `activity` and `epoch` are transient_local, depth 1.** A subscriber that joins late gets the last message published, then the live stream. That subscriber may be foxglove_bridge when the first Mac subscribes. All three are full-state messages, so a copy delivered twice is harmless: §5's `seq` rule ignores it.
- **Every other topic is volatile.** A late joiner gets nothing published before it subscribed.
  - `map_bricks` and `events`: the Mac gets the map and the recent events by `RequestSync` (§5). A cached old message would also break §5's restart detection, which relies on live `seq` only rising.
  - The sync topics, fallback headers included: an answer is for the requests outstanding when it is sent, not for a later subscriber.
  - `request_sync` and `operator`: a restarted node must not receive a stored request or action again. A replayed `OperatorAction` would act twice.
- **The nodes match this only for `activity`.** The Python node (`spark/room_memory`) and the C++ node (`spark/room_memory_cpp`) declare `activity` transient_local (PR #14). They publish `status` volatile, leave every other topic at the ROS 2 default (volatile) without declaring it, and have no `epoch` topic. The change is planned in `docs/phase3-live-integration.md` §2.5.
- In ROS 2 a volatile subscriber still receives from a transient_local publisher. A transient_local subscriber receives nothing from a volatile one.

**Transport (D28, open: Q5).** Two options are on the table, and Trevor has not chosen. Both carry the same CDR bytes, and the node answers whatever arrives on `/room_memory/request_sync`, so the protocol does not depend on the choice.
- **A dedicated bridge on :8767** (implemented in PR #14's launch file, off with `foxglove_bridge:=false`). The Mac requests over the same bridge's client publishing, or over a relay.
- **The existing bridge on :8765, with the requests relayed over the ssh-authenticated command-center API on :8766.** :8765 needs `labcommand_msgs` in its environment for the memory topics. `docs/phase3-live-integration.md` (P11) designs the relay.
- Phase 3's design combines the two: the :8767 bridge, kept read-only, for the Spark → Mac topics, and the :8766 relay for `RequestSync` and `OperatorAction` (`docs/phase3-live-integration.md`, P11, P12). It waits on the same choice.

The :8767 option as implemented: the room-memory container starts its own foxglove_bridge on :8767 (launch argument `foxglove_bridge`, default on), with `topic_whitelist` `^/room_memory/.*`, so the raw clouds never leave through it, and `labcommand_msgs` in its environment, so Foxglove and the Mac get the full message definitions. It is read-only by default (capabilities `connectionGraph` only). foxglove_bridge 3.4.1 parses `client_topic_whitelist` but does not enforce it, so turning on client publishing (`foxglove_client_publish:=true`) lets any client that reaches the port publish on any topic. The node answers whatever arrives on `/room_memory/request_sync`, so a relay that publishes the Mac's bytes there works the same way.

**Keyframes are the (re)connect snapshot (D29).** The bridge shares one ROS subscription across its clients, so the node cannot see a Mac (re)connect. The periodic keyframe on the live topic plays that role: a Mac that (re)connects holds a current map within `keyframe_period_s` without sending anything, and `RequestSync` makes it immediate where it can be sent.

## 3. Messages

The `.msg` text below is the legal ROS 2 version of §2.4. §8 (D1) lists every syntax change. Field names, types and order are as §2.4 gives them, except for `MapBricks`, whose brick arrays moved into a compressed payload (D9). §3.5 adds `RequestSync` (D19), §3.6 adds `OperatorAction` (D25), §3.7 adds `Activity` (D33), and §3.8 adds `EventsFallback` (D27). The *missing* column says how "not available" travels on the wire, and the *rendering* column says what the Mac shows then. "Stale" is the message-level rendering of §5.

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
uint64 EVENTS_SEQ_NONE=18446744073709551615
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
uint64 events_seq
```

`map_epoch` in §2.4's MemoryStatus is the common field and is not repeated (D2). `events_seq` is not in §2.4; D26 adds it.

| field | type | unit | valid | missing on the wire | rendering when missing / invalid |
|---|---|---|---|---|---|
| `state` | `uint8` | enum | `STATE_*` (§4.1) | cannot be missing | other value: message rejected; status stale after 3 s |
| `n_bricks` | `uint32` | bricks | bricks in the published reference map, the one `map_bricks` carries (D30) | cannot be missing | stale: "—" |
| `n_open_events` | `uint32` | events | events in CANDIDATE or CONFIRMED, as published: an accepted event counts until its ACCEPTED transition is published (§4.2) | cannot be missing | stale: "—" |
| `coverage_known_frac` | `float32` | fraction 0–1 | observed share of the room volume, defined below | NaN = room volume not yet known | "coverage —", never 0 % |
| `pose_residual_m` | `float32` | m | finite, ≥ 0; the rig-pose estimate defined below | NaN = guard has not run | "guard not run", never "0 m" |
| `window_s` | `float32` | s | finite, > 0 (0.5–1.0 expected) | NaN = not reported | "window —" |
| `per_sensor_hz` | `uint16[2]` | Hz, frames received per second, rounded | `[0]` Mid-360, `[1]` JT128 (same order as `sensors`) | `65535` = not measured | "—". `0` renders "0 Hz", a measured zero |
| `dropped_windows` | `uint32` | windows since node start | any | cannot be missing | stale: "—" |
| `events_seq` | `uint64` | count | `seq` of the newest message published on live `events` in this generation: the events high-water mark (§5) | `EVENTS_SEQ_NONE` = no event published yet | — |

**Definitions (v1, as the Spark computes them).**
- **`coverage_known_frac`.** Of the published reference map (D30): the number of known voxels (occupied or free) inside the axis-aligned voxel-index box of its occupied voxels, divided by the number of voxels in that box. The box stands in for the room volume until one is configured; voxels inside walls and furniture are in it and can never be observed, so the value stays well below 1 (0.17–0.18 on the JT128 recording). NaN while the reference has no occupied voxel. It is computed during BUILDING too, from the reference built so far.
- **`pose_residual_m`.** The size of the translation that best registers the latest window onto the reference, per sensor, then the largest over sensors. For each axis separately, the window's occupied voxels are shifted by −4 … +4 voxels and counted where they land on reference-occupied voxels. The best shift is refined by the parabola through that count and its two neighbours, unless it is ±4. The value is the length of the three per-axis shifts, in meters. Walls normal to an axis register that axis, and floors and ceilings do not mask a horizontal move. On the static recording it is 2.8 mm median (3.4 mm max). A real (2, 2, 0) cm move of the JT128 reads 26 mm, (5, 5, 0) cm reads 74 mm and 3 cm vertical reads 35 mm. It saturates near 0.35 m. NaN while BUILDING and when no shift overlaps at all. The memory stops learning from a window whose estimate exceeds `learn_max_offset_m` (0.02 m). Acting on it (RIG_MOVED, §4.1) is not implemented yet.

**Wire size.** `4 + C + 40` bytes: 76 B for "lab", 84 B for "lab-a".

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
| `since_epoch` | `uint64` | epoch | `map_epoch` of the message at the Mac's baseline for `topic` (§5's `seq` rule); for `TOPIC_MAP_BRICKS` the epoch the mirror holds | `SINCE_NONE` = the Mac holds no usable baseline for `topic` (§5) | `SINCE_NONE` in only one of `since_epoch`, `since_seq`: *invalid* |
| `since_seq` | `uint64` | count | `seq` of that message | `SINCE_NONE`, together with `since_epoch` | as above |
| `topic` | `uint8` | enum | `TOPIC_*` | cannot be missing | other value: *invalid* |

`SINCE_NONE` is 2⁶⁴ − 1, not 0, because epoch 0 and seq 0 are real values. In Swift `since` is an Optional `Position`. The Spark answers by the rules in §5.

**Wire size.** `4 + C + 17` bytes: 53 B for "lab".

### 3.6 `OperatorAction`: one operator action, Mac → Spark

Proposed in PR #13 (D25). It is published on `/room_memory/operator` (§2), and the room-memory node is its only subscriber.

```
uint8 ACTION_ACKNOWLEDGE=0
uint8 ACTION_ACCEPT=1
uint8 ACTION_DISMISS=2
uint8 ACTION_LABEL=3
uint8 ACTION_MARK_GROUND_TRUTH=4
uint8 ACTION_PAUSE=5
uint8 ACTION_RESUME=6
uint8 ACTION_REREFERENCE=7
uint64 EVENT_NONE=18446744073709551615
uint8 STATE_NONE=255
string room_id
uint64 map_epoch
uint64 seq
builtin_interfaces/Time stamp
uint8 action
uint64 event_id
uint8 expected_state
string text
string operator_id
string action_id
```

The common fields of §1.4 are the message's header. `std_msgs/Header` is not used: it would break "common fields first" (D2) and add a dependency.

| field | type | unit | valid | missing on the wire | rendering when missing / invalid |
|---|---|---|---|---|---|
| `map_epoch` | `uint64` | epoch | the epoch of the map the operator was shown | cannot be missing | ACCEPT or REREFERENCE on another epoch than the node's published one: refused and logged |
| `seq` | `uint64` | count | this client's action counter, +1 per action, from 0 | cannot be missing | — |
| `stamp` | `builtin_interfaces/Time` | s, ns | when the operator acted; the time of a MARK_GROUND_TRUTH mark | cannot be missing | `{0, 0}`: *invalid* here, unlike §1.4, because a mark needs its time |
| `action` | `uint8` | enum | `ACTION_*` | cannot be missing | other value: *invalid* |
| `event_id` | `uint64` | id | the target event for ACKNOWLEDGE, ACCEPT, DISMISS and LABEL; an event or `EVENT_NONE` for MARK_GROUND_TRUTH; `EVENT_NONE` for PAUSE, RESUME and REREFERENCE | `EVENT_NONE` | the wrong one of the two for the action: *invalid*; an id the node does not know: refused and logged |
| `expected_state` | `uint8` | enum | the `ChangeEvent.STATE_*` the operator saw, when `event_id` is set; `STATE_NONE` when it is not | `STATE_NONE` | set when `event_id` is not, or the reverse: *invalid*. ACCEPT or DISMISS whose event is no longer in that state on the node, counting an accept not yet published: refused and logged |
| `text` | `string` | — | UTF-8, no NUL, ≤ 256 bytes; the label for LABEL, the description for MARK_GROUND_TRUTH, an optional note otherwise | `""` = no text | empty on LABEL or MARK_GROUND_TRUTH: *invalid* |
| `operator_id` | `string` | — | non-empty UTF-8, no NUL, ≤ 64 bytes; the name the operator gave the app | cannot be missing | empty: *invalid*. It is self-asserted, not authenticated |
| `action_id` | `string` | — | a client-generated UUID, 36 lowercase characters | cannot be missing | other format: *invalid*. A repeated `action_id` is ignored, so a retried send acts once |

| action | target | effect on the Spark | what the Mac sees next |
|---|---|---|---|
| ACKNOWLEDGE | an event, any state | the event log records that the operator saw it; no state change | nothing on the wire in v1 |
| ACCEPT | a CONFIRMED event | §4.2 "operator accept": CONFIRMED → ACCEPTED, the reference absorbs the change and `map_epoch` bumps | a delta or snapshot that carries the change, then the ACCEPTED transition with that message's `map_epoch` (§4.2) |
| DISMISS | a CONFIRMED event | §4.2 "operator revert": CONFIRMED → REVERTED; the reference is unchanged | the REVERTED transition |
| LABEL | an event | the event log stores `text` as the label of the event's region | nothing on the wire in v1 |
| MARK_GROUND_TRUTH | an event, or none | the event log stores a ground-truth mark at `stamp` with `text`, for scoring | nothing on the wire |
| PAUSE | the room | §4.1 any → PAUSED | `MemoryStatus.state` PAUSED |
| RESUME | the room | §4.1 PAUSED → the state the guard check gives | `MemoryStatus.state` |
| REREFERENCE | the room | re-baselining: §4.1 "operator re-references the room", RIG_MOVED → BUILDING; the new reference bumps `map_epoch`. §4.1 has no re-reference from other states, so there it is refused | `MemoryStatus.state` BUILDING, then a snapshot |

- **No reply message.** The outcome is what the Spark already publishes. ACKNOWLEDGE, LABEL and MARK_GROUND_TRUTH change nothing on the wire, so other Macs do not see them in v1. Showing them would need a field in `ChangeEvent`, which is a breaking change.
- **Compare-and-set.** `map_epoch` is compared with the epoch the node has published, not one it holds back. `expected_state` is compared with the node's own state of the event. That includes an accept whose ACCEPTED transition is still waiting for its map message (§4.2), so a second operator cannot dismiss an event that was already accepted. A refused action changes nothing. The node logs every action with its `action_id`, `operator_id`, arrival time and outcome.
- **Validation is symmetric (D16).** The node drops an *invalid* action and logs it. The Mac's encoder refuses to write one.

**Wire size.** 117 B for "lab" with an empty `text`, `operator_id` "trevor" and a 36-character `action_id`. Each string adds its UTF-8 length and up to 3 bytes of alignment.

### 3.7 `Activity`: transient motion next to the rig, 1 Hz

```
uint8 SENSOR_MID360=1
uint8 SENSOR_JT128=2
string room_id
uint64 map_epoch
uint64 seq
builtin_interfaces/Time stamp
bool active
uint8 sensors
uint8 largest_sensor
uint32 n_regions
uint32 voxel_count
float32[3] centroid
float32[6] aabb
float32 since_s
```

Activity is never a change: nothing in it becomes a `ChangeEvent`, and `MemoryStatus` and `ChangeEvent` are unchanged (D33). An *activity region* is a change region of one sensor's tracking window that the window's own evidence supports and that either appeared new in that window or lost its shape (§4.2). Regions matched to an open event are not activity. `active` false means no such region, not a motionless room: still people, motion below `min_voxels` and motion inside an open event do not show.

| field | type | unit | valid | missing on the wire | rendering when missing / invalid |
|---|---|---|---|---|---|
| `seq` | `uint64` | count | per-topic counter, +1 per message | cannot be missing | — |
| `active` | `bool` | — | a tracking window since the previous `Activity` had activity | cannot be missing | other value than 0/1: message rejected |
| `sensors` | `uint8` | bitmask | `SENSOR_*` with activity in the newest such window; 0 when inactive | cannot be missing | undefined bit: message rejected |
| `largest_sensor` | `uint8` | enum | the `SENSOR_*` bit whose frame `centroid` and `aabb` are in (D30); one bit of `sensors`; 0 when inactive | cannot be missing | otherwise rejected |
| `n_regions` | `uint32` | count | activity regions in that window, all sensors; 0 when inactive | cannot be missing | — |
| `voxel_count` | `uint32` | voxels | their voxels (a move counts its new pose); 0 when inactive | cannot be missing | — |
| `centroid`, `aabb` | `float32[3]`, `float32[6]` | m | the largest activity region (most voxels; ties to the lower sensor bit), as a `ChangeEvent` would carry it | NaN ×3, NaN ×6 when inactive | partly NaN: rejected |
| `since_s` | `float32` | s | from the newest window with activity to `stamp`, ≥ 0 | NaN = none since the node started | negative: rejected |

**Wire size.** `4 + C + 52` bytes: 88 B for "lab".

### 3.8 `EventsFallback`: header of an events fallback, Spark → Mac

Proposed in PR #13 (D27). The Spark publishes one on `/room_memory/sync/events_fallback` (§2) with each events fallback (§5), right before the fallback's messages. It is the only way the Mac learns that an answer is a fallback, and the only thing that lets it give up a gap. Its `.msg` is `ros/labcommand_msgs/msg/EventsFallback.msg`.

```
string room_id
uint64 map_epoch
uint64 seq
builtin_interfaces/Time stamp
uint64 since_epoch
uint64 since_seq
uint64 kept_first_seq
uint64 kept_last_seq
uint64[] resent_seqs
```

| field | type | unit | valid | missing on the wire | rendering when missing / invalid |
|---|---|---|---|---|---|
| `map_epoch` | `uint64` | epoch | the node's published epoch when the fallback is sent | cannot be missing | — |
| `seq` | `uint64` | count | +1 per header on this topic, from 0. The Mac keeps no baseline for it, and a gap here triggers nothing | cannot be missing | — |
| `since_epoch` | `uint64` | epoch | copied from the `RequestSync` this fallback answers | 2⁶⁴ − 1, the value of `RequestSync.SINCE_NONE`, copied from a request that had none | 2⁶⁴ − 1 in only one of `since_epoch`, `since_seq`: *invalid* |
| `since_seq` | `uint64` | count | copied likewise | as above | as above |
| `kept_first_seq` | `uint64` | count | `seq` of the oldest kept `events` message | cannot be missing | above `kept_last_seq`: *invalid* |
| `kept_last_seq` | `uint64` | count | `seq` of the newest kept `events` message. The fallback sends every `seq` from `kept_first_seq` to here | cannot be missing | — |
| `resent_seqs` | `uint64[]` | count | the `seq`s of the open events' latest messages that the fallback sends before the kept ones, strictly increasing, each below `kept_first_seq` | `[]` = none | out of order, or not below `kept_first_seq`: *invalid* |

- **What it states.** Every `events` `seq` up to `kept_last_seq` that it does not name was, when it was sent, neither kept nor the latest message of an open event. The kept messages only move forward, and an event's latest message only gets newer, so no later answer can hold such a `seq` either. The header describes the Spark's kept messages, not the requester, so it holds for every Mac that sent the same `since`.
- **What it does not state.** An answer published *before* the header can still be in flight, carrying a `seq` the header omits. Such a `seq` left the kept messages, or stopped being its event's latest, only because later events were published, so `kept_last_seq` is newer than everything in that earlier answer. `/room_memory/sync/events` has one publisher and arrives in publish order, losing messages but never reordering them, as §5's `seq` rule assumes for live topics. So once the Mac has received a copy of `kept_last_seq` on that topic, every earlier answer has arrived or been lost. §5 waits for that copy.
- **No header without events.** If no event has been published in this generation, a fallback has nothing to send, and the Spark publishes neither it nor a header. The request is then done by its mark, `EVENTS_SEQ_NONE` (§5).
- **Invalid is lost.** The Mac drops an *invalid* header and logs it. Like a lost header, that gives up nothing, and the 5 s repeat recovers (§5).

**Wire size.** 72 B for "lab" with no resent `seq`, and 76 + 8n B with n ≥ 1. Computed by hand from §1.1, not encoded.
string room_id
uint64 map_epoch
uint64 seq
builtin_interfaces/Time stamp

## 4. State machines

The Spark owns both machines. The Mac renders them, and it flags a transition that is not in these tables as a protocol violation. It keeps the last legal state and logs the violation; it does not render the illegal state. Event transitions are judged in `seq` order: only a message newer than the last one applied to an event changes its state. An older one, filled in later by a replay, goes to the event's history only (§5's per-event order), and is not a violation.

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
| — | CANDIDATE | disagreement between the reference and recent layers where both have evidence that stays *settled* for `candidate_windows` (40) consecutive windows: each window's own evidence supports it (`fresh_min_voxels` 3 and `fresh_min_fraction` 0.5 of its voxels, both poses of a move), it keeps its shape (`stable_min_fraction` 0.9 of its voxels within `tolerance` of where they were when the count started, and back), and no activity region (§3.7) is within `activity_radius_m` (1.0 m). A stale window, a lost shape, nearby activity or a window without points starts the count again. Anything shorter sends nothing (D31, D33) | dashed outline, "candidate"; not counted as a change |
| CANDIDATE | CONFIRMED | persists ≥ T_commit (start at 5 s; tuned in PR 2.3) | solid overlay; counted; arrow for MOVED |
| CANDIDATE | EXPIRED | gone before T_commit (walk-through): no matching region for `expire_windows` (3) consecutive windows; superseded by a MOVED pairing; or memory left TRACKING for RIG_MOVED | removed from the overlay; history only |
| CONFIRMED | ACCEPTED | operator accept; the reference absorbs it and `map_epoch` bumps. The Spark publishes ACCEPTED only after the `map_bricks` message that carries the change, with that message's `map_epoch` | history entry. Once the mirror's epoch is ≥ the event's `map_epoch`: overlay removed, and the map shows the new state. Until then: region outlined "accepted · map updating", not counted as a change |
| CONFIRMED | REVERTED | the region returned to the reference state, or operator revert | removed; history only |

- ACCEPTED, REVERTED and EXPIRED are terminal. Only CONFIRMED renders as a change. CANDIDATE renders as a candidate, and REVERTED or EXPIRED never render as a change.
- An event's `kind` never changes. Moved pairing happens before confirmation: the paired CANDIDATEs go EXPIRED, and a new MOVED event starts at CANDIDATE (D17).
- A Mac that connects mid-session accepts any state as the first one it sees for an `event_id`. The same holds for the first message about an `event_id` after a gap in `events` (§5): the transitions in between may be lost.
- **ACCEPTED follows its map message.** Removing the overlay claims that the map now contains the change. So the Spark sends the map change first: the ACCEPTED transition is held until the delta or snapshot that carries the change is published, then published right after it. Several accepts merged into one delta, or overtaken by a snapshot, are all released after that message. Until then the event stays CONFIRMED on the wire. The two messages travel on different topics, and the map can be STALE, so the Mac also checks the epoch before it removes the overlay (table above).

## 5. Resync rule

From §2.4: *if the Mac sees a `seq` gap, or holds `map_epoch ≠ base_epoch` for a delta, it discards local state, renders "stale", and requests a `KIND_SNAPSHOT`. It never patches a map it cannot prove is current.*

`RequestSync` (§3.5, D19) is that request. Its answer is either a snapshot or the deltas the Mac missed. A replayed delta is provably current, because it chains from the epoch the Mac holds and passes the same `seq` and `base_epoch` checks as a live one. So the Mac keeps its last good mirror while STALE instead of discarding it. It never shows that mirror as current.

**The `seq` rule (D23).** This is the only statement of how `seq` orders messages. The per-topic rules after it say only what a gap does on each topic.
- **Generation and baseline.** A *generation* is one run of the Spark node. Within a generation, each topic's `seq` starts at 0 and rises by 1 per message (§1.4).
  - For each topic the Mac keeps a **baseline**, the `seq` up to which it has applied every message. On `map_bricks`, `status` and `epoch` that is the last message applied. On `events` it can be lower (below).
  - A live topic and its sync topic share one baseline.
  - Separately, the Mac keeps the highest `seq` received on each live topic.
  - On `events` it also keeps, per `event_id`, the `seq` of the last message applied to that event's state (per-event order, below).
- **Duplicates and older messages are ignored.** A message with `seq` ≤ the baseline is ignored and counted. It is not applied, and it is not a gap. This holds on live and sync topics alike, with two exceptions for `KIND_SNAPSHOT`. One with `seq` equal to the baseline is applied: a snapshot answer carries the latest live `seq` (below), which is the baseline itself when the Mac missed nothing. And snapshots are ordered by `map_epoch` first: one above the mirror's epoch applies at any `seq`, and one below it is ignored at any `seq`. Within one generation an answer below the baseline never carries a higher epoch, because epochs only rise with `seq`, so this cannot rewind a mirror. It lets the new generation's snapshot replace an old one still in flight after a restart (below).
- **Next.** A message with `seq` = baseline + 1 is applied under its topic's rules.
- **Gap.** A message with `seq` > baseline + 1 is a gap, handled per topic below. A snapshot is not a gap: it applies at any `seq` ≥ the baseline, and at any `seq` when its `map_epoch` is above the mirror's. It is ignored when its `map_epoch` is below the mirror's (above). It sets the baseline to its own `seq`.
- **New generation.** After a restart, `seq` starts again at 0, and the rules above would ignore the new node. The Mac recognizes a new generation in these three cases and no others:
  1. On live `map_bricks` or live `events`, a `seq` ≤ the highest `seq` already received on that live topic. Both topics are volatile (§2) and reach the Mac in publish order, so within one generation their live `seq` only rises, across reconnects too.
  2. On `status`, two consecutive messages with `seq` ≤ the baseline, the second exactly 1 above the first. The bridge can deliver its transient_local copy of the latest message again (§2), possibly more than once in a row when several clients subscribe. Those copies repeat one `seq`, so they never match. A restarted node counts up from 0, so it does. `status` runs at 1 Hz, so this catches a restart within about 2 s, even in a static room where `map_bricks` and `events` are silent.
  3. A restart signal from outside this protocol, where the deployment provides one. Phase 3 proposes `node_started_at` on the command-center API.

  On a new generation the Mac clears every topic's baseline and highest live `seq`, and every event's per-event `seq`. A kept per-event `seq` would make every message of the restarted node look older than the event's state.
  - The map goes STALE, with its last good mirror kept.
  - The event list becomes "incomplete since \<stamp of the last good message\>".
  - Both requests go out with `since = SINCE_NONE`.
  - The message that revealed the generation is then handled as the first message after a (re)connect.
  - **Messages of the old generation still in flight.** The old node's messages can still be queued in the bridge or on the network, and messages carry no generation id. The Mac does not fence them out; two properties keep them from blocking recovery.
    - **Snapshots are ordered by `map_epoch` first** (above). The node raises its epoch at every restart: it keeps the room's last `map_epoch` on disk, a node that loads saved references takes the next one at start and saves it at once, and BUILDING publishes no map or event until TRACKING bumps it (map answers, below). So the new generation's snapshots carry a higher epoch than anything the old node published. An old snapshot answer at `seq` 100 that sets the cleared baseline to 100 is replaced by the next new snapshot at any `seq`, and an old one arriving after a new one is ignored. This needs the node's state directory (`state_dir`, launch default `~/room-memory`). Without one the epoch restarts at 0, and the node must not be deployed that way.
    - **An old message that sets a baseline or a highest live `seq` above the new generation's makes the next new message match case 1 or 2 again.** The Mac then starts one more, spurious, new generation and resyncs again. There are finitely many old messages in flight, so this ends.
    - **Residual (open, D23).** An old `events` message that arrives on a sync topic after the reset is applied as an old transition the Mac missed. Its `seq` is remembered, and the new generation's message with the same `seq` can then be ignored as a duplicate. Closing that needs a generation id on the wire.
- **Across a disconnect.** Cases 1 and 2 cannot see a restart that happened while the Mac was disconnected.
  - The new generation's `seq` may already be above the old baseline.
  - The restarted node's kept messages may even include one with the saved `since_seq` and the same `map_epoch`. That is likely on `events`, which do not bump the epoch. A replay from there would continue the old history with new-generation messages.
  - So after a (re)connect the Mac keeps a baseline only if case 3 confirms that the node did not restart meanwhile. Otherwise it clears all baselines as for a new generation. Without a case-3 signal, every (re)connect requests with `SINCE_NONE`.

Applied per topic:

**`map_bricks`.** A message with `seq` ≤ the baseline is ignored under the `seq` rule. Within one generation it was published before the message the mirror last applied, so it never holds a newer state.

Any one of these makes the map STALE:
- A gap under the `seq` rule, or a new generation.
- A delta whose `base_epoch` ≠ the mirror's epoch.
- A delta as the first message after (re)connect.
- `MemoryStatus.map_epoch` ≠ the mirror's epoch in 2 consecutive status messages. This catches a lost delta that no later delta exposes (D20).

While STALE, the Mac behaves as follows.
- It renders the map greyed, labelled "resyncing", with no change overlays drawn as current.
- It sends `RequestSync{TOPIC_MAP_BRICKS, since = (mirror epoch, baseline)}`, or `since` = `SINCE_NONE` if it holds no mirror or no usable baseline (the `seq` rule). The node cannot see a (re)connect through the bridge, so the Mac does not wait for a snapshot first; a keyframe on the live topic (D29) that arrives before the answer serves as well.
- It applies, from the live topic or the sync topic, only what the live rules allow:
  - A snapshot is accepted at any `seq` ≥ the baseline, at any `seq` once the baseline is cleared, or at any `seq` with a `map_epoch` above the mirror's; never with one below it (the `seq` rule). It sets the baseline and the epoch.
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
3. **No answer** before the first live map message (BUILDING): there is no map yet, and the live snapshot that starts TRACKING ends the Mac's STALE state.

A request whose `room_id` is not the node's room gets no answer, and one that does not decode is dropped and logged. `seq` restarts at 0 with the node, so a node that loads saved references takes the next `map_epoch` (and saves it at once): a `since` from before a restart never names a message after it. The room's last epoch is kept apart from the references (`<state_dir>/<room_id>/map_epoch`), so this holds whichever sensors each run had.

**`events`.** Duplicates and older messages are ignored under the `seq` rule. A gap, or a new generation, marks the event list "incomplete since \<stamp of the last good message\>". Open events show "may be outdated". Missed transitions are never invented.
- A message after a gap is still applied: §4.2 accepts the first state seen for an `event_id` after a gap. The Mac remembers its `seq`. The baseline stays below the gap, so a replay can fill it; the baseline then moves past the remembered messages, and a replay that sends one again is ignored.
- **Per-event order.** A message above the baseline whose `seq` is below its event's per-event `seq` is older than the state the Mac shows. That is what a replay sends when it fills a gap below a message applied after it.
  - The Mac adds it to the event's history in `seq` order, and it counts toward the baseline.
  - It does not change the event's state, and it is not a protocol violation.
  - Only a message above the per-event `seq` changes the state. §4.2's transition check compares it with the state the per-event `seq` belongs to, subject to §4.2's rule for the first message about an event after a gap.
  - Example: CONFIRMED at `seq` 11 is lost, and ACCEPTED at 12 is applied after the gap, with the baseline left at 10. The replay's 11 goes into the history and the baseline moves to 12. The event stays ACCEPTED.
- **High-water mark (D26).** `MemoryStatus.events_seq` is the `seq` of the newest live `events` message. While no events request is outstanding, `events_seq` above the baseline in 2 status messages with rising `seq` is a gap. A status copy delivered twice repeats its `seq` (case 2 of the `seq` rule), so it does not count. This catches a lost event that no later event exposes, as D20 does for the map.
- The Mac sends `RequestSync{TOPIC_EVENTS, since = (map_epoch, seq) of the message at the baseline}`, or `SINCE_NONE` when it holds no usable baseline. It does the same on every (re)connect, and it repeats the request after 5 s like the map request.

The Spark keeps its last 100 `ChangeEvent` messages. It answers a `TOPIC_EVENTS` request in one of two ways:
1. **Replay** when the kept messages include the one with `seq = since_seq` and `map_epoch = since_epoch`. It republishes every message after it on `/room_memory/sync/events`, unchanged, in `seq` order: none if nothing was missed.
2. **Bounded fallback** otherwise. The Spark first publishes an `EventsFallback` header (§3.8) on `/room_memory/sync/events_fallback`. It copies the request's `since` and names every `seq` the fallback sends. Then, unchanged, in `seq` order, it publishes on `/room_memory/sync/events`:
   - the latest message of each open event (CANDIDATE or CONFIRMED) that is older than the kept messages;
   - then all kept messages.

   That is at most (open events + 100) messages after the header.

**How the Mac uses a fallback.** It applies a fallback's messages as it applies any message: those above its baseline by `event_id`, under the per-event order above, accepting the first message per `event_id` as in §4.2. Nothing in a `ChangeEvent` says which answer it belongs to, and nothing needs to, because only the header can give up a gap.
- **Giving up a gap.** The Mac moves its baseline across a `seq` it has not received only when all of these hold:
  - It has received, while its events request was outstanding, a header whose `since` is one it sent for that request, the first time or in a repeat.
  - It has received every `seq` that header names above its baseline: `resent_seqs`, and `kept_first_seq` to `kept_last_seq`, from either topic.
  - It has received a copy of `kept_last_seq` on `/room_memory/sync/events`, from any answer. The live copy does not count.

  The baseline then moves up to `kept_last_seq`, then past the messages remembered above it, as after any gap. The `seq`s it crossed without receiving them are given up, not filled. §3.8 says why no later answer can bring them, and why the sync copy of `kept_last_seq` shows that no earlier answer is still bringing them.
- **Re-checked on every arrival.** The header, the fallback's messages and the mark travel on different topics, in no fixed order. So the Mac re-checks whenever any of them arrives.
- **Nothing else gives up a gap.** It does not matter which answer arrives first, or how far the live topic or the mark has moved. Without a matching header, or before every `seq` it names and the sync copy of `kept_last_seq` have arrived, the baseline stays below the gap. The request stays open, and the 5 s repeat brings a new answer. A lost header or fallback message therefore only delays completion.
- **Example: another Mac's replay first.** A Mac at baseline 10 receives another Mac's replay starting at 12 before its own replay. It applies 12 onward after the gap and remembers them. No header matches its `since`, so 11 is not given up, and its own replay fills it (Codex, PR #13).
- **Example: a fallback still in flight.** A Mac receives the header and a fallback's first message, then live 201 and a mark of 201. The header's other `seq`s have not all arrived, so the baseline stays put and the request stays open until they do (Codex, PR #13).
- Events published after the answer arrive live, so a fallback ends even while events keep coming.
- The history stays "incomplete since \<stamp\>", because transitions older than the kept messages are gone. Events the Mac held open that were not re-sent keep "may be outdated" until a later transition for them arrives: their closing transition may be among the lost ones.

**When an events request is done.** The baseline has reached both the highest `seq` received on live `events` and the request's **mark**. The mark is the `events_seq` of the first status message received while the request is outstanding whose `seq` is above that of an earlier status also received during the request. A mark of `EVENTS_SEQ_NONE` is reached by any baseline, a cleared one included. If the baseline moved by a header during the request, the answer was a fallback, and the history stays incomplete. A request sent with `SINCE_NONE` never clears "incomplete" either: the Mac then holds no baseline that ties its history to the node's kept messages, and after a new generation the old node's missed transitions are in none of them. That includes a restarted node with no events yet, which reports `EVENTS_SEQ_NONE` and sends no header: the request is done, the history stays "incomplete since \<stamp\>", and events held open from before keep "may be outdated" until a later transition for them arrives. A Mac with no history before the request has nothing marked incomplete. Otherwise the gap is then closed and "incomplete" clears.
- **Why `events_seq`, not the live `seq`.** A Mac that disconnects at `seq` 10 and misses 11–12 may reconnect after events stop. Its highest live `seq` is still 10, equal to its baseline, so the request would seem done at once, and the Mac would stop reading the sync topic before 11–12 arrive. `events_seq` = 12 keeps the request open until they are applied.
- **Why a second, newer status.** The first status received can be the copy the bridge cached before the Mac subscribed (§2), possibly delivered more than once, so its `events_seq` may miss events published since. A status with a higher `seq` was published after that copy, so after the Mac subscribed to live `events`. Every event the Mac missed is then at or below the mark, and every later one reaches the Mac live.
- **No answer boundaries.** Neither condition depends on where an answer ends, or on which Mac it was meant for. A lost sync message leaves the baseline below the mark, so the request stays open and the 5 s repeat recovers it. Example: a Mac applies 10 from a replay, the rest of that replay is lost, and it misses live 11–12. Another Mac's replay that starts at 10 then arrives. Its 10 is ignored under the `seq` rule, and the request stays open until 11–12 are applied.

The map's high-water mark is `MemoryStatus.map_epoch`: a mirror that missed deltas holds an older epoch than `status` reports, and `status` reaches a connecting Mac at once (transient_local). So the map stays STALE until the missed deltas are applied (the condition for a current map above).

**Sync topics are shared.** Every Mac subscribed to a sync topic receives every answer. A Mac reads a sync topic only while it has a request outstanding on that topic; `/room_memory/sync/events_fallback` belongs to `TOPIC_EVENTS`. It applies only what the rules above allow. An answer meant for another Mac is therefore either one it can use or one it ignores. A header whose `since` the Mac did not send is ignored.

**Older transitions on a sync topic.** A sync message with `seq` ≤ the baseline is ignored under the `seq` rule. That includes an older snapshot answer meant for another Mac. A message above the baseline but older than its event's state, in a replay or a fallback, goes to history only under the per-event order above. Neither counts as a protocol violation.

**`status`, `activity`, `epoch`.** These are full-state messages, so a gap is only counted, and the message is applied. All three are transient_local (§2, D24). When any client subscribes, the bridge may deliver its stored copy of the latest message again, also to a Mac that already has it. The `seq` rule ignores that copy. Copies repeat one `seq`, and case 2 needs two low messages counting up by 1, so copies never look like a restart. Each is stale when its newest message is older than 3 periods: 3 s for `status` (inside PR 3.1's "stale within 5 s") and `activity`, and 15 s for `epoch`.

## 6. Honest-state rendering rules

These extend the app's existing contract (missing ≠ zero, a running container is not a running stream) to the map.

1. **UNKNOWN ≠ free.** A voxel with no `known` bit, or inside an absent or removed brick, renders hatched/transparent ("not observed"). It is never drawn as empty space, and never counted as free.
2. **Absence of returns ≠ absence of object.** DISAPPEARED comes only from the Spark, on free-space evidence: rays now pass through a formerly occupied voxel. The Mac never derives a change from missing points.
3. **`alpha = NaN` ≠ 0.** NaN renders "uncalibrated". It is never "α = 0", which would claim a perfect false-alarm level, and never hidden as if it were zero. This follows the brief's recommendation in §5 Q11.
4. **Other NaN floats are "—", not 0.** `evidence`, `coverage_known_frac`, `pose_residual_m` and `window_s` render "—" or their named phrase ("guard not run"), never 0 % or 0 m.
5. **Rates.** `0` Hz is a measured zero (the sensor is silent). `65535` is "—" (not measured).
6. **Stale is not current.** A stale topic shows "STALE · \<age\>". Its last values are never presented as live.
7. **Only CONFIRMED is a change.** CANDIDATE is shown as a candidate. REVERTED and EXPIRED never render as changes. ACCEPTED is history. Until the mirror's epoch is ≥ the event's `map_epoch`, its region is outlined "accepted · map updating" (§4.2), so the map is not taken to contain a change it does not show yet.
8. **RIG_MOVED suppresses change overlays** from the current window. The banner gives the residual.
9. **No partial application.** A message that fails to decode, or fails the delta preconditions, changes nothing on screen except the diagnostics count and the STALE state.
10. **Minimum detectable size is stated.** The view states the smallest object size the room's `voxel_size` supports, with the figure from PR 3.2's scoring (brief R5). It does not imply that smaller changes would be seen.

## 7. Byte budget

**Budget (§2.5).** ≤ 5 KB/s mean and ≤ 2 MB per (re)connect in memory mode. A snapshot targets ≤ 500 KB, with a hard cap of 2 MB.

| stream | rate | message size ("lab", default zlib) | steady state |
|---|---|---|---|
| MemoryStatus | 1 Hz | 76 B (exact) | 0.08 KB/s |
| Activity | 1 Hz | 88 B (exact) | 0.09 KB/s |
| Epoch | 0.2 Hz | 53 B (exact) | 0.01 KB/s |
| ChangeEvent, no mask | per transition, < 1/min | 136 B (exact) | < 0.01 KB/s |
| ChangeEvent with a 25 cm box mask | per transition | 253 B (measured) | < 0.01 KB/s |
| MapBricks delta, 25 cm box added | ≤ 0.2 Hz, on reference change | 367 B (measured) | < 0.1 KB/s |
| MapBricks snapshot | TRACKING start, every `keyframe_period_s` (D29), and the answer to a request that cannot be replayed | 52,863–59,024 B (measured, see below) | — |
| RequestSync (Mac → Spark) | on (re)connect and on a gap | 53 B (exact) | — |
| OperatorAction (Mac → Spark) | per operator action | 117 B (exact, for the §3.6 example) | — |
| EventsFallback | one per events fallback | 72 B with no resent `seq`, 76 + 8n B with n ≥ 1 (computed, §3.8) | — |

**Measured on the real 5-minute static recording** `static5m` (`spark/room_memory_cpp/PLAN.md`), both lidars, node defaults, the JT128's reference as the map: 0.89 KB/s mean over all room-memory topics. Tracking teaches the reference nothing (`learn_windows` 0), so there are no deltas, and Trevor's motion next to the rig is activity, not events.

| stream | messages | bytes | mean |
|---|---|---|---|
| MapBricks snapshot (keyframe, 43 KB) | 5 | 217,080 | 0.73 KB/s |
| MapBricks delta | 0 | 0 | 0 |
| MemoryStatus | 299 | 22,724 | 0.08 KB/s |
| Activity (71 of them active) | 299 | 26,312 | 0.09 KB/s |
| ChangeEvent | 0 | 0 | 0 |

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
- A (re)connect costs one snapshot plus the latest status and epoch, about 0.06 MB with zlib. The 2 MB per-(re)connect budget leaves room for the events fallback (§5), at most (open events + 100) × a few hundred bytes plus its header.

## 8. Decisions for review

Each item is a place where §2.4 was not legal ROS 2, was underspecified, or conflicted with the honest-state rules. Each states what this doc does and why. All need Codex review. Trevor decided D5, D9 and D19 on 2026-10-14; the parts of D19 marked below go beyond his decision and need his confirmation. D28–D33 come from the Spark implementation (Phase 2b, PR #14) and the static recording. D23–D27 are PR #13's.

- **D1. ROS 2 syntax.** §2.4's text was made legal ROS 2. Names, types and order are unchanged except in `MapBricks`, where D9 moved `compression` ahead of the brick arrays and packed those into `payload`. The syntax changes:
  - Constants go one per line (`KIND_SNAPSHOT=0, KIND_DELTA=1` became two lines).
  - Fields joined with `;` go one per line.
  - ROS 1 `time` became `builtin_interfaces/Time` (`int32 sec`, `uint32 nanosec`), for `stamp` as well.
  - `per_sensor_hz[2]` became `uint16[2] per_sensor_hz`.
  - "`MapBricks mask # optional`" became `MapBricks[<=1] mask`, since ROS 2 has no optional fields.
  - Named constants were added for values §2.4 described in comments: `COMPRESSION_*`, `SENSOR_*`, `HZ_UNAVAILABLE`. `RequestSync` (D19) adds `TOPIC_*` and `SINCE_NONE`. `OperatorAction` (D25) adds `ACTION_*`, `EVENT_NONE` and `STATE_NONE`. D26 adds `EVENTS_SEQ_NONE`.
- **D2. Common fields.**
  - They are `string room_id`, `uint64 map_epoch`, `uint64 seq` and `builtin_interfaces/Time stamp`, first in every message and in the brief's order.
  - They are written inline rather than as a shared header message; the wire bytes are identical either way.
  - MemoryStatus's own `map_epoch` is the common one, and Epoch's "epoch id" is `map_epoch`.
  - `RequestSync` carries them too: `map_epoch` repeats `since_epoch` and `seq` counts the client's requests (D19).
  - So does `OperatorAction`: `map_epoch` is the epoch the operator was shown, and `seq` counts the client's actions (D25).
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
- **D18. Topic names and QoS** (§2) are proposals for PR 2.4. They include the request topic and the two sync topics of D19, the operator topic of D25, the fallback-header topic of D27, and the activity topic of D33. Durability is D24.
- **D19. `RequestSync`.** Trevor decided on 2026-10-14 that the Mac sends `RequestSync{room_id, since_epoch, since_seq, stamp}`, which the Spark answers with a snapshot or the missed deltas and events. §3.5 defines the message and §5 the answer rule. These parts go beyond his field list and need his confirmation:
  - **Common fields.** `map_epoch` and `seq` were added because §1.4 puts the four common fields first in every message. `map_epoch` repeats `since_epoch`, and both directions validate that. `seq` counts the client's requests.
  - **`topic`.** `seq` is per topic, so one `since_seq` cannot describe both `map_bricks` and `events`. The Mac sends one request per stale topic.
  - **`SINCE_NONE` = 2⁶⁴ − 1** means "holds nothing". It is not 0, because epoch 0 and seq 0 are real values (§1.5).
  - **Answers go out on separate sync topics** (`/room_memory/sync/map_bricks`, `/room_memory/sync/events`, and D27's `/room_memory/sync/events_fallback` for fallback headers), not on the live topics. Replays keep their original `seq`.
    - On a live topic, other Macs would see `seq` go backwards. They could not tell a replay from a node restart, which §5 treats as a new generation (the `seq` rule, case 1).
    - With separate topics the live rules stay strict, and a Mac reads a sync topic only while its own request is outstanding.
  - **The answer is bounded.**
    - Maps: at most 9 replayed deltas, else one snapshot.
    - Events: replay from the last 100 messages, else the bounded fallback of §5, with its header (D27). That fallback leaves the history marked incomplete instead of reconstructing it.
  - **`RequestSync` adds to the Spark's own snapshots; it does not replace them.** The Spark still sends a snapshot on the live topic when TRACKING starts and then every `keyframe_period_s` (D29), so a Mac gets a map within that period without sending anything.
  - **Not verified:** whether the deployed foxglove_bridge accepts client publishing (`clientPublish`) for a custom message type. PR 2.4 / 3.1 must check it. The Spark cannot notice a (re)connect through the bridge (D29). If client publish is unavailable, the Mac cannot request. A stale map then waits for the Spark's next own snapshot, and an event gap stays open.
- **D20. Staleness.**
  - `status` is stale after 3 s and `epoch` after 15 s.
  - A missed trailing delta is caught by `MemoryStatus.map_epoch` disagreeing with the mirror twice in a row.
- **D21. `package.xml` license** says `Proprietary` because the repo has no LICENSE file. Trevor should confirm.
- **D22. Payload limit.** The uncompressed `MapBricks` payload, `134 × n_bricks + 4 × n_removed`, is at most 16 MiB (16,777,216 bytes), about 125,000 bricks or 8,000 m³ fully covered. Without a limit, a 2 MB zlib payload could declare and inflate to over 2 GB. The decoder rejects a larger declared size before inflating, and the encoder refuses to write one (D16). The measured room needs 547 KB.
- **D23. One `seq` rule, with generations** (PR #13; Phase 3's P13). §5 states it once. It needs Trevor's and Codex's review.
  - **The conflict it removes.** §5 said both "a snapshot is always accepted" and "a sync message with `seq` ≤ the last applied is ignored". After a node restart, `seq` starts again at 0 and the two collide. A Mac holding the old, higher `seq` could ignore every answer and stay STALE.
  - **Why not "accept any snapshot".** The first Phase 3 draft applied a snapshot answer whatever its `seq`. On the shared sync topic, that lets an older snapshot meant for another Mac rewind a Mac that is current. So "`seq` ≤ baseline is ignored" now holds everywhere. The only exception is a snapshot at exactly the baseline, which is the "nothing missed" answer. A restart is handled as a new generation instead.
  - **How a restart is recognized.** The live `seq` of `map_bricks` or `events` goes down (this relies on those topics staying volatile, D24). Or `status` comes in low twice in a row, counting up by 1. That works in a static room, and repeated transient_local copies cannot match it, because they repeat one `seq`. Or a signal from outside the protocol.
  - **Old messages in flight.** Clearing the baselines alone lets a message the old node published, still in flight, set a new baseline above the new node's `seq`s (Codex, PR #13). Two fences were tried and dropped. One by `stamp` fails on `map_bricks` and `events`, because replays re-send messages unchanged, `stamp` included: a new-generation event published before the restart was recognized was fenced forever, and a fallback naming it never completed. One by `map_epoch` needs an epoch known to belong to the old generation, and the messages that reveal a restart already carry the new one (Codex, PR #13). §5 instead orders snapshots by epoch first, which the node raises at every restart, and lets an old message in flight cause one more, spurious, new generation.
  - **Open: a generation id.** The residual (§5) is an old `events` message on a sync topic after the reset, whose remembered `seq` can hide the new generation's message with the same `seq`. Only a generation id in every message closes it, which changes every message's bytes. That is left to Trevor.
  - **Behaviour change on reconnect.** The Mac reuses a baseline after a disconnect only when an outside signal confirms the node did not restart. Before, it always sent its saved `since`. A restarted node could then answer with a replay of new-generation messages, as if they continued the old history. Without such a signal every reconnect now sends `SINCE_NONE`. That costs one map snapshot, as a reconnect with nothing missed already did, and the bounded events fallback, with the history marked incomplete.
  - **The events baseline** is the `seq` up to which every message has been applied, not the last one applied. A replay can then fill a gap below messages applied after it. `since` names the message at the baseline.
  - **Per-event order.** A message that fills such a gap can be older than the state its event already shows: a lost CONFIRMED at 11 arrives after ACCEPTED at 12. Applying it would be CONFIRMED after ACCEPTED, which §4.2 rejects. So the Mac keeps the `seq` of the last message applied to each event's state. An older message goes to history and advances the baseline, without changing the state. A new generation clears these `seq`s with the baselines.
- **D24. Durability** (PR #13; Phase 3's P2). It needs Trevor's and Codex's review.
  - `status`, `activity` and `epoch` are transient_local with depth 1, so a Mac shows the memory state as soon as it subscribes. Without it, the Mac would wait up to 1 s for `status` and `activity` and 5 s for `epoch`.
  - Every other topic is volatile, and says so explicitly (§2).
  - The nodes match it only for `activity` (PR #14): both publish `status` volatile. `docs/phase3-live-integration.md` §2.5 plans the change.
- **D25. `OperatorAction`** (PR #13; Phase 3's P6). It needs Trevor's and Codex's review. §3.6 defines it, with `ros/labcommand_msgs/msg/OperatorAction.msg`.
  - The actions asked for (acknowledge, dismiss, re-baseline, label) and Phase 3's earlier list map onto the existing transitions. Acknowledge and label are log-only. Dismiss is §4.2's "operator revert". Re-baseline is §4.1's "operator re-references the room". Pause and resume are §4.1's operator pause and resume. Accept is §4.2's "operator accept". Mark ground truth is for PR 3.2's scoring.
  - The common fields serve as the header, not `std_msgs/Header` (D2).
  - `operator_id` is self-asserted. Neither the bridge's client-publish path nor Phase 3's ssh relay can tell which person sent an action.
  - **ACCEPTED follows its map message** (§4.2). If ACCEPTED went out first, every Mac would remove the overlay while its map still lacked the change. A delta can wait 5 s for the 0.2 Hz slot (§2), and longer behind Phase 3's byte bucket (`docs/phase3-live-integration.md` §3.3). So the node publishes ACCEPTED right after the message that carries the change. The Mac also removes the overlay only once its mirror's epoch is ≥ the event's `map_epoch`. `expected_state` is compared with the node's own state, so an accept still waiting for its map message cannot also be dismissed.
  - Open: re-referencing outside RIG_MOVED, and showing labels and acknowledgements on other Macs. Both would change §4.1 or `ChangeEvent`.
- **D26. Events high-water mark** (PR #13; Phase 3's P16). It needs Trevor's and Codex's review. §5 defines it, and §3.3 adds `uint64 events_seq` to `MemoryStatus`. That changes `MemoryStatus` bytes: 8 more per message.
  - **`events_seq` is the newest events `seq`.** Before this, a replay was complete once it reached the highest live `seq` the Mac had seen. A Mac that missed the last events before a quiet period has seen none of them on the live topic, so its request was done at once and the history stayed wrongly complete. The map already had its high-water mark in `map_epoch`.
  - **Not an in-band end marker.** An earlier draft ended every answer with its last message sent again. Sync topics are shared, and the repeat was an ordinary `ChangeEvent`. So after a lost marker, another answer starting at the same `seq` read as the marker, and the request closed before the missed events arrived (Codex, PR #13). `events_seq` is not part of any answer. A lost message only delays completion.
  - **The mark decides when a request is done, not whether a gap is given up.** That takes the fallback header (D27).
- **D27. Events fallback header** (PR #13, after Codex's review of it; Phase 3's P17). It needs Trevor's and Codex's review. §3.8 defines `EventsFallback` and its topic, with `ros/labcommand_msgs/msg/EventsFallback.msg`,, and §5 the rule. It adds one message type and one topic.
  - **Why an explicit header.** The previous draft recognized a fallback by its first message not being the `since` message, which a replay re-sent first. On the shared sync topic, another Mac's replay can arrive first, and the `since` message can be lost. Either read as a fallback, and the Mac gave up messages that its own replay was still bringing (Codex, PR #13). The `since` message is no longer re-sent.
  - **Why it names the `seq`s.** A fallback's contents are sparse: open events older than the kept messages, then the kept range. Reaching the mark on the live topic does not show they have arrived. The previous draft closed a fallback while its messages were still in flight, and ignored them afterwards (Codex, PR #13). With the list, the Mac gives up only `seq`s the Spark no longer holds, and only after receiving every one it does.
  - **Why it also waits for `kept_last_seq` on the sync topic.** Headers travel on their own topic, so another Mac's header can arrive while the Mac's own earlier answer is still in flight with a `seq` that header omits. The sync topic keeps publish order, so its copy of `kept_last_seq` arrives after that earlier answer (§3.8).
  - **Why keyed by `since`, not by client.** `RequestSync` carries no client identity, and adding one would change its bytes. The header states what the Spark holds, not who asked, so a header for the same `since` is valid for every Mac that sent it.
  - **Why not in band.** `ChangeEvent`s are re-sent unchanged, and an ordinary `ChangeEvent` cannot mark an answer: D26's lost-marker case.
  - **Why not the reply to the `:8766` POST** (Phase 3's P11). The protocol stays independent of the request transport, and bridge client publishing has no reply. A fallback can also be deferred past the request (Phase 3's P4).
  - Replays and map answers need no header. A replay gives up nothing, and a snapshot says what it is (`KIND_SNAPSHOT`).
- **D28. A dedicated, read-only bridge on :8767** (§2; a proposal: the transport is open, Q5). The memory topics need `labcommand_msgs` in the bridge's environment, which the lidar bridge on :8765 does not have, and a separate bridge leaves LIVE untouched. Read-only by default because foxglove_bridge 3.4.1 does not enforce `client_topic_whitelist`. `RequestSync` then needs `foxglove_client_publish:=true` or a relay.
- **D29. Periodic keyframes.** §2.4's "snapshot on (re)connect" cannot be detected through the bridge, so the live topic carries a snapshot every `keyframe_period_s` (60 s). Changed bricks between keyframes go out as deltas at most every `delta_period_s` (5 s, the ≤ 0.2 Hz of §2). A snapshot replaces the next delta once the deltas since the last snapshot would exceed `keyframe_delta_ratio` (0.5) of its size. A keyframe of an unchanged map does not bump `map_epoch`; every delta does.
- **D30. One published map.** There is no JT128↔Mid-360 extrinsic yet (brief R1), so the references stay per sensor and `map_bricks` carries one of them, `map_sensor` (default: the JT128 when configured; its repeating pattern gives the denser map). `n_bricks` and `coverage_known_frac` describe that map. Events of the other sensor are in that sensor's own frame.
- **D31. A persistence gate before CANDIDATE (strengthened by D33).** On the static recording the node sent ~120 CANDIDATE/EXPIRED messages per minute. Most of them came from a reference built as the union of 5 windows of the Mid-360's sparse non-repetitive pattern; per-voxel window counts fix that (`spark/room_memory_cpp/PLAN.md`). As a second line, a region now has to persist `candidate_windows` windows before its CANDIDATE is sent. An open candidate expires only after `expire_windows` unmatched windows, so an intermittently seen change does not flicker. T_commit (CANDIDATE → CONFIRMED) is unchanged and still PR 2.3.
- **D32. No events in the near field (superseded by D33: removed).** The rig stands about 1.3 m from where someone works. Live and in both static recordings, something low there (z up to ~0.8 m in the sensor frames) shifts every few seconds to minutes, seen by both lidars at once, and it was 100 % of the remaining events (live 2–18 per minute). A change region with a voxel center within `near_range_m` (2.0 m) of its sensor is not reported, whichever pose of a move it is. With it, the diff's `tolerance` 2 and `min_voxels` 8 remove the far fragments of the same motion. This is a blind zone, not noise removal: a real change within 2 m of either lidar is not reported. 0.3 m cubes from 2.2 m out are still detected at the same latency (`spark/room_memory_cpp/PLAN.md`, "False change events"). *Removed:* the "something low" was Trevor working at a desk (he confirmed it), real transient motion, and a blind zone also hid real changes next to the rig. `near_range_m` is gone; `tolerance` and `min_voxels` are back at 1 and 6. D33 replaces it.
- **D33. Activity is not change.** Motion next to the rig (a person working) is reported on `/room_memory/activity` (§3.7), never as a `ChangeEvent`. A region becomes a CANDIDATE only after it has stayed settled for `candidate_windows` windows (§4.2): fresh, the same shape, and no activity within `activity_radius_m`; so a change made by a person is reported that long after they stop moving next to it. `candidate_windows` is 40 (40 s at `window_s` 1): on both static recordings, looped for 30 minutes, the longest settled run of anything Trevor left is 24 windows (the part of his BUILDING-time pose he moved away from). 0.3 m boxes 1.0–4.6 m from either lidar are CANDIDATEs 40–43 s after they appear, disappear or move. Tracking teaches the reference nothing (`learn_windows` 0): learning while he worked wrote his surroundings into the reference and produced a 45-window still residue. Limits: a person who stays perfectly still for longer than the gate after shifting is reported, and so is a reference built while someone sat in it once they leave; without the JT128↔Mid-360 extrinsic (D30) one sensor's activity cannot hold the other's regions. The Mac does not decode `Activity` yet.

## 9. Verification status

- **Hand goldens.** `Tests/Fixtures/room/golden/hand/*.hex` were written byte by byte from these rules, with every span annotated. There are 12: 9 little endian and 3 big endian, covering the five messages of §3.1–§3.5 (`OperatorAction` and `EventsFallback` have none yet). `spark/room_memory/tests/fixtures/hand` adds two `Activity` goldens (active, and idle with NaN geometry), checked by the Python and C++ codecs only; the Swift suite does not decode `Activity` yet. The tests check:
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
- **`OperatorAction` (§3.6, D25): the `.msg` only.** There is no Swift codec and no hand golden yet. The 117 B wire size was computed by hand from §1.1, not encoded. Codecs and goldens are Phase 3 stage S5.
- **`EventsFallback` (§3.8, D27): the `.msg` only.** There is no Swift codec, hand golden or node support yet. The wire size was computed by hand from §1.1, not encoded.
- **The Spark side of §5 is implemented, except PR #13's additions to it,** in `spark/room_memory_cpp` (the node) and in the Python oracle `spark/room_memory`. The differential tests check the two byte for byte: the map log and replay, sync snapshots, event replay and the bounded fallback, the 100-message log and the room and decode checks. Not implemented: the `EventsFallback` header before a fallback (D27), and `status` and `epoch` as transient_local (D24); `docs/phase3-live-integration.md` plans both. `RoomMemoryStore` (the Mac side) is PR 1.1. The bridge's client-publish path has not been tried against a live bridge.

## 10. Open questions for Trevor (brief §5, Q1–Q4, and Q5)

Q1–Q4 are copied from the brief and left unanswered. Q5 was added by PR #14.

1. **Is the rig static for v1?** If it will move between sessions, relocalization (and place recognition) moves from optional to required.
2. **Room and targets:** room size, smallest object that must be detected (5 cm? 10 cm?), acceptable latency (seconds vs minutes), acceptable false events per hour. These set voxel size, T_commit and the PR 3.2 bar.
3. **HDC's role.** Your own record (exp119/122/123, exp113, D-33) says a superposed memory is not a cheaper map, and HDC is parked. Should the app be HDC-free with HDC kept only as an optional RT3 comparison, or do you want to reopen HDC? This brief assumes the former.
4. **Fixtures:** may the 12 Oct box takes (`*ROBOT_box_*`, `*speedybee_box_*` on the Spark) be copied to the Mac as golden fixtures? This brief did not access the Spark.
5. **Transport (D28).** A dedicated foxglove_bridge on :8767 in the room-memory container (requests through its client publishing or a relay), or the existing bridge on :8765 with requests relayed over the ssh-authenticated command-center API on :8766? Both stay in this doc until you choose (§2).
