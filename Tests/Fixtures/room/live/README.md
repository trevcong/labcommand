# Live room-memory captures (Spark bridge `ws://100.118.187.99:8767`, 2026-10-15)

Raw CDR payloads (the Foxglove message-data frame minus its 13-byte header), captured by a
read-only subscriber: it never advertised a channel or published. foxglove-bridge 3.6.0,
`capabilities: ["connectionGraph"]` (no `clientPublish`).

| file | topic | what |
|---|---|---|
| `status_events_seq.bin` | `/room_memory/status` | TRACKING, epoch 2399, 1977 bricks, `events_seq` 6790 |
| `status.bin` | `/room_memory/status` | after a node restart: `events_seq` = EVENTS_SEQ_NONE |
| `map_snapshot.bin` | `/room_memory/map_bricks` | zlib snapshot, 43 KB on the wire |
| `map_delta_1brick.bin` | `/room_memory/map_bricks` | zlib delta 2399 → 2400, one brick |

Both statuses are 76 bytes: they carry Phase 3's `events_seq`, which main's Phase 0 codec rejected
as 8 trailing bytes. `LiveRoomTests.swift` decodes them all.

**Legacy layout.** The four captures predate publisher generations (protocol §5, D34): no message
carries `publisher_generation`. They stay as legacy decode fixtures, and are not re-captured. The
Mac decodes them in legacy mode (PLAN-PUBGEN §4.6): identity unverified, the map never current,
event history `unknown`, and no `RequestSync` sent. `status.bin`'s `EVENTS_SEQ_NONE` is not
restart evidence (D37).
