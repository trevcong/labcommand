# Hand CDR goldens: provenance

Copied byte for byte from `Tests/Fixtures/room/golden/hand/` on the Phase 0 branch (`labcommand-memory` worktree), commit `95f49f2` ("Apply D5/D9/D19: keep all-free bricks, header compression (zlib default), RequestSync message"). They were derived by hand from `ros/labcommand_msgs/msg/*.msg` and `docs/ROOM-MEMORY-PROTOCOL.md` §1; every span is annotated. `memory_status_le.hex` was refreshed from `phase3-design` (PR #13) commit `d751db6` for `events_seq`. `activity_le.hex` and `activity_idle_le.hex` are not copies: they were derived by hand on `room-memory-cpp` from `ros/labcommand_msgs/msg/Activity.msg` (protocol §3.6) in the same annotated format, and the Swift suite has no copy yet (it does not decode `Activity`). Format: whitespace-separated hex bytes, `#` starts a comment.

| file | sha256 |
|---|---|
| `activity_idle_le.hex` | `e389eb520462842584988251bef696f1388f94814e5326a2e6ff344d09171db1` |
| `activity_le.hex` | `1e26a83b70b21a831c34a9e2cc5b8e21bf9e157df6f1b75ab5ea836bc2190d79` |
| `change_event_appeared_le.hex` | `3b102fc7bff4788d702d456ec60a168a10a23b9b92559f8a27df50b7463f0491` |
| `change_event_moved_mask_le.hex` | `f486818b3d6df538ea17e95de3bc23db59934241c37c850d160604ca130f9905` |
| `epoch_be.hex` | `c2b121f7156c09f1126bb94c352b826ec1bd01e71f01f081e52ead257c3599c6` |
| `epoch_le.hex` | `5eb7544d8e8cbc2a6e92f770e1dcdbbde74b7af3359e71febcac03ebf303dd8b` |
| `map_bricks_delta_be.hex` | `3ea657d2000708eb5dae81456ad802584544e1980ffbb40b462050e54a256054` |
| `map_bricks_delta_le.hex` | `1b48dafbd91316e4a0386fe9202d3f80fd2721177ba3635edd191034e7c6eb3a` |
| `map_bricks_delta_lzfse_le.hex` | `ddceca55f37a2c65263f44496138e5a062be4c5a04f3dbad4356799d63839280` |
| `map_bricks_snapshot_zlib_le.hex` | `9b80f3d96469b5a4d85f39ba4e42022d055a00eac86ce94cff8b098472e85f73` |
| `memory_status_le.hex` | `39d08f9bed9a80d4cabffcd034307a4e7909de30ee8a925ea245d5783c8246b5` |
| `request_sync_events_none_le.hex` | `fdd4ee58eb7753752a11a66079f9f36f4e67c292aae742b697b406c454e3762b` |
| `request_sync_map_be.hex` | `a301146eb9f9cf0512d1c679fb270a1c6ec95d88319ad87dc9b13c594b0503a2` |
| `request_sync_map_le.hex` | `8d1da54561d93a617fbaf87b5cc450cd47763a765dcb6a8ad191f467d9a7664f` |

`test_messages.py` checks these hashes, so a silent edit here fails the suite. To refresh after a protocol change, copy the files again and update this table.
