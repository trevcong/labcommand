# Hand CDR goldens: provenance

Copied byte for byte from `Tests/Fixtures/room/golden/hand/` on the Phase 0 branch (`labcommand-memory` worktree), commit `95f49f2` ("Apply D5/D9/D19: keep all-free bricks, header compression (zlib default), RequestSync message"). They were derived by hand from `ros/labcommand_msgs/msg/*.msg` and `docs/ROOM-MEMORY-PROTOCOL.md` §1; every span is annotated. `memory_status_le.hex` was refreshed from `phase3-design` (PR #13) commit `d751db6` for `events_seq`. `activity_le.hex` and `activity_idle_le.hex` are not copies: they were derived by hand on `room-memory-cpp` from `ros/labcommand_msgs/msg/Activity.msg` (protocol §3.7) in the same annotated format, and copied to `Tests/Fixtures/room/golden/hand/` on `protocol-v2-followups`, when the Swift suite began decoding `Activity`. The protocol v2 goldens (`*_v2_*.hex`, `sync_end_*.hex`) were derived by hand on `pubgen` from the v2 `.msg` files and `PLAN-PUBGEN.md` §2.2 (`activity_v2_le.hex`, `epoch_v2_le.hex` and `epoch_v2_be.hex` on `protocol-v2-followups`, protocol D38), in the same annotated format, and copied to `Tests/Fixtures/room/golden/hand/`; each v2 golden that extends a legacy one repeats its values and appends the new fields, and every v2 golden uses the one publisher generation 1760400000000. `rig_pose_le.hex` and `imu_le.hex` were derived by hand on `rig-guard` the same way, from `ros/labcommand_msgs/msg/RigPose.msg` (protocol §3.9) and ROS 2 Jazzy's `sensor_msgs/msg/Imu.msg`; the Swift suite has no copy of them (it decodes neither). Format: whitespace-separated hex bytes, `#` starts a comment.

| file | sha256 |
|---|---|
| `activity_idle_le.hex` | `e389eb520462842584988251bef696f1388f94814e5326a2e6ff344d09171db1` |
| `activity_le.hex` | `1e26a83b70b21a831c34a9e2cc5b8e21bf9e157df6f1b75ab5ea836bc2190d79` |
| `activity_v2_le.hex` | `47d460c031c6b2c6c20f5b52b1c03f470dac7b93a8da25a3290346c0601f4fed` |
| `change_event_appeared_le.hex` | `3b102fc7bff4788d702d456ec60a168a10a23b9b92559f8a27df50b7463f0491` |
| `change_event_appeared_v2_le.hex` | `68fd743260fb137355e5da6685fe5ab5558b82fc63f7b6981801f18ca6e6e93a` |
| `change_event_moved_mask_le.hex` | `f486818b3d6df538ea17e95de3bc23db59934241c37c850d160604ca130f9905` |
| `change_event_moved_mask_v2_le.hex` | `600c2d39487a7173c6f39bca00830c2c28d27c1adc165b61679c42695c951f71` |
| `epoch_be.hex` | `c2b121f7156c09f1126bb94c352b826ec1bd01e71f01f081e52ead257c3599c6` |
| `epoch_le.hex` | `5eb7544d8e8cbc2a6e92f770e1dcdbbde74b7af3359e71febcac03ebf303dd8b` |
| `epoch_v2_be.hex` | `5e837b817325d0700747bb2e8a9e714d839b3db876c2d28eadce7457ecd1d2ef` |
| `epoch_v2_le.hex` | `ac7d6e31b97a52585093f1b0a2895a50894e1e14b407623e62535995994e4bef` |
| `imu_le.hex` | `4d396f417dbb6f9d0c8c0f28a15bf633c82382b05f82fd730e38417a396e52ac` |
| `map_bricks_delta_be.hex` | `3ea657d2000708eb5dae81456ad802584544e1980ffbb40b462050e54a256054` |
| `map_bricks_delta_le.hex` | `1b48dafbd91316e4a0386fe9202d3f80fd2721177ba3635edd191034e7c6eb3a` |
| `map_bricks_delta_lzfse_le.hex` | `ddceca55f37a2c65263f44496138e5a062be4c5a04f3dbad4356799d63839280` |
| `map_bricks_delta_v2_be.hex` | `85d16e83aa2f54458862930eb0208a5486a76e3a74a2e7cbfdcc0a839ddedf54` |
| `map_bricks_delta_v2_le.hex` | `d1c38a7bf54f51d2288ec2ee8ea84ff540230bef660d4e5fcbee07f24fa51b4e` |
| `map_bricks_snapshot_zlib_le.hex` | `9b80f3d96469b5a4d85f39ba4e42022d055a00eac86ce94cff8b098472e85f73` |
| `memory_status_le.hex` | `39d08f9bed9a80d4cabffcd034307a4e7909de30ee8a925ea245d5783c8246b5` |
| `memory_status_v2_le.hex` | `b9cc20dc39af195efdd3f4a8629312a1d39a71bea31aff6de4b0c504bf23b366` |
| `request_sync_events_none_le.hex` | `fdd4ee58eb7753752a11a66079f9f36f4e67c292aae742b697b406c454e3762b` |
| `request_sync_events_v2_le.hex` | `ddd4237dd608d76234fe024568aed0d281ffbe986ccf070890c213f072bd518d` |
| `request_sync_map_be.hex` | `a301146eb9f9cf0512d1c679fb270a1c6ec95d88319ad87dc9b13c594b0503a2` |
| `request_sync_map_le.hex` | `8d1da54561d93a617fbaf87b5cc450cd47763a765dcb6a8ad191f467d9a7664f` |
| `request_sync_map_v2_be.hex` | `38afaeb4b32705afb668f3039681588937fb77f4b32fb128ece6eb6e94d7dcd1` |
| `rig_pose_le.hex` | `b0c027d497f428669c4d98da8d57e43d3b52a7708316b9ec7be65cd5f3a788ec` |
| `sync_end_events_empty_le.hex` | `56c5c18649f8d2e8df47493fff5e43f9e361cd30b5e97ac412fc10146662f2c9` |
| `sync_end_events_fallback_le.hex` | `2b6431a0a57e5a0f80be40617c8acbdd2cfef35d3edda99d17e9e921a6616439` |
| `sync_end_map_replay_be.hex` | `8d4f12e1ada741ff15e817847e30132bcfd0e7704ea1f406ac5599060e293d27` |

`test_messages.py` checks these hashes, so a silent edit here fails the suite. To refresh after a protocol change, copy the files again and update this table.
