# labcommand_msgs/MapBricks: the room's voxel memory as 8x8x8-voxel bricks.
# A full snapshot (on connect / resync) or a delta (since base_epoch).
# Contract: docs/ROOM-MEMORY-PROTOCOL.md in the LabCommand repo.

uint8 KIND_SNAPSHOT=0
uint8 KIND_DELTA=1

uint8 COMPRESSION_NONE=0
uint8 COMPRESSION_ZSTD=1

# Common fields, first in every labcommand_msgs message.
string room_id                    # room this memory belongs to
uint64 map_epoch                  # reference-map epoch this message describes (after a delta is applied)
uint64 seq                        # per-topic counter, +1 per message; 0 inside ChangeEvent.mask
builtin_interfaces/Time stamp     # when the Spark produced this message

uint8 kind                        # KIND_SNAPSHOT or KIND_DELTA
uint64 base_epoch                 # delta applies only if the Mac holds exactly this epoch; 0 in a snapshot
float32 voxel_size                # m, edge of one voxel; 0.05 by default, fixed per room
uint8 brick_dim                   # voxels per brick edge; 8 -> 8x8x8 = 512 voxels per brick
int16[] brick_ijk                 # 3 per brick (brick coords; int16 x 0.4 m brick = about +-13 km), strictly increasing (i, j, k)
uint8[] occ_bits                  # 64 B per brick: bit = occupied
uint8[] known_bits                # 64 B per brick: bit = observed (occupied or free); absent bit = UNKNOWN
uint32[] removed_bricks_idx       # delta only: indices into the base_epoch brick list of bricks that became all-unknown
uint8 compression                 # COMPRESSION_NONE; COMPRESSION_ZSTD is reserved and rejected by v1 decoders
