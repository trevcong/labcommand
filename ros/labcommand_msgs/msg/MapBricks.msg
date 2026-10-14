# labcommand_msgs/MapBricks: the room's voxel memory as 8x8x8-voxel bricks.
# A full snapshot (node start, (re)connect, or the answer to a RequestSync) or a delta (since base_epoch).
# Contract: docs/ROOM-MEMORY-PROTOCOL.md in the LabCommand repo.

uint8 KIND_SNAPSHOT=0
uint8 KIND_DELTA=1

uint8 COMPRESSION_NONE=0
uint8 COMPRESSION_ZLIB=1          # raw DEFLATE (RFC 1951, no zlib header): Python zlib.decompress(payload, -15)
uint8 COMPRESSION_LZFSE=2         # Apple LZFSE

# Common fields, first in every labcommand_msgs message.
string room_id                    # room this memory belongs to
uint64 map_epoch                  # reference-map epoch this message describes (after a delta is applied)
uint64 seq                        # per-topic counter, +1 per message; 0 inside ChangeEvent.mask
builtin_interfaces/Time stamp     # when the Spark produced this message

uint8 compression                 # COMPRESSION_*: how payload is encoded; ZLIB by default
uint8 kind                        # KIND_SNAPSHOT or KIND_DELTA
uint64 base_epoch                 # delta applies only if the Mac holds exactly this epoch; 0 in a snapshot
float32 voxel_size                # m, edge of one voxel; 0.05 by default, fixed per room
uint8 brick_dim                   # voxels per brick edge; 8 -> 8x8x8 = 512 voxels per brick
uint32 n_bricks                   # bricks in payload
uint32 n_removed                  # removed brick indices in payload (delta only)
# payload before compression is exactly 134 * n_bricks + 4 * n_removed bytes (at most 16 MiB),
# packed little endian whatever the CDR byte order, with no padding:
#   int16  brick_ijk[3 * n_bricks]     brick coords (int16 x 0.4 m brick = about +-13 km), strictly increasing (i, j, k)
#   uint8  occ_bits[64 * n_bricks]     per brick: bit = occupied
#   uint8  known_bits[64 * n_bricks]   per brick: bit = observed (occupied or free); absent bit = UNKNOWN
#   uint32 removed_bricks_idx[n_removed]  indices into the base_epoch brick list of bricks that became all-unknown
uint8[] payload
