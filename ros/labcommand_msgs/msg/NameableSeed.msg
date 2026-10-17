# labcommand_msgs/NameableSeed: an accepted APPEARED the operator can still name.
# Contract: docs/ROOM-MEMORY-PROTOCOL.md section 3.15.

uint8 SENSOR_MID360=1
uint8 SENSOR_JT128=2

uint64 event_generation          # never 0
uint64 event_id                  # never 0
uint8 sensor                     # one SENSOR_* bit; a seed does not cross sensors
float32[3] centroid              # m, room frame
float32[6] aabb                  # m: min_x, min_y, min_z, max_x, max_y, max_z
uint32 voxel_count               # voxels in the seed; at least 1
