# labcommand_msgs/ChangeEvent: one message per lifecycle transition of one change.
# Contract: docs/ROOM-MEMORY-PROTOCOL.md in the LabCommand repo.

uint8 KIND_APPEARED=0
uint8 KIND_DISAPPEARED=1
uint8 KIND_MOVED=2

uint8 STATE_CANDIDATE=0
uint8 STATE_CONFIRMED=1
uint8 STATE_ACCEPTED=2
uint8 STATE_REVERTED=3
uint8 STATE_EXPIRED=4

uint8 SENSOR_MID360=1             # bit 0 of sensors
uint8 SENSOR_JT128=2              # bit 1 of sensors

# Common fields, first in every labcommand_msgs message.
string room_id
uint64 map_epoch
uint64 seq
builtin_interfaces/Time stamp

uint64 event_id                   # unique per room; every transition of one change repeats it
uint8 kind                        # KIND_*
uint8 state                       # STATE_*
float32[3] centroid               # m, room frame
float32[6] aabb                   # m, room frame: min_x, min_y, min_z, max_x, max_y, max_z
float32[3] from_centroid          # m, room frame; MOVED only, NaN x3 otherwise
uint32 voxel_count                # voxels in the region
float32 volume_m3                 # m^3, voxel_count * voxel_size^3
float32 evidence                  # 0..1, fraction of the region's voxels with free/occupied evidence; NaN = not computed
float32 alpha                     # 0..1, calibrated false-alarm level if the research layer provides one, else NaN (rendered "uncalibrated")
uint8 sensors                     # bitmask of SENSOR_*: which sensor(s) observed it
builtin_interfaces/Time first_seen
builtin_interfaces/Time last_seen
MapBricks[<=1] mask               # optional: the region's voxels only (usually < 1 KB with zlib)
