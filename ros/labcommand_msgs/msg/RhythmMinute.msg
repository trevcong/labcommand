# labcommand_msgs/RhythmMinute: one minute of per-lidar region occupancy.
# Topic /room_memory/rhythm. sensor is the file-header code and sits immediately after room_id.
# 1 Mid-360, 2 JT128. Flag bits: 0 partial, 1 discontinuity, 2 unavailable, 3 overflow, 4 lost.

uint8 SENSOR_MID360=1
uint8 SENSOR_JT128=2

string room_id
uint8 sensor
uint64 map_epoch
uint64 seq
builtin_interfaces/Time stamp

uint16 windows_closed
uint16 windows_with_points
uint16 windows_unavailable
uint32 clipped
uint32 regions_dropped
uint8 pose_ok
uint8 flags
uint16 n_regions
int16[] ix
int16[] iy
int16[] iz
uint16[] hit_windows
uint16[] mean_voxels
# Protocol v2 (section 1.4). The hand codec omits this when absent (the legacy layout).
# Never 0. The live publisher always writes the process generation.
uint64 publisher_generation
