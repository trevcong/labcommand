# labcommand_msgs/Activity: transient motion next to the rig (people, things being handled), 1 Hz.
# Never a change: activity does not become a ChangeEvent. A ChangeEvent CANDIDATE needs a region to stay
# settled for candidate_windows windows with no activity next to it.
# Contract: docs/ROOM-MEMORY-PROTOCOL.md in the LabCommand repo (section 3.7, D33).

uint8 SENSOR_MID360=1             # bit 0 of sensors
uint8 SENSOR_JT128=2              # bit 1 of sensors

# Common fields, first in every labcommand_msgs message.
string room_id
uint64 map_epoch
uint64 seq                        # per-topic counter, +1 per message
builtin_interfaces/Time stamp

bool active                       # some tracking window since the previous Activity message had activity
uint8 sensors                     # bitmask of SENSOR_*: sensors with activity in the newest such window; 0 when inactive
uint8 largest_sensor              # SENSOR_* bit whose frame centroid and aabb are in; 0 when inactive
uint32 n_regions                  # activity regions in that window, all sensors; 0 when inactive
uint32 voxel_count                # their voxels; 0 when inactive
float32[3] centroid               # m, the largest activity region, frame of largest_sensor; NaN x3 when inactive
float32[6] aabb                   # m, its box: min_x, min_y, min_z, max_x, max_y, max_z; NaN x6 when inactive
float32 since_s                   # s since the newest window with activity; NaN = none since the node started
