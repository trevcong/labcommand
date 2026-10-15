# labcommand_msgs/MemoryStatus: room-memory health, 1 Hz.
# Contract: docs/ROOM-MEMORY-PROTOCOL.md in the LabCommand repo.

uint8 STATE_BUILDING=0
uint8 STATE_TRACKING=1
uint8 STATE_RIG_MOVED=2
uint8 STATE_DEGRADED=3
uint8 STATE_PAUSED=4

uint16 HZ_UNAVAILABLE=65535       # per_sensor_hz value when the rate was not measured
uint64 EVENTS_SEQ_NONE=18446744073709551615  # events_seq before the first event is published

# Common fields, first in every labcommand_msgs message.
string room_id
uint64 map_epoch
uint64 seq
builtin_interfaces/Time stamp

uint8 state                       # STATE_*
uint32 n_bricks                   # bricks in the reference map
uint32 n_open_events              # events in CANDIDATE or CONFIRMED
float32 coverage_known_frac       # 0..1, fraction of room volume observed; NaN = room volume not yet known
float32 pose_residual_m           # m, last rig-pose guard residual; NaN = guard has not run
float32 window_s                  # s, accumulation window length; NaN = not reported
uint16[2] per_sensor_hz           # Hz, sensor frames received per second, rounded: [0] Mid-360, [1] JT128; HZ_UNAVAILABLE = not measured
uint32 dropped_windows            # windows dropped since the node started
uint64 events_seq                 # seq of the newest live events message (high-water mark, section 5), or EVENTS_SEQ_NONE
