# labcommand_msgs/RigPose: the rig-pose guard (protocol §3.8, D34-D38), on /room_memory/rig.
# Reliable, transient_local, keep-last 1; published on every outcome change and every 10 s.
# Poses map each sensor's current frame into its reference frame: p_ref = R p + t.
uint8 OUTCOME_IN_PLACE=0          # last check accepted the current pose
uint8 OUTCOME_CHECKING=1          # a trigger fired; promotion and learning held
uint8 OUTCOME_MOVED=2             # move evidence; changes suspended (MemoryStatus RIG_MOVED); estimating
uint8 OUTCOME_REALIGNED=3         # new pose applied for every moved sensor; same room frame
uint8 OUTCOME_RELEARNING=4        # confirmed move not alignable; state archived and rebuilt; new room frame
uint8 OUTCOME_INCONCLUSIVE=5      # alignment rejected and no move evidence; pose kept; never relearns

uint8 CAUSE_IMU_SHOCK=1
uint8 CAUSE_IMU_TILT=2
uint8 CAUSE_POSE_OFFSET=4
uint8 CAUSE_PERIODIC=8
uint8 CAUSE_STARTUP=16
uint8 CAUSE_SENSOR_RESUMED=32

uint8 SENSOR_MID360=1
uint8 SENSOR_JT128=2

uint8 CHANNEL_NONE=0              # not checked in this decision
uint8 CHANNEL_IN_PLACE=1
uint8 CHANNEL_MOVED=2
uint8 CHANNEL_REJECTED=3
uint8 CHANNEL_UNAVAILABLE=4

string room_id
uint64 map_epoch
uint64 seq
builtin_interfaces/Time stamp

uint8 outcome                     # OUTCOME_*
uint8 causes                      # bitmask of CAUSE_*: what started the current or last check; 0 = none yet
uint8 sensors                     # bitmask of SENSOR_*: sensors available in the decision this message reports
uint8[2] channel                  # CHANNEL_* per sensor in that decision, [0] Mid-360 [1] JT128
uint64 frame_epoch                # map_epoch at which the current room frame began (changes only on relearn)
uint32 n_realigned                # REALIGNED decisions since the frame began
float32[7] mid360_pose            # tx ty tz (m), qx qy qz qw: Mid-360 frame -> its reference frame; NaN x7 = not configured
float32[7] jt128_pose             # same, JT128
float32[2] delta_m                # |t| of the increment of the last applied estimate, [0] Mid-360 [1] JT128; NaN = none
float32[2] delta_deg              # rotation angle of that increment; NaN = none
float32[2] inlier_frac            # last check, 0..1; NaN = not run or no registration cloud
float32[2] rms_m                  # last check; NaN = not run
float32[2] observe                # last check, lambda_min(H)/sum(w) (§3.8); NaN = not run
float32 tilt_deg                  # IMU gravity vs the gravity baseline of the last accepted decision; NaN = no IMU
bool imu_ok                       # IMU samples within the last second
builtin_interfaces/Time moved_at  # first trigger of the current or last move; {0, 0} = none
