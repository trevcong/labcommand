# Recorded Mid-360 schema

`mid360_recorded_custommsg.ros2msg` is the exact embedded `ros2msg` schema
for `livox_ros_driver2/msg/CustomMsg` from
`~/Developer/spatio-data/recordings/session_20260808_211213/bag/bag_0.mcap`.
Extracted locally on 2026-10-19 under `.spatio-bigio.lock`, without copying
point data. SHA-256:
`e1955256d7a20d899b742d06f929cba135a3393edd3a69cc3cb6dc762ba53230`.

The root and CustomPoint declarations match the copied driver `.msg` files
after comments and whitespace are removed. The real schema has repeated
spaces in `uint8  lidar_id` and `uint8[3]  rsvd`, and an unqualified
`CustomPoint[]`. The original synthetic fixture used single spaces and a
qualified `livox_ros_driver2/CustomPoint[]`. Neither difference changes the
wire layout. Tests retain the recorded text verbatim and reject changed
field types, names, order and array sizes.

Units come from the copied publisher/consumer source, not whitespace or
numeric magnitude: `livox_ros_driver2/src/lddc.cpp:367-395` assigns timebase
to the ROS stamp and subtracts it for point offsets;
`FAST_LIO_ROS2/src/preprocess.cpp:180-181` divides offsets by 1e6 to obtain
milliseconds. This establishes nanoseconds for the supported source
contract; publisher clock selection and physical synchronization remain
unproved. The recording's `provenance.txt` identifies driver `4a1def9` and
FAST-LIO `2fffc57`; it is provenance, not independent deployed-binary proof.
