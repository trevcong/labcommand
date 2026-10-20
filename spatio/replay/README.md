# Spatio offline FAST-LIO replay

Re-runs the Spark's FAST_LIO_ROS2 (copied source, unmodified) on a copied
recording inside a local colima container. It writes a session folder in the shared
contract format: `trajectory.tum`, `map.pcd`, `manifest.json` and `logs/`.

```sh
spatio/replay/build_image.sh                       # once; tags spatio/replay:<fastlio-rev>
spatio/replay/replay.sh session_20260808_211213 <run-id>             # Mid-360
spatio/replay/replay.sh session_20260808_211213 <run-id> --historical
spatio/replay/replay.sh 20260826T221936Z_JT128_room_walk <run-id> \
    --config spatio/replay/configs/jt128_reconstructed.yaml \
    --fastdds-profile spatio/replay/configs/fastdds_big_shm.xml
python3 -m unittest discover -s spatio/replay/tests
```

What a replay does, all in one container (`--network none`, private
`ROS_DOMAIN_ID`, recording volume mounted read-only, CPU/memory/pids limits, all
capabilities dropped):

1. `inspect_bag.py` writes `logs/input_timing.json`. It covers header vs receipt time,
   gaps, IMU coverage of each scan and the IMU unit guess.
2. FAST-LIO starts with the recording's config and records `/Odometry` and
   `/cloud_registered` to `logs/output_bag`.
3. `ros2 bag play` plays **only** the lidar and IMU topics. The bags also contain
   the original run's `/Odometry`, and that must not mix in.
4. The runner waits until `/Odometry` has been quiet for 5 s, then stops the
   recorder and FAST-LIO with SIGINT.
5. `bag_to_tum.py` writes `trajectory.tum` from header stamps. `bag_to_map.py`
   feeds the recorded clouds through the copied `map_saver.py` (0.03 m voxel),
   because this FAST-LIO port never writes a map itself.

colima here shares no usable host folder, so a bind mount would silently be an
empty VM directory. The recording is therefore copied once into a named volume
(`spatio-rec-<name>`) and checked by sha256. Tools are added with `docker cp`, and
outputs are copied back out the same way.

`tum.py`, `timing.py`, `manifest.py` and `compare_tum.py` use only the
standard library and run on the Mac. `inspect_bag.py`, `bag_to_tum.py`,
`bag_to_map.py` and `odom_watch.py` need ROS and run in the container.
