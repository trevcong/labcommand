# Spatio offline FAST-LIO replay

Re-runs the Spark's FAST_LIO_ROS2 (copied source, unmodified) on a copied
recording inside a local colima container. It writes a session folder in the shared
contract format: `trajectory.tum`, `map.pcd`, `manifest.json` and `logs/`.

```sh
spatio/replay/build_image.sh                       # once; tags spatio/replay:<fastlio-rev>
spatio/replay/replay.sh <recording-dir> <run-id> [--config FILE] [--sessions-root DIR]
spatio/replay/replay.sh <recording-dir> <run-id> --historical --historical-map map.pcd
python3 -m unittest discover -s spatio/replay/tests
python3 spatio/replay/session_table.py ~/Developer/spatio-data/sessions/*   # results table
```

Nothing in the replay is specific to one recording (task I, Trevor's 22:31
direction). `recording.py` reads the recording itself:

- **Topics** come from the FAST-LIO config (`common.lid_topic`, `common.imu_topic`).
  `--config` defaults to the single `*.yaml` containing `lid_topic` inside the
  recording; anything else must be passed explicitly.
- **Sensor kind** comes from the message types in `bag/metadata.yaml`. They must
  agree with the config's `preprocess.lidar_type` (1 = Livox CustomMsg, 2–4 =
  PointCloud2). The IMU must be `sensor_msgs/msg/Imu`.
- **Copy completeness:** every bag file listed in `metadata.yaml` exists, and its
  MCAP summary has the same per-topic counts. This replaced the old check of
  the one-off copy log.
- Only one replay runs at a time on this machine. A second one waits on
  `$TMPDIR/spatio-replay.lock`.

Examples with the two test samples (not defaults anywhere in code):

```sh
spatio/replay/replay.sh ~/Developer/spatio-data/recordings/session_20260808_211213 <run-id>   # its mid360.yaml is found
spatio/replay/replay.sh ~/Developer/spatio-data/recordings/20260826T221936Z_JT128_room_walk <run-id> \
    --config spatio/replay/configs/jt128_fastlio_desktop_2026-08-26.yaml \
    --fastdds-profile spatio/replay/configs/fastdds_big_shm.xml --map-voxel 0.05
```

JT128 configs:

- `jt128_fastlio_desktop_2026-08-26.yaml` is a byte-identical copy of
  `~/Desktop/researchfindings/scripts/jt128_fastlio.yaml` (sha256 `e9d6659d…`,
  mtime 2026-08-26 18:21 CDT). That's the file the original post-process launch
  (`post_jt128_launch.py`) loads as `/ws/jt128_fastlio.yaml`. The Spark's own
  `~/hesai_ws` copy was not copied, so this is the closest available config. It is
  not verified to be the one actually used.
- `jt128_reconstructed.yaml` is the earlier guess (velodyne.yaml with only the
  changes the data forced). Kept for comparison.

The JT128 bag holds the adapter's *output* (`/slam/points`, `/slam/imu`), so the
adapter is not re-run: FAST-LIO gets exactly what the adapter published in the
original run.

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
   feeds the recorded clouds through the copied `map_saver.py` (`--map-voxel`,
   default 0.03 m),
   because this FAST-LIO port never writes a map itself.

colima here shares no usable host folder, so a bind mount would silently be an
empty VM directory. The recording is therefore copied once into a named volume
(`spatio-rec-<name>`) and checked by sha256. Tools are added with `docker cp`, and
outputs are copied back out the same way.

`tum.py`, `timing.py`, `manifest.py` and `compare_tum.py` use only the
standard library and run on the Mac. `inspect_bag.py`, `bag_to_tum.py`,
`bag_to_map.py` and `odom_watch.py` need ROS and run in the container.
