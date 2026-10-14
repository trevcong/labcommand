# room_memory on ROS 2 Jazzy, next to the lidar-dual container (brief §2.2). Not built or
# deployed from the Mac. The build context is the repository root, which must hold
# ros/labcommand_msgs from the Phase 0 branch:
#
#   docker build -f spark/room_memory/Dockerfile -t labcommand/room-memory:jazzy .
#   docker run --rm --net=host --ipc=host -v ~/room-memory:/root/room-memory labcommand/room-memory:jazzy
#
# --net=host and --ipc=host let DDS discovery and shared memory reach the lidar drivers.
FROM ros:jazzy-ros-base

RUN apt-get update \
    && apt-get install -y --no-install-recommends python3-numpy python3-pytest ros-jazzy-sensor-msgs \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ws
COPY ros/labcommand_msgs src/labcommand_msgs
COPY spark/room_memory src/room_memory
COPY Tests/Fixtures/change-detection fixtures/change-detection
COPY Tests/Fixtures/room/snapshots fixtures/room/snapshots

RUN . /opt/ros/jazzy/setup.sh \
    && colcon build --packages-select labcommand_msgs room_memory

RUN . /opt/ros/jazzy/setup.sh && . install/setup.sh \
    && cd src/room_memory \
    && ROOM_MEMORY_CD_FIXTURES=/ws/fixtures/change-detection ROOM_MEMORY_SNAPSHOTS=/ws/fixtures/room/snapshots python3 -m pytest -q tests

ENTRYPOINT ["/ws/src/room_memory/docker-entrypoint.sh"]
CMD ["ros2", "launch", "room_memory", "room_memory.launch.py"]
