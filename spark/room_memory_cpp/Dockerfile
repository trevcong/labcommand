# room_memory_cpp on ROS 2 Jazzy, next to the lidar-dual container (brief 2.2). Built on the
# Spark itself, not on the Mac. The build context is the repository root, which must hold
# ros/labcommand_msgs from the Phase 0 branch (the generic publishers need its type support
# at run time):
#
#   docker build -f spark/room_memory_cpp/Dockerfile -t labcommand/room-memory-cpp:jazzy .
#   docker run --rm --net=host --ipc=host -v ~/room-memory:/root/room-memory labcommand/room-memory-cpp:jazzy
#
# --net=host and --ipc=host let DDS discovery and shared memory reach the lidar drivers.
# CPU_FLAGS defaults to the build machine's CPU; set it (e.g. -march=armv9-a) when the image
# is built on a different host than the one it runs on.
FROM ros:jazzy-ros-base

ARG CPU_FLAGS=-mcpu=native

RUN apt-get update \
    && apt-get install -y --no-install-recommends zlib1g-dev ros-jazzy-sensor-msgs python3-numpy python3-pytest \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ws
COPY ros/labcommand_msgs src/labcommand_msgs
COPY spark/room_memory_cpp src/room_memory_cpp
# The Python package is the oracle of the differential tests; it is not installed or run.
COPY spark/room_memory /oracle/room_memory
COPY Tests/Fixtures/change-detection fixtures/change-detection

RUN . /opt/ros/jazzy/setup.sh \
    && colcon build --packages-select labcommand_msgs room_memory_cpp --cmake-args \
        -DCMAKE_BUILD_TYPE=Release -DROOM_MEMORY_NATIVE=OFF "-DCMAKE_CXX_FLAGS=${CPU_FLAGS}" \
        -DROOM_MEMORY_PYTHON=/usr/bin/python3 -DROOM_MEMORY_PYTHON_PACKAGE=/oracle/room_memory \
        -DROOM_MEMORY_CD_FIXTURES=/ws/fixtures/change-detection

RUN . /opt/ros/jazzy/setup.sh && . install/setup.sh \
    && colcon test --packages-select room_memory_cpp --event-handlers console_direct+ \
    && colcon test-result --verbose

ENTRYPOINT ["/ws/src/room_memory_cpp/docker-entrypoint.sh"]
CMD ["ros2", "launch", "room_memory_cpp", "room_memory_cpp.launch.py"]
