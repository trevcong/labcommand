# Spatio offline FAST-LIO replay image (built by build_image.sh; do not build
# directly -- the context is a staging dir assembled from spark-src copies).
#
# Contains: FAST_LIO_ROS2 (copied Spark tree, unmodified) + a messages-only
# livox_ros_driver2 + the copied historical map_saver.py. No sensor drivers.
FROM ros:jazzy-ros-base

RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-jazzy-pcl-ros \
        ros-jazzy-pcl-conversions \
        ros-jazzy-sensor-msgs-py \
        ros-jazzy-rosbag2-storage-mcap \
        libpcl-dev \
        libeigen3-dev \
        python3-dev \
        python3-numpy \
    && rm -rf /var/lib/apt/lists/*

# FAST_LIO_ROS2 compiles ROOT_DIR from its source path; keep it at the same
# path the Spark used (/ws/src/FAST_LIO_ROS2/) so log paths match.
COPY src/ /ws/src/
WORKDIR /ws
RUN . /opt/ros/jazzy/setup.sh \
    && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release \
    && rm -rf build log

COPY tools/map_saver.py /opt/spatio/map_saver.py
COPY provenance.json /opt/spatio/provenance.json

ARG FASTLIO_REVISION
LABEL spatio.fastlio_revision=$FASTLIO_REVISION
