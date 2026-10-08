FROM ros:noetic-ros-base-focal@sha256:72b8bc59035dc0a5b8e07aae28c16caa84192971d72d207c72ed734fb1d5e97d
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV DEBIAN_FRONTEND=noninteractive
ARG FAST_LIO_COMMIT=7cc4175de6f8ba2edf34bab02a42195b141027e9
ARG LIVOX_DRIVER_COMMIT=3d240d5666129e1a3052e78ee8487a04b08fdda3
ARG LIVOX_SDK_COMMIT=9306596a2bf15c1343bc023b497465ed0a32909d
LABEL org.opencontainers.image.source="https://github.com/hku-mars/FAST_LIO" \
      org.opencontainers.image.revision="${FAST_LIO_COMMIT}" \
      io.robomaster.livox-driver.revision="${LIVOX_DRIVER_COMMIT}" \
      io.robomaster.livox-sdk.revision="${LIVOX_SDK_COMMIT}"
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake git ca-certificates libeigen3-dev libpcl-dev libapr1-dev \
    python3-dev python3-numpy python3-matplotlib \
    ros-noetic-pcl-ros ros-noetic-eigen-conversions ros-noetic-tf \
    ros-noetic-rosbag ros-noetic-nav-msgs ros-noetic-message-generation \
    && rm -rf /var/lib/apt/lists/*
RUN git clone https://github.com/Livox-SDK/Livox-SDK.git /opt/Livox-SDK \
    && cd /opt/Livox-SDK && git checkout --detach "${LIVOX_SDK_COMMIT}" \
    && git submodule update --init --recursive \
    && cmake -S . -B build -DCMAKE_BUILD_TYPE=Release \
    && cmake --build build -- -j2 && cmake --install build
WORKDIR /opt/catkin_ws
RUN mkdir -p src \
    && git clone https://github.com/Livox-SDK/livox_ros_driver.git src/livox_ros_driver \
    && cd src/livox_ros_driver && git checkout --detach "${LIVOX_DRIVER_COMMIT}" \
    && git submodule update --init --recursive
RUN git clone https://github.com/hku-mars/FAST_LIO.git src/FAST_LIO \
    && cd src/FAST_LIO && git checkout --detach "${FAST_LIO_COMMIT}" \
    && git submodule update --init --recursive
# Build generated Livox messages before upstream FAST_LIO (which omits an
# explicit add_dependencies edge for those headers). No upstream source patches.
RUN source /opt/ros/noetic/setup.bash \
    && catkin_make -j2 -l2 --pkg livox_ros_driver \
    && source devel/setup.bash && catkin_make -j2 -l2 \
    && test -x devel/lib/fast_lio/fastlio_mapping \
    && { git -C src/FAST_LIO rev-parse HEAD; \
         git -C src/FAST_LIO submodule status --recursive; \
         git -C src/livox_ros_driver rev-parse HEAD; \
         git -C /opt/Livox-SDK rev-parse HEAD; \
         dpkg-query -W; } > /opt/build-manifest.txt
COPY config.yaml offline.launch replay.cpp CMakeLists.txt entrypoint.sh /opt/fast_lio/
RUN source /opt/ros/noetic/setup.bash \
    && source /opt/catkin_ws/devel/setup.bash \
    && cmake -S /opt/fast_lio -B /opt/fast_lio/build \
         -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/opt/fast_lio \
    && cmake --build /opt/fast_lio/build -- -j2 \
    && cd /opt/fast_lio/build && ctest --output-on-failure \
    && cmake --install /opt/fast_lio/build \
    && test -x /opt/fast_lio/bin/fast_lio_replay
ENV ROS_MASTER_URI=http://127.0.0.1:11311 ROS_IP=127.0.0.1 ROS_HOME=/tmp/ros OMP_NUM_THREADS=2
ENTRYPOINT ["/opt/fast_lio/entrypoint.sh"]
