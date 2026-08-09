# ============================================================
# TurtleBot4 Recruitment Task - Full Environment Dockerfile
# ROS 2 Humble + Ignition Gazebo (Fortress) + TurtleBot4 Simulator
# + WebSocket navigation node + OpenCV sphere detection + SLAM
# ============================================================

FROM osrf/ros:humble-desktop

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble

# ------------------------------------------------------------
# System dependencies + Gazebo/TurtleBot4 packages
# ------------------------------------------------------------
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    ros-humble-turtlebot4-desktop \
    ros-humble-turtlebot4-simulator \
    ros-humble-turtlebot4-navigation \
    ros-humble-slam-toolbox \
    ros-humble-cv-bridge \
    ros-humble-rmw-cyclonedds-cpp \
    mesa-utils \
    libgl1-mesa-dri \
    libgl1-mesa-glx \
    && rm -rf /var/lib/apt/lists/*

# ------------------------------------------------------------
# Python dependencies
# NOTE: websockets is pinned <11 - the provided websocket_broadcaster.py
# uses an old-style handler signature incompatible with websockets>=11
# (see project notes: causes silent "RuntimeError: no running event loop"
# thread crash under newer versions).
# ------------------------------------------------------------
RUN pip3 install --no-cache-dir \
    "websockets<11" \
    "numpy<2" \
    opencv-python-headless
# ------------------------------------------------------------
# WSL / software-rendering environment variables
# These were required to fix:
#   1. Gazebo GUI crash (Ogre::UnimplementedException) under WSLg
#   2. Camera sensor producing blank/gray frames under software GL
# Safe to keep set even on native Linux with a real GPU - if you
# have GPU passthrough working, you can override/unset these at
# `docker run` time instead.
# ------------------------------------------------------------
ENV LIBGL_ALWAYS_SOFTWARE=1
ENV IGN_GAZEBO_RENDER_ENGINE=ogre

# ------------------------------------------------------------
# Workspace setup
# ------------------------------------------------------------
WORKDIR /root/Recruitment-task

# Copy only what's needed to build first (better Docker layer caching),
# then copy the rest.
COPY src/ ./src/

RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y || true && \
    colcon build --symlink-install"

# Copy remaining project files (world file, guidelines, etc.)
COPY . .

# ------------------------------------------------------------
# Entrypoint
# ------------------------------------------------------------
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["sim"]
