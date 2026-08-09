#!/bin/bash
set -e

source /opt/ros/humble/setup.bash
source /root/Recruitment-task/install/setup.bash

# The launch file copies level3.sdf into the CWD as a workaround for a
# Gazebo resource-path lookup quirk - always run from the workspace root.
cd /root/Recruitment-task

case "$1" in
  sim)
    echo "Starting Gazebo simulation + WebSocket broadcaster..."
    exec ros2 launch questions main_assignment.launch.py
    ;;
  navigator)
    echo "Starting waypoint navigator (WebSocket nav + CV sphere detection)..."
    exec ros2 run questions waypoint_navigator
    ;;
  slam)
    echo "Starting SLAM toolbox..."
    exec ros2 launch slam_toolbox online_async_launch.py \
      slam_params_file:=/root/Recruitment-task/slam_config/mapper_params_online_async.yaml
    ;;
  bash)
    exec /bin/bash
    ;;
  *)
    echo "Unknown command: $1"
    echo "Usage: docker run <image> [sim|navigator|slam|bash]"
    exec /bin/bash
    ;;
esac
