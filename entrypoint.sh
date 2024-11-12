#!/bin/bash
# Source ROS 2 and workspace setup
source /opt/ros/humble/setup.sh
source /ros2_ws/install/setup.sh

# Run the launch file to start Next.js and ROS nodes
ros2 launch fleetglue_cpp nextjs_with_ros.launch.py
