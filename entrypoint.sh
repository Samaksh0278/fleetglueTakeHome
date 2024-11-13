#!/bin/bash

# Source the global ROS 2 environment
source /opt/ros/humble/setup.sh

# Source the workspace if the build was successful
if [ -f "/ros2_ws/install/setup.sh" ]; then
    source /ros2_ws/install/setup.sh
fi

# Start the ROS launch file
ros2 launch fleetglue_cpp nextjs_with_ros.launch.py
