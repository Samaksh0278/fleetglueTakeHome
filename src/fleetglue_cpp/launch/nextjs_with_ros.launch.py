from launch import LaunchDescription
from launch.actions import ExecuteProcess
import os

def generate_launch_description():
    # Define the path to the Next.js project directory
    nextjs_dir = os.path.join(
        os.getenv('HOME'), 'ros2_ws', 'src', 'fleetglue_cpp', 'nextjs_api'
    )
    #nextjs_dir = "/ros2_ws/src/fleetglue_cpp/nextjs_api"
    return LaunchDescription([
        # Start the Next.js API server
        ExecuteProcess(
            cmd=['npm', 'run', 'dev'],
            cwd=nextjs_dir,
            output='screen'
        ),
        
        # Start the ROS node (node1 in fleetglue_cpp package)
        ExecuteProcess(
            cmd=['ros2', 'run', 'fleetglue_cpp', 'node1'],
            output='screen'
        ),
        # Start node2 (action server node in fleetglue_cpp package)
        ExecuteProcess(
            cmd=['ros2', 'run', 'fleetglue_cpp', 'node2'],
            output='screen'
        )
    ])
