from launch import LaunchDescription
from launch.actions import ExecuteProcess
import os

def generate_launch_description():
    # Define the path to the Next.js project directory
    nextjs_dir = os.path.join(
        os.getenv('HOME'), 'ros2_ws', 'src', 'fleetglue_cpp', 'nextjs_api'   #this is to run locally
    )
    nextjs_dir = "/ros2_ws/src/fleetglue_cpp/nextjs_api"                    #this is with docker
    return LaunchDescription([
        # Start the Next.js API server
        ExecuteProcess(
            cmd=['npm', 'run', 'dev'],
            cwd=nextjs_dir,
            output='screen'
        ),
        
        # Start RN1
        ExecuteProcess(
            cmd=['ros2', 'run', 'fleetglue_cpp', 'RN1'],
            output='screen'
        ),
        # Start RN2
        ExecuteProcess(
            cmd=['ros2', 'run', 'fleetglue_cpp', 'RN2'],
            output='screen'
        )
    ])
