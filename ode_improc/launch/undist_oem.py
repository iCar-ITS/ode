

import os
from launch import LaunchDescription
import launch_ros.actions
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    return LaunchDescription([
        launch_ros.actions.Node(
            package="image_proc",
            executable="rectify_node",
            name="rectify_node",
            namespace="cameratengah",
            output="both",
            remappings=[
                    ('image', 'image_raw')
                ],
        )
    ])