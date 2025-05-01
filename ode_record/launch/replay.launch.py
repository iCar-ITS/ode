import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import launch
import launch_ros.actions

def generate_launch_description():

    param_dir_default = os.path.join(get_package_share_directory('ode_record'), 'config/replay.yaml')

    logger = LaunchConfiguration("log_level")

    return LaunchDescription([
        DeclareLaunchArgument(
            "log_level",
            default_value=["info"],
            description="Logging level",
        ),
        DeclareLaunchArgument(
            name="config_file",
            default_value=param_dir_default,
            description="Path to config file",
        ),
        Node(
            package='ode_record',
            executable='replay',
            name='replay',
            output='screen',
            parameters=[LaunchConfiguration('config_file')], 
            arguments=['--ros-args', '--log-level', logger]
        ),
    ])