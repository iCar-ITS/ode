import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    param_dir_default = os.path.join(get_package_share_directory('ode_improc'), 'config/config.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            name="config_file",
            default_value=param_dir_default,
            description="Path to config file",
        ),
        Node(
            package='ode_improc',
            executable='undistort_node',
            namespace='camerakiri',
            name='undistort_node',
            output='screen',
            parameters=[LaunchConfiguration('config_file')], 
        ),
        Node(
            package='ode_improc',
            executable='undistort_node',
            namespace='cameratengah',
            name='undistort_node',
            output='screen',
            parameters=[LaunchConfiguration('config_file')], 
        ),
        Node(
            package='ode_improc',
            executable='undistort_node',
            namespace='camerakanan',
            name='undistort_node',
            output='screen',
            parameters=[LaunchConfiguration('config_file')], 
        )
    ])