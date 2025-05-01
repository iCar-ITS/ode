import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    param_dir_default = os.path.join(get_package_share_directory('ode_record'), 'config/record.yaml')

    telemetry = Node(
        package="ode_record",
        executable="telemetry.py",
        name="telemetry",
        parameters=[{
            "INFLUXDB_URL": "http://127.0.0.1:8086",
            "INFLUXDB_USERNAME": "its",
            "INFLUXDB_PASSWORD": "itssurabaya",
            "INFLUXDB_ORG": "its",
            "INFLUXDB_BUCKET": "robotics",
            "ROBOT_NAME": "omoda", 
        }],
        output="screen",
        remappings=[('/gps/fix', '/fix')],
        respawn=True,
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            name="config_file",
            default_value=param_dir_default,
            description="Path to config file",
        ),
        Node(
            package='ode_record',
            executable='record',
            name='record',
            output='screen',
            parameters=[LaunchConfiguration('config_file')], 
        ),
        # telemetry,
    ])