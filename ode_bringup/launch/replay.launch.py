import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
import launch_ros.actions

def generate_launch_description():
    pkg_share = get_package_share_directory("ode_bringup")
    default_model_path = os.path.join(pkg_share, "model/omoda.urdf")

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {"robot_description": Command(["xacro ", LaunchConfiguration("model")])}
        ],
    )

    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        arguments=[default_model_path],
    )

    camera_rectify_node2 = launch_ros.actions.Node(
        package="image_proc",
        executable="rectify_node",
        name="rectify_node",
        namespace="cameratengah",
        output="both",
        remappings=[
                ('image', 'image_raw')
            ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                name="model",
                default_value=default_model_path,
                description="Absolute path to robot urdf file",
            ),
            joint_state_publisher_node,
            robot_state_publisher_node,
            # camera_rectify_node2,
        ]
    )
