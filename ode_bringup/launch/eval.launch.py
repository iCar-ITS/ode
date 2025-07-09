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

    camera_rectify_node2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory("ode_improc"), "launch"), "/undistort.py"]
        )
    )

    viz_pcl2depth = Node(
        package="ode_fusion",
        executable="pcl2depth2",
        parameters=[{
            "sync_queue": 20,
        }],
    )

    pcl_concate = Node(
        package="ode_fusion",
        executable="pcl_concat",
    )

    labeling = Node(
        package='ode_label',
        executable='live_labeling',
        parameters=[{
            "k_max_value": 128,
            "k_min_value": 8,
            }],
        output='screen',
    )

    yolo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            "/home/zainir17/ta_ws/src/YOLOv5_ROS2-YOu-can-Leverage-On-ROS2/yolov5_ros2/launch/yolov5_ros2_node.launch.py"
        ),
        launch_arguments={
            "sub_topic": "/cameratengah/image_rect",
            "pub_topic": "/yolov5/image",
            "weights": "/home/zainir17/ta_ws/src/yolov5_ros2/yolov5_ros2/weights/yolov5s.pt",
            "device": "",
        }.items()
    )

    pred = Node(
        package='ode_predict',
        executable='predict_node',
        parameters=[{
            "model_path": "/home/zainir17/Documents/TA-ICAR/ode_predict_6/weight/model_resnet_weights_20250706-060215.pth",
            "device": 'cuda',
            }],
        remappings=[
            ('/bounding_boxes', '/yolov5/image/bounding_boxes'),
            ('/image', '/yolov5/image_raw'),
        ],
        output='screen',
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
            camera_rectify_node2,
            viz_pcl2depth,
            pcl_concate,
            labeling,
            pred,
            yolo,
        ]
    )
