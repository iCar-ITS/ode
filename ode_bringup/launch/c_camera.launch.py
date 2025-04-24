"""Launch the velodyne driver, pointcloud, and laserscan nodes with default configuration."""

import os

import ament_index_python.packages
import launch
import launch_ros.actions
import yaml


def generate_launch_description():
    driver_share_dir = ament_index_python.packages.get_package_share_directory(
        "ode_bringup"
    )
    driver_params_file = os.path.join(driver_share_dir, "config", "camera_driver.yaml")

    camera_driver_node2 = launch_ros.actions.Node(
        package="v4l2_camera",
        executable="v4l2_camera_node",
        name="camera_node",
        namespace="cameratengah",
        output="both",
        parameters=[driver_params_file],
    )

    ## Undistord Image

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


    return launch.LaunchDescription(
        [
            camera_driver_node2,
            camera_rectify_node2
        ]
    )
