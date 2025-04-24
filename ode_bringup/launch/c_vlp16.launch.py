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
    driver_params_file = os.path.join(driver_share_dir, "config", "vlp16_driver.yaml")

    velodyne_driver_node2 = launch_ros.actions.Node(
        package="velodyne_driver",
        executable="velodyne_driver_node",
        namespace="velodynetengah",
        output="both",
        parameters=[driver_params_file],
    )

    with open(driver_params_file, "r") as f:
        convert_params2 = yaml.safe_load(f)["velodynetengah"][
            "velodyne_transform_node"
        ]["ros__parameters"]
        
    convert_params2["calibration"] = os.path.join(
        driver_share_dir, "config", "VLP16db.yaml"
    )

    velodyne_transform_node2 = launch_ros.actions.Node(
        package="velodyne_pointcloud",
        executable="velodyne_transform_node",
        output="both",
        namespace="velodynetengah",
        parameters=[convert_params2],
    )

    return launch.LaunchDescription(
        [
            velodyne_driver_node2,
            velodyne_transform_node2,
            launch.actions.RegisterEventHandler(
                event_handler=launch.event_handlers.OnProcessExit(
                    target_action=velodyne_driver_node2,
                    on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
                )
            ),
        ]
    )
