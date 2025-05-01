from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    
    rosbag = ExecuteProcess(
        cmd=[
            "ros2",
            "bag",
            "record",
            "/camerakiri/image_rect",
            "/cameratengah/image_rect",
            "/camerakanan/image_rect",
            "/velodynekiri/velodyne_points",
            "/velodynekanan/velodyne_points",
            '/lidartengah/lidar_points',
            # "-b 100000000",
            "-d 120",
            # "-p 1000",
            "--compression-mode",
            "file",
            "--compression-format",
            "zstd",
        ],
        name="rosbag",
    )

    return LaunchDescription(
        [
            rosbag
        ]
    )
