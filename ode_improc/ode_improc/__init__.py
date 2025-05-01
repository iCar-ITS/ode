import rclpy

# Initialize the ROS 2 package
def init_ros2_package():
    """
    Initialize the ROS 2 package for 'ode_improc'.
    """
    rclpy.init()

__all__ = ['init_ros2_package']