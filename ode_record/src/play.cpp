#include "rclcpp/rclcpp.hpp"

class MinimalNode : public rclcpp::Node
{
public:
  MinimalNode() : Node("minimal_node")
  {
    RCLCPP_INFO(this->get_logger(), "Minimal ROS 2 Node has been started.");
  }
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MinimalNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}