#include <rclcpp/rclcpp.hpp>
#include <message_filters/synchronizer.hpp>
#include <message_filters/subscriber.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>


class LabelingNode : public rclcpp::Node
{
private:

public:

};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LabelingNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
