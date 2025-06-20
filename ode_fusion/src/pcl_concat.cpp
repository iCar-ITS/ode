#include <rclcpp/rclcpp.hpp>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/filter.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <message_filters/subscriber.h>
#include "message_filters/synchronizer.hpp"
#include "message_filters/sync_policies/approximate_time.hpp"
#include "message_filters/message_traits.hpp"

class PclConcatNode : public rclcpp::Node
{
private:
  using PCL_Sync = message_filters::Synchronizer
                    <message_filters::sync_policies::ApproximateTime
                      <sensor_msgs::msg::PointCloud2,
                       sensor_msgs::msg::PointCloud2,
                       sensor_msgs::msg::PointCloud2>>;

  // std::vector<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>> subscribers_;
  std::array<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>, 3> subscribers_;
  std::shared_ptr<PCL_Sync> synchronizers_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;

  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
public:
  PclConcatNode()
  : Node("pcl_concat_node")
  {
    std::vector<std::string> def_topic_names = {"/velodynekiri/velodyne_points", "/lidartengah/lidar_points", "/velodynekanan/velodyne_points"};
    this->declare_parameter("topic_name", def_topic_names);

    auto topic_names = this->get_parameter("topic_name").as_string_array();
    if(topic_names.size() != 3) {
      RCLCPP_ERROR(this->get_logger(), "Please provide exactly 3 topic names");
      return;
    }
    for (int i=0; i<3; i++) {
        RCLCPP_INFO(this->get_logger(), "Topic name: %s", topic_names[i].c_str());
        // auto sub = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>(this, topic_name);
        // subscribers_.push_back(sub);
        subscribers_[i].subscribe(this, topic_names[i]);
    }

    synchronizers_ = std::make_shared<PCL_Sync>(
        message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, sensor_msgs::msg::PointCloud2, sensor_msgs::msg::PointCloud2>(10), subscribers_[0], subscribers_[1], subscribers_[2]);

    synchronizers_->registerCallback(std::bind(&PclConcatNode::pcl_callback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

    pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/pcl_concat/lidar_points", 5);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  }

  void pcl_callback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg1, 
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg2, 
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg3 )
  {
    // Convert PointCloud2 to PCL PointCloud
    pcl::PointCloud<pcl::PointXYZI> cloud[3];
    pcl::fromROSMsg(*msg1, cloud[0]);
    pcl::fromROSMsg(*msg2, cloud[1]);
    pcl::fromROSMsg(*msg3, cloud[2]);
    std::string msg_frame[] = {msg1->header.frame_id, msg2->header.frame_id, msg3->header.frame_id};

    for(int i=0; i<3; i++) 
    {
      geometry_msgs::msg::TransformStamped transform_stamped;
      try {
        transform_stamped = tf_buffer_->lookupTransform(
              "base_lidar",  // e.g., "base_link"
              msg_frame[i],
              tf2::TimePointZero  // or rclcpp::Time(cloud->header.stamp)
            );
      } catch (tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "Could not transform: %s", ex.what());
        return;
      }

      pcl_ros::transformPointCloud(cloud[i], cloud[i], transform_stamped);
    }

    // Concatenate the point clouds
    // pcl::PointCloud<pcl::PointXYZI> cloud_concat;
    // cloud_concat += cloud[0];
    cloud[0] += cloud[1];
    cloud[0] += cloud[2];

    RCLCPP_DEBUG(this->get_logger(), "Concatenated point cloud with %zu points", cloud[0].points.size());

    // Convert PCL PointCloud back to PointCloud2
    sensor_msgs::msg::PointCloud2 output;
    pcl::toROSMsg(cloud[0], output);
    output.header.frame_id = "base_lidar"; // Set the frame_id for the output message
    output.header.stamp = msg2->header.stamp; // Set the timestamp for the output message

    // Publish the concatenated point cloud 
    pub_->publish(output);  
  }

};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PclConcatNode>());
  rclcpp::shutdown();
  return 0;
}