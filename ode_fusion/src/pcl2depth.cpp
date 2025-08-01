#include <cstdio>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/filter.h>
#include <pcl_conversions/pcl_conversions.h>
#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <yaml-cpp/yaml.h>
// #include <message_filters/subscriber.h>
// #include <message_filters/time_synchronizer.h>

class Pcl2DepthNode : public rclcpp::Node
{
private:
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr im_sub_;

  sensor_msgs::msg::Image image_msg_;
  sensor_msgs::msg::CameraInfo cam_info_msg_;

  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

  bool is_ = false;

  void pcl_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    // Convert PointCloud2 to PCL PointCloud
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*msg, cloud);
    // RCLCPP_INFO(this->get_logger(), "Depth image started");
    
    geometry_msgs::msg::TransformStamped transform_stamped;
    try {
      transform_stamped = tf_buffer_->lookupTransform(
            "cameratengah",  // e.g., "base_link"
            "lidartengah",
            tf2::TimePointZero  // or rclcpp::Time(cloud->header.stamp)
          );
    } catch (tf2::TransformException &ex) {
      RCLCPP_WARN(this->get_logger(), "Could not transform: %s", ex.what());
      return;
    }

    pcl::PointCloud<pcl::PointXYZ> pcl_cloud_transformed;
    pcl_ros::transformPointCloud(cloud, pcl_cloud_transformed, transform_stamped);

    cam_info_msg_.header = msg->header;

    image_geometry::PinholeCameraModel cam_model;
    cam_model.fromCameraInfo(cam_info_msg_);

    int width = 854;
    int height = 480;
    
    cv::Mat depth_image = cv::Mat::zeros(height, width, CV_32FC1);

    for (const auto& point : pcl_cloud_transformed.points)
    {
      if (point.x > 0) // Ensure the point is in front of the camera
      {
        // cv::Mat point_mat = (cv::Mat_<double>(4, 1) << -point.y, -point.z, point.x, 1.0);
        cv::Point3d point_3d(-point.y, -point.z, point.x);  

        // cv::Mat uvw = projection_matrix * point_mat;

        cv::Point2d uv = cam_model.project3dToPixel(point_3d);

        if (uv.x >= 0 && uv.x < width && uv.y >= 0 && uv.y < height)
        {
          float depth_value = std::hypot(point.x, point.y, point.z);
          // if(depth_value < 6.0)
          {
            // depth_value = 0.0;
            depth_image.at<float>(uv.y, uv.x) = depth_value;
            depth_image.at<float>(std::max(uv.y-1, 0.0), uv.x) = depth_value;
            depth_image.at<float>(uv.y, std::max(uv.x-1, 0.0)) = depth_value;
            depth_image.at<float>(std::max(uv.y-1, 0.0), std::max(uv.x-1, 0.0)) = depth_value;
          }
        }
      }
    }

    // cv::Mat kernel = cv::getGaussianKernel(9, 1.0, CV_32F);
    // cv::filter2D(depth_image, depth_image, -1, kernel);
    
    cv::Mat gray_image = cv::Mat::zeros(height, width, CV_8UC1);
    
    cv::normalize(depth_image, gray_image, 0, 255, cv::NORM_MINMAX, CV_8UC1);
    
    cv::Mat overlay_image;
    if(is_)
    {
      auto rgb_image = cv_bridge::toCvCopy(image_msg_);
      cv::Mat mono_image;
      cv::cvtColor(rgb_image->image, mono_image, cv::COLOR_BGR2GRAY);
      cv::addWeighted(mono_image, 0.2, gray_image, 1.0, 0.0, overlay_image);
    }
    
    // Convert to ROS Image message
    sensor_msgs::msg::Image img_msg;
    cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", overlay_image).toImageMsg(img_msg);
    // cv_bridge::CvImage(std_msgs::msg::Header(), "32FC1", depth_image).toImageMsg(img_msg);
    img_msg.header.stamp = this->now();
    img_msg.header.frame_id = msg->header.frame_id;

    // Publish the depth image
    depth_pub_->publish(img_msg);

    // cv::imshow("Depth Image", overlay_image);

    // RCLCPP_INFO(this->get_logger(), "Depth image published");
  }

public:
  Pcl2DepthNode()
  : Node("pcl2depth_node")
  {
    auto config = YAML::LoadFile("/home/zainir17/ta_ws/src/ode/ode_fusion/config/camera_info.yaml");
    cam_info_msg_.width = config["image_width"].as<int>();
    cam_info_msg_.height = config["image_height"].as<int>();
    cam_info_msg_.k[0] = config["camera_matrix"][0].as<double>();
    cam_info_msg_.k[1] = config["camera_matrix"][1].as<double>();
    cam_info_msg_.k[2] = config["camera_matrix"][2].as<double>();
    cam_info_msg_.k[3] = config["camera_matrix"][3].as<double>();
    cam_info_msg_.k[4] = config["camera_matrix"][4].as<double>();
    cam_info_msg_.k[5] = config["camera_matrix"][5].as<double>();
    cam_info_msg_.k[6] = config["camera_matrix"][6].as<double>();
    cam_info_msg_.k[7] = config["camera_matrix"][7].as<double>();
    cam_info_msg_.k[8] = config["camera_matrix"][8].as<double>();
    cam_info_msg_.r[0] = config["rectification_matrix"][0].as<double>();
    cam_info_msg_.r[1] = config["rectification_matrix"][1].as<double>();
    cam_info_msg_.r[2] = config["rectification_matrix"][2].as<double>();
    cam_info_msg_.r[3] = config["rectification_matrix"][3].as<double>();
    cam_info_msg_.r[4] = config["rectification_matrix"][4].as<double>();
    cam_info_msg_.r[5] = config["rectification_matrix"][5].as<double>();
    cam_info_msg_.r[6] = config["rectification_matrix"][6].as<double>();
    cam_info_msg_.r[7] = config["rectification_matrix"][7].as<double>(); 
    cam_info_msg_.r[8] = config["rectification_matrix"][8].as<double>();

    for (int i = 0; i < 5; ++i) {
      cam_info_msg_.d.push_back(config["distortion_coefficients"][i].as<double>());
    }

    cam_info_msg_.p[0] = config["projection_matrix"][0].as<double>();
    cam_info_msg_.p[1] = config["projection_matrix"][1].as<double>();
    cam_info_msg_.p[2] = config["projection_matrix"][2].as<double>();
    cam_info_msg_.p[3] = config["projection_matrix"][3].as<double>();
    cam_info_msg_.p[4] = config["projection_matrix"][4].as<double>();
    cam_info_msg_.p[5] = config["projection_matrix"][5].as<double>();
    cam_info_msg_.p[6] = config["projection_matrix"][6].as<double>();
    cam_info_msg_.p[7] = config["projection_matrix"][7].as<double>();
    cam_info_msg_.p[8] = config["projection_matrix"][8].as<double>();
    cam_info_msg_.p[9] = config["projection_matrix"][9].as<double>();
    cam_info_msg_.p[10] = config["projection_matrix"][10].as<double>();
    cam_info_msg_.p[11] = config["projection_matrix"][11].as<double>();

    cam_info_msg_.distortion_model = config["distortion_model"].as<std::string>();
    cam_info_msg_.header.frame_id = "cameratengah";

    pcl_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/pcl_concat/lidar_points", 10, std::bind(&Pcl2DepthNode::pcl_callback, this, std::placeholders::_1));
      
    im_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/cameratengah/image_rect", 10, [this](const sensor_msgs::msg::Image::SharedPtr msg) {
        // RCLCPP_INFO(rclcpp::get_logger("pcl2depth_node"), "Received image");
        image_msg_ = *msg;
        is_ = true;
      });

    depth_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
      "depth_image", 10);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    RCLCPP_INFO(this->get_logger(), "Pcl2DepthNode initialized");
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Pcl2DepthNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
