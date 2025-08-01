#include <cstdio>
#include <cmath>
#include <limits>

#include <rclcpp/rclcpp.hpp>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <yaml-cpp/yaml.h>

#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/filter.h>
#include <pcl/range_image/range_image.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/colors.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.hpp>
#include <message_filters/message_traits.hpp>
#include <message_filters/cache.hpp>

class Pcl2DepthNode : public rclcpp::Node
{
private:
  using SubsSyncPolicy = message_filters::sync_policies::ApproximateTime
                      <sensor_msgs::msg::PointCloud2, sensor_msgs::msg::Image>;

  using SubsSync = message_filters::Synchronizer
                    <SubsSyncPolicy>;

  std::shared_ptr<SubsSync> synchronizers_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_sub_;

  message_filters::Subscriber<sensor_msgs::msg::Image>im_sub_;

  std::shared_ptr<message_filters::Cache<sensor_msgs::msg::Image>> im_cache_;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_map_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr proj_pub_;

  // sensor_msgs::msg::Image image_msg_;
  sensor_msgs::msg::CameraInfo cam_info_msg_;

  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

  float max_depth_ = 100.0f; // Maximum depth value to consider

  int width_ = 854;
  int height_ = 480;

  int sync_window_ms_ = 10;

  bool is_ = false;





  
  rclcpp::Duration abs_time_diff(const rclcpp::Time& t1, const rclcpp::Time& t2)
  {
    return (t1 > t2) ? (t1 - t2) : (t2 - t1);
  }






  void callback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr& pcl_msg)
  {

    auto header = pcl_msg->header;
    header.frame_id = "cameratengah";


    /**
     ************* SYNCRONIZATION *************
     */

    // Sync the point cloud and image messages
    rclcpp::Time b_time = pcl_msg->header.stamp;

    rclcpp::Duration window(0, sync_window_ms_ * 1000000);
    auto im_candidates = im_cache_->getInterval(b_time - window, b_time + window);
    if (im_candidates.empty()) {
      RCLCPP_DEBUG(this->get_logger(), "No image message found near point cloud timestamp");
      return;
    }

    sensor_msgs::msg::Image::ConstSharedPtr im_msg = nullptr;
    rclcpp::Duration min_diff = rclcpp::Duration::from_seconds(1000);

    for (auto& msg : im_candidates) {
      rclcpp::Duration diff = abs_time_diff(b_time, msg->header.stamp);\
      if (diff < min_diff) {
        min_diff = diff;
        im_msg = msg;
      }
    }



    /**
     ************* PCL TRANSFORM *************
     */

    // Convert PointCloud2 to PCL PointCloud
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*pcl_msg, cloud);
    
    // Get the transform from "lidartengah" to "cameratengah"
    geometry_msgs::msg::TransformStamped transform_stamped;
    try {
      transform_stamped = tf_buffer_->lookupTransform(
            "cameratengah",  // e.g., "base_link"
            "base_lidar",
            tf2::TimePointZero  // or rclcpp::Time(cloud->header.stamp)
          );
    } catch (tf2::TransformException &ex) {
      RCLCPP_WARN(this->get_logger(), "Could not transform: %s", ex.what());
      return;
    }

    // Apply the transform to the point cloud
    pcl::PointCloud<pcl::PointXYZ> pcl_cloud_transformed;
    pcl_ros::transformPointCloud(cloud, pcl_cloud_transformed, transform_stamped);



    /**
     ************* FILTERING PCL ONLY INSIDE CAMERA *************
     */

    // Initialize pin hole camera model
    cam_info_msg_.header = im_msg->header;
    image_geometry::PinholeCameraModel cam_model;
    cam_model.fromCameraInfo(cam_info_msg_);

    // Cloud where the camera see the points
    auto projected_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());

    float max_range = 0.f;

    // Loop through each point in the transformed point cloud
    for (const auto& point : pcl_cloud_transformed.points)
    {
      if (point.x > 0) // Ensure the point is in front of the camera
      {
        // Change the point coordinate system from "x to front" to "z to front"
        cv::Point3d point_3d(-point.y, -point.z, point.x);  

        // Get the pixel coordinates in the image
        cv::Point2d uv = cam_model.project3dToPixel(point_3d);

        if (uv.x >= 0 && uv.x < width_ && uv.y >= 0 && uv.y < height_)
        {
          // // Calculate the depth value (distance from the camera)
          // // Here we use the Euclidean distance from the camera origin
          // // to the point in 3D space
          // float depth_value = std::hypot(point.x, point.y, point.z);

          // // Check if the depth value is within the maximum depth limit
          // if(depth_value > max_depth_) continue;

          // auto& depth_pixel = depth_image.at<float>(uv.y, uv.x);
          // if(depth_pixel > 0)
          //   depth_value = std::min(depth_image.at<float>(uv.y, uv.x), depth_value);

          // // Set the depth value in the depth image
          // // and also set the neighboring pixels to the same depth value
          // // This is to avoid holes in the depth image
          // depth_image.at<float>(uv.y, uv.x) = depth_value;
          // depth_image.at<float>(std::max(uv.y-1, 0.0), uv.x) = depth_value;
          // depth_image.at<float>(uv.y, std::max(uv.x-1, 0.0)) = depth_value;
          // depth_image.at<float>(std::max(uv.y-1, 0.0), std::max(uv.x-1, 0.0)) = depth_value;

          // // Update the maximum range if the current depth value is greater
          // if (depth_value > max_range && std::isfinite(depth_value)) {
          //   max_range = depth_value;
          // }

          // Add the point to the projected cloud
          projected_cloud->points.push_back(point);
        }
      }
    }

    
    // Process the depth image
    // depth_image.setTo(0, depth_image != depth_image); // NaN jadi 0
    // cv::resize(depth_image, depth_image, cv::Size(width, height), 0.5, 0.5, cv::INTER_LINEAR);

    // cv::Mat depth_display;
    // depth_image.convertTo(depth_display, CV_8U, 255.0f / max_range);

    // // Create a mask for the depth image only where point are exists
    // cv::Mat mask_image;
    // cv::threshold(depth_display, mask_image, 1, 255, cv::THRESH_BINARY);

    // // Apply a colormap to the depth image for better visualization
    // cv::applyColorMap(depth_display, depth_display, cv::COLORMAP_JET);

    // // Check if the image message is received
    // // if(!is_) return;



    // /**
    //  ************* PUBLISH DEPTH IMAGE AND DEPTH MAP *************
    //  */

    // // Create an overlay image with the original image
    // auto im = cv_bridge::toCvCopy(im_msg, "bgr8");
    // cv::Mat overlay_image = im->image.clone();

    // depth_display.copyTo(overlay_image, mask_image);
    
    // // Convert to ROS Image message
    // sensor_msgs::msg::Image img_msg;
    // cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", overlay_image).toImageMsg(img_msg);
    // img_msg.header.stamp = pcl_msg->header.stamp;
    // img_msg.header.frame_id = "cameratengah"; //im_msg->header.frame_id;

    // sensor_msgs::msg::Image depth_img_msg;
    // cv_bridge::CvImage(std_msgs::msg::Header(), "32FC1", depth_image).toImageMsg(depth_img_msg);
    // depth_img_msg.header.stamp = pcl_msg->header.stamp;
    // depth_img_msg.header.frame_id = "cameratengah"; //im_msg->header.frame_id;

    // // Publish the depth image
    // depth_map_pub_->publish(img_msg);
    // depth_pub_->publish(depth_img_msg);

    
    /**
     ************* FITERING FLAT SURFACE IN PCL *************
     */

    // Filtering the road in projected cloud
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    tree->setInputCloud(projected_cloud);

    pcl::PointCloud<pcl::PointNormal>::Ptr prj_norm_cloud (new pcl::PointCloud<pcl::PointNormal>);
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::PointNormal> ne;
    ne.setInputCloud(projected_cloud);
    ne.setSearchMethod(tree);
    ne.setViewPoint(std::numeric_limits<float>::max (), std::numeric_limits<float>::max (), std::numeric_limits<float>::max ());
    ne.setKSearch(25);
    ne.compute(*prj_norm_cloud);

    auto proj_filt_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

    for(size_t i = 0; i < prj_norm_cloud->points.size(); ++i)
    {
      auto& point = prj_norm_cloud->points[i];
      // Convert the normal vector to a unit vector
      Eigen::Vector3f normal(point.normal_x, point.normal_y, point.normal_z);
      normal.normalize();

      // Check if the normal vector is pointing upwards (z-axis)
      if (std::abs(normal.z()) < 0.9) {
        proj_filt_cloud->points.push_back(projected_cloud->points[i]);
      }
    }



    /**
     ************* PUBLISH FILTERED PCL *************
     */

    // Convert the projected cloud to a ROS PointCloud2 message
    sensor_msgs::msg::PointCloud2 proj_msg;
    pcl::toROSMsg(*proj_filt_cloud, proj_msg);
    proj_msg.header = header;

    // Publish the projected cloud
    proj_pub_->publish(proj_msg);



    /**
     ************* GET DEPTH INFORMATION FROM PROJECTED PCL *************
     */
    
    // Image with depth values only
    cv::Mat depth_image = cv::Mat::zeros(height_, width_, CV_32FC1);

    for (const auto& point : proj_filt_cloud->points)
    {
      // Change the point coordinate system from "x to front" to "z to front"
      cv::Point3d point_3d(-point.y, -point.z, point.x);  

      // Get the pixel coordinates in the image
      cv::Point2d uv = cam_model.project3dToPixel(point_3d);

      // if (uv.x >= 0 && uv.x < width_ && uv.y >= 0 && uv.y < height_)
      {
        // Calculate the depth value (distance from the camera)
        // Here we use the Euclidean distance from the camera origin
        // to the point in 3D space
        float depth_value = std::hypot(point.x, point.y, point.z);

        // Check if the depth value is within the maximum depth limit
        if(depth_value > max_depth_) continue;

        auto& depth_pixel = depth_image.at<float>(uv.y, uv.x);
        if(depth_pixel > 0)
          depth_value = std::min(depth_image.at<float>(uv.y, uv.x), depth_value);

        // Set the depth value in the depth image
        depth_image.at<float>(uv.y, uv.x) = depth_value;

        // Update the maximum range if the current depth value is greater
        if (depth_value > max_range && std::isfinite(depth_value)) {
          max_range = depth_value;
        }
      }
    }
    
    // Process the depth image
    depth_image.setTo(0, depth_image != depth_image); // NaN jadi 0
    cv::resize(depth_image, depth_image, cv::Size(width_, height_), 0.5, 0.5, cv::INTER_LINEAR);

    cv::Mat depth_display;
    depth_image.convertTo(depth_display, CV_8U, 255.0f / max_range);

    // Create a mask for the depth image only where point are exists
    cv::Mat mask_image;
    cv::threshold(depth_display, mask_image, 1, 255, cv::THRESH_BINARY);

    // Apply a colormap to the depth image for better visualization
    cv::applyColorMap(depth_display, depth_display, cv::COLORMAP_JET);

    // Create an overlay image with the original image
    auto im = cv_bridge::toCvCopy(im_msg, "bgr8");
    cv::Mat overlay_image = im->image.clone();

    depth_display.copyTo(overlay_image, mask_image);




    /**
     ************* PUBLISH DEPTH IMAGE AND DEPTH MAP *************
     */

    // Publish the depth overlay image
    sensor_msgs::msg::Image img_msg;
    cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", overlay_image).toImageMsg(img_msg);
    img_msg.header = header;
    depth_map_pub_->publish(img_msg);

    // Publish the actual depth image
    sensor_msgs::msg::Image depth_img_msg;
    cv_bridge::CvImage(std_msgs::msg::Header(), "32FC1", depth_image).toImageMsg(depth_img_msg);
    depth_img_msg.header = header;
    depth_pub_->publish(depth_img_msg);

  }




public:

  Pcl2DepthNode()
  : Node("pcl2depth_node")
  {
    this->declare_parameter("max_depth", 100.0);
    max_depth_ = this->get_parameter("max_depth").as_double();

    this->declare_parameter("width", 854);
    width_ = this->get_parameter("width").as_int();

    this->declare_parameter("height", 480);
    height_ = this->get_parameter("height").as_int();

    this->declare_parameter("sync_window_ms", 100);
    sync_window_ms_ = this->get_parameter("sync_window_ms").as_int();

    this->declare_parameter("camera_info", "/home/zainir17/ta_ws/src/ode/ode_fusion/config/camera_info.yaml");
    auto camera_info_path = this->get_parameter("camera_info").as_string();

    auto cam_info_config = YAML::LoadFile(camera_info_path);
    cam_info_msg_.width = cam_info_config["image_width"].as<int>();
    cam_info_msg_.height = cam_info_config["image_height"].as<int>();
    cam_info_msg_.k[0] = cam_info_config["camera_matrix"][0].as<double>();
    cam_info_msg_.k[1] = cam_info_config["camera_matrix"][1].as<double>();
    cam_info_msg_.k[2] = cam_info_config["camera_matrix"][2].as<double>();
    cam_info_msg_.k[3] = cam_info_config["camera_matrix"][3].as<double>();
    cam_info_msg_.k[4] = cam_info_config["camera_matrix"][4].as<double>();
    cam_info_msg_.k[5] = cam_info_config["camera_matrix"][5].as<double>();
    cam_info_msg_.k[6] = cam_info_config["camera_matrix"][6].as<double>();
    cam_info_msg_.k[7] = cam_info_config["camera_matrix"][7].as<double>();
    cam_info_msg_.k[8] = cam_info_config["camera_matrix"][8].as<double>();
    cam_info_msg_.r[0] = cam_info_config["rectification_matrix"][0].as<double>();
    cam_info_msg_.r[1] = cam_info_config["rectification_matrix"][1].as<double>();
    cam_info_msg_.r[2] = cam_info_config["rectification_matrix"][2].as<double>();
    cam_info_msg_.r[3] = cam_info_config["rectification_matrix"][3].as<double>();
    cam_info_msg_.r[4] = cam_info_config["rectification_matrix"][4].as<double>();
    cam_info_msg_.r[5] = cam_info_config["rectification_matrix"][5].as<double>();
    cam_info_msg_.r[6] = cam_info_config["rectification_matrix"][6].as<double>();
    cam_info_msg_.r[7] = cam_info_config["rectification_matrix"][7].as<double>(); 
    cam_info_msg_.r[8] = cam_info_config["rectification_matrix"][8].as<double>();

    for (int i = 0; i < 5; ++i) {
      cam_info_msg_.d.push_back(cam_info_config["distortion_coefficients"][i].as<double>());
    }

    cam_info_msg_.p[0] = cam_info_config["projection_matrix"][0].as<double>();
    cam_info_msg_.p[1] = cam_info_config["projection_matrix"][1].as<double>();
    cam_info_msg_.p[2] = cam_info_config["projection_matrix"][2].as<double>();
    cam_info_msg_.p[3] = cam_info_config["projection_matrix"][3].as<double>();
    cam_info_msg_.p[4] = cam_info_config["projection_matrix"][4].as<double>();
    cam_info_msg_.p[5] = cam_info_config["projection_matrix"][5].as<double>();
    cam_info_msg_.p[6] = cam_info_config["projection_matrix"][6].as<double>();
    cam_info_msg_.p[7] = cam_info_config["projection_matrix"][7].as<double>();
    cam_info_msg_.p[8] = cam_info_config["projection_matrix"][8].as<double>();
    cam_info_msg_.p[9] = cam_info_config["projection_matrix"][9].as<double>();
    cam_info_msg_.p[10] = cam_info_config["projection_matrix"][10].as<double>();
    cam_info_msg_.p[11] = cam_info_config["projection_matrix"][11].as<double>();

    cam_info_msg_.distortion_model = cam_info_config["distortion_model"].as<std::string>();
    cam_info_msg_.header.frame_id = "cameratengah";


    this->declare_parameter("sync_queue", 5);
    int sync_queue = this->get_parameter("sync_queue").as_int();

    pcl_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/pcl_concat/lidar_points", 
      rclcpp::SensorDataQoS(), 
      std::bind(&Pcl2DepthNode::callback, this, std::placeholders::_1));
      
    im_sub_.subscribe(this, "/cameratengah/image_rect", rmw_qos_profile_sensor_data);

    im_cache_ = std::make_shared<message_filters::Cache<sensor_msgs::msg::Image>>(im_sub_, 5);
    im_cache_->setCacheSize(sync_queue);

    depth_map_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
      "depth_map_image", rclcpp::SensorDataQoS());

    depth_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
      "depth_image", rclcpp::SensorDataQoS());

    proj_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "projected_cloud", rclcpp::SensorDataQoS());

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
