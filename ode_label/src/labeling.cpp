#include <algorithm>
#include <iostream>
#include <fstream>
#include <filesystem>
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <boundingboxes/msg/bounding_boxes.hpp>

#include <message_filters/synchronizer.hpp>
#include <message_filters/subscriber.hpp>
#include <message_filters/sync_policies/approximate_time.hpp>
#include <message_filters/message_traits.hpp>
#include <message_filters/cache.hpp>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

class LabelingNode : public rclcpp::Node
{
private:

  struct LabelData
  {
    uint32_t id;
    float depth_value;
    float x_center;
    float y_center;
    float width;
    float height;
  };

  using SyncPolicy = message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::Image,
    boundingboxes::msg::BoundingBoxes
    >;

  using Sync = message_filters::Synchronizer<SyncPolicy>;

  message_filters::Subscriber<sensor_msgs::msg::Image> depth_image_sub_;
  message_filters::Subscriber<sensor_msgs::msg::Image> rgb_image_sub_;
  message_filters::Subscriber<boundingboxes::msg::BoundingBoxes> boxes_sub_;

  std::shared_ptr<message_filters::Cache<boundingboxes::msg::BoundingBoxes>> boxes_cache_;
  std::shared_ptr<message_filters::Cache<sensor_msgs::msg::Image>> rgb_cache_;

  std::shared_ptr<Sync> sync_;

  std::vector<std::string> classes_ = {"person", "bicycle", "car", "motorcycle", "truck", "bus"};

  std::string save_path_;

  uint64_t seq_;
  uint32_t k_max_value_;
  uint32_t k_min_value_;
  float max_distance_;
  float min_distance_;

  rclcpp::Duration abs_time_diff(const rclcpp::Time& t1, const rclcpp::Time& t2)
  {
    return (t1 > t2) ? (t1 - t2) : (t2 - t1);
  }

  // void callback(
  //   const sensor_msgs::msg::Image::ConstSharedPtr & im_msg,
  //   const boundingboxes::msg::BoundingBoxes::ConstSharedPtr & boxes_msg)
  void callback(
    const sensor_msgs::msg::Image::ConstSharedPtr & im_msg)
  {
    // Sync the depth image and bboxes messages
    rclcpp::Time b_time = im_msg->header.stamp;

    rclcpp::Duration window(0, 100 * 1000000);  // ±150 ms window
    auto bb_candidates = boxes_cache_->getInterval(b_time - window, b_time + window);
    if (bb_candidates.empty()) {
      RCLCPP_DEBUG(this->get_logger(), "No image message found near point cloud timestamp");
      return;
    }

    auto rgb_candidates = rgb_cache_->getInterval(b_time - window, b_time + window);
    if (rgb_candidates.empty()) {
      RCLCPP_DEBUG(this->get_logger(), "No RGB image message found near point cloud timestamp");
      return;
    }

    boundingboxes::msg::BoundingBoxes::ConstSharedPtr boxes_msg = nullptr;
    rclcpp::Duration min_diff = rclcpp::Duration::from_seconds(1000);
    for (auto& msg : bb_candidates) {
      rclcpp::Duration diff = abs_time_diff(b_time, msg->header.stamp);
      if (diff < min_diff) {
        min_diff = diff;
        boxes_msg = msg;
      }
    }

    sensor_msgs::msg::Image::ConstSharedPtr rgb_msg = nullptr;
    min_diff = rclcpp::Duration::from_seconds(1000);
    for (auto& msg : rgb_candidates) {
      rclcpp::Duration diff = abs_time_diff(b_time, msg->header.stamp);
      if (diff < min_diff) {
        min_diff = diff;
        rgb_msg = msg;
      }
    }

    RCLCPP_DEBUG(this->get_logger(), "Processing image at time %s with %zu bounding boxes",
        im_msg->header.frame_id.c_str(), boxes_msg->bounding_boxes.size());

    // Convert ROS Image message to cv::Mat
    cv::Mat depth_image;
    cv::Mat bgr_image;
    try {
      cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(im_msg, sensor_msgs::image_encodings::TYPE_32FC1);
      depth_image = cv_ptr->image;

      cv_bridge::CvImagePtr cv_bgr_ptr = cv_bridge::toCvCopy(rgb_msg, sensor_msgs::image_encodings::BGR8);
      bgr_image = cv_bgr_ptr->image;
    } catch (const cv_bridge::Exception &e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    std::vector<LabelData> labels;

    for(auto& box : boxes_msg->bounding_boxes) {

      // Only process boxes with class IDs that are in the specified classes
      auto it = std::find(classes_.begin(), classes_.end(), box.class_id);
      if(it == classes_.end()) continue;
      uint32_t class_index = std::distance(classes_.begin(), it);

      // Extract ROI (Region of Interest) from the image using bounding box
      cv::Rect roi(box.xmin, box.ymin, box.xmax - box.xmin, box.ymax - box.ymin);

      // Ensure ROI is within image bounds
      roi &= cv::Rect(0, 0, depth_image.cols, depth_image.rows);

      cv::Mat cropped_image;

      if (roi.width > 0 && roi.height > 0) {
        cropped_image = depth_image(roi);

        // Process the cropped image or save it as needed
        RCLCPP_DEBUG(this->get_logger(), "Extracted ROI: x=%d, y=%d, width=%d, height=%d",
            roi.x, roi.y, roi.width, roi.height);
      } else {
        RCLCPP_DEBUG(this->get_logger(), "Invalid ROI, skipping...");
        continue;
      }

      // Remove all zero values from the ROI image
      std::vector<float> non_zero_values;
      non_zero_values.reserve(cropped_image.rows * cropped_image.cols);
      for (int i = 0; i < cropped_image.rows; ++i) {
        const float* row_ptr = cropped_image.ptr<float>(i);
        for (int j = 0; j < cropped_image.cols; ++j) {
          if (row_ptr[j] > min_distance_) {
            non_zero_values.push_back(row_ptr[j]);
          }
        }
      }

      // Find the mean of the k smallest values
      uint32_t k = k_max_value_; // Adjust 'k' as needed

      if (non_zero_values.size() < k_min_value_) continue;
      while(non_zero_values.size() < k && k > 2) k /= 2;

      std::nth_element(non_zero_values.begin(), non_zero_values.begin() + k, non_zero_values.end());
      float mean = std::accumulate(non_zero_values.begin(), non_zero_values.begin() + k, 0.0f) / (float)k;
      RCLCPP_DEBUG(this->get_logger(), "Mean of %zu smallest non-zero values: %f", k, mean);

      if(mean > max_distance_) {
        // RCLCPP_DEBUG(this->get_logger(), "Mean depth value %f exceeds max distance %f, skipping box", mean, max_distance_);
        continue;
      }

      std::cout << "Class: " << box.class_id
                << ", Bounding Box: [" << box.xmin << ", " << box.ymin
                << ", " << box.xmax << ", " << box.ymax << "]"
                << ", Mean Depth: " << mean
                << std::endl;

      // Generate YOLO-style annotation
      float x_center = (box.xmin + box.xmax) / 2.0f / depth_image.cols;
      float y_center = (box.ymin + box.ymax) / 2.0f / depth_image.rows;
      float width = (box.xmax - box.xmin) / (float)depth_image.cols;
      float height = (box.ymax - box.ymin) / (float)depth_image.rows;

      labels.push_back({class_index, mean, x_center, y_center, width, height});
    }

    // if(labels.empty()) {
    //   RCLCPP_DEBUG(this->get_logger(), "No valid labels found, skipping saving data");
    //   return;
    // }

    std::string seq_str;
    {
        std::stringstream ss;
        ss << std::setw(10) << std::setfill('0') << seq_;
        seq_str = ss.str();
    }

    // Save as dataset
    std::ofstream dataset_file(save_path_ + "/data/" + seq_str + ".txt", std::ios::trunc | std::ios::out);
    if (!dataset_file.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open dataset file for writing");
      return;
    }
    for (const auto& label : labels) {
      dataset_file << label.id << " "
                   << label.x_center << " "
                   << label.y_center << " "
                   << label.width << " "
                   << label.height << " "
                   << label.depth_value << "\n";
    }
    cv::imwrite(save_path_ + "/data/" + seq_str + ".jpg", bgr_image);

    // Save the labeled image with bounding boxes and annotations
    cv::Mat labeled_image = bgr_image.clone();
    for (const auto& label : labels) {
      std::stringstream label_stream;
      label_stream << classes_[label.id] << " " << std::setprecision(2) << label.depth_value << "m";

      int xmin = static_cast<int>((label.x_center - label.width / 2.0f) * depth_image.cols);
      int ymin = static_cast<int>((label.y_center - label.height / 2.0f) * depth_image.rows);
      int xmax = static_cast<int>((label.x_center + label.width / 2.0f) * depth_image.cols);
      int ymax = static_cast<int>((label.y_center + label.height / 2.0f) * depth_image.rows);

      cv::rectangle(labeled_image, cv::Point(xmin, ymin), cv::Point(xmax, ymax), cv::Scalar(0, 255, 0), 2);
      cv::putText(labeled_image, label_stream.str(), cv::Point(xmin, ymin - 10), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 255, 0), 2);
    }

    cv::imwrite(save_path_ + "/annotated_data/" + seq_str + ".jpg", labeled_image);
    

    seq_ += 1;
  }
public:
  LabelingNode() : Node("labeling_node"), seq_(0)
  {
    this->declare_parameter("save_path", std::string());
    this->declare_parameter("start_seq", 0);
    this->declare_parameter("k_max_value", 32);
    this->declare_parameter("k_min_value", 8);
    this->declare_parameter("max_distance", 50.0);
    this->declare_parameter("min_distance", 1.0);

    k_max_value_ = this->get_parameter("k_max_value").as_int();
    k_min_value_ = this->get_parameter("k_min_value").as_int();
    seq_ = this->get_parameter("start_seq").as_int();
    max_distance_ = this->get_parameter("max_distance").as_double();
    min_distance_ = this->get_parameter("min_distance").as_double();


    save_path_ = this->get_parameter("save_path").as_string();

    if (save_path_.empty()) {
      RCLCPP_ERROR(this->get_logger(), "Save path is not set. Please set the 'save_path' parameter.");
      return;
    }

    std::time_t now = std::time(0);
    auto tm = *std::localtime(&now);
    std::ostringstream oss;
    oss << "/" << std::put_time(&tm, "%Y%m%d_%H%M%S");
    save_path_ += oss.str();

    std::filesystem::create_directories(save_path_+"/");
    std::filesystem::create_directories(save_path_+"/data/");
    std::filesystem::create_directories(save_path_+"/annotated_data/");

    depth_image_sub_.subscribe(this, "/depth_image", rmw_qos_profile_sensor_data);
    depth_image_sub_.registerCallback(std::bind(&LabelingNode::callback, this, std::placeholders::_1));

    rgb_image_sub_.subscribe(this, "/cameratengah/image_rect", rmw_qos_profile_sensor_data);
    rgb_cache_ = std::make_shared<message_filters::Cache<sensor_msgs::msg::Image>>(rgb_image_sub_, 4);

    boxes_sub_.subscribe(this, "/yolov5/image/bounding_boxes", rmw_qos_profile_sensor_data);
    boxes_cache_ = std::make_shared<message_filters::Cache<boundingboxes::msg::BoundingBoxes>>(boxes_sub_, 4);
    // sync_ = std::make_shared<Sync>(SyncPolicy(10), image_sub_, boxes_sub_);

    // sync_->registerCallback(std::bind(&LabelingNode::callback, this,
    //   std::placeholders::_1, std::placeholders::_2));
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LabelingNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
