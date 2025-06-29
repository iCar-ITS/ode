#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <sensor_msgs/msg/image.hpp>

class RectifyNode : public rclcpp::Node
{
  private:
    std::string topic_;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;

    cv::Mat cam_mtx_;
    cv::Mat dist_coeff_mtx_;
    
    cv::Mat map1, map2;

    bool is_init_ = false;

  public:
    RectifyNode()
    : Node("rectify_node")
    {
      this->declare_parameter("topic_name", "/image");
      topic_ = this->get_parameter("topic_name").as_string();

      RCLCPP_INFO(this->get_logger(), "Topic name: %s", topic_.c_str());

      this->declare_parameter("image_width", 0);

      this->declare_parameter("camera_matrix", std::vector<double>{0.0});
      this->declare_parameter("dist_coeff", std::vector<double>{0.0});

      cam_mtx_.create(3,3, CV_64F);
      std::vector<double> cam_mtx_vec;
      this->get_parameter("camera_matrix", cam_mtx_vec);
      for(int x = 0; x < 3; ++x)
      {
        for(int y = 0; y < 3; ++y)
        {
          cam_mtx_.at<double>(x,y) = cam_mtx_vec[x*3+y];
        }
      }

      std::vector<double> dist_coeff_vec;
      this->get_parameter("dist_coeff", dist_coeff_vec);
      dist_coeff_mtx_.create(1,dist_coeff_vec.size(), CV_64F);
      for (int i = 0; i < dist_coeff_vec.size(); ++i)
      {
        dist_coeff_mtx_.at<double>(0,i) = dist_coeff_vec[i];
      }

      pub_ = this->create_publisher<sensor_msgs::msg::Image>(topic_+"/image_rect", rclcpp::SensorDataQoS());

      sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        topic_+"/image_raw", rclcpp::SensorDataQoS(), std::bind(&RectifyNode::callback, this, std::placeholders::_1));
        
    }

    cv::Mat scale_image(const cv::Mat& image, float scale_factor)
    {
      cv::Size new_size;
      new_size.width = static_cast<int>(image.cols * scale_factor);
      new_size.height = static_cast<int>(image.rows * scale_factor);

      cv::Mat scaled_image;
      cv::resize(image, scaled_image, new_size);
      return scaled_image;
    }

    cv::Mat resize_image(const cv::Mat& image, int new_width, int new_height = 0)
    {
      if(new_width == 0) return image;
      if (new_height == 0)
      {
        new_height = static_cast<int>(image.rows * (static_cast<float>(new_width) / image.cols));
      }
      
      cv::Size new_size(new_width, new_height);
      cv::Mat resized_image;
      cv::resize(image, resized_image, new_size);
      return resized_image;
    }

    void callback(const sensor_msgs::msg::Image& msg) 
    {
      auto im = cv_bridge::toCvCopy(msg);
      if (!im)
      {
          RCLCPP_ERROR(this->get_logger(), "Failed to convert image.");
          return;
      }
      cv::Mat image;
      try
      {
        cv::cvtColor(im->image, image, cv::COLOR_YUV2BGR_YUYV);
      }
      catch (const cv::Exception& e)
      {
        image = im->image;
      }
      image = resize_image(image, this->get_parameter("image_width").as_int());

      if(!is_init_)
      {
        auto new_mtx = cv::getOptimalNewCameraMatrix(cam_mtx_, dist_coeff_mtx_, image.size(), 0, image.size(), 0);
        
        cv::initUndistortRectifyMap(
          cam_mtx_, dist_coeff_mtx_, cv::Mat(),
          new_mtx, image.size(),
          CV_16SC2, map1, map2);

        is_init_ = true;
        RCLCPP_INFO(this->get_logger(), "Camera matrix: %s", topic_.c_str());
      }
        
      cv::Mat rect_image;

      cv::remap(image, rect_image, map1, map2, cv::INTER_LINEAR);

      sensor_msgs::msg::Image rect_msg = msg;
      
      cv_bridge::CvImage cv_image;
      cv_image.image = rect_image;
      cv_image.encoding = "bgr8";
      cv_image.toImageMsg(rect_msg);

      rect_msg.header = msg.header;
      
      pub_->publish(rect_msg);
    }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RectifyNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
