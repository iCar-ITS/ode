#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <vector>
#include "pcl_record.hpp"
#include "im_record.hpp"
#include "navsat_record.hpp"

class Record : public rclcpp::Node
{
private:
  rclcpp::TimerBase::SharedPtr sequence_timer_;

  std::vector<std::shared_ptr<PCLRecord>> pcl_subs_list_;
  std::vector<std::shared_ptr<ImRecord>> cam_subs_list_;
  std::shared_ptr<NSRecord> navsat_subs_;

  std::string start_time_;
  std::string directory_;

public:
  Record() : Node("record_node")
  {
    this->declare_parameter("output_dir", "");
    this->declare_parameter("frequency", 1.0);
    this->declare_parameter("lidar_list", std::vector<std::string>());
    this->declare_parameter("camera_list", std::vector<std::string>());
    this->declare_parameter("navsat_topic", std::string());
  }

  void init() 
  {
    if(!this->has_parameter("output_dir"))
    {
      RCLCPP_ERROR(this->get_logger(), "Output directory is not set.");
      throw std::runtime_error("Output directory is not set.");
    }

    RCLCPP_INFO(this->get_logger(), "Output directory: %s", 
      this->get_parameter("output_dir").as_string().c_str());

    // listing lidar topics 
    auto lidar_list = this->get_parameter("lidar_list").as_string_array();
    for(auto& lidar : lidar_list)
    {
      pcl_subs_list_.emplace_back(std::make_shared<PCLRecord>(lidar, this->shared_from_this()));
    }

    // listing camera topics 
    auto cam_list = this->get_parameter("camera_list").as_string_array();
    for(auto& cam : cam_list)
    {
      cam_subs_list_.emplace_back(std::make_shared<ImRecord>(cam, this->shared_from_this()));
    }

    // listing navsat topic
    auto navsat_topic = this->get_parameter("navsat_topic").as_string();
    if(!navsat_topic.empty())
    {
      navsat_subs_ = std::make_shared<NSRecord>(navsat_topic, this->shared_from_this());
    }

    // Create wall timer with period based on frequency parameter
    {
      auto samp_freq = this->get_parameter("frequency").as_double();
      auto samp_time = 1.00 / samp_freq;
      uint32_t time_us = samp_time * 1000000;
      auto time_c_us = std::chrono::microseconds(time_us);
      sequence_timer_ = this->create_wall_timer(
        time_c_us, 
        std::bind(&Record::main_loop, this));
      RCLCPP_INFO(this->get_logger(), "Recording period\t: %fs.", samp_time);
    }


    std::time_t now = std::time(0);
    auto tm = *std::localtime(&now);
    std::ostringstream oss;
    oss << std::put_time(&tm, "%d%m%Y_%H%M%S");
    start_time_ = oss.str();

    directory_ = this->get_parameter("output_dir").as_string();
    directory_ += "/ode_record_" + start_time_;

    RCLCPP_INFO(this->get_logger(), "Record node has been started.");
  }

  void main_loop()
  {
    for(auto& i : pcl_subs_list_)
    {
      i->save(directory_, std::chrono::system_clock::now());
    }
    for(auto& i : cam_subs_list_)
    {
      i->save(directory_, std::chrono::system_clock::now());
    }
    if(navsat_subs_)
    {
      navsat_subs_->save(directory_, std::chrono::system_clock::now());
    }
  }
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Record>();
  node->init();
  
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}