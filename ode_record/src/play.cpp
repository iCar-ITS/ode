#include "rclcpp/rclcpp.hpp"
#include "pcl_play.hpp"
#include "im_play.hpp"
#include <memory>

class Play : public rclcpp::Node
{
private:
  std::vector<std::shared_ptr<PlayRecord>> pub_list_;
  rclcpp::TimerBase::SharedPtr sequence_timer_;

  std::chrono::time_point<std::chrono::system_clock> start_time_;

  double update_period_ = 0.0;

  bool is_sync_ = false;
  bool is_burst_ = false;

public:
  Play() : Node("replay_node")
  {
    this->declare_parameter("record_dir", "");
    this->declare_parameter("lidar_list", std::vector<std::string>());
    this->declare_parameter("camera_list", std::vector<std::string>());
    this->declare_parameter("navsat_topic", std::string());
    this->declare_parameter("period", 0.0);
    this->declare_parameter("repeat", false);
    this->declare_parameter("burst", false);
    this->declare_parameter("burst_period", 1.0);
  }

  void init() 
  {
    auto shared_this = this->shared_from_this();

    if(!this->has_parameter("record_dir"))
    {
      RCLCPP_ERROR(this->get_logger(), "Record directory is not set.");
      throw std::runtime_error("Record directory is not set.");
    }

    auto dir = this->get_parameter("record_dir").as_string();

    RCLCPP_INFO(this->get_logger(), "Record directory: %s", 
    dir.c_str());

    // listing lidar topics 
    auto lidar_list = this->get_parameter("lidar_list").as_string_array();
    for(auto& lidar : lidar_list)
    {
      std::shared_ptr<PlayRecord> play_obj = std::make_shared<PCLPlay>(dir, lidar, shared_this);
      pub_list_.emplace_back(play_obj);
    }

    // listing camera topics 
    auto cam_list = this->get_parameter("camera_list").as_string_array();
    for(auto& cam : cam_list)
    {
      std::shared_ptr<PlayRecord> play_obj = std::make_shared<ImagePlay>(dir, cam, shared_this);
      pub_list_.emplace_back(play_obj);
    }

    // listing navsat topic
    // auto navsat_topic = this->get_parameter("navsat_topic").as_string();
    // if(!navsat_topic.empty())
    // {
    //   navsat_subs_ = std::make_shared<NSRecord>(navsat_topic, this->shared_from_this());
    // }

    // Create wall timer with period based on frequency parameter
    is_burst_ = this->get_parameter("burst").as_bool();
    update_period_ = this->get_parameter("period").as_double();
    auto time_period = std::chrono::microseconds();
    if(is_burst_)
    {
      RCLCPP_INFO(this->get_logger(), "Burst mode is enabled.");
      auto burst_period = this->get_parameter("burst_period").as_double();
      is_sync_ = true;
      time_period = std::chrono::microseconds(int64_t(burst_period * 1000000));
      RCLCPP_INFO(this->get_logger(), "Play period\t: %fs.", update_period_);
    }
    else if(update_period_ > 0.0)
    {
      is_sync_ = true;
      time_period = std::chrono::microseconds(int64_t(update_period_ * 1000000));
      RCLCPP_INFO(this->get_logger(), "Play period\t: %fs.", update_period_);
    }
    else 
    {
      is_sync_ = false;
      time_period = std::chrono::microseconds(100000);
      RCLCPP_INFO(this->get_logger(), "Publishing topic based on timestamp");
    }
    
    sequence_timer_ = this->create_wall_timer(
      time_period, 
      std::bind(&Play::main_loop, this));

    RCLCPP_INFO(this->get_logger(), "Replay node has been started.");
  }

  void main_loop()
  {
    static uint32_t seq = 0;
    if(is_sync_)
    {
      for(auto& pub : pub_list_)
      {
        pub->pub_sync(seq);
      }
    }
    else
    {
      if(seq == 0)
      {
        auto start_time = std::chrono::system_clock::now();
        for(auto& pub : pub_list_)
        {
          pub->start(start_time);
        }
      }
    }

    bool is_finish = true;
      for(auto& pub : pub_list_)
      {
        is_finish &= pub->is_finish();
      }

      if(is_finish)
      {
        if(this->get_parameter("repeat").as_bool() && is_sync_)
        {
          seq = 0;
          is_finish = false;
        }
        else 
        {
          RCLCPP_INFO(this->get_logger(), "All topics have been finished.");
          rclcpp::shutdown();
          while(rclcpp::ok());
        }
      }

    if(is_burst_) {
      auto duration = std::chrono::system_clock::now() - start_time_;
      if(duration > std::chrono::microseconds(int64_t(update_period_ * 1000000)))
      {
        start_time_ = std::chrono::system_clock::now();
        seq++;
      }
    }
    else {
      seq++;
    }
  }

};  // class Play

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Play>();
  node->init();
  
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  return 0;
}