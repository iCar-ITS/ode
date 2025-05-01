#pragma once 

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <mutex>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <chrono>
#include <filesystem>
#include <memory>
#include <algorithm>
#include <iomanip>
#include <sstream>
#include <atomic>
#include "dobuff.hpp"

class PCLRecord
{
private:
    std::string topic_name_;
    std::string folder_name_;
    std::string frame_id_;
    std::string dir_topic_;
    // sensor_msgs::msg::PointCloud2 msg_;

    // std::mutex mutex_;

    DuoBuffer<sensor_msgs::msg::PointCloud2> buffer_;

    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subs_;
    rclcpp::CallbackGroup::SharedPtr callback_group_;
    
    uint32_t seq_ = 0;
    bool is_init_ = false;
    std::atomic<bool> is_valid_ = false;
protected:
    
public:
    PCLRecord(
        const std::string & topic_name,
        rclcpp::Node::SharedPtr node)
        : topic_name_(topic_name),
        node_(node)
        {

        callback_group_ = node_->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);
        auto options = rclcpp::SubscriptionOptions();
        options.callback_group = callback_group_;
        
        subs_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
            topic_name_,
            5,
            std::bind(&PCLRecord::callback, this, std::placeholders::_1),
            options);

        // std::time_t now = std::time(0);
        // auto tm = *std::localtime(&now);
        // std::ostringstream oss;
        // oss << std::put_time(&tm, "%d%m%Y_%H%M%S");
        folder_name_ = topic_name_;
        std::replace( folder_name_.begin(), folder_name_.end(), '/', '_');

        RCLCPP_INFO(node_->get_logger(), "Lidar topic: %s", topic_name_.c_str());
    }

    ~PCLRecord() = default;

    void callback(const sensor_msgs::msg::PointCloud2& msg)
    {
        // std::lock_guard<std::mutex> lock(mutex_);
        // if(mutex_.try_lock())
        // {
        //     msg_ = msg;
        //     mutex_.unlock();
        // }
        buffer_.set(msg);
        is_valid_ = true;
    }

    sensor_msgs::msg::PointCloud2 get_msg()
    {
        // std::lock_guard<std::mutex> lock(mutex_);
        // return msg_;
        return sensor_msgs::msg::PointCloud2();
    }

    std::string get_topic_name() const
    {
        return topic_name_;
    }

    std::string get_frame_id() const
    {
        return frame_id_;
    }

    void save(
        const std::string& dir, 
        std::chrono::time_point<std::chrono::system_clock> time,
        uint32_t seq) 
    {
        if (!is_valid_)
        {
            RCLCPP_ERROR(node_->get_logger(), "[%s] No valid point cloud data to save.", topic_name_.c_str());
            return;
        }

        seq_ = seq;
        
        // Lock the mutex to ensure thread safety
        // std::lock_guard<std::mutex> lock(mutex_);
        
        // Create string
        std::string seq_str;
        {
            std::stringstream ss;
            ss << std::setw(10) << std::setfill('0') << seq_;
            seq_str = ss.str();
        }

        buffer_.claim();
        auto& msg_ = buffer_.get();
        
        // Check init status
        if (!is_init_)
        {
            dir_topic_ = dir + "/" + folder_name_;
            frame_id_ = msg_.header.frame_id;


            // Create directory if it doesn't exist
            std::filesystem::create_directories(dir_topic_+"/");
            std::filesystem::create_directories(dir_topic_+"/data/");

            // Create metadata
            std::ofstream metadata_file(dir_topic_ + "/metadata.txt", std::ios::app | std::ios::out);
            if (metadata_file.is_open())
            {
                metadata_file << "topic: " << topic_name_ << std::endl
                              << "frame_id: " << frame_id_ << std::endl;
                metadata_file.close();
            }
            else
            {
                RCLCPP_ERROR(node_->get_logger(), "Unable to open file for writing metadata.");
                throw std::runtime_error("Unable to open file for writing metadata.");
            }

            std::cout << "dir " << dir_topic_ << std::endl;
            RCLCPP_INFO(node_->get_logger(), "Topic %s ok!", dir_topic_.c_str());

            is_init_ = true;
        }

        if(!std::filesystem::exists(dir_topic_))
        {
            RCLCPP_ERROR(node_->get_logger(), "Directory does not exist.");
            return;
        }

        // Save point cloud
        {
            // Convert sensor_msgs::msg::PointCloud2 to pcl::PCLPointCloud2
            pcl::PCLPointCloud2 pcl_msg;
            pcl_conversions::toPCL(msg_, pcl_msg);

            pcl::io::savePCDFile(
                dir_topic_ + "/data/" + seq_str + ".pcd",
                pcl_msg);
        }

        // Save timestamp
        {
            int64_t timestamp = time.time_since_epoch().count();
            std::ofstream timestamp_file(dir_topic_ + "/timestamps.txt", std::ios::app);
            if (timestamp_file.is_open())
            {
                timestamp_file << seq_str << ' ' << timestamp << std::endl;
                timestamp_file.close();
            }
            else
            {
                RCLCPP_ERROR(node_->get_logger(), "Unable to open file for writing timestamps.");
            }
        }

        is_valid_ = false;
    }
};