#pragma once 

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <mutex>
#include <chrono>
#include <filesystem>
#include <memory>
#include <algorithm>
#include <iomanip>
#include <sstream>
#include <fstream>
#include <iostream>

class NSRecord
{
private:
    std::string topic_name_;
    std::string folder_name_;
    std::string frame_id_;
    sensor_msgs::msg::NavSatFix msg_;

    std::mutex mutex_;

    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr subs_;
    rclcpp::CallbackGroup::SharedPtr callback_group_;
    
    uint32_t seq_ = 0;
    bool is_valid_ = false;
    bool is_init_ = false;
protected:
    
public:
    NSRecord(
        const std::string & topic_name,
        rclcpp::Node::SharedPtr node)
    : topic_name_(topic_name),
    node_(node)
    {
        callback_group_ = node_->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);
        auto options = rclcpp::SubscriptionOptions();
        options.callback_group = callback_group_;

        subs_ = node_->create_subscription<sensor_msgs::msg::NavSatFix>(
            topic_name_,
            rclcpp::SensorDataQoS(),
            std::bind(&NSRecord::callback, this, std::placeholders::_1),
            options);

        folder_name_ = topic_name_;
        std::replace( folder_name_.begin(), folder_name_.end(), '/', '_');

        RCLCPP_INFO(node_->get_logger(), "NavSat topic: %s", topic_name_.c_str());
    }

    ~NSRecord() = default;

    void callback(const sensor_msgs::msg::NavSatFix& msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        msg_ = msg;
        is_valid_ = true;
    }

    sensor_msgs::msg::NavSatFix get_msg()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return msg_;
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
        std::chrono::time_point<std::chrono::system_clock> time) 
    {
        if (!is_valid_)
        {
            RCLCPP_ERROR(node_->get_logger(), "[%s] No valid point cloud data to save.", topic_name_.c_str());
            return;
        }

        // Lock the mutex to ensure thread safety
        std::lock_guard<std::mutex> lock(mutex_);

        // Create string
        static std::string dir_topic = dir + "/" + folder_name_;
        std::string seq_str;
        {
            std::stringstream ss;
            ss << std::setw(10) << std::setfill('0') << seq_;
            seq_str = ss.str();
        }

        // Check init status
        if (!is_init_)
        {
            frame_id_ = msg_.header.frame_id;

            // Create directory if it doesn't exist
            std::filesystem::create_directories(dir_topic+"/");

            // GNS service string
            std::string gns_service;
            if(msg_.status.status == sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX)
            {
                gns_service = "no_fix";
            }
            else if(msg_.status.status == sensor_msgs::msg::NavSatStatus::STATUS_FIX)
            {
                gns_service = "fix";
            }
            else if(msg_.status.status == sensor_msgs::msg::NavSatStatus::STATUS_SBAS_FIX)
            {
                gns_service = "sbas_fix";
            }
            else
            {
                gns_service = "unknown";
            }

            // Create metadata
            std::ofstream metadata_file(dir_topic + "/metadata.txt", std::ios::app | std::ios::out);
            if (metadata_file.is_open())
            {
                metadata_file << "topic: " << topic_name_ << std::endl
                              << "frame_id: " << frame_id_ << std::endl
                              << "service: " << gns_service << std::endl;
                metadata_file.close();
            }
            else
            {
                RCLCPP_ERROR(node_->get_logger(), "Unable to open file for writing metadata.");
                throw std::runtime_error("Unable to open file for writing metadata.");
            }

            // Create directory for coordinate and timestamp
            std::ofstream coord_file(dir_topic + "/coordinate.txt", std::ios::trunc);
            if (coord_file.is_open())
            {
                coord_file << std::left << std::setw(14) << "seq";
                coord_file << std::left  << std::setw(20) << "latitude";
                coord_file << std::left  << std::setw(20) << "longitude";
                coord_file << std::left  << std::setw(20) << "altitude";
                coord_file << std::left  << std::setw(10) << "cov_type";
                coord_file << std::left  << std::setw(10) << "covariance" << std::endl;
                coord_file.close();
            }
            else
            {
                RCLCPP_ERROR(node_->get_logger(), "Unable to open file for writing coordinates.");
            }

            // Check if directory exists
            if(!std::filesystem::exists(dir_topic))
            {
                RCLCPP_ERROR(node_->get_logger(), "Directory does not exist.");
                throw std::runtime_error("Directory does not exist.");
            }

            // OK
            RCLCPP_INFO(node_->get_logger(), "Directory created: %s", dir_topic.c_str());
            is_init_ = true;
        }

        // Save coordinate
        {
            static std::string coor_dir = dir_topic + "/coordinate.txt";
            std::ofstream coord_file(coor_dir, std::ios::app);
            if (coord_file.is_open())
            {
                coord_file << std::left  << std::setw(14) << seq_str;
                coord_file << std::left  << std::setw(20) << msg_.latitude;
                coord_file << std::left  << std::setw(20) << msg_.longitude;
                coord_file << std::left  << std::setw(20) << msg_.altitude;
                coord_file << std::left  << std::setw(10) << int(msg_.position_covariance_type);
                coord_file << std::left  << "[";
                for(size_t i = 0; i < msg_.position_covariance.size(); ++i)
                {
                    if(i != 0)
                    {
                        coord_file << ", ";
                    }
                    coord_file << msg_.position_covariance[i];
                }
                coord_file  << "]";
                coord_file  << std::endl;
                coord_file.close();
            }
            else
            {
                RCLCPP_ERROR(node_->get_logger(), "Unable to open file for writing coordinates.");
            }
        }

        // Save timestamp
        {
            int64_t timestamp = time.time_since_epoch().count();
            static std::string ts_dir = dir_topic + "/timestamps.txt";
            std::ofstream timestamp_file(ts_dir, std::ios::app);
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

        seq_ += 1;

        is_valid_ = false;
    }
};