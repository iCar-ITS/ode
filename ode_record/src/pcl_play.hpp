#pragma once

#pragma once 

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include "play.hpp"

class PCLPlay : public PlayRecord
{
private:
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
    
public:
    PCLPlay(
        const std::string& record_dir,
        const std::string& topic_name,
        rclcpp::Node::SharedPtr& node)
    : PlayRecord(record_dir, topic_name, node)
    {
        // Create publisher
        pub_ = node->create_publisher<sensor_msgs::msg::PointCloud2>(
            topic_name,
            5);
    }

    ~PCLPlay() = default;

    virtual void pub_record() final override
    {
        std::string pcd_file = this->get_dir_topic() + "/data/" + this->get_seq_str() + ".pcd";

        sensor_msgs::msg::PointCloud2 msg;

        if(pcl::io::loadPCDFile(pcd_file, msg) != 0)
        {
            RCLCPP_DEBUG(node()->get_logger(), "Unable to load PCD file %s", pcd_file.c_str());
            return;
        }

        msg.header.frame_id = this->get_frame_id();
        msg.header.stamp = node()->now();

        RCLCPP_DEBUG(node()->get_logger(), "Publishing %s", pcd_file.c_str());

        pub_->publish(msg);
    }

};