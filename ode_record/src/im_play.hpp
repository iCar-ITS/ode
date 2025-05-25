#pragma once

#pragma once 

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/srv/set_camera_info.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include "play.hpp"

class ImagePlay : public PlayRecord
{
private:
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub_;
    rclcpp::Service<sensor_msgs::srv::SetCameraInfo>::SharedPtr set_camera_info_srv_;

    sensor_msgs::msg::CameraInfo camera_info_msg_;

    bool is_init_ = false;
    bool has_camera_info_ = false;
    
public:
    ImagePlay(
        const std::string& record_dir,
        const std::string& topic_name,
        rclcpp::Node::SharedPtr& node)
    : PlayRecord(record_dir, topic_name, node)
    {
        // Create publisher
        pub_ = node->create_publisher<sensor_msgs::msg::Image>(
            topic_name,
            5);

        // Remove the last slash and everything after it from the topic name
        std::string info_topic_name;
        auto last_slash_pos = topic_name.find_last_of('/');
        if (last_slash_pos != std::string::npos)
        {
            info_topic_name = topic_name.substr(0, last_slash_pos);
        }
        
        try
        {
            read_camera_info();
            
            info_pub_ = node->create_publisher<sensor_msgs::msg::CameraInfo>(
                info_topic_name + "/camera_info",
                5);
    
            set_camera_info_srv_ = node->create_service<sensor_msgs::srv::SetCameraInfo>(
                info_topic_name + "/set_camera_info",
                std::bind(&ImagePlay::set_camera_info_callback, this,
                            std::placeholders::_1, std::placeholders::_2));
                    
            has_camera_info_ = true;
        } 
        catch (const std::exception& e)
        {
            RCLCPP_ERROR(node->get_logger(), "Error reading camera info: %s", e.what());
            // throw std::runtime_error("Error reading camera info");
        }

    }

    ~ImagePlay() = default;

    virtual void pub_record() final override
    {

        std::string image_file = this->get_dir_topic() + "/data/" + this->get_seq_str() + ".jpg";

        sensor_msgs::msg::Image msg;

        cv::Mat image = cv::imread(image_file, cv::IMREAD_UNCHANGED);
        if(image.empty())
        {
            RCLCPP_DEBUG(node()->get_logger(), "Unable to load image file %s", image_file.c_str());
            return;
        }

        cv_bridge::CvImage cv_image;
        cv_image.image = image;
        cv_image.encoding = sensor_msgs::image_encodings::BGR8;
        cv_image.header.frame_id = this->get_frame_id();
        cv_image.toImageMsg(msg);

        RCLCPP_DEBUG(node()->get_logger(), "Publishing %s", image_file.c_str());

        auto now = node()->now();

        if(has_camera_info_)
        {
            camera_info_msg_.header.stamp = now;
            info_pub_->publish(camera_info_msg_);
        }

        msg.header.stamp = now;

        pub_->publish(msg);
    }

    void set_camera_info_callback(
        [[maybe_unused]] const std::shared_ptr<sensor_msgs::srv::SetCameraInfo::Request> request,
        std::shared_ptr<sensor_msgs::srv::SetCameraInfo::Response> response)
    {
        // Handle the camera info setting here
        // RCLCPP_INFO(node()->get_logger(), "Setting camera info");
        // request;
        response->success = true;
    }

    void read_camera_info() 
    {
        // Read camera info from file
        std::string camera_info_file = this->get_dir_topic() + "/camera_info.yaml";
        
        auto yaml_node = YAML::LoadFile(camera_info_file);
        if (!yaml_node)
        {
            RCLCPP_ERROR(node()->get_logger(), "Unable to open camera info file %s", camera_info_file.c_str());
            throw std::runtime_error("Unable to open camera info file");
        }
        
        camera_info_msg_.header.frame_id = this->get_frame_id();
        camera_info_msg_.header.stamp = node()->now();
        camera_info_msg_.width = yaml_node["image_width"].as<int>();
        camera_info_msg_.height = yaml_node["image_height"].as<int>();
        camera_info_msg_.binning_x = yaml_node["binning_x"].as<int>();
        camera_info_msg_.binning_y = yaml_node["binning_y"].as<int>();
        camera_info_msg_.roi.x_offset = yaml_node["roi"]["x_offset"].as<int>();
        camera_info_msg_.roi.y_offset = yaml_node["roi"]["y_offset"].as<int>();
        camera_info_msg_.roi.width = yaml_node["roi"]["width"].as<int>();
        camera_info_msg_.roi.height = yaml_node["roi"]["height"].as<int>();
        camera_info_msg_.roi.do_rectify = yaml_node["roi"]["do_rectify"].as<bool>();

        camera_info_msg_.distortion_model = yaml_node["distortion_model"].as<std::string>();
        
        RCLCPP_INFO(node()->get_logger(), "Reading camera info from %s", camera_info_file.c_str());
        auto k = yaml_node["camera_matrix"].as<std::vector<double>>();
        std::copy_n(k.begin(), camera_info_msg_.k.size(), camera_info_msg_.k.begin());

        auto d = yaml_node["distortion_coefficients"].as<std::vector<double>>();
        std::copy_n(d.begin(), camera_info_msg_.d.size(), camera_info_msg_.d.begin());

        auto r = yaml_node["rectification_matrix"].as<std::vector<double>>();
        std::copy_n(r.begin(), camera_info_msg_.r.size(), camera_info_msg_.r.begin());

        auto p = yaml_node["projection_matrix"].as<std::vector<double>>();
        std::copy_n(p.begin(), camera_info_msg_.p.size(), camera_info_msg_.p.begin());

        RCLCPP_INFO(node()->get_logger(), "Camera info loaded from %s", camera_info_file.c_str());
    }

};