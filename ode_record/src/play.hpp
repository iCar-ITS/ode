#pragma once

#include <rclcpp/rclcpp.hpp>
#include <mutex>
#include <chrono>
#include <filesystem>
#include <memory>
#include <algorithm>
#include <iomanip>
#include <sstream>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <exception>

class PlayRecord
{
private:
    std::string topic_name_;
    std::string dir_topic_;
    std::string frame_id_;

    rclcpp::Node::SharedPtr node_;
    std::chrono::time_point<std::chrono::system_clock> pub_time_;
    std::vector<int64_t> timestamps_;

    uint32_t seq_ = 0;
    
    std::atomic<bool> is_finish_ = false;
protected:
    
public:
    PlayRecord(
        const std::string & record_dir,
        const std::string & topic_name,
        rclcpp::Node::SharedPtr& node)
    : topic_name_(topic_name),
    node_(node)
    {
        // check if the directory exists
        std::string folder_name_ = topic_name_;
        std::replace( folder_name_.begin(), folder_name_.end(), '/', '_');
        
        dir_topic_ = record_dir + "/" + folder_name_;
        if(!std::filesystem::exists(dir_topic_))
        {
            RCLCPP_ERROR(node_->get_logger(), "Directory %s does not exist", dir_topic_.c_str());
            throw std::runtime_error("Directory does not exist. . Abort!");
        }

        //load metadata
        auto meta_file = YAML::LoadFile(dir_topic_ + "/metadata.txt");

        if(meta_file["topic"].as<std::string>() != topic_name_)
        {
            RCLCPP_ERROR(node_->get_logger(), "Topic name mismatch. Expected %s, got %s", 
                topic_name_.c_str(), meta_file["topic"].as<std::string>().c_str());
            throw std::runtime_error("Topic name mismatch. Abort!");
        }

        if(!meta_file["frame_id"])
        {
            RCLCPP_ERROR(node_->get_logger(), "Frame ID not found in metadata.");
            throw std::runtime_error("Frame ID not found in metadata. Abort!");
        }
        frame_id_ = meta_file["frame_id"].as<std::string>();

        //load timestamps
        load_timestamp();

        RCLCPP_INFO(node_->get_logger(), "Topic: %s", topic_name_.c_str());
    }

    ~PlayRecord() = default;

    void start(std::chrono::time_point<std::chrono::system_clock>& time)
    {
        pub_time_ = time;
        std::thread(&PlayRecord::loop, this).detach();
    }

    void loop()
    {
        RCLCPP_INFO(node_->get_logger(), "Starting thread for replaying %s", topic_name_.c_str());
        while(!is_finish_ && rclcpp::ok())
        {
            auto start_time = std::chrono::system_clock::now();

            publish();
            
            int64_t dur_int = 0;
            std::chrono::nanoseconds pub_dur;
            uint32_t next_seq = seq_ + 1;
            
            while(rclcpp::ok())
            {
                if(next_seq >= timestamps_.size())
                {
                    RCLCPP_ERROR(node_->get_logger(), "No more timestamps available.");
                    is_finish_ = true;
                    break;
                }

                if(timestamps_[next_seq] == 0)
                {
                    RCLCPP_ERROR(node_->get_logger(), "Sequence number %d in topic %s was skipped while recording.", next_seq, topic_name_.c_str());
                    next_seq++;
                    continue;
                }

                if(is_finish_) break;

                dur_int = timestamps_[next_seq] - timestamps_[seq_]; 
                pub_dur = std::chrono::system_clock::now() - start_time;
                if(pub_dur > std::chrono::nanoseconds(dur_int))
                {
                    RCLCPP_WARN(node_->get_logger(), "Publishing took longer than expected. Skipping to next timestamp.");
                    next_seq++;
                    continue;
                }
                else break;
            }

            if(is_finish_) break;
            
            seq_ = next_seq;

            auto dur_c = std::chrono::nanoseconds(dur_int)-pub_dur;
            float dur_f = dur_c.count() / 1e9;
            RCLCPP_DEBUG(node_->get_logger(), "Thread %s sleep for %f.", topic_name_.c_str(), dur_f);

            std::this_thread::sleep_for(std::chrono::nanoseconds(dur_int)-pub_dur);
        }
        RCLCPP_INFO(node_->get_logger(), "Topic %s has been finished.", topic_name_.c_str());
    }

    void pub_sync(uint32_t seq) 
    {
        seq_ = seq;
        std::thread(&PlayRecord::publish, this).detach();
    }

    void load_timestamp() 
    {
        std::string timestamp_file = dir_topic_ + "/timestamps.txt";

        std::ifstream file(timestamp_file);
        if (!file.is_open())
        {
            RCLCPP_ERROR(node_->get_logger(), "Unable to open file %s", timestamp_file.c_str());
            throw std::runtime_error("Unable to open file. Abort!");
        }

        std::string line;
        uint32_t seq = 0;
        while (std::getline(file, line))
        {
            std::istringstream iss(line);
            
            uint32_t seq_num;
            iss >> seq_num;

            int64_t timestamp;
            iss >> timestamp;
            try 
            { 
                timestamps_.at(seq_num) = timestamp; 
            }
            catch (const std::out_of_range& e)
            {
                timestamps_.resize(seq_num+1, 0);
                timestamps_.at(seq_num) = timestamp; 
            }

            // if(seq_num != seq)
            // {
            //     try { timestamps_.at(seq_num); }
            //     catch (const std::out_of_range& e)
            //     {
            //         timestamps_.push_back(0);
            //     }
            // }
            // else 
            // {
            //     int64_t timestamp;
            //     iss >> timestamp;
            //     timestamps_.push_back(timestamp);
            // }
                
            seq++;
        }

        file.close();

        RCLCPP_INFO(node_->get_logger(), "Loaded %zu timestamps from %s", timestamps_.size(), timestamp_file.c_str());
    }

    void publish() 
    {
        if(seq_ >= timestamps_.size())
        {
            is_finish_ = true;
            RCLCPP_ERROR(node_->get_logger(), "No more timestamps available.");
            return;
        }
        this->pub_record();
    }

    const std::string& get_topic_name() const
    {
        return topic_name_;
    }

    const std::string& get_frame_id() const
    {
        return frame_id_;
    }

    const std::string& get_dir_topic() const
    {
        return dir_topic_;
    }

    rclcpp::Node::SharedPtr& node() 
    {
        return node_;
    }

    std::string get_seq_str()
    {
        std::stringstream ss;
        ss << std::setw(10) << std::setfill('0') << seq_;
        return ss.str();
    }

    bool is_finish()
    {
        return is_finish_;
    }

    virtual void pub_record() = 0;

};