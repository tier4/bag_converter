/*
 *  Copyright (C) 2025 Seyond Inc.
 *
 *  License: Apache License
 *
 *  Example program demonstrating the use of SeyondDecoder library
 */

#include "seyond_decoder/msg/seyond_scan.hpp"
#include "seyond_decoder/seyond_decoder.hpp"

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <chrono>

class SeyondDecoderExample : public rclcpp::Node
{
public:
  SeyondDecoderExample() : Node("seyond_decoder_example")
  {
    // Configure decoder
    seyond::DecoderConfig config;
    config.max_range = this->declare_parameter<double>("max_range", 200.0);
    config.min_range = this->declare_parameter<double>("min_range", 0.3);
    config.coordinate_mode = this->declare_parameter<int>("coordinate_mode", 0);
    config.use_reflectance = this->declare_parameter<bool>("use_reflectance", true);
    config.frame_id = this->declare_parameter<std::string>("frame_id", "lidar");

    decoder_ = std::make_unique<seyond::SeyondDecoder>(config);

    // Create subscriber for SeyondScan messages
    std::string input_topic = this->declare_parameter<std::string>("input_topic", "/seyond_scan");
    scan_sub_ = this->create_subscription<seyond_decoder::msg::SeyondScan>(
      input_topic, 10, std::bind(&SeyondDecoderExample::scanCallback, this, std::placeholders::_1));

    // Create publisher for PointCloud2 messages
    std::string output_topic = this->declare_parameter<std::string>("output_topic", "/pointcloud");
    pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic, 10);

    RCLCPP_INFO(this->get_logger(), "SeyondDecoder example started");
    RCLCPP_INFO(this->get_logger(), "Subscribing to: %s", input_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "Publishing to: %s", output_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "Configuration:");
    RCLCPP_INFO(this->get_logger(), "  - max_range: %.2f", config.max_range);
    RCLCPP_INFO(this->get_logger(), "  - min_range: %.2f", config.min_range);
    RCLCPP_INFO(this->get_logger(), "  - coordinate_mode: %d", config.coordinate_mode);
    RCLCPP_INFO(
      this->get_logger(), "  - use_reflectance: %s", config.use_reflectance ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "  - frame_id: %s", config.frame_id.c_str());
  }

private:
  void scanCallback(const seyond_decoder::msg::SeyondScan::SharedPtr msg)
  {
    auto start = std::chrono::high_resolution_clock::now();

    // Convert SeyondScan to PointCloud2
    auto pointcloud_msg = decoder_->convert(msg);

    if (pointcloud_msg) {
      // Publish the converted point cloud
      pointcloud_pub_->publish(*pointcloud_msg);

      auto end = std::chrono::high_resolution_clock::now();
      auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);

      RCLCPP_DEBUG(this->get_logger(), "Conversion took %ld microseconds", duration.count());

      // Log statistics periodically
      message_count_++;
      if (message_count_ % 100 == 0) {
        RCLCPP_INFO(this->get_logger(), "Processed %zu SeyondScan messages", message_count_);
      }
    } else {
      RCLCPP_WARN(this->get_logger(), "Failed to convert SeyondScan message");
    }
  }

  std::unique_ptr<seyond::SeyondDecoder> decoder_;
  rclcpp::Subscription<seyond_decoder::msg::SeyondScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;
  size_t message_count_ = 0;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SeyondDecoderExample>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}