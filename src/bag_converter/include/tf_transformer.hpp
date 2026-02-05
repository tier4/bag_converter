/*
 *  Copyright (C) 2025 TierIV Inc.
 *
 *  License: Apache License
 *
 *  TF2 coordinate transformation for PointCloud2 messages
 */

#ifndef BAG_CONVERTER__TF_TRANSFORMER_HPP
#define BAG_CONVERTER__TF_TRANSFORMER_HPP

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include <tf2/buffer_core.h>

#include <algorithm>
#include <string>

namespace bag_converter
{
enum class BagConverterTfMode;
}

namespace bag_converter::tf_transformer
{

class TfTransformer
{
public:
  explicit TfTransformer(BagConverterTfMode tf_mode);

  /// Add transforms from a serialized TFMessage
  void add_transforms(rclcpp::SerializedMessage & serialized_msg, const std::string & topic_name);

  /// Transform a PointCloud2 to the target frame. Returns true if transformed.
  bool transform(sensor_msgs::msg::PointCloud2 & cloud, const std::string & frame);

  /// Check if a frame exists in the TF buffer
  bool has_frame(const std::string & frame) const;

private:
  BagConverterTfMode tf_mode_;
  tf2::BufferCore buffer_;
  rclcpp::Serialization<tf2_msgs::msg::TFMessage> serializer_;
};

}  // namespace bag_converter::tf_transformer

#endif  // BAG_CONVERTER__TF_TRANSFORMER_HPP
