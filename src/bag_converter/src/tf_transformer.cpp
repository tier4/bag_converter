/*
 *  Copyright (C) 2025 TierIV Inc.
 *
 *  License: Apache License
 *
 *  Implementation of TF2 coordinate transformation for PointCloud2 messages
 */

#include "tf_transformer.hpp"

#include "bag_converter.hpp"

#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

#include <tf2/exceptions.h>
#include <tf2/time.h>

#include <chrono>
#include <string>

static const rclcpp::Logger g_tf_logger = rclcpp::get_logger("tf_transformer");

namespace bag_converter::tf_transformer
{

TfTransformer::TfTransformer(BagConverterTfMode tf_mode) : tf_mode_(tf_mode)
{
}

void TfTransformer::add_transforms(
  rclcpp::SerializedMessage & serialized_msg, const std::string & topic_name)
{
  tf2_msgs::msg::TFMessage tf_msg;
  serializer_.deserialize_message(&serialized_msg, &tf_msg);

  const bool is_static = (tf_mode_ == BagConverterTfMode::kStatic) || (topic_name == "/tf_static");

  for (const auto & transform : tf_msg.transforms) {
    try {
      buffer_.setTransform(transform, "bag", is_static);
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN(g_tf_logger, "Failed to add transform: %s", e.what());
    }
  }
}

bool TfTransformer::transform(sensor_msgs::msg::PointCloud2 & cloud, const std::string & frame)
{
  if (cloud.header.frame_id == frame) {
    return true;
  }

  try {
    tf2::TimePoint time_point = tf2::TimePointZero;
    if (tf_mode_ == BagConverterTfMode::kDynamic) {
      time_point = tf2::TimePoint(
        std::chrono::seconds(cloud.header.stamp.sec) +
        std::chrono::nanoseconds(cloud.header.stamp.nanosec));
    }
    auto transform_stamped = buffer_.lookupTransform(frame, cloud.header.frame_id, time_point);

    sensor_msgs::msg::PointCloud2 transformed_cloud;
    tf2::doTransform(cloud, transformed_cloud, transform_stamped);
    cloud = std::move(transformed_cloud);
    return true;
  } catch (const tf2::TransformException & e) {
    RCLCPP_WARN(
      g_tf_logger, "TF transform failed (%s -> %s): %s", cloud.header.frame_id.c_str(),
      frame.c_str(), e.what());
    return false;
  }
}

bool TfTransformer::has_frame(const std::string & frame) const
{
  return buffer_._frameExists(frame);
}

}  // namespace bag_converter::tf_transformer
