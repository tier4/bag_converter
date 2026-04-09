/*
 *  Copyright (C) 2025 TierIV Inc.
 *
 *  License: Apache License
 *
 *  Header stamp extraction utilities for ROS 2 message types
 */

#include "header_stamp.hpp"

#include <rclcpp/serialization.hpp>
#include <seyond/msg/seyond_scan.hpp>

#include <can_msgs/msg/frame.hpp>
#include <geometry_msgs/msg/accel_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nebula_msgs/msg/nebula_packets.hpp>
#include <oxts_msgs/msg/imu_bias.hpp>
#include <oxts_msgs/msg/lever_arm.hpp>
#include <oxts_msgs/msg/nav_sat_ref.hpp>
#include <oxts_msgs/msg/ncom.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace bag_converter
{
namespace
{

template <typename MsgT>
std::optional<rclcpp::Time> deserialize_header_stamp(rclcpp::SerializedMessage & serialized_msg)
{
  MsgT msg;
  rclcpp::Serialization<MsgT> ser;
  ser.deserialize_message(&serialized_msg, &msg);
  return rclcpp::Time(msg.header.stamp);
}

}  // namespace

std::optional<rclcpp::Time> extract_header_stamp(
  const rcutils_uint8_array_t & serialized_data, const std::string & topic_type)
{
  if (kTypesWithHeader.find(topic_type) == kTypesWithHeader.end()) {
    return std::nullopt;
  }
  try {
    rclcpp::SerializedMessage serialized_msg(serialized_data);
    // sensor_msgs
    if (topic_type == "sensor_msgs/msg/CameraInfo")
      return deserialize_header_stamp<sensor_msgs::msg::CameraInfo>(serialized_msg);
    if (topic_type == "sensor_msgs/msg/CompressedImage")
      return deserialize_header_stamp<sensor_msgs::msg::CompressedImage>(serialized_msg);
    if (topic_type == "sensor_msgs/msg/Imu")
      return deserialize_header_stamp<sensor_msgs::msg::Imu>(serialized_msg);
    if (topic_type == "sensor_msgs/msg/Image")
      return deserialize_header_stamp<sensor_msgs::msg::Image>(serialized_msg);
    if (topic_type == "sensor_msgs/msg/LaserScan")
      return deserialize_header_stamp<sensor_msgs::msg::LaserScan>(serialized_msg);
    if (topic_type == "sensor_msgs/msg/NavSatFix")
      return deserialize_header_stamp<sensor_msgs::msg::NavSatFix>(serialized_msg);
    if (topic_type == "sensor_msgs/msg/PointCloud2")
      return deserialize_header_stamp<sensor_msgs::msg::PointCloud2>(serialized_msg);
    // geometry_msgs
    if (topic_type == "geometry_msgs/msg/AccelStamped")
      return deserialize_header_stamp<geometry_msgs::msg::AccelStamped>(serialized_msg);
    if (topic_type == "geometry_msgs/msg/PoseStamped")
      return deserialize_header_stamp<geometry_msgs::msg::PoseStamped>(serialized_msg);
    if (topic_type == "geometry_msgs/msg/TwistStamped")
      return deserialize_header_stamp<geometry_msgs::msg::TwistStamped>(serialized_msg);
    if (topic_type == "geometry_msgs/msg/WrenchStamped")
      return deserialize_header_stamp<geometry_msgs::msg::WrenchStamped>(serialized_msg);
    // nav_msgs
    if (topic_type == "nav_msgs/msg/Odometry")
      return deserialize_header_stamp<nav_msgs::msg::Odometry>(serialized_msg);
    // can_msgs
    if (topic_type == "can_msgs/msg/Frame")
      return deserialize_header_stamp<can_msgs::msg::Frame>(serialized_msg);
    // oxts_msgs
    if (topic_type == "oxts_msgs/msg/ImuBias")
      return deserialize_header_stamp<oxts_msgs::msg::ImuBias>(serialized_msg);
    if (topic_type == "oxts_msgs/msg/LeverArm")
      return deserialize_header_stamp<oxts_msgs::msg::LeverArm>(serialized_msg);
    if (topic_type == "oxts_msgs/msg/NavSatRef")
      return deserialize_header_stamp<oxts_msgs::msg::NavSatRef>(serialized_msg);
    if (topic_type == "oxts_msgs/msg/Ncom")
      return deserialize_header_stamp<oxts_msgs::msg::Ncom>(serialized_msg);
    // LiDAR packet types
    if (topic_type == "nebula_msgs/msg/NebulaPackets")
      return deserialize_header_stamp<nebula_msgs::msg::NebulaPackets>(serialized_msg);
    if (topic_type == "seyond/msg/SeyondScan")
      return deserialize_header_stamp<seyond::msg::SeyondScan>(serialized_msg);
  } catch (...) {
    return std::nullopt;
  }
  return std::nullopt;
}

}  // namespace bag_converter
