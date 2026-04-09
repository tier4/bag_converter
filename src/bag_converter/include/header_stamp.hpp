/*
 *  Copyright (C) 2025 TierIV Inc.
 *
 *  License: Apache License
 *
 *  Header stamp extraction utilities for ROS 2 message types
 */

#ifndef BAG_CONVERTER__HEADER_STAMP_HPP
#define BAG_CONVERTER__HEADER_STAMP_HPP

#include <rclcpp/rclcpp.hpp>

#include <rcutils/types/uint8_array.h>

#include <optional>
#include <set>
#include <string>

namespace bag_converter
{

/**
 * @brief Set of ROS 2 message types that have std_msgs/msg/Header (used for log_time override).
 *
 * For these types, header.stamp is extracted by deserializing the message and
 * reading the header field, so log_time can be overridden from header.stamp when
 * --use-header-stamp-as-log-time is enabled.
 */
inline const std::set<std::string> kTypesWithHeader = {
  // LiDAR packet types (for log_time override when keeping original topics)
  "nebula_msgs/msg/NebulaPackets",
  "seyond/msg/SeyondScan",
  // sensor_msgs
  "sensor_msgs/msg/CameraInfo",
  "sensor_msgs/msg/CompressedImage",
  "sensor_msgs/msg/Imu",
  "sensor_msgs/msg/Image",
  "sensor_msgs/msg/LaserScan",
  "sensor_msgs/msg/NavSatFix",
  "sensor_msgs/msg/PointCloud2",
  // geometry_msgs
  "geometry_msgs/msg/AccelStamped",
  "geometry_msgs/msg/PoseStamped",
  "geometry_msgs/msg/TwistStamped",
  "geometry_msgs/msg/WrenchStamped",
  // nav_msgs
  "nav_msgs/msg/Odometry",
  // can_msgs
  "can_msgs/msg/Frame",
  // oxts_msgs
  "oxts_msgs/msg/ImuBias",
  "oxts_msgs/msg/LeverArm",
  "oxts_msgs/msg/NavSatRef",
  "oxts_msgs/msg/Ncom",
};

/**
 * @brief Extract header.stamp by deserializing the message.
 *
 * For types in kTypesWithHeader, deserializes to the concrete message type and
 * returns header.stamp. Returns std::nullopt if the type is unsupported or
 * deserialization fails.
 */
std::optional<rclcpp::Time> extract_header_stamp(
  const rcutils_uint8_array_t & serialized_data, const std::string & topic_type);

}  // namespace bag_converter

#endif  // BAG_CONVERTER__HEADER_STAMP_HPP
