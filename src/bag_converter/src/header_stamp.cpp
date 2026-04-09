/*
 *  Copyright (C) 2025 TierIV Inc.
 *
 *  License: Apache License
 *
 *  Header stamp extraction utilities for ROS 2 message types
 */

#include "header_stamp.hpp"

#include <cstring>

namespace bag_converter
{

// CDR layout for messages where std_msgs/msg/Header is the first field:
//   Bytes 0-1: CDR representation identifier (0x00 0x01 = CDR little-endian)
//   Bytes 2-3: CDR options (usually 0x00 0x00)
//   Bytes 4-7: header.stamp.sec   (int32_t)
//   Bytes 8-11: header.stamp.nanosec (uint32_t)
//
// All types in kTypesWithHeader have Header as their first field, so this
// fixed-offset read is valid for all of them. This avoids full message
// deserialization, which is critical for large messages (e.g., Image).

static constexpr size_t kCdrHeaderSize = 4;
static constexpr size_t kStampSecOffset = kCdrHeaderSize;
static constexpr size_t kStampNanosecOffset = kCdrHeaderSize + sizeof(int32_t);
static constexpr size_t kMinBufferSize = kStampNanosecOffset + sizeof(uint32_t);  // 12 bytes

std::optional<rclcpp::Time> extract_header_stamp(
  const rcutils_uint8_array_t & serialized_data, const std::string & topic_type)
{
  if (kTypesWithHeader.find(topic_type) == kTypesWithHeader.end()) {
    return std::nullopt;
  }

  if (serialized_data.buffer_length < kMinBufferSize) {
    return std::nullopt;
  }

  const uint8_t * buf = serialized_data.buffer;

  // Verify CDR little-endian encoding (0x00 0x01)
  if (buf[0] != 0x00 || buf[1] != 0x01) {
    return std::nullopt;
  }

  int32_t sec;
  uint32_t nanosec;
  std::memcpy(&sec, buf + kStampSecOffset, sizeof(sec));
  std::memcpy(&nanosec, buf + kStampNanosecOffset, sizeof(nanosec));

  return rclcpp::Time(sec, nanosec);
}

}  // namespace bag_converter
