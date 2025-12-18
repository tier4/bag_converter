/*
 *  Copyright (C) 2025 TierIV Inc.
 *
 *  License: Apache License
 *
 *  Implementation of SeyondPCDDecoder for bag_converter package
 */

#include "seyond_decoder.hpp"

#include <sdk_common/inno_lidar_api.h>
#include <sdk_common/inno_lidar_packet.h>
#include <sdk_common/inno_lidar_packet_utils.h>
#include <rclcpp/rclcpp.hpp>

#include <pcl_conversions/pcl_conversions.h>

#include <cstring>
#include <type_traits>

namespace bag_converter::decoder
{

// SeyondPCDDecoder implementation
SeyondPCDDecoder::SeyondPCDDecoder(const SeyondPCDDecoderConfig & config)
: config_(config), anglehv_table_init_(false), current_ts_start_(0.0)
{
  data_buffer_.resize(2 * 1024 * 1024);  // 2MB buffer
}

SeyondPCDDecoder::~SeyondPCDDecoder() = default;

sensor_msgs::msg::PointCloud2::SharedPtr SeyondPCDDecoder::decode(
  const seyond_decoder::msg::SeyondScan & input)
{
  pcl::PointCloud<PointXYZIT> cloud;
  cloud.header.frame_id = input.header.frame_id.empty() ? config_.frame_id : input.header.frame_id;
  cloud.header.stamp =
    input.header.stamp.sec * 1000000ULL + input.header.stamp.nanosec / 1000;
  cloud.points.reserve(100000);  // Reserve space for points

  // Initialize angle HV table if not already initialized
  if (!anglehv_table_init_) {
    // Look for HV table packet in the scan
    for (const auto & packet : input.packets) {
      if (packet.type == seyond_decoder::msg::SeyondPacket::PACKET_TYPE_HVTABLE) {
        anglehv_table_.resize(packet.data.size());
        std::memcpy(anglehv_table_.data(), packet.data.data(), packet.data.size());
        anglehv_table_init_ = true;
        break;
      }
    }
  }

  // Process each packet in the scan
  for (const auto & packet : input.packets) {
    if (packet.type == seyond_decoder::msg::SeyondPacket::PACKET_TYPE_POINTS) {
      process_packet(packet, cloud);
    }
  }

  // Convert to ROS message
  auto msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
  pcl::toROSMsg(cloud, *msg);
  msg->header.stamp = input.header.stamp;
  msg->header.frame_id = cloud.header.frame_id;

  return msg;
}

void SeyondPCDDecoder::process_packet(
  const seyond_decoder::msg::SeyondPacket & packet, pcl::PointCloud<PointXYZIT> & cloud)
{
  if (packet.data.empty()) {
    return;
  }

  const InnoDataPacket * pkt = reinterpret_cast<const InnoDataPacket *>(packet.data.data());
  convert_and_parse(pkt, cloud);
}

void SeyondPCDDecoder::convert_and_parse(
  const InnoDataPacket * pkt, pcl::PointCloud<PointXYZIT> & cloud)
{
  if (CHECK_SPHERE_POINTCLOUD_DATA(pkt->type)) {
    // Convert sphere to xyz
    if (anglehv_table_init_) {
      inno_lidar_convert_to_xyz_pointcloud2(
        pkt, reinterpret_cast<InnoDataPacket *>(&data_buffer_[0]), data_buffer_.size(), false,
        reinterpret_cast<InnoDataPacket *>(anglehv_table_.data()));
    } else {
      inno_lidar_convert_to_xyz_pointcloud(
        pkt, reinterpret_cast<InnoDataPacket *>(&data_buffer_[0]), data_buffer_.size(), false);
    }
    data_packet_parse(reinterpret_cast<InnoDataPacket *>(&data_buffer_[0]), cloud);
  } else if (CHECK_XYZ_POINTCLOUD_DATA(pkt->type)) {
    data_packet_parse(pkt, cloud);
  } else {
    RCLCPP_ERROR(
      rclcpp::get_logger("bag_converter.decoder"), "Packet type %d is not supported", pkt->type);
  }
}

void SeyondPCDDecoder::data_packet_parse(
  const InnoDataPacket * pkt, pcl::PointCloud<PointXYZIT> & cloud)
{
  // Calculate the point timestamp
  current_ts_start_ = pkt->common.ts_start_us / us_in_second_c;

  // Adapt different data structures from different lidars
  if (CHECK_EN_XYZ_POINTCLOUD_DATA(pkt->type)) {
    const InnoEnXyzPoint * pt = reinterpret_cast<const InnoEnXyzPoint *>(
      reinterpret_cast<const char *>(pkt) + sizeof(InnoDataPacket));
    point_xyz_data_parse<const InnoEnXyzPoint *>(
      config_.use_reflectance && pkt->use_reflectance, pkt->item_number, pt, cloud);
  } else {
    const InnoXyzPoint * pt = reinterpret_cast<const InnoXyzPoint *>(
      reinterpret_cast<const char *>(pkt) + sizeof(InnoDataPacket));
    point_xyz_data_parse<const InnoXyzPoint *>(
      config_.use_reflectance && pkt->use_reflectance, pkt->item_number, pt, cloud);
  }
}

template <typename PointType>
void SeyondPCDDecoder::point_xyz_data_parse(
  bool is_use_refl, uint32_t point_num, PointType point_ptr, pcl::PointCloud<PointXYZIT> & cloud)
{
  for (uint32_t i = 0; i < point_num; ++i, ++point_ptr) {
    if (point_ptr->radius > config_.max_range || point_ptr->radius < config_.min_range) {
      continue;
    }

    PointXYZIT point;

    // Set intensity based on point type and configuration
    if constexpr (std::is_same<PointType, const InnoEnXyzPoint *>::value) {
      point.intensity = is_use_refl ? static_cast<float>(point_ptr->reflectance)
                                    : static_cast<float>(point_ptr->intensity);
    } else if constexpr (std::is_same<PointType, const InnoXyzPoint *>::value) {
      point.intensity = static_cast<float>(point_ptr->refl);
    }

    // Set timestamp (convert to microseconds from scan start)
    // point_ptr->ts_10us is in 10 microsecond units
    // Convert to microseconds: ts_10us * 10 = microseconds
    point.t_us = point_ptr->ts_10us * 10;

    // Coordinate transformation
    coordinate_transfer(&point, config_.coordinate_mode, point_ptr->x, point_ptr->y, point_ptr->z);

    cloud.points.push_back(point);
  }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = false;
}

void SeyondPCDDecoder::coordinate_transfer(
  PointXYZIT * point, int mode, float x, float y, float z)
{
  switch (mode) {
    case 0:
      point->x = x;  // up
      point->y = y;  // right
      point->z = z;  // forward
      break;
    case 1:
      point->x = y;  // right
      point->y = z;  // forward
      point->z = x;  // up
      break;
    case 2:
      point->x = y;  // right
      point->y = x;  // up
      point->z = z;  // forward
      break;
    case 3:
      point->x = z;   // forward
      point->y = -y;  // -right
      point->z = x;   // up
      break;
    case 4:
      point->x = z;  // forward
      point->y = x;  // up
      point->z = y;  // right
      break;
    default:
      // default
      point->x = x;  // up
      point->y = y;  // right
      point->z = z;  // forward
      break;
  }
}

SeyondPCDDecoderConfig SeyondPCDDecoder::get_config() const
{
  return config_;
}

void SeyondPCDDecoder::set_config(const SeyondPCDDecoderConfig & config)
{
  config_ = config;
}

void SeyondPCDDecoder::set_angle_hv_table(const std::vector<char> & table)
{
  anglehv_table_ = table;
  anglehv_table_init_ = true;
}

void SeyondPCDDecoder::clear_angle_hv_table()
{
  anglehv_table_.clear();
  anglehv_table_init_ = false;
}

// Explicit template instantiations
template void SeyondPCDDecoder::point_xyz_data_parse<const InnoEnXyzPoint *>(
  bool, uint32_t, const InnoEnXyzPoint *, pcl::PointCloud<PointXYZIT> &);
template void SeyondPCDDecoder::point_xyz_data_parse<const InnoXyzPoint *>(
  bool, uint32_t, const InnoXyzPoint *, pcl::PointCloud<PointXYZIT> &);

}  // namespace bag_converter::decoder

