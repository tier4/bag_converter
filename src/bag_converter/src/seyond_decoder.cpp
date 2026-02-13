/*
 *  Copyright (C) 2025 TierIV Inc.
 *
 *  License: Apache License
 *
 *  Implementation of SeyondPCDDecoder for bag_converter package
 */

#include "seyond_decoder.hpp"

#include "sdk_common/inno_lidar_api.h"
#include "sdk_common/inno_lidar_packet.h"
#include "sdk_common/inno_lidar_packet_utils.h"

#include <rclcpp/rclcpp.hpp>

#include <pcl_conversions/pcl_conversions.h>

#include <cstring>
#include <type_traits>

namespace bag_converter::decoder::seyond
{

// SeyondPCDDecoder implementation
template <typename OutputPointT>
SeyondPCDDecoder<OutputPointT>::SeyondPCDDecoder(const SeyondPCDDecoderConfig & config)
: config_(config), anglehv_table_init_(false), scan_start_us_(0.0), pkt_offset_us_(0.0)
{
  data_buffer_.resize(defaults::data_buffer_size_bytes);
}

template <typename OutputPointT>
SeyondPCDDecoder<OutputPointT>::~SeyondPCDDecoder() = default;

template <typename OutputPointT>
sensor_msgs::msg::PointCloud2::SharedPtr SeyondPCDDecoder<OutputPointT>::decode_typed(
  const bag_converter::msg::SeyondScan & input)
{
  // Create point cloud for processing
  pcl::PointCloud<OutputPointT> cloud;
  cloud.header.frame_id = input.header.frame_id.empty() ? config_.frame_id : input.header.frame_id;
  cloud.header.stamp = input.header.stamp.sec * 1000000ULL + input.header.stamp.nanosec / 1000;
  cloud.points.reserve(defaults::initial_points_capacity);

  // Initialize angle HV table if not already initialized
  if (!anglehv_table_init_) {
    // Look for HV table packet in the scan
    for (const auto & packet : input.packets) {
      if (packet.type == bag_converter::msg::SeyondPacket::PACKET_TYPE_HVTABLE) {
        anglehv_table_.resize(packet.data.size());
        std::memcpy(anglehv_table_.data(), packet.data.data(), packet.data.size());
        anglehv_table_init_ = true;
        RCLCPP_INFO(
          rclcpp::get_logger("bag_converter.decoder.seyond"),
          "Angle HV table initialized (size: %zu bytes)", packet.data.size());
        break;
      }
    }
  }

  // Find the first POINTS packet to get the scan start time
  scan_start_us_ = 0.0;
  for (const auto & packet : input.packets) {
    if (
      packet.type == bag_converter::msg::SeyondPacket::PACKET_TYPE_POINTS && !packet.data.empty()) {
      const auto * pkt = reinterpret_cast<const InnoDataPacket *>(packet.data.data());
      scan_start_us_ = pkt->common.ts_start_us;
      break;
    }
  }

  // Process each packet in the scan
  size_t filtered_packet_count = 0;
  for (const auto & packet : input.packets) {
    if (
      packet.type == bag_converter::msg::SeyondPacket::PACKET_TYPE_POINTS && !packet.data.empty()) {
      const auto * pkt = reinterpret_cast<const InnoDataPacket *>(packet.data.data());
      // Filter by confidence level for Falcon packets if configured
      if (config_.min_conf_level > 0) {
        bool is_falcon = (pkt->common.lidar_type == INNO_LIDAR_TYPE_FALCON);
        if (is_falcon && pkt->confidence_level < static_cast<uint16_t>(config_.min_conf_level)) {
          ++filtered_packet_count;
          continue;
        }
      }
      process_packet(packet, cloud);
    }
  }
  if (filtered_packet_count > 0) {
    RCLCPP_WARN(
      rclcpp::get_logger("bag_converter.decoder.seyond"),
      "Filtered %zu packet(s) with confidence level below %d", filtered_packet_count,
      config_.min_conf_level);
  }

  // Check if point cloud is empty
  if (cloud.points.empty()) {
    RCLCPP_WARN(
      rclcpp::get_logger("bag_converter.decoder.seyond"),
      "Decoded point cloud is empty (no points found in scan)");
  }

  // Create PointCloud2 message
  auto pc2_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
  pcl::toROSMsg(cloud, *pc2_msg);

  // Set header timestamp from input message
  pc2_msg->header.stamp = input.header.stamp;
  pc2_msg->header.frame_id = cloud.header.frame_id;

  return pc2_msg;
}

template <typename OutputPointT>
void SeyondPCDDecoder<OutputPointT>::process_packet(
  const bag_converter::msg::SeyondPacket & packet, pcl::PointCloud<OutputPointT> & cloud)
{
  if (packet.data.empty()) {
    return;
  }

  // Convert packet data to InnoDataPacket structure
  const auto * pkt = reinterpret_cast<const InnoDataPacket *>(packet.data.data());
  convert_and_parse(pkt, cloud);
}

template <typename OutputPointT>
void SeyondPCDDecoder<OutputPointT>::convert_and_parse(
  const InnoDataPacket * pkt, pcl::PointCloud<OutputPointT> & cloud)
{
  if (CHECK_SPHERE_POINTCLOUD_DATA(pkt->type)) {
    // Compact (Robin W / Robin Elite) requires AngleHV table; non-Compact sphere does not
    if (CHECK_CO_SPHERE_POINTCLOUD_DATA(pkt->type)) {
      if (!anglehv_table_init_) {
        RCLCPP_ERROR(
          rclcpp::get_logger("bag_converter.decoder.seyond"),
          "Compact pointcloud (type %d) requires AngleHV table; skipping packet", pkt->type);
        return;
      }
      inno_lidar_convert_to_xyz_pointcloud2(
        pkt, reinterpret_cast<InnoDataPacket *>(&data_buffer_[0]), data_buffer_.size(), false,
        reinterpret_cast<InnoDataPacket *>(anglehv_table_.data()));
    } else {
      inno_lidar_convert_to_xyz_pointcloud(
        pkt, reinterpret_cast<InnoDataPacket *>(&data_buffer_[0]), data_buffer_.size(), false);
    }
    data_packet_parse(reinterpret_cast<InnoDataPacket *>(&data_buffer_[0]), cloud);
  } else if (CHECK_XYZ_POINTCLOUD_DATA(pkt->type)) {
    // Directly parse xyz point cloud data
    data_packet_parse(pkt, cloud);
  } else {
    RCLCPP_ERROR(
      rclcpp::get_logger("bag_converter.decoder.seyond"), "Packet type %d is not supported",
      pkt->type);
  }
}

template <typename OutputPointT>
void SeyondPCDDecoder<OutputPointT>::data_packet_parse(
  const InnoDataPacket * pkt, pcl::PointCloud<OutputPointT> & cloud)
{
  // Calculate the packet's time offset from scan start (in microseconds)
  pkt_offset_us_ = pkt->common.ts_start_us - scan_start_us_;

  // Store per-packet LiDAR state for use in point_xyz_data_parse
  pkt_lidar_mode_ = pkt->common.lidar_mode;
  pkt_lidar_status_ = pkt->common.lidar_status;

  // Parse point data based on packet type (different lidars use different structures)
  if (CHECK_EN_XYZ_POINTCLOUD_DATA(pkt->type)) {
    const InnoEnXyzPoint * pt = reinterpret_cast<const InnoEnXyzPoint *>(
      reinterpret_cast<const char *>(pkt) + sizeof(InnoDataPacket));
    point_xyz_data_parse<const InnoEnXyzPoint *>(
      config_.use_reflectance && pkt->use_reflectance, pkt->item_number, pt, cloud);
  } else if (CHECK_XYZ_POINTCLOUD_DATA(pkt->type)) {
    const InnoXyzPoint * pt = reinterpret_cast<const InnoXyzPoint *>(
      reinterpret_cast<const char *>(pkt) + sizeof(InnoDataPacket));
    point_xyz_data_parse<const InnoXyzPoint *>(
      config_.use_reflectance && pkt->use_reflectance, pkt->item_number, pt, cloud);
  } else {
    RCLCPP_ERROR(
      rclcpp::get_logger("bag_converter.decoder.seyond"), "Packet type %d is not supported",
      pkt->type);
    return;
  }
}

template <typename OutputPointT>
template <typename PointType>
void SeyondPCDDecoder<OutputPointT>::point_xyz_data_parse(
  bool is_use_refl, uint32_t point_num, PointType point_ptr, pcl::PointCloud<OutputPointT> & cloud)
{
  for (size_t i = 0; i < point_num; ++i, ++point_ptr) {
    // Filter points by range
    if (point_ptr->radius > config_.max_range || point_ptr->radius < config_.min_range) {
      continue;
    }

    OutputPointT point;

    // Set intensity based on point type and configuration
    if constexpr (std::is_same<PointType, const InnoEnXyzPoint *>::value) {
      point.intensity = is_use_refl ? static_cast<float>(point_ptr->reflectance)
                                    : static_cast<float>(point_ptr->intensity);
    } else if constexpr (std::is_same<PointType, const InnoXyzPoint *>::value) {
      point.intensity = static_cast<float>(point_ptr->refl);
    }

    // Set timestamp (relative time from scan start)
    // point_ptr->ts_10us is relative to the packet's ts_start_us (in 10us units)
    // pkt_offset_us_ is the packet's offset from scan start (in us)
    // point.timestamp = (pkt_offset_us_ + ts_10us * 10) * 1000 = time from scan start in ns
    // point.t_us = timestamp / 1000 = time from scan start in us (deprecated)
    if constexpr (
      std::is_same_v<OutputPointT, bag_converter::point::PointXYZIT> ||
      std::is_same_v<OutputPointT, bag_converter::point::PointEnXYZIT>) {
      point.timestamp = static_cast<uint32_t>((pkt_offset_us_ + point_ptr->ts_10us * 10) * 1000);
      point.t_us = point.timestamp / 1000;
    }
    // refl_type: point classification (0: normal, 1: ground, 2: fog); -1 when not available
    if constexpr (std::is_same_v<OutputPointT, bag_converter::point::PointEnXYZIT>) {
      if constexpr (std::is_same<PointType, const InnoXyzPoint *>::value) {
        point.refl_type = static_cast<int8_t>(point_ptr->type);
        point.elongation = static_cast<int16_t>(point_ptr->elongation);
      } else {
        point.refl_type = -1;   // InnoEnXyzPoint has no type field
        point.elongation = -1;  // InnoEnXyzPoint has no elongation field
      }
      point.lidar_status = static_cast<int8_t>(pkt_lidar_status_);
      point.lidar_mode = static_cast<int8_t>(pkt_lidar_mode_);
    }

    // Apply coordinate transformation (Autoware coordinate system)
    coordinate_transfer(&point, point_ptr->x, point_ptr->y, point_ptr->z);

    cloud.points.push_back(point);
  }

  // Set point cloud dimensions: unorganized (1D) point cloud
  // height = 1 means the point cloud is treated as a 1D array of points
  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = false;
}

template <typename OutputPointT>
void SeyondPCDDecoder<OutputPointT>::coordinate_transfer(
  OutputPointT * point, float x, float y, float z)
{
  // Convert from Seyond coordinate system (x=up, y=right, z=forward)
  // to Autoware coordinate system (x=forward, y=left, z=up)
  point->x = z;   // forward
  point->y = -y;  // left (negative right)
  point->z = x;   // up
}

template <typename OutputPointT>
SeyondPCDDecoderConfig SeyondPCDDecoder<OutputPointT>::get_config() const
{
  return config_;
}

template <typename OutputPointT>
void SeyondPCDDecoder<OutputPointT>::set_config(const SeyondPCDDecoderConfig & config)
{
  config_ = config;
}

template <typename OutputPointT>
void SeyondPCDDecoder<OutputPointT>::set_angle_hv_table(const std::vector<char> & table)
{
  anglehv_table_ = table;
  anglehv_table_init_ = true;
}

template <typename OutputPointT>
void SeyondPCDDecoder<OutputPointT>::clear_angle_hv_table()
{
  anglehv_table_.clear();
  anglehv_table_init_ = false;
}

// Explicit template instantiations
template class SeyondPCDDecoder<bag_converter::point::PointXYZIT>;
template class SeyondPCDDecoder<bag_converter::point::PointXYZI>;
template class SeyondPCDDecoder<bag_converter::point::PointEnXYZIT>;

// Explicit template instantiations for point_xyz_data_parse
template void
SeyondPCDDecoder<bag_converter::point::PointXYZIT>::point_xyz_data_parse<const InnoEnXyzPoint *>(
  bool, uint32_t, const InnoEnXyzPoint *, pcl::PointCloud<bag_converter::point::PointXYZIT> &);
template void
SeyondPCDDecoder<bag_converter::point::PointXYZIT>::point_xyz_data_parse<const InnoXyzPoint *>(
  bool, uint32_t, const InnoXyzPoint *, pcl::PointCloud<bag_converter::point::PointXYZIT> &);
template void
SeyondPCDDecoder<bag_converter::point::PointXYZI>::point_xyz_data_parse<const InnoEnXyzPoint *>(
  bool, uint32_t, const InnoEnXyzPoint *, pcl::PointCloud<bag_converter::point::PointXYZI> &);
template void
SeyondPCDDecoder<bag_converter::point::PointXYZI>::point_xyz_data_parse<const InnoXyzPoint *>(
  bool, uint32_t, const InnoXyzPoint *, pcl::PointCloud<bag_converter::point::PointXYZI> &);
template void
SeyondPCDDecoder<bag_converter::point::PointEnXYZIT>::point_xyz_data_parse<const InnoEnXyzPoint *>(
  bool, uint32_t, const InnoEnXyzPoint *, pcl::PointCloud<bag_converter::point::PointEnXYZIT> &);
template void
SeyondPCDDecoder<bag_converter::point::PointEnXYZIT>::point_xyz_data_parse<const InnoXyzPoint *>(
  bool, uint32_t, const InnoXyzPoint *, pcl::PointCloud<bag_converter::point::PointEnXYZIT> &);

}  // namespace bag_converter::decoder::seyond
