/*
 *  Copyright (C) 2025 Seyond Inc.
 *
 *  License: Apache License
 */

#include "seyond_decoder/seyond_decoder.hpp"

#include <chrono>
#include <cstring>

namespace seyond
{

SeyondDecoder::SeyondDecoder(const DecoderConfig & config)
: config_(config), anglehv_table_init_(false)
{
  data_buffer_.resize(2 * 1024 * 1024);  // 2MB buffer
}

SeyondDecoder::~SeyondDecoder() = default;

sensor_msgs::msg::PointCloud2::SharedPtr SeyondDecoder::convert(
  const seyond_decoder::msg::SeyondScan::SharedPtr & scan_msg)
{
  return convert(*scan_msg);
}

sensor_msgs::msg::PointCloud2::SharedPtr SeyondDecoder::convert(
  const seyond_decoder::msg::SeyondScan & scan_msg)
{
  pcl::PointCloud<PointXYZIT> cloud;
  cloud.header.frame_id = scan_msg.header.frame_id;
  cloud.header.stamp =
    scan_msg.header.stamp.sec * 1000000ULL + scan_msg.header.stamp.nanosec / 1000;
  cloud.points.reserve(100000);  // Reserve space for points

  if (!anglehv_table_init_) {
    const auto & packet = scan_msg.packets.back();
    if (packet.type == seyond_decoder::msg::SeyondPacket::PACKET_TYPE_HVTABLE) {
      anglehv_table_.resize(packet.data.size());
      std::memcpy(anglehv_table_.data(), packet.data.data(), packet.data.size());
      anglehv_table_init_ = true;
    }
  }

  // Process each packet in the scan
  for (const auto & packet : scan_msg.packets) {
    if (packet.type == seyond_decoder::msg::SeyondPacket::PACKET_TYPE_POINTS) {
      processPacket(packet, cloud);
    }
    // else if (packet.type == seyond_decoder::msg::SeyondPacket::PACKET_TYPE_HVTABLE) {
    //   // Handle HV table packet if needed
    //   if (packet.data.size() > 0) {
    //     anglehv_table_.resize(packet.data.size());
    //     std::memcpy(anglehv_table_.data(), packet.data.data(), packet.data.size());
    //     anglehv_table_init_ = true;
    //   }
    // }
  }

  // Convert to ROS message
  auto msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
  pcl::toROSMsg(cloud, *msg);
  msg->header.stamp = scan_msg.header.stamp;
  msg->header.frame_id = scan_msg.header.frame_id;
  return msg;
}

void SeyondDecoder::setConfig(const DecoderConfig & config)
{
  config_ = config;
}

void SeyondDecoder::setAngleHVTable(const std::vector<char> & table)
{
  anglehv_table_ = table;
  anglehv_table_init_ = true;
}

void SeyondDecoder::clearAngleHVTable()
{
  anglehv_table_.clear();
  anglehv_table_init_ = false;
}

void SeyondDecoder::processPacket(
  const seyond_decoder::msg::SeyondPacket & packet, pcl::PointCloud<PointXYZIT> & cloud)
{
  if (packet.data.empty()) {
    return;
  }

  const InnoDataPacket * pkt = reinterpret_cast<const InnoDataPacket *>(packet.data.data());
  convertAndParse(pkt, cloud);
}

void SeyondDecoder::convertAndParse(const InnoDataPacket * pkt, pcl::PointCloud<PointXYZIT> & cloud)
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
    dataPacketParse(reinterpret_cast<InnoDataPacket *>(&data_buffer_[0]), cloud);
  } else if (CHECK_XYZ_POINTCLOUD_DATA(pkt->type)) {
    dataPacketParse(pkt, cloud);
  } else {
    RCLCPP_ERROR(
      rclcpp::get_logger("seyond_decoder"), "Packet type %d is not supported", pkt->type);
  }
}

void SeyondDecoder::dataPacketParse(const InnoDataPacket * pkt, pcl::PointCloud<PointXYZIT> & cloud)
{
  // Calculate the point timestamp
  current_ts_start_ = pkt->common.ts_start_us / us_in_second_c;

  // Adapt different data structures from different lidars
  if (CHECK_EN_XYZ_POINTCLOUD_DATA(pkt->type)) {
    const InnoEnXyzPoint * pt = reinterpret_cast<const InnoEnXyzPoint *>(
      reinterpret_cast<const char *>(pkt) + sizeof(InnoDataPacket));
    pointXyzDataParse<const InnoEnXyzPoint *>(pkt->use_reflectance, pkt->item_number, pt, cloud);
  } else {
    const InnoXyzPoint * pt = reinterpret_cast<const InnoXyzPoint *>(
      reinterpret_cast<const char *>(pkt) + sizeof(InnoDataPacket));
    pointXyzDataParse<const InnoXyzPoint *>(pkt->use_reflectance, pkt->item_number, pt, cloud);
  }
}

template <typename PointType>
void SeyondDecoder::pointXyzDataParse(
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

    // Set timestamp
    point.timestamp = point_ptr->ts_10us / ten_us_in_second_c + current_ts_start_;

    // Coordinate transformation
    coordinateTransfer(&point, config_.coordinate_mode, point_ptr->x, point_ptr->y, point_ptr->z);

    cloud.points.push_back(point);
  }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = false;
}

void SeyondDecoder::coordinateTransfer(PointXYZIT * point, int mode, float x, float y, float z)
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

// Explicit template instantiations
template void SeyondDecoder::pointXyzDataParse<const InnoEnXyzPoint *>(
  bool, uint32_t, const InnoEnXyzPoint *, pcl::PointCloud<PointXYZIT> &);
template void SeyondDecoder::pointXyzDataParse<const InnoXyzPoint *>(
  bool, uint32_t, const InnoXyzPoint *, pcl::PointCloud<PointXYZIT> &);

}  // namespace seyond