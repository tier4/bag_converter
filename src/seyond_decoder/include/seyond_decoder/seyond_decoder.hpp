/*
 *  Copyright (C) 2025 Seyond Inc.
 *
 *  License: Apache License
 */

#pragma once

#include "sdk_common/inno_lidar_api.h"
#include "sdk_common/inno_lidar_packet.h"
#include "sdk_common/inno_lidar_packet_utils.h"
#include "seyond_decoder/msg/seyond_packet.hpp"
#include "seyond_decoder/msg/seyond_scan.hpp"
#include "seyond_decoder/point_types.h"

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <memory>
#include <string>
#include <vector>

namespace seyond
{

struct DecoderConfig
{
  double max_range = 200.0;
  double min_range = 0.3;
  int coordinate_mode = 3;
  bool use_reflectance = true;
  std::string frame_id = "lidar";
};

class SeyondDecoder
{
public:
  explicit SeyondDecoder(const DecoderConfig & config = DecoderConfig());
  ~SeyondDecoder();

  sensor_msgs::msg::PointCloud2::SharedPtr convert(
    const seyond_decoder::msg::SeyondScan::SharedPtr & scan_msg);

  sensor_msgs::msg::PointCloud2::SharedPtr convert(
    const seyond_decoder::msg::SeyondScan & scan_msg);

  void setConfig(const DecoderConfig & config);
  DecoderConfig getConfig() const { return config_; }

  void setAngleHVTable(const std::vector<char> & table);
  void clearAngleHVTable();
  bool hasAngleHVTable() const { return anglehv_table_init_; }

private:
  void processPacket(
    const seyond_decoder::msg::SeyondPacket & packet, pcl::PointCloud<PointXYZIT> & cloud);

  void convertAndParse(const InnoDataPacket * pkt, pcl::PointCloud<PointXYZIT> & cloud);

  void dataPacketParse(const InnoDataPacket * pkt, pcl::PointCloud<PointXYZIT> & cloud);

  template <typename PointType>
  void pointXyzDataParse(
    bool is_use_refl, uint32_t point_num, PointType point_ptr, pcl::PointCloud<PointXYZIT> & cloud);

  void coordinateTransfer(PointXYZIT * point, int mode, float x, float y, float z);

private:
  DecoderConfig config_;
  bool anglehv_table_init_;
  std::vector<char> anglehv_table_;
  std::vector<uint8_t> data_buffer_;
  double current_ts_start_;

  static constexpr double us_in_second_c = 1000000.0;
  static constexpr double ten_us_in_second_c = 100000.0;
};

}  // namespace seyond