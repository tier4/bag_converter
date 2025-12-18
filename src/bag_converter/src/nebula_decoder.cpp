/*
 *  Copyright (C) 2025 TierIV Inc.
 *
 *  License: Apache License
 *
 *  Implementation of NebulaPCDDecoder for bag_converter package
 */

#include "nebula_decoder.hpp"

#include <nebula_decoders/nebula_decoders_seyond/decoders/seyond_packet.hpp>
#include <nebula_decoders/nebula_decoders_seyond/seyond_driver.hpp>
#include <rclcpp/rclcpp.hpp>

#include <pcl_conversions/pcl_conversions.h>

#include <iomanip>
#include <iostream>
#include <type_traits>

namespace bag_converter::decoder::nebula
{

// NebulaPCDDecoder implementation
template <typename OutputPointT>
NebulaPCDDecoder<OutputPointT>::NebulaPCDDecoder(const NebulaPCDDecoderConfig & config)
: config_(config)
{
  // Create sensor configuration
  sensor_config_ = std::make_shared<::nebula::drivers::SeyondSensorConfiguration>();

  // Set basic network config (these won't be used for offline processing)
  sensor_config_->host_ip = defaults::dummy_host_ip;
  sensor_config_->sensor_ip = defaults::dummy_sensor_ip;
  sensor_config_->data_port = defaults::dummy_data_port;
  sensor_config_->gnss_port = defaults::dummy_gnss_port;
  sensor_config_->packet_mtu_size = defaults::dummy_packet_mtu_size;

  // Set sensor model and return mode
  sensor_config_->sensor_model = ::nebula::drivers::SensorModelFromString(config.sensor_model);
  sensor_config_->return_mode = ::nebula::drivers::ReturnModeFromString(config.return_mode);

  // Set other parameters
  sensor_config_->frame_id = config.frame_id;
  sensor_config_->scan_phase = config.scan_phase;
  sensor_config_->frequency_ms = static_cast<uint16_t>(config.frequency_ms);
  sensor_config_->use_sensor_time = config.use_sensor_time;

  // Set min/max range
  sensor_config_->min_range = config.min_range;
  sensor_config_->max_range = config.max_range;

  // Create calibration configuration
  calibration_config_ = std::make_shared<::nebula::drivers::SeyondCalibrationConfiguration>();

  // Initialize the driver
  try {
    driver_ =
      std::make_shared<::nebula::drivers::SeyondDriver>(sensor_config_, calibration_config_);

    // Check actual driver status
    auto driver_status = driver_->GetStatus();  // Note: driver_->GetStatus() is from nebula library
    if (driver_status != ::nebula::Status::OK) {
      throw std::runtime_error("Failed to initialize driver: driver status is not OK");
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      rclcpp::get_logger("bag_converter.decoder"), "Failed to initialize driver: %s", e.what());
    throw;  // Re-throw the exception to terminate initialization
  }
}

template <typename OutputPointT>
NebulaPCDDecoder<OutputPointT>::~NebulaPCDDecoder() = default;

template <typename OutputPointT>
sensor_msgs::msg::PointCloud2::SharedPtr NebulaPCDDecoder<OutputPointT>::decode(
  const nebula_msgs::msg::NebulaPackets & input)
{
  // Process NebulaPackets
  process_nebula_packets(input);

  // Flush cloud points to get the complete point cloud
  auto nebula_cloud = flush_cloud_points();

  if (!nebula_cloud || nebula_cloud->empty()) {
    return nullptr;
  }

  // Convert NebulaPointCloud to OutputPointT for PCL conversion
  pcl::PointCloud<OutputPointT> pc2_cloud;
  pc2_cloud.header = nebula_cloud->header;
  pc2_cloud.header.frame_id = config_.frame_id;
  pc2_cloud.width = nebula_cloud->width;
  pc2_cloud.height = nebula_cloud->height;
  pc2_cloud.is_dense = nebula_cloud->is_dense;

  for (const auto & pt : nebula_cloud->points) {
    OutputPointT pc2_pt;
    pc2_pt.x = pt.x;
    pc2_pt.y = pt.y;
    pc2_pt.z = pt.z;
    pc2_pt.intensity = pt.intensity;
    if constexpr (std::is_same_v<OutputPointT, bag_converter::point::PointXYZIT>) {
      pc2_pt.t_us = pt.time_stamp;
    }
    pc2_cloud.push_back(pc2_pt);
  }

  // Create PointCloud2 message
  auto pc2_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
  pcl::toROSMsg(pc2_cloud, *pc2_msg);

  // Set header timestamp from input message
  pc2_msg->header.stamp = input.header.stamp;
  pc2_msg->header.frame_id = config_.frame_id;

  return pc2_msg;
}

template <typename OutputPointT>
std::tuple<::nebula::drivers::NebulaPointCloudPtr, bool>
NebulaPCDDecoder<OutputPointT>::process_packet(const std::vector<uint8_t> & packet)
{
  // Process the packet - the driver returns a cloud when scan is complete
  auto [cloud, cloud_timestamp] = driver_->ParseCloudPacket(packet);

  // Check if a complete scan is available
  // (ParseCloudPacket returns non-null cloud when scan is complete)
  if (cloud != nullptr) {
    return {cloud, true};
  }
  return {nullptr, false};
}

template <typename OutputPointT>
::nebula::drivers::NebulaPointCloudPtr NebulaPCDDecoder<OutputPointT>::process_packets(
  const std::vector<std::vector<uint8_t>> & packets)
{
  ::nebula::drivers::NebulaPointCloudPtr complete_cloud;
  double scan_timestamp_s = 0;
  bool once = false;

  for (const auto & packet : packets) {
    auto [cloud, cloud_timestamp] = driver_->ParseCloudPacket(packet);

    if (cloud && !cloud->empty() && !once) {
      // A complete scan was returned
      once = true;
      complete_cloud = cloud;
      scan_timestamp_s = cloud_timestamp;
    }
    // NOTE: If multiple scans are packed in the same NebulaPackets message,
    // return the first complete scan, the rest are buffered in the driver for the next scan.
  }

  // Set the timestamp in the point cloud header
  if (complete_cloud && scan_timestamp_s > 0) {
    // Convert seconds to microseconds (PCL header.stamp is in microseconds)
    complete_cloud->header.stamp = static_cast<uint64_t>(scan_timestamp_s * 1e6);
  }

  // Return the last complete cloud if available
  return complete_cloud;
}

template <typename OutputPointT>
::nebula::drivers::NebulaPointCloudPtr NebulaPCDDecoder<OutputPointT>::process_nebula_packets(
  const nebula_msgs::msg::NebulaPackets & packets)
{
  std::vector<std::vector<uint8_t>> packet_data_vec;

  // Extract packet data from nebula_msgs
  for (const auto & packet : packets.packets) {
    packet_data_vec.push_back(packet.data);
  }

  return process_packets(packet_data_vec);
}

template <typename OutputPointT>
::nebula::drivers::NebulaPointCloudPtr NebulaPCDDecoder<OutputPointT>::flush_cloud_points()
{
  auto [cloud, cloud_timestamp] = driver_->FlushCloudPoints();

  if (cloud && cloud_timestamp > 0) {
    // Convert seconds to microseconds (PCL header.stamp is in microseconds)
    cloud->header.stamp = static_cast<uint64_t>(cloud_timestamp * 1e6);
    return cloud;
  }
  return nullptr;
}

template <typename OutputPointT>
NebulaPCDDecoderConfig NebulaPCDDecoder<OutputPointT>::get_config() const
{
  return config_;
}

template <typename OutputPointT>
void NebulaPCDDecoder<OutputPointT>::set_config(const NebulaPCDDecoderConfig & config)
{
  config_ = config;

  // Update sensor configuration
  sensor_config_->sensor_model = ::nebula::drivers::SensorModelFromString(config.sensor_model);
  sensor_config_->return_mode = ::nebula::drivers::ReturnModeFromString(config.return_mode);
  sensor_config_->frame_id = config.frame_id;
  sensor_config_->scan_phase = config.scan_phase;
  sensor_config_->frequency_ms = static_cast<uint16_t>(config.frequency_ms);
  sensor_config_->use_sensor_time = config.use_sensor_time;
  sensor_config_->min_range = config.min_range;
  sensor_config_->max_range = config.max_range;
}

// Explicit template instantiations
template class NebulaPCDDecoder<bag_converter::point::PointXYZIT>;
template class NebulaPCDDecoder<bag_converter::point::PointXYZI>;

}  // namespace bag_converter::decoder::nebula
