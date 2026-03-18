/*
 *  Copyright (C) 2025 TierIV Inc.
 *
 *  License: Apache License
 *
 *  SeyondPCDDecoder class definition for bag_converter package
 */

#ifndef BAG_CONVERTER__SEYOND_DECODER_HPP
#define BAG_CONVERTER__SEYOND_DECODER_HPP

#include "base_decoder.hpp"
#include "point_types.hpp"

#include <bag_converter/msg/seyond_packet.hpp>
#include <bag_converter/msg/seyond_scan.hpp>

// Forward declarations for SDK types
struct InnoDataPacket;
struct InnoXyzPoint;
struct InnoEnXyzPoint;

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace bag_converter::decoder::seyond
{

// Default configuration constants
namespace defaults
{
inline constexpr size_t data_buffer_size_bytes = 2 * 1024 * 1024;  // 2MB buffer
inline constexpr size_t initial_points_capacity =
  200000;  // Initial capacity for point cloud reservation
}  // namespace defaults

// Configuration for the decoder
struct SeyondPCDDecoderConfig : public BasePCDDecoderConfig
{
};

/**
 * @brief SeyondPCDDecoder class that inherits from PCDDecoder
 *
 * This class provides the decode interface required by PCDDecoder and uses
 * the Seyond decoder logic for packet processing.
 *
 * @tparam OutputPointT The point type used for output point cloud (default:
 * bag_converter::point::PointXYZIT)
 */
template <typename OutputPointT = bag_converter::point::PointXYZIT>
class SeyondPCDDecoder : public PCDDecoder<bag_converter::msg::SeyondScan, OutputPointT>
{
public:
  /**
   * @brief Constructor with configuration
   * @param config Decoder configuration
   */
  explicit SeyondPCDDecoder(const SeyondPCDDecoderConfig & config = SeyondPCDDecoderConfig());

  /**
   * @brief Destructor
   */
  ~SeyondPCDDecoder() override;

  // Disable copy and move operations (inherited from base class)
  SeyondPCDDecoder(const SeyondPCDDecoder &) = delete;
  SeyondPCDDecoder & operator=(const SeyondPCDDecoder &) = delete;
  SeyondPCDDecoder(SeyondPCDDecoder &&) = delete;
  SeyondPCDDecoder & operator=(SeyondPCDDecoder &&) = delete;

  /**
   * @brief Decode SeyondScan message to PointCloud2 (required by PCDDecoder)
   * @param input The SeyondScan message to decode
   * @return Shared pointer to PointCloud2 message, or nullptr if decoding fails
   */
  sensor_msgs::msg::PointCloud2::SharedPtr decode_typed(
    const bag_converter::msg::SeyondScan & input) override;

private:
  SeyondPCDDecoderConfig config_;
  bool anglehv_table_init_;
  std::vector<char> anglehv_table_;
  std::vector<uint8_t> data_buffer_;

  // Timestamp tracking for relative time calculation
  double scan_start_us_;  // Scan start time in microseconds (epoch time)
  double pkt_offset_us_;  // Current packet's offset from scan start in microseconds

  // Per-packet LiDAR state (from InnoCommonHeader)
  uint8_t pkt_lidar_mode_;    // enum InnoLidarMode
  uint8_t pkt_lidar_status_;  // enum InnoLidarStatus
  uint8_t pkt_lidar_type_;    // enum InnoLidarType (for en_xyzit lidar_type)

  // Per-packet version from InnoCommonVersion (experimental, en_xyzit only)
  uint8_t pkt_version_major_;
  uint8_t pkt_version_minor_;

  // Whether intensity needs to be scaled from [0, 4095] to [0, 255]
  bool scale_intensity_12bit_ = false;

  // Reusable point cloud buffer (avoids per-scan heap allocation)
  pcl::PointCloud<OutputPointT> cloud_;

  void set_angle_hv_table(const std::vector<char> & table);
  void clear_angle_hv_table();
  bool has_angle_hv_table() const { return anglehv_table_init_; }

  void process_packet(
    const bag_converter::msg::SeyondPacket & packet, pcl::PointCloud<OutputPointT> & cloud);

  void convert_and_parse(const InnoDataPacket * pkt, pcl::PointCloud<OutputPointT> & cloud);

  void data_packet_parse(const InnoDataPacket * pkt, pcl::PointCloud<OutputPointT> & cloud);

  template <typename PointType>
  void point_xyz_data_parse(
    bool is_use_refl, uint32_t point_num, PointType point_ptr,
    pcl::PointCloud<OutputPointT> & cloud);

  void coordinate_transfer(OutputPointT * point, float x, float y, float z);
};

}  // namespace bag_converter::decoder::seyond

#endif  // BAG_CONVERTER__SEYOND_DECODER_HPP
