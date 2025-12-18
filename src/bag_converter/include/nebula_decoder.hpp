/*
 *  Copyright (C) 2025 TierIV Inc.
 *
 *  License: Apache License
 *
 *  NebulaPCDDecoder class definition for bag_converter package
 */

#ifndef BAG_CONVERTER__NEBULA_DECODER_HPP
#define BAG_CONVERTER__NEBULA_DECODER_HPP

#include "base_decoder.hpp"
#include "point_types.hpp"

#include <nebula_common/nebula_common.hpp>
#include <nebula_common/nebula_status.hpp>
#include <nebula_common/seyond/seyond_common.hpp>

// Forward declarations to avoid include issues
namespace nebula
{
namespace drivers
{
class SeyondDriver;
class SeyondSensorConfiguration;
class SeyondCalibrationConfiguration;
}  // namespace drivers
}  // namespace nebula

#include <nebula_msgs/msg/nebula_packets.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <tuple>
#include <vector>

namespace bag_converter::decoder::nebula
{

// Default configuration constants
namespace defaults
{
inline constexpr const char * sensor_model = "Falcon";
inline constexpr const char * frame_id = "seyond";
inline constexpr double min_range = 0.3;
inline constexpr double max_range = 200.0;
inline constexpr double scan_phase = 0.0;
inline constexpr double frequency_ms = 100.0;
inline constexpr bool use_sensor_time = true;
inline constexpr const char * dummy_host_ip = "192.168.1.100";
inline constexpr const char * dummy_sensor_ip = "192.168.1.201";
inline constexpr uint16_t dummy_data_port = 2368;
inline constexpr uint16_t dummy_gnss_port = 2369;
inline constexpr uint16_t dummy_packet_mtu_size = 1500;
}  // namespace defaults

// Configuration for the decoder
struct NebulaPCDDecoderConfig
{
  // Sensor configuration
  std::string sensor_model = defaults::sensor_model;  // or "Robin_W"
  std::string frame_id = defaults::frame_id;

  // Range filtering (will be set in sensor config)
  double min_range = defaults::min_range;
  double max_range = defaults::max_range;

  // Processing options
  double scan_phase = defaults::scan_phase;
  double frequency_ms = defaults::frequency_ms;
  bool use_sensor_time = defaults::use_sensor_time;
};

/**
 * @brief NebulaPCDDecoder class that inherits from PCDDecoder
 *
 * This class provides the decode interface required by PCDDecoder and directly uses
 * the nebula driver for packet processing.
 *
 * @tparam OutputPointT The point type used for output point cloud (default:
 * bag_converter::point::PointXYZIT)
 */
template <typename OutputPointT = bag_converter::point::PointXYZIT>
class NebulaPCDDecoder : public PCDDecoder<nebula_msgs::msg::NebulaPackets, OutputPointT>
{
public:
  /**
   * @brief Constructor with configuration
   * @param config Decoder configuration
   */
  explicit NebulaPCDDecoder(const NebulaPCDDecoderConfig & config = NebulaPCDDecoderConfig());

  /**
   * @brief Destructor
   */
  ~NebulaPCDDecoder() override;

  // Disable copy and move operations (inherited from base class)
  NebulaPCDDecoder(const NebulaPCDDecoder &) = delete;
  NebulaPCDDecoder & operator=(const NebulaPCDDecoder &) = delete;
  NebulaPCDDecoder(NebulaPCDDecoder &&) = delete;
  NebulaPCDDecoder & operator=(NebulaPCDDecoder &&) = delete;

  /**
   * @brief Decode NebulaPackets message to PointCloud2 (required by PCDDecoder)
   * @param input The NebulaPackets message to decode
   * @return Shared pointer to PointCloud2 message, or nullptr if decoding fails
   */
  sensor_msgs::msg::PointCloud2::SharedPtr decode(
    const nebula_msgs::msg::NebulaPackets & input) override;

  /**
   * @brief Process single packet (for streaming processing) - main interface
   * @param packet Raw packet bytes
   * @return Tuple of (complete_cloud, scan_complete_flag)
   */
  std::tuple<::nebula::drivers::NebulaPointCloudPtr, bool> process_packet(
    const std::vector<uint8_t> & packet);

  /**
   * @brief Process multiple packets at once
   * @param packets Vector of packet data
   * @return Complete point cloud if available
   */
  ::nebula::drivers::NebulaPointCloudPtr process_packets(
    const std::vector<std::vector<uint8_t>> & packets);

  /**
   * @brief Convert NebulaPackets message to point cloud
   * @param packets Nebula packets container
   * @return Nebula format point cloud
   */
  ::nebula::drivers::NebulaPointCloudPtr process_nebula_packets(
    const nebula_msgs::msg::NebulaPackets & packets);

  /**
   * @brief Flush any remaining points in the decoder
   * @return Point cloud if available
   */
  ::nebula::drivers::NebulaPointCloudPtr flush_cloud_points();

  /**
   * @brief Get configuration
   * @return Current decoder configuration
   */
  // NOLINTNEXTLINE(modernize-use-nodiscard)
  NebulaPCDDecoderConfig get_config() const;

  /**
   * @brief Set configuration
   * @param config New decoder configuration
   */
  void set_config(const NebulaPCDDecoderConfig & config);

private:
  NebulaPCDDecoderConfig config_;

  // Nebula driver components (like seyond_rosbag_to_pcd)
  std::shared_ptr<::nebula::drivers::SeyondDriver> driver_;
  std::shared_ptr<::nebula::drivers::SeyondSensorConfiguration> sensor_config_;
  std::shared_ptr<::nebula::drivers::SeyondCalibrationConfiguration> calibration_config_;
};

}  // namespace bag_converter::decoder::nebula

#endif  // BAG_CONVERTER__NEBULA_DECODER_HPP
