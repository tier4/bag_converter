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

#include <seyond_decoder/msg/seyond_packet.hpp>
#include <seyond_decoder/msg/seyond_scan.hpp>

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

namespace bag_converter::decoder
{

// Default configuration constants
namespace seyond_defaults
{
inline constexpr double min_range = 0.3;
inline constexpr double max_range = 200.0;
inline constexpr int coordinate_mode = 3;
inline constexpr bool use_reflectance = true;
inline constexpr const char * frame_id = "lidar";
}  // namespace seyond_defaults

// Configuration for the decoder
struct SeyondPCDDecoderConfig
{
  double min_range = seyond_defaults::min_range;
  double max_range = seyond_defaults::max_range;
  int coordinate_mode = seyond_defaults::coordinate_mode;
  bool use_reflectance = seyond_defaults::use_reflectance;
  std::string frame_id = seyond_defaults::frame_id;
};

/**
 * @brief SeyondPCDDecoder class that inherits from PCDDecoder
 *
 * This class provides the decode interface required by PCDDecoder and uses
 * the Seyond decoder logic for packet processing.
 */
class SeyondPCDDecoder : public PCDDecoder<seyond_decoder::msg::SeyondScan>
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

  /**
   * @brief Decode SeyondScan message to PointCloud2 (required by PCDDecoder)
   * @param input The SeyondScan message to decode
   * @return Shared pointer to PointCloud2 message, or nullptr if decoding fails
   */
  sensor_msgs::msg::PointCloud2::SharedPtr decode(
    const seyond_decoder::msg::SeyondScan & input) override;

  /**
   * @brief Get configuration
   * @return Current decoder configuration
   */
  SeyondPCDDecoderConfig get_config() const;

  /**
   * @brief Set configuration
   * @param config New decoder configuration
   */
  void set_config(const SeyondPCDDecoderConfig & config);

  /**
   * @brief Set angle HV table
   * @param table HV table data
   */
  void set_angle_hv_table(const std::vector<char> & table);

  /**
   * @brief Clear angle HV table
   */
  void clear_angle_hv_table();

  /**
   * @brief Check if angle HV table is initialized
   * @return True if HV table is initialized
   */
  bool has_angle_hv_table() const { return anglehv_table_init_; }

private:
  void process_packet(
    const seyond_decoder::msg::SeyondPacket & packet,
    pcl::PointCloud<bag_converter::point::PointXYZIT> & cloud);

  void convert_and_parse(
    const InnoDataPacket * pkt, pcl::PointCloud<bag_converter::point::PointXYZIT> & cloud);

  void data_packet_parse(
    const InnoDataPacket * pkt, pcl::PointCloud<bag_converter::point::PointXYZIT> & cloud);

  template <typename PointType>
  void point_xyz_data_parse(
    bool is_use_refl, uint32_t point_num, PointType point_ptr,
    pcl::PointCloud<bag_converter::point::PointXYZIT> & cloud);

  void coordinate_transfer(
    bag_converter::point::PointXYZIT * point, int mode, float x, float y, float z);

private:
  SeyondPCDDecoderConfig config_;
  bool anglehv_table_init_;
  std::vector<char> anglehv_table_;
  std::vector<uint8_t> data_buffer_;
  double current_ts_start_;

  static constexpr double us_in_second_c = 1000000.0;
  static constexpr double ten_us_in_second_c = 100000.0;
};

}  // namespace bag_converter::decoder

#endif  // BAG_CONVERTER__SEYOND_DECODER_HPP
