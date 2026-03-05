/*
 *  Copyright (C) 2025 TierIV Inc.
 *
 *  License: Apache License
 *
 *  NebulaPCDDecoder class definition for bag_converter package
 *
 *  Decodes NebulaPackets (nebula_drs) by adapting them to SeyondPCDDecoder,
 *  bypassing the nebula driver for better performance.
 */

#ifndef BAG_CONVERTER__NEBULA_DECODER_HPP
#define BAG_CONVERTER__NEBULA_DECODER_HPP

#include "base_decoder.hpp"
#include "point_types.hpp"
#include "seyond_decoder.hpp"

#include <nebula_msgs/msg/nebula_packets.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace bag_converter::decoder::nebula
{

// Default configuration constants
namespace defaults
{
inline constexpr const char * frame_id = "seyond";
inline constexpr double min_range = 0.3;
inline constexpr double max_range = 200.0;
}  // namespace defaults

// Configuration for the decoder
struct NebulaPCDDecoderConfig
{
  // frame_id for output PointCloud2 (NebulaPackets.header.frame_id is always empty)
  std::string frame_id = defaults::frame_id;

  double min_range = defaults::min_range;
  double max_range = defaults::max_range;

  // Unused fields kept for API compatibility with bag_converter.cpp
  double scan_phase = 0.0;
  double frequency_ms = 100.0;
  bool use_sensor_time = true;
};

/**
 * @brief NebulaPCDDecoder that adapts NebulaPackets for SeyondPCDDecoder
 *
 * Instead of using the nebula driver (which copies every packet, tracks frame
 * boundaries, and requires an extra NebulaPoint -> OutputPointT conversion),
 * this class directly adapts NebulaPackets to the SeyondPCDDecoder path:
 *   1. Detect AngleHV calibration packets from binary header
 *   2. Apply protocol v1 compatibility (16-byte header padding) if needed
 *   3. Filter non-data packets
 *   4. Delegate point decoding to SeyondPCDDecoder's convert_and_parse()
 *
 * @tparam OutputPointT The point type used for output point cloud
 */
template <typename OutputPointT = bag_converter::point::PointXYZIT>
class NebulaPCDDecoder : public PCDDecoder<nebula_msgs::msg::NebulaPackets, OutputPointT>
{
public:
  explicit NebulaPCDDecoder(const NebulaPCDDecoderConfig & config = NebulaPCDDecoderConfig());

  ~NebulaPCDDecoder() override;

  NebulaPCDDecoder(const NebulaPCDDecoder &) = delete;
  NebulaPCDDecoder & operator=(const NebulaPCDDecoder &) = delete;
  NebulaPCDDecoder(NebulaPCDDecoder &&) = delete;
  NebulaPCDDecoder & operator=(NebulaPCDDecoder &&) = delete;

  sensor_msgs::msg::PointCloud2::SharedPtr decode_typed(
    const nebula_msgs::msg::NebulaPackets & input) override;

  [[nodiscard]] NebulaPCDDecoderConfig get_config() const;
  void set_config(const NebulaPCDDecoderConfig & config);

private:
  NebulaPCDDecoderConfig config_;

  // Internal SeyondPCDDecoder that does the actual point decoding
  seyond::SeyondPCDDecoder<OutputPointT> seyond_decoder_;
};

}  // namespace bag_converter::decoder::nebula

#endif  // BAG_CONVERTER__NEBULA_DECODER_HPP
