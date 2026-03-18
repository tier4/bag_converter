/*
 *  Copyright (C) 2025 TierIV Inc.
 *
 *  License: Apache License
 *
 *  NebulaPCDDecoder: thin wrapper around SeyondPCDDecoder that converts
 *  NebulaPackets to SeyondScan internally and delegates point decoding.
 */

#ifndef BAG_CONVERTER__NEBULA_DECODER_HPP
#define BAG_CONVERTER__NEBULA_DECODER_HPP

#include "base_decoder.hpp"
#include "point_types.hpp"
#include "seyond_decoder.hpp"

#include <bag_converter/msg/seyond_scan.hpp>

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
}  // namespace defaults

/**
 * @brief Convert NebulaPackets to SeyondScan (data packet filtering and v1 compat).
 * @param input Raw packets from nebula_drs
 * @param frame_id Frame ID for the output scan header
 * @return SeyondScan with HVTABLE and POINTS packets only, or empty packets if none
 */
bag_converter::msg::SeyondScan nebula_packets_to_seyond_scan(
  const nebula_msgs::msg::NebulaPackets & input, const std::string & frame_id);

// Configuration for the decoder
struct NebulaPCDDecoderConfig : public BasePCDDecoderConfig
{
  // frame_id for output PointCloud2 (NebulaPackets.header.frame_id is always empty)
  std::string frame_id = defaults::frame_id;
};

/**
 * @brief Thin wrapper around SeyondPCDDecoder.
 *
 * Converts NebulaPackets to SeyondScan internally (via nebula_packets_to_seyond_scan)
 * and delegates point cloud decoding to SeyondPCDDecoder.
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

private:
  NebulaPCDDecoderConfig config_;

  // Internal SeyondPCDDecoder that does the actual point decoding
  seyond::SeyondPCDDecoder<OutputPointT> seyond_decoder_;
};

}  // namespace bag_converter::decoder::nebula

#endif  // BAG_CONVERTER__NEBULA_DECODER_HPP
