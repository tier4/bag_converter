/*
 *  Copyright (C) 2025 TierIV Inc.
 *
 *  License: Apache License
 *
 *  Implementation of NebulaPCDDecoder for bag_converter package
 *
 *  Adapts NebulaPackets (raw UDP datagrams from nebula_drs) to
 *  SeyondScan format and delegates decoding to SeyondPCDDecoder.
 */

#include "nebula_decoder.hpp"

#include "sdk_client/inno_lidar_packet_v1_adapt.h"
#include "sdk_common/inno_lidar_packet.h"
#include "sdk_common/inno_lidar_packet_utils.h"

#include <rclcpp/rclcpp.hpp>

namespace bag_converter::decoder::nebula
{

namespace
{

/// Size of a Robin W AngleHV table packet as prepended by nebula_drs: raw calibration blob
/// (InnoDataPacket header + InnoAngleHVTable) without a valid header.
constexpr size_t kRobinWAngleHVTablePacketSize = sizeof(InnoDataPacket) + sizeof(InnoAngleHVTable);

/// If the first NebulaPacket is the raw calibration blob from nebula_drs (Robin W), it has
/// this exact size but may have an uninitialized or invalid InnoDataPacket header. Fix the
/// header so SeyondPCDDecoder and the SDK recognize it as INNO_ROBINW_ITEM_TYPE_ANGLEHV_TABLE.
void fix_robinw_anglehv_table_header(std::vector<uint8_t> & buffer)
{
  if (buffer.size() < sizeof(InnoDataPacket)) {
    return;
  }
  auto * pkt = reinterpret_cast<InnoDataPacket *>(buffer.data());
  pkt->common.version.magic_number = kInnoMagicNumberDataPacket;
  pkt->common.size = static_cast<uint32_t>(buffer.size());
  pkt->type = INNO_ROBINW_ITEM_TYPE_ANGLEHV_TABLE;
  pkt->item_number = 1;
  pkt->item_size = static_cast<uint16_t>(sizeof(InnoAngleHVTable));
  pkt->is_first_sub_frame = 1;
  pkt->is_last_sub_frame = 1;
  pkt->sub_idx = 0;
}

}  // anonymous namespace

bag_converter::msg::SeyondScan nebula_packets_to_seyond_scan(
  const nebula_msgs::msg::NebulaPackets & input, const std::string & frame_id)
{
  // SeyondScan (from Tier4 seyond_ros_driver) contains only data packets (POINTS, HVTABLE); it does
  // not contain status packets or invalid packets. NebulaPackets are raw UDP datagrams and may
  // include status or malformed packets. We detect and skip them below so the output matches
  // SeyondScan semantics.

  bag_converter::msg::SeyondScan scan;
  scan.header = input.header;
  scan.header.frame_id = frame_id;

  const auto logger = rclcpp::get_logger("bag_converter.decoder.nebula");

  size_t skip_status = 0;
  size_t skip_invalid = 0;

  for (size_t i = 0; i < input.packets.size(); ++i) {
    const auto & nebula_pkt = input.packets[i];
    const auto & orig = nebula_pkt.data;
    if (orig.empty()) {
      continue;
    }

    // Lazy copy: use const pointer to original data by default, only copy when mutation is needed.
    // This avoids per-packet allocation for the majority of packets (valid v2 data or skipped).
    std::vector<uint8_t> buffer;
    const std::vector<uint8_t> * data_ptr = &orig;

    // nebula_drs prepends the Robin W calibration as raw bytes (no valid InnoDataPacket header).
    // It is always the first packet when present. Only treat as raw blob when size matches AND
    // the header is not already a valid data packet (magic != kInnoMagicNumberDataPacket), so we
    // never overwrite a proper HVTABLE or mis-detect a same-size POINTS packet.
    if (
      i == 0 && orig.size() == kRobinWAngleHVTablePacketSize &&
      orig.size() >= sizeof(InnoDataPacket)) {
      const auto * pkt = reinterpret_cast<const InnoDataPacket *>(orig.data());
      if (
        pkt->common.version.magic_number == kInnoMagicNumberDataPacket &&
        CHECK_ANGLEHV_TABLE_DATA(pkt->type)) {
        // Already a valid HVTABLE packet (e.g. from sensor or replayed SeyondScan), emit as-is.
        bag_converter::msg::SeyondPacket seyond_pkt;
        seyond_pkt.stamp = nebula_pkt.stamp;
        seyond_pkt.type = bag_converter::msg::SeyondPacket::PACKET_TYPE_HVTABLE;
        seyond_pkt.data = orig;
        scan.packets.push_back(std::move(seyond_pkt));
        continue;
      }
      if (pkt->common.version.magic_number != kInnoMagicNumberDataPacket) {
        // Raw calibration blob: header uninitialized or wrong magic → copy, fix, and emit.
        buffer = orig;
        fix_robinw_anglehv_table_header(buffer);
        RCLCPP_INFO(
          logger,
          "First packet recognized as Robin W AngleHV table (raw calibration blob), fixed header "
          "and emitted as HVTABLE");
        bag_converter::msg::SeyondPacket seyond_pkt;
        seyond_pkt.stamp = nebula_pkt.stamp;
        seyond_pkt.type = bag_converter::msg::SeyondPacket::PACKET_TYPE_HVTABLE;
        seyond_pkt.data = std::move(buffer);
        scan.packets.push_back(std::move(seyond_pkt));
        continue;
      }
      // Same size but valid data packet with different type (e.g. POINTS) → fall through to normal
      // path for magic/type/v1 handling and classification.
    }

    if (data_ptr->size() < sizeof(InnoCommonVersion)) {
      ++skip_invalid;
      continue;
    }
    const auto magic = reinterpret_cast<const InnoCommonVersion *>(data_ptr->data())->magic_number;
    if (magic == kInnoMagicNumberStatusPacket) {
      ++skip_status;
      continue;
    }
    if (magic != kInnoMagicNumberDataPacket) {
      ++skip_invalid;
      continue;
    }

    // Apply protocol v1 -> v2 compatibility (SDK: InnoPacketV1Adapt)
    // Only copy when v1 adaptation is actually needed.
    if (data_ptr->size() >= sizeof(InnoDataPacketV1)) {
      const auto * v1 = reinterpret_cast<const InnoDataPacketV1 *>(data_ptr->data());
      if (v1->common.version.major_version == InnoPacketV1Adapt::kInnoProtocolMajorV1) {
        buffer = orig;
        buffer.insert(
          buffer.begin() + sizeof(InnoDataPacketV1), InnoPacketV1Adapt::kMemorryFrontGap, 0);
        auto * pkt = reinterpret_cast<InnoDataPacket *>(buffer.data());
        pkt->common.size = v1->common.size + InnoPacketV1Adapt::kMemorryFrontGap;
        pkt->common.version.major_version = InnoPacketV1Adapt::kInnoProtocolMajorV2;
        pkt->common.version.minor_version = InnoPacketV1Adapt::kInnoProtocolMinorV2;
        data_ptr = &buffer;
      }
    }

    if (data_ptr->size() < sizeof(InnoDataPacket)) {
      continue;
    }

    const auto * pkt = reinterpret_cast<const InnoDataPacket *>(data_ptr->data());
    if (pkt->type == INNO_ITEM_TYPE_MESSAGE || pkt->type == INNO_ITEM_TYPE_MESSAGE_LOG) {
      continue;
    }

    bag_converter::msg::SeyondPacket seyond_pkt;
    seyond_pkt.stamp = nebula_pkt.stamp;
    seyond_pkt.type = CHECK_ANGLEHV_TABLE_DATA(pkt->type)
                        ? bag_converter::msg::SeyondPacket::PACKET_TYPE_HVTABLE
                        : bag_converter::msg::SeyondPacket::PACKET_TYPE_POINTS;
    if (data_ptr == &buffer) {
      seyond_pkt.data = std::move(buffer);
    } else {
      seyond_pkt.data = orig;
    }
    scan.packets.push_back(std::move(seyond_pkt));
  }

  if (skip_status > 0) {
    RCLCPP_DEBUG(logger, "Skipped %zu status packets", skip_status);
  }
  if (skip_invalid > 0) {
    RCLCPP_WARN(logger, "Skipped %zu invalid packets", skip_invalid);
  }

  return scan;
}

// NebulaPCDDecoder implementation
template <typename OutputPointT>
NebulaPCDDecoder<OutputPointT>::NebulaPCDDecoder(const NebulaPCDDecoderConfig & config)
: config_(config)
{
  seyond::SeyondPCDDecoderConfig seyond_config;
  seyond_config.min_range = config.min_range;
  seyond_config.max_range = config.max_range;
  seyond_decoder_.set_config(seyond_config);
}

template <typename OutputPointT>
NebulaPCDDecoder<OutputPointT>::~NebulaPCDDecoder() = default;

template <typename OutputPointT>
sensor_msgs::msg::PointCloud2::SharedPtr NebulaPCDDecoder<OutputPointT>::decode_typed(
  const nebula_msgs::msg::NebulaPackets & input)
{
  bag_converter::msg::SeyondScan scan = nebula_packets_to_seyond_scan(input, config_.frame_id);

  if (scan.packets.empty()) {
    return nullptr;
  }

  return seyond_decoder_.decode_typed(scan);
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

  seyond::SeyondPCDDecoderConfig seyond_config;
  seyond_config.min_range = config.min_range;
  seyond_config.max_range = config.max_range;
  seyond_decoder_.set_config(seyond_config);
}

// Explicit template instantiations
template class NebulaPCDDecoder<bag_converter::point::PointXYZIT>;
template class NebulaPCDDecoder<bag_converter::point::PointXYZI>;
template class NebulaPCDDecoder<bag_converter::point::PointEnXYZIT>;

}  // namespace bag_converter::decoder::nebula
