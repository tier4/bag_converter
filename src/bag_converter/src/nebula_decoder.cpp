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

#include <cstddef>
#include <cstring>

namespace bag_converter::decoder::nebula
{

namespace
{

// Minimum packet size for validation (no SDK equivalent)
constexpr size_t kMinPacketSize = 60;

// Byte offsets derived from SDK struct layout
constexpr size_t kMagicNumberOffset =
  offsetof(InnoCommonHeader, version) + offsetof(InnoCommonVersion, magic_number);
constexpr size_t kMajorVersionOffset =
  offsetof(InnoCommonHeader, version) + offsetof(InnoCommonVersion, major_version);
constexpr size_t kMinorVersionOffset =
  offsetof(InnoCommonHeader, version) + offsetof(InnoCommonVersion, minor_version);
constexpr size_t kSizeFieldOffset = offsetof(InnoCommonHeader, size);

/// Check if a raw packet has a valid Seyond data packet header.
bool is_valid_data_packet(const std::vector<uint8_t> & data)
{
  if (data.size() < kMinPacketSize) {
    return false;
  }
  uint16_t magic;
  std::memcpy(&magic, data.data() + kMagicNumberOffset, sizeof(magic));
  if (magic != kInnoMagicNumberDataPacket) {
    return false;
  }
  // Reject non-data packet types (MESSAGE=2, MESSAGE_LOG=3)
  // The type field offset depends on protocol version, but for filtering
  // we use the InnoDataPacket layout after v1 compat has been applied.
  return true;
}

/// Check if a raw packet is an AngleHV calibration table packet.
/// Must be called AFTER v1 compatibility has been applied.
bool is_anglehv_table_packet(const std::vector<uint8_t> & data)
{
  if (data.size() < sizeof(InnoDataPacket)) {
    return false;
  }
  const auto * pkt = reinterpret_cast<const InnoDataPacket *>(data.data());
  return CHECK_ANGLEHV_TABLE_DATA(pkt->type);
}

/// Apply protocol v1 -> v2 compatibility by inserting 16 zero bytes
/// at offset kV1HeaderLen to pad the header to v2 size.
/// Modifies the buffer in place and updates the packet size field.
void apply_v1_compat(std::vector<uint8_t> & buffer)
{
  if (buffer.size() < sizeof(InnoDataPacketV1)) {
    return;
  }
  uint8_t major_version = buffer[kMajorVersionOffset];
  if (major_version != InnoPacketV1Adapt::kInnoProtocolMajorV1) {
    return;
  }
  // Update packet size field
  uint32_t packet_size;
  std::memcpy(&packet_size, &buffer[kSizeFieldOffset], sizeof(packet_size));
  packet_size += InnoPacketV1Adapt::kMemorryFrontGap;
  std::memcpy(&buffer[kSizeFieldOffset], &packet_size, sizeof(packet_size));
  // Update version fields to match SDK conversion output
  buffer[kMajorVersionOffset] = InnoPacketV1Adapt::kInnoProtocolMajorV2;
  buffer[kMinorVersionOffset] = InnoPacketV1Adapt::kInnoProtocolMinorV2;
  // Insert zero bytes at the old header boundary to pad to v2
  buffer.insert(buffer.begin() + sizeof(InnoDataPacketV1), InnoPacketV1Adapt::kMemorryFrontGap, 0);
}

}  // anonymous namespace

// NebulaPCDDecoder implementation
template <typename OutputPointT>
NebulaPCDDecoder<OutputPointT>::NebulaPCDDecoder(const NebulaPCDDecoderConfig & config)
: config_(config)
{
  // Configure the internal SeyondPCDDecoder
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
  // Build a SeyondScan message from adapted NebulaPackets
  bag_converter::msg::SeyondScan scan;
  scan.header = input.header;
  // NebulaPackets.header.frame_id is always empty (nebula_drs does not set it)
  scan.header.frame_id = config_.frame_id;

  for (const auto & nebula_pkt : input.packets) {
    if (nebula_pkt.data.empty()) {
      continue;
    }

    // Copy the raw data so we can apply v1 compat in place
    std::vector<uint8_t> buffer = nebula_pkt.data;

    // Validate magic number and minimum size
    if (!is_valid_data_packet(buffer)) {
      continue;
    }

    // Apply protocol v1 -> v2 compatibility (16-byte header padding)
    apply_v1_compat(buffer);

    // Now check packet type after v1 compat
    if (buffer.size() < sizeof(InnoDataPacket)) {
      continue;
    }

    const auto * pkt = reinterpret_cast<const InnoDataPacket *>(buffer.data());

    // Filter non-data packets
    if (pkt->type == INNO_ITEM_TYPE_MESSAGE || pkt->type == INNO_ITEM_TYPE_MESSAGE_LOG) {
      continue;
    }

    // Build a SeyondPacket
    bag_converter::msg::SeyondPacket seyond_pkt;
    seyond_pkt.stamp = nebula_pkt.stamp;

    if (is_anglehv_table_packet(buffer)) {
      seyond_pkt.type = bag_converter::msg::SeyondPacket::PACKET_TYPE_HVTABLE;
    } else {
      seyond_pkt.type = bag_converter::msg::SeyondPacket::PACKET_TYPE_POINTS;
    }

    seyond_pkt.data = std::move(buffer);
    scan.packets.push_back(std::move(seyond_pkt));
  }

  if (scan.packets.empty()) {
    return nullptr;
  }

  // Delegate to SeyondPCDDecoder
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
