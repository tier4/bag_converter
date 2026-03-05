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

#include "sdk_common/inno_lidar_packet.h"
#include "sdk_common/inno_lidar_packet_utils.h"

#include <rclcpp/rclcpp.hpp>

#include <cstring>

namespace bag_converter::decoder::nebula
{

namespace
{

// Protocol constants (matching nebula_drs SeyondDecoder values)
constexpr uint16_t kMagicNumber = 0x176A;
constexpr uint8_t kProtocolMajorV1 = 1;
constexpr uint16_t kV1HeaderLen = 54;  // InnoDataPacketV1 header size
constexpr uint16_t kV2HeaderLen = 70;  // InnoDataPacket header size (kV1HeaderLen + 16)
constexpr size_t kMinPacketSize = 60;

// Byte offset of major_version in the raw packet buffer
constexpr size_t kMajorVersionOffset = 2;

/// Check if a raw packet has a valid Seyond data packet header.
bool is_valid_data_packet(const std::vector<uint8_t> & data)
{
  if (data.size() < kMinPacketSize) {
    return false;
  }
  uint16_t magic;
  std::memcpy(&magic, data.data(), sizeof(magic));
  if (magic != kMagicNumber) {
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
  if (buffer.size() < kV1HeaderLen) {
    return;
  }
  uint8_t major_version = buffer[kMajorVersionOffset];
  if (major_version != kProtocolMajorV1) {
    return;
  }
  // Update packet size field (uint32_t at offset 4 in InnoCommonHeader)
  constexpr size_t kSizeFieldOffset = 4;
  uint32_t packet_size;
  std::memcpy(&packet_size, &buffer[kSizeFieldOffset], sizeof(packet_size));
  packet_size += 16;
  std::memcpy(&buffer[kSizeFieldOffset], &packet_size, sizeof(packet_size));
  // Insert 16 zero bytes at the old header boundary
  buffer.insert(buffer.begin() + kV1HeaderLen, 16, 0);
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
  seyond_config.frame_id = config.frame_id;
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

    // Filter non-data packets (MESSAGE=2, MESSAGE_LOG=3)
    if (pkt->type == 2 || pkt->type == 3) {
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
  seyond_config.frame_id = config.frame_id;
  seyond_decoder_.set_config(seyond_config);
}

// Explicit template instantiations
template class NebulaPCDDecoder<bag_converter::point::PointXYZIT>;
template class NebulaPCDDecoder<bag_converter::point::PointXYZI>;
template class NebulaPCDDecoder<bag_converter::point::PointEnXYZIT>;

}  // namespace bag_converter::decoder::nebula
