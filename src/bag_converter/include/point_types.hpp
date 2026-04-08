/*
 *  Copyright (C) 2025 Seyond Inc.
 *
 *  License: Apache License
 *
 *  Point type definitions for bag converter
 */

#ifndef BAG_CONVERTER__POINT_TYPES_HPP
#define BAG_CONVERTER__POINT_TYPES_HPP

#include <pcl/point_types.h>

#include <cstdint>

namespace bag_converter
{

/**
 * @brief Output point type for point cloud conversion
 */
enum class PointType { kXYZIT, kXYZI, kEnXYZIT, kNebula };

}  // namespace bag_converter

namespace bag_converter::point
{

/**
 * Point types in this header are LiDAR-agnostic: they are not tied to a specific sensor.
 * Depending on the LiDAR and packet format, some fields may be unavailable (e.g. packet
 * version absent, or only major present). Use -1 for any field that cannot be obtained
 * from the packet.
 */

/**
 * @brief Point type with XYZ coordinates and intensity
 *
 * This point type is used for point cloud conversion in bag files.
 * It includes:
 * - x, y, z coordinates (float)
 * - intensity (float)
 */
struct EIGEN_ALIGN16 PointXYZI
{
  PCL_ADD_POINT4D;
  float intensity;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/**
 * @brief Point type with XYZ coordinates, intensity, and timestamp
 *
 * This point type is used for point cloud conversion in bag files.
 * It includes:
 * - x, y, z coordinates (float)
 * - intensity (float)
 * - t_us: [DEPRECATED, will be removed in v0.6.0] Replaced by `timestamp`.
 *         Relative timestamp from scan start in microseconds (uint32_t)
 * - timestamp: relative timestamp from scan start in nanoseconds (uint32_t)
 */
struct EIGEN_ALIGN16 PointXYZIT
{
  PCL_ADD_POINT4D;
  float intensity;
  uint32_t t_us;  ///< @deprecated Will be removed in v0.6.0. Replaced by `timestamp`.
  uint32_t timestamp;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/// Availability flags for PointEnXYZIT extended fields. When a bit is set, the corresponding
/// value is valid; when clear, the value is 0 and must be ignored.
namespace en_xyzit_flags
{
constexpr uint16_t HAS_REFL_TYPE = 1u << 0;
constexpr uint16_t HAS_ELONGATION = 1u << 1;
constexpr uint16_t HAS_LIDAR_STATUS = 1u << 2;
constexpr uint16_t HAS_LIDAR_MODE = 1u << 3;
constexpr uint16_t HAS_PKT_VERSION_MAJOR = 1u << 4;
constexpr uint16_t HAS_PKT_VERSION_MINOR = 1u << 5;
constexpr uint16_t HAS_LIDAR_TYPE = 1u << 6;
}  // namespace en_xyzit_flags

/**
 * @brief Extended point type: XYZ + intensity + timestamp + optional extended fields
 *
 * Experimental. Extended fields use an availability mask (flags) and minimal unsigned types.
 * When a property is not supported, its flag bit is 0 and the value is 0 (must be ignored).
 *
 * Base: x, y, z (float), intensity (float), t_us (deprecated), timestamp (uint32_t).
 * Extended (all uint8_t except flags): flags, refl_type (0-2), elongation (0-15),
 * lidar_status (0-3), lidar_mode (0-9), pkt_version_major/minor (0-255), lidar_type (0-7).
 * Value when not supported: 0.
 */
struct EIGEN_ALIGN16 PointEnXYZIT
{
  PCL_ADD_POINT4D;
  float intensity;
  uint32_t t_us;  ///< @deprecated Will be removed in v0.6.0. Replaced by `timestamp`.
  uint32_t timestamp;
  uint16_t flags;        ///< en_xyzit_flags: which extended fields are valid
  uint8_t refl_type;     ///< 0=normal, 1=ground, 2=fog. When not supported: 0.
  uint8_t elongation;    ///< 0-15. When not supported: 0.
  uint8_t lidar_status;  ///< 0=none, 1=transition, 2=normal, 3=failed. When not supported: 0.
  uint8_t
    lidar_mode;  ///< 1=sleep, 2=standby, 3=work_normal, 6=protection, etc. When not supported: 0.
  uint8_t pkt_version_major;  ///< 0-255. When not supported: 0.
  uint8_t pkt_version_minor;  ///< 0-255. When not supported: 0.
  uint8_t lidar_type;  ///< Seyond LiDAR type 0-7 (see docs/en_xyzit.md). When not supported: 0.
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/// nebula ReturnType values used for PointXYZIRCAEDT.return_type
namespace return_type
{
constexpr uint8_t UNKNOWN = 0;
constexpr uint8_t STRONGEST = 3;
constexpr uint8_t SECONDSTRONGEST = 8;
}  // namespace return_type

/**
 * @brief Nebula-compatible point type: XYZ + intensity + return_type + channel + azimuth +
 *        elevation + distance + time_stamp
 *
 * Matches nebula::drivers::PointXYZIRCAEDT layout for downstream compatibility with
 * Autoware/nebula consumers.
 *
 * This point type intentionally does NOT use PCL_ADD_POINT4D to avoid a 32-bit dummy word,
 * following nebula's design for SSE-aligned structs.
 *
 * Note: azimuth and elevation are always 0 because the decoder operates on XYZ-converted packets
 * which do not retain the original spherical angles. This matches nebula's own decoder behavior.
 */
struct alignas(16) PointXYZIRCAEDT
{
  float x;
  float y;
  float z;
  uint8_t intensity;    ///< Reflectance/intensity (0-255)
  uint8_t return_type;  ///< nebula ReturnType (see return_type namespace)
  uint16_t channel;     ///< Laser channel / ring ID
  float azimuth;        ///< Horizontal angle (rad); always 0 (unavailable from XYZ packets)
  float elevation;      ///< Vertical angle (rad); always 0 (unavailable from XYZ packets)
  float distance;       ///< Distance from sensor origin (meters)
  uint32_t time_stamp;  ///< Relative timestamp from scan start (nanoseconds)
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/**
 * @brief Convert PointType enum to string representation
 * @param point_type The point type enum value
 * @return String representation of the point type
 */
inline const char * point_type_to_string(bag_converter::PointType point_type)
{
  switch (point_type) {
    case bag_converter::PointType::kXYZIT:
      return "xyzit";
    case bag_converter::PointType::kXYZI:
      return "xyzi";
    case bag_converter::PointType::kEnXYZIT:
      return "en_xyzit";
    case bag_converter::PointType::kNebula:
      return "nebula";
  }
  return "unknown";
}

}  // namespace bag_converter::point

// Register point types with PCL
POINT_CLOUD_REGISTER_POINT_STRUCT(
  bag_converter::point::PointXYZI,
  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity))

POINT_CLOUD_REGISTER_POINT_STRUCT(
  bag_converter::point::PointXYZIT,
  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(uint32_t, t_us, t_us)(
    uint32_t, timestamp, timestamp))

POINT_CLOUD_REGISTER_POINT_STRUCT(
  bag_converter::point::PointEnXYZIT,
  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(uint32_t, t_us, t_us)(
    uint32_t, timestamp, timestamp)(uint16_t, flags, flags)(uint8_t, refl_type, refl_type)(
    uint8_t, elongation, elongation)(uint8_t, lidar_status, lidar_status)(
    uint8_t, lidar_mode, lidar_mode)(uint8_t, pkt_version_major, pkt_version_major)(
    uint8_t, pkt_version_minor, pkt_version_minor)(uint8_t, lidar_type, lidar_type))

POINT_CLOUD_REGISTER_POINT_STRUCT(
  bag_converter::point::PointXYZIRCAEDT,
  (float, x, x)(float, y, y)(float, z, z)(std::uint8_t, intensity, intensity)(
    std::uint8_t, return_type,
    return_type)(std::uint16_t, channel, channel)(float, azimuth, azimuth)(
    float, elevation, elevation)(float, distance, distance)(std::uint32_t, time_stamp, time_stamp))

#endif  // BAG_CONVERTER__POINT_TYPES_HPP
