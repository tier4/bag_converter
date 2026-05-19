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
enum class PointType { kXYZIT, kXYZI, kEnXYZIT };

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
 * - intensity (float, 0–255)
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
 * - intensity (float, 0–255)
 * - time_stamp: relative timestamp from scan start in nanoseconds (uint32_t)
 */
struct EIGEN_ALIGN16 PointXYZIT
{
  PCL_ADD_POINT4D;
  float intensity;
  uint32_t time_stamp;
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
constexpr uint16_t HAS_IS_2ND_RETURN = 1u << 7;
constexpr uint16_t HAS_MULTI_RETURN_MODE = 1u << 8;
constexpr uint16_t HAS_USE_REFLECTANCE = 1u << 9;
}  // namespace en_xyzit_flags

/**
 * @brief Extended point type: XYZ + intensity + timestamp + optional extended fields
 *
 * Experimental. Extended fields use an availability mask (flags) and minimal unsigned types.
 * When a property is not supported, its flag bit is 0 and the value is 0 (must be ignored).
 *
 * Base: x, y, z (float), intensity (float, 0–255), time_stamp (uint32_t).
 * Extended (all uint8_t except flags): flags, refl_type (0-2), elongation (0-15),
 * lidar_status (0-3), lidar_mode (0-9), pkt_version_major/minor (0-255), lidar_type (0-7),
 * is_2nd_return (0-1), multi_return_mode (0-3), use_reflectance (0-1).
 * Value when not supported: 0.
 */
struct EIGEN_ALIGN16 PointEnXYZIT
{
  PCL_ADD_POINT4D;
  float intensity;
  uint32_t time_stamp;
  uint16_t flags;        ///< en_xyzit_flags: which extended fields are valid
  uint8_t refl_type;     ///< 0=normal, 1=ground, 2=fog. When not supported: 0.
  uint8_t elongation;    ///< 0-15. When not supported: 0.
  uint8_t lidar_status;  ///< 0=none, 1=transition, 2=normal, 3=failed. When not supported: 0.
  uint8_t
    lidar_mode;  ///< 1=sleep, 2=standby, 3=work_normal, 6=protection, etc. When not supported: 0.
  uint8_t pkt_version_major;  ///< 0-255. When not supported: 0.
  uint8_t pkt_version_minor;  ///< 0-255. When not supported: 0.
  uint8_t lidar_type;     ///< Seyond LiDAR type 0-7 (see docs/en_xyzit.md). When not supported: 0.
  uint8_t is_2nd_return;  ///< 0=1st echo, 1=2nd echo. When not supported: 0.
  uint8_t multi_return_mode;  ///< 0=none, 1=single, 2=2_strongest, 3=2_strongest_furthest.
                              ///< When not supported: 0.
  uint8_t use_reflectance;    ///< 0=intensity mode, 1=reflectance mode. When not supported: 0.
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
  (float, x,
   x)(float, y, y)(float, z, z)(float, intensity, intensity)(uint32_t, time_stamp, time_stamp))

POINT_CLOUD_REGISTER_POINT_STRUCT(
  bag_converter::point::PointEnXYZIT,
  (float, x,
   x)(float, y, y)(float, z, z)(float, intensity, intensity)(uint32_t, time_stamp, time_stamp)(
    uint16_t, flags, flags)(uint8_t, refl_type, refl_type)(uint8_t, elongation, elongation)(
    uint8_t, lidar_status,
    lidar_status)(uint8_t, lidar_mode, lidar_mode)(uint8_t, pkt_version_major, pkt_version_major)(
    uint8_t, pkt_version_minor,
    pkt_version_minor)(uint8_t, lidar_type, lidar_type)(uint8_t, is_2nd_return, is_2nd_return)(
    uint8_t, multi_return_mode, multi_return_mode)(uint8_t, use_reflectance, use_reflectance))

#endif  // BAG_CONVERTER__POINT_TYPES_HPP
