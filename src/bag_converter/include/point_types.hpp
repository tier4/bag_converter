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

/**
 * @brief Extended point type: XYZ + intensity + timestamp + refl_type (packet type)
 *
 * This extended type is experimental and may occasionally reflect features of specific
 * LiDAR(s). As with all point types here, use -1 for any field that cannot be obtained
 * from the packet (LiDAR-dependent).
 *
 * It includes:
 * - x, y, z coordinates (float)
 * - intensity (float)
 * - t_us: [DEPRECATED, will be removed in v0.6.0] Replaced by `timestamp`.
 *         Relative timestamp from scan start in microseconds (uint32_t)
 * - timestamp: relative timestamp from scan start in nanoseconds (uint32_t)
 * - refl_type: point classification (0: normal, 1: ground, 2: fog; -1: not available; int8_t)
 * - elongation: raw elongation value 0-15 when available; -1: not available (int16_t)
 * - lidar_status: LiDAR status (0: none, 1: transition, 2: normal, 3: failed; -1: not available)
 * - lidar_mode: LiDAR mode (1: sleep, 2: standby, 3: work_normal, 6: protection; -1: not available)
 * - pkt_version_major: packet protocol major (0-255), or -1 when not provided (experimental,
 *   en_xyzit only)
 * - pkt_version_minor: packet protocol minor (0-255), or -1 when not provided (experimental,
 *   en_xyzit only)
 * - lidar_type: LiDAR type from packet (experimental, Seyond LiDAR only; -1 for other sensors or
 *   when not provided)
 */
struct EIGEN_ALIGN16 PointEnXYZIT
{
  PCL_ADD_POINT4D;
  float intensity;
  uint32_t t_us;  ///< @deprecated Will be removed in v0.6.0. Replaced by `timestamp`.
  uint32_t timestamp;
  int8_t refl_type;
  int16_t elongation;
  int8_t lidar_status;
  int8_t lidar_mode;
  int16_t pkt_version_major;  ///< Experimental, en_xyzit only. -1 when not provided.
  int16_t pkt_version_minor;  ///< Experimental, en_xyzit only. -1 when not provided.
  int16_t lidar_type;  ///< Experimental, Seyond LiDAR only. -1 when not provided or non-Seyond.
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
  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(uint32_t, t_us, t_us)(
    uint32_t, timestamp, timestamp))

POINT_CLOUD_REGISTER_POINT_STRUCT(
  bag_converter::point::PointEnXYZIT,
  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(uint32_t, t_us, t_us)(
    uint32_t, timestamp, timestamp)(int8_t, refl_type, refl_type)(int16_t, elongation, elongation)(
    int8_t, lidar_status,
    lidar_status)(int8_t, lidar_mode, lidar_mode)(int16_t, pkt_version_major, pkt_version_major)(
    int16_t, pkt_version_minor, pkt_version_minor)(int16_t, lidar_type, lidar_type))

#endif  // BAG_CONVERTER__POINT_TYPES_HPP
