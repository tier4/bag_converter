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
 * - t_us: relative timestamp from scan start in microseconds (uint32_t)
 * - timestamp: relative timestamp from scan start in nanoseconds (uint32_t)
 */
struct EIGEN_ALIGN16 PointXYZIT
{
  PCL_ADD_POINT4D;
  float intensity;
  uint32_t t_us;
  uint32_t timestamp;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/**
 * @brief Extended point type: XYZ + intensity + timestamp + refl_type (packet type)
 *
 * Extended version of PointXYZIT with refl_type from the sensor packet.
 * It includes:
 * - x, y, z coordinates (float)
 * - intensity (float)
 * - t_us: relative timestamp from scan start in microseconds (uint32_t)
 * - timestamp: relative timestamp from scan start in nanoseconds (uint32_t)
 * - refl_type: point classification (0: normal, 1: ground, 2: fog; -1: not available; int8_t)
 * - elongation: raw elongation value 0-15 when available; -1: not available (int16_t)
 */
struct EIGEN_ALIGN16 PointEnXYZIT
{
  PCL_ADD_POINT4D;
  float intensity;
  uint32_t t_us;
  uint32_t timestamp;
  int8_t refl_type;
  int16_t elongation;
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
    uint32_t, timestamp, timestamp)(int8_t, refl_type, refl_type)(int16_t, elongation, elongation))

#endif  // BAG_CONVERTER__POINT_TYPES_HPP
