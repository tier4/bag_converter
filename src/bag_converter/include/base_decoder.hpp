/*
 *  Copyright (C) 2025 Seyond Inc.
 *
 *  License: Apache License
 *
 *  PCD decoder class template for converting various message types to point clouds
 */

#ifndef BAG_CONVERTER__PCD_DECODER_HPP
#define BAG_CONVERTER__PCD_DECODER_HPP

#include "point_types.hpp"

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <memory>

namespace bag_converter::decoder
{

/**
 * @brief PCD decoder class template
 *
 * This abstract class provides a common interface for decoders that convert
 * various message types to PointCloud2.
 *
 * @tparam InputScanT The input scan message type to be decoded
 * @tparam OutputPointT The point type used internally for point cloud processing
 * (default: bag_converter::point::PointXYZIT)
 */
template <typename InputScanT, typename OutputPointT = bag_converter::point::PointXYZIT>
class PCDDecoder
{
public:
  PCDDecoder() = default;
  virtual ~PCDDecoder() = default;

  // Disable copy and move operations for abstract base class
  PCDDecoder(const PCDDecoder &) = delete;
  PCDDecoder & operator=(const PCDDecoder &) = delete;
  PCDDecoder(PCDDecoder &&) = delete;
  PCDDecoder & operator=(PCDDecoder &&) = delete;

  /**
   * @brief Decode input scan message to PointCloud2
   *
   * Pure virtual method that must be implemented by derived classes.
   * Converts the input scan message of type InputScanT to a PointCloud2 message.
   *
   * @param input The input scan message to decode
   * @return Shared pointer to PointCloud2 message, or nullptr if decoding fails
   */
  virtual sensor_msgs::msg::PointCloud2::SharedPtr decode(const InputScanT & input) = 0;
};

}  // namespace bag_converter::decoder

#endif  // BAG_CONVERTER__PCD_DECODER_HPP
