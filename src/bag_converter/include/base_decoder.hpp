/*
 *  Copyright (C) 2025 TierIV Inc.
 *
 *  License: Apache License
 *
 *  PCD decoder base class and template for converting various message types to point clouds
 */

#ifndef BAG_CONVERTER_BASE_DECODER_HPP
#define BAG_CONVERTER_BASE_DECODER_HPP

#include "point_types.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <memory>

namespace bag_converter::decoder
{

namespace defaults
{

inline constexpr double min_range = 0.3;
inline constexpr double max_range = 200.0;
inline constexpr size_t min_points_per_scan = 1000;

}  // namespace defaults

struct BasePCDDecoderConfig
{
  double min_range = defaults::min_range;
  double max_range = defaults::max_range;
};

/**
 * @brief Abstract base class for PCD decoders using type erasure pattern
 *
 * This non-template base class provides a common interface for all decoders,
 * allowing them to be stored in a single container and used polymorphically.
 * The decode method accepts serialized messages, enabling type-agnostic processing.
 */
class BasePCDDecoder
{
public:
  BasePCDDecoder() = default;
  virtual ~BasePCDDecoder() = default;

  // Disable copy and move operations for abstract base class
  BasePCDDecoder(const BasePCDDecoder &) = delete;
  BasePCDDecoder & operator=(const BasePCDDecoder &) = delete;
  BasePCDDecoder(BasePCDDecoder &&) = delete;
  BasePCDDecoder & operator=(BasePCDDecoder &&) = delete;

  /**
   * @brief Decode serialized message to PointCloud2
   *
   * Pure virtual method that accepts a serialized ROS message.
   * Derived classes deserialize the message internally and convert to PointCloud2.
   *
   * @param serialized_msg The serialized ROS message to decode
   * @return Shared pointer to PointCloud2 message, or nullptr if decoding fails, the point cloud
   * is empty, or the point cloud contains too few points
   */
  virtual sensor_msgs::msg::PointCloud2::SharedPtr decode(
    const rclcpp::SerializedMessage & serialized_msg) = 0;
};

/**
 * @brief PCD decoder class template
 *
 * This abstract class provides a typed interface for decoders that convert
 * specific message types to PointCloud2. It inherits from BasePCDDecoder
 * and handles deserialization internally.
 *
 * @tparam InputScanT The input scan message type to be decoded
 * @tparam OutputPointT The point type used internally for point cloud processing
 * (default: bag_converter::point::PointXYZIT)
 */
template <typename InputScanT, typename OutputPointT = bag_converter::point::PointXYZIT>
class PCDDecoder : public BasePCDDecoder
{
public:
  PCDDecoder() = default;
  ~PCDDecoder() override = default;

  // Disable copy and move operations
  PCDDecoder(const PCDDecoder &) = delete;
  PCDDecoder & operator=(const PCDDecoder &) = delete;
  PCDDecoder(PCDDecoder &&) = delete;
  PCDDecoder & operator=(PCDDecoder &&) = delete;

  /**
   * @brief Decode serialized message to PointCloud2 (implements PCDDecoderBase)
   *
   * Deserializes the message and delegates to the typed decode method. Returns nullptr if the
   * decoded PointCloud2 is empty or contains too few points.
   *
   * @param serialized_msg The serialized ROS message to decode
   * @return Shared pointer to PointCloud2 message, or nullptr if decoding fails, the point cloud
   * is empty, or the point cloud contains too few points
   */
  sensor_msgs::msg::PointCloud2::SharedPtr decode(
    const rclcpp::SerializedMessage & serialized_msg) override
  {
    InputScanT input_msg;
    serializer_.deserialize_message(&serialized_msg, &input_msg);
    auto decoded_msg = decode_typed(input_msg);
    if (!decoded_msg) {
      return nullptr;
    }

    if (decoded_msg->width == 0 || decoded_msg->data.empty()) {
      return nullptr;
    }

    const size_t num_points = decoded_msg->width * decoded_msg->height;
    if (num_points < defaults::min_points_per_scan) {
      return nullptr;
    }

    return decoded_msg;
  }

  /**
   * @brief Decode typed input scan message to PointCloud2
   *
   * Pure virtual method that must be implemented by derived classes.
   * Converts the input scan message of type InputScanT to a PointCloud2 message.
   *
   * @param input The input scan message to decode
   * @return Shared pointer to PointCloud2 message, or nullptr if decoding fails or contains no
   * points
   */
  virtual sensor_msgs::msg::PointCloud2::SharedPtr decode_typed(const InputScanT & input) = 0;

protected:
private:
  rclcpp::Serialization<InputScanT> serializer_;
};

}  // namespace bag_converter::decoder

#endif  // BAG_CONVERTER_BASE_DECODER_HPP
