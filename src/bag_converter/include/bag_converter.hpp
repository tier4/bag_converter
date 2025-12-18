/*
 *  Copyright (C) 2025 TierIV Inc.
 *
 *  License: Apache License
 *
 *  Unified bag converter for Seyond LiDAR topics
 *  Automatically detects and converts both NebulaPackets and SeyondScan messages
 */

#ifndef BAG_CONVERTER__BAG_CONVERTER_HPP
#define BAG_CONVERTER__BAG_CONVERTER_HPP

#include "nebula_decoder.hpp"
#include "point_types.hpp"
#include "seyond_decoder.hpp"

#include <bag_converter/msg/seyond_scan.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_storage/storage_options.hpp>

#include <nebula_msgs/msg/nebula_packets.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <utility>

namespace bag_converter
{

namespace defaults
{
inline constexpr size_t min_points_per_scan = 1000;

// Common configuration defaults
inline constexpr const char * frame_id = "lidar";
inline constexpr double min_range = 0.3;
inline constexpr double max_range = 200.0;
inline constexpr bool keep_original_topics = false;

// Seyond-specific defaults
inline constexpr int coordinate_mode = 3;
inline constexpr bool use_reflectance = false;

// Output point type default
inline constexpr const char * point_type = "xyzit";
}  // namespace defaults

/**
 * @brief Configuration for the unified bag converter
 */
struct Config
{
  std::string input_bag_path;
  std::string output_bag_path;

  // Common configuration
  std::string frame_id = defaults::frame_id;
  double min_range = defaults::min_range;
  double max_range = defaults::max_range;
  bool keep_original_topics = defaults::keep_original_topics;

  // Seyond-specific configuration
  int coordinate_mode = defaults::coordinate_mode;
  bool use_reflectance = defaults::use_reflectance;

  // Output point type: "xyzit" or "xyzi"
  std::string point_type = defaults::point_type;
};

/**
 * @brief Extract sensor info (frame_id and sensor_model) from topic name
 * @param topic_name The topic name to extract info from
 * @param suffix The suffix to look for (e.g., "/nebula_packets" or "/seyond_packets")
 * @param default_frame_id Default frame_id if extraction fails
 * @return Pair of (frame_id, sensor_model)
 */
std::pair<std::string, std::string> extract_sensor_info(
  const std::string & topic_name, const std::string & suffix, const std::string & default_frame_id);

/**
 * @brief Generate output topic name from input topic name
 * @param input_topic The input topic name
 * @param input_suffix The suffix to replace (e.g., "/nebula_packets")
 * @param output_suffix The replacement suffix (e.g., "/nebula_points")
 * @return The output topic name
 */
std::string generate_output_topic(
  const std::string & input_topic, const std::string & input_suffix,
  const std::string & output_suffix);

/**
 * @brief Print conversion summary
 * @param conversion_counts Conversion counts per topic (input -> (output, count))
 */
void print_summary(const std::map<std::string, std::pair<std::string, size_t>> & conversion_counts);

/**
 * @brief Print usage information
 * @param program_name The program name (argv[0])
 */
void print_usage(const char * program_name);

/**
 * @brief Parse command line arguments into Config
 * @param argc Argument count
 * @param argv Argument values
 * @param config Output configuration
 * @return True if parsing successful, false if help was requested or error
 */
bool parse_arguments(int argc, char ** argv, Config & config);

/**
 * @brief Run the bag conversion process
 * @param config The converter configuration
 * @return 0 on success, non-zero on failure
 */
int run(const Config & config);

}  // namespace bag_converter

#endif  // BAG_CONVERTER__BAG_CONVERTER_HPP
