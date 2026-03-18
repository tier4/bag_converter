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

#include "memory_management.hpp"
#include "nebula_decoder.hpp"
#include "point_types.hpp"
#include "seyond_decoder.hpp"
#include "timescale.hpp"

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
#include <filesystem>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace bag_converter
{

namespace defaults
{
// Common configuration defaults
inline constexpr double min_range = 0.1;
inline constexpr double max_range = 250.0;
inline constexpr bool keep_original_topics = false;

// Progress logging interval
inline constexpr size_t progress_log_interval = 1000;

// Minimum valid header.stamp epoch (2000-01-01T00:00:00Z = 946684800 seconds)
inline constexpr int64_t header_stamp_min_epoch_sec = 946'684'800;

}  // namespace defaults

/**
 * @brief Set of ROS 2 message types known to have std_msgs/msg/Header as the first field.
 *
 * For these types, header.stamp is located at a fixed CDR offset (bytes 4-11),
 * enabling reliable extraction without full deserialization.
 */
inline const std::set<std::string> kTypesWithHeader = {
  // sensor_msgs
  "sensor_msgs/msg/CameraInfo",
  "sensor_msgs/msg/CompressedImage",
  "sensor_msgs/msg/Imu",
  "sensor_msgs/msg/Image",
  "sensor_msgs/msg/LaserScan",
  "sensor_msgs/msg/NavSatFix",
  "sensor_msgs/msg/PointCloud2",
  // geometry_msgs
  "geometry_msgs/msg/AccelStamped",
  "geometry_msgs/msg/PoseStamped",
  "geometry_msgs/msg/TwistStamped",
  "geometry_msgs/msg/WrenchStamped",
  // nav_msgs
  "nav_msgs/msg/Odometry",
  // can_msgs
  "can_msgs/msg/Frame",
  // oxts_msgs
  "oxts_msgs/msg/ImuBias",
  "oxts_msgs/msg/LeverArm",
  "oxts_msgs/msg/NavSatRef",
  "oxts_msgs/msg/Ncom",
};

/**
 * @brief Return code for conversion functions
 */
enum class BagConverterResultStatus { kSuccess = 0, kError = 1, kSkipped = 2 };

/**
 * @brief Driver type for decoder creation
 */
enum class DriverType { kNebula, kSeyond };

/**
 * @brief TF transform mode
 */
enum class BagConverterTfMode { kStatic, kDynamic };

/**
 * @brief Statistics for bag conversion per topic
 */
struct BagConverterStats
{
  std::string output_topic;
  size_t decoded_count = 0;
  size_t skipped_count = 0;
};

/**
 * @brief Configuration for the unified bag converter
 */
struct BagConverterConfig
{
  std::string src_bag_path;
  std::string dst_bag_path;

  // Common configuration
  double min_range = defaults::min_range;
  double max_range = defaults::max_range;
  bool keep_original_topics = defaults::keep_original_topics;

  // Output point type
  PointType point_type = PointType::kXYZIT;

  // Timescale correction
  bool timescale_correction = true;
  std::string timescale_correction_ref = "utc";

  // TF2 coordinate transformation
  std::string frame;
  BagConverterTfMode tf_mode = BagConverterTfMode::kStatic;

  // Output compression
  std::string comp_algo = "zstd";
  std::string comp_level = "default";

  // Log time override: use header.stamp as log_time for messages with Header
  bool use_header_stamp_as_log_time = false;

  // Passthrough mode (process all messages even without decodable topics)
  bool passthrough = false;

  // Overwrite existing output files (default: skip)
  bool overwrite = false;

  // Delete source files after successful conversion
  bool delete_sources = false;

  // Batch mode
  bool batch_mode = false;

  // Merge mode (merge + convert pipeline)
  bool merge = false;
  std::vector<std::string> input_dirs;
};

/**
 * @brief Extract frame_id from topic name by taking the parent path segment
 * @param topic_name The topic name (e.g., "/sensing/lidar/front/nebula_packets")
 * @return frame_id string (e.g., "lidar_front")
 */
std::string extract_frame_id(const std::string & topic_name);

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
 * @param conversion_stats Conversion statistics per topic
 */
void print_summary(const std::map<std::string, BagConverterStats> & conversion_stats);

/**
 * @brief Create a temporary YAML config file for mcap compression settings
 * @param comp_algo Compression algorithm: "none", "lz4", or "zstd"
 * @param comp_level Compression level: "fastest", "fast", "default", "slow", or "slowest"
 * @return Path to the temporary YAML file
 */
std::string create_compression_config(
  const std::string & comp_algo, const std::string & comp_level);

/**
 * @brief Print usage information
 */
void print_usage();

/**
 * @brief Print version information
 */
void print_version();

/**
 * @brief Parse command line arguments into BagConverterConfig
 * @param argc Argument count
 * @param argv Argument values
 * @param config Output configuration
 * @return std::nullopt if parsing successful, exit code otherwise (0 for help, 1 for error)
 */
std::optional<int> parse_arguments(int argc, char ** argv, BagConverterConfig & config);

/**
 * @brief Run the bag conversion process
 * @param config The converter configuration
 * @return 0 on success, non-zero on failure
 */
int run(const BagConverterConfig & config);

/**
 * @brief Result tracking for batch conversion
 */
struct BatchResult
{
  size_t success_count = 0;
  size_t fail_count = 0;
  size_t skip_count = 0;
  std::vector<std::string> failed_files;
  std::vector<std::string> skipped_files;
};

/**
 * @brief Run batch conversion on all bag files in a directory
 * @param config The converter configuration (src_bag_path = input dir, dst_bag_path = output dir)
 * @return 0 if all succeeded, 1 if any failed
 */
int run_batch(const BagConverterConfig & config);

/**
 * @brief Find all bag files recursively in the given directory
 * @param input_dir Directory to search
 * @return Sorted vector of bag file paths (.mcap, .db3, .sqlite3)
 */
std::vector<std::filesystem::path> find_bag_files(const std::filesystem::path & input_dir);

/**
 * @brief Finalize output bag: move storage file to dst_bag_path and clean up temp directory
 * @param temp_dir The temporary directory containing the bag output
 * @param dst_path The final destination path for the bag file
 * @return true on success, false on failure
 */
bool finalize_output_bag(
  const std::filesystem::path & temp_dir, const std::filesystem::path & dst_path);

/**
 * @brief Run merge-and-convert pipeline
 * @param config The converter configuration (with merge=true and input_dirs populated)
 * @return 0 on success, non-zero on failure
 */
int run_merge_and_convert(const BagConverterConfig & config);

}  // namespace bag_converter

#endif  // BAG_CONVERTER__BAG_CONVERTER_HPP
