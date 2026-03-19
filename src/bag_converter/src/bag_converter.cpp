/*
 *  Copyright (C) 2025 TierIV Inc.
 *
 *  License: Apache License
 *
 *  Unified bag converter for Seyond LiDAR topics
 *  Automatically detects and converts both NebulaPackets and SeyondScan messages
 */

#include "bag_converter.hpp"

#include "memory_management.hpp"
#include "merge.hpp"
#include "tf_transformer.hpp"

#include <seyond/msg/seyond_scan.hpp>

#include <can_msgs/msg/frame.hpp>
#include <geometry_msgs/msg/accel_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nebula_msgs/msg/nebula_packets.hpp>
#include <oxts_msgs/msg/imu_bias.hpp>
#include <oxts_msgs/msg/lever_arm.hpp>
#include <oxts_msgs/msg/nav_sat_ref.hpp>
#include <oxts_msgs/msg/ncom.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <algorithm>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <queue>
#include <set>
#include <vector>

namespace fs = std::filesystem;

static const rclcpp::Logger g_logger = rclcpp::get_logger("bag_converter");

namespace bag_converter
{

/**
 * @brief Extract header.stamp by deserializing the message.
 *
 * For types in kTypesWithHeader, deserializes to the concrete message type and
 * returns header.stamp. Returns std::nullopt if the type is unsupported or
 * deserialization fails.
 */
static std::optional<rclcpp::Time> extract_header_stamp(
  const rcutils_uint8_array_t & serialized_data, const std::string & topic_type)
{
  if (g_types_with_header.find(topic_type) == g_types_with_header.end()) {
    return std::nullopt;
  }
  try {
    rclcpp::SerializedMessage serialized_msg(serialized_data);
    // sensor_msgs
    if (topic_type == "sensor_msgs/msg/CameraInfo") {
      sensor_msgs::msg::CameraInfo msg;
      rclcpp::Serialization<sensor_msgs::msg::CameraInfo> ser;
      ser.deserialize_message(&serialized_msg, &msg);
      return rclcpp::Time(msg.header.stamp);
    }
    if (topic_type == "sensor_msgs/msg/CompressedImage") {
      sensor_msgs::msg::CompressedImage msg;
      rclcpp::Serialization<sensor_msgs::msg::CompressedImage> ser;
      ser.deserialize_message(&serialized_msg, &msg);
      return rclcpp::Time(msg.header.stamp);
    }
    if (topic_type == "sensor_msgs/msg/Imu") {
      sensor_msgs::msg::Imu msg;
      rclcpp::Serialization<sensor_msgs::msg::Imu> ser;
      ser.deserialize_message(&serialized_msg, &msg);
      return rclcpp::Time(msg.header.stamp);
    }
    if (topic_type == "sensor_msgs/msg/Image") {
      sensor_msgs::msg::Image msg;
      rclcpp::Serialization<sensor_msgs::msg::Image> ser;
      ser.deserialize_message(&serialized_msg, &msg);
      return rclcpp::Time(msg.header.stamp);
    }
    if (topic_type == "sensor_msgs/msg/LaserScan") {
      sensor_msgs::msg::LaserScan msg;
      rclcpp::Serialization<sensor_msgs::msg::LaserScan> ser;
      ser.deserialize_message(&serialized_msg, &msg);
      return rclcpp::Time(msg.header.stamp);
    }
    if (topic_type == "sensor_msgs/msg/NavSatFix") {
      sensor_msgs::msg::NavSatFix msg;
      rclcpp::Serialization<sensor_msgs::msg::NavSatFix> ser;
      ser.deserialize_message(&serialized_msg, &msg);
      return rclcpp::Time(msg.header.stamp);
    }
    if (topic_type == "sensor_msgs/msg/PointCloud2") {
      sensor_msgs::msg::PointCloud2 msg;
      rclcpp::Serialization<sensor_msgs::msg::PointCloud2> ser;
      ser.deserialize_message(&serialized_msg, &msg);
      return rclcpp::Time(msg.header.stamp);
    }
    // geometry_msgs
    if (topic_type == "geometry_msgs/msg/AccelStamped") {
      geometry_msgs::msg::AccelStamped msg;
      rclcpp::Serialization<geometry_msgs::msg::AccelStamped> ser;
      ser.deserialize_message(&serialized_msg, &msg);
      return rclcpp::Time(msg.header.stamp);
    }
    if (topic_type == "geometry_msgs/msg/PoseStamped") {
      geometry_msgs::msg::PoseStamped msg;
      rclcpp::Serialization<geometry_msgs::msg::PoseStamped> ser;
      ser.deserialize_message(&serialized_msg, &msg);
      return rclcpp::Time(msg.header.stamp);
    }
    if (topic_type == "geometry_msgs/msg/TwistStamped") {
      geometry_msgs::msg::TwistStamped msg;
      rclcpp::Serialization<geometry_msgs::msg::TwistStamped> ser;
      ser.deserialize_message(&serialized_msg, &msg);
      return rclcpp::Time(msg.header.stamp);
    }
    if (topic_type == "geometry_msgs/msg/WrenchStamped") {
      geometry_msgs::msg::WrenchStamped msg;
      rclcpp::Serialization<geometry_msgs::msg::WrenchStamped> ser;
      ser.deserialize_message(&serialized_msg, &msg);
      return rclcpp::Time(msg.header.stamp);
    }
    // nav_msgs
    if (topic_type == "nav_msgs/msg/Odometry") {
      nav_msgs::msg::Odometry msg;
      rclcpp::Serialization<nav_msgs::msg::Odometry> ser;
      ser.deserialize_message(&serialized_msg, &msg);
      return rclcpp::Time(msg.header.stamp);
    }
    // can_msgs
    if (topic_type == "can_msgs/msg/Frame") {
      can_msgs::msg::Frame msg;
      rclcpp::Serialization<can_msgs::msg::Frame> ser;
      ser.deserialize_message(&serialized_msg, &msg);
      return rclcpp::Time(msg.header.stamp);
    }
    // oxts_msgs
    if (topic_type == "oxts_msgs/msg/ImuBias") {
      oxts_msgs::msg::ImuBias msg;
      rclcpp::Serialization<oxts_msgs::msg::ImuBias> ser;
      ser.deserialize_message(&serialized_msg, &msg);
      return rclcpp::Time(msg.header.stamp);
    }
    if (topic_type == "oxts_msgs/msg/LeverArm") {
      oxts_msgs::msg::LeverArm msg;
      rclcpp::Serialization<oxts_msgs::msg::LeverArm> ser;
      ser.deserialize_message(&serialized_msg, &msg);
      return rclcpp::Time(msg.header.stamp);
    }
    if (topic_type == "oxts_msgs/msg/NavSatRef") {
      oxts_msgs::msg::NavSatRef msg;
      rclcpp::Serialization<oxts_msgs::msg::NavSatRef> ser;
      ser.deserialize_message(&serialized_msg, &msg);
      return rclcpp::Time(msg.header.stamp);
    }
    if (topic_type == "oxts_msgs/msg/Ncom") {
      oxts_msgs::msg::Ncom msg;
      rclcpp::Serialization<oxts_msgs::msg::Ncom> ser;
      ser.deserialize_message(&serialized_msg, &msg);
      return rclcpp::Time(msg.header.stamp);
    }
    // LiDAR packet types
    if (topic_type == "nebula_msgs/msg/NebulaPackets") {
      nebula_msgs::msg::NebulaPackets msg;
      rclcpp::Serialization<nebula_msgs::msg::NebulaPackets> ser;
      ser.deserialize_message(&serialized_msg, &msg);
      return rclcpp::Time(msg.header.stamp);
    }
    if (topic_type == "seyond/msg/SeyondScan") {
      seyond::msg::SeyondScan msg;
      rclcpp::Serialization<seyond::msg::SeyondScan> ser;
      ser.deserialize_message(&serialized_msg, &msg);
      return rclcpp::Time(msg.header.stamp);
    }
  } catch (...) {
    return std::nullopt;
  }
  return std::nullopt;
}

/**
 * @brief Override log_time with header.stamp for a pass-through message.
 *
 * Deserializes the message (for types in kTypesWithHeader), reads header.stamp,
 * applies optional timescale correction, then validates stamp >=
 * header_stamp_min_epoch_sec; if invalid, keeps original log_time.
 */
static void override_log_time_from_header(
  std::shared_ptr<rosbag2_storage::SerializedBagMessage> & bag_msg, const std::string & topic_type,
  const BagConverterConfig & config)
{
  auto stamp = extract_header_stamp(*bag_msg->serialized_data, topic_type);
  if (!stamp) {
    return;
  }
  uint64_t stamp_ns = stamp->nanoseconds();
  if (config.timescale_correction) {
    stamp_ns = timescale::correct_timescale(
      stamp_ns, static_cast<uint64_t>(bag_msg->time_stamp), config.timescale_correction_ref);
  }

  const int64_t stamp_sec = static_cast<int64_t>(stamp_ns) / 1'000'000'000;
  if (stamp_sec < defaults::header_stamp_min_epoch_sec) {
    RCLCPP_ERROR(
      g_logger,
      "Header stamp below minimum: stamp=%ld < minimum=%ld for type '%s'. "
      "Keeping original log_time.",
      stamp_sec, defaults::header_stamp_min_epoch_sec, topic_type.c_str());
    return;  // invalid stamp, keep original log_time
  }

  bag_msg->time_stamp = static_cast<rcutils_time_point_value_t>(stamp_ns);
}

/// Extract frame_id from topic name by taking the parent path segment.
/// e.g. "/sensing/lidar/front/nebula_packets" -> "lidar_front"
std::string extract_frame_id(const std::string & topic_name)
{
  std::filesystem::path topic(topic_name);
  auto parent = topic.parent_path().filename().string();
  return parent.empty() ? "seyond" : "lidar_" + parent;
}

std::string generate_output_topic(
  const std::string & input_topic, const std::string & input_suffix,
  const std::string & output_suffix)
{
  size_t pos = input_topic.find(input_suffix);
  if (pos == std::string::npos) {
    return input_topic + output_suffix;
  }
  return input_topic.substr(0, pos) + output_suffix;
}

void print_summary(const std::map<std::string, BagConverterStats> & conversion_stats)
{
  RCLCPP_INFO(g_logger, "========== Conversion Summary ==========");

  for (const auto & [input_topic, stats] : conversion_stats) {
    RCLCPP_INFO(g_logger, "  [%s]", input_topic.c_str());
    RCLCPP_INFO(g_logger, "      Destination: %s", stats.output_topic.c_str());
    RCLCPP_INFO(g_logger, "      Decoded: %zu messages", stats.decoded_count);
    RCLCPP_INFO(g_logger, "      Skipped: %zu messages", stats.skipped_count);
  }

  RCLCPP_INFO(g_logger, "========================================");
}

static std::string capitalize(const std::string & s)
{
  std::string result = s;
  if (!result.empty()) {
    result[0] = static_cast<char>(std::toupper(result[0]));
  }
  return result;
}

std::string create_compression_config(const std::string & comp_algo, const std::string & comp_level)
{
  // Map CLI values to YAML values expected by rosbag2_storage_mcap
  static const std::map<std::string, std::string> algo_map = {
    {"none", "None"}, {"lz4", "Lz4"}, {"zstd", "Zstd"}};

  fs::path config_path = fs::temp_directory_path() / "bag_converter_compression.yaml";
  std::ofstream ofs(config_path);
  ofs << "compression: " << algo_map.at(comp_algo) << "\n";
  ofs << "compressionLevel: " << capitalize(comp_level) << "\n";
  return config_path.string();
}

void print_usage()
{
  std::cout
    << "Usage: bag_converter <input_bag> <output_bag> [options]\n"
    << "       bag_converter <input_dir> <output_dir> [options]\n"
    << "       bag_converter <input_dir_0> [input_dir_1 ...] <output_dir> --merge [options]\n"
    << "\nBag converter for Seyond LiDAR topics.\n"
    << "Automatically detects and converts both NebulaPackets and SeyondScan messages.\n"
    << "\nSupported input formats:\n"
    << "  - nebula_msgs/msg/NebulaPackets (topics containing '/nebula_packets')\n"
    << "  - seyond/msg/SeyondScan (topics containing '/seyond_packets')\n"
    << "\nOutput format:\n"
    << "  - sensor_msgs/msg/PointCloud2 (point type specified by --point-type)\n"
    << "\nTimescale correction:\n"
    << "  Detects the timescale of the sensor timestamp (from LiDAR) by comparing it with\n"
    << "  the rosbag recording time, and corrects it to match the recording timescale.\n"
    << "  Supported timescales: UTC, TAI (+37s from UTC), GPS (+18s from UTC).\n"
    << "  Use --timescale-correction-ref to specify the rosbag recording timescale.\n"
    << "\nIf <input> is a directory, all bag files (.mcap, .db3, .sqlite3) in it are\n"
    << "converted. The directory structure is mirrored in the output directory.\n"
    << "\nMerge mode (--merge):\n"
    << "  When --merge is specified, bag files from one or more input directories are\n"
    << "  first merged (by naming pattern), then converted. The last positional argument\n"
    << "  is the output directory; all preceding positional arguments are input directories.\n"
    << "  Input files must follow the naming pattern:\n"
    << "    <sensing_system_id>_<module_id>_<rest>.(mcap|db3|sqlite3)\n"
    << "  Files with the same sensing_system_id and rest are merged together.\n"
    << "\nOptions:\n"
    << "  --keep-original           Keep original packet topics in output bag\n"
    << "  --min-range <value>       Minimum range in meters (default: 0.1)\n"
    << "  --max-range <value>       Maximum range in meters (default: 250.0)\n"
    << "  --point-type <type>       Output point type: xyzit, xyzi, or en_xyzit (default: xyzit)\n"
    << "  --timescale-correction <on|off>  Enable/disable timescale correction (default: on)\n"
    << "  --timescale-correction-ref <utc|tai|gps>  Rosbag recording timescale (default: utc)\n"
    << "  --base-frame <frame>      Transform PointCloud2 to specified TF frame\n"
    << "                            (requires tf2_msgs/msg/TFMessage topics in the bag)\n"
    << "  --tf-mode <static|dynamic>  TF mode (default: static)\n"
    << "                            static:  uses first TF message(s), fixed transform\n"
    << "                            dynamic: pre-loads all TF, time-dependent lookup\n"
    << "                            In both modes, TF data is pre-loaded before processing,\n"
    << "                            so transforms are available even if TF messages appear\n"
    << "                            after point cloud messages in the bag.\n"
    << "  --merge                   Merge bag files from distributed log modules, then\n"
    << "                            convert. Accepts multiple input directories.\n"
    << "  --use-header-stamp-as-log-time\n"
    << "                            Override mcap log_time with header.stamp for messages\n"
    << "                            with a known std_msgs/msg/Header. Applies timescale\n"
    << "                            correction before validation. Invalid stamps (before\n"
    << "                            year 2000) are left unchanged.\n"
    << "  --passthrough             Process all messages even without decodable topics\n"
    << "  --overwrite               Overwrite existing output files (default: skip)\n"
    << "  --comp-algo <none|lz4|zstd>  Output compression algorithm (default: zstd)\n"
    << "  --comp-level <fastest|fast|default|slow|slowest>\n"
    << "                            Output compression level (default: default)\n"
    << "  --delete                  Delete source bag files after successful processing.\n"
    << "                            In merge mode, deletes the original input bag files\n"
    << "                            after each group is successfully merged and converted.\n"
    << "  -h, --help                Show this help message\n"
    << "  -v, --version             Show version information\n";
}

void print_version()
{
  std::cout << BAG_CONVERTER_VERSION << std::endl;
}

/**
 * @brief Check if an option takes a value argument
 */
static bool option_takes_value(const std::string & arg)
{
  return arg == "--min-range" || arg == "--max-range" || arg == "--point-type" ||
         arg == "--timescale-correction" || arg == "--timescale-correction-ref" ||
         arg == "--base-frame" || arg == "--tf-mode" || arg == "--comp-algo" ||
         arg == "--comp-level";
}

std::optional<int> parse_arguments(int argc, char ** argv, BagConverterConfig & config)
{
  // Pre-scan for --help, --version, --merge, --delete
  for (int i = 1; i < argc; i++) {
    std::string arg = argv[i];
    if (arg == "--help" || arg == "-h") {
      print_usage();
      return 0;
    }
    if (arg == "--version" || arg == "-v") {
      print_version();
      return 0;
    }
    if (arg == "--merge") {
      config.merge = true;
    }
    if (arg == "--delete") {
      config.delete_sources = true;
    }
  }

  // Collect positional arguments (non-option args)
  std::vector<std::string> positional;
  for (int i = 1; i < argc; i++) {
    std::string arg = argv[i];
    if (arg[0] == '-') {
      // Skip the value of options that take one
      if (option_takes_value(arg) && i + 1 < argc) {
        i++;
      }
      continue;
    }
    positional.push_back(arg);
  }

  // Determine input/output paths based on mode
  if (config.merge) {
    if (positional.size() < 2) {
      std::cerr
        << "Error: --merge requires at least one input directory and one output directory.\n";
      print_usage();
      return 1;
    }
    config.dst_bag_path = positional.back();
    config.input_dirs.assign(positional.begin(), positional.end() - 1);
    config.batch_mode = true;
  } else {
    if (positional.size() < 2) {
      print_usage();
      return 1;
    }
    config.src_bag_path = positional[0];
    config.dst_bag_path = positional[1];
    if (fs::is_directory(config.src_bag_path)) {
      config.batch_mode = true;
    }
  }

  // Parse option flags
  for (int i = 1; i < argc; i++) {
    std::string arg = argv[i];
    if (arg[0] != '-') {
      continue;  // Skip positional args
    }
    if (arg == "--keep-original") {
      config.keep_original_topics = true;
    } else if (arg == "--min-range" && i + 1 < argc) {
      config.min_range = std::stod(argv[++i]);
    } else if (arg == "--max-range" && i + 1 < argc) {
      config.max_range = std::stod(argv[++i]);
    } else if (arg == "--point-type" && i + 1 < argc) {
      std::string point_type_str = argv[++i];
      if (point_type_str == "xyzit") {
        config.point_type = PointType::kXYZIT;
      } else if (point_type_str == "xyzi") {
        config.point_type = PointType::kXYZI;
      } else if (point_type_str == "en_xyzit") {
        config.point_type = PointType::kEnXYZIT;
      } else {
        std::cerr << "Error: Invalid point type '" << point_type_str
                  << "'. Must be 'xyzit', 'xyzi', or 'en_xyzit'.\n";
        return 1;
      }
    } else if (arg == "--timescale-correction" && i + 1 < argc) {
      std::string value = argv[++i];
      if (value == "on") {
        config.timescale_correction = true;
      } else if (value == "off") {
        config.timescale_correction = false;
      } else {
        std::cerr << "Error: Invalid value for --timescale-correction '" << value
                  << "'. Must be 'on' or 'off'.\n";
        return 1;
      }
    } else if (arg == "--timescale-correction-ref" && i + 1 < argc) {
      std::string value = argv[++i];
      if (value == "utc" || value == "tai" || value == "gps") {
        config.timescale_correction_ref = value;
      } else {
        std::cerr << "Error: Invalid value for --timescale-correction-ref '" << value
                  << "'. Must be 'utc', 'tai', or 'gps'.\n";
        return 1;
      }
    } else if (arg == "--base-frame" && i + 1 < argc) {
      config.frame = argv[++i];
    } else if (arg == "--tf-mode" && i + 1 < argc) {
      std::string value = argv[++i];
      if (value == "static") {
        config.tf_mode = BagConverterTfMode::kStatic;
      } else if (value == "dynamic") {
        config.tf_mode = BagConverterTfMode::kDynamic;
      } else {
        std::cerr << "Error: Invalid value for --tf-mode '" << value
                  << "'. Must be 'static' or 'dynamic'.\n";
        return 1;
      }
    } else if (arg == "--comp-algo" && i + 1 < argc) {
      std::string value = argv[++i];
      if (value == "none" || value == "lz4" || value == "zstd") {
        config.comp_algo = value;
      } else {
        std::cerr << "Error: Invalid value for --comp-algo '" << value
                  << "'. Must be 'none', 'lz4', or 'zstd'.\n";
        return 1;
      }
    } else if (arg == "--comp-level" && i + 1 < argc) {
      std::string value = argv[++i];
      if (
        value == "fastest" || value == "fast" || value == "default" || value == "slow" ||
        value == "slowest") {
        config.comp_level = value;
      } else {
        std::cerr << "Error: Invalid value for --comp-level '" << value
                  << "'. Must be 'fastest', 'fast', 'default', 'slow', or 'slowest'.\n";
        return 1;
      }
    } else if (arg == "--use-header-stamp-as-log-time") {
      config.use_header_stamp_as_log_time = true;
    } else if (arg == "--passthrough") {
      config.passthrough = true;
    } else if (arg == "--overwrite") {
      config.overwrite = true;
    } else if (arg == "--merge" || arg == "--delete") {
      // Already processed in pre-scan
    } else if (arg == "--help" || arg == "-h" || arg == "--version" || arg == "-v") {
      // Already handled
    } else {
      std::cerr << "Error: Unknown option '" << arg << "'.\n";
      print_usage();
      return 1;
    }
  }

  return std::nullopt;
}

/**
 * @brief Finalize output bag: move storage file to dst_bag_path and clean up temp directory
 * @return true on success, false on failure
 */
bool finalize_output_bag(const fs::path & temp_dir, const fs::path & dst_path)
{
  std::error_code ec;

  // Find the storage file in temp directory (any file that is not metadata.yaml)
  fs::path storage_file;
  for (const auto & entry : fs::directory_iterator(temp_dir, ec)) {
    if (entry.path().filename() != "metadata.yaml") {
      storage_file = entry.path();
      break;
    }
  }

  if (storage_file.empty()) {
    RCLCPP_ERROR(g_logger, "No storage file found in temp directory");
    return false;
  }

  // Move storage file to destination
  fs::rename(storage_file, dst_path, ec);
  if (ec) {
    RCLCPP_ERROR(g_logger, "Failed to move bag file: %s", ec.message().c_str());
    return false;
  }

  // Remove temp directory
  fs::remove_all(temp_dir, ec);
  return true;
}

/**
 * @brief Create a decoder for the given topic based on driver type
 * @param driver_type The driver type (DriverType::kNebula or DriverType::kSeyond)
 * @param topic_name The topic name
 * @param config The converter configuration
 * @param conversion_stats Map to track conversion counts
 * @return Unique pointer to the created decoder
 */
template <typename PointT>
std::unique_ptr<decoder::BasePCDDecoder> create_decoder(
  DriverType driver_type, const std::string & topic_name, const BagConverterConfig & config,
  std::map<std::string, BagConverterStats> & conversion_stats)
{
  switch (driver_type) {
    case DriverType::kNebula: {
      decoder::nebula::NebulaPCDDecoderConfig decoder_config;
      decoder_config.min_range = config.min_range;
      decoder_config.max_range = config.max_range;
      decoder_config.frame_id = extract_frame_id(topic_name);

      std::string output_topic =
        generate_output_topic(topic_name, "/nebula_packets", "/nebula_points");
      conversion_stats[topic_name] = {output_topic, 0, 0};

      RCLCPP_INFO(
        g_logger, "Found NebulaPackets topic: %s -> %s (frame_id: %s)", topic_name.c_str(),
        output_topic.c_str(), decoder_config.frame_id.c_str());

      return std::make_unique<decoder::nebula::NebulaPCDDecoder<PointT>>(decoder_config);
    }

    case DriverType::kSeyond: {
      decoder::seyond::SeyondPCDDecoderConfig decoder_config;
      decoder_config.min_range = config.min_range;
      decoder_config.max_range = config.max_range;

      std::string output_topic =
        generate_output_topic(topic_name, "/seyond_packets", "/seyond_points");
      conversion_stats[topic_name] = {output_topic, 0, 0};

      RCLCPP_INFO(
        g_logger, "Found SeyondScan topic: %s -> %s", topic_name.c_str(), output_topic.c_str());

      return std::make_unique<decoder::seyond::SeyondPCDDecoder<PointT>>(decoder_config);
    }
  }

  // This should never be reached due to exhaustive switch
  throw std::logic_error("Unhandled DriverType in create_decoder");
}

enum class ProcessMessageResult { NotDecodable, Decoded, Skipped, Error };

/**
 * Process one message for a LiDAR packet topic (NebulaPackets or SeyondScan).
 * Creates decoder on first use, decodes, applies timescale/transform, writes PointCloud2.
 * @return NotDecodable if topic_type is not a decodable type; caller should write raw message.
 */
template <typename PointT>
ProcessMessageResult process_lidar_message(
  const std::string & topic_name, const std::string & topic_type,
  const rclcpp::SerializedMessage & serialized_msg, const rclcpp::Time & log_time,
  const BagConverterConfig & config,
  std::map<std::string, std::unique_ptr<decoder::BasePCDDecoder>> & decoders,
  std::map<std::string, BagConverterStats> & conversion_stats,
  tf_transformer::TfTransformer * transformer, rosbag2_cpp::Writer & writer,
  rclcpp::Serialization<sensor_msgs::msg::PointCloud2> & pc2_serializer)
{
  const bool is_nebula = (topic_type == "nebula_msgs/msg/NebulaPackets");
  const bool is_seyond = (topic_type == "seyond/msg/SeyondScan");
  if (!is_nebula && !is_seyond) {
    return ProcessMessageResult::NotDecodable;
  }

  if (decoders.find(topic_name) == decoders.end()) {
    const DriverType driver_type = is_nebula ? DriverType::kNebula : DriverType::kSeyond;
    decoders[topic_name] =
      create_decoder<PointT>(driver_type, topic_name, config, conversion_stats);
  }

  auto & dec = decoders[topic_name];
  auto pcd_msg = dec->decode(serialized_msg);

  if (!pcd_msg) {
    return ProcessMessageResult::Skipped;
  }

  if (config.timescale_correction) {
    const std::uint64_t sensor_time_ns =
      static_cast<std::uint64_t>(pcd_msg->header.stamp.sec) * 1000000000 +
      static_cast<std::uint64_t>(pcd_msg->header.stamp.nanosec);
    const std::uint64_t sensor_time_ns_corrected = timescale::correct_timescale(
      sensor_time_ns, log_time.nanoseconds(), config.timescale_correction_ref);
    if (sensor_time_ns_corrected != sensor_time_ns) {
      pcd_msg->header.stamp.sec = static_cast<builtin_interfaces::msg::Time::_sec_type>(
        sensor_time_ns_corrected / 1000000000);
      pcd_msg->header.stamp.nanosec = static_cast<builtin_interfaces::msg::Time::_nanosec_type>(
        sensor_time_ns_corrected % 1000000000);
    }
  }

  if (transformer) {
    if (!transformer->transform(*pcd_msg, config.frame)) {
      RCLCPP_ERROR(g_logger, "TF transform failed for frame: %s", pcd_msg->header.frame_id.c_str());
      return ProcessMessageResult::Error;
    }
  }

  auto pc2_serialized = std::make_shared<rclcpp::SerializedMessage>();
  pc2_serializer.serialize_message(pcd_msg.get(), pc2_serialized.get());

  // Use timescale-corrected header.stamp as log_time when enabled
  const auto write_time =
    config.use_header_stamp_as_log_time ? rclcpp::Time(pcd_msg->header.stamp) : log_time;
  writer.write(
    pc2_serialized, conversion_stats[topic_name].output_topic, "sensor_msgs/msg/PointCloud2",
    write_time);
  conversion_stats[topic_name].decoded_count++;
  return ProcessMessageResult::Decoded;
}

template <typename PointT>
BagConverterResultStatus run_impl(const BagConverterConfig & config)
{
  if (!fs::exists(config.src_bag_path)) {
    RCLCPP_ERROR(g_logger, "Input bag file does not exist: %s", config.src_bag_path.c_str());
    return BagConverterResultStatus::kError;
  }

  const fs::path dst_path(config.dst_bag_path);

  // Check if input and output paths refer to the same file
  if (fs::exists(dst_path) && fs::canonical(config.src_bag_path) == fs::canonical(dst_path)) {
    RCLCPP_ERROR(
      g_logger, "Input and output bag paths must not be the same: %s", config.src_bag_path.c_str());
    return BagConverterResultStatus::kError;
  }

  const fs::path temp_dir = dst_path.string() + "_tmp";

  // Skip if output already exists and --overwrite is not set
  if (fs::exists(dst_path) && !config.overwrite) {
    RCLCPP_INFO(
      g_logger, "Output file already exists, skipping: %s (use --overwrite to replace)",
      dst_path.c_str());
    return BagConverterResultStatus::kSkipped;
  }

  // Create parent directory if needed
  if (dst_path.has_parent_path() && !fs::exists(dst_path.parent_path())) {
    RCLCPP_INFO(g_logger, "Creating output directory: %s", dst_path.parent_path().c_str());
    fs::create_directories(dst_path.parent_path());
  }

  // Remove existing output file if present
  if (fs::exists(dst_path)) {
    RCLCPP_INFO(g_logger, "Removing existing output file: %s", dst_path.c_str());
    fs::remove(dst_path);
  }

  // Remove existing temp directory if present
  if (fs::exists(temp_dir)) {
    RCLCPP_INFO(g_logger, "Removing existing temp directory: %s", temp_dir.c_str());
    fs::remove_all(temp_dir);
  }

  // Open input bag
  rosbag2_storage::StorageOptions storage_options_in;
  storage_options_in.uri = config.src_bag_path;

  rosbag2_cpp::Reader reader;
  try {
    reader.open(storage_options_in);
    memory_management::fadvise_sequential_access(config.src_bag_path);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(g_logger, "Error opening input bag: %s", e.what());
    return BagConverterResultStatus::kError;
  }

  const auto bag_metadata = reader.get_metadata();

  // Create TF transformer if frame is specified
  std::unique_ptr<tf_transformer::TfTransformer> transformer;
  if (!config.frame.empty()) {
    transformer = std::make_unique<tf_transformer::TfTransformer>(config.tf_mode);
  }

  // Build topic type map from metadata and check for decodable topics
  std::map<std::string, std::string> topic_types;
  bool has_decodable_topics = false;
  bool has_tf_topic = false;
  bool has_tf_messages = false;
  for (const auto & topic_info : bag_metadata.topics_with_message_count) {
    topic_types[topic_info.topic_metadata.name] = topic_info.topic_metadata.type;
    const auto & type = topic_info.topic_metadata.type;
    if (type == "nebula_msgs/msg/NebulaPackets" || type == "seyond/msg/SeyondScan") {
      has_decodable_topics = true;
    }
    if (type == "tf2_msgs/msg/TFMessage") {
      has_tf_topic = true;
      if (topic_info.message_count > 0) {
        has_tf_messages = true;
      }
    }
  }

  if (!has_decodable_topics && !config.passthrough) {
    RCLCPP_WARN(
      g_logger, "Skipping conversion: no decodable topics found in %s",
      config.src_bag_path.c_str());
    return BagConverterResultStatus::kSkipped;
  }

  if (!has_decodable_topics && config.passthrough) {
    RCLCPP_INFO(
      g_logger, "Passthrough mode: no decodable topics, all messages will be passed through");
  }

  if (transformer && !has_tf_topic) {
    RCLCPP_ERROR(
      g_logger, "No TF topics found in %s. Cannot transform to frame '%s'",
      config.src_bag_path.c_str(), config.frame.c_str());
    return BagConverterResultStatus::kError;
  }

  if (transformer && !has_tf_messages) {
    RCLCPP_ERROR(
      g_logger, "TF topics exist but contain no messages in %s", config.src_bag_path.c_str());
    return BagConverterResultStatus::kError;
  }

  // Open output bag (write to temp directory)
  rosbag2_storage::StorageOptions storage_options_out;
  storage_options_out.uri = temp_dir.string();
  storage_options_out.storage_id = bag_metadata.storage_identifier;
  storage_options_out.storage_config_uri =
    create_compression_config(config.comp_algo, config.comp_level);

  rosbag2_cpp::Writer writer;
  try {
    writer.open(storage_options_out);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(g_logger, "Error opening output bag: %s", e.what());
    return BagConverterResultStatus::kError;
  }
  // Create output topics based on input metadata
  std::set<std::string> created_topics;
  for (const auto & topic_info : bag_metadata.topics_with_message_count) {
    const auto & topic_metadata = topic_info.topic_metadata;
    const auto & topic_type = topic_metadata.type;

    bool is_nebula = (topic_type == "nebula_msgs/msg/NebulaPackets");
    bool is_seyond = (topic_type == "seyond/msg/SeyondScan");

    if (is_nebula || is_seyond) {
      // Create point cloud output topic
      std::string output_topic;
      if (is_nebula) {
        output_topic =
          generate_output_topic(topic_metadata.name, "/nebula_packets", "/nebula_points");
      } else {
        output_topic =
          generate_output_topic(topic_metadata.name, "/seyond_packets", "/seyond_points");
      }

      rosbag2_storage::TopicMetadata pc_topic_meta;
      pc_topic_meta.name = output_topic;
      pc_topic_meta.type = "sensor_msgs/msg/PointCloud2";
      pc_topic_meta.serialization_format = "cdr";
      writer.create_topic(pc_topic_meta);
      created_topics.insert(output_topic);

      if (config.keep_original_topics) {
        writer.create_topic(topic_metadata);
        created_topics.insert(topic_metadata.name);
      }
    } else {
      writer.create_topic(topic_metadata);
      created_topics.insert(topic_metadata.name);
    }
  }

  // Pre-load TF data
  if (transformer) {
    const char * tf_mode_str = config.tf_mode == BagConverterTfMode::kStatic ? "static" : "dynamic";
    RCLCPP_INFO(g_logger, "Loading TF data (%s mode)...", tf_mode_str);
    rosbag2_cpp::Reader tf_reader;
    tf_reader.open(storage_options_in);
    memory_management::fadvise_sequential_access(config.src_bag_path);
    while (tf_reader.has_next()) {
      auto msg = tf_reader.read_next();
      auto it = topic_types.find(msg->topic_name);
      if (it != topic_types.end() && it->second == "tf2_msgs/msg/TFMessage") {
        rclcpp::SerializedMessage serialized_tf(*msg->serialized_data);
        transformer->add_transforms(serialized_tf, msg->topic_name);
        if (config.tf_mode == BagConverterTfMode::kStatic && transformer->has_frame(config.frame)) {
          break;
        }
      }
    }
    if (!transformer->has_frame(config.frame)) {
      RCLCPP_ERROR(g_logger, "Frame '%s' does not exist in TF data", config.frame.c_str());
      return BagConverterResultStatus::kError;
    }
    RCLCPP_INFO(g_logger, "TF data loaded");
  }

  // Serializer for output PointCloud2
  rclcpp::Serialization<sensor_msgs::msg::PointCloud2> pc2_serializer;

  // Unified decoder map using polymorphism (type erasure pattern)
  std::map<std::string, std::unique_ptr<decoder::BasePCDDecoder>> decoders;

  // Topic output mapping and conversion statistics
  std::map<std::string, BagConverterStats> conversion_stats;

  size_t message_count = 0;

  RCLCPP_INFO(g_logger, "Processing messages...");

  while (rclcpp::ok() && reader.has_next()) {
    auto bag_msg = reader.read_next();
    message_count++;

    const auto & topic_name = bag_msg->topic_name;
    auto type_it = topic_types.find(topic_name);
    if (type_it == topic_types.end()) {
      RCLCPP_ERROR(
        g_logger, "Unknown topic in bag metadata: \"%s\". Conversion stopped.", topic_name.c_str());
      return BagConverterResultStatus::kError;
    }

    const auto & topic_type = type_it->second;

    bool is_nebula = (topic_type == "nebula_msgs/msg/NebulaPackets");
    bool is_seyond = (topic_type == "seyond/msg/SeyondScan");

    if (!is_nebula && !is_seyond) {
      if (config.use_header_stamp_as_log_time) {
        override_log_time_from_header(bag_msg, topic_type, config);
      }
      writer.write(bag_msg);
      if (message_count % defaults::progress_log_interval == 0) {
        RCLCPP_INFO(g_logger, "Processed %zu messages...", message_count);
      }
      continue;
    }

    if (config.keep_original_topics) {
      if (config.use_header_stamp_as_log_time) {
        override_log_time_from_header(bag_msg, topic_type, config);
      }
      writer.write(bag_msg);
    }
    rclcpp::SerializedMessage serialized_msg(*bag_msg->serialized_data);
    const auto log_time = rclcpp::Time(bag_msg->time_stamp);
    const auto result = process_lidar_message<PointT>(
      topic_name, topic_type, serialized_msg, log_time, config, decoders, conversion_stats,
      transformer.get(), writer, pc2_serializer);
    if (result == ProcessMessageResult::Error) {
      return BagConverterResultStatus::kError;
    }

    if (message_count % defaults::progress_log_interval == 0) {
      RCLCPP_INFO(g_logger, "Processed %zu messages...", message_count);
    }
  }

  if (!rclcpp::ok()) {
    RCLCPP_WARN(g_logger, "Interrupted by user (Ctrl+C), conversion terminated early");
  }

  print_summary(conversion_stats);

  // Close writer and finalize output
  writer.close();
  if (!finalize_output_bag(temp_dir, dst_path)) {
    return BagConverterResultStatus::kError;
  }

  RCLCPP_INFO(g_logger, "Output written to: %s", dst_path.c_str());
  return BagConverterResultStatus::kSuccess;
}

static BagConverterResultStatus run_single(const BagConverterConfig & config)
{
  switch (config.point_type) {
    case PointType::kXYZI:
      return run_impl<point::PointXYZI>(config);
    case PointType::kXYZIT:
      return run_impl<point::PointXYZIT>(config);
    case PointType::kEnXYZIT:
      return run_impl<point::PointEnXYZIT>(config);
  }
  return BagConverterResultStatus::kError;
}

static void log_config(const BagConverterConfig & config)
{
  RCLCPP_INFO(g_logger, "BagConverterConfiguration:");
  RCLCPP_INFO(g_logger, "  Min range: %.1f m", config.min_range);
  RCLCPP_INFO(g_logger, "  Max range: %.1f m", config.max_range);
  RCLCPP_INFO(g_logger, "  Point type: %s", point::point_type_to_string(config.point_type));
  RCLCPP_INFO(g_logger, "  Keep original topics: %s", config.keep_original_topics ? "yes" : "no");
  RCLCPP_INFO(g_logger, "  Timescale correction: %s", config.timescale_correction ? "on" : "off");
  RCLCPP_INFO(g_logger, "  Timescale correction ref: %s", config.timescale_correction_ref.c_str());
  if (!config.frame.empty()) {
    RCLCPP_INFO(g_logger, "  TF frame: %s", config.frame.c_str());
    RCLCPP_INFO(
      g_logger, "  TF mode: %s",
      config.tf_mode == BagConverterTfMode::kStatic ? "static" : "dynamic");
  }
}

int run(const BagConverterConfig & config)
{
  log_config(config);
  auto ret = run_single(config);
  if (ret == BagConverterResultStatus::kError) {
    return 1;
  }

  if (config.delete_sources && ret == BagConverterResultStatus::kSuccess) {
    std::error_code ec;
    fs::remove(config.src_bag_path, ec);
    if (ec) {
      RCLCPP_WARN(
        g_logger, "Failed to delete source file '%s': %s", config.src_bag_path.c_str(),
        ec.message().c_str());
    } else {
      RCLCPP_INFO(g_logger, "Deleted source file: %s", config.src_bag_path.c_str());
    }
  }

  return 0;
}

std::vector<fs::path> find_bag_files(const fs::path & input_dir)
{
  static const std::set<std::string> bag_extensions = {".mcap", ".db3", ".sqlite3"};
  std::vector<fs::path> files;

  try {
    for (const auto & entry : fs::recursive_directory_iterator(input_dir)) {
      if (entry.is_regular_file() && bag_extensions.count(entry.path().extension().string()) > 0) {
        files.push_back(entry.path());
      }
    }
  } catch (const fs::filesystem_error & e) {
    RCLCPP_WARN(
      g_logger, "Error while traversing input directory '%s': %s", input_dir.string().c_str(),
      e.what());
  }

  std::sort(files.begin(), files.end());
  return files;
}

static void print_batch_summary(const BatchResult & result)
{
  const size_t total = result.success_count + result.fail_count + result.skip_count;
  RCLCPP_INFO(g_logger, "========== Batch Summary ==========");
  RCLCPP_INFO(g_logger, "  Total files: %zu", total);
  RCLCPP_INFO(g_logger, "  Success: %zu", result.success_count);
  RCLCPP_INFO(g_logger, "  Failed: %zu", result.fail_count);
  RCLCPP_INFO(g_logger, "  Skipped: %zu", result.skip_count);

  if (!result.failed_files.empty()) {
    RCLCPP_WARN(g_logger, "  Failed files:");
    for (const auto & f : result.failed_files) {
      RCLCPP_WARN(g_logger, "    - %s", f.c_str());
    }
  }

  if (!result.skipped_files.empty()) {
    RCLCPP_WARN(g_logger, "  Skipped files:");
    for (const auto & f : result.skipped_files) {
      RCLCPP_WARN(g_logger, "    - %s", f.c_str());
    }
  }

  RCLCPP_INFO(g_logger, "===================================");
}

int run_batch(const BagConverterConfig & config)
{
  const fs::path input_dir(config.src_bag_path);
  const fs::path output_dir(config.dst_bag_path);

  // Validate input directory
  if (!fs::exists(input_dir) || !fs::is_directory(input_dir)) {
    RCLCPP_ERROR(g_logger, "Input path is not an existing directory: %s", input_dir.c_str());
    return 1;
  }

  // Check input != output
  {
    std::error_code ec;
    const auto input_canonical = fs::canonical(input_dir, ec);
    if (!ec && fs::exists(output_dir)) {
      const auto output_canonical = fs::canonical(output_dir, ec);
      if (!ec && input_canonical == output_canonical) {
        RCLCPP_ERROR(
          g_logger, "Input and output directories must not be the same: %s", input_dir.c_str());
        return 1;
      }
    }
  }

  // Create output directory if needed
  if (!fs::exists(output_dir)) {
    RCLCPP_INFO(g_logger, "Creating output directory: %s", output_dir.c_str());
    std::error_code create_ec;
    fs::create_directories(output_dir, create_ec);
    if (create_ec) {
      RCLCPP_ERROR(
        g_logger, "Failed to create output directory %s: %s", output_dir.c_str(),
        create_ec.message().c_str());
      return 1;
    }
  }

  // Find bag files
  const auto bag_files = find_bag_files(input_dir);
  if (bag_files.empty()) {
    RCLCPP_WARN(g_logger, "No bag files found in: %s", input_dir.c_str());
    return 0;
  }

  RCLCPP_INFO(g_logger, "Found %zu bag file(s) in: %s", bag_files.size(), input_dir.c_str());

  log_config(config);

  BatchResult result;

  for (size_t i = 0; i < bag_files.size(); ++i) {
    if (!rclcpp::ok()) {
      RCLCPP_WARN(g_logger, "Interrupted by user (Ctrl+C), batch processing stopped");
      break;
    }

    const auto & bag_file = bag_files[i];
    const auto relative = fs::relative(bag_file, input_dir);
    const auto output_path = output_dir / relative;

    RCLCPP_INFO(g_logger, "[%zu/%zu] Processing: %s", i + 1, bag_files.size(), relative.c_str());

    // Build per-file config
    BagConverterConfig file_config = config;
    file_config.src_bag_path = bag_file.string();
    file_config.dst_bag_path = output_path.string();

    try {
      auto ret = run_single(file_config);
      if (ret == BagConverterResultStatus::kSkipped) {
        result.skip_count++;
        result.skipped_files.push_back(relative.string());
      } else if (ret == BagConverterResultStatus::kError) {
        RCLCPP_ERROR(g_logger, "  Failed to convert: %s", relative.c_str());
        result.fail_count++;
        result.failed_files.push_back(relative.string());
      } else {
        result.success_count++;
        if (config.delete_sources) {
          std::error_code del_ec;
          fs::remove(bag_file, del_ec);
          if (del_ec) {
            RCLCPP_WARN(
              g_logger, "Failed to delete source file '%s': %s", bag_file.c_str(),
              del_ec.message().c_str());
          } else {
            RCLCPP_INFO(g_logger, "  Deleted source: %s", relative.c_str());
          }
        }
      }
    } catch (const std::exception & e) {
      RCLCPP_ERROR(g_logger, "  Exception converting %s: %s", relative.c_str(), e.what());
      result.fail_count++;
      result.failed_files.push_back(relative.string());
    }
  }

  print_batch_summary(result);

  return result.fail_count > 0 ? 1 : 0;
}

/**
 * @brief Combined merge + convert for a single group (single-pass)
 *
 * K-way merges multiple bag files while simultaneously decoding LiDAR packet
 * topics to PointCloud2. This avoids writing an intermediate merged bag to disk.
 */
template <typename PointT>
static int64_t merge_and_convert_group(
  const std::vector<fs::path> & bag_files, const fs::path & output_path,
  const std::string & storage_identifier, const BagConverterConfig & config)
{
  // 1. Collect topic union
  auto topic_union = merge::collect_topic_union(bag_files);
  if (!topic_union.has_value()) {
    return -1;
  }

  // 2. Build topic type map and detect decodable/TF topics
  std::map<std::string, std::string> topic_types;
  bool has_decodable_topics = false;
  bool has_tf_topic = false;
  for (const auto & [name, meta] : topic_union.value()) {
    topic_types[name] = meta.type;
    if (meta.type == "nebula_msgs/msg/NebulaPackets" || meta.type == "seyond/msg/SeyondScan") {
      has_decodable_topics = true;
    }
    if (meta.type == "tf2_msgs/msg/TFMessage") {
      has_tf_topic = true;
    }
  }

  if (!has_decodable_topics) {
    RCLCPP_WARN(g_logger, "No decodable topics found. Performing merge only (no conversion).");
  }

  // 3. Create TF transformer and pre-load from all input bags
  std::unique_ptr<tf_transformer::TfTransformer> transformer;
  if (!config.frame.empty()) {
    if (!has_tf_topic) {
      RCLCPP_ERROR(
        g_logger, "No TF topics found in input bags. Cannot transform to frame '%s'",
        config.frame.c_str());
      return -1;
    }

    transformer = std::make_unique<tf_transformer::TfTransformer>(config.tf_mode);
    const char * tf_mode_str = config.tf_mode == BagConverterTfMode::kStatic ? "static" : "dynamic";
    RCLCPP_INFO(
      g_logger, "Loading TF data from %zu bags (%s mode)...", bag_files.size(), tf_mode_str);

    for (const auto & bag_path : bag_files) {
      rosbag2_storage::StorageOptions opts;
      opts.uri = bag_path.string();
      rosbag2_cpp::Reader tf_reader;
      tf_reader.open(opts);
      memory_management::fadvise_sequential_access(bag_path.string());
      while (tf_reader.has_next()) {
        auto msg = tf_reader.read_next();
        auto it = topic_types.find(msg->topic_name);
        if (it != topic_types.end() && it->second == "tf2_msgs/msg/TFMessage") {
          rclcpp::SerializedMessage serialized_tf(*msg->serialized_data);
          transformer->add_transforms(serialized_tf, msg->topic_name);
          if (
            config.tf_mode == BagConverterTfMode::kStatic && transformer->has_frame(config.frame)) {
            break;
          }
        }
      }
      if (config.tf_mode == BagConverterTfMode::kStatic && transformer->has_frame(config.frame)) {
        break;
      }
    }

    if (!transformer->has_frame(config.frame)) {
      RCLCPP_ERROR(g_logger, "Frame '%s' does not exist in TF data", config.frame.c_str());
      return -1;
    }
    RCLCPP_INFO(g_logger, "TF data loaded");
  }

  // 4. Remove stale temp directory
  fs::path temp_dir = output_path.string() + "_tmp";
  std::error_code ec;
  if (fs::exists(temp_dir)) {
    RCLCPP_WARN(
      g_logger, "Removing stale temp directory from previous run: %s", temp_dir.string().c_str());
    fs::remove_all(temp_dir, ec);
    if (ec) {
      RCLCPP_ERROR(
        g_logger, "Failed to remove stale temp directory '%s': %s", temp_dir.string().c_str(),
        ec.message().c_str());
      return -1;
    }
  }

  // 5. Open writer
  rosbag2_storage::StorageOptions storage_options_out;
  storage_options_out.uri = temp_dir.string();
  storage_options_out.storage_id = storage_identifier;
  storage_options_out.storage_config_uri =
    create_compression_config(config.comp_algo, config.comp_level);

  rosbag2_cpp::Writer writer;
  writer.open(storage_options_out);

  // 6. Create output topics
  for (const auto & [name, meta] : topic_union.value()) {
    bool is_nebula = (meta.type == "nebula_msgs/msg/NebulaPackets");
    bool is_seyond = (meta.type == "seyond/msg/SeyondScan");

    if (is_nebula || is_seyond) {
      // Create PointCloud2 output topic
      std::string output_topic;
      if (is_nebula) {
        output_topic = generate_output_topic(name, "/nebula_packets", "/nebula_points");
      } else {
        output_topic = generate_output_topic(name, "/seyond_packets", "/seyond_points");
      }

      rosbag2_storage::TopicMetadata pc_meta;
      pc_meta.name = output_topic;
      pc_meta.type = "sensor_msgs/msg/PointCloud2";
      pc_meta.serialization_format = "cdr";
      writer.create_topic(pc_meta);

      if (config.keep_original_topics) {
        writer.create_topic(meta);
      }
    } else {
      writer.create_topic(meta);
    }
  }

  // 7. Open all readers and seed the min-heap
  std::vector<std::unique_ptr<rosbag2_cpp::Reader>> readers;
  std::priority_queue<merge::HeapEntry, std::vector<merge::HeapEntry>, merge::HeapCompare> min_heap;

  for (size_t i = 0; i < bag_files.size(); ++i) {
    auto reader = std::make_unique<rosbag2_cpp::Reader>();
    rosbag2_storage::StorageOptions storage_options_in;
    storage_options_in.uri = bag_files[i].string();
    reader->open(storage_options_in);
    memory_management::fadvise_sequential_access(bag_files[i].string());

    if (reader->has_next()) {
      auto msg = reader->read_next();
      min_heap.push({msg, i});
    }
    readers.push_back(std::move(reader));
  }

  // 8. Decoder map and serializer
  rclcpp::Serialization<sensor_msgs::msg::PointCloud2> pc2_serializer;
  std::map<std::string, std::unique_ptr<decoder::BasePCDDecoder>> decoders;
  std::map<std::string, BagConverterStats> conversion_stats;

  // 9. K-way merge loop with inline conversion
  int64_t message_count = 0;
  while (!min_heap.empty() && rclcpp::ok()) {
    auto entry = min_heap.top();
    min_heap.pop();

    const auto & topic_name = entry.message->topic_name;
    auto type_it = topic_types.find(topic_name);
    if (type_it == topic_types.end()) {
      RCLCPP_ERROR(
        g_logger, "Unknown topic in bag metadata: \"%s\". Conversion stopped.", topic_name.c_str());
      writer.close();
      fs::remove_all(temp_dir, ec);
      return -1;
    }
    const auto & topic_type = type_it->second;

    const bool is_nebula = (topic_type == "nebula_msgs/msg/NebulaPackets");
    const bool is_seyond = (topic_type == "seyond/msg/SeyondScan");

    if (is_nebula || is_seyond) {
      if (config.keep_original_topics) {
        if (config.use_header_stamp_as_log_time) {
          override_log_time_from_header(entry.message, topic_type, config);
        }
        writer.write(entry.message);
      }
      rclcpp::SerializedMessage serialized_msg(*entry.message->serialized_data);
      const auto log_time = rclcpp::Time(entry.message->time_stamp);
      const auto result = process_lidar_message<PointT>(
        topic_name, topic_type, serialized_msg, log_time, config, decoders, conversion_stats,
        transformer.get(), writer, pc2_serializer);
      if (result == ProcessMessageResult::Error) {
        writer.close();
        fs::remove_all(temp_dir, ec);
        return -1;
      }
    } else {
      if (config.use_header_stamp_as_log_time) {
        override_log_time_from_header(entry.message, topic_type, config);
      }
      writer.write(entry.message);
    }

    ++message_count;
    if (message_count % defaults::progress_log_interval == 0) {
      RCLCPP_INFO(g_logger, "  Processed %ld messages...", message_count);
    }
    // Read next message from the same reader
    auto & reader = readers[entry.reader_index];
    if (reader->has_next()) {
      auto next_msg = reader->read_next();
      min_heap.push({next_msg, entry.reader_index});
    }
  }

  if (!rclcpp::ok()) {
    RCLCPP_WARN(g_logger, "Interrupted by user (Ctrl+C), merge+convert terminated early");
  }

  print_summary(conversion_stats);

  // Close writer and finalize
  writer.close();
  if (!finalize_output_bag(temp_dir, output_path)) {
    return -1;
  }

  return message_count;
}

int run_merge_and_convert(const BagConverterConfig & config)
{
  log_config(config);

  merge::MergeConfig merge_config;
  merge_config.input_dirs = config.input_dirs;
  merge_config.output_dir = config.dst_bag_path;
  merge_config.overwrite = config.overwrite;
  merge_config.delete_sources = config.delete_sources;
  merge_config.comp_algo = config.comp_algo;
  merge_config.comp_level = config.comp_level;

  auto processor = [&config](
                     const std::vector<fs::path> & bag_files, const fs::path & output_path,
                     const std::string & storage_identifier) -> int64_t {
    switch (config.point_type) {
      case PointType::kXYZI:
        return merge_and_convert_group<point::PointXYZI>(
          bag_files, output_path, storage_identifier, config);
      case PointType::kXYZIT:
        return merge_and_convert_group<point::PointXYZIT>(
          bag_files, output_path, storage_identifier, config);
      case PointType::kEnXYZIT:
        return merge_and_convert_group<point::PointEnXYZIT>(
          bag_files, output_path, storage_identifier, config);
    }
    return -1;
  };

  return merge::run_merge(merge_config, processor);
}

}  // namespace bag_converter

int main(int argc, char ** argv)
{
  bag_converter::BagConverterConfig config;
  auto parse_result = bag_converter::parse_arguments(argc, argv, config);
  if (parse_result.has_value()) {
    return parse_result.value();
  }

  rclcpp::init(argc, argv);

  int result = 0;
  if (config.merge) {
    result = bag_converter::run_merge_and_convert(config);
  } else if (config.batch_mode) {
    result = bag_converter::run_batch(config);
  } else {
    result = bag_converter::run(config);
  }

  rclcpp::shutdown();
  return result;
}
