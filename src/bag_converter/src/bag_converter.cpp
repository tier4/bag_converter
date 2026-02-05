/*
 *  Copyright (C) 2025 TierIV Inc.
 *
 *  License: Apache License
 *
 *  Unified bag converter for Seyond LiDAR topics
 *  Automatically detects and converts both NebulaPackets and SeyondScan messages
 */

#include "bag_converter.hpp"

#include "tf_transformer.hpp"

#include <algorithm>
#include <filesystem>
#include <iostream>
#include <set>
#include <vector>

namespace fs = std::filesystem;

static const rclcpp::Logger g_logger = rclcpp::get_logger("bag_converter");

namespace bag_converter
{

std::pair<std::string, std::string> extract_sensor_info(
  const std::string & topic_name, const std::string & suffix, const std::string & default_frame_id)
{
  std::string frame_id = default_frame_id;
  std::string sensor_model;

  size_t last_slash = topic_name.rfind(suffix);
  if (last_slash != std::string::npos && last_slash > 0) {
    size_t second_last_slash = topic_name.rfind('/', last_slash - 1);
    if (second_last_slash != std::string::npos) {
      std::string sensor_pos =
        topic_name.substr(second_last_slash + 1, last_slash - second_last_slash - 1);
      frame_id = "lidar_" + sensor_pos;

      if (sensor_pos == "front" || sensor_pos == "rear") {
        sensor_model = "Falcon";
      } else if (sensor_pos == "left" || sensor_pos == "right") {
        sensor_model = "Robin_W";
      }
    }
  }

  return {frame_id, sensor_model};
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

void print_usage(const char * program_name)
{
  std::cout
    << "Usage: " << program_name << " <input_bag> <output_bag> [options]\n"
    << "       " << program_name << " <input_dir> <output_dir> [options]\n"
    << "\nUnified bag converter for Seyond LiDAR topics.\n"
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
    << "\nOptions:\n"
    << "  --keep-original           Keep original packet topics in output bag\n"
    << "  --min-range <value>       Minimum range in meters (default: 0.3)\n"
    << "  --max-range <value>       Maximum range in meters (default: 200.0)\n"
    << "  --point-type <type>       Output point type: xyzit or xyzi (default: xyzit)\n"
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
    << "  -h, --help                Show this help message\n"
    << "  -v, --version             Show version information\n";
}

void print_version()
{
  std::cout << BAG_CONVERTER_VERSION << std::endl;
}

std::optional<int> parse_arguments(int argc, char ** argv, BagConverterConfig & config)
{
  for (int i = 1; i < argc; i++) {
    std::string arg = argv[i];
    if (arg == "--help" || arg == "-h") {
      print_usage(argv[0]);
      return 0;  // Help requested, exit with success
    }
    if (arg == "--version" || arg == "-v") {
      print_version();
      return 0;  // Version requested, exit with success
    }
  }

  if (argc < 3) {
    print_usage(argv[0]);
    return 1;  // Error: missing arguments
  }

  config.src_bag_path = argv[1];
  config.dst_bag_path = argv[2];

  // Auto-detect batch mode if input path is a directory
  if (fs::is_directory(config.src_bag_path)) {
    config.batch_mode = true;
  }

  for (int i = 3; i < argc; i++) {
    std::string arg = argv[i];
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
      } else {
        std::cerr << "Error: Invalid point type '" << point_type_str
                  << "'. Must be 'xyzit' or 'xyzi'.\n";
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
    } else {
      std::cerr << "Error: Unknown option '" << arg << "'.\n";
      print_usage(argv[0]);
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

      auto [frame_id, sensor_model] =
        extract_sensor_info(topic_name, "/nebula_packets", config.frame_id);
      decoder_config.frame_id = frame_id;
      decoder_config.sensor_model = sensor_model;

      std::string output_topic =
        generate_output_topic(topic_name, "/nebula_packets", "/nebula_points");
      conversion_stats[topic_name] = {output_topic, 0, 0};

      RCLCPP_INFO(
        g_logger, "Found NebulaPackets topic: %s -> %s (sensor_model: %s, frame_id: %s)",
        topic_name.c_str(), output_topic.c_str(), decoder_config.sensor_model.c_str(),
        decoder_config.frame_id.c_str());

      return std::make_unique<decoder::nebula::NebulaPCDDecoder<PointT>>(decoder_config);
    }

    case DriverType::kSeyond: {
      decoder::seyond::SeyondPCDDecoderConfig decoder_config;
      decoder_config.min_range = config.min_range;
      decoder_config.max_range = config.max_range;
      decoder_config.use_reflectance = config.use_reflectance;

      auto [frame_id, _] = extract_sensor_info(topic_name, "/seyond_packets", config.frame_id);
      decoder_config.frame_id = frame_id;

      std::string output_topic =
        generate_output_topic(topic_name, "/seyond_packets", "/seyond_points");
      conversion_stats[topic_name] = {output_topic, 0, 0};

      RCLCPP_INFO(
        g_logger, "Found SeyondScan topic: %s -> %s (frame_id: %s)", topic_name.c_str(),
        output_topic.c_str(), decoder_config.frame_id.c_str());

      return std::make_unique<decoder::seyond::SeyondPCDDecoder<PointT>>(decoder_config);
    }
  }

  // This should never be reached due to exhaustive switch
  throw std::logic_error("Unhandled DriverType in create_decoder");
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

  if (!has_decodable_topics) {
    RCLCPP_WARN(
      g_logger, "Skipping conversion: no decodable topics found in %s",
      config.src_bag_path.c_str());
    return BagConverterResultStatus::kSkipped;
  }

  if (transformer && !has_tf_topic) {
    RCLCPP_WARN(
      g_logger, "Skipping conversion: no TF topics found in %s", config.src_bag_path.c_str());
    return BagConverterResultStatus::kSkipped;
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
      writer.write(bag_msg);
      continue;
    }

    const auto & topic_type = type_it->second;

    bool is_nebula = (topic_type == "nebula_msgs/msg/NebulaPackets");
    bool is_seyond = (topic_type == "seyond/msg/SeyondScan");

    if (!is_nebula && !is_seyond) {
      // Other messages: pass through
      writer.write(bag_msg);

      if (message_count % 1000 == 0) {
        RCLCPP_INFO(g_logger, "Processed %zu messages...", message_count);
      }
      continue;
    }

    // Keep original if requested
    if (config.keep_original_topics) {
      writer.write(bag_msg);
    }

    // Create decoder on first encounter
    if (decoders.find(topic_name) == decoders.end()) {
      DriverType driver_type = is_nebula ? DriverType::kNebula : DriverType::kSeyond;
      decoders[topic_name] =
        create_decoder<PointT>(driver_type, topic_name, config, conversion_stats);
    }

    // Decode using polymorphic interface
    rclcpp::SerializedMessage serialized_msg(*bag_msg->serialized_data);
    auto & decoder = decoders[topic_name];
    auto pointcloud_msg = decoder->decode(serialized_msg);

    if (pointcloud_msg) {
      // Skip status packets (messages with too few points)
      const size_t num_points = pointcloud_msg->width * pointcloud_msg->height;
      if (num_points < defaults::min_points_per_scan) {
        RCLCPP_INFO(g_logger, "Status packets detected (skipped decoding this message)");
        conversion_stats[topic_name].skipped_count++;
        continue;
      }

      if (config.timescale_correction) {
        const std::uint64_t sensor_time_ns =
          static_cast<std::uint64_t>(pointcloud_msg->header.stamp.sec) * 1000000000 +
          static_cast<std::uint64_t>(pointcloud_msg->header.stamp.nanosec);
        const std::uint64_t sensor_time_ns_corrected = timescale::correct_timescale(
          sensor_time_ns, rclcpp::Time(bag_msg->time_stamp).nanoseconds(),
          config.timescale_correction_ref);
        if (sensor_time_ns_corrected != sensor_time_ns) {
          pointcloud_msg->header.stamp.sec = sensor_time_ns_corrected / 1000000000;
          pointcloud_msg->header.stamp.nanosec = sensor_time_ns_corrected % 1000000000;
        }
      }

      if (transformer) {
        if (!transformer->transform(*pointcloud_msg, config.frame)) {
          RCLCPP_ERROR(
            g_logger, "TF transform failed for frame: %s", pointcloud_msg->header.frame_id.c_str());
          return BagConverterResultStatus::kError;
        }
      }

      auto pc2_msg_serialized = std::make_shared<rclcpp::SerializedMessage>();
      pc2_serializer.serialize_message(pointcloud_msg.get(), pc2_msg_serialized.get());

      writer.write(
        pc2_msg_serialized, conversion_stats[topic_name].output_topic,
        "sensor_msgs/msg/PointCloud2", rclcpp::Time(bag_msg->time_stamp));

      conversion_stats[topic_name].decoded_count++;
    }

    if (message_count % 1000 == 0) {
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
  return ret == BagConverterResultStatus::kError ? 1 : 0;
}

static std::vector<fs::path> find_bag_files(const fs::path & input_dir)
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

}  // namespace bag_converter

int main(int argc, char ** argv)
{
  bag_converter::BagConverterConfig config;
  auto parse_result = bag_converter::parse_arguments(argc, argv, config);
  if (parse_result.has_value()) {
    return parse_result.value();
  }

  rclcpp::init(argc, argv);
  int result = config.batch_mode ? bag_converter::run_batch(config) : bag_converter::run(config);
  rclcpp::shutdown();

  return result;
}
