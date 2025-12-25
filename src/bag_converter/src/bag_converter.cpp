/*
 *  Copyright (C) 2025 TierIV Inc.
 *
 *  License: Apache License
 *
 *  Unified bag converter for Seyond LiDAR topics
 *  Automatically detects and converts both NebulaPackets and SeyondScan messages
 */

#include "bag_converter.hpp"

#include <filesystem>
#include <iostream>
#include <set>

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

void print_summary(const std::map<std::string, std::pair<std::string, size_t>> & conversion_counts)
{
  RCLCPP_INFO(g_logger, "========== Conversion Summary ==========");

  for (const auto & [input_topic, output_info] : conversion_counts) {
    const auto & [output_topic, count] = output_info;
    RCLCPP_INFO(g_logger, "  [%s]", input_topic.c_str());
    RCLCPP_INFO(g_logger, "      Destination: %s", output_topic.c_str());
    RCLCPP_INFO(g_logger, "      Decoded: %zu messages", count);
  }

  RCLCPP_INFO(g_logger, "========================================");
}

void print_usage(const char * program_name)
{
  std::cout << "Usage: " << program_name << " <input_bag> <output_bag> [options]\n"
            << "\nUnified bag converter for Seyond LiDAR topics.\n"
            << "Automatically detects and converts both NebulaPackets and SeyondScan messages.\n"
            << "\nSupported input formats:\n"
            << "  - nebula_msgs/msg/NebulaPackets (topics containing '/nebula_packets')\n"
            << "  - seyond/msg/SeyondScan or bag_converter/msg/SeyondScan "
               "(topics containing '/seyond_packets')\n"
            << "\nOutput format:\n"
            << "  - sensor_msgs/msg/PointCloud2 with PointXYZIT fields\n"
            << "\nOptions:\n"
            << "  --keep-original           Keep original packet topics in output bag\n"
            << "  --min-range <value>       Minimum range in meters (default: 0.3)\n"
            << "  --max-range <value>       Maximum range in meters (default: 200.0)\n"
            << "  --point-type <type>       Output point type: xyzit or xyzi (default: xyzit)\n"
            << "  -h, --help                Show this help message\n";
}

bool parse_arguments(int argc, char ** argv, Config & config)
{
  for (int i = 1; i < argc; i++) {
    std::string arg = argv[i];
    if (arg == "--help" || arg == "-h") {
      print_usage(argv[0]);
      return false;
    }
  }

  if (argc < 3) {
    print_usage(argv[0]);
    return false;
  }

  config.input_bag_path = argv[1];
  config.output_bag_path = argv[2];

  for (int i = 3; i < argc; i++) {
    std::string arg = argv[i];
    if (arg == "--keep-original") {
      config.keep_original_topics = true;
    } else if (arg == "--min-range" && i + 1 < argc) {
      config.min_range = std::stod(argv[++i]);
    } else if (arg == "--max-range" && i + 1 < argc) {
      config.max_range = std::stod(argv[++i]);
    } else if (arg == "--point-type" && i + 1 < argc) {
      config.point_type = argv[++i];
      if (config.point_type != "xyzit" && config.point_type != "xyzi") {
        std::cerr << "Error: Invalid point type '" << config.point_type
                  << "'. Must be 'xyzit' or 'xyzi'.\n";
        return false;
      }
    }
  }

  return true;
}

/**
 * @brief Create a Nebula decoder for the given topic
 */
template <typename PointT>
std::unique_ptr<decoder::BasePCDDecoder> create_nebula_decoder(
  const std::string & topic_name, const Config & config,
  std::map<std::string, std::pair<std::string, size_t>> & conversion_counts)
{
  decoder::nebula::NebulaPCDDecoderConfig decoder_config;
  decoder_config.min_range = config.min_range;
  decoder_config.max_range = config.max_range;

  auto [frame_id, sensor_model] =
    extract_sensor_info(topic_name, "/nebula_packets", config.frame_id);
  decoder_config.frame_id = frame_id;
  decoder_config.sensor_model = sensor_model;

  std::string output_topic = generate_output_topic(topic_name, "/nebula_packets", "/nebula_points");
  conversion_counts[topic_name] = {output_topic, 0};

  RCLCPP_INFO(
    g_logger, "Found NebulaPackets topic: %s -> %s (sensor_model: %s, frame_id: %s)",
    topic_name.c_str(), output_topic.c_str(), decoder_config.sensor_model.c_str(),
    decoder_config.frame_id.c_str());

  return std::make_unique<decoder::nebula::NebulaPCDDecoder<PointT>>(decoder_config);
}

/**
 * @brief Create a Seyond decoder for the given topic
 */
template <typename PointT>
std::unique_ptr<decoder::BasePCDDecoder> create_seyond_decoder(
  const std::string & topic_name, const Config & config,
  std::map<std::string, std::pair<std::string, size_t>> & conversion_counts)
{
  decoder::seyond::SeyondPCDDecoderConfig decoder_config;
  decoder_config.min_range = config.min_range;
  decoder_config.max_range = config.max_range;
  decoder_config.use_reflectance = config.use_reflectance;

  auto [frame_id, _] = extract_sensor_info(topic_name, "/seyond_packets", config.frame_id);
  decoder_config.frame_id = frame_id;

  std::string output_topic = generate_output_topic(topic_name, "/seyond_packets", "/seyond_points");
  conversion_counts[topic_name] = {output_topic, 0};

  RCLCPP_INFO(
    g_logger, "Found SeyondScan topic: %s -> %s (frame_id: %s)", topic_name.c_str(),
    output_topic.c_str(), decoder_config.frame_id.c_str());

  return std::make_unique<decoder::seyond::SeyondPCDDecoder<PointT>>(decoder_config);
}

template <typename PointT>
int run_impl(const Config & config)
{
  if (!fs::exists(config.input_bag_path)) {
    RCLCPP_ERROR(g_logger, "Input bag file does not exist: %s", config.input_bag_path.c_str());
    return 1;
  }

  // Open input bag
  rosbag2_storage::StorageOptions storage_options_in;
  storage_options_in.uri = config.input_bag_path;

  rosbag2_cpp::Reader reader;
  try {
    reader.open(storage_options_in);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(g_logger, "Error opening input bag: %s", e.what());
    return 1;
  }

  const auto bag_metadata = reader.get_metadata();

  // Build topic type map from metadata
  std::map<std::string, std::string> topic_types;
  for (const auto & topic_info : bag_metadata.topics_with_message_count) {
    topic_types[topic_info.topic_metadata.name] = topic_info.topic_metadata.type;
  }

  // Open output bag
  rosbag2_storage::StorageOptions storage_options_out;
  storage_options_out.uri = config.output_bag_path;
  storage_options_out.storage_id = bag_metadata.storage_identifier;

  rosbag2_cpp::Writer writer;
  try {
    writer.open(storage_options_out);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(g_logger, "Error opening output bag: %s", e.what());
    return 1;
  }

  // Create output topics based on input metadata
  std::set<std::string> created_topics;
  for (const auto & topic_info : bag_metadata.topics_with_message_count) {
    const auto & topic_metadata = topic_info.topic_metadata;
    const auto & topic_type = topic_metadata.type;

    bool is_nebula = (topic_type == "nebula_msgs/msg/NebulaPackets");
    bool is_seyond =
      (topic_type == "seyond/msg/SeyondScan" || topic_type == "bag_converter/msg/SeyondScan");

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

  // Serializer for output PointCloud2
  rclcpp::Serialization<sensor_msgs::msg::PointCloud2> pc2_serializer;

  // Unified decoder map using polymorphism (type erasure pattern)
  std::map<std::string, std::unique_ptr<decoder::BasePCDDecoder>> decoders;

  // Topic output mapping and conversion counts: input_topic -> (output_topic, count)
  std::map<std::string, std::pair<std::string, size_t>> conversion_counts;

  size_t message_count = 0;

  RCLCPP_INFO(g_logger, "Processing messages...");

  while (reader.has_next()) {
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
    bool is_seyond =
      (topic_type == "seyond/msg/SeyondScan" || topic_type == "bag_converter/msg/SeyondScan");

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
      if (is_nebula) {
        decoders[topic_name] = create_nebula_decoder<PointT>(topic_name, config, conversion_counts);
      } else {
        decoders[topic_name] = create_seyond_decoder<PointT>(topic_name, config, conversion_counts);
      }
    }

    // Decode using polymorphic interface
    rclcpp::SerializedMessage serialized_msg(*bag_msg->serialized_data);
    auto & decoder = decoders[topic_name];
    auto pointcloud_msg = decoder->decode(serialized_msg);

    if (pointcloud_msg) {
      auto pc2_msg_serialized = std::make_shared<rclcpp::SerializedMessage>();
      pc2_serializer.serialize_message(pointcloud_msg.get(), pc2_msg_serialized.get());

      writer.write(
        pc2_msg_serialized, conversion_counts[topic_name].first, "sensor_msgs/msg/PointCloud2",
        rclcpp::Time(bag_msg->time_stamp));

      conversion_counts[topic_name].second++;
    }

    if (message_count % 1000 == 0) {
      RCLCPP_INFO(g_logger, "Processed %zu messages...", message_count);
    }
  }

  print_summary(conversion_counts);

  return 0;
}

int run(const Config & config)
{
  RCLCPP_INFO(g_logger, "Configuration:");
  RCLCPP_INFO(g_logger, "  Min range: %.1f m", config.min_range);
  RCLCPP_INFO(g_logger, "  Max range: %.1f m", config.max_range);
  RCLCPP_INFO(g_logger, "  Point type: %s", config.point_type.c_str());
  RCLCPP_INFO(g_logger, "  Keep original topics: %s", config.keep_original_topics ? "yes" : "no");

  if (config.point_type == "xyzi") {
    return run_impl<point::PointXYZI>(config);
  }
  return run_impl<point::PointXYZIT>(config);
}

}  // namespace bag_converter

int main(int argc, char ** argv)
{
  bag_converter::Config config;
  if (!bag_converter::parse_arguments(argc, argv, config)) {
    return 1;
  }

  rclcpp::init(argc, argv);
  int result = bag_converter::run(config);
  rclcpp::shutdown();

  return result;
}
