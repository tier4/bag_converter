/**
 * @file seyond_nebula_bag_decoder.cpp
 * @brief Decode Seyond LiDAR nebula packets from rosbag and convert to point clouds
 */

#include "point_types.hpp"
#include "nebula_decoder.hpp"

#include <builtin_interfaces/msg/time.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_storage/storage_options.hpp>

#include <nebula_msgs/msg/nebula_packet.hpp>
#include <nebula_msgs/msg/nebula_packets.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <cstdint>
#include <filesystem>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

inline constexpr double k_utc_offset_to_tai_sec = 37;
inline constexpr double k_utc_offset_to_gps_sec = 18;
inline constexpr double k_gps_offset_to_tai_sec = 19;
inline constexpr double k_timescale_correction_tolerance_sec = 0.35;
inline constexpr size_t k_min_points_per_scan = 1000;

std::uint64_t correct_timescale(
  std::uint64_t ref_time_ns, std::uint64_t time_ns_to_correct,
  const std::string & ref_timescale = "utc")
{
  const double ref_time_sec = static_cast<double>(ref_time_ns) / 1e9;
  const double time_sec_to_correct = static_cast<double>(time_ns_to_correct) / 1e9;
  const auto diff = ref_time_sec - time_sec_to_correct;
  if (ref_timescale == "utc") {
    // ref: UTC, to-correct: TAI
    if (
      diff > (-k_utc_offset_to_tai_sec - k_timescale_correction_tolerance_sec) &&
      diff < (-k_utc_offset_to_tai_sec + k_timescale_correction_tolerance_sec)) {
      return time_ns_to_correct - static_cast<std::uint64_t>(k_utc_offset_to_tai_sec * 1e9);
    }
    // ref: UTC, to-correct: GPS
    if (
      diff > (-k_utc_offset_to_gps_sec - k_timescale_correction_tolerance_sec) &&
      diff < (-k_utc_offset_to_gps_sec + k_timescale_correction_tolerance_sec)) {
      return time_ns_to_correct - static_cast<std::uint64_t>(k_utc_offset_to_gps_sec * 1e9);
    }
  } else if (ref_timescale == "tai") {
    // ref: TAI, to-correct: UTC
    if (
      diff > (k_utc_offset_to_tai_sec - k_timescale_correction_tolerance_sec) &&
      diff < (k_utc_offset_to_tai_sec + k_timescale_correction_tolerance_sec)) {
      return time_ns_to_correct + static_cast<std::uint64_t>(k_utc_offset_to_tai_sec * 1e9);
    }
    // ref: TAI, to-correct: GPS
    if (
      diff > (k_gps_offset_to_tai_sec - k_timescale_correction_tolerance_sec) &&
      diff < (k_gps_offset_to_tai_sec + k_timescale_correction_tolerance_sec)) {
      return time_ns_to_correct + static_cast<std::uint64_t>(k_gps_offset_to_tai_sec * 1e9);
    }
  } else if (ref_timescale == "gps") {
    // ref: GPS, to-correct: UTC
    if (
      diff > (k_utc_offset_to_gps_sec - k_timescale_correction_tolerance_sec) &&
      diff < (k_utc_offset_to_gps_sec + k_timescale_correction_tolerance_sec)) {
      return time_ns_to_correct + static_cast<std::uint64_t>(k_utc_offset_to_gps_sec * 1e9);
    }
    // ref: GPS, to-correct: TAI
    if (
      diff > (-k_gps_offset_to_tai_sec - k_timescale_correction_tolerance_sec) &&
      diff < (-k_gps_offset_to_tai_sec + k_timescale_correction_tolerance_sec)) {
      return time_ns_to_correct - static_cast<std::uint64_t>(k_gps_offset_to_tai_sec * 1e9);
    }
  }
  return time_ns_to_correct;
}
namespace fs = std::filesystem;

class SeyondNebulaBagDecoder
{
public:
  struct Config
  {
    std::string input_bag_path;
    std::string output_bag_path;

    // Decoder configuration
    std::string sensor_model = "Falcon";
    std::string return_mode = "Dual";
    std::string frame_id = "lidar_top";
    double min_range = 0.3;
    double max_range = 200.0;
    int coordinate_mode = 3;
    bool use_reflectance = true;
    std::string calibration_file;
    bool keep_original_topics = false;
    bool enable_timescale_correction = true;
    std::string system_timescale = "utc";
  };

  explicit SeyondNebulaBagDecoder(Config config) : config_(std::move(config)) {}

  bool process()
  {
    // Check input file exists
    if (!fs::exists(config_.input_bag_path)) {
      std::cerr << "Error: Input bag file does not exist: " << config_.input_bag_path << std::endl;
      return false;
    }

    // Open input bag
    rosbag2_storage::StorageOptions storage_options_in;
    storage_options_in.uri = config_.input_bag_path;

    rosbag2_cpp::Reader reader;
    try {
      reader.open(storage_options_in);
    } catch (const std::exception & e) {
      std::cerr << "Error opening input bag: " << e.what() << std::endl;
      return false;
    }

    // Get bag metadata
    const auto bag_metadata = reader.get_metadata();

    // Discover all nebula_packets topics to convert
    std::map<std::string, std::string> topic_mapping;  // input_topic -> output_topic
    std::map<std::string, std::unique_ptr<bag_converter::decoder::NebulaPCDDecoder>> decoders;

    std::cout << "Scanning for nebula packet topics..." << std::endl;

    for (const auto & topic_info : bag_metadata.topics_with_message_count) {
      const auto & topic_metadata = topic_info.topic_metadata;

      // Check if this is a nebula packets topic
      if (
        topic_metadata.type == "nebula_msgs/msg/NebulaPackets" &&
        topic_metadata.name.find("/nebula_packets") != std::string::npos) {
        // Generate converted topic name (replace nebula_packets with pointcloud)
        size_t pos = topic_metadata.name.find("/nebula_packets");
        if (pos == std::string::npos) {
          continue;
        }
        std::string converted_topic = topic_metadata.name.substr(0, pos) + "/nebula_points";
        topic_mapping[topic_metadata.name] = converted_topic;

        // Create decoder for this topic with appropriate frame_id
        bag_converter::decoder::NebulaPCDDecoderConfig decoder_config;
        decoder_config.sensor_model = config_.sensor_model;
        decoder_config.return_mode = config_.return_mode;
        decoder_config.min_range = config_.min_range;
        decoder_config.max_range = config_.max_range;

        // Extract sensor name from topic (e.g., /sensing/lidar/front/nebula_packets ->
        // lidar_front)
        size_t last_slash = topic_metadata.name.rfind("/nebula_packets");
        if (last_slash != std::string::npos && last_slash > 0) {
          size_t second_last_slash = topic_metadata.name.rfind('/', last_slash - 1);
          if (second_last_slash != std::string::npos) {
            std::string sensor_pos =
              topic_metadata.name.substr(second_last_slash + 1, last_slash - second_last_slash - 1);
            decoder_config.frame_id = "lidar_" + sensor_pos;
            if (sensor_pos == "front" || sensor_pos == "rear") {
              decoder_config.sensor_model = "Falcon";
            } else if (sensor_pos == "left" || sensor_pos == "right") {
              decoder_config.sensor_model = "Robin_W";
            }
          }
        }

        decoders[topic_metadata.name] =
          std::make_unique<bag_converter::decoder::NebulaPCDDecoder>(decoder_config);

        std::cout << "Found a decodable topic: " << topic_metadata.name
                  << " (sensor_model: " << decoder_config.sensor_model
                  << ", return_mode: " << decoder_config.return_mode
                  << ", min_range=" << decoder_config.min_range
                  << ", max_range=" << decoder_config.max_range
                  << ", frame_id=" << decoder_config.frame_id << ")" << std::endl;
      }
    }

    if (topic_mapping.empty()) {
      std::cout << "No nebula packet topics found in the input bag: " << config_.input_bag_path
                << std::endl;
      return true;
    }

    // Prepare output bag
    rosbag2_storage::StorageOptions storage_options_out;
    storage_options_out.uri = config_.output_bag_path;
    storage_options_out.storage_id = bag_metadata.storage_identifier;

    rosbag2_cpp::Writer writer;
    try {
      writer.open(storage_options_out);
    } catch (const std::exception & e) {
      std::cerr << "Error opening output bag: " << e.what() << std::endl;
      return false;
    }

    // Create topics in output bag
    for (const auto & topic_info : bag_metadata.topics_with_message_count) {
      const auto & topic_metadata = topic_info.topic_metadata;

      // Check if this is a nebula topic to convert
      auto it = topic_mapping.find(topic_metadata.name);
      if (it != topic_mapping.end()) {
        // Create converted point cloud topic
        rosbag2_storage::TopicMetadata pointcloud_topic_meta;
        pointcloud_topic_meta.name = it->second;
        pointcloud_topic_meta.type = "sensor_msgs/msg/PointCloud2";
        pointcloud_topic_meta.serialization_format = "cdr";
        writer.create_topic(pointcloud_topic_meta);

        // Keep original nebula packets topic if requested
        if (config_.keep_original_topics) {
          writer.create_topic(topic_metadata);
        }
      } else {
        // Not a nebula topic, copy as-is
        writer.create_topic(topic_metadata);
      }
    }

    // Process messages
    rclcpp::Serialization<nebula_msgs::msg::NebulaPackets> nebula_serializer;
    rclcpp::Serialization<sensor_msgs::msg::PointCloud2> pc2_serializer;

    std::map<std::string, size_t> topic_conversion_success_counts;

    while (reader.has_next()) {
      auto bag_msg = reader.read_next();

      // Check if this is a nebula topic to convert
      auto it = topic_mapping.find(bag_msg->topic_name);
      if (it == topic_mapping.end()) {
        // Write other messages as-is and continue
        writer.write(bag_msg);
        continue;
      }

      // Write original nebula packets message if requested
      if (config_.keep_original_topics) {
        writer.write(bag_msg);
      }

      // Deserialize message
      rclcpp::SerializedMessage serialized_msg(*bag_msg->serialized_data);
      nebula_msgs::msg::NebulaPackets nebula_msg;
      nebula_serializer.deserialize_message(&serialized_msg, &nebula_msg);

      // Get decoder for this topic
      auto & decoder = decoders[bag_msg->topic_name];

      // Decode packets to point cloud
      decoder->process_nebula_packets(nebula_msg);
      auto nebula_cloud = decoder->flush_cloud_points();
      if (
        !nebula_cloud || nebula_cloud->empty() ||
        nebula_cloud->points.size() <= k_min_points_per_scan) {
        std::cerr << "A diagnostic packet found in topic " << bag_msg->topic_name << " (skipped)"
                  << std::endl;
        continue;
      }

      // Process and write the cloud
      const auto & input_topic = bag_msg->topic_name;
      const auto & output_topic = it->second;
      const rclcpp::Time receive_timestamp(bag_msg->time_stamp);

      // Apply timescale correction
      if (config_.enable_timescale_correction) {
        const uint64_t sensor_time_ns = nebula_cloud->header.stamp * 1000;
        const auto sensor_time_ns_corrected = correct_timescale(
          receive_timestamp.nanoseconds(), sensor_time_ns, config_.system_timescale);
        nebula_cloud->header.stamp = sensor_time_ns_corrected / 1000;
      }

      // Convert to PointXYZIT for PCL conversion
      pcl::PointCloud<bag_converter::point::PointXYZIT> pc2_cloud;
      pc2_cloud.header = nebula_cloud->header;
      pc2_cloud.header.frame_id = decoders[input_topic]->get_config().frame_id;
      pc2_cloud.width = nebula_cloud->width;
      pc2_cloud.height = nebula_cloud->height;
      pc2_cloud.is_dense = nebula_cloud->is_dense;
      for (const auto & pt : nebula_cloud->points) {
        bag_converter::point::PointXYZIT pc2_pt;
        pc2_pt.x = pt.x;
        pc2_pt.y = pt.y;
        pc2_pt.z = pt.z;
        pc2_pt.intensity = pt.intensity;
        pc2_pt.t_us = pt.time_stamp;
        pc2_cloud.push_back(pc2_pt);
      }

      // Create PointCloud2 message
      sensor_msgs::msg::PointCloud2 pc2_msg;
      pcl::toROSMsg(pc2_cloud, pc2_msg);

      // Serialize and write to bag
      auto pc2_msg_serialized = std::make_shared<rclcpp::SerializedMessage>();
      pc2_serializer.serialize_message(&pc2_msg, pc2_msg_serialized.get());

      // Write to bag file
      writer.write(
        pc2_msg_serialized, output_topic, "sensor_msgs/msg/PointCloud2", receive_timestamp);

      topic_conversion_success_counts[input_topic]++;
    }

    std::cout << "\n========== Conversion Summary ==========" << std::endl;
    for (const auto & [topic, count] : topic_conversion_success_counts) {
      std::cout << "[" << topic << "]" << std::endl;
      std::cout << "    Destination topic: " << topic_mapping[topic] << std::endl;
      std::cout << "    Decoded: " << count << " messages" << std::endl;
    }
    std::cout << "========================================" << std::endl;

    return true;
  }

private:
  Config config_;
};

void print_usage(const char * program_name)
{
  std::cout
    << "Usage: " << program_name << " <input_bag> <output_bag> [options]\n"
    << "\nThis tool automatically detects and converts all nebula packet topics.\n"
    << "Topics containing '/nebula_packets' will be converted to '/nebula_points'.\n"
    << "\nTimescale Correction:\n"
    << "  Timescale correction automatically detects time offset between system timestamp\n"
    << "  and sensor timestamp. If a timescale difference occurs between system and sensor\n"
    << "  timestamps, correction is applied. The --system-timescale option specifies the\n"
    << "  timescale of the reference system timestamp.\n"
    << "\nOptions:\n"
    << "  --keep-original-topics        Keep original /nebula_packets topics in output bag\n"
    << "  --no-timescale-correction      Disable timescale correction (UTC/TAI/GPS conversion)\n"
    << "  --system-timescale <scale>     Timescale of reference system timestamp for correction\n"
    << "                                  (utc/tai/gps, default: utc)\n";
}

int main(int argc, char ** argv)
{
  // Check for help option
  for (int i = 1; i < argc; i++) {
    std::string arg = argv[i];
    if (arg == "--help" || arg == "-h") {
      print_usage(argv[0]);
      return 0;
    }
  }

  // Parse command line arguments
  if (argc < 3) {
    print_usage(argv[0]);
    return 1;
  }

  SeyondNebulaBagDecoder::Config config;
  config.input_bag_path = argv[1];
  config.output_bag_path = argv[2];

  // Parse optional arguments
  for (int i = 3; i < argc; i++) {
    std::string arg = argv[i];
    if (arg == "--keep-original-topics") {
      config.keep_original_topics = true;
    } else if (arg == "--no-timescale-correction") {
      config.enable_timescale_correction = false;
    } else if (arg == "--system-timescale") {
      if (i + 1 >= argc) {
        std::cerr << "Error: --system-timescale requires a value (utc/tai/gps)" << std::endl;
        return 1;
      }
      config.system_timescale = argv[++i];
      if (
        config.system_timescale != "utc" && config.system_timescale != "tai" &&
        config.system_timescale != "gps") {
        std::cerr << "Error: --system-timescale must be one of: utc, tai, gps" << std::endl;
        return 1;
      }
    }
  }

  // Initialize ROS2 (required for serialization)
  rclcpp::init(argc, argv);

  // Process bag
  SeyondNebulaBagDecoder decoder(config);
  bool success = decoder.process();

  // Cleanup
  rclcpp::shutdown();

  return success ? 0 : 1;
}