/*
 *  Copyright (C) 2025 Seyond Inc.
 *
 *  License: Apache License
 *
 *  Automatic bag conversion for all Seyond LiDAR topics
 */

#include "seyond_decoder/msg/seyond_scan.hpp"
#include "seyond_decoder/seyond_decoder.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/writer.hpp>

#include "sensor_msgs/msg/point_cloud2.hpp"

#include <algorithm>
#include <iostream>
#include <map>
#include <memory>
#include <string>

class SeyondBagConverter
{
public:
  SeyondBagConverter(
    const std::string & input_bag, const std::string & output_bag,
    const seyond::DecoderConfig & config = seyond::DecoderConfig())
  : input_bag_path_(input_bag), output_bag_path_(output_bag), default_config_(config)
  {
    std::cout << "SeyondBagConverter initialized" << std::endl;
    std::cout << "Input bag: " << input_bag_path_ << std::endl;
    std::cout << "Output bag: " << output_bag_path_ << std::endl;
  }

  void process()
  {
    // Setup reader
    rosbag2_cpp::Reader reader;
    rosbag2_storage::StorageOptions read_storage_options;
    read_storage_options.uri = input_bag_path_;
    read_storage_options.storage_id = "mcap";

    rosbag2_cpp::ConverterOptions converter_options{
      rmw_get_serialization_format(), rmw_get_serialization_format()};

    reader.open(read_storage_options, converter_options);

    // Setup writer
    rosbag2_cpp::Writer writer;
    rosbag2_storage::StorageOptions write_storage_options;
    write_storage_options.uri = output_bag_path_;
    write_storage_options.storage_id = "mcap";
    write_storage_options.max_bagfile_duration = 60;

    writer.open(write_storage_options, converter_options);

    // Get all topics and detect Seyond topics
    auto topics_and_types = reader.get_all_topics_and_types();
    std::map<std::string, std::string> seyond_topic_mapping;  // original -> converted topic name
    std::map<std::string, std::unique_ptr<seyond::SeyondDecoder>> decoders;

    for (const auto & topic_metadata : topics_and_types) {
      // Check if this is a Seyond packet topic
      if (
        topic_metadata.name.find("/seyond_packets") != std::string::npos &&
        topic_metadata.type == "seyond/msg/SeyondScan") {
        // Create converted topic name by replacing /seyond_packets with /seyond_points
        std::string converted_topic = topic_metadata.name;
        size_t pos = converted_topic.find("/seyond_packets");
        if (pos != std::string::npos) {
          converted_topic.replace(pos, 15, "/seyond_points");  // 15 is length of "/seyond_packets"
        }

        seyond_topic_mapping[topic_metadata.name] = converted_topic;

        // Create decoder for this topic with appropriate frame_id
        seyond::DecoderConfig config = default_config_;
        // Extract sensor name from topic (e.g., /sensing/lidar/front/seyond_packets -> front)
        size_t last_slash = topic_metadata.name.rfind("/seyond_packets");
        if (last_slash != std::string::npos && last_slash > 0) {
          size_t second_last_slash = topic_metadata.name.rfind('/', last_slash - 1);
          if (second_last_slash != std::string::npos) {
            config.frame_id =
              topic_metadata.name.substr(second_last_slash + 1, last_slash - second_last_slash - 1);
          }
        }

        decoders[topic_metadata.name] = std::make_unique<seyond::SeyondDecoder>(config);

        std::cout << "Found Seyond topic: " << topic_metadata.name << " -> " << converted_topic
                  << " (frame_id: " << config.frame_id << ")" << std::endl;

        // Create PointCloud2 topic in output bag
        rosbag2_storage::TopicMetadata pointcloud_topic_metadata;
        pointcloud_topic_metadata.name = converted_topic;
        pointcloud_topic_metadata.type = "sensor_msgs/msg/PointCloud2";
        pointcloud_topic_metadata.serialization_format = rmw_get_serialization_format();
        writer.create_topic(pointcloud_topic_metadata);

      } else {
        // Copy other topics as-is
        rosbag2_storage::TopicMetadata new_topic_metadata;
        new_topic_metadata.name = topic_metadata.name;
        new_topic_metadata.type = topic_metadata.type;
        new_topic_metadata.serialization_format = topic_metadata.serialization_format;
        new_topic_metadata.offered_qos_profiles = topic_metadata.offered_qos_profiles;
        writer.create_topic(new_topic_metadata);
      }
    }

    if (seyond_topic_mapping.empty()) {
      std::cout << "No Seyond packet topics found in the input bag!" << std::endl;
      std::cout
        << "Looking for topics containing '/seyond_packets' with type 'seyond/msg/SeyondScan'"
        << std::endl;
    } else {
      std::cout << "\nFound " << seyond_topic_mapping.size() << " Seyond topic(s) to convert"
                << std::endl;
    }

    // Serializers
    rclcpp::Serialization<seyond_decoder::msg::SeyondScan> seyond_serializer;
    rclcpp::Serialization<sensor_msgs::msg::PointCloud2> pointcloud_serializer;

    // Process messages
    size_t message_count = 0;
    size_t converted_count = 0;
    std::map<std::string, size_t> topic_conversion_counts;

    while (reader.has_next()) {
      auto bag_message = reader.read_next();
      message_count++;

      // Check if this is a Seyond topic to convert
      auto it = seyond_topic_mapping.find(bag_message->topic_name);
      if (it != seyond_topic_mapping.end()) {
        // Deserialize SeyondScan
        seyond_decoder::msg::SeyondScan scan_msg;
        rclcpp::SerializedMessage serialized_msg(*bag_message->serialized_data);
        seyond_serializer.deserialize_message(&serialized_msg, &scan_msg);

        // Convert to PointCloud2 using appropriate decoder
        auto & decoder = decoders[bag_message->topic_name];
        auto pointcloud_msg = decoder->convert(scan_msg);

        if (pointcloud_msg) {
          // Serialize and write PointCloud2
          auto serialized_msg = std::make_shared<rclcpp::SerializedMessage>();
          pointcloud_serializer.serialize_message(pointcloud_msg.get(), serialized_msg.get());

          // Write to bag file
          writer.write(
            serialized_msg,
            it->second,  // Use converted topic name
            "sensor_msgs/msg/PointCloud2", rclcpp::Time(bag_message->time_stamp));

          converted_count++;
          topic_conversion_counts[bag_message->topic_name]++;
        }
      } else {
        // Write other messages as-is
        auto copied_message = std::make_shared<rosbag2_storage::SerializedBagMessage>();
        copied_message->topic_name = bag_message->topic_name;
        copied_message->time_stamp = bag_message->time_stamp;
        copied_message->serialized_data = bag_message->serialized_data;
        writer.write(copied_message);
      }

      if (message_count % 1000 == 0) {
        std::cout << "Processed " << message_count << " messages, converted " << converted_count
                  << " SeyondScan messages" << std::endl;
      }
    }

    std::cout << "\n========== Conversion Summary ==========" << std::endl;
    std::cout << "Total messages processed: " << message_count << std::endl;
    std::cout << "Total SeyondScan messages converted: " << converted_count << std::endl;

    if (!topic_conversion_counts.empty()) {
      std::cout << "\nConversion details by topic:" << std::endl;
      for (const auto & [topic, count] : topic_conversion_counts) {
        std::cout << "  " << topic << ": " << count << " messages" << std::endl;
        std::cout << "    -> " << seyond_topic_mapping[topic] << std::endl;
      }
    }
    std::cout << "========================================" << std::endl;
  }

private:
  std::string input_bag_path_;
  std::string output_bag_path_;
  seyond::DecoderConfig default_config_;
  // Multiple decoders are now created dynamically in process()
};

int main(int argc, char ** argv)
{
  if (argc < 3) {
    std::cerr << "Usage: " << argv[0] << " <input_bag> <output_bag> [options]" << std::endl;
    std::cerr << "\nThis tool automatically detects and converts all Seyond LiDAR topics."
              << std::endl;
    std::cerr << "Topics containing '/seyond_packets' will be converted to '/seyond_points'."
              << std::endl;
    std::cerr << "\nOptions:" << std::endl;
    std::cerr << "  --max-range <value>    Maximum range in meters (default: 200.0)" << std::endl;
    std::cerr << "  --min-range <value>    Minimum range in meters (default: 0.3)" << std::endl;
    std::cerr << "  --coordinate-mode <n>  Coordinate mode 0-4 (default: 0)" << std::endl;
    return 1;
  }

  std::string input_bag = argv[1];
  std::string output_bag = argv[2];

  try {
    // Configure decoder with command line options
    seyond::DecoderConfig config;
    config.max_range = 200.0;
    config.min_range = 0.3;
    config.coordinate_mode = 3;
    config.use_reflectance = false;
    config.frame_id = "lidar";  // Will be overridden per topic

    // Parse optional command line arguments
    for (int i = 3; i < argc; i++) {
      std::string arg = argv[i];
      if (arg == "--max-range" && i + 1 < argc) {
        config.max_range = std::stod(argv[++i]);
      } else if (arg == "--min-range" && i + 1 < argc) {
        config.min_range = std::stod(argv[++i]);
      } else if (arg == "--coordinate-mode" && i + 1 < argc) {
        config.coordinate_mode = std::stoi(argv[++i]);
      }
    }

    std::cout << "Configuration:" << std::endl;
    std::cout << "  Max range: " << config.max_range << " m" << std::endl;
    std::cout << "  Min range: " << config.min_range << " m" << std::endl;
    std::cout << "  Coordinate mode: " << config.coordinate_mode << std::endl;
    std::cout << "  Output format: PointXYZIT (with timestamps)" << std::endl;
    std::cout << std::endl;

    // Create and run converter
    SeyondBagConverter converter(input_bag, output_bag, config);
    converter.process();
  } catch (const std::exception & e) {
    std::cerr << "Error: " << e.what() << std::endl;
    return 1;
  }
  return 0;
}