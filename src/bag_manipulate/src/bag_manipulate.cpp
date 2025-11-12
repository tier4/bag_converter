#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rcpputils/filesystem_helper.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_storage/storage_options.hpp>

#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

class BagManipulator
{
public:
  BagManipulator(const std::string & input_bag, const std::string & output_bag)
  : input_bag_path_(input_bag), output_bag_path_(output_bag)
  {
    if (!rcpputils::fs::exists(input_bag_path_)) {
      throw std::runtime_error("Input bag file does not exist: " + input_bag_path_);
    }

    if (rcpputils::fs::exists(output_bag_path_)) {
      throw std::runtime_error("Output bag file already exists: " + output_bag_path_);
    }
  }

  void process()
  {
    rosbag2_cpp::Reader reader;
    rosbag2_storage::StorageOptions read_storage_options;
    read_storage_options.uri = input_bag_path_;
    read_storage_options.storage_id = "mcap";

    rosbag2_cpp::ConverterOptions converter_options{
      rmw_get_serialization_format(), rmw_get_serialization_format()};

    reader.open(read_storage_options, converter_options);

    rosbag2_cpp::Writer writer;
    rosbag2_storage::StorageOptions write_storage_options;
    write_storage_options.uri = output_bag_path_;
    write_storage_options.storage_id = "mcap";

    writer.open(write_storage_options, converter_options);

    auto topics_and_types = reader.get_all_topics_and_types();
    for (const auto & topic_metadata : topics_and_types) {
      rosbag2_storage::TopicMetadata new_topic_metadata;
      new_topic_metadata.name = topic_metadata.name;
      new_topic_metadata.type = topic_metadata.type;
      new_topic_metadata.serialization_format = topic_metadata.serialization_format;
      new_topic_metadata.offered_qos_profiles = topic_metadata.offered_qos_profiles;

      writer.create_topic(new_topic_metadata);

      std::cout << "Created topic: " << topic_metadata.name << " [" << topic_metadata.type << "]"
                << std::endl;
    }

    std::map<std::string, std::string> topic_type_map;
    for (const auto & topic_metadata : topics_and_types) {
      topic_type_map[topic_metadata.name] = topic_metadata.type;
    }

    size_t message_count = 0;
    size_t processed_count = 0;

    while (reader.has_next()) {
      auto bag_message = reader.read_next();
      message_count++;

      auto serialized_msg = std::make_shared<rosbag2_storage::SerializedBagMessage>();
      serialized_msg->topic_name = bag_message->topic_name;
      serialized_msg->time_stamp = bag_message->time_stamp;
      serialized_msg->serialized_data = bag_message->serialized_data;

      bool should_process =
        shouldProcessMessage(bag_message->topic_name, topic_type_map[bag_message->topic_name]);

      if (should_process) {
        processMessage(serialized_msg, topic_type_map[bag_message->topic_name]);
        processed_count++;
      }

      writer.write(serialized_msg);

      if (message_count % 1000 == 0) {
        std::cout << "Processed " << message_count << " messages..." << std::endl;
      }
    }

    std::cout << "\nProcessing completed!" << std::endl;
    std::cout << "Total messages: " << message_count << std::endl;
    std::cout << "Modified messages: " << processed_count << std::endl;
  }

protected:
  virtual bool shouldProcessMessage(const std::string & topic_name, const std::string & topic_type)
  {
    (void)topic_name;
    (void)topic_type;
    return false;
  }

  virtual void processMessage(
    std::shared_ptr<rosbag2_storage::SerializedBagMessage> msg, const std::string & topic_type)
  {
    (void)msg;
    (void)topic_type;
  }

private:
  std::string input_bag_path_;
  std::string output_bag_path_;
};

void print_usage(const char * program_name)
{
  std::cout << "Usage: " << program_name << " <input_bag> <output_bag>" << std::endl;
  std::cout << "  input_bag:  Path to the input ROS2 bag file" << std::endl;
  std::cout << "  output_bag: Path for the output ROS2 bag file" << std::endl;
}

int main(int argc, char ** argv)
{
  if (argc != 3) {
    print_usage(argv[0]);
    return 1;
  }

  std::string input_bag = argv[1];
  std::string output_bag = argv[2];

  try {
    std::cout << "Starting bag manipulation..." << std::endl;
    std::cout << "Input:  " << input_bag << std::endl;
    std::cout << "Output: " << output_bag << std::endl << std::endl;

    BagManipulator manipulator(input_bag, output_bag);
    manipulator.process();

  } catch (const std::exception & e) {
    std::cerr << "Error: " << e.what() << std::endl;
    return 1;
  }

  return 0;
}