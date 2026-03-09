/*
 *  Copyright (C) 2025 TierIV Inc.
 *
 *  License: Apache License
 *
 *  Merge: merge multiple rosbag2 files from distributed log modules
 */

#include "merge.hpp"

#include "bag_converter.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_storage/storage_options.hpp>

#include <algorithm>
#include <filesystem>
#include <map>
#include <memory>
#include <optional>
#include <queue>
#include <set>
#include <string>
#include <tuple>
#include <vector>

namespace fs = std::filesystem;

static const rclcpp::Logger g_logger = rclcpp::get_logger("bag_converter_merge");

namespace bag_converter
{
namespace merge
{

/**
 * @brief Parsed components of a bag filename
 */
struct ParsedBagFilename
{
  std::string sensing_system_id;
  std::string module_id;
  std::string rest;
  std::string extension;
};

/**
 * @brief Group key for merging: (sensing_system_id, rest)
 */
struct MergeGroupKey
{
  std::string sensing_system_id;
  std::string rest;

  bool operator<(const MergeGroupKey & other) const
  {
    return std::tie(sensing_system_id, rest) < std::tie(other.sensing_system_id, other.rest);
  }
};

/**
 * @brief Result tracking for merge operation
 */
struct MergeResult
{
  size_t merged_count = 0;
  size_t skipped_single_count = 0;
  size_t skipped_pattern_count = 0;
  size_t failed_count = 0;
  size_t total_messages = 0;
  size_t deleted_files = 0;
  std::vector<std::string> failed_groups;
};

/**
 * @brief Parse a bag filename into its components
 * @param filename The filename (stem + extension, no directory)
 * @return Parsed components, or nullopt if filename doesn't match the pattern
 */
static std::optional<ParsedBagFilename> parse_bag_filename(const std::string & filename)
{
  // Split into stem and extension
  auto dot_pos = filename.rfind('.');
  if (dot_pos == std::string::npos || dot_pos == 0) {
    return std::nullopt;
  }

  std::string stem = filename.substr(0, dot_pos);
  std::string extension = filename.substr(dot_pos);

  // Find first underscore (separates sensing_system_id from rest)
  auto pos1 = stem.find('_');
  if (pos1 == std::string::npos || pos1 == 0) {
    return std::nullopt;
  }

  // Find second underscore (separates module_id from rest)
  auto pos2 = stem.find('_', pos1 + 1);
  if (pos2 == std::string::npos || pos2 == pos1 + 1) {
    return std::nullopt;
  }

  // Must have something after the second underscore
  if (pos2 + 1 >= stem.size()) {
    return std::nullopt;
  }

  ParsedBagFilename result;
  result.sensing_system_id = stem.substr(0, pos1);
  result.module_id = stem.substr(pos1 + 1, pos2 - pos1 - 1);
  result.rest = stem.substr(pos2 + 1);
  result.extension = extension;
  return result;
}

std::optional<std::map<std::string, rosbag2_storage::TopicMetadata>> collect_topic_union(
  const std::vector<fs::path> & bag_files)
{
  std::map<std::string, rosbag2_storage::TopicMetadata> topic_union;

  for (const auto & bag_path : bag_files) {
    rosbag2_storage::StorageOptions storage_options;
    storage_options.uri = bag_path.string();

    rosbag2_cpp::Reader reader;
    reader.open(storage_options);
    const auto metadata = reader.get_metadata();

    for (const auto & topic_info : metadata.topics_with_message_count) {
      const auto & topic_meta = topic_info.topic_metadata;
      auto it = topic_union.find(topic_meta.name);

      if (it != topic_union.end()) {
        RCLCPP_ERROR(
          g_logger, "Duplicate topic '%s' found across bags. Skipping group.",
          topic_meta.name.c_str());
        return std::nullopt;
      } else {
        topic_union[topic_meta.name] = topic_meta;
      }
    }
  }

  return topic_union;
}

/**
 * @brief Merge a group of bag files into a single output bag
 * @param bag_files List of bag file paths to merge
 * @param output_path Final output file path
 * @param storage_identifier Storage format identifier (e.g., "mcap", "sqlite3")
 * @return Number of messages written, or -1 on error
 */
static int64_t merge_group(
  const std::vector<fs::path> & bag_files, const fs::path & output_path,
  const std::string & storage_identifier)
{
  // Collect topic union
  auto topic_union = collect_topic_union(bag_files);
  if (!topic_union.has_value()) {
    return -1;
  }

  // Remove stale temp directory from previous failed runs
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

  // Open writer
  rosbag2_storage::StorageOptions storage_options_out;
  storage_options_out.uri = temp_dir.string();
  storage_options_out.storage_id = storage_identifier;

  rosbag2_cpp::Writer writer;
  writer.open(storage_options_out);

  // Create all topics
  for (const auto & [topic_name, topic_meta] : topic_union.value()) {
    writer.create_topic(topic_meta);
  }

  // Open all readers and seed the min-heap
  std::vector<std::unique_ptr<rosbag2_cpp::Reader>> readers;
  std::priority_queue<HeapEntry, std::vector<HeapEntry>, HeapCompare> min_heap;

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

  // K-way merge loop
  int64_t message_count = 0;
  while (!min_heap.empty() && rclcpp::ok()) {
    auto entry = min_heap.top();
    min_heap.pop();

    writer.write(entry.message);
    ++message_count;

    // Log progress
    if (message_count % defaults::progress_log_interval == 0) {
      RCLCPP_INFO(g_logger, "  Merged %ld messages...", message_count);
    }

    // Read next message from the same reader
    auto & reader = readers[entry.reader_index];
    if (reader->has_next()) {
      auto next_msg = reader->read_next();
      min_heap.push({next_msg, entry.reader_index});
    }
  }

  if (!rclcpp::ok()) {
    RCLCPP_WARN(g_logger, "Interrupted by user (Ctrl+C), merge terminated early");
  }

  // Close writer
  writer.close();

  // Finalize output
  if (!finalize_output_bag(temp_dir, output_path)) {
    return -1;
  }

  return message_count;
}

static void print_merge_summary(const MergeResult & result)
{
  RCLCPP_INFO(g_logger, "========== Merge Summary ==========");
  RCLCPP_INFO(g_logger, "  Groups merged: %zu", result.merged_count);
  RCLCPP_INFO(g_logger, "  Groups skipped (single file): %zu", result.skipped_single_count);
  RCLCPP_INFO(g_logger, "  Files skipped (pattern mismatch): %zu", result.skipped_pattern_count);
  RCLCPP_INFO(g_logger, "  Groups failed: %zu", result.failed_count);
  RCLCPP_INFO(g_logger, "  Total messages merged: %zu", result.total_messages);

  if (result.deleted_files > 0) {
    RCLCPP_INFO(g_logger, "  Source files deleted: %zu", result.deleted_files);
  }

  if (!result.failed_groups.empty()) {
    RCLCPP_WARN(g_logger, "  Failed groups:");
    for (const auto & g : result.failed_groups) {
      RCLCPP_WARN(g_logger, "    - %s", g.c_str());
    }
  }

  RCLCPP_INFO(g_logger, "===================================");
}

int run_merge(const MergeConfig & config, MergeGroupProcessor processor)
{
  fs::path output_dir(config.output_dir);

  // Validate input directories
  for (const auto & input_dir_str : config.input_dirs) {
    fs::path input_dir(input_dir_str);
    if (!fs::is_directory(input_dir)) {
      RCLCPP_ERROR(g_logger, "Input path is not a directory: %s", input_dir.string().c_str());
      return 1;
    }
  }

  // Create output directory
  std::error_code ec;
  fs::create_directories(output_dir, ec);
  if (ec) {
    RCLCPP_ERROR(
      g_logger, "Failed to create output directory '%s': %s", output_dir.string().c_str(),
      ec.message().c_str());
    return 1;
  }

  // Find all bag files across all input directories
  std::vector<fs::path> all_bag_files;
  for (const auto & input_dir_str : config.input_dirs) {
    auto bag_files = find_bag_files(fs::path(input_dir_str));
    RCLCPP_INFO(g_logger, "Found %zu bag files in '%s'", bag_files.size(), input_dir_str.c_str());
    all_bag_files.insert(all_bag_files.end(), bag_files.begin(), bag_files.end());
  }

  if (all_bag_files.empty()) {
    RCLCPP_WARN(g_logger, "No bag files found in any input directory");
    return 0;
  }

  RCLCPP_INFO(g_logger, "Total bag files found: %zu", all_bag_files.size());

  // Parse filenames and group
  std::map<MergeGroupKey, std::vector<fs::path>> groups;
  // Track extension per group for output filename
  std::map<MergeGroupKey, std::string> group_extensions;
  MergeResult result;

  for (const auto & bag_path : all_bag_files) {
    std::string filename = bag_path.filename().string();
    auto parsed = parse_bag_filename(filename);

    if (!parsed.has_value()) {
      RCLCPP_WARN(g_logger, "File does not match expected pattern, skipping: %s", filename.c_str());
      ++result.skipped_pattern_count;
      continue;
    }

    MergeGroupKey key{parsed->sensing_system_id, parsed->rest};
    groups[key].push_back(bag_path);

    // Store extension from first file in group
    if (group_extensions.find(key) == group_extensions.end()) {
      group_extensions[key] = parsed->extension;
    }
  }

  RCLCPP_INFO(g_logger, "Identified %zu merge groups", groups.size());

  // Process each group
  for (const auto & [key, files] : groups) {
    if (!rclcpp::ok()) {
      RCLCPP_WARN(g_logger, "Interrupted by signal, stopping merge");
      break;
    }

    std::string group_name = key.sensing_system_id + "_" + key.rest;

    if (files.size() < 2) {
      RCLCPP_WARN(
        g_logger, "Skipping group '%s' (single file, nothing to merge)", group_name.c_str());
      ++result.skipped_single_count;
      continue;
    }

    // Determine output path
    std::string output_filename = group_name + group_extensions[key];
    fs::path output_path = output_dir / output_filename;

    if (fs::exists(output_path)) {
      RCLCPP_INFO(g_logger, "Removing existing output file: %s", output_path.string().c_str());
      fs::remove(output_path);
    }

    RCLCPP_INFO(g_logger, "Processing group '%s' (%zu files)...", group_name.c_str(), files.size());
    for (const auto & f : files) {
      RCLCPP_INFO(g_logger, "  - %s", f.filename().string().c_str());
    }

    // Get storage identifier from first file
    std::string storage_identifier;
    {
      rosbag2_storage::StorageOptions storage_options;
      storage_options.uri = files[0].string();
      rosbag2_cpp::Reader reader;
      reader.open(storage_options);
      storage_identifier = reader.get_metadata().storage_identifier;
    }

    int64_t msg_count;
    if (processor) {
      msg_count = processor(files, output_path, storage_identifier);
    } else {
      msg_count = merge_group(files, output_path, storage_identifier);
    }

    if (msg_count < 0) {
      RCLCPP_ERROR(g_logger, "Failed to process group '%s'", group_name.c_str());
      ++result.failed_count;
      result.failed_groups.push_back(group_name);
      continue;
    }

    RCLCPP_INFO(g_logger, "Processed %ld messages -> %s", msg_count, output_path.string().c_str());
    ++result.merged_count;
    result.total_messages += static_cast<size_t>(msg_count);

    // Delete source files if requested
    if (config.delete_sources) {
      for (const auto & f : files) {
        std::error_code del_ec;
        fs::remove(f, del_ec);
        if (del_ec) {
          RCLCPP_WARN(
            g_logger, "Failed to delete source file '%s': %s", f.string().c_str(),
            del_ec.message().c_str());
        } else {
          RCLCPP_INFO(g_logger, "  Deleted source: %s", f.filename().string().c_str());
          ++result.deleted_files;
        }
      }
    }
  }

  print_merge_summary(result);
  return result.failed_count > 0 ? 1 : 0;
}

}  // namespace merge
}  // namespace bag_converter
