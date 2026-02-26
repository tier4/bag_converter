/*
 *  Copyright (C) 2025 TierIV Inc.
 *
 *  License: Apache License
 *
 *  Merge: merge multiple rosbag2 files from distributed log modules
 */

#ifndef BAG_CONVERTER__MERGE_HPP
#define BAG_CONVERTER__MERGE_HPP

#include <rosbag2_storage/serialized_bag_message.hpp>
#include <rosbag2_storage/topic_metadata.hpp>

#include <filesystem>
#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace bag_converter
{
namespace merge
{

/**
 * @brief Configuration for the merge operation
 */
struct MergeConfig
{
  std::vector<std::string> input_dirs;
  std::string output_dir;
  bool delete_sources = false;
};

/**
 * @brief Entry in the min-heap for k-way merge
 */
struct HeapEntry
{
  std::shared_ptr<rosbag2_storage::SerializedBagMessage> message;
  size_t reader_index;
};

/**
 * @brief Comparator for min-heap (smallest timestamp first)
 */
struct HeapCompare
{
  bool operator()(const HeapEntry & a, const HeapEntry & b) const
  {
    return a.message->time_stamp > b.message->time_stamp;
  }
};

/**
 * @brief Function type for processing a single merge group
 * @param bag_files The bag files in this group
 * @param output_path The output file path
 * @param storage_identifier Storage format (e.g., "mcap", "sqlite3")
 * @return Number of messages processed, or -1 on error
 */
using MergeGroupProcessor = std::function<int64_t(
  const std::vector<std::filesystem::path> & bag_files, const std::filesystem::path & output_path,
  const std::string & storage_identifier)>;

/**
 * @brief Collect the union of all topics from multiple bag files
 * @param bag_files List of bag file paths
 * @return Map of topic name to TopicMetadata, or nullopt if duplicate topics found across bags
 */
std::optional<std::map<std::string, rosbag2_storage::TopicMetadata>> collect_topic_union(
  const std::vector<std::filesystem::path> & bag_files);

/**
 * @brief Run the merge process
 * @param config The merge configuration
 * @param processor Optional custom group processor. If nullptr, uses default k-way merge.
 * @return 0 on success, non-zero on failure
 */
int run_merge(const MergeConfig & config, MergeGroupProcessor processor = nullptr);

}  // namespace merge
}  // namespace bag_converter

#endif  // BAG_CONVERTER__MERGE_HPP
