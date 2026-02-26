/*
 *  Copyright (C) 2025 TierIV Inc.
 *
 *  License: Apache License
 *
 *  Merge subcommand: merge multiple rosbag2 files from distributed log modules
 */

#ifndef BAG_CONVERTER__MERGE_HPP
#define BAG_CONVERTER__MERGE_HPP

#include <optional>
#include <string>

namespace bag_converter
{
namespace merge
{

/**
 * @brief Configuration for the merge subcommand
 */
struct MergeConfig
{
  std::string input_dir;
  std::string output_dir;
  bool delete_after_merge = false;
};

/**
 * @brief Print usage information for the merge subcommand
 */
void print_merge_usage();

/**
 * @brief Parse merge subcommand arguments
 * @param argc Argument count
 * @param argv Argument values
 * @param config Output configuration
 * @return std::nullopt if parsing successful, exit code otherwise (0 for help, 1 for error)
 */
std::optional<int> parse_merge_arguments(int argc, char ** argv, MergeConfig & config);

/**
 * @brief Run the merge process
 * @param config The merge configuration
 * @return 0 on success, non-zero on failure
 */
int run_merge(const MergeConfig & config);

}  // namespace merge
}  // namespace bag_converter

#endif  // BAG_CONVERTER__MERGE_HPP
