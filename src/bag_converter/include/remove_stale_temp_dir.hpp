/*
 *  Copyright (C) 2025 TierIV Inc.
 *
 *  License: Apache License
 *
 *  Remove leftover writer temp directory (output_path + "_tmp") from a failed run.
 */

#ifndef BAG_CONVERTER__REMOVE_STALE_TEMP_DIR_HPP
#define BAG_CONVERTER__REMOVE_STALE_TEMP_DIR_HPP

#include <rclcpp/rclcpp.hpp>

#include <filesystem>
#include <string>
#include <system_error>

namespace bag_converter
{

/** If `output_path + "_tmp"` exists, remove it. Returns false if removal fails. */
inline bool remove_stale_temp_dir_if_exists(
  const std::filesystem::path & output_path, const rclcpp::Logger & logger)
{
  const std::filesystem::path temp_dir = output_path.string() + "_tmp";
  std::error_code ec;
  if (!std::filesystem::exists(temp_dir)) {
    return true;
  }
  RCLCPP_WARN(
    logger, "Removing stale temp directory from previous run: %s", temp_dir.string().c_str());
  std::filesystem::remove_all(temp_dir, ec);
  if (ec) {
    RCLCPP_ERROR(
      logger, "Failed to remove stale temp directory '%s': %s", temp_dir.string().c_str(),
      ec.message().c_str());
    return false;
  }
  return true;
}

}  // namespace bag_converter

#endif  // BAG_CONVERTER__REMOVE_STALE_TEMP_DIR_HPP
