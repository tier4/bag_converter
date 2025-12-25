/*
 *  Copyright (C) 2025 TierIV Inc.
 *
 *  License: Apache License
 *
 *  Implementation of timescale correction utilities
 */

#include "timescale.hpp"

#include <stdexcept>

namespace bag_converter::timescale
{

std::uint64_t correct_timescale(
  std::uint64_t target_timestamp, std::uint64_t ref_timestamp, const std::string & ref_timescale)
{
  const double ref_sec = static_cast<double>(ref_timestamp) / 1e9;
  const double target_sec = static_cast<double>(target_timestamp) / 1e9;
  const auto diff = ref_sec - target_sec;
  const double tol = offsets::correction_tolerance_sec;

  if (ref_timescale == "utc") {
    // ref: UTC, to-correct: TAI
    if (diff > (-offsets::utc_to_tai_sec - tol) && diff < (-offsets::utc_to_tai_sec + tol)) {
      return target_timestamp - static_cast<std::uint64_t>(offsets::utc_to_tai_sec * 1e9);
    }
    // ref: UTC, to-correct: GPS
    if (diff > (-offsets::utc_to_gps_sec - tol) && diff < (-offsets::utc_to_gps_sec + tol)) {
      return target_timestamp - static_cast<std::uint64_t>(offsets::utc_to_gps_sec * 1e9);
    }
  } else if (ref_timescale == "tai") {
    // ref: TAI, to-correct: UTC
    if (diff > (offsets::utc_to_tai_sec - tol) && diff < (offsets::utc_to_tai_sec + tol)) {
      return target_timestamp + static_cast<std::uint64_t>(offsets::utc_to_tai_sec * 1e9);
    }
    // ref: TAI, to-correct: GPS
    if (diff > (offsets::gps_to_tai_sec - tol) && diff < (offsets::gps_to_tai_sec + tol)) {
      return target_timestamp + static_cast<std::uint64_t>(offsets::gps_to_tai_sec * 1e9);
    }
  } else if (ref_timescale == "gps") {
    // ref: GPS, to-correct: UTC
    if (diff > (offsets::utc_to_gps_sec - tol) && diff < (offsets::utc_to_gps_sec + tol)) {
      return target_timestamp + static_cast<std::uint64_t>(offsets::utc_to_gps_sec * 1e9);
    }
    // ref: GPS, to-correct: TAI
    if (diff > (-offsets::gps_to_tai_sec - tol) && diff < (-offsets::gps_to_tai_sec + tol)) {
      return target_timestamp - static_cast<std::uint64_t>(offsets::gps_to_tai_sec * 1e9);
    }
  } else {
    // R"(...)" is a raw string literal (C++11), allowing quotes without escaping
    throw std::invalid_argument(
      R"(ref_timescale must be "utc", "tai", or "gps", but got: )" + ref_timescale);
  }

  // No correction needed
  return target_timestamp;
}

}  // namespace bag_converter::timescale
