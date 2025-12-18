/*
 *  Copyright (C) 2025 TierIV Inc.
 *
 *  License: Apache License
 *
 *  Implementation of timescale correction utilities
 */

#include "timescale.hpp"

namespace bag_converter::timescale
{

std::uint64_t correct_timescale(
  std::uint64_t ref_time_ns, std::uint64_t time_ns_to_correct, const std::string & ref_timescale)
{
  const double ref_time_sec = static_cast<double>(ref_time_ns) / 1e9;
  const double time_sec_to_correct = static_cast<double>(time_ns_to_correct) / 1e9;
  const auto diff = ref_time_sec - time_sec_to_correct;

  if (ref_timescale == "utc") {
    // ref: UTC, to-correct: TAI
    if (
      diff > (-offsets::utc_to_tai_sec - offsets::correction_tolerance_sec) &&
      diff < (-offsets::utc_to_tai_sec + offsets::correction_tolerance_sec)) {
      return time_ns_to_correct - static_cast<std::uint64_t>(offsets::utc_to_tai_sec * 1e9);
    }
    // ref: UTC, to-correct: GPS
    if (
      diff > (-offsets::utc_to_gps_sec - offsets::correction_tolerance_sec) &&
      diff < (-offsets::utc_to_gps_sec + offsets::correction_tolerance_sec)) {
      return time_ns_to_correct - static_cast<std::uint64_t>(offsets::utc_to_gps_sec * 1e9);
    }
  } else if (ref_timescale == "tai") {
    // ref: TAI, to-correct: UTC
    if (
      diff > (offsets::utc_to_tai_sec - offsets::correction_tolerance_sec) &&
      diff < (offsets::utc_to_tai_sec + offsets::correction_tolerance_sec)) {
      return time_ns_to_correct + static_cast<std::uint64_t>(offsets::utc_to_tai_sec * 1e9);
    }
    // ref: TAI, to-correct: GPS
    if (
      diff > (offsets::gps_to_tai_sec - offsets::correction_tolerance_sec) &&
      diff < (offsets::gps_to_tai_sec + offsets::correction_tolerance_sec)) {
      return time_ns_to_correct + static_cast<std::uint64_t>(offsets::gps_to_tai_sec * 1e9);
    }
  } else if (ref_timescale == "gps") {
    // ref: GPS, to-correct: UTC
    if (
      diff > (offsets::utc_to_gps_sec - offsets::correction_tolerance_sec) &&
      diff < (offsets::utc_to_gps_sec + offsets::correction_tolerance_sec)) {
      return time_ns_to_correct + static_cast<std::uint64_t>(offsets::utc_to_gps_sec * 1e9);
    }
    // ref: GPS, to-correct: TAI
    if (
      diff > (-offsets::gps_to_tai_sec - offsets::correction_tolerance_sec) &&
      diff < (-offsets::gps_to_tai_sec + offsets::correction_tolerance_sec)) {
      return time_ns_to_correct - static_cast<std::uint64_t>(offsets::gps_to_tai_sec * 1e9);
    }
  }

  // No correction needed or timescale not recognized
  return time_ns_to_correct;
}

}  // namespace bag_converter::timescale
