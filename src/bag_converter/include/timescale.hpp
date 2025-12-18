/*
 *  Copyright (C) 2025 TierIV Inc.
 *
 *  License: Apache License
 *
 *  Timescale correction utilities for converting between UTC, TAI, and GPS time
 */

#ifndef BAG_CONVERTER__TIMESCALE_HPP
#define BAG_CONVERTER__TIMESCALE_HPP

#include <cstdint>
#include <string>

namespace bag_converter::timescale
{

// Time offset constants between different timescales
namespace offsets
{
inline constexpr double utc_to_tai_sec = 37.0;  // UTC to TAI offset (leap seconds as of 2024)
inline constexpr double utc_to_gps_sec = 18.0;  // UTC to GPS offset
inline constexpr double gps_to_tai_sec = 19.0;  // GPS to TAI offset
inline constexpr double correction_tolerance_sec = 0.35;  // Tolerance for detecting timescale
}  // namespace offsets

/**
 * @brief Correct timestamp from one timescale to match the reference timescale
 *
 * This function detects the timescale of the input timestamp by comparing it with
 * a reference timestamp and applies the appropriate correction.
 *
 * @param ref_time_ns Reference timestamp in nanoseconds (known timescale)
 * @param time_ns_to_correct Timestamp to correct in nanoseconds (unknown timescale)
 * @param ref_timescale The timescale of the reference timestamp ("utc", "tai", or "gps")
 * @return Corrected timestamp in nanoseconds, aligned to the reference timescale
 */
std::uint64_t correct_timescale(
  std::uint64_t ref_time_ns, std::uint64_t time_ns_to_correct,
  const std::string & ref_timescale = "utc");

}  // namespace bag_converter::timescale

#endif  // BAG_CONVERTER__TIMESCALE_HPP
