#ifndef SEYOND_NEBULA_DECODER_HPP
#define SEYOND_NEBULA_DECODER_HPP

#include <nebula_common/nebula_common.hpp>
#include <nebula_common/nebula_status.hpp>
#include <nebula_common/seyond/seyond_common.hpp>

// Forward declarations to avoid include issues
namespace nebula
{
namespace drivers
{
class SeyondDriver;
class SeyondSensorConfiguration;
class SeyondCalibrationConfiguration;
}  // namespace drivers
}  // namespace nebula

#include <nebula_msgs/msg/nebula_packet.hpp>
#include <nebula_msgs/msg/nebula_packets.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <cstdint>
#include <memory>
#include <string>
#include <tuple>
#include <vector>

namespace seyond_nebula_decoder
{

// Default configuration constants
namespace defaults
{
inline constexpr const char * sensor_model = "Falcon";
inline constexpr const char * return_mode = "Dual";
inline constexpr const char * frame_id = "seyond";
inline constexpr double min_range = 0.3;
inline constexpr double max_range = 200.0;
inline constexpr double scan_phase = 0.0;
inline constexpr double frequency_ms = 100.0;
inline constexpr bool use_sensor_time = true;
inline constexpr const char * calibration_file = "";
}  // namespace defaults

// Configuration for the decoder
struct DecoderConfig
{
  // Sensor configuration
  std::string sensor_model = defaults::sensor_model;  // or "Robin_W"
  std::string return_mode = defaults::return_mode;
  std::string frame_id = defaults::frame_id;

  // Range filtering (will be set in sensor config)
  double min_range = defaults::min_range;
  double max_range = defaults::max_range;

  // Processing options
  double scan_phase = defaults::scan_phase;
  double frequency_ms = defaults::frequency_ms;
  bool use_sensor_time = defaults::use_sensor_time;

  // Calibration
  std::string calibration_file = defaults::calibration_file;
};

/// @brief Decoder class using actual nebula driver like seyond_rosbag_to_pcd
class SeyondNebulaDecoder
{
public:
  /// @brief Constructor with configuration
  explicit SeyondNebulaDecoder(const DecoderConfig & config = DecoderConfig());
  ~SeyondNebulaDecoder();

  /// @brief Process single packet (for streaming processing) - main interface
  /// @param packet Raw packet bytes
  /// @return Tuple of (complete_cloud, scan_complete_flag)
  std::tuple<nebula::drivers::NebulaPointCloudPtr, bool> ProcessPacket(
    const std::vector<uint8_t> & packet);

  /// @brief Process multiple packets at once
  /// @param packets Vector of packet data
  /// @return Complete point cloud if available
  nebula::drivers::NebulaPointCloudPtr ProcessPackets(
    const std::vector<std::vector<uint8_t>> & packets);

  /// @brief Convert NebulaPackets message to point cloud
  /// @param packets Nebula packets container
  /// @return Nebula format point cloud
  nebula::drivers::NebulaPointCloudPtr ConvertNebulaPackets(
    const nebula_msgs::msg::NebulaPackets & packets);

  /// @brief Get current status
  nebula::Status GetStatus() const { return status_; }

  /// @brief Get/set configuration
  DecoderConfig GetConfig() const { return config_; }
  void SetConfig(const DecoderConfig & config);

private:
  DecoderConfig config_;
  nebula::Status status_;

  // Nebula driver components (like seyond_rosbag_to_pcd)
  std::shared_ptr<nebula::drivers::SeyondDriver> driver_;
  std::shared_ptr<nebula::drivers::SeyondSensorConfiguration> sensor_config_;
  std::shared_ptr<nebula::drivers::SeyondCalibrationConfiguration> calibration_config_;
};

}  // namespace seyond_nebula_decoder

#endif  // SEYOND_NEBULA_DECODER_HPP