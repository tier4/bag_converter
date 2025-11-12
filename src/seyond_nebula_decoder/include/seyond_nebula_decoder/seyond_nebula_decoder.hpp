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

// Configuration for the decoder
struct DecoderConfig
{
  // Sensor configuration
  std::string sensor_model = "Falcon_Kinetic";  // or "Robin_W"
  std::string return_mode = "Dual";
  std::string frame_id = "seyond";

  // Range filtering (will be set in sensor config)
  double min_range = 0.3;
  double max_range = 200.0;

  // Processing options
  double scan_phase = 0.0;
  double frequency_ms = 100.0;
  bool use_sensor_time = true;

  // Calibration
  std::string calibration_file = "";
};

/// @brief Decoder class using actual nebula driver like seyond_rosbag_to_pcd
class SeyondNebulaDecoder
{
public:
  /// @brief Constructor with configuration
  explicit SeyondNebulaDecoder(const DecoderConfig & config = DecoderConfig());
  ~SeyondNebulaDecoder();

  /// @brief Process single packet (for streaming processing) - main interface
  /// @param packet_data Raw packet bytes
  /// @param timestamp Timestamp in seconds (not used by driver)
  /// @return Tuple of (complete_cloud, scan_complete_flag)
  std::tuple<nebula::drivers::NebulaPointCloudPtr, bool> ProcessPacket(
    const std::vector<uint8_t> & packet_data, double timestamp = 0.0);

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

  /// @brief Reset decoder state
  void Reset();

  /// @brief Reinitialize driver (useful for error recovery)
  void ReinitializeDriver();

  /// @brief Set calibration data
  void SetCalibrationData(const std::vector<uint8_t> & calibration_data);

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

  // Buffer for partial packets if needed
  std::vector<std::vector<uint8_t>> packet_buffer_;

  // Debug logging
  bool first_packet_logged_ = false;
  int calibration_packets_seen_ = 0;

  // Calibration data storage (for RobinW)
  bool has_calibration_from_packet_ = false;
  std::vector<uint8_t> stored_calibration_packet_;
};

}  // namespace seyond_nebula_decoder

#endif  // SEYOND_NEBULA_DECODER_HPP