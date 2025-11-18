#include "seyond_nebula_decoder/seyond_nebula_decoder.hpp"

#include <nebula_decoders/nebula_decoders_seyond/decoders/seyond_packet.hpp>
#include <nebula_decoders/nebula_decoders_seyond/seyond_driver.hpp>

#include <iomanip>
#include <iostream>

namespace seyond_nebula_decoder
{

SeyondNebulaDecoder::SeyondNebulaDecoder(const DecoderConfig & config) : config_(config)
{
  // Create sensor configuration
  sensor_config_ = std::make_shared<nebula::drivers::SeyondSensorConfiguration>();

  // Set basic network config (these won't be used for offline processing)
  sensor_config_->host_ip = "192.168.1.100";
  sensor_config_->sensor_ip = "192.168.1.201";
  sensor_config_->data_port = 2368;
  sensor_config_->gnss_port = 2369;
  sensor_config_->packet_mtu_size = 1500;

  // Set sensor model and return mode
  sensor_config_->sensor_model = nebula::drivers::SensorModelFromString(config.sensor_model);
  sensor_config_->return_mode = nebula::drivers::ReturnModeFromString(config.return_mode);

  // Set other parameters
  sensor_config_->frame_id = config.frame_id;
  sensor_config_->scan_phase = config.scan_phase;
  sensor_config_->frequency_ms = static_cast<uint16_t>(config.frequency_ms);
  sensor_config_->use_sensor_time = config.use_sensor_time;

  // Set min/max range
  sensor_config_->min_range = config.min_range;
  sensor_config_->max_range = config.max_range;

  // Create calibration configuration
  calibration_config_ = std::make_shared<nebula::drivers::SeyondCalibrationConfiguration>();

  // Load calibration from file if provided
  if (!config.calibration_file.empty()) {
    std::cerr << "Calibration file loading not yet implemented, using default calibration "
                 "settings..."
              << std::endl;
  }

  // Initialize the driver
  try {
    driver_ = std::make_shared<nebula::drivers::SeyondDriver>(sensor_config_, calibration_config_);
    status_ = nebula::Status::OK;

    // Check actual driver status
    auto driver_status = driver_->GetStatus();
    if (driver_status != nebula::Status::OK) {
      std::cerr << "Driver status is not OK (error code: " << driver_status << ")" << std::endl;
    }
  } catch (const std::exception & e) {
    std::cerr << "Failed to initialize driver: " << e.what() << "\n";
    status_ = nebula::Status::SENSOR_CONFIG_ERROR;
  }
}

SeyondNebulaDecoder::~SeyondNebulaDecoder() = default;

std::tuple<nebula::drivers::NebulaPointCloudPtr, bool> SeyondNebulaDecoder::ProcessPacket(
  const std::vector<uint8_t> & packet)
{
  if (status_ != nebula::Status::OK) {
    std::cerr << "Driver status not OK (error code: " << status_ << ")" << std::endl;
    return {nullptr, false};
  }

  // Process the packet - the driver returns a cloud when scan is complete
  auto [cloud, cloud_timestamp] = driver_->ParseCloudPacket(packet);

  // Check if a complete scan is available
  // (ParseCloudPacket returns non-null cloud when scan is complete)
  if (cloud != nullptr) {
    return {cloud, true};
  }
  return {nullptr, false};
}

nebula::drivers::NebulaPointCloudPtr SeyondNebulaDecoder::ProcessPackets(
  const std::vector<std::vector<uint8_t>> & packets)
{
  nebula::drivers::NebulaPointCloudPtr complete_cloud;
  double scan_timestamp_s = 0.0;

  for (const auto & packet : packets) {
    auto [cloud, cloud_timestamp] = driver_->ParseCloudPacket(packet);

    if (cloud && !cloud->empty()) {
      // A complete scan was returned
      complete_cloud = cloud;
      scan_timestamp_s = cloud_timestamp;  // Capture timestamp parsed from packet data
    }
  }

  // Set the timestamp in the point cloud header
  if (complete_cloud && scan_timestamp_s > 0.0) {
    // Convert seconds to microseconds (PCL header.stamp is in microseconds)
    complete_cloud->header.stamp = static_cast<uint64_t>(scan_timestamp_s * 1e6);
  }

  // Return the last complete cloud if available
  return complete_cloud;
}

nebula::drivers::NebulaPointCloudPtr SeyondNebulaDecoder::ConvertNebulaPackets(
  const nebula_msgs::msg::NebulaPackets & packets)
{
  std::vector<std::vector<uint8_t>> packet_data_vec;

  // Extract packet data from nebula_msgs
  for (const auto & packet : packets.packets) {
    packet_data_vec.push_back(packet.data);
  }

  // Process all packets
  return ProcessPackets(packet_data_vec);
}

void SeyondNebulaDecoder::ReinitializeDriver()
{
  // Reinitialize the driver with the same configuration
  try {
    driver_ = std::make_shared<nebula::drivers::SeyondDriver>(sensor_config_, calibration_config_);
    status_ = driver_->GetStatus();
  } catch (const std::exception & e) {
    std::cerr << "Failed to reinitialize driver: " << e.what() << "\n";
    status_ = nebula::Status::SENSOR_CONFIG_ERROR;
  }
}

void SeyondNebulaDecoder::SetCalibrationData(const std::vector<uint8_t> & calibration_data)
{
  // For RobinW, the calibration packet contains the full AngleHV table
  // We need to pass this to the driver in the format it expects

  // Convert vector to string (the driver expects this format)
  std::string calib_string(calibration_data.begin(), calibration_data.end());

  // Create new calibration configuration
  auto new_calibration_config = std::make_shared<nebula::drivers::SeyondCalibrationConfiguration>();

  // Load the calibration string directly - the driver will parse it
  new_calibration_config->LoadFromString(calib_string);

  // Store the calibration configuration
  calibration_config_ = new_calibration_config;

  // Reinitialize driver with new calibration
  ReinitializeDriver();

  std::cout << "Calibration data set (" << calibration_data.size() << " bytes)\n";
}

void SeyondNebulaDecoder::SetConfig(const DecoderConfig & config)
{
  config_ = config;

  // Update sensor configuration
  sensor_config_->sensor_model = nebula::drivers::SensorModelFromString(config.sensor_model);
  sensor_config_->return_mode = nebula::drivers::ReturnModeFromString(config.return_mode);
  sensor_config_->frame_id = config.frame_id;
  sensor_config_->scan_phase = config.scan_phase;
  sensor_config_->frequency_ms = static_cast<uint16_t>(config.frequency_ms);
  sensor_config_->use_sensor_time = config.use_sensor_time;
  sensor_config_->min_range = config.min_range;
  sensor_config_->max_range = config.max_range;

  // Reinitialize driver with new configuration
  ReinitializeDriver();
}

}  // namespace seyond_nebula_decoder