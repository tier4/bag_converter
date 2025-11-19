# Seyond Nebula Decoder

A library for decoding Nebula packets from Seyond LiDAR sensors and converting them to point clouds.

## Overview

This library provides functionality to:
- Decode raw packet data from Seyond LiDAR sensors
- Convert the decoded data to PCL point clouds
- Support various Seyond sensor models (Falcon Kinetic, Robin W, etc.)

## Features

- Packet-to-pointcloud conversion
- Support for multiple Seyond sensor models
- Configurable return modes (Single, Dual)
- Range filtering (min/max range)
- Calibration data support
- ROS-independent library (can be used in any C++ application)

## Installation

```bash
# Build the package
cd ~/ros2_ws
colcon build --packages-select seyond_nebula_decoder
```

## Usage as a Library

```cpp
#include <seyond_nebula_decoder/seyond_nebula_decoder.hpp>

// Create decoder with configuration
seyond_nebula_decoder::DecoderConfig config;
config.sensor_model = "Falcon";
config.return_mode = "Dual";
config.min_range = 0.1;
config.max_range = 300.0;

auto decoder = std::make_unique<seyond_nebula_decoder::SeyondNebulaDecoder>(config);

// Decode single packet
std::vector<uint8_t> packet_data = ...;  // Raw packet data
uint64_t timestamp_ns = ...;  // Timestamp in nanoseconds

auto [point_cloud, scan_complete] = decoder->DecodePacket(packet_data, timestamp_ns);

if (scan_complete && point_cloud) {
    // Process complete point cloud
    std::cout << "Point cloud with " << point_cloud->size() << " points" << std::endl;
}

// Or decode multiple packets at once
std::vector<std::vector<uint8_t>> packets = ...;
auto complete_cloud = decoder->DecodePackets(packets);

// Using NebulaPackets structure
seyond_nebula_decoder::NebulaPackets nebula_packets;
nebula_packets.frame_id = "lidar";
nebula_packets.timestamp_ns = ...;
// Fill packets...

auto cloud = decoder->DecodePackets(nebula_packets);
```

## Configuration

The `DecoderConfig` structure supports the following parameters:

- `sensor_model` (string, default: "Falcon"): Sensor model
- `return_mode` (string, default: "Dual"): Return mode (Single/Dual)
- `frame_id` (string, default: "seyond"): Frame ID for identification
- `scan_phase` (double, default: 0.0): Scan phase offset
- `frequency_ms` (double, default: 100.0): Scan frequency in milliseconds
- `use_sensor_time` (bool, default: true): Use sensor timestamps
- `calibration_file` (string, default: ""): Path to calibration file
- `min_range` (double, default: 0.1): Minimum range in meters
- `max_range` (double, default: 300.0): Maximum range in meters

## API Reference

### Main Classes

#### `SeyondNebulaDecoder`

Main decoder class for processing Seyond LiDAR packets.

**Methods:**
- `DecodePacket(packet_data, timestamp_ns)`: Decode a single packet
- `DecodePackets(packets)`: Decode multiple packets
- `Reset()`: Reset internal state
- `SetCalibrationData(data)`: Set calibration data
- `UpdateConfig(config)`: Update decoder configuration
- `GetStatus()`: Get current decoder status
- `GetPacketsProcessed()`: Get number of packets processed
- `GetScansCompleted()`: Get number of scans completed

## Dependencies

- PCL (Point Cloud Library)
- Eigen3
- C++17

## Note

This is a stub implementation that provides the interface for Seyond Nebula packet decoding.
The actual nebula driver integration would require the nebula_common and nebula_decoders packages.

## License

Apache License 2.0