# Robin W Intensity Range and Packet Version

## Problem

Robin W LiDAR with packet protocol major version <= 3 reports intensity values in the range **[0, 4095]** (12-bit), while all other Seyond models and Robin W with protocol version >= 4 use **[0, 255]** (8-bit).

bag_converter normalizes Robin W intensity to [0, 255] when the protocol version indicates 12-bit values.

## Scaling Condition

Intensity scaling is applied when **both** conditions are met:

- `lidar_type == RobinW` (value `1` in `InnoLidarType` / `SeyondLidarType`)
- `pkt_version_major <= 3`

## Scaling Formula

```cpp
intensity = intensity * (255.0f / 4095.0f);
```

No clamping is applied so that out-of-range values remain visible for debugging.

## Where Scaling is Applied

### seyond_decoder (SeyondScan input)

In [seyond_decoder.cpp](../src/bag_converter/src/seyond_decoder.cpp):

- `data_packet_parse()` reads `lidar_type` and `major_version` from each `InnoDataPacket` header and sets the `scale_intensity_12bit_` flag.
- `point_xyz_data_parse()` applies scaling immediately after setting `point.intensity`.

### nebula_decoder (NebulaPackets input)

In [nebula_decoder.cpp](../src/bag_converter/src/nebula_decoder.cpp):

- `extract_packet_meta()` reads `lidar_type` and `major_version` from the first valid data packet.
- `decode_typed()` checks `PacketMeta::needs_intensity_scaling()` and applies scaling when converting each point.

## Protocol Version Reference

See [seyond_packet_protocol_versions.md](seyond_packet_protocol_versions.md) for details on Seyond packet protocol versions and the SDK's automatic v1-to-v2 conversion.

Key version values:

| Source         | major | minor | Intensity range |
| -------------- | ----- | ----- | --------------- |
| NebulaPackets  | 1     | 0     | [0, 4095]       |
| SeyondScan     | 2     | 1     | [0, 4095]       |
| Protocol >= v4 | 4     | 0     | [0, 255]        |

Note: NebulaPackets contains raw UDP packets (v1), while SeyondScan contains SDK-processed packets (v2). Both report the same 12-bit intensity for Robin W with firmware that uses protocol <= v3. The version difference between them is due to the SDK's automatic v1-to-v2 header conversion (see linked document).
