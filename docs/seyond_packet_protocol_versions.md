# Seyond Packet Protocol Versions

This document describes the Seyond LiDAR packet protocol versions (v1 and v2), the structural differences between them, and how the SDK performs automatic conversion.

**Path note:** In this repository the Seyond SDK is provided by the `seyond_ros_driver` submodule at `src/dependencies/seyond_ros_driver/src/seyond_lidar_ros/src/seyond_sdk`. Paths below are relative to that SDK root (e.g. `src/sdk_client/...`).

## Background

Seyond LiDAR sensors (e.g. Falcon K) transmit raw UDP data using the **v1** protocol format. The Seyond client SDK automatically converts these packets to the **v2** format before delivering them to the application. This means:

- **NebulaPackets** (raw UDP capture): contains v1 packets (`major_version = 1`, `minor_version = 0`)
- **SeyondScan** (SDK-processed): contains v2 packets (`major_version = 2`, `minor_version = 1`)

Both represent the same point data; only the header format differs.

## Structural Difference: v1 vs v2

The only difference is a **16-byte reserved field** (`extend_reserved[4]`) inserted between the flags/ROI fields and the payload in v2.

### InnoDataPacketV1 (54 bytes header)

Defined in `src/sdk_client/inno_lidar_packet_v1_adapt.h`.

| Offset | Size | Field                                                     |
| ------ | ---- | --------------------------------------------------------- |
| 0      | 26B  | `common` (InnoCommonHeader)                               |
| 26     | 12B  | `idx`, `sub_idx`, `sub_seq`                               |
| 38     | 10B  | `type`, `item_number`, `item_size`, `topic`               |
| 48     | 2B   | Bitfield flags (scanner_direction, use_reflectance, etc.) |
| 50     | 4B   | `roi_h_angle`, `roi_v_angle`                              |
| 54     | —    | `payload[0]` (point data begins here)                     |

### InnoDataPacket v2 (70 bytes header)

Defined in `src/sdk_common/inno_lidar_packet.h`.

| Offset | Size    | Field                                                     |
| ------ | ------- | --------------------------------------------------------- |
| 0      | 26B     | `common` (InnoCommonHeader)                               |
| 26     | 12B     | `idx`, `sub_idx`, `sub_seq`                               |
| 38     | 10B     | `type`, `item_number`, `item_size`, `topic`               |
| 48     | 2B      | Bitfield flags (scanner_direction, use_reflectance, etc.) |
| 50     | 4B      | `roi_h_angle`, `roi_v_angle`                              |
| **54** | **16B** | **`extend_reserved[4]`** (inserted by SDK, zeroed)        |
| 70     | —       | `payload[0]` (point data begins here)                     |

The source code comments confirm this:

> "The output from the FalconK LiDAR does not include the extend_reserved field, which is inserted by the client SDK"
>
> — `inno_lidar_packet.h`, lines 845–846

## SDK Automatic v1 → v2 Conversion

The conversion is performed by `InnoPacketV1Adapt::check_data_packet_v1_and_convert_packet()` in `src/sdk_client/inno_lidar_packet_v1_adapt.h`.

### Steps

1. Detect v1 by checking `major_version == 1`
2. Shift the buffer pointer back by 16 bytes and `memmove` the v1 header
3. Zero-fill `extend_reserved[4]`
4. Update `common.size += 16`
5. Set `major_version = 2`, `minor_version = 1`
6. Recalculate CRC32

### Packet types subject to conversion

- `INNO_ITEM_TYPE_SPHERE_POINTCLOUD` — sphere-coordinate point cloud
- `INNO_ITEM_TYPE_XYZ_POINTCLOUD` — Cartesian point cloud
- `INNO_ITEM_TYPE_MESSAGE` / `INNO_ITEM_TYPE_MESSAGE_LOG` — log messages
- `INNO_FALCON_RING_ID_TABLE` — Falcon ring ID table
- `InnoStatusPacket` — status packets (separate conversion function)

## Protocol Version Constants

Defined in `src/sdk_common/inno_lidar_packet.h`:

| Constant                      | Value    | Description                        |
| ----------------------------- | -------- | ---------------------------------- |
| `kInnoMagicNumberDataPacket`  | `0x176A` | Magic number for data packets      |
| `kInnoMajorVersionDataPacket` | `4`      | Current SDK protocol major version |
| `kInnoMinorVersionDataPacket` | `0`      | Current SDK protocol minor version |

Defined in `src/sdk_client/inno_lidar_packet_v1_adapt.h`:

| Constant               | Value | Description                           |
| ---------------------- | ----- | ------------------------------------- |
| `kInnoProtocolMajorV1` | `1`   | v1 protocol major                     |
| `kInnoProtocolMajorV2` | `2`   | v2 protocol major (conversion target) |
| `kInnoProtocolMinorV2` | `1`   | v2 protocol minor (conversion target) |

## Connection-Time Version Negotiation

During TCP/HTTP connection setup (`src/utils/net_manager.cpp`), the SDK sends its protocol version to the LiDAR via HTTP headers:

```text
X-INNO-MAJOR-VERSION: 4
X-INNO-MINOR-VERSION: 0
```

The LiDAR responds with its own version. The SDK verifies compatibility in `InnoUtils::verify_lidar_version()` (`src/utils/utils.cpp`):

- If the LiDAR's major version exceeds the SDK's → error (SDK upgrade required)
- If the LiDAR's minor version exceeds the SDK's → warning

## Implications for bag_converter

When reading `pkt_version_major` / `pkt_version_minor` from rosbag data:

- **SeyondScan topics**: version is always v2 (`major = 2`, `minor = 1`) because the SDK has already converted the packets
- **NebulaPackets topics**: version reflects the raw LiDAR output (e.g. `major = 1`, `minor = 0` for Falcon K)

This version difference is expected and does not indicate a firmware mismatch or data corruption. The `extend_reserved` field is the only structural addition in v2, and it is always zeroed.

### bag_converter v1 compatibility

`NebulaPCDDecoder` ([nebula_decoder.cpp](../src/bag_converter/src/nebula_decoder.cpp)) applies its own v1 → v2 adaptation when processing NebulaPackets. It inserts 16 zero bytes at offset 54 and updates the packet size field, matching the approach used by nebula_drs `SeyondDecoder::ProtocolCompatibility()`. After adaptation, the packet is tagged as a `SeyondPacket` and delegated to `SeyondPCDDecoder` for decoding.

SeyondScan packets are always v2 (converted by the SDK before recording), so no v1 compatibility is needed for that path.
