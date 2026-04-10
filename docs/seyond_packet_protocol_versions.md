# Seyond Packet Protocol Versions

This document describes the Seyond LiDAR packet protocol versions (v1 and v2), the structural differences between them, and how the SDK performs automatic conversion.

**Path note:** In this repository the Seyond SDK is provided by the `seyond_ros_driver` submodule at `src/dependencies/seyond_ros_driver/src/seyond_lidar_ros/src/seyond_sdk`. Paths below are relative to that SDK root (e.g. `src/sdk_client/...`).

## Background

Seyond LiDAR sensors (e.g. Falcon K) transmit raw UDP data using the **v1** protocol format. The Seyond client SDK automatically converts these packets to the **v2** format before delivering them to the application. This means:

- **NebulaPackets** (raw UDP capture): often **v1** in the header (`major_version = 1`, `minor_version = 0`)
- **SeyondScan** (SDK-processed): typically **v2** (`major_version = 2`, `minor_version = 1`)

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
> — `inno_lidar_packet.h` (around `extend_reserved`, lines 808–810 in the bundled SDK)

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

| Constant               | Value | Description                                         |
| ---------------------- | ----- | --------------------------------------------------- |
| `kInnoProtocolMajorV1` | `1`   | v1 protocol major                                   |
| `kInnoProtocolMajorV2` | `2`   | v2 protocol major (conversion target)               |
| `kInnoProtocolMinorV2` | `1`   | v2 protocol minor (conversion target)               |
| `kMemorryFrontGap`     | 16\*  | `sizeof(InnoDataPacket) - sizeof(InnoDataPacketV1)` |

\*With the bundled SDK structs, this gap is **16 bytes** (v1 header ends at offset 54; v2 inserts `extend_reserved` before `payload`).

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

Recorded packets may show **v1 or v2** in `InnoCommonHeader.version` depending on whether data was captured raw or passed through the SDK first. That difference alone does not indicate corruption. The `extend_reserved` region is the structural addition in v2 (zeroed when inserted).

Intensity scaling for **Robin W** uses `lidar_type` and packet `major_version`; see [seyond_intensity_scaling.md](seyond_intensity_scaling.md).

### bag_converter v1 compatibility

<!-- AUTO-GENERATED: keep in sync with nebula_packets_to_seyond_scan in nebula_decoder.cpp -->

`NebulaPCDDecoder` builds a `SeyondScan` via `nebula_packets_to_seyond_scan` ([nebula_decoder.cpp](../src/bag_converter/src/nebula_decoder.cpp)) and decodes with `SeyondPCDDecoder`.

For each payload with magic `kInnoMagicNumberDataPacket`, if the buffer is at least `sizeof(InnoDataPacketV1)` and `common.version.major_version == InnoPacketV1Adapt::kInnoProtocolMajorV1`, the code copies the bytes, inserts **`InnoPacketV1Adapt::kMemorryFrontGap`** zero bytes immediately after the v1 header (at offset `sizeof(InnoDataPacketV1)`), adds **`kMemorryFrontGap`** to `common.size`, and sets **`major_version`** / **`minor_version`** to **`kInnoProtocolMajorV2`** / **`kInnoProtocolMinorV2`**. This matches the layout gap defined in the SDK (`kMemorryFrontGap` = `sizeof(InnoDataPacket) - sizeof(InnoDataPacketV1)`). It does **not** run the full SDK routine (e.g. CRC32 update in `InnoPacketV1Adapt`).

The first `NebulaPackets` entry may be a **Robin W AngleHV** calibration blob without a valid `InnoDataPacket` header; when detected, the code repairs the header so downstream decoding treats it as `INNO_ROBINW_ITEM_TYPE_ANGLEHV_TABLE`.

Paths that already carry v2-shaped headers are copied through without insertion.

<!-- END AUTO-GENERATED -->

SeyondScan inputs are normally already v2 from the SDK, so this adaptation is only needed for raw-style captures.
