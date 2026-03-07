# en_xyzit point type

The `en_xyzit` point type is an **experimental** extension of the default PointCloud2 output. Enable it with `--point-type en_xyzit`. It adds the same normal fields as `xyzit` (x, y, z, intensity, timestamp) plus extended properties for reflection classification, elongation, lidar status/mode, packet version, and lidar type.

**Subject to change.** This document is the canonical reference for the layout and semantics of `en_xyzit`.

## Extended fields overview

All extended fields are supported for both SeyondScan and NebulaPackets input. The `NebulaPCDDecoder` adapts NebulaPackets to `SeyondScan` format and delegates to the same `SeyondPCDDecoder`, so the output is identical regardless of input type.

| Property            | Type   | Description / values                                            |
| ------------------- | ------ | --------------------------------------------------------------- |
| `flags`             | uint16 | Availability mask; see below                                    |
| `refl_type`         | uint8  | Point classification: 0 = normal, 1 = ground, 2 = fog           |
| `elongation`        | uint8  | Raw elongation 0–15                                             |
| `lidar_status`      | uint8  | 0 = none, 1 = transition, 2 = normal, 3 = failed                |
| `lidar_mode`        | uint8  | 1 = sleep, 2 = standby, 3 = work_normal, 6 = protection         |
| `pkt_version_major` | uint8  | Packet protocol major (0–255)                                   |
| `pkt_version_minor` | uint8  | Packet protocol minor (0–255)                                   |
| `lidar_type`        | uint8  | Seyond LiDAR model; see [LiDAR type values](#lidar-type-values) |

Note: `refl_type` and `elongation` are only available when the packet contains `InnoXyzPoint` data (as opposed to `InnoEnXyzPoint`). The `flags` mask indicates which fields are valid for each point.

## Availability flags and extended layout

### Semantics

- **Availability mask** (`flags`, `uint16_t`): one bit per extended property.
  - Bit set ⇒ the corresponding value is valid.
  - Bit clear ⇒ the value is **0** and must be ignored.
- **Values**: unsigned types with the smallest range that fits; no sentinel.

Decoder: set the bit when the property is filled; leave clear when not available. Consumer: test the bit before using the value.

### Layout (extended part)

| Field             | Type     | Size  | Value range    | Flag bit |
| ----------------- | -------- | ----- | -------------- | -------- |
| flags             | uint16_t | 2     | bits 0–6 used  | —        |
| refl_type         | uint8_t  | 1     | 0, 1, 2        | 0        |
| elongation        | uint8_t  | 1     | 0–15           | 1        |
| lidar_status      | uint8_t  | 1     | 0–3            | 2        |
| lidar_mode        | uint8_t  | 1     | 0–9 (e.g. 1–6) | 3        |
| pkt_version_major | uint8_t  | 1     | 0–255          | 4        |
| pkt_version_minor | uint8_t  | 1     | 0–255          | 5        |
| lidar_type        | uint8_t  | 1     | 0–7            | 6        |
| **Total**         |          | **9** |                |          |

Bits 7–15 of `flags` are reserved.

### Flag constants (C++)

In this codebase the constants are in `bag_converter::point::en_xyzit_flags`:

```cpp
namespace en_xyzit_flags {
  constexpr uint16_t HAS_REFL_TYPE        = 1u << 0;
  constexpr uint16_t HAS_ELONGATION       = 1u << 1;
  constexpr uint16_t HAS_LIDAR_STATUS    = 1u << 2;
  constexpr uint16_t HAS_LIDAR_MODE       = 1u << 3;
  constexpr uint16_t HAS_PKT_VERSION_MAJOR = 1u << 4;
  constexpr uint16_t HAS_PKT_VERSION_MINOR = 1u << 5;
  constexpr uint16_t HAS_LIDAR_TYPE       = 1u << 6;
  // bits 7–15 reserved
}
```

## LiDAR type values

The `lidar_type` field is populated for Seyond LiDAR. When not supported (e.g. other sensors), the value is **0** and must be ignored; check the `HAS_LIDAR_TYPE` bit in `flags` to know if the value is valid.

| Value | Name / alias         | Description           |
| ----- | -------------------- | --------------------- |
| 0     | Falcon (FalconK1)    | Falcon K1             |
| 1     | RobinW               | Robin W               |
| 2     | RobinE               | Robin E               |
| 3     | Falcon2.1 / FalconK2 | Falcon 2.1, Falcon K2 |
| 4     | FalconIII            | Falcon III            |
| 5     | RobinELite           | Robin E Lite          |
| 6     | RobinE2              | Robin E2              |
| 7     | HB                   | Hummingbird           |
