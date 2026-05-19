# en_xyzit point type

The `en_xyzit` point type is an **experimental** extension of the default PointCloud2 output. Enable it with `--point-type en_xyzit`. It adds the same normal fields as `xyzit` (x, y, z, intensity, time_stamp) plus extended properties for reflection classification, elongation, lidar status/mode, packet version, lidar type, return information, and intensity-field semantics.

**Subject to change.** This document is the canonical reference for the layout and semantics of `en_xyzit`.

## Extended fields overview

All extended fields are supported for both SeyondScan and NebulaPackets input. The `NebulaPCDDecoder` adapts NebulaPackets to `SeyondScan` format and delegates to the same `SeyondPCDDecoder`, so the output is identical regardless of input type.

| Property            | Type   | Description / values                                                                     |
| ------------------- | ------ | ---------------------------------------------------------------------------------------- |
| `flags`             | uint16 | Availability mask; see below                                                             |
| `refl_type`         | uint8  | Point classification: 0 = normal, 1 = ground, 2 = fog                                    |
| `elongation`        | uint8  | Raw elongation 0–15                                                                      |
| `lidar_status`      | uint8  | 0 = none, 1 = transition, 2 = normal, 3 = failed                                         |
| `lidar_mode`        | uint8  | 1 = sleep, 2 = standby, 3 = work_normal, 6 = protection                                  |
| `pkt_version_major` | uint8  | Packet protocol major (0–255)                                                            |
| `pkt_version_minor` | uint8  | Packet protocol minor (0–255)                                                            |
| `lidar_type`        | uint8  | Seyond LiDAR model; see [LiDAR type values](#lidar-type-values)                          |
| `is_2nd_return`     | uint8  | 0 = 1st return (1st echo), 1 = 2nd return (2nd echo). Per-point.                         |
| `multi_return_mode` | uint8  | Per-packet LiDAR mode; see [Multi-return mode values](#multi-return-mode-values).        |
| `use_reflectance`   | uint8  | Per-packet `intensity` semantics; see [use_reflectance values](#use_reflectance-values). |

Note: `refl_type` and `elongation` are only available when the packet contains `InnoXyzPoint` data (as opposed to `InnoEnXyzPoint`). The other extended fields (`lidar_status`, `lidar_mode`, `pkt_version_*`, `lidar_type`, `is_2nd_return`, `multi_return_mode`, `use_reflectance`) are available for both packet variants. The `flags` mask indicates which fields are valid for each point.

## Availability flags and extended layout

### Semantics

- **Availability mask** (`flags`, `uint16_t`): one bit per extended property.
  - Bit set ⇒ the corresponding value is valid.
  - Bit clear ⇒ the value is **0** and must be ignored.
- **Values**: unsigned types with the smallest range that fits; no sentinel.

Decoder: set the bit when the property is filled; leave clear when not available. Consumer: test the bit before using the value.

### Layout (extended part)

| Field             | Type     | Size   | Value range    | Flag bit |
| ----------------- | -------- | ------ | -------------- | -------- |
| flags             | uint16_t | 2      | bits 0–9 used  | —        |
| refl_type         | uint8_t  | 1      | 0, 1, 2        | 0        |
| elongation        | uint8_t  | 1      | 0–15           | 1        |
| lidar_status      | uint8_t  | 1      | 0–3            | 2        |
| lidar_mode        | uint8_t  | 1      | 0–9 (e.g. 1–6) | 3        |
| pkt_version_major | uint8_t  | 1      | 0–255          | 4        |
| pkt_version_minor | uint8_t  | 1      | 0–255          | 5        |
| lidar_type        | uint8_t  | 1      | 0–7            | 6        |
| is_2nd_return     | uint8_t  | 1      | 0, 1           | 7        |
| multi_return_mode | uint8_t  | 1      | 0–3            | 8        |
| use_reflectance   | uint8_t  | 1      | 0, 1           | 9        |
| **Total**         |          | **12** |                |          |

Bits 10–15 of `flags` are reserved.

### Flag constants (C++)

In this codebase the constants are in `bag_converter::point::en_xyzit_flags`:

```cpp
namespace en_xyzit_flags {
  constexpr uint16_t HAS_REFL_TYPE         = 1u << 0;
  constexpr uint16_t HAS_ELONGATION        = 1u << 1;
  constexpr uint16_t HAS_LIDAR_STATUS      = 1u << 2;
  constexpr uint16_t HAS_LIDAR_MODE        = 1u << 3;
  constexpr uint16_t HAS_PKT_VERSION_MAJOR = 1u << 4;
  constexpr uint16_t HAS_PKT_VERSION_MINOR = 1u << 5;
  constexpr uint16_t HAS_LIDAR_TYPE        = 1u << 6;
  constexpr uint16_t HAS_IS_2ND_RETURN     = 1u << 7;
  constexpr uint16_t HAS_MULTI_RETURN_MODE = 1u << 8;
  constexpr uint16_t HAS_USE_REFLECTANCE   = 1u << 9;
  // bits 10–15 reserved
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

## Multi-return mode values

The `multi_return_mode` field reflects the LiDAR's operating mode for that packet (`InnoDataPacket::multi_return_mode`, 3 bits). The same value is copied to every point produced from that packet. Validity is gated by the `HAS_MULTI_RETURN_MODE` bit in `flags`.

| Value | SDK enum (`InnoMultipleReturnMode`)              | Returns per firing | Meaning                                                      |
| ----- | ------------------------------------------------ | ------------------ | ------------------------------------------------------------ |
| 0     | `INNO_MULTIPLE_RETURN_MODE_NONE`                 | 1                  | Mode not set / unknown. Treated as single return by the SDK. |
| 1     | `INNO_MULTIPLE_RETURN_MODE_SINGLE`               | 1                  | Single return.                                               |
| 2     | `INNO_MULTIPLE_RETURN_MODE_2_STRONGEST`          | 2                  | Two strongest returns per firing.                            |
| 3     | `INNO_MULTIPLE_RETURN_MODE_2_STRONGEST_FURTHEST` | 2                  | One strongest return plus one furthest return per firing.    |

### Relationship with `is_2nd_return`

`is_2nd_return` (per-point) reports the physical echo order — 0 = 1st echo, 1 = 2nd echo — and is independent of how the point was scheduled into a return slot. In the `2_STRONGEST_FURTHEST` mode, the "furthest" return may arrive before or after the "strongest" return in time, so `is_2nd_return` cannot be inferred from packet structure alone and must be read from the point itself.

## use_reflectance values

The `use_reflectance` field reflects which sensor field was used to populate `intensity` for that packet (`InnoDataPacket::use_reflectance`, 1 bit). The same value is copied to every point produced from that packet. Validity is gated by the `HAS_USE_REFLECTANCE` bit in `flags`.

| Value | Meaning          | Source field in `InnoEnXyzPoint`            |
| ----- | ---------------- | ------------------------------------------- |
| 0     | Intensity mode   | `InnoEnXyzPoint::intensity` (raw intensity) |
| 1     | Reflectance mode | `InnoEnXyzPoint::reflectance` (reflectance) |

For `InnoXyzPoint` (Falcon SPHERE), the sensor only exposes a single `refl` field that is used regardless of `use_reflectance`; the mode flag is still copied through for downstream introspection.

Robin W intensity scaling (see [seyond_intensity_scaling.md](seyond_intensity_scaling.md)) is applied uniformly after the source field is selected.
