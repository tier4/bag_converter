# en_xyzit: availability flags and extended field layout

This document is the canonical reference for the **availability flags** and **extended field layout** of the `en_xyzit` point type (experimental).

## Semantics

- **Availability mask** (`flags`, `uint16_t`): one bit per extended property.
  - Bit set ⇒ the corresponding value is valid.
  - Bit clear ⇒ the value is **0** and must be ignored.
- **Values**: unsigned types with the smallest range that fits; no sentinel. When a property is not supported, its value is **0** and must be ignored.

Decoder: set the bit when the property is filled; leave clear when not available. Consumer: test the bit before using the value.

## Layout (extended part)

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

## Flag constants (C++)

In this codebase the constants are in `bag_converter::point::en_xyzit_flags`:

```cpp
namespace en_xyzit_flags {
  constexpr uint16_t HAS_REFL_TYPE        = 1u << 0;
  constexpr uint16_t HAS_ELONGATION       = 1u << 1;
  constexpr uint16_t HAS_LIDAR_STATUS    = 1u << 2;
  constexpr uint16_t HAS_LIDAR_MODE      = 1u << 3;
  constexpr uint16_t HAS_PKT_VERSION_MAJOR = 1u << 4;
  constexpr uint16_t HAS_PKT_VERSION_MINOR = 1u << 5;
  constexpr uint16_t HAS_LIDAR_TYPE      = 1u << 6;
  // bits 7–15 reserved
}
```
