# Tier4 seyond_ros_driver vs Original (Seyond-Inc) — Source Code Comparison

This document summarizes the differences between:

- **[tier4/seyond_ros_driver](https://github.com/tier4/seyond_ros_driver)** (branch: `tier4/develop`, commit 516db2d as of survey)
- **[Seyond-Inc/seyond_ros_driver](https://github.com/Seyond-Inc/seyond_ros_driver)** (branch: `main`, commit 41b1f20 as of survey)

The survey was done by cloning both repositories and comparing source files under `src/seyond_lidar_ros/`. Both drivers use the same **inno-lidar-sdk**; the SDK delivers only **data packets** to `lidar_data_callback` and **status packets** to `lidar_status_callback` separately.

---

## 1. Status packets in SeyondScan

**Conclusion: Neither driver puts status packets into SeyondScan.**

In both codebases:

- The packet publish callback (which fills `SeyondScan.packets`) is registered with `register_publish_packet_callback()` and is **only invoked from `lidar_data_callback()`**, which receives `const InnoDataPacket *`.
- `lidar_status_callback(const InnoStatusPacket *pkt)` is implemented in both and only performs sanity check and logging (e.g. `inno_lidar_printf_status_packet`); it **never** calls the packet publish callback or adds anything to SeyondScan.

So for both Tier4 and original, SeyondScan contains only what is pushed in the data-callback path (POINTS and, in Tier4, HVTABLE as a separate packet type). Status packets never enter SeyondScan.

---

## 2. Message format (SeyondScan / SeyondPacket)

This is the most important difference for **bag_converter**, which expects the **Tier4 message layout**.

### Original (Seyond-Inc)

#### SeyondScan.msg

```text
float64                 timestamp
uint32                  size
bool                    is_last_scan
SeyondPacket[]          packets
```

#### SeyondPacket.msg

```text
bool                    has_table
uint8[]                 data
uint8[]                 table
```

- No `std_msgs/Header` on the scan.
- No explicit packet `type` or `stamp`; HVTABLE is indicated by `has_table` and the table payload is in `table`.

### Tier4

#### SeyondScan.msg

```text
std_msgs/Header header
SeyondPacket[] packets
```

#### SeyondPacket.msg

```text
uint8 PACKET_TYPE_POINTS = 0
uint8 PACKET_TYPE_HVTABLE = 1

builtin_interfaces/Time stamp
uint8 type
uint8[] data
```

- Scan has `header` (frame_id, stamp).
- Each packet has `stamp`, `type` (POINTS or HVTABLE), and `data` only (no separate `has_table`/`table`).

**Impact**: bag_converter’s `SeyondScan` / `SeyondPacket` types and decoding logic (e.g. `PACKET_TYPE_POINTS`, `PACKET_TYPE_HVTABLE`) match the **Tier4** definition. Rosbags recorded with the **original** driver use a different schema and are not compatible with bag_converter without conversion.

---

## 3. Repository and file layout

| Aspect             | Original (Seyond-Inc)                                         | Tier4                                                     |
| ------------------ | ------------------------------------------------------------- | --------------------------------------------------------- |
| Default branch     | `main`                                                        | `tier4/develop`                                           |
| ROS support        | ROS1 + ROS2 (adapters)                                        | ROS2 only                                                 |
| Node entry         | `node/seyond_node.cc` → `ROSNode` + adapter                   | `src/driver/seyond_node.cc` → `SeyondNode` (single class) |
| Driver adapter     | `ros1_driver_adapter.hpp`, `ros2_driver_adapter.hpp`          | None (node embeds driver usage)                           |
| Config / YAML      | `yaml_tools.hpp`, multi-file config                           | Parameters only (no YAML tools in tree)                   |
| Multi-LiDAR fusion | `multi_fusion/ros1_multi_fusion.hpp`, `ros2_multi_fusion.hpp` | Not present                                               |
| Test / demo        | `ros1_test.hpp`, `ros2_test.hpp`                              | `ros2_test.hpp` only                                      |
| Launch             | `start.py`, `start_with_config.py`, `test.launch`, etc.       | `seyond.launch.xml` only                                  |

Tier4 has a simpler, ROS2-only layout and no ROS1 or fusion code in the surveyed tree.

---

## 4. Driver core (driver_lidar.cc / driver_lidar.h)

### License and copyright

- Original: Long BSD-2 license header in each file.
- Tier4: Short Apache-style header with “Copyright (C) 2025 Seyond Inc.” and “License: Apache License”.

### Config representation

- **Original**: Single `LidarConfig param_` in the driver; all options (lidar*name, udp_port, hv_table_file, transform_enable, etc.) accessed as `param*.field`.
- **Tier4**: No `param_`; same options stored as direct member variables (e.g. `lidar_name_`, `udp_port_`, `hv_table_file_`, `transform_enable_`, `x_`, `y_`, `z_`, `pitch_`, `yaw_`, `roll_`, `transform_matrix_`).

### LidarConfig (driver_lidar.h)

- **Original**: Contains `aggregate_num`, `enable_falcon_ring`. `CommonConfig` has `fusion_enable`, `fusion_topic`.
- **Tier4**: No `aggregate_num`, no `enable_falcon_ring`, no `fusion_enable` / `fusion_topic`.

### Falcon ring (original only)

- Original: When `enable_falcon_ring` is true, uses `inno_lidar_set_attribute_string(..., "use_ring_id", "1")`, `inno_lidar_get_ring_id_converter()`, and `RingIdConverterInterface *ring_id_converter_`; in point parse, `point.scan_id = point_ptr->ring_id` when enabled.
- Tier4: No `ring_id_converter_`, no `sdk_common/ring_id_converter_interface.h`; no Falcon ring handling.

### Frame / packet callback semantics

- **Original**: In `lidar_data_callback`, when `param_.packet_mode` is true, calls `packet_publish_cb_(..., frame_start_ts_, is_next_frame)`. On first packet of a new frame, `frame_start_ts_` is set to `pkt->common.ts_start_us`; that **frame start** time is passed for all packets of that frame.
- **Tier4**: Calls `packet_publish_cb_(reinterpret_cast<const int8_t *>(pkt), pkt_len, pkt->common.ts_start_us, next_idx_flag)` — i.e. passes **each packet’s** `ts_start_us` as the timestamp, not a single frame start.

So Tier4 packet stamps are per-packet; original (in the adapter) uses a single timestamp per frame for the scan.

### Other driver_lidar differences

- Transform: Original uses `param_.transform_enable`, `param_.x`, etc.; Tier4 uses `transform_enable_`, `x_`, etc.
- Tier4 removes the long BSD block from the top of `driver_lidar.cc` / `driver_lidar.h` and `point_types.h`.

---

## 5. Packet publishing and SeyondScan construction

### Original (ros2_driver_adapter.hpp)

- `publishPacket(int8_t* pkt, uint64_t pkt_len, double timestamp, bool next_idx)`:
  - On `next_idx`: sets `inno_scan_msg_->timestamp`, `size`, `is_last_scan = true`, publishes, then resets the message.
  - Else if `packets_width_ >= lidar_config_.aggregate_num`: publishes with `is_last_scan = false` (mid-frame publish).
  - Every packet is appended with `msg.data`; `msg.has_table = false` by default. Periodically (every `table_send_hz_` frames) one packet is sent with `has_table = true` and `msg.table` set from `anglehv_table_`.
- So original: **aggregate_num**-based batching, HVTABLE sent **inside** a packet’s `has_table`/`table` fields, no explicit `type` or `stamp` in the message definition.

### Tier4 (seyond_node.cc)

- `publishPacket(...)`:
  - Only publishes on **next_idx** (frame boundary). No mid-frame publish; no `aggregate_num`.
  - On frame boundary: sets `inno_scan_msg_->header.stamp` from first packet’s stamp, `header.frame_id`; if `anglehv_table_init_`, appends a **separate** `SeyondPacket` with `type = PACKET_TYPE_HVTABLE` and `data = anglehv_table_`; then publishes.
  - Each data packet is appended with `msg.type = PACKET_TYPE_POINTS`, `msg.stamp = rclcpp::Time(timestamp * 1000)`, `msg.data = pkt`.
- So Tier4: **Frame-boundary-only** publish, **explicit** POINTS vs HVTABLE via `type`, and HVTABLE as a **separate** packet with its own `data`.

---

## 6. Point cloud / frame publishing

- **Original**: `publishFrame(frame, timestamp)` uses the callback `timestamp` (frame time) for `header.stamp` (nanoseconds derived from `timestamp * 1000`).
- **Tier4**: `publishFrame(frame, timestamp)` uses the **first point’s** `timestamp` for `header.stamp` when the frame is non-empty; otherwise falls back to the provided `timestamp`.

---

## 7. Summary table

| Topic                            | Original (Seyond-Inc)                  | Tier4                               |
| -------------------------------- | -------------------------------------- | ----------------------------------- |
| Status in SeyondScan             | No                                     | No                                  |
| SeyondScan fields                | timestamp, size, is_last_scan, packets | header, packets                     |
| SeyondPacket fields              | has_table, data, table                 | stamp, type, data                   |
| HVTABLE in message               | has_table + table in one packet        | Separate packet with type = HVTABLE |
| Packet publish trigger           | next_idx or aggregate_num              | next_idx only                       |
| Packet timestamp                 | frame*start_ts* (one per frame)        | ts_start_us per packet              |
| ROS1 support                     | Yes (adapter)                          | No                                  |
| Fusion / YAML / multi-lidar      | Yes                                    | No                                  |
| Falcon ring (enable_falcon_ring) | Yes                                    | No                                  |
| Config in driver                 | param\_ (LidarConfig)                  | Member variables                    |

---

## 8. Conclusion for bag_converter

- **SeyondScan source**: bag_converter uses the `seyond::msg::SeyondScan` / `seyond::msg::SeyondPacket` types from [seyond_ros_driver](https://github.com/tier4/seyond_ros_driver) and expects input bags to be recorded with that driver. The message format (header + packets with type and stamp) and frame-boundary publishing with explicit HVTABLE packets match Tier4, not the original.
- **Status packets**: Neither Tier4 nor the original driver adds status packets to SeyondScan; both rely on the SDK’s separate data vs status callbacks. So the “no status in SeyondScan” assumption holds for both.
- **Compatibility**: Rosbags recorded with the **original** Seyond-Inc driver use a different SeyondScan/SeyondPacket schema and are not directly compatible with bag_converter; conversion would require adapting to the Tier4 message layout (header, type, stamp, data-only packets, HVTABLE as a separate packet).
