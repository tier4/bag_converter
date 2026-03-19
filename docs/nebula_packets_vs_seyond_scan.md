# NebulaPackets vs SeyondScan: Packet Generation Differences

This document compares how **NebulaPackets** (from `nebula_drs`) and **SeyondScan** (from `seyond_ros_driver`) wrap raw Seyond LiDAR data into ROS messages. Understanding these differences is essential for building a unified decoder in `bag_converter`.

**SeyondScan assumption**: `bag_converter` uses the `seyond::msg::SeyondScan` and `seyond::msg::SeyondPacket` types from [seyond_ros_driver](https://github.com/tier4/seyond_ros_driver) and expects input bags to be recorded with that driver. The behavior and message format described for SeyondScan in this document refer to that driver.

## 1. Message Definitions

### NebulaPackets (nebula_drs)

```text
# NebulaPackets.msg
std_msgs/Header   header
NebulaPacket[]     packets

# NebulaPacket.msg
builtin_interfaces/Time stamp
uint8[] data
```

- **No explicit packet type field** -- the type must be inferred from the raw binary data.

### SeyondScan ([seyond_ros_driver](https://github.com/tier4/seyond_ros_driver))

```text
# SeyondScan.msg
std_msgs/Header header
SeyondPacket[] packets

# SeyondPacket.msg
uint8 PACKET_TYPE_POINTS = 0
uint8 PACKET_TYPE_HVTABLE = 1

builtin_interfaces/Time stamp
uint8 type    # packet type
uint8[] data  # packet contents
```

- **Explicit `type` field** distinguishes POINTS packets from HVTABLE packets at the message level.

## 2. Packet Data Content

Both formats store the raw `InnoDataPacket` / `SeyondDataPacket` binary as the `data` field, but they originate from different SDK layers and there are subtle differences in what ends up inside.

### NebulaPackets: Direct UDP capture

Source: `SeyondHwInterface::ReceiveSensorPacketCallback()` -> `SeyondRosWrapper::ReceiveCloudPacketCallback()`

1. Raw UDP datagrams are received directly from the LiDAR sensor via a UDP socket.
2. Each UDP datagram is wrapped as-is into a `NebulaPacket.data`.
3. The data is a raw `SeyondDataPacket` binary, exactly as transmitted over the wire.
4. **All packet types arrive via UDP** -- the type is embedded in the `SeyondDataPacket.type` field (offset byte index in the binary header).

### SeyondScan: SDK callback output

Source: `DriverLidar::lidar_data_callback()` -> `SeyondNode::publishPacket()`

1. The Seyond SDK receives UDP packets internally via `StageClientRead` (`stage_client_read.cpp`).
2. The SDK performs the following processing before invoking the callback:
   - **Protocol v1-to-v2 conversion**: If `major_version == 1`, the SDK calls `InnoPacketV1Adapt::check_data_packet_v1_and_convert_packet()`, which inserts a 16-byte gap (`extend_reserved[4]`), updates `common.size`, sets `major_version = 2` / `minor_version = 1`, and recalculates CRC32.
   - **CRC32 verification**: Validates packet integrity via `InnoPacketReader::verify_packet_crc32()`.
   - **Packet structure validation**: Checks magic number, item sizes, return modes, etc.
3. The SDK provides the processed data as `InnoDataPacket*` (equivalent to `SeyondDataPacket*`).
4. Each packet is copied into `SeyondPacket.data` via `std::memcpy`.

**Key difference**: NebulaPackets wraps raw UDP datagrams (may contain v1 protocol headers); SeyondScan wraps SDK-processed `InnoDataPacket` output (always v2 format, CRC-verified). Both contain the same `InnoDataPacket` binary structure, but SeyondScan packets have already been normalized and validated by the SDK.

## 3. AngleHV Calibration Table Handling

This is one of the most significant differences and directly impacts decoding of Robin W compact packets.

### NebulaPackets

The AngleHV table is acquired through one of two sources:

1. **HTTP request to LiDAR sensor** (primary): `SeyondHwInterface::GetLidarCalibrationString()` (`seyond_hw_interface.cpp:294-323`) uses Boost.Asio `HttpClientDriver` to perform `GET /command/?get_anglehv_table` -- the same HTTP endpoint as `seyond_ros_driver`. The response is stored in `SeyondCalibrationConfiguration` as a raw binary string.

2. **From data stream**: If no external calibration is available, the nebula decoder looks for an AngleHV table packet in the UDP data stream (`seyond_decoder.cpp:344-355`). LiDAR sensors may transmit the table as a packet with `type == SEYOND_ROBINW_ITEM_TYPE_ANGLEHV_TABLE (100)`.

When publishing `NebulaPackets`, the calibration packet is **synthesized and prepended** to the packet array at the start of each frame:

- `decoder_wrapper.cpp:194-211`: If the sensor is Robin W and calibration data exists, a `NebulaPacket` containing the calibration string is inserted at index 0 of the `packets` array.
- The calibration `data` field contains the **raw calibration string** from `GetCalibrationString()`, which is `sizeof(SeyondDataPacket) + sizeof(SeyondAngleHVTable)` bytes.
- This is a reconstructed packet -- it has a `NebulaPacket.stamp` from `high_resolution_clock::now()` but no SDK-generated packet header fields beyond the calibration data itself.
- **The calibration packet has NO `SeyondDataPacket` header with `type = SEYOND_ROBINW_ITEM_TYPE_ANGLEHV_TABLE (100)`** in a consistently initialized state. The nebula decoder's constructor (`seyond_decoder.cpp:61-99`) reconstructs the full packet header when loading from the calibration string, setting `type = SEYOND_ROBINW_ITEM_TYPE_ANGLEHV_TABLE`, magic number, etc.

### SeyondScan

The AngleHV table is acquired through one of three sources (in priority order):

1. **External file** (highest priority): If the `hv_table_file_` parameter is set, the driver reads the binary file at startup in the `DriverLidar` constructor (`driver_lidar.cc:98-115`). The file contents are loaded directly into `anglehv_table_` and `anglehv_table_init_` is set to true.

2. **HTTP request to LiDAR sensor** (live connection): If no file is provided, the first time a Compact sphere packet (`CHECK_CO_SPHERE_POINTCLOUD_DATA`) arrives in `lidar_data_callback()` (`driver_lidar.cc:407-416`), the SDK calls `inno_lidar_get_anglehv_table()`. This function internally performs an HTTP GET request to the sensor at `/command/?get_anglehv_table` (`lidar_client_communication.cpp:100-110`), receives the binary response, verifies its CRC32, and caches the result.

3. **From SeyondScan packets** (rosbag replay): When `replay_rosbag` mode is enabled and the table has not been initialized, the driver extracts the HVTABLE from a `PACKET_TYPE_HVTABLE` packet in the incoming SeyondScan message (`seyond_node.cc:98-101`).

Once acquired, the table is embedded into SeyondScan messages:

- On each frame boundary (`next_idx == true`), if `anglehv_table_init_` is true, a `SeyondPacket` with `type = PACKET_TYPE_HVTABLE` is **appended at the end** of the packet array (`seyond_node.cc:122-127`).
- The `data` field contains the raw `anglehv_table_` bytes (the `InnoDataPacket` + `InnoAngleHVTable` structure).
- The same cached table data is copied into every frame.
- The explicit `type = PACKET_TYPE_HVTABLE (1)` makes identification trivial.

### Summary of calibration differences

| Aspect                    | NebulaPackets                                                                  | SeyondScan                                                                           |
| ------------------------- | ------------------------------------------------------------------------------ | ------------------------------------------------------------------------------------ |
| Position in packets array | **First** (index 0)                                                            | **Last** (appended after all POINTS packets)                                         |
| Type identification       | Must inspect binary data (magic number / packet type field)                    | Explicit `SeyondPacket.type == PACKET_TYPE_HVTABLE`                                  |
| Data origin               | HTTP `GET /command/?get_anglehv_table` via Boost.Asio, or from UDP data stream | HTTP `GET /command/?get_anglehv_table` via SDK, external file, or from rosbag replay |
| Content format            | Raw HTTP response bytes (= SeyondDataPacket + SeyondAngleHVTable)              | Raw InnoDataPacket + InnoAngleHVTable bytes (CRC-verified by SDK)                    |
| Present when              | Only for Robin W with calibration available                                    | Only when `anglehv_table_init_` is true                                              |

## 4. Protocol Version Compatibility

### NebulaPackets

The nebula decoder (`SeyondDecoder::unpack()`) includes **protocol version compatibility handling**:

```cpp
void SeyondDecoder::ProtocolCompatibility(std::vector<uint8_t> & buffer) {
  uint8_t major_version = buffer[kSeyondPktMajorVersionSection];
  if (major_version == kSeyondProtocolMajorV1) {
    // Add 16 bytes to the header for v1 -> v2 compatibility
    *packet_size += 16;
    buffer.insert(buffer.begin() + kSeyondProtocolOldHeaderLen, 16, 0);
  }
}
```

- Protocol v1 uses `InnoDataPacketV1` (54-byte header), which lacks the 16-byte `extend_reserved[4]` field present in v2's `InnoDataPacket` (70-byte header). The difference is exactly `sizeof(InnoDataPacket) - sizeof(InnoDataPacketV1) = 16` bytes.
- The nebula decoder **inserts 16 zero bytes** and adjusts the packet size to pad v1 packets to v2 format before parsing.
- This means raw v1 UDP packets in NebulaPackets bags need this transformation before parsing.

Note: The SDK performs a more thorough conversion (`InnoPacketV1Adapt`): it also CRC-verifies the v1 packet, updates version fields to v2, and recalculates CRC32. The nebula decoder's simpler approach (just inserting zero bytes) does not update the version or CRC fields.

### SeyondScan

The Seyond SDK handles protocol v1 compatibility internally in `StageClientRead::read_packet_()` (`stage_client_read.cpp`) using `InnoPacketV1Adapt::check_data_packet_v1_and_convert_packet()` (`inno_lidar_packet_v1_adapt.h`). The conversion process is:

1. Detect v1 by checking `major_version == kInnoProtocolMajorV1 (1)`.
2. Move the v1 packet data backwards by `kMemorryFrontGap` bytes (= `sizeof(InnoDataPacket) - sizeof(InnoDataPacketV1)` = 16 bytes) to create space for the `extend_reserved[4]` field.
3. Zero-fill the `extend_reserved[4]` field.
4. Update `common.size += 16`, set `major_version = 2`, `minor_version = 1`.
5. Recalculate CRC32 via `InnoPacketReader::set_packet_crc32()`.

As a result, **packets in SeyondScan bags are always in v2 format** -- no additional protocol version patching is needed when decoding.

**Impact for bag_converter**: When decoding NebulaPackets, the v1/v2 protocol compatibility transform must be applied. When decoding SeyondScan, it is not needed.

## 5. Packet Validation

### NebulaPackets

The nebula decoder performs strict validation before processing (`SeyondDecoder::IsPacketValid()`):

- Verifies `buffer.size() >= packet_size` field
- Checks magic number == `0x176A`
- Rejects packet types 2 (MESSAGE) and 3 (MESSAGE_LOG)
- Rejects packets smaller than 60 bytes

Non-data packets (messages, logs, status packets) that arrive via UDP are **silently discarded** during validation.

### SeyondScan

- The SDK filters packets before invoking the callback -- only data packets are delivered.
- The `publishPacket()` function assigns `type = PACKET_TYPE_POINTS` to all SDK-delivered packets without further validation.
- Non-data packets are never present in SeyondScan messages.

**Impact for bag_converter**: NebulaPackets bags may contain non-decodable packets (though they would fail validation). SeyondScan bags contain only decodable POINTS and HVTABLE packets.

## 6. Frame Boundary Detection

### NebulaPackets

- The nebula decoder tracks `current_packet_id_` (= `SeyondDataPacket.idx`, the frame index).
- A frame boundary is detected when `seyond_pkt->idx` changes (increases or resets to 0).
- `has_scanned_` flag triggers scan publication.
- Frame boundary detection and NebulaPackets publication are tightly coupled in `decoder_wrapper.cpp`.

### SeyondScan

- The driver tracks `current_frame_id_` (= `pkt->idx`).
- When `pkt->idx` changes, `next_idx` flag is set to true.
- This triggers publication of the accumulated SeyondScan and creation of a new one.
- The HVTABLE packet is appended just before publication.

Both use the same mechanism (`pkt->idx` change), so packets within a single scan message should correspond to the same frame.

## 7. Timestamp Assignment

### NebulaPackets

- `NebulaPacket.stamp` is set from `std::chrono::high_resolution_clock::now()` at the time of UDP reception.
- This is **host wall-clock time**, not LiDAR time.
- The `NebulaPackets.header.stamp` is set from the first packet's stamp.

### SeyondScan

- `SeyondPacket.stamp` is set from `rclcpp::Time(timestamp * 1000)` where `timestamp` comes from the SDK callback parameter.
- This is the **SDK-provided timestamp** (derived from the LiDAR's internal clock / PTP sync).
- The `SeyondScan.header.stamp` is set from the first packet's stamp.

**Impact for bag_converter**: The `stamp` field in individual packets has different time bases. However, the actual point timestamps are derived from `InnoDataPacket.common.ts_start_us` and per-point `ts_10us` fields, which are identical in both formats (embedded in the raw data).

## 8. Intensity Scaling

### NebulaPackets (nebula decoder)

In `compact_data_packet_parse_()`:

```cpp
if (pkt->use_reflectance)
  intensity_scaling_factor = pkt->common.version.major_version > 3 ? 1.0 : 255.0 / 4095.0;
else
  intensity_scaling_factor = 255.0 / 1600.0;
```

- Reflectance mode: scales 12-bit [0, 4095] to 8-bit [0, 255] for protocol <= v3; no scaling for v4+.
- Intensity mode: scales [0, 1600] to [0, 255].
- For non-compact (XYZ/EnXYZ) packets: raw `refl` / `intensity` / `reflectance` fields are used directly with no scaling.

### SeyondScan ([seyond_ros_driver](https://github.com/tier4/seyond_ros_driver))

[seyond_ros_driver](https://github.com/tier4/seyond_ros_driver) in packet mode does not decode points -- it stores raw packets. The decoding logic in frame mode (or subscriber callback) uses the same SDK functions (`inno_lidar_convert_to_xyz_pointcloud2`, etc.) as `bag_converter/seyond_decoder.cpp`.

### bag_converter's seyond_decoder.cpp

```cpp
// Robin W with protocol major version <= 3 reports intensity in [0, 4095]
scale_intensity_12bit_ =
  (pkt->common.lidar_type == INNO_LIDAR_TYPE_ROBINW && pkt->common.version.major_version <= 3);
// ...
if (scale_intensity_12bit_) {
  point.intensity = point.intensity * (255.0f / 4095.0f);
}
```

- Currently only handles the reflectance mode 12-bit scaling for Robin W.
- Does **not** handle the intensity mode scaling (255/1600) that nebula applies.

## 9. Summary of Key Differences for Unified Decoding

| Aspect                      | NebulaPackets                                        | SeyondScan                                                                  |
| --------------------------- | ---------------------------------------------------- | --------------------------------------------------------------------------- |
| Packet type field           | None (infer from binary)                             | Explicit `type` field                                                       |
| Raw data source             | UDP datagram                                         | SDK callback output                                                         |
| Protocol v1 compat          | Must apply 16-byte header padding (simple zero-fill) | Already converted to v2 by SDK (`InnoPacketV1Adapt`) with CRC recalculation |
| HVTABLE position            | First packet in array                                | Last packet in array                                                        |
| HVTABLE format              | Raw calibration string                               | Raw InnoDataPacket binary                                                   |
| Non-data packets            | May be present (filtered by validation)              | Never present                                                               |
| Packet timestamp base       | Host wall-clock                                      | SDK / LiDAR clock                                                           |
| Intensity scaling (compact) | Reflectance: 255/4095 (<=v3); Intensity: 255/1600    | Delegated to decoder                                                        |

## 10. Implementation in bag_converter

`NebulaPCDDecoder` is a thin wrapper around `SeyondPCDDecoder`: it converts NebulaPackets to SeyondScan internally and delegates point decoding. This eliminates the nebula driver dependency and ensures identical point cloud output regardless of input type.

### 10.1 How NebulaPackets → SeyondScan conversion works

The conversion is implemented in `nebula_packets_to_seyond_scan()` ([nebula_decoder.cpp](../src/bag_converter/src/nebula_decoder.cpp)). It iterates over `input.packets` in order and builds a `SeyondScan` with `header` (from `input.header` plus the given `frame_id`) and a `packets` array of `SeyondPacket` messages. Each input packet is handled as follows.

**Input**: One `NebulaPacket` (binary `data` + `stamp`). No explicit type field.

**Per-packet flow**:

1. **Empty packet**  
   If `nebula_pkt.data.empty()`, the packet is skipped (not added to the scan).

2. **First packet: Robin W prepended calibration (nebula_drs)**  
   When publishing NebulaPackets for Robin W, nebula_drs inserts the **raw calibration string** from `GetCalibrationString()` as the first packet’s `data` — it does **not** build a valid `InnoDataPacket` header (magic/type may be uninitialized).  
   The converter only treats the first packet as this blob when **both** (1) `data.size() == sizeof(InnoDataPacket) + sizeof(InnoAngleHVTable)` and (2) the header is **not** already a valid data packet (see below). Then it:

   - Overwrites the first `sizeof(InnoDataPacket)` bytes to form a valid header: `magic_number = kInnoMagicNumberDataPacket`, `type = INNO_ROBINW_ITEM_TYPE_ANGLEHV_TABLE`, `common.size`, `item_number`, `item_size`, `is_first_sub_frame`, `is_last_sub_frame`, `sub_idx`.
   - Appends a `SeyondPacket` with `type = PACKET_TYPE_HVTABLE`, `stamp = nebula_pkt.stamp`, and the (possibly modified) `data`.  
     Then continues to the next input packet.

   **Safer detection**: To avoid false positives (e.g. a same-size POINTS packet), the code first reads `magic_number` and `InnoDataPacket.type`. If `magic_number == kInnoMagicNumberDataPacket` and `type` is already `INNO_ROBINW_ITEM_TYPE_ANGLEHV_TABLE`, the packet is emitted as HVTABLE **without** overwriting the header. If `magic_number != kInnoMagicNumberDataPacket`, the packet is treated as the raw blob and the header is fixed. If the magic is valid but the type is something else (e.g. POINTS), the packet is **not** treated as HVTABLE; it falls through to the normal path and is classified by type. This way we never overwrite a valid packet or mis-identify another packet type as the calibration blob.

3. **All other packets: magic and type from binary**
   - If `data.size() < sizeof(InnoCommonVersion)`, the packet is skipped and an INFO log is emitted (invalid or unknown magic).
   - Read `magic_number` from the first bytes (SDK: `InnoCommonVersion`).
     - If `magic == kInnoMagicNumberStatusPacket`: skip and log (status packet).
     - If `magic != kInnoMagicNumberDataPacket`: skip and log (invalid).
   - **Protocol v1 → v2**: If `data.size() >= sizeof(InnoDataPacketV1)` and `major_version == InnoPacketV1Adapt::kInnoProtocolMajorV1`, insert 16 zero bytes after the v1 header (SDK: `kMemorryFrontGap`), then set `common.size`, `major_version`, and `minor_version` to the v2 values.
   - If after that `data.size() < sizeof(InnoDataPacket)`, skip the packet.
   - Read `InnoDataPacket.type`. If `type` is `INNO_ITEM_TYPE_MESSAGE` or `INNO_ITEM_TYPE_MESSAGE_LOG`, skip (no SeyondPacket added).
   - **Classify**: If `CHECK_ANGLEHV_TABLE_DATA(pkt->type)` then emit `PACKET_TYPE_HVTABLE`, else emit `PACKET_TYPE_POINTS`.
   - Append a `SeyondPacket` with that `type`, `stamp = nebula_pkt.stamp`, and the (possibly v1-compat modified) `data`.

**Output**: A `SeyondScan` whose `packets` array contains only HVTABLE and POINTS packets, in the same order as the accepted input packets (with skipped packets omitted). No explicit reordering is done; e.g. a prepended HVTABLE at index 0 remains first.

### 10.2 Summary of conversion steps (checklist)

1. **Packet validation**: For non–first packets (or first packet not matching Robin W table size), check magic (`kInnoMagicNumberDataPacket` / `kInnoMagicNumberStatusPacket`). Skip status and invalid packets. No minimum size beyond what is needed to read the magic.
2. **Prepended calibration**: First packet with size `sizeof(InnoDataPacket) + sizeof(InnoAngleHVTable)` and **invalid header** (magic ≠ data packet) is normalized to a valid HVTABLE packet and emitted as `PACKET_TYPE_HVTABLE`. If the header is already valid (magic + type ANGLEHV_TABLE), it is emitted as HVTABLE without overwriting. If the header is valid but type is different, the packet is handled by the normal path.
3. **Protocol v1 → v2**: For remaining data packets with `major_version == 1`, insert 16 zero bytes after the v1 header and update size/version (SDK: `InnoPacketV1Adapt`).
4. **Packet type classification**: After v1 compat, use `InnoDataPacket.type`: `CHECK_ANGLEHV_TABLE_DATA` → `PACKET_TYPE_HVTABLE`, others (except MESSAGE/MESSAGE_LOG) → `PACKET_TYPE_POINTS`. MESSAGE and MESSAGE_LOG are not added to the scan.
5. **Order**: Packet order from NebulaPackets is preserved. `SeyondPCDDecoder` does not assume HVTABLE at a fixed position; it scans all packets for HVTABLE to initialize the angle table, then processes POINTS.

### 10.3 How the differences are absorbed

- **HVTABLE position**: NebulaPackets has it first; SeyondScan from the driver has it last. The decoder is order-agnostic, so no reordering is needed.
- **Raw prepended calibration**: Recognized by size and header fixed in place so it becomes a valid `PACKET_TYPE_HVTABLE` packet.
- **Explicit type field**: Inferred from binary (magic + `InnoDataPacket.type` or first-packet size) and set on each `SeyondPacket.type`.
- **v1 compat and non-data filtering**: Applied during the conversion so the output matches what SeyondPCDDecoder expects. Decoding logic (coordinate conversion, intensity scaling, AngleHV handling) is shared via a single `SeyondPCDDecoder` implementation.

### 10.4 Ideas for safer raw AngleHV table detection

The first-packet Robin W calibration blob is currently detected by: (1) index 0, (2) size `sizeof(InnoDataPacket) + sizeof(InnoAngleHVTable)`, and (3) **invalid header** (`magic_number != kInnoMagicNumberDataPacket`). That avoids overwriting a valid HVTABLE and avoids mis-classifying a same-size POINTS packet as HVTABLE. Further hardening ideas:

| Idea                             | Description                                                                                                                                                                                      | Pros / cons                                                                                  |
| -------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ | -------------------------------------------------------------------------------------------- |
| **Magic check (implemented)**    | Only treat as raw blob when `magic_number != kInnoMagicNumberDataPacket`. Valid HVTABLE is emitted as-is; valid same-size other type falls through to normal path.                               | No overwrite of valid packets; minimal change.                                               |
| **Type check (implemented)**     | When magic is valid, read `InnoDataPacket.type`. If already `INNO_ROBINW_ITEM_TYPE_ANGLEHV_TABLE`, emit as HVTABLE without fix. If another type, fall through so it is classified as POINTS etc. | Prevents same-size POINTS from being turned into HVTABLE.                                    |
| **Body signature**               | If the SDK defines a magic or fixed bytes at the start of `InnoAngleHVTable`, check bytes at offset `sizeof(InnoDataPacket)` and only treat as blob when the signature matches.                  | Stronger guarantee that the payload is really an angle table; depends on SDK layout.         |
| **CRC / checksum**               | If the calibration blob or table has a known CRC/checksum in the payload, verify it before fixing.                                                                                               | Integrity check; requires SDK spec and may differ for raw HTTP response vs SDK-built packet. |
| **Heuristic “looks like table”** | e.g. check that values at known offsets in the table body are in expected ranges (e.g. angles in [-π, π]).                                                                                       | Can reduce false positives; fragile if format or scale changes.                              |
