# bag_converter

## Overview

`bag_converter` is a tool for converting rosbag2 files containing Nebula packets or Seyond scan messages to PointCloud2 messages. The tool automatically detects and decodes:

- Nebula packet topics (`/nebula_packets`) and converts them to point cloud topics (`/nebula_points`)
- Seyond scan topics (`/seyond_packets`) and converts them to point cloud topics (`/seyond_points`)

**Supported storage formats:** mcap, sqlite3

## Install

```shell
# clone repository
git clone https://github.com/tier4/bag_converter.git
cd bag_converter

# build
cd docker
./build.sh

# clean build
./build.sh --no-cache
```

### Switching Versions

To use a specific version, checkout the corresponding tag and rebuild:

```shell
# list available versions
git tag

# checkout a specific version and rebuild
git checkout v0.4.0
cd docker
./build.sh
```

## Usage

```shell
./bag_converter <input_bag> <output_bag> [options]
./bag_converter <input_dir> <output_dir> [options]
./bag_converter <input_dir_0> [input_dir_1 ...] <output_dir> --merge [options]
```

If the input path is a directory, all bag files (`.mcap`, `.db3`, `.sqlite3`) in it are automatically converted. The directory structure is mirrored in the output, and output filenames match the input filenames. All options are applied to every file. If a file fails to convert, the error is logged and processing continues with the remaining files.

### Options

| Option                        | Description                                                                                                                                                                                                                  |
| ----------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `--help`, `-h`                | Show help message                                                                                                                                                                                                            |
| `--version`, `-v`             | Show version                                                                                                                                                                                                                 |
| `--point-type <type>`         | Output point type: `xyzit` (default), `xyzi`, or `en_xyzit`. The `en_xyzit` type adds extended fields (refl_type, elongation, lidar_status, lidar_mode, and packet version); see [Output: PointCloud2](#output-pointcloud2). |
| `--keep-original`             | Keep original packet topics in output bag                                                                                                                                                                                    |
| `--base-frame <frame>`        | Transform PointCloud2 to the specified TF frame                                                                                                                                                                              |
| `--tf-mode <static\|dynamic>` | TF mode: `static` (default) or `dynamic`                                                                                                                                                                                     |
| `--merge`                     | Merge bag files from distributed log modules and convert in a single pass. Accepts multiple input directories. The last positional argument is the output directory.                                                         |
| `--delete`                    | Delete source bag files after successful processing. In merge mode, deletes the original input bag files after each group is successfully merged and converted.                                                              |

The `--base-frame` option transforms all output PointCloud2 messages to the specified coordinate frame using TF data (`tf2_msgs/msg/TFMessage`) from the input bag. The `--tf-mode` option controls how TF data is handled:

- **static** (default): Only the first TF message(s) are read. The same fixed transform is applied to all point clouds. Suitable when the sensor mount does not change.
- **dynamic**: All TF messages are pre-loaded. Each point cloud is transformed using the time-dependent TF lookup matching its timestamp. Required when the TF tree changes over time.

In both modes, TF data is pre-loaded from the bag before processing begins, so transforms are always available even if TF messages appear after point cloud messages in the bag. The conversion fails if the specified frame is not found or if any point cloud cannot be transformed.

### Merge mode

When `--merge` is specified, the tool merges and converts bag files from one or more input directories in a single pass. For each merge group, messages from all input bags are k-way merged in timestamp order while simultaneously decoding LiDAR packet topics to PointCloud2.

Input files must follow the naming pattern:

```text
<sensing_system_id>_<module_id>_<rest>.(mcap|db3|sqlite3)
```

- `sensing_system_id`: Vehicle identifier (no underscores)
- `module_id`: Log module / ECU identifier (no underscores)
- `rest`: Remaining part (e.g., timestamp), can contain underscores

Files with the same `sensing_system_id` and `rest` are grouped and merged into a single output bag. The output filename drops `module_id`: `<sensing_system_id>_<rest>.<ext>`. Messages are interleaved in timestamp order. Files that do not match the pattern or groups with only a single file are skipped.

All conversion options (e.g., `--point-type`, `--base-frame`) are applied during the merge. Use `--delete` to remove the original input bag files after each group is successfully processed.

### Examples

```shell
# Basic conversion
./bag_converter input.mcap output.mcap

# Show help
./bag_converter --help

# Specify output point type (xyzi without timestamp field)
./bag_converter input.mcap output.mcap --point-type xyzi

# Keep original packet topics in output
./bag_converter input.mcap output.mcap --keep-original

# Convert all bag files in a directory (batch mode)
./bag_converter /path/to/input_dir /path/to/output_dir

# Batch mode with options
./bag_converter /path/to/input_dir /path/to/output_dir --point-type xyzi

# Transform point clouds to base_link frame (static TF, default)
./bag_converter input.mcap output.mcap --base-frame base_link

# Convert and delete source file after success
./bag_converter input.mcap output.mcap --delete

# Merge + convert from multiple input directories
./bag_converter /path/to/input_dir_0 /path/to/input_dir_1 /path/to/output_dir --merge

# Merge + convert with options
./bag_converter /path/to/input_dir /path/to/output_dir --merge --point-type xyzi

# Merge + convert and delete original input files
./bag_converter /path/to/input_dir /path/to/output_dir --merge --delete
```

## Message Types

### Input

The tool supports two input message types:

#### NebulaPackets

The input bag file contains `nebula_msgs::msg::NebulaPackets` messages on topics ending with `/nebula_packets`. This format is used when recording with the **Nebula** universal lidar driver (using its Seyond driver for Seyond LiDAR sensors). The messages contain raw packet data.

#### SeyondScan

The input bag file contains `seyond::msg::SeyondScan` messages on topics ending with `/seyond_packets`. This format is used when recording with the **official Seyond SDK**. The messages contain scan data from Seyond LiDAR sensors.

### Output: PointCloud2

The output bag file contains `sensor_msgs::msg::PointCloud2` messages on topics ending with `/nebula_points` (for Nebula input) or `/seyond_points` (for Seyond input). The PointCloud2 messages have the following structure:

#### Fields

**Normal fields** (present for point types xyzit, xyzi, en_xyzit):

- `x` (float32): X coordinate in meters
- `y` (float32): Y coordinate in meters
- `z` (float32): Z coordinate in meters
- `intensity` (float32): Intensity value
- `t_us` (uint32): **[DEPRECATED, will be removed in v0.6.0]** Replaced by `timestamp`. Relative timestamp in microseconds from the scan start time (xyzit, en_xyzit only)
- `timestamp` (uint32): Relative timestamp in nanoseconds from the scan start time (xyzit, en_xyzit only)

**Extended fields** (experimental; en_xyzit only; subject to change). Support depends on the input message type and decoder. For the `flags` mask, bit layout, and "value 0 when not supported" semantics, see [docs/flags.md](docs/flags.md).

| Property            | Type   | SeyondScan | NebulaPackets | Description / values                                    |
| ------------------- | ------ | ---------- | ------------- | ------------------------------------------------------- |
| `flags`             | uint16 | ✓          | ✓             | Availability mask; see [flags.md](docs/flags.md)        |
| `refl_type`         | uint8  | ✓          | —             | Point classification: 0 = normal, 1 = ground, 2 = fog   |
| `elongation`        | uint8  | ✓          | —             | Raw elongation 0–15                                     |
| `lidar_status`      | uint8  | ✓          | —             | 0 = none, 1 = transition, 2 = normal, 3 = failed        |
| `lidar_mode`        | uint8  | ✓          | —             | 1 = sleep, 2 = standby, 3 = work_normal, 6 = protection |
| `pkt_version_major` | uint8  | ✓          | ✓             | Packet protocol major (0–255)                           |
| `pkt_version_minor` | uint8  | ✓          | ✓             | Packet protocol minor (0–255)                           |
| `lidar_type`        | uint8  | ✓          | ✓             | Seyond LiDAR only. [Values](docs/lidar_type.md)         |

#### Timestamps

All timestamps in PointCloud2 messages are based on UTC (Coordinated Universal Time).

- **`header.stamp`**: Absolute timestamp representing the scan start time in UTC. This timestamp is generated by the LiDAR and represents when the scan began.
- **`t_us` field**: **[DEPRECATED, will be removed in v0.6.0]** Replaced by `timestamp`. Relative timestamp in microseconds from the scan start time (`header.stamp`)
- **`timestamp` field**: Relative timestamp in nanoseconds from the scan start time (`header.stamp`)

The absolute timestamp for each point (in nanoseconds, UTC) can be calculated as:

```text
absolute_timestamp_ns = (header.stamp.sec * 1,000,000,000 + header.stamp.nanosec) + timestamp
```

Or, using the deprecated `t_us` field:

```text
absolute_timestamp_ns = (header.stamp.sec * 1,000,000,000 + header.stamp.nanosec) + (t_us * 1,000)
```

## Utility Scripts

The `scripts/` directory contains Python scripts for inspecting and debugging rosbag files. See [scripts/README.md](scripts/README.md) for details.
