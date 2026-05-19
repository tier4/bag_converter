# Intensity scaling (Seyond / Inno packets)

Decoder: **`SeyondPCDDecoder`** — [seyond_decoder.cpp](../src/bag_converter/src/seyond_decoder.cpp) (`data_packet_parse` sets `robin_w_intensity_scale_`, `point_xyz_data_parse` applies it). `use_reflectance` only selects the source field; scaling is unchanged.

| Model (`lidar_type`) | Packet `major_version` | Raw range (sensor field) | Scale        | Stored `float32` |
| -------------------- | ---------------------- | ------------------------ | ------------ | ---------------- |
| Not Robin W          | any                    | [0, 255]                 | none         | [0, 255]         |
| Robin W              | ≤ 3                    | [0, 4095]                | × (255/4095) | [0, 255]         |
| Robin W              | ≥ 4                    | [0, 255]                 | × (255/255)  | [0, 255]         |

After scaling (or when none), values are clamped to [0, 255] and written as `float`.

## Disabling intensity scaling

Pass `--no-intensity-scaling` on the CLI to bypass both the Robin W multiplier and the [0, 255] clamp. The decoder then writes the raw sensor field value (`InnoEnXyzPoint::intensity` / `reflectance`, or `InnoXyzPoint::refl`) into `point.intensity` as a `float`. This is useful for raw-data analysis or debugging; downstream consumers must be ready to handle values outside [0, 255] (Robin W raw ≤ 4095, Falcon raw ≤ 65535).

The flag has no effect on the Nebula decoder, which does not perform intensity scaling.

Packet header layout: [seyond_packet_protocol_versions.md](seyond_packet_protocol_versions.md).
