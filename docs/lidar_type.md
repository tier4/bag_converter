# LiDAR Type

This field is supported for Seyond LiDAR only. When not supported (e.g. other sensors), the value is **0** and must be ignored; check the `HAS_LIDAR_TYPE` bit in the en_xyzit [flags](flags.md) to know if the value is valid.

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
