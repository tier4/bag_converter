# Scripts

Utility scripts for inspecting and debugging rosbag files.

## Prerequisites

- ROS 2 Humble (or later) environment sourced
- Python 3.10+

## Install Dependencies

```bash
# 1. Source ROS 2 environment (provides rclpy, rosbag2_py, sensor_msgs, etc.)
source /opt/ros/humble/setup.bash

# 2. Install additional Python dependencies
pip install --user -r requirements.txt
```

## Scripts

### decode_pointcloud2.py

Decode and inspect PointCloud2 messages from rosbag files.

```bash
# Decode all PointCloud2 topics
python3 scripts/decode_pointcloud2.py /path/to/bag.mcap

# Show every 5000th point
python3 scripts/decode_pointcloud2.py /path/to/bag.mcap --point-step 5000
```
