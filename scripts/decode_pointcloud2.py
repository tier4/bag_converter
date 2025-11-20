#!/usr/bin/env python3
"""
Script to decode PointCloud2 messages from rosbag2 files using rosbag2_py.

This script can decode any PointCloud2 message format by automatically detecting
and reading all available fields. It supports:
- Reading PointCloud2 messages from rosbag2 files
- Automatic field detection and decoding
- Displaying point data with configurable number of points per message
- Handling relative timestamps (e.g., t_us field) and calculating absolute timestamps
"""

import argparse
from pathlib import Path

import numpy as np
import rclpy
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2 as pc2
import rosbag2_py


def decode_pointcloud2(bag_path: str, topic_name: str = None, max_points_to_show: int = 100):
    """
    Decode PointCloud2 messages from a rosbag2 file.

    Args:
        bag_path: Path to the rosbag2 directory or file
        topic_name: Optional topic name to filter. If None, reads all PointCloud2 topics.
        max_points_to_show: Maximum number of points to display per message (default: 100)
    """
    # Initialize ROS 2 (required for message deserialization)
    rclpy.init()

    try:
        # Create reader
        reader = rosbag2_py.SequentialReader()
        
        # Set storage options
        storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='')
        converter_options = rosbag2_py.ConverterOptions(
            input_serialization_format='cdr',
            output_serialization_format='cdr'
        )
        
        reader.open(storage_options, converter_options)
        
        # Get topic types
        topic_types = reader.get_all_topics_and_types()
        
        # Filter PointCloud2 topics
        pc2_topics = {
            topic_metadata.name: topic_metadata.type
            for topic_metadata in topic_types
            if topic_metadata.type == 'sensor_msgs/msg/PointCloud2'
        }
        
        if not pc2_topics:
            print("No PointCloud2 topics found in the bag file.")
            return
        
        print(f"Found {len(pc2_topics)} PointCloud2 topic(s):")
        for topic in pc2_topics.keys():
            print(f"  - {topic}")
        
        # Select topic to read
        if topic_name:
            if topic_name not in pc2_topics:
                print(f"Error: Topic '{topic_name}' not found in bag file.")
                return
            selected_topics = {topic_name}
        else:
            selected_topics = set(pc2_topics.keys())
        
        # Set topic filter if needed
        if len(selected_topics) < len(pc2_topics):
            storage_filter = rosbag2_py.StorageFilter(topics=list(selected_topics))
            reader.set_filter(storage_filter)
        
        # Get message type
        msg_type = get_message('sensor_msgs/msg/PointCloud2')
        
        message_count = 0
        
        # Read messages
        while reader.has_next():
            (topic, data, timestamp) = reader.read_next()
            
            # Filter by topic if needed
            if topic not in selected_topics:
                continue
            
            # Deserialize message
            msg = deserialize_message(data, msg_type)
            
            if not isinstance(msg, PointCloud2):
                continue
            
            message_count += 1
            
            # Decode point cloud
            points = decode_point(msg)
            
            if points is not None:
                # Get actual number of points from first available field
                field_names = list(points.keys())
                if not field_names:
                    continue
                
                num_points = len(points[field_names[0]]) if len(points[field_names[0]]) > 0 else 0
                
                if num_points == 0:
                    continue
                
                # Calculate base timestamp in microseconds (if t_us field exists)
                base_time_us = None
                if 't_us' in points and len(points['t_us']) > 0:
                    base_time_us = msg.header.stamp.sec * 1_000_000 + msg.header.stamp.nanosec // 1_000
                
                # Convert timestamp from nanoseconds to seconds and nanoseconds
                received_sec = timestamp // 1_000_000_000
                received_nanosec = timestamp % 1_000_000_000
                
                print(f"\n[{topic}] Message #{message_count}")
                print(f"  Scan started at: {msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}")
                print(f"  Message received at: {received_sec}.{received_nanosec:09d}")
                print(f"  Frame ID: {msg.header.frame_id}")
                print(f"  Number of points decoded: {num_points}")
                print(f"  Available fields: {', '.join(field_names)}")
                
                # Show summary of first N points
                num_points_to_show = min(max_points_to_show, num_points)
                print(f"  First {num_points_to_show} points:")
                
                for i in range(num_points_to_show):
                    # Build point info string dynamically
                    point_info_parts = []
                    for field_name in field_names:
                        field_data = points[field_name]
                        if len(field_data) > i:
                            if field_name == 't_us' and base_time_us is not None:
                                # Calculate absolute timestamp for t_us
                                abs_time_us = base_time_us + field_data[i]
                                abs_time_sec = abs_time_us / 1_000_000.0
                                point_info_parts.append(f"{field_name}={field_data[i]}, abs_time={abs_time_sec:.6f}")
                            elif isinstance(field_data[i], (float, np.floating)):
                                point_info_parts.append(f"{field_name}={field_data[i]:.3f}")
                            else:
                                point_info_parts.append(f"{field_name}={field_data[i]}")
                    
                    print(f"    Point {i}: {', '.join(point_info_parts)}")
        
        print(f"\nTotal messages processed: {message_count}")
        
    finally:
        # Reader is automatically closed when it goes out of scope
        rclpy.shutdown()


def decode_point(msg: PointCloud2) -> dict:
    """
    Decode PointCloud2 message using sensor_msgs_py.

    Reads all available fields from the PointCloud2 message and returns them
    as a dictionary of numpy arrays.

    Note:
        msg.header.stamp represents the scan start timestamp (absolute time).
        If a field named 't_us' exists, it contains relative timestamps from
        the scan start in microseconds.

    Args:
        msg: PointCloud2 message (msg.header.stamp is the scan start timestamp)

    Returns:
        Dictionary with numpy arrays for all available fields.
        Field names match the PointCloud2 field names.
        Returns None if decoding fails.
    """
    try:
        # Check available fields
        available_fields = [f.name for f in msg.fields]
        
        if not available_fields:
            print("Error: No fields found in PointCloud2 message")
            return None
        
        # Read all available fields
        fields_to_read = available_fields
            
        # Read points using read_points which is more robust than read_points_numpy
        # for mixed data types
        points_gen = pc2.read_points(msg, field_names=fields_to_read, skip_nans=True)
        
        # Convert to list of tuples then to numpy structured array for easier handling
        points_list = list(points_gen)
        
        if not points_list:
            # Return empty dict with all available fields
            return {field: np.array([]) for field in available_fields}
        
        # Map PointField datatype to numpy dtype
        # PointField datatype constants: FLOAT32=7, UINT32=6, INT32=5, etc.
        def field_datatype_to_numpy(field):
            """Convert PointField datatype to numpy dtype."""
            if field.datatype == PointField.FLOAT32:
                return np.float32
            elif field.datatype == PointField.FLOAT64:
                return np.float64
            elif field.datatype == PointField.UINT32:
                return np.uint32
            elif field.datatype == PointField.INT32:
                return np.int32
            elif field.datatype == PointField.UINT8:
                return np.uint8
            elif field.datatype == PointField.INT8:
                return np.int8
            elif field.datatype == PointField.UINT16:
                return np.uint16
            elif field.datatype == PointField.INT16:
                return np.int16
            else:
                # Default to float32
                return np.float32
        
        # Create structured array dtype from available fields
        dtype_list = []
        field_map = {f.name: f for f in msg.fields}
        for field_name in fields_to_read:
            if field_name in field_map:
                field = field_map[field_name]
                dtype_list.append((field_name, field_datatype_to_numpy(field)))
        
        cloud_array = np.array(points_list, dtype=dtype_list)
        
        # Extract all available fields dynamically
        result = {}
        for field_name in fields_to_read:
            if field_name in cloud_array.dtype.names:
                result[field_name] = cloud_array[field_name]
            else:
                # Field not found in array, create empty array
                result[field_name] = np.array([])
        
        return result
        
    except Exception as e:
        print(f"Error decoding PointCloud2: {e}")
        import traceback
        traceback.print_exc()
        return None


def main():
    """Main function."""
    parser = argparse.ArgumentParser(
        description='Decode PointCloud2 messages with PointXYZIT from rosbag2',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s /path/to/bag
  %(prog)s /path/to/bag --topic /sensing/lidar/top/nebula_points
        """
    )
    
    parser.add_argument(
        'bag_path',
        type=str,
        help='Path to the rosbag2 directory or file'
    )
    
    parser.add_argument(
        '--topic',
        type=str,
        default=None,
        help='Optional topic name to filter. If not specified, reads all PointCloud2 topics.'
    )
    
    parser.add_argument(
        '--max-points-to-show',
        type=int,
        default=100,
        metavar='N',
        help='Maximum number of points to display per message (default: 10)'
    )
    
    args = parser.parse_args()
    
    bag_path = args.bag_path
    topic_name = args.topic
    max_points_to_show = args.max_points_to_show
    
    if not Path(bag_path).exists():
        parser.error(f"Bag path does not exist: {bag_path}")
    
    if max_points_to_show < 0:
        parser.error("--max-points-to-show must be non-negative")
    
    decode_pointcloud2(bag_path, topic_name, max_points_to_show)


if __name__ == '__main__':
    main()

