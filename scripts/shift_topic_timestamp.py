#!/usr/bin/env python3
"""
Script to shift message receive timestamps for specified topics in rosbag2 files.

This script reads a rosbag2 file and creates a new rosbag2 file with the
receive timestamps of specified topics shifted by given offsets.
"""

import argparse
import re
import shutil
import tempfile
from pathlib import Path

import rosbag2_py


def parse_topic_offset(spec: str) -> tuple[str, int]:
    """
    Parse a topic:offset specification.

    Args:
        spec: String in format "<topic_name>:<offset><unit>"
              e.g., "/sensing/hoge:-50ms", "/camera:10ms", "/lidar:1s"

    Returns:
        Tuple of (topic_name, offset_ns)

    Raises:
        ValueError: If the specification format is invalid
    """
    # Match pattern: topic_name:offset_with_unit
    # Topic name can contain /, _, letters, numbers
    # Offset is a number (possibly negative) followed by a unit
    match = re.match(r'^(.+):(-?\d+(?:\.\d+)?)(ms|s|us|ns)$', spec)
    if not match:
        raise ValueError(
            f"Invalid format: '{spec}'. Expected format: <topic_name>:<offset><unit> "
            f"(e.g., /sensing/hoge:-50ms, /camera:10ms, /lidar:1s)"
        )

    topic_name = match.group(1)
    offset_value = float(match.group(2))
    unit = match.group(3)

    # Convert to nanoseconds
    if unit == 'ns':
        offset_ns = int(offset_value)
    elif unit == 'us':
        offset_ns = int(offset_value * 1_000)
    elif unit == 'ms':
        offset_ns = int(offset_value * 1_000_000)
    elif unit == 's':
        offset_ns = int(offset_value * 1_000_000_000)
    else:
        raise ValueError(f"Unknown unit: {unit}")

    return topic_name, offset_ns


def shift_topic_timestamps(
    input_bag_path: str,
    output_bag_path: str,
    topic_offsets: dict[str, int],
):
    """
    Shift message receive timestamps for specified topics.

    Args:
        input_bag_path: Path to the input rosbag2 directory or file
        output_bag_path: Path to the output rosbag2 directory or file
        topic_offsets: Dictionary mapping topic names to offset in nanoseconds
    """
    # Create reader
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=input_bag_path, storage_id='')
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr'
    )
    reader.open(storage_options, converter_options)

    # Get topic metadata
    topic_types = reader.get_all_topics_and_types()

    # Check if the specified topics exist
    available_topics = [t.name for t in topic_types]
    target_topics = set(topic_offsets.keys())
    missing_topics = target_topics - set(available_topics)
    if missing_topics:
        print("Error: The following topics were not found in bag file:")
        for name in missing_topics:
            print(f"  - {name}")
        print("Available topics:")
        for name in available_topics:
            print(f"  - {name}")
        return

    # Create writer in a temporary directory, then move the mcap file to output path
    output_path = Path(output_bag_path)
    temp_dir = tempfile.mkdtemp()
    temp_bag_dir = Path(temp_dir) / "bag"

    try:
        writer = rosbag2_py.SequentialWriter()
        output_storage_options = rosbag2_py.StorageOptions(
            uri=str(temp_bag_dir),
            storage_id='mcap',
        )
        writer.open(output_storage_options, converter_options)

        # Register all topics with the writer
        for topic_metadata in topic_types:
            writer.create_topic(topic_metadata)

        # Process messages
        message_count = 0
        shifted_count = 0

        print(f"Processing bag file: {input_bag_path}")
        print(f"Output bag file: {output_bag_path}")
        print("Topic offsets:")
        for topic, offset_ns in topic_offsets.items():
            offset_ms = offset_ns / 1_000_000
            print(f"  {topic}: {offset_ms:+.3f} ms")
        print()

        while reader.has_next():
            (topic, data, timestamp) = reader.read_next()
            message_count += 1

            # Shift timestamp if this is one of the target topics
            if topic in topic_offsets:
                offset_ns = topic_offsets[topic]
                new_timestamp = timestamp + offset_ns
                # Ensure timestamp doesn't go negative
                if new_timestamp < 0:
                    print(f"Warning: Timestamp would become negative at message #{message_count}, clamping to 0")
                    new_timestamp = 0
                writer.write(topic, data, new_timestamp)
                shifted_count += 1
            else:
                writer.write(topic, data, timestamp)

        # Close writer to flush data
        del writer

        # Find the mcap file in temp directory and move it to output path
        mcap_files = list(temp_bag_dir.glob("*.mcap"))
        if mcap_files:
            shutil.move(str(mcap_files[0]), str(output_path))
        else:
            print("Error: No mcap file was created")
            return

        print(f"Total messages processed: {message_count}")
        print(f"Messages with shifted timestamps: {shifted_count}")
        print(f"Output written to: {output_bag_path}")

    finally:
        # Clean up temporary directory
        shutil.rmtree(temp_dir, ignore_errors=True)


def main():
    """Main function."""
    parser = argparse.ArgumentParser(
        description='Shift message receive timestamps for specified topics in rosbag2',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Format:
  <topic_name>:<offset><unit>

  Supported units: ns (nanoseconds), us (microseconds), ms (milliseconds), s (seconds)

Examples:
  # Shift /sensing/hoge backward by 50 ms
  %(prog)s input.mcap output.mcap /sensing/hoge:-50ms

  # Shift multiple topics with different offsets
  %(prog)s input.mcap output.mcap /sensing/hoge:-50ms /sensing/camera0:10ms

  # Using different units
  %(prog)s input.mcap output.mcap /lidar:1s /camera:-500us
        """
    )

    parser.add_argument(
        'input_bag',
        type=str,
        help='Path to the input rosbag2 directory or file'
    )

    parser.add_argument(
        'output_bag',
        type=str,
        help='Path to the output mcap file (e.g., output.mcap)'
    )

    parser.add_argument(
        'topic_offsets',
        type=str,
        nargs='+',
        metavar='TOPIC:OFFSET',
        help='Topic and offset pairs in format <topic_name>:<offset><unit> (e.g., /sensing/hoge:-50ms)'
    )

    args = parser.parse_args()

    if not Path(args.input_bag).exists():
        parser.error(f"Input bag path does not exist: {args.input_bag}")

    if Path(args.output_bag).exists():
        parser.error(f"Output bag path already exists: {args.output_bag}")

    # Parse topic:offset specifications
    topic_offsets = {}
    for spec in args.topic_offsets:
        try:
            topic_name, offset_ns = parse_topic_offset(spec)
            topic_offsets[topic_name] = offset_ns
        except ValueError as e:
            parser.error(str(e))

    shift_topic_timestamps(
        args.input_bag,
        args.output_bag,
        topic_offsets,
    )


if __name__ == '__main__':
    main()
