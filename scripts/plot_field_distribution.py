#!/usr/bin/env python3
"""
Plot the value distribution of a specified PointCloud2 field from a rosbag file.

All PointCloud2 topics in the bag are checked for the specified field.
Topics that contain the field are overlaid on a single histogram.
Topics that do not contain the field are skipped with a warning.
"""

import argparse
import math
import signal
import sys
from collections import defaultdict
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import rclpy
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2 as pc2
import rosbag2_py


POINTFIELD_DTYPE_MAP = {
    PointField.FLOAT32: np.float32,
    PointField.FLOAT64: np.float64,
    PointField.UINT32: np.uint32,
    PointField.INT32: np.int32,
    PointField.UINT8: np.uint8,
    PointField.INT8: np.int8,
    PointField.UINT16: np.uint16,
    PointField.INT16: np.int16,
}


def read_field_values(bag_path: str, field_name: str) -> dict[str, np.ndarray]:
    """
    Read values of a specific field from all PointCloud2 topics in a rosbag.

    Args:
        bag_path: Path to the rosbag2 directory or file
        field_name: The PointCloud2 field to extract (e.g., "intensity")

    Returns:
        Dictionary mapping topic name to numpy array of field values.
        Topics without the specified field are excluded.
    """
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='')
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr',
    )
    reader.open(storage_options, converter_options)

    topic_types = reader.get_all_topics_and_types()
    pc2_topics = {
        t.name for t in topic_types
        if t.type == 'sensor_msgs/msg/PointCloud2'
    }

    if not pc2_topics:
        print("No PointCloud2 topics found in the bag file.")
        return {}

    print(f"Found {len(pc2_topics)} PointCloud2 topic(s):")
    for topic in sorted(pc2_topics):
        print(f"  - {topic}")

    msg_type = get_message('sensor_msgs/msg/PointCloud2')

    # Track which topics have the requested field (checked on first message)
    topic_has_field: dict[str, bool] = {}
    topic_field_dtype: dict[str, np.dtype] = {}
    values_per_topic: dict[str, list[np.ndarray]] = defaultdict(list)
    msg_count_per_topic: dict[str, int] = defaultdict(int)

    while reader.has_next():
        (topic, data, _timestamp) = reader.read_next()

        if topic not in pc2_topics:
            continue

        msg = deserialize_message(data, msg_type)
        if not isinstance(msg, PointCloud2):
            continue

        # Check field existence on first message per topic
        if topic not in topic_has_field:
            available = {f.name: f for f in msg.fields}
            if field_name not in available:
                print(f"  Warning: '{field_name}' not found in {topic} "
                      f"(available: {', '.join(available.keys())}). Skipping.")
                topic_has_field[topic] = False
                continue
            topic_has_field[topic] = True
            field = available[field_name]
            topic_field_dtype[topic] = POINTFIELD_DTYPE_MAP.get(
                field.datatype, np.float32)

        if not topic_has_field[topic]:
            continue

        # Extract field values
        points_gen = pc2.read_points(msg, field_names=[field_name], skip_nans=True)
        arr = np.fromiter(
            (p[0] for p in points_gen),
            dtype=topic_field_dtype[topic],
        )
        if len(arr) > 0:
            values_per_topic[topic].append(arr)
        msg_count_per_topic[topic] += 1

    # Concatenate arrays per topic
    result = {}
    for topic, arrays in values_per_topic.items():
        combined = np.concatenate(arrays)
        result[topic] = combined
        print(f"  {topic}: {msg_count_per_topic[topic]} messages, "
              f"{len(combined)} points, "
              f"range=[{combined.min()}, {combined.max()}]")

    return result


def plot_distribution(
    values_per_topic: dict[str, np.ndarray],
    field_name: str,
    bag_name: str,
    bins: int = 50,
    output_path: str | None = None,
):
    """
    Plot per-topic histograms as subplots in a single figure.

    Args:
        values_per_topic: Dictionary mapping topic name to numpy array of values
        field_name: Field name (used for title/label)
        bins: Number of bins (default: 50)
        output_path: If set, save plot to this path instead of showing
    """
    if not values_per_topic:
        print("No data to plot.")
        return

    topics = sorted(values_per_topic.keys())
    n_topics = len(topics)

    # Grid layout as close to square as possible
    ncols = math.ceil(math.sqrt(n_topics))
    nrows = math.ceil(n_topics / ncols)

    fig, axes = plt.subplots(nrows, ncols, figsize=(5 * ncols, 4 * nrows),
                             squeeze=False)

    for i, topic in enumerate(topics):
        r, c = divmod(i, ncols)
        ax = axes[r, c]
        values = values_per_topic[topic]
        ax.hist(values, bins=bins, alpha=0.7)
        ax.set_yscale('log')
        ax.set_ylabel('Count')
        ax.set_xlabel(field_name)
        ax.set_title(topic)

    # Hide unused subplots
    for i in range(n_topics, nrows * ncols):
        r, c = divmod(i, ncols)
        axes[r, c].set_visible(False)

    fig.suptitle(f'Distribution of "{field_name}"\n{bag_name}',
                 fontsize=14)
    fig.canvas.manager.set_window_title(f'{field_name} distribution')
    fig.tight_layout()

    if output_path:
        fig.savefig(output_path, dpi=150, bbox_inches='tight')
        print(f"Saved to {output_path}")
    else:
        plt.show()

    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(
        description='Plot value distribution of a PointCloud2 field from a rosbag',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s /path/to/bag.mcap intensity
  %(prog)s /path/to/bag.mcap intensity --bins 100
  %(prog)s /path/to/bag.mcap intensity -o distribution.png
        """,
    )

    parser.add_argument('bag_path', type=str, help='Path to the rosbag2 file')
    parser.add_argument('field', type=str, help='PointCloud2 field name to plot (e.g., intensity, x, t_us)')

    parser.add_argument('--bins', type=int, default=50, help='Number of bins (default: 50)')

    parser.add_argument('-o', '--output', type=str, default=None, metavar='FILE',
                        help='Save plot to file instead of showing interactively')

    args = parser.parse_args()

    if not Path(args.bag_path).exists():
        parser.error(f"Bag path does not exist: {args.bag_path}")

    signal.signal(signal.SIGINT, lambda *_: sys.exit(130))

    rclpy.init()
    try:
        values = read_field_values(args.bag_path, args.field)
        bag_name = Path(args.bag_path).name
        plot_distribution(values, args.field, bag_name=bag_name,
                          bins=args.bins, output_path=args.output)
    finally:
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
