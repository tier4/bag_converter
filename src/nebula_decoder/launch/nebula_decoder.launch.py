from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for Nebula Decoder node."""

    # Declare launch arguments
    sensor_model_arg = DeclareLaunchArgument(
        'sensor_model',
        default_value='Robin_W',
        description='Sensor model (e.g., Robin_W, Falcon_Kinetic)'
    )

    return_mode_arg = DeclareLaunchArgument(
        'return_mode',
        default_value='Dual',
        description='Return mode (e.g., Dual, Single)'
    )

    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='seyond',
        description='Frame ID for output point cloud'
    )

    min_range_arg = DeclareLaunchArgument(
        'min_range',
        default_value='0.3',
        description='Minimum range in meters'
    )

    max_range_arg = DeclareLaunchArgument(
        'max_range',
        default_value='200.0',
        description='Maximum range in meters'
    )

    scan_phase_arg = DeclareLaunchArgument(
        'scan_phase',
        default_value='0.0',
        description='Scan phase offset'
    )

    frequency_ms_arg = DeclareLaunchArgument(
        'frequency_ms',
        default_value='100.0',
        description='Scan frequency in milliseconds'
    )

    use_sensor_time_arg = DeclareLaunchArgument(
        'use_sensor_time',
        default_value='true',
        description='Use sensor timestamp instead of system time'
    )

    # Create the node
    nebula_decoder_node = Node(
        package='nebula_decoder',
        executable='nebula_decoder_node_exe',
        name='nebula_decoder',
        output='screen',
        parameters=[{
            'sensor_model': LaunchConfiguration('sensor_model'),
            'return_mode': LaunchConfiguration('return_mode'),
            'frame_id': LaunchConfiguration('frame_id'),
            'min_range': LaunchConfiguration('min_range'),
            'max_range': LaunchConfiguration('max_range'),
            'scan_phase': LaunchConfiguration('scan_phase'),
            'frequency_ms': LaunchConfiguration('frequency_ms'),
            'use_sensor_time': LaunchConfiguration('use_sensor_time'),
        }],
        remappings=[
            ('~/nebula_packets', '/nebula_packets'),
            ('~/nebula_points', '/nebula_points'),
        ]
    )

    return LaunchDescription([
        sensor_model_arg,
        return_mode_arg,
        frame_id_arg,
        min_range_arg,
        max_range_arg,
        scan_phase_arg,
        frequency_ms_arg,
        use_sensor_time_arg,
        nebula_decoder_node,
    ])
