"""Launch file for Phi Motion Torso node"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from phi_motion_torso.controller import LiftConfig


def generate_launch_description():
    """Generate launch description for torso control node"""
    cfg = LiftConfig()

    # Declare launch arguments
    can_channel_arg = DeclareLaunchArgument(
        'can_channel',
        default_value=cfg.channel,
        description='CAN interface name'
    )

    can_bitrate_arg = DeclareLaunchArgument(
        'can_bitrate',
        default_value=str(cfg.bitrate),
        description='CAN bitrate'
    )

    node_id_arg = DeclareLaunchArgument(
        'node_id',
        default_value=str(cfg.node_id),
        description='CANopen node ID'
    )

    status_rate_arg = DeclareLaunchArgument(
        'status_publish_rate',
        default_value='50.0',
        description='Status publishing rate in Hz'
    )

    eds_file_arg = DeclareLaunchArgument(
        'eds_file',
        default_value=cfg.eds_file,
        description='Path to EDS file'
    )

    control_mode_arg = DeclareLaunchArgument(
        'control_mode',
        default_value='action',
        description='Control mode: action or topic'
    )

    # Create node
    torso_node = Node(
        package='phi_motion_torso',
        executable='torso_node.py',
        name='phi_motion_torso_node',
        output='screen',
        parameters=[{
            'can_channel': LaunchConfiguration('can_channel'),
            'can_bitrate': LaunchConfiguration('can_bitrate'),
            'node_id': LaunchConfiguration('node_id'),
            'status_publish_rate': LaunchConfiguration('status_publish_rate'),
            'eds_file': LaunchConfiguration('eds_file'),
            'control_mode': LaunchConfiguration('control_mode'),
        }]
    )

    return LaunchDescription([
        can_channel_arg,
        can_bitrate_arg,
        node_id_arg,
        status_rate_arg,
        eds_file_arg,
        control_mode_arg,
        torso_node,
    ])
