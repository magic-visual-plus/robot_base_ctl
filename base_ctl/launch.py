#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # 声明参数
    cmd_topic_arg = DeclareLaunchArgument(
        'cmd_topic',
        default_value='/cmd_vel',
        description='Command velocity topic'
    )

    return LaunchDescription([
        cmd_topic_arg,

        # 启动 bridge.py - 底盘控制桥接节点
        Node(
            package='robot_base_ctl',
            executable='bridge.py',
            name='lekiwi_base_bridge',
            output='screen',
            parameters=[{
                'cmd_topic': LaunchConfiguration('cmd_topic'),
                'vx_max': 0.2,
                'vy_max': 0.2,
                'wz_max': 1.0,
                'can_channel': 'can0',
                'bitrate': 1000000,
                # 'eds_path': '/home/nvidia/github/robot_base_ctl/motor/moons/CANOPEN-EDS-MBDV-Servo-SingleAxis-V1.1.1.eds',
                # 'encoder_cpr': 65536,  # 2**16
                # 'gear_ratio': 10.0,
                # 'wheel_radius': 0.1015,
                # 'base_radius': 0.203,
                # 'max_wheel_rad_s': 6.283185307179586,  # 2.0 * pi
                # 'dir_left': 1,
                # 'dir_back': 1,
                # 'dir_right': 1,
            }],
        ),

        # 启动 zoh_rev.py - 辅助控制器节点
        Node(
            package='robot_base_ctl',
            executable='zoh_rev.py',
            name='assist_controller_pose_twist',
            output='screen',
        ),
    ])
