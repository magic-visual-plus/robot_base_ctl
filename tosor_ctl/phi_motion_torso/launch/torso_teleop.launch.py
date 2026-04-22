"""Launch file for Phi Torso teleop node (whole_body -> MoveToHeight)."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("phi_motion_torso")
    default_params = os.path.join(pkg_share, "config", "torso_teleop_node.yaml")

    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=default_params,
        description="Path to teleop node params YAML",
    )

    teleop_node = Node(
        package="phi_motion_torso",
        executable="torso_teleop_node.py",
        name="phi_torso_teleop_node",
        output="screen",
        parameters=[LaunchConfiguration("params_file")],
    )

    return LaunchDescription([params_file_arg, teleop_node])

