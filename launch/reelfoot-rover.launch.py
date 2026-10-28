#!/usr/bin/env python3
"""Launch all reelfoot nodes: drive, encoder, and sonar_ring.
This file is installed to `share/reelfoot/launch/` by the setup.py.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    drive_node = Node(
        package='reelfoot',
        executable='drive',
        name='drive_node',
        output='screen',
    )

    encoder_node = Node(
        package='reelfoot',
        executable='encoder_node',
        name='encoder_node',
        output='screen',
    )

    sonar_node = Node(
        package='reelfoot',
        executable='sonar_ring',
        name='sonar_ring_node',
        output='screen',
    )

    return LaunchDescription([
        drive_node,
        encoder_node,
        sonar_node,
    ])
