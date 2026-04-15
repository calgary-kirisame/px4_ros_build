#!/usr/bin/env python
"""Launch velocity control node."""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='px4_single_plan',
            namespace='px4_single_plan',
            executable='velocity_control',
            name='velocity',
        ),
    ])
