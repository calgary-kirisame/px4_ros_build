#!/usr/bin/env python
"""Launch the offboard waypoint runner and safety CLI."""
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='px4_single_plan',
            executable='offboard_waypoint_runner',
            name='offboard_waypoint_runner',
            namespace='px4_single_plan',
        ),
        ExecuteProcess(
            cmd=['ros2', 'run', 'px4_single_plan', 'offboard_safety'],
            shell=False,
            output='screen',
        ),
    ])
