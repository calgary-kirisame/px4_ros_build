#!/usr/bin/env python
"""Launch one GPS-backed 5 m square waypoint mission."""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='px4_single_plan',
            executable='start_mission_gate',
            name='start_mission_gate',
            namespace='px4_single_plan',
            output='screen',
        ),
        Node(
            package='px4_single_plan',
            executable='offboard_waypoint_runner',
            name='offboard_waypoint_runner_square_5m',
            namespace='px4_single_plan',
            parameters=[{
                'vehicle_namespace': '',
                'waypoint_file': 'waypoints_square_5m.yaml',
                'field_config_file': 'square_5m_field_gps.yaml',
                'takeoff_altitude_m': 5.0,
                'send_takeoff_command': False,
                'waypoint_z_relative_to_launch': True,
                'terminate_return_altitude_m': 5.0,
                'wait_for_start_confirmation': True,
                'camera_image_topic': '/camera/image_raw',
            }],
        ),
    ])
