#!/usr/bin/env python
"""Launch 4 GPS-backed quarter-field waypoint runners for real flight (test config)."""
from launch import LaunchDescription
from launch_ros.actions import Node
from px4_single_plan.field_config import DEFAULT_FIELD_CONFIG_FILE

FIELD_CONFIG_FILE = DEFAULT_FIELD_CONFIG_FILE


def generate_launch_description():
    ld = LaunchDescription()

    ld.add_action(
        Node(
            package='px4_single_plan',
            executable='start_mission_gate',
            name='start_mission_gate',
            namespace='px4_single_plan',
            output='screen',
        )
    )

    waypoint_files = [
        'waypoints_q4_1.yaml',
        'waypoints_q4_2.yaml',
        'waypoints_q4_3.yaml',
        'waypoints_q4_4.yaml',
    ]
    for idx, waypoint_file in enumerate(waypoint_files):
        namespace = f'px4_{idx}'
        ld.add_action(
            Node(
                package='px4_single_plan',
                executable='offboard_waypoint_runner',
                name=f'offboard_waypoint_runner_{idx}',
                namespace='px4_single_plan',
                parameters=[{
                    'vehicle_namespace': namespace,
                    'waypoint_file': waypoint_file,
                    'field_config_file': FIELD_CONFIG_FILE,
                    'wait_for_start_confirmation': True,
                    'camera_image_topic': f'/{namespace}/camera/image_raw',
                }],
            )
        )

    return ld
