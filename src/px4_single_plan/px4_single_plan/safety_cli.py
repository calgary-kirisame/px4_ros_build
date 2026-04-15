#!/usr/bin/env python
"""Simple safety CLI to arm/takeoff PX4 via px4_msgs VehicleCommand.

This CLI asks for explicit user confirmation before sending critical commands.
Run after sourcing ROS2 and workspace: `ros2 run px4_single_plan offboard_safety`
"""
import sys
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from px4_msgs.msg import VehicleCommand


class SafetyCLI(Node):
    def __init__(self):
        super().__init__('offboard_safety_cli')
        self.pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', 10)

    def publish_cmd(self, command, params=None):
        if params is None:
            params = [0.0]*7
        msg = VehicleCommand()
        msg.command = int(command)
        msg.param1 = float(params[0])
        msg.param2 = float(params[1])
        msg.param3 = float(params[2])
        msg.param4 = float(params[3])
        msg.param5 = float(params[4])
        msg.param6 = float(params[5])
        msg.param7 = float(params[6])
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(Clock().now().nanoseconds / 1000)
        self.pub.publish(msg)


def main(argv=None):
    rclpy.init(args=argv)
    node = SafetyCLI()

    try:
        print('\nPX4 Safety CLI')
        print("Type 'ARM' to arm, 'DISARM' to disarm, 'TAKEOFF' to request takeoff, or 'QUIT' to exit.")
        while True:
            cmd = input('> ').strip().upper()
            if cmd == 'ARM':
                confirm = input("Confirm ARM (type 'YES' to proceed): ")
                if confirm.strip().upper() == 'YES':
                    node.publish_cmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, [1.0])
                    print('ARM command sent')
                else:
                    print('ARM cancelled')
            elif cmd == 'DISARM':
                confirm = input("Confirm DISARM (type 'YES' to proceed): ")
                if confirm.strip().upper() == 'YES':
                    node.publish_cmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, [0.0])
                    print('DISARM command sent')
                else:
                    print('DISARM cancelled')
            elif cmd == 'TAKEOFF':
                try:
                    alt = float(input('Takeoff altitude (m) [default 5.0]: ') or '5.0')
                except ValueError:
                    alt = 5.0
                confirm = input(f"Confirm TAKEOFF to {alt} m (type 'YES' to proceed): ")
                if confirm.strip().upper() == 'YES':
                    # VEHICLE_CMD_NAV_TAKEOFF param7 = altitude
                    node.publish_cmd(VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF, [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, alt])
                    print('TAKEOFF command sent')
                else:
                    print('TAKEOFF cancelled')
            elif cmd == 'QUIT' or cmd == 'EXIT':
                break
            elif cmd == 'HELP' or cmd == '?':
                print("Available: ARM, DISARM, TAKEOFF, QUIT")
            else:
                print('Unknown command; type HELP for available commands')

    except (KeyboardInterrupt, EOFError):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
