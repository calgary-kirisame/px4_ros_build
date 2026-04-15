#!/usr/bin/env python
"""Simple offboard waypoint runner using px4_msgs (rclpy).

Usage: source ROS2 and workspace setup, then run:
`ros2 run px4_single_plan offboard_waypoint_runner`

Parameters:
- `vehicle_namespace` (string): PX4 namespace, e.g. `px4_1`. Empty means root `/fmu/*`.
- `waypoint_file` (string): waypoint YAML in `px4_single_plan` share (default `waypoints_q4_1.yaml`).
- `field_config_file` (string): shared field/launch GPS config in `px4_single_plan` share.
- `takeoff_altitude_m` (float): takeoff altitude command (default `5.0`).
- `land_after_waypoint_index` (int): waypoint index to trigger landing, `-1` means last waypoint.
"""
import os
import math
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleStatus, VehicleLocalPosition, VehicleAttitude, VehicleCommandAck
from sensor_msgs.msg import Image
from geometry_msgs.msg import Vector3
from std_msgs.msg import Bool
from px4_single_plan.field_config import (
    DEFAULT_FIELD_CONFIG_FILE,
    build_field_setup,
    field_xy_to_local,
    load_package_yaml,
    resolve_vehicle_launch_point,
)

try:
    import cv2
    from cv_bridge import CvBridge
    _IMAGE_SAVE_AVAILABLE = True
except Exception:
    cv2 = None
    CvBridge = None
    _IMAGE_SAVE_AVAILABLE = False

FORCE_ARM_MAGIC = 21196.0
ARMING_STATE_ARMED_VALUE = 2
DEFAULT_TAKEOFF_ALTITUDE_M = 5.0
DEFAULT_CAMERA_TRIGGER_DISTANCE_M = 4.0


class OffboardRunner(Node):
    def __init__(self):
        super().__init__('offboard_waypoint_runner')
        self.declare_parameter('vehicle_namespace', '')
        self.declare_parameter('waypoint_file', 'waypoints_q4_1.yaml')
        self.declare_parameter('field_config_file', DEFAULT_FIELD_CONFIG_FILE)
        self.declare_parameter('takeoff_altitude_m', DEFAULT_TAKEOFF_ALTITUDE_M)
        self.declare_parameter('land_after_waypoint_index', -1)
        self.declare_parameter('loop_waypoints', False)
        self.declare_parameter('wait_for_start_confirmation', False)
        self.declare_parameter('status_stale_timeout_s', 5.0)
        self.declare_parameter('camera_trigger_distance_m', DEFAULT_CAMERA_TRIGGER_DISTANCE_M)
        self.declare_parameter('terminate_return_altitude_m', DEFAULT_TAKEOFF_ALTITUDE_M)
        self.declare_parameter('enable_camera_image_saving', True)
        self.declare_parameter('camera_image_topic', '')
        self.declare_parameter('camera_output_root', os.path.expanduser('~/px4_captures'))
        self.declare_parameter('camera_image_extension', 'jpg')

        self.vehicle_namespace = self.get_parameter('vehicle_namespace').value.strip('/')
        self.waypoint_file = self.get_parameter('waypoint_file').value
        self.field_config_file = self.get_parameter('field_config_file').value.strip()
        self.takeoff_altitude_m = float(self.get_parameter('takeoff_altitude_m').value)
        self.land_after_waypoint_index = int(self.get_parameter('land_after_waypoint_index').value)
        self.loop_waypoints = bool(self.get_parameter('loop_waypoints').value)
        self.wait_for_start_confirmation = bool(self.get_parameter('wait_for_start_confirmation').value)
        self.status_stale_timeout_s = float(self.get_parameter('status_stale_timeout_s').value)
        self.camera_trigger_distance_m = float(self.get_parameter('camera_trigger_distance_m').value)
        self.terminate_return_altitude_m = float(self.get_parameter('terminate_return_altitude_m').value)
        self.enable_camera_image_saving = bool(self.get_parameter('enable_camera_image_saving').value)
        self.camera_image_topic = self.get_parameter('camera_image_topic').value.strip()
        self.camera_output_root = os.path.expanduser(self.get_parameter('camera_output_root').value)
        self.camera_image_extension = self.get_parameter('camera_image_extension').value.strip('.').lower() or 'jpg'

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # publishers (PX4 listens on /<namespace>/fmu/in/* for multi-vehicle).
        self.cmd_pub = self.create_publisher(VehicleCommand, self._topic('in/vehicle_command'), 10)
        self.offboard_pub = self.create_publisher(OffboardControlMode, self._topic('in/offboard_control_mode'), qos_profile)
        self.traj_pub = self.create_publisher(TrajectorySetpoint, self._topic('in/trajectory_setpoint'), qos_profile)

        # subscribers for state.
        # Subscribe to both legacy and *_v1 topic names to handle px4_msgs topic-version differences.
        self.status_sub = self.create_subscription(VehicleStatus, self._topic('out/vehicle_status'), self.status_cb, qos_profile)
        self.status_sub_v1 = self.create_subscription(VehicleStatus, self._topic('out/vehicle_status_v1'), self.status_cb, qos_profile)
        self.pos_sub = self.create_subscription(VehicleLocalPosition, self._topic('out/vehicle_local_position'), self.pos_cb, qos_profile)
        self.pos_sub_v1 = self.create_subscription(VehicleLocalPosition, self._topic('out/vehicle_local_position_v1'), self.pos_cb, qos_profile)
        self.att_sub = self.create_subscription(VehicleAttitude, self._topic('out/vehicle_attitude'), self.att_cb, qos_profile)
        self.ack_sub = self.create_subscription(VehicleCommandAck, self._topic('out/vehicle_command_ack'), self.ack_cb, qos_profile)

        # internal state
        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.arm_state = VehicleStatus.ARMING_STATE_DISARMED
        self.failsafe = False
        self.preflight_ok = False
        self.latest_arming_reason = 0
        self.latest_disarming_reason = 0
        self._status_received = False
        self._last_status_us = 0
        self._last_link_alive_us = 0
        self._last_takeoff_cmd_us = 0
        self._last_offboard_mode_cmd_us = 0
        self._takeoff_announced = False
        self._offboard_started = False
        self._landing_initiated = False
        self._landing_completed = False
        self._changing_terminate_altitude = False
        self._returning_to_launch = False
        self._launch_position = None
        self.pos = Vector3()
        self.yaw = 0.0
        self._initial_yaw = 0.0
        self._initial_yaw_set = False
        self._camera_trigger_sent = False
        self._saved_image_count = 0
        self._image_error_count = 0
        self._first_image_received = False
        self._camera_start_time_us = int(self.get_clock().now().nanoseconds / 1000)
        self._last_camera_wait_log_us = 0

        # read waypoints
        self.read_waypoints()
        self.wp_idx = 0
        self._wp_reached_since_us = None

        # timers
        self.arm_timer = self.create_timer(0.2, self._arm_state_machine)
        self.offboard_timer = None

        # throttle status logs while waiting for arming
        self._last_wait_log_us = 0
        self._last_reason_log = None
        self._start_confirmed = not self.wait_for_start_confirmation
        self._end_mission_requested = False
        self._last_start_wait_log_us = 0

        if self.wait_for_start_confirmation:
            self.start_sub = self.create_subscription(Bool, 'start_mission', self._start_cb, 10)
        else:
            self.start_sub = None
        self.end_sub = self.create_subscription(Bool, 'end_mission', self._end_cb, 10)

        self._camera_sub = None
        self._camera_dir = None
        self._camera_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self._cv_bridge = CvBridge() if _IMAGE_SAVE_AVAILABLE else None
        self._configure_camera_saving()
        self._camera_watchdog_timer = self.create_timer(5.0, self._camera_watchdog_cb)

        ns_label = self.vehicle_namespace if self.vehicle_namespace else 'root'
        self.get_logger().info(
            f'Offboard runner ready: namespace={ns_label}, waypoint_file={self.waypoint_file}, '
            f'takeoff_altitude_m={self.takeoff_altitude_m}, land_after_waypoint_index={self.land_after_waypoint_index}, '
            f'loop_waypoints={self.loop_waypoints}, wait_for_start_confirmation={self.wait_for_start_confirmation}, '
            f'camera_trigger_distance_m={self.camera_trigger_distance_m}, '
            f'terminate_return_altitude_m={self.terminate_return_altitude_m}'
        )

    def _start_cb(self, msg: Bool):
        if msg.data and not self._start_confirmed:
            self._start_confirmed = True
            self.get_logger().info('Received start confirmation. Beginning arm/takeoff sequence.')

    def _end_cb(self, msg: Bool):
        if not msg.data or self._end_mission_requested:
            return
        self._end_mission_requested = True
        if self._launch_position is None:
            self.get_logger().warn('Received end mission command before launch position was recorded. Landing immediately.')
            self._initiate_landing()
            return
        self._changing_terminate_altitude = True
        self._returning_to_launch = False
        self._wp_reached_since_us = None
        self.get_logger().info(
            f'Received end mission command. First moving to terminate altitude '
            f'{self.terminate_return_altitude_m:.1f} m, then returning to launch position '
            f'x={self._launch_position[0]:.2f}, y={self._launch_position[1]:.2f}.'
        )

    def _topic(self, suffix: str) -> str:
        if self.vehicle_namespace:
            return f'/{self.vehicle_namespace}/fmu/{suffix}'
        return f'/fmu/{suffix}'

    def _default_camera_topic(self) -> str:
        if self.vehicle_namespace:
            return f'/{self.vehicle_namespace}/camera/image_raw'
        return '/camera/image_raw'

    def _configure_camera_saving(self):
        if not self.enable_camera_image_saving:
            self.get_logger().info('Camera image saving disabled by parameter.')
            return

        if not _IMAGE_SAVE_AVAILABLE:
            self.get_logger().warn(
                'Camera image saving requested but cv_bridge/opencv import failed. '
                'Install `cv_bridge` and `python3-opencv` to enable image storage.'
            )
            return

        image_topic = self.camera_image_topic if self.camera_image_topic else self._default_camera_topic()
        drone_label = self.vehicle_namespace if self.vehicle_namespace else 'root'
        self._camera_dir = os.path.join(self.camera_output_root, drone_label)
        os.makedirs(self._camera_dir, exist_ok=True)
        self._camera_sub = self.create_subscription(Image, image_topic, self._camera_image_cb, self._camera_qos)
        self.get_logger().info(f'Saving camera images from {image_topic} to {self._camera_dir}')

    def _camera_image_cb(self, msg: Image):
        if not _IMAGE_SAVE_AVAILABLE or self._camera_dir is None:
            return

        try:
            self._first_image_received = True
            # Keep incoming encoding to support mono/color streams from Gazebo cameras.
            cv_image = self._cv_bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            stamp_sec = int(msg.header.stamp.sec)
            stamp_nsec = int(msg.header.stamp.nanosec)
            if stamp_sec == 0 and stamp_nsec == 0:
                now = self.get_clock().now().to_msg()
                stamp_sec = int(now.sec)
                stamp_nsec = int(now.nanosec)
            filename = f"img_{stamp_sec}_{stamp_nsec:09d}.{self.camera_image_extension}"
            out_path = os.path.join(self._camera_dir, filename)
            if cv2.imwrite(out_path, cv_image):
                self._saved_image_count += 1
                if self._saved_image_count % 20 == 0:
                    self.get_logger().info(f'Saved {self._saved_image_count} images in {self._camera_dir}')
            else:
                self.get_logger().warn(f'Failed to write image file: {out_path}')
        except Exception as e:
            self._image_error_count += 1
            if self._image_error_count <= 5 or (self._image_error_count % 50 == 0):
                self.get_logger().warn(
                    f'Image save failed (#{self._image_error_count}, encoding={msg.encoding}, '
                    f'width={msg.width}, height={msg.height}): {e}'
                )

    def _camera_watchdog_cb(self):
        if not self.enable_camera_image_saving:
            return
        if self._camera_dir is None:
            return
        if self._first_image_received:
            return

        now_us = int(self.get_clock().now().nanoseconds / 1000)
        if now_us - self._camera_start_time_us < 15_000_000:
            return
        if now_us - self._last_camera_wait_log_us < 10_000_000:
            return
        self._last_camera_wait_log_us = now_us
        image_topic = self.camera_image_topic if self.camera_image_topic else self._default_camera_topic()
        self.get_logger().warn(
            f'No camera frames received yet on {image_topic}. '
            'Check ros_gz_bridge and Gazebo camera topic mapping.'
        )

    def _enable_distance_camera_trigger(self):
        if self._camera_trigger_sent:
            return
        self._publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_TRIGGER_CONTROL, 1.0, 0.0)
        self._publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_CAM_TRIGG_DIST,
            self.camera_trigger_distance_m,
            0.0,
        )
        self._camera_trigger_sent = True
        self.get_logger().info(
            f'Enabled PX4 distance camera trigger: every {self.camera_trigger_distance_m:.1f} m.'
        )

    def _disable_camera_trigger(self):
        self._publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_TRIGGER_CONTROL, 0.0, 0.0)
        self._publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_CAM_TRIGG_DIST, 0.0, 0.0)
        self._camera_trigger_sent = False

    def read_waypoints(self):
        try:
            cfg, waypoint_path = load_package_yaml(self.waypoint_file)
            self.dis_min = float(cfg.get('dis_min', 0.5))
            self.wp_hold_s = float(cfg.get('wp_hold_s', 1.0))

            resolved_field_config_file = str(cfg.get('field_config_file', self.field_config_file) or '').strip()
            shared_cfg = {}
            field_config_path = waypoint_path
            if resolved_field_config_file:
                shared_cfg, field_config_path = load_package_yaml(resolved_field_config_file)

            self.field_config = dict(shared_cfg.get('field', {}) or {})
            self.field_config.update(cfg.get('field', {}) or {})
            self.launch_points = dict(shared_cfg.get('launch_points', {}) or {})
            self.launch_points.update(cfg.get('launch_points', {}) or {})
            self.field_setup = build_field_setup(
                {
                    'field': self.field_config,
                    'launch_points': self.launch_points,
                },
                source_path=field_config_path,
            )
            self.field_width_m = float(self.field_setup.get('width_m', 0.0) or 0.0)
            self.field_height_m = float(self.field_setup.get('height_m', 0.0) or 0.0)
            self.field_heading_deg = float(self.field_setup.get('heading_deg', 0.0) or 0.0)
            self.field_heading_rad = float(self.field_setup.get('heading_rad', 0.0) or 0.0)
            self.waypoint_frame = str(
                cfg.get('waypoint_frame', 'field' if self.launch_points or self.field_config else 'local')
            ).strip().lower()
            self._launch_point_field = self._resolve_launch_point_field()

            raw_waypoints = cfg.get('wp', []) or []
            self.waypoints = [self._normalize_waypoint(wp, idx) for idx, wp in enumerate(raw_waypoints)]
            if not self.waypoints:
                raise RuntimeError('No waypoints found')
            if self.land_after_waypoint_index < 0:
                self.land_after_waypoint_index = len(self.waypoints) - 1
            else:
                self.land_after_waypoint_index = min(self.land_after_waypoint_index, len(self.waypoints) - 1)
            self.get_logger().info(
                f'Loaded {len(self.waypoints)} {self.waypoint_frame} waypoints from {self.waypoint_file} '
                f'using field config {os.path.basename(field_config_path)} '
                f'for launch point x={self._launch_point_field["x"]:.2f}, y={self._launch_point_field["y"]:.2f}'
            )
        except Exception as e:
            self.get_logger().error(f'Failed to read waypoint setup for {self.waypoint_file}: {e}')
            self.field_setup = build_field_setup({'field': {}, 'launch_points': {}}, source_path=self.waypoint_file)
            self.field_config = {}
            self.field_width_m = 0.0
            self.field_height_m = 0.0
            self.field_heading_deg = 0.0
            self.field_heading_rad = 0.0
            self.launch_points = {}
            self.waypoint_frame = 'local'
            self._launch_point_field = {'x': 0.0, 'y': 0.0, 'yaw_deg': 0.0}
            self.waypoints = [{'x': 0.0, 'y': 0.0, 'z': 5.0, 'field_x': 0.0, 'field_y': 0.0}]
            self.dis_min = 0.5
            self.wp_hold_s = 1.0
            self.land_after_waypoint_index = 0

    def _resolve_launch_point_field(self):
        return resolve_vehicle_launch_point(self.vehicle_namespace, self.launch_points, self.field_setup)

    def _field_to_local_xy(self, field_x: float, field_y: float):
        return field_xy_to_local(
            field_x,
            field_y,
            self._launch_point_field['x'],
            self._launch_point_field['y'],
            self.field_heading_rad,
        )

    def _normalize_waypoint(self, wp, idx: int):
        field_x = float(wp.get('x', wp.get('x_m', 0.0)) or 0.0)
        field_y = float(wp.get('y', wp.get('y_m', 0.0)) or 0.0)
        target_z = float(wp.get('z', wp.get('z_m', 0.0)) or 0.0)

        if self.waypoint_frame == 'field':
            local_x, local_y = self._field_to_local_xy(field_x, field_y)
            if self.field_width_m > 0.0 and not (0.0 <= field_x <= self.field_width_m):
                self.get_logger().warning(
                    f'Waypoint {idx} x={field_x:.2f} m is outside field width 0..{self.field_width_m:.2f} m'
                )
            if self.field_height_m > 0.0 and not (0.0 <= field_y <= self.field_height_m):
                self.get_logger().warning(
                    f'Waypoint {idx} y={field_y:.2f} m is outside field height 0..{self.field_height_m:.2f} m'
                )
        else:
            local_x = field_x
            local_y = field_y

        normalized = {
            'x': local_x,
            'y': local_y,
            'z': target_z,
            'field_x': field_x,
            'field_y': field_y,
        }

        if 'yaw' in wp:
            normalized['yaw'] = float(wp['yaw'])
        elif 'yaw_deg' in wp:
            normalized['yaw'] = math.radians(float(wp['yaw_deg']))

        if 'hold_s' in wp:
            normalized['hold_s'] = float(wp['hold_s'])

        return normalized

    def status_cb(self, msg: VehicleStatus):
        self._status_received = True
        self._last_status_us = int(self.get_clock().now().nanoseconds / 1000)
        self._last_link_alive_us = self._last_status_us
        self.nav_state = msg.nav_state
        self.arm_state = msg.arming_state
        self.failsafe = msg.failsafe
        self.preflight_ok = msg.pre_flight_checks_pass
        self.latest_arming_reason = msg.latest_arming_reason
        self.latest_disarming_reason = msg.latest_disarming_reason

    def pos_cb(self, msg: VehicleLocalPosition):
        self.pos.x = msg.x
        self.pos.y = msg.y
        self.pos.z = msg.z
        if self._launch_position is None and all(math.isfinite(v) for v in (msg.x, msg.y, msg.z)):
            self._launch_position = (float(msg.x), float(msg.y), float(msg.z))
            self.get_logger().info(
                f'Recorded launch position: x={self._launch_position[0]:.2f}, '
                f'y={self._launch_position[1]:.2f}, z={self._launch_position[2]:.2f}'
            )

    def ack_cb(self, msg: VehicleCommandAck):
        if msg.command in (
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF,
            VehicleCommand.VEHICLE_CMD_NAV_LAND,
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
            VehicleCommand.VEHICLE_CMD_DO_TRIGGER_CONTROL,
            VehicleCommand.VEHICLE_CMD_DO_SET_CAM_TRIGG_DIST,
        ):
            self._last_link_alive_us = int(self.get_clock().now().nanoseconds / 1000)
            self.get_logger().info(
                f'CommandAck: cmd={msg.command} result={msg.result} p1={msg.result_param1} p2={msg.result_param2}'
            )

    def att_cb(self, msg: VehicleAttitude):
        q = msg.q
        # Extract yaw from quaternion (PX4 q order: w, x, y, z).
        # yaw = atan2(2*(w*z + x*y), 1 - 2*(y^2 + z^2))
        self.yaw = math.atan2(2.0 * (q[0] * q[3] + q[1] * q[2]), 1.0 - 2.0 * (q[2] * q[2] + q[3] * q[3]))
        if not self._initial_yaw_set:
            self._initial_yaw = self.yaw
            self._initial_yaw_set = True

    @staticmethod
    def _wrap_pi(angle_rad: float) -> float:
        return math.atan2(math.sin(angle_rad), math.cos(angle_rad))

    def _publish_vehicle_command(self, command, param1=0.0, param2=0.0, param3=0.0, param4=0.0, param5=0.0, param6=0.0, param7=0.0):
        m = VehicleCommand()
        m.command = int(command)
        m.param1 = float(param1)
        m.param2 = float(param2)
        m.param3 = float(param3)
        m.param4 = float(param4)
        m.param5 = float(param5)
        m.param6 = float(param6)
        m.param7 = float(param7)
        # In multi-vehicle SITL, per-namespace bridge delivers to the intended PX4
        # instance; using broadcast system target avoids sysid mismatches.
        m.target_system = 0
        m.target_component = 1
        m.source_system = 1
        m.source_component = 1
        m.from_external = True
        m.timestamp = int(Clock().now().nanoseconds / 1000)
        self.cmd_pub.publish(m)

    def _arm_state_machine(self):
        now_us = int(self.get_clock().now().nanoseconds / 1000)
        if not self._start_confirmed:
            if now_us - self._last_start_wait_log_us > 1_000_000:
                self._last_start_wait_log_us = now_us
                self.get_logger().info('Waiting for start confirmation on topic /px4_single_plan/start_mission')
            return

        link_ref_us = max(self._last_status_us, self._last_link_alive_us)
        stale_timeout_us = int(max(1.0, self.status_stale_timeout_s) * 1_000_000)
        if (not self._status_received) or (now_us - link_ref_us > stale_timeout_us):
            if now_us - self._last_wait_log_us > 1_000_000:
                self._last_wait_log_us = now_us
                self.get_logger().warn(
                    f'No fresh PX4 link health ({self._topic("out/vehicle_status")}) for >{self.status_stale_timeout_s:.1f}s. '
                    'Check MicroXRCEAgent and DDS bridge connection.'
                )
            return

        armed = int(self.arm_state) == ARMING_STATE_ARMED_VALUE

        # If not armed, try to arm. Once armed and in loiter/takeoff, send takeoff and then start offboard
        if not armed:
            self._takeoff_announced = False
            self._offboard_started = False
            self._camera_trigger_sent = False
            if self._landing_initiated and not self._landing_completed:
                self._landing_completed = True
                self.get_logger().info('Landing complete (vehicle disarmed).')
            if self._landing_completed:
                return
            # MAV_CMD_COMPONENT_ARM_DISARM: param2=21196 forces arming and bypasses pre-arm checks.
            self._publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0, FORCE_ARM_MAGIC)

            if now_us - self._last_wait_log_us > 1_000_000:
                self._last_wait_log_us = now_us
                self.get_logger().info(
                    f'Waiting for ARM: arm_state={self.arm_state}, preflight_ok={self.preflight_ok}, failsafe={self.failsafe}, nav_state={self.nav_state}'
                )
            reason_tuple = (self.latest_arming_reason, self.latest_disarming_reason)
            if reason_tuple != self._last_reason_log:
                self._last_reason_log = reason_tuple
                self.get_logger().info(
                    f'Arm/disarm reasons: latest_arming_reason={self.latest_arming_reason}, '
                    f'latest_disarming_reason={self.latest_disarming_reason}'
                )
                if self.latest_disarming_reason == VehicleStatus.ARM_DISARM_REASON_PREFLIGHT_INACTION:
                    self.get_logger().warn(
                        'PX4 auto-disarmed due to preflight inactivity. Increase/disable COM_DISARM_PRFLT in pxh>, '
                        'e.g. `param set COM_DISARM_PRFLT 60` or `param set COM_DISARM_PRFLT -1`.'
                    )
            return

        if self._landing_initiated:
            return
        if self._end_mission_requested and self.offboard_timer is None:
            self._initiate_landing()
            return

        if not self._camera_trigger_sent:
            self._enable_distance_camera_trigger()

        # armed: keep retrying takeoff command until we are in takeoff/offboard or above ~4.5m (NED z <= -4.5)
        if not self._takeoff_announced:
            self.get_logger().info(
                f'Armed — sending TAKEOFF to {self.takeoff_altitude_m:.1f}m and waiting for climb'
            )
            self._takeoff_announced = True

        if now_us - self._last_takeoff_cmd_us > 1_000_000:
            self._last_takeoff_cmd_us = now_us
            self._publish_vehicle_command(
                VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                self.takeoff_altitude_m,
            )

        # Also command OFFBOARD mode while armed so position setpoints can drive climb if AUTO_TAKEOFF does not.
        if now_us - self._last_offboard_mode_cmd_us > 1_000_000:
            self._last_offboard_mode_cmd_us = now_us
            self._publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)

        # Start setpoint stream as soon as we're armed so offboard mode has valid inputs.
        if self.offboard_timer is None:
            self.get_logger().info('Armed — starting offboard setpoint publisher')
            self.offboard_timer = self.create_timer(1.0/20.0, self._offboard_loop)

        takeoff_started = (
            self.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_TAKEOFF
            or self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD
            or self.pos.z <= -4.5
        )

        if takeoff_started and not self._offboard_started:
            self._offboard_started = True
            self.get_logger().info('Takeoff active — starting offboard setpoint publisher')
            if self.offboard_timer is None:
                self.offboard_timer = self.create_timer(1.0/20.0, self._offboard_loop)

    def _terminate_altitude_target(self):
        return (
            float(self.pos.x),
            float(self.pos.y),
            -abs(self.terminate_return_altitude_m),
        )

    def _return_to_launch_target(self):
        if self._launch_position is None:
            return None
        return (
            float(self._launch_position[0]),
            float(self._launch_position[1]),
            -abs(self.terminate_return_altitude_m),
        )

    def _offboard_loop(self):
        if self._landing_initiated:
            return

        # publish OffboardControlMode
        off = OffboardControlMode()
        off.timestamp = int(Clock().now().nanoseconds / 1000)
        off.position = True
        off.velocity = False
        off.acceleration = False
        self.offboard_pub.publish(off)

        # current target waypoint or terminate sequence target
        if self._changing_terminate_altitude:
            target = self._terminate_altitude_target()
            target_yaw = self._initial_yaw if self._initial_yaw_set else 0.0
        elif self._returning_to_launch:
            target = self._return_to_launch_target()
            if target is None:
                self.get_logger().warn('Launch position unavailable during return-to-launch. Landing immediately.')
                self._initiate_landing()
                return
            target_yaw = self._initial_yaw if self._initial_yaw_set else 0.0
        else:
            wp = self.waypoints[self.wp_idx]
            target = (float(wp['x']), float(wp['y']), float(wp['z']))
            target_yaw = float(wp.get('yaw', self._initial_yaw if self._initial_yaw_set else 0.0))

        # publish trajectory setpoint
        traj = TrajectorySetpoint()
        traj.timestamp = int(Clock().now().nanoseconds / 1000)
        traj.position[0] = target[0]
        traj.position[1] = target[1]
        traj.position[2] = target[2]
        traj.velocity[0] = float('nan')
        traj.velocity[1] = float('nan')
        traj.velocity[2] = float('nan')
        traj.acceleration[0] = float('nan')
        traj.acceleration[1] = float('nan')
        traj.acceleration[2] = float('nan')
        # Use fixed/mission yaw (or waypoint-provided yaw) to avoid continuous yaw spinning.
        traj.yaw = self._wrap_pi(target_yaw)
        traj.yawspeed = 0.0
        self.traj_pub.publish(traj)

        dx = target[0] - self.pos.x
        dy = target[1] - self.pos.y
        dz = target[2] - self.pos.z
        dist = math.sqrt(dx*dx + dy*dy + dz*dz)
        now_us = int(self.get_clock().now().nanoseconds / 1000)

        if self._changing_terminate_altitude:
            if dist <= float(self.dis_min):
                if self._wp_reached_since_us is None:
                    self._wp_reached_since_us = now_us
                elif (now_us - self._wp_reached_since_us) >= int(self.wp_hold_s * 1_000_000):
                    self._wp_reached_since_us = None
                    self._changing_terminate_altitude = False
                    self._returning_to_launch = True
                    self.get_logger().info('Reached terminate altitude. Returning to launch position.')
            else:
                self._wp_reached_since_us = None
            return

        if self._returning_to_launch:
            if dist <= float(self.dis_min):
                if self._wp_reached_since_us is None:
                    self._wp_reached_since_us = now_us
                elif (now_us - self._wp_reached_since_us) >= int(self.wp_hold_s * 1_000_000):
                    self._wp_reached_since_us = None
                    self._returning_to_launch = False
                    self.get_logger().info('Reached launch position. Initiating landing.')
                    self._initiate_landing()
            else:
                self._wp_reached_since_us = None
            return

        # check distance and advance waypoint
        active_hold_s = float(self.waypoints[self.wp_idx].get('hold_s', self.wp_hold_s))
        if dist <= float(self.dis_min):
            if self._wp_reached_since_us is None:
                self._wp_reached_since_us = now_us
            elif (now_us - self._wp_reached_since_us) >= int(active_hold_s * 1_000_000):
                self._wp_reached_since_us = None
                if self.loop_waypoints and self.wp_idx >= len(self.waypoints) - 1:
                    self.wp_idx = 0
                    self.get_logger().info(f'Looping back to waypoint {self.wp_idx}: {self.waypoints[self.wp_idx]}')
                    return
                if self.wp_idx == self.land_after_waypoint_index:
                    self._initiate_landing()
                    return
                self.wp_idx += 1
                self.get_logger().info(f'Advancing to waypoint {self.wp_idx}: {self.waypoints[self.wp_idx]}')
        else:
            self._wp_reached_since_us = None

    def _initiate_landing(self):
        if self._landing_initiated:
            return
        self._landing_initiated = True
        self.get_logger().info(
            f'Reached waypoint {self.wp_idx}. Initiating LAND at start location.'
        )
        self._disable_camera_trigger()
        self._publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        if self.offboard_timer is not None:
            self.offboard_timer.cancel()
            self.offboard_timer = None


def main(args=None):
    rclpy.init(args=args)
    node = OffboardRunner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
