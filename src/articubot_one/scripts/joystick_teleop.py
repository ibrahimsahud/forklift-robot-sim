#!/usr/bin/python3
"""Immediate D-pad control for the robot-safe pipeline."""

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, Float32, String


MOVE_COMMAND = 0.10
MOVE_VOLTAGE = 1.26


class JoystickTeleop(Node):
    def __init__(self):
        super().__init__('joystick_teleop')
        defaults = {
            'joy_topic': '/joy',
            'cmd_vel_topic': '/cmd_vel',
            # SHANWAN exposes its D-pad as Hat0X/Hat0Y (axes 4/5).
            'dpad_axis_horizontal': 4,
            'dpad_axis_vertical': 5,
            # Raw Linux js values match jstest: LEFT=+1 and UP=-1.
            'dpad_left_sign': 1.0,
            'dpad_up_sign': -1.0,
            'dpad_threshold': 0.5,
            'joy_timeout_ms': 500,
            'publish_rate_hz': 20.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.p = {
            name: self.get_parameter(name).value for name in defaults
        }

        self.dpad_axis_horizontal = int(self.p['dpad_axis_horizontal'])
        self.dpad_axis_vertical = int(self.p['dpad_axis_vertical'])
        self.dpad_left_sign = float(self.p['dpad_left_sign'])
        self.dpad_up_sign = float(self.p['dpad_up_sign'])
        self.dpad_threshold = float(self.p['dpad_threshold'])
        self.timeout_ns = int(self.p['joy_timeout_ms']) * 1_000_000
        self.publish_rate_hz = float(self.p['publish_rate_hz'])

        if not 0.0 < self.dpad_threshold <= 1.0:
            raise ValueError('dpad_threshold must be in (0.0, 1.0]')
        if self.timeout_ns <= 0:
            raise ValueError('joy_timeout_ms must be positive')
        if self.publish_rate_hz < 10.0:
            raise ValueError('publish_rate_hz must be at least 10 Hz')
        if self.dpad_left_sign not in (-1.0, 1.0):
            raise ValueError('dpad_left_sign must be -1.0 or 1.0')
        if self.dpad_up_sign not in (-1.0, 1.0):
            raise ValueError('dpad_up_sign must be -1.0 or 1.0')

        self.last_joy_ns = 0
        self.desired_twist = Twist()
        self.active_direction = None
        self.last_axis4_raw = 0.0
        self.last_axis5_raw = 0.0
        self.last_status_log = None
        self.last_dpad_log = None

        indices = (
            self.dpad_axis_horizontal,
            self.dpad_axis_vertical,
        )
        self.mapping_configured = (
            all(index >= 0 for index in indices)
            and self.dpad_axis_horizontal != self.dpad_axis_vertical
        )
        initial_status = (
            'WAITING_FOR_JOYSTICK'
            if self.mapping_configured else 'MAPPING_UNCONFIGURED'
        )
        self.status = initial_status

        self.cmd_pub = self.create_publisher(
            Twist, str(self.p['cmd_vel_topic']), 10)
        self.status_pub = self.create_publisher(
            String, '/joystick/status', 10)
        self.connected_pub = self.create_publisher(
            Bool, '/joystick/connected', 10)
        self.speed_pub = self.create_publisher(
            Float32, '/joystick/speed_voltage_level', 10)
        self.create_subscription(
            Joy, str(self.p['joy_topic']), self.on_joy,
            qos_profile_sensor_data)
        # Keep one periodic publisher at 20 Hz. Immediate safety stops clear
        # desired_twist before publishing zero, so the timer cannot restore an
        # old command.
        self.publish_timer = self.create_timer(
            1.0 / self.publish_rate_hz,
            self.publish_command,
        )

        if not self.mapping_configured:
            self.get_logger().error(
                'D-pad mapping is unconfigured; both hat axes must be '
                'set from observed /joy data. Output is locked at zero.')
        self.get_logger().info(
            f'Speed voltage level: {MOVE_VOLTAGE:.2f}V; '
            'D-pad release: immediate')

    def now_ns(self):
        return self.get_clock().now().nanoseconds

    def set_status(self, status):
        self.status = status
        if status != self.last_status_log:
            if status in (
                    'JOYSTICK_TIMEOUT', 'MAPPING_UNCONFIGURED',
                    'MAPPING_OUT_OF_RANGE', 'REVERSE_DISABLED',
                    'DIRECTION_CONFLICT'):
                self.get_logger().warning(status)
            else:
                self.get_logger().info(status)
            self.last_status_log = status

    def stop(self, status, publish_immediately=False):
        had_motion = self.active_direction is not None
        self.active_direction = None
        self.desired_twist = Twist()
        self.set_status(status)
        if publish_immediately and had_motion:
            self.cmd_pub.publish(Twist())

    def command_active_direction(self):
        if self.active_direction is None:
            self.desired_twist = Twist()
            self.set_status('STOP')
            return
        command = Twist()
        command.linear.x = MOVE_COMMAND
        if self.active_direction == 'FORWARD':
            command.angular.z = 0.0
        elif self.active_direction == 'TURN_LEFT':
            command.angular.z = 1.0
        elif self.active_direction == 'TURN_RIGHT':
            command.angular.z = -1.0
        else:
            self.stop('DIRECTION_CONFLICT', publish_immediately=True)
            return
        self.desired_twist = command
        self.set_status(self.active_direction)

    def log_dpad(self, axis4_raw, axis5_raw):
        active = self.active_direction or 'NONE'
        published = (
            self.status
            if self.status in ('FORWARD', 'TURN_LEFT', 'TURN_RIGHT')
            else 'STOP'
        )
        signature = (
            round(axis4_raw, 3),
            round(axis5_raw, 3),
            active,
            published,
        )
        if signature == self.last_dpad_log:
            return
        self.last_dpad_log = signature
        self.get_logger().info(
            f'axis4={axis4_raw:.3f}; '
            f'axis5={axis5_raw:.3f}; '
            f'active={active}; '
            f'published={published}')

    def message_has_mapping(self, message):
        return (
            self.dpad_axis_horizontal < len(message.axes)
            and self.dpad_axis_vertical < len(message.axes)
        )

    def on_joy(self, message):
        self.last_joy_ns = self.now_ns()

        if not self.mapping_configured:
            self.stop('MAPPING_UNCONFIGURED')
            return
        if not self.message_has_mapping(message):
            self.stop('MAPPING_OUT_OF_RANGE', publish_immediately=True)
            return

        axis4_raw = float(message.axes[self.dpad_axis_horizontal])
        axis5_raw = float(message.axes[self.dpad_axis_vertical])
        self.last_axis4_raw = axis4_raw
        self.last_axis5_raw = axis5_raw
        horizontal = axis4_raw * self.dpad_left_sign
        vertical = axis5_raw * self.dpad_up_sign
        left = horizontal > self.dpad_threshold
        right = horizontal < -self.dpad_threshold
        up = vertical > self.dpad_threshold
        down = vertical < -self.dpad_threshold
        pressed_count = sum((left, right, up, down))
        dpad_neutral = pressed_count == 0

        # DOWN always clears the active direction and stops immediately.
        if down:
            self.stop('REVERSE_DISABLED', publish_immediately=True)
            self.log_dpad(axis4_raw, axis5_raw)
            return

        if pressed_count > 1:
            self.stop('DIRECTION_CONFLICT', publish_immediately=True)
            self.log_dpad(axis4_raw, axis5_raw)
            return

        if dpad_neutral:
            # The direct Linux reader only publishes a zero when the kernel
            # reports a real release. Stop immediately with no hold timer.
            self.stop('STOP', publish_immediately=True)
            self.log_dpad(axis4_raw, axis5_raw)
            return

        if up:
            self.active_direction = 'FORWARD'
        elif left:
            self.active_direction = 'TURN_LEFT'
        elif right:
            self.active_direction = 'TURN_RIGHT'

        self.command_active_direction()
        self.log_dpad(axis4_raw, axis5_raw)

    def publish_command(self):
        now_ns = self.now_ns()
        fresh = (
            self.last_joy_ns != 0
            and now_ns - self.last_joy_ns <= self.timeout_ns
        )
        if not self.mapping_configured:
            self.stop('MAPPING_UNCONFIGURED')
        elif not fresh:
            self.stop('JOYSTICK_TIMEOUT')

        # Keep publishing the current direct Linux joystick state at 20 Hz.
        self.cmd_pub.publish(self.desired_twist)
        connected = Bool()
        connected.data = fresh
        self.connected_pub.publish(connected)
        status = String()
        status.data = self.status
        self.status_pub.publish(status)
        speed = Float32()
        speed.data = MOVE_VOLTAGE
        self.speed_pub.publish(speed)


def main(args=None):
    rclpy.init(args=args)
    node = JoystickTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.cmd_pub.publish(Twist())
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
