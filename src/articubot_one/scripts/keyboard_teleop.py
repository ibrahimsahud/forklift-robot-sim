#!/usr/bin/python3
"""Persistent keyboard control for the robot-safe /cmd_vel pipeline."""

import math
import os
import select
import termios
import time
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool, Trigger


HELP = """
Safe robot keyboard control

  i / Up       move forward at the selected voltage level
  j / Left     forward-left arc
  l / Right    forward-right arc
  k / Space    immediate stop
  + / = / q    increase voltage level by 0.01 V
  - / _ / z    decrease voltage level by 0.01 V
  , / Down     reverse disabled: command STOP
  e            activate E-stop
  r            release E-stop and request fault clear (only while stopped)
  x            stop and quit keyboard node

The requested command is published continuously. Safety temporarily blocks it;
after obstacle clearance it resumes unless STOP was pressed.
"""

ARROWS = {
    '\x1b[A': 'forward',
    '\x1b[B': 'reverse',
    '\x1b[C': 'right',
    '\x1b[D': 'left',
    '\x1bOA': 'forward',
    '\x1bOB': 'reverse',
    '\x1bOC': 'right',
    '\x1bOD': 'left',
}

KEYS = {
    'i': 'forward',
    ',': 'reverse',
    'j': 'left',
    'l': 'right',
    'k': 'stop',
    ' ': 'stop',
}


class TerminalReader:
    """Read the controlling terminal even when launched by ros2 launch."""

    def __enter__(self):
        self.fd = os.open('/dev/tty', os.O_RDONLY | os.O_NONBLOCK)
        self.settings = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)
        return self

    def __exit__(self, exc_type, exc, traceback):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.settings)
        os.close(self.fd)

    def read_key(self, timeout):
        readable, _, _ = select.select([self.fd], [], [], timeout)
        if not readable:
            return None
        first = os.read(self.fd, 1).decode(errors='ignore')
        if first != '\x1b':
            return first
        sequence = first
        deadline = time.monotonic() + 0.04
        while time.monotonic() < deadline and len(sequence) < 4:
            readable, _, _ = select.select(
                [self.fd], [], [], max(0.0, deadline - time.monotonic()))
            if not readable:
                break
            sequence += os.read(self.fd, 1).decode(errors='ignore')
            if sequence in ARROWS:
                break
        return sequence


class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('maximum_linear_speed', 0.50)
        self.declare_parameter('maximum_angular_speed', 0.40)
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('minimum_move_voltage', 1.26)
        self.declare_parameter('maximum_move_voltage', 1.30)
        self.declare_parameter('voltage_step', 0.01)
        self.declare_parameter('initial_speed_level', 1)
        # Kept declared so older launch files that pass it still start safely.
        self.declare_parameter('initial_pwm', -1)

        topic = str(self.get_parameter('cmd_vel_topic').value)
        self.maximum_linear_speed = float(
            self.get_parameter('maximum_linear_speed').value)
        self.maximum_angular_speed = float(
            self.get_parameter('maximum_angular_speed').value)
        self.publish_rate = max(
            10.0, float(self.get_parameter('publish_rate').value))
        minimum_voltage = float(
            self.get_parameter('minimum_move_voltage').value)
        maximum_voltage = float(
            self.get_parameter('maximum_move_voltage').value)
        voltage_step = float(self.get_parameter('voltage_step').value)

        if not 0.0 < minimum_voltage <= maximum_voltage <= 1.30:
            raise ValueError(
                'movement voltages must satisfy 0 < min <= max <= 1.30')
        if voltage_step <= 0.0:
            raise ValueError('voltage_step must be positive')
        steps = int(round((maximum_voltage - minimum_voltage) / voltage_step))
        if not math.isclose(
                minimum_voltage + steps * voltage_step,
                maximum_voltage, abs_tol=1e-6):
            raise ValueError(
                'minimum/maximum voltage must align to voltage_step')

        self.voltage_levels = [
            round(minimum_voltage + index * voltage_step, 2)
            for index in range(steps + 1)
        ]
        initial_level = int(
            self.get_parameter('initial_speed_level').value)
        self.speed_level_index = max(
            0, min(len(self.voltage_levels) - 1, initial_level - 1))

        self.requested_motion = 'stop'
        # Fail-safe until the first /safety/stop_active=false message.
        self.safety_stop_active = True
        self.publisher = self.create_publisher(Twist, topic, 10)
        self.create_subscription(
            Bool, '/safety/stop_active', self.safety_callback, 10)

        # Preserve existing forklift status/E-stop integrations.
        self.direction_configured = False
        self.create_subscription(
            Bool,
            '/forklift_motor/direction_output_configured',
            self.direction_callback,
            10,
        )
        self.create_subscription(
            String, '/forklift_motor/status', self.status_callback, 10)
        self.estop_client = self.create_client(
            SetBool, '/forklift_motor/emergency_stop')
        self.clear_client = self.create_client(
            Trigger, '/forklift_motor/clear_faults')

        self.get_logger().info(f'keyboard -> {topic}; current motion: STOP')
        self.get_logger().info(
            f'Speed voltage level: {self.selected_voltage:.2f}V')
        self.get_logger().info(
            f'Minimum movement speed: {self.voltage_levels[0]:.2f}V')
        self.get_logger().info(
            f'Maximum speed: {self.voltage_levels[-1]:.2f}V')

    @property
    def selected_voltage(self):
        return self.voltage_levels[self.speed_level_index]

    @property
    def command_scale(self):
        # Level 1..N is encoded as 1/N..N/N of max_linear_command.
        return (self.speed_level_index + 1) / len(self.voltage_levels)

    def direction_callback(self, message):
        if message.data != self.direction_configured:
            self.direction_configured = message.data
            self.get_logger().info(
                f'DIRECTION_OUTPUT_CONFIGURED={self.direction_configured}')

    def status_callback(self, message):
        self.get_logger().info(f'HARDWARE: {message.data}')

    def safety_callback(self, message):
        was_active = self.safety_stop_active
        self.safety_stop_active = bool(message.data)
        if self.safety_stop_active and not was_active:
            self.get_logger().warning(
                'SAFETY STOP: obstacle active; requested motion retained')
        elif not self.safety_stop_active and was_active:
            self.get_logger().info(
                f'Safety clear: resuming requested motion '
                f'{self.requested_motion.upper()}')

    def change_speed_level(self, delta):
        old_index = self.speed_level_index
        self.speed_level_index = max(
            0,
            min(len(self.voltage_levels) - 1, old_index + delta),
        )
        if self.speed_level_index == old_index:
            if delta > 0:
                self.get_logger().info(
                    f'Maximum speed: {self.voltage_levels[-1]:.2f}V')
            else:
                self.get_logger().info(
                    f'Minimum movement speed: '
                    f'{self.voltage_levels[0]:.2f}V')
            return
        self.get_logger().info(
            f'Speed voltage level: {self.selected_voltage:.2f}V')

    def stop(self, log=True):
        self.requested_motion = 'stop'
        self.publisher.publish(Twist())
        if log:
            self.get_logger().info('STOP')

    def activate_estop(self):
        self.stop()
        request = SetBool.Request()
        request.data = True
        self.estop_client.call_async(request)
        self.get_logger().error('E-STOP requested')

    def release_estop(self):
        self.stop()
        request = SetBool.Request()
        request.data = False
        future = self.estop_client.call_async(request)

        def clear_after_release(completed):
            try:
                if completed.result().success:
                    self.clear_client.call_async(Trigger.Request())
                    self.get_logger().warning(
                        'E-stop released; zero-command fault clear requested')
            except Exception as error:
                self.get_logger().error(
                    f'Could not release E-stop: {error}')

        future.add_done_callback(clear_after_release)

    def command(self, motion):
        command = Twist()
        linear = self.maximum_linear_speed * self.command_scale
        if motion == 'forward':
            command.linear.x = linear
        elif motion in ('left', 'right'):
            command.linear.x = linear
            command.angular.z = 1.0 if motion == 'left' else -1.0
        return command

    def start_motion(self, motion):
        if motion == 'reverse':
            self.stop(log=False)
            self.get_logger().error('REVERSE_DISABLED')
            return
        self.requested_motion = motion
        self.get_logger().info(motion.upper())
        self.get_logger().info(
            f'Speed voltage level: {self.selected_voltage:.2f}V')

    def publish_motion(self):
        if self.requested_motion == 'stop':
            self.publisher.publish(Twist())
            return
        self.publisher.publish(self.command(self.requested_motion))

    def handle_key(self, key):
        if key is None:
            return True
        if key in ARROWS:
            action = ARROWS[key]
        else:
            key = key.lower()
            action = KEYS.get(key)
        if key in ('+', '=', 'q'):
            self.change_speed_level(1)
        elif key in ('-', '_', 'z'):
            self.change_speed_level(-1)
        elif key == 'e':
            self.activate_estop()
        elif key == 'r':
            self.release_estop()
        elif key == 'x':
            self.stop()
            return False
        elif action == 'stop':
            self.stop()
        elif action is not None:
            self.start_motion(action)
        return True


def main():
    rclpy.init()
    node = KeyboardTeleop()
    print(HELP, flush=True)
    period = 1.0 / node.publish_rate
    try:
        with TerminalReader() as terminal:
            running = True
            while rclpy.ok() and running:
                rclpy.spin_once(node, timeout_sec=0.0)
                running = node.handle_key(terminal.read_key(period))
                node.publish_motion()
    except (KeyboardInterrupt, OSError) as error:
        if not isinstance(error, KeyboardInterrupt):
            node.get_logger().error(
                f'Keyboard terminal unavailable: {error}')
    finally:
        try:
            node.stop()
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
