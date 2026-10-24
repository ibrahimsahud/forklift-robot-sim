#!/usr/bin/python3

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Joy

try:
    from articubot_one.frisby_gamepad_mapping import map_frisby_controls
except ModuleNotFoundError as exc:
    # CMake installs this script as a standalone executable.  Keep it usable
    # when a launch shell did not inherit this workspace's Python path.
    if exc.name != 'articubot_one':
        raise
    from frisby_gamepad_mapping import map_frisby_controls


class FrisbyGamepadTeleop(Node):
    def __init__(self):
        super().__init__('frisby_gamepad_teleop')
        self.declare_parameter('base_pwm', 63)
        self.declare_parameter('right_base_pwm', 63)
        self.declare_parameter('turbo_pwm', 120)
        self.declare_parameter('maximum_pwm', 150)
        self.declare_parameter('maximum_wheel_speed_m_s', 0.20)
        self.declare_parameter('wheel_separation', 0.62)
        self.declare_parameter('deadzone', 0.20)
        self.declare_parameter('joy_timeout_sec', 0.30)
        self.base_pwm = int(self.get_parameter('base_pwm').value)
        self.right_base_pwm = int(self.get_parameter('right_base_pwm').value)
        self.turbo_pwm = int(self.get_parameter('turbo_pwm').value)
        self.maximum_pwm = int(self.get_parameter('maximum_pwm').value)
        self.maximum_wheel_speed = float(
            self.get_parameter('maximum_wheel_speed_m_s').value
        )
        self.wheel_separation = float(
            self.get_parameter('wheel_separation').value
        )
        self.deadzone = float(self.get_parameter('deadzone').value)
        self.timeout = float(self.get_parameter('joy_timeout_sec').value)
        # Validate configuration before publishing anything.
        map_frisby_controls(
            (), (), deadzone=self.deadzone, base_pwm=self.base_pwm,
            right_base_pwm=self.right_base_pwm,
            turbo_pwm=self.turbo_pwm, maximum_pwm=self.maximum_pwm,
            maximum_speed_m_s=self.maximum_wheel_speed,
        )
        self.last_joy_time = self.get_clock().now()
        self.last_command_nonzero = False
        self.last_turbo = False
        self.publisher = self.create_publisher(Twist, '/cmd_vel_joy', 10)
        self.create_subscription(
            Joy, '/joy', self.on_joy, qos_profile_sensor_data
        )
        self.create_timer(0.05, self.watchdog)
        self.get_logger().info(
            f'Frisby ready: joystick PWM left={self.base_pwm} '
            f'right={self.right_base_pwm}; '
            f'hold L1 or R1 for PWM={self.turbo_pwm}'
        )

    def on_joy(self, msg: Joy):
        self.last_joy_time = self.get_clock().now()
        drive = map_frisby_controls(
            msg.axes, msg.buttons, deadzone=self.deadzone,
            base_pwm=self.base_pwm, turbo_pwm=self.turbo_pwm,
            right_base_pwm=self.right_base_pwm,
            maximum_pwm=self.maximum_pwm,
            maximum_speed_m_s=self.maximum_wheel_speed,
        )
        linear = 0.5 * (drive.left_wheel_m_s + drive.right_wheel_m_s)
        angular = (
            (drive.right_wheel_m_s - drive.left_wheel_m_s) /
            self.wheel_separation
        )
        command = Twist()
        command.linear.x = linear
        command.angular.z = angular
        self.publisher.publish(command)
        self.last_command_nonzero = bool(linear or angular)
        if drive.turbo != self.last_turbo:
            self.get_logger().info(
                f'Frisby speed mode: PWM={drive.selected_pwm}'
            )
            self.last_turbo = drive.turbo

    def watchdog(self):
        age = (self.get_clock().now() - self.last_joy_time).nanoseconds * 1e-9
        if age > self.timeout and self.last_command_nonzero:
            self.publisher.publish(Twist())
            self.last_command_nonzero = False


def main(args=None):
    rclpy.init(args=args)
    node = FrisbyGamepadTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.publisher.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
