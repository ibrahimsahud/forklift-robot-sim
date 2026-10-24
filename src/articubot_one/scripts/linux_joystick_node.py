#!/usr/bin/python3
"""Publish Linux joystick state directly from /dev/input/js*.

This avoids the SDL remapping used by joy_node and preserves the same raw
axis numbering and signs shown by jstest.
"""

import errno
import os
import struct

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Joy


JS_EVENT_BUTTON = 0x01
JS_EVENT_AXIS = 0x02
JS_EVENT_INIT = 0x80
JS_EVENT = struct.Struct('IhBB')


class LinuxJoystickNode(Node):
    def __init__(self):
        super().__init__('linux_joystick_node')
        self.declare_parameter('device_path', '/dev/input/js0')
        self.declare_parameter('joy_topic', '/joy')
        self.declare_parameter('autorepeat_rate', 20.0)
        self.declare_parameter('reconnect_period_ms', 500)

        self.device_path = str(
            self.get_parameter('device_path').value)
        self.autorepeat_rate = float(
            self.get_parameter('autorepeat_rate').value)
        self.reconnect_ns = int(
            self.get_parameter('reconnect_period_ms').value) * 1_000_000
        if self.autorepeat_rate <= 0.0:
            raise ValueError('autorepeat_rate must be positive')
        if self.reconnect_ns <= 0:
            raise ValueError('reconnect_period_ms must be positive')

        self.publisher = self.create_publisher(
            Joy,
            str(self.get_parameter('joy_topic').value),
            qos_profile_sensor_data,
        )
        self.fd = None
        self.read_buffer = bytearray()
        self.axes = []
        self.buttons = []
        self.have_state = False
        self.next_open_ns = 0
        self.open_error_logged = False

        # Read input events with low latency. A separate 20 Hz timer repeats
        # the current state even when the physical control has not changed.
        self.read_timer = self.create_timer(0.002, self.poll_device)
        self.repeat_timer = self.create_timer(
            1.0 / self.autorepeat_rate,
            self.repeat_state,
        )
        self.try_open()

    def now_ns(self):
        return self.get_clock().now().nanoseconds

    def try_open(self):
        if self.fd is not None or self.now_ns() < self.next_open_ns:
            return
        self.next_open_ns = self.now_ns() + self.reconnect_ns
        try:
            self.fd = os.open(
                self.device_path,
                os.O_RDONLY | os.O_NONBLOCK,
            )
        except OSError as exc:
            if not self.open_error_logged:
                self.get_logger().warning(
                    f'Waiting for Linux joystick: {self.device_path}: {exc}')
                self.open_error_logged = True
            return

        self.read_buffer.clear()
        self.axes = []
        self.buttons = []
        self.have_state = False
        self.open_error_logged = False
        self.get_logger().info(
            f'Opened Linux joystick directly: {self.device_path}')

    def close_device(self, reason):
        if self.fd is None:
            return
        # A disconnect is an immediate stop, not a delayed joystick timeout.
        if self.have_state:
            self.axes = [0.0] * len(self.axes)
            self.buttons = [0] * len(self.buttons)
            self.publish_state()
        try:
            os.close(self.fd)
        except OSError:
            pass
        self.fd = None
        self.have_state = False
        self.read_buffer.clear()
        self.next_open_ns = self.now_ns() + self.reconnect_ns
        self.get_logger().warning(
            f'Linux joystick disconnected: {reason}')

    def ensure_axis(self, index):
        if index >= len(self.axes):
            self.axes.extend([0.0] * (index + 1 - len(self.axes)))

    def ensure_button(self, index):
        if index >= len(self.buttons):
            self.buttons.extend([0] * (index + 1 - len(self.buttons)))

    @staticmethod
    def normalized_axis(value):
        return max(-1.0, min(1.0, float(value) / 32767.0))

    def apply_event(self, value, event_type, number):
        event_type &= ~JS_EVENT_INIT
        if event_type == JS_EVENT_AXIS:
            self.ensure_axis(number)
            self.axes[number] = self.normalized_axis(value)
            return True
        if event_type == JS_EVENT_BUTTON:
            self.ensure_button(number)
            self.buttons[number] = 1 if value else 0
            return True
        return False

    def poll_device(self):
        if self.fd is None:
            self.try_open()
            return

        received_event = False
        while True:
            try:
                data = os.read(self.fd, JS_EVENT.size * 64)
            except BlockingIOError:
                break
            except OSError as exc:
                if exc.errno in (
                        errno.ENODEV, errno.ENXIO, errno.EIO,
                        errno.EBADF):
                    self.close_device(str(exc))
                    return
                raise

            if not data:
                self.close_device('end of device stream')
                return
            self.read_buffer.extend(data)

            while len(self.read_buffer) >= JS_EVENT.size:
                raw_event = self.read_buffer[:JS_EVENT.size]
                del self.read_buffer[:JS_EVENT.size]
                _, value, event_type, number = JS_EVENT.unpack(raw_event)
                received_event |= self.apply_event(
                    value, event_type, number)

        if received_event:
            self.have_state = True
            # Publish changes immediately. In particular, a D-pad release
            # reaches joystick_teleop without waiting for the repeat timer.
            self.publish_state()

    def publish_state(self):
        if not self.have_state:
            return
        message = Joy()
        message.header.stamp = self.get_clock().now().to_msg()
        message.axes = list(self.axes)
        message.buttons = list(self.buttons)
        self.publisher.publish(message)

    def repeat_state(self):
        if self.fd is not None and self.have_state:
            self.publish_state()

    def destroy_node(self):
        if self.fd is not None:
            if self.have_state:
                self.axes = [0.0] * len(self.axes)
                self.buttons = [0] * len(self.buttons)
                self.publish_state()
            try:
                os.close(self.fd)
            except OSError:
                pass
            self.fd = None
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LinuxJoystickNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
