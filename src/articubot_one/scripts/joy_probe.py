#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy


class JoyProbe(Node):
    def __init__(self) -> None:
        super().__init__("joy_probe")
        self.last_axes = []
        self.last_buttons = []
        self.create_subscription(Joy, "/joy", self.cb, 10)
        self.get_logger().info("joy_probe started. Move sticks / press buttons to see mapping.")

    def cb(self, msg: Joy) -> None:
        if not self.last_axes:
            self.last_axes = [0.0] * len(msg.axes)
            self.last_buttons = [0] * len(msg.buttons)

        moved = []
        for i, v in enumerate(msg.axes):
            if abs(v - self.last_axes[i]) > 0.15:
                moved.append(f"axis[{i}]={v:.2f}")
            self.last_axes[i] = v

        pressed = []
        for i, b in enumerate(msg.buttons):
            if b != self.last_buttons[i]:
                pressed.append(f"btn[{i}]={b}")
            self.last_buttons[i] = b

        if moved or pressed:
            self.get_logger().info(" | ".join(moved + pressed))


def main() -> None:
    rclpy.init()
    node = JoyProbe()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
