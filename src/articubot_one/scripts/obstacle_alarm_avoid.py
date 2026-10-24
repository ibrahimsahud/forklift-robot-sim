#!/usr/bin/env python3

from dataclasses import dataclass
from typing import List, Optional

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import Bool, String


@dataclass
class MotionStep:
    linear_x: float
    angular_z: float
    duration_sec: float


class ObstacleAlarmAvoid(Node):
    def __init__(self) -> None:
        super().__init__("obstacle_alarm_avoid")

        self.declare_parameter("input_cmd_topic", "/cmd_vel_joy")
        self.declare_parameter("output_cmd_topic", "/cmd_vel")
        self.declare_parameter("distance_topic", "/sensors/distance")
        self.declare_parameter("buzzer_cmd_topic", "/sensors/buzzer_cmd")
        self.declare_parameter("led_cmd_topic", "/sensors/led_cmd")
        self.declare_parameter("status_topic", "/obstacle_guard/status")

        self.declare_parameter("threshold_m", 0.50)
        self.declare_parameter("clear_m", 0.60)
        self.declare_parameter("alarm_wait_sec", 10.0)
        self.declare_parameter("control_hz", 20.0)
        self.declare_parameter("turn_speed", 0.70)
        self.declare_parameter("forward_speed", 0.18)
        self.declare_parameter("turn_time_sec", 1.20)
        self.declare_parameter("bypass_forward_sec", 1.50)
        self.declare_parameter("avoid_direction", "left")

        self.input_cmd_topic = str(self.get_parameter("input_cmd_topic").value)
        self.output_cmd_topic = str(self.get_parameter("output_cmd_topic").value)
        self.distance_topic = str(self.get_parameter("distance_topic").value)
        self.buzzer_cmd_topic = str(self.get_parameter("buzzer_cmd_topic").value)
        self.led_cmd_topic = str(self.get_parameter("led_cmd_topic").value)
        self.status_topic = str(self.get_parameter("status_topic").value)

        self.threshold_m = float(self.get_parameter("threshold_m").value)
        self.clear_m = float(self.get_parameter("clear_m").value)
        self.alarm_wait_sec = float(self.get_parameter("alarm_wait_sec").value)
        self.control_hz = float(self.get_parameter("control_hz").value)
        self.turn_speed = float(self.get_parameter("turn_speed").value)
        self.forward_speed = float(self.get_parameter("forward_speed").value)
        self.turn_time_sec = float(self.get_parameter("turn_time_sec").value)
        self.bypass_forward_sec = float(self.get_parameter("bypass_forward_sec").value)
        self.avoid_direction = str(self.get_parameter("avoid_direction").value).strip().lower()

        if self.avoid_direction not in ("left", "right"):
            self.get_logger().warn("Invalid avoid_direction, using 'left'.")
            self.avoid_direction = "left"

        self.cmd_pub = self.create_publisher(Twist, self.output_cmd_topic, 10)
        self.buzzer_pub = self.create_publisher(Bool, self.buzzer_cmd_topic, 10)
        self.led_pub = self.create_publisher(Bool, self.led_cmd_topic, 10)
        self.status_pub = self.create_publisher(String, self.status_topic, 10)

        self.create_subscription(Twist, self.input_cmd_topic, self.cmd_callback, 10)
        self.create_subscription(Range, self.distance_topic, self.distance_callback, 10)

        period = 1.0 / max(1.0, self.control_hz)
        self.create_timer(period, self.control_loop)

        self.last_input_cmd = Twist()
        self.latest_distance_m: Optional[float] = None

        self.state = "NORMAL"
        self.alarm_start_time = None
        self.steps: List[MotionStep] = []
        self.step_index = 0
        self.step_start_time = None

        self.last_alarm_cmd = None
        self.last_status = ""

        self.get_logger().info(
            f"Obstacle guard running: threshold={self.threshold_m:.2f}m, "
            f"alarm_wait={self.alarm_wait_sec:.1f}s, direction={self.avoid_direction}"
        )

    def now_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def cmd_callback(self, msg: Twist) -> None:
        self.last_input_cmd = msg

    def distance_callback(self, msg: Range) -> None:
        if msg.range <= 0.0:
            return
        self.latest_distance_m = float(msg.range)

    def obstacle_near(self) -> bool:
        if self.latest_distance_m is None:
            return False
        return self.latest_distance_m <= self.threshold_m

    def obstacle_cleared(self) -> bool:
        if self.latest_distance_m is None:
            return True
        return self.latest_distance_m >= self.clear_m

    def publish_status(self, text: str) -> None:
        if text == self.last_status:
            return
        self.last_status = text
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)
        self.get_logger().info(text)

    def publish_alarm(self, enabled: bool) -> None:
        if self.last_alarm_cmd == enabled:
            return
        self.last_alarm_cmd = enabled
        b = Bool()
        b.data = enabled
        self.buzzer_pub.publish(b)
        self.led_pub.publish(b)

    def publish_twist(self, linear_x: float, angular_z: float) -> None:
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        self.cmd_pub.publish(msg)

    def build_avoid_steps(self) -> List[MotionStep]:
        sign = 1.0 if self.avoid_direction == "left" else -1.0

        # S-curve: turn, pass side, counter-turn, pass, return heading.
        return [
            MotionStep(0.0, sign * self.turn_speed, self.turn_time_sec),
            MotionStep(self.forward_speed, 0.0, self.bypass_forward_sec),
            MotionStep(0.0, -sign * self.turn_speed, 2.0 * self.turn_time_sec),
            MotionStep(self.forward_speed, 0.0, self.bypass_forward_sec),
            MotionStep(0.0, sign * self.turn_speed, self.turn_time_sec),
            MotionStep(self.forward_speed, 0.0, 0.8),
        ]

    def start_alarm_wait(self) -> None:
        self.state = "ALARM_WAIT"
        self.alarm_start_time = self.now_sec()
        self.publish_status("OBSTACLE_DETECTED: alarm ON, waiting up to 10s")

    def start_avoidance(self) -> None:
        self.state = "AVOID"
        self.steps = self.build_avoid_steps()
        self.step_index = 0
        self.step_start_time = self.now_sec()
        self.publish_status("OBSTACLE_PERSISTED_10S: starting avoidance maneuver")

    def stop_and_normal(self) -> None:
        self.state = "NORMAL"
        self.alarm_start_time = None
        self.steps = []
        self.step_index = 0
        self.step_start_time = None
        self.publish_alarm(False)
        self.publish_status("PATH_CLEAR: return to joystick control")

    def control_loop(self) -> None:
        now = self.now_sec()

        if self.state == "NORMAL":
            if self.obstacle_near():
                self.start_alarm_wait()
                self.publish_alarm(True)
                self.publish_twist(0.0, 0.0)
                return

            self.publish_alarm(False)
            self.publish_twist(self.last_input_cmd.linear.x, self.last_input_cmd.angular.z)
            return

        if self.state == "ALARM_WAIT":
            self.publish_alarm(True)
            self.publish_twist(0.0, 0.0)

            if self.obstacle_cleared():
                self.stop_and_normal()
                return

            if self.alarm_start_time is not None and (now - self.alarm_start_time) >= self.alarm_wait_sec:
                self.start_avoidance()
            return

        if self.state == "AVOID":
            self.publish_alarm(True)

            if self.step_index >= len(self.steps):
                # Finished maneuver: keep alarm if still near.
                if self.obstacle_near():
                    self.start_alarm_wait()
                else:
                    self.stop_and_normal()
                return

            step = self.steps[self.step_index]
            self.publish_twist(step.linear_x, step.angular_z)

            if self.step_start_time is None:
                self.step_start_time = now

            if (now - self.step_start_time) >= step.duration_sec:
                self.step_index += 1
                self.step_start_time = now


def main() -> None:
    rclpy.init()
    node = ObstacleAlarmAvoid()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_twist(0.0, 0.0)
        node.publish_alarm(False)
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
