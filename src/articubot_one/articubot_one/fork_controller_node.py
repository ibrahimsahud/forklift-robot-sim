#!/usr/bin/env python3

from enum import Enum
from typing import Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, Float64MultiArray, String


class ForkMode(Enum):
    HOLDING = "HOLDING"
    MOVING_UP = "MOVING_UP"
    MOVING_DOWN = "MOVING_DOWN"
    STOPPED = "STOPPED"
    ERROR = "ERROR"


class ForkController(Node):
    def __init__(self) -> None:
        super().__init__("fork_controller")

        self.declare_parameter("command_topic", "/fork_cmd")
        self.declare_parameter("state_topic", "/fork_state")
        self.declare_parameter("gz_position_topic", "/fork_position_cmd")
        self.declare_parameter(
            "ros2_control_command_topic",
            "/fork_joint_controller/commands",
        )
        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter("joint_name", "fork_carriage_joint")
        self.declare_parameter("lower_height", 0.0)
        self.declare_parameter("initial_height", 0.0)
        self.declare_parameter("lift_height", 0.35)
        self.declare_parameter("height_tolerance", 0.01)
        self.declare_parameter("command_rate_hz", 20.0)
        self.declare_parameter("state_rate_hz", 5.0)
        self.declare_parameter("publish_ros2_control_command", True)

        self.command_topic = str(self.get_parameter("command_topic").value)
        self.state_topic = str(self.get_parameter("state_topic").value)
        self.gz_position_topic = str(self.get_parameter("gz_position_topic").value)
        self.ros2_control_command_topic = str(
            self.get_parameter("ros2_control_command_topic").value
        )
        self.joint_state_topic = str(self.get_parameter("joint_state_topic").value)
        self.joint_name = str(self.get_parameter("joint_name").value)
        self.lower_height = float(self.get_parameter("lower_height").value)
        self.initial_height = float(self.get_parameter("initial_height").value)
        self.lift_height = float(self.get_parameter("lift_height").value)
        self.height_tolerance = float(self.get_parameter("height_tolerance").value)
        command_rate_hz = float(self.get_parameter("command_rate_hz").value)
        state_rate_hz = float(self.get_parameter("state_rate_hz").value)
        self.publish_ros2_control_command = bool(
            self.get_parameter("publish_ros2_control_command").value
        )

        self.current_height: Optional[float] = None
        self.target_height = self.clamp_height(self.initial_height)
        self.mode = ForkMode.HOLDING
        self.last_command = "INIT"
        self.error_text = ""

        self.state_pub = self.create_publisher(String, self.state_topic, 10)
        self.gz_position_pub = self.create_publisher(Float64, self.gz_position_topic, 10)
        self.ros2_control_pub = None
        if self.publish_ros2_control_command:
            self.ros2_control_pub = self.create_publisher(
                Float64MultiArray,
                self.ros2_control_command_topic,
                10,
            )
        self.create_subscription(String, self.command_topic, self.command_callback, 10)
        self.create_subscription(
            JointState,
            self.joint_state_topic,
            self.joint_state_callback,
            10,
        )

        self.create_timer(1.0 / max(command_rate_hz, 1.0), self.publish_target)
        self.create_timer(1.0 / max(state_rate_hz, 1.0), self.publish_state)

        self.get_logger().info(
            f"Fork controller listening on {self.command_topic}; "
            f"publishing {self.state_topic}, {self.gz_position_topic}"
            + (
                f", and {self.ros2_control_command_topic}"
                if self.publish_ros2_control_command
                else ""
            )
        )

    def command_callback(self, msg: String) -> None:
        command = msg.data.strip().upper()
        if command == "LIFT_UP":
            self.set_target(self.lift_height, command, ForkMode.MOVING_UP)
        elif command == "LIFT_DOWN":
            self.set_target(self.lower_height, command, ForkMode.MOVING_DOWN)
        elif command.startswith("LIFT_TO:"):
            try:
                target_height = float(command.split(":", 1)[1])
            except ValueError:
                self.mode = ForkMode.ERROR
                self.last_command = command or "<empty>"
                self.error_text = f"invalid_target_height={self.last_command}"
                self.get_logger().warn(f"Ignoring invalid fork command: {self.last_command}")
                self.publish_state()
                return
            target_mode = ForkMode.MOVING_UP
            if self.current_height is not None and target_height < self.current_height:
                target_mode = ForkMode.MOVING_DOWN
            self.set_target(target_height, command, target_mode)
        elif command == "LIFT_STOP":
            hold_height = self.current_height
            if hold_height is None:
                hold_height = self.target_height
            self.set_target(hold_height, command, ForkMode.STOPPED)
        else:
            self.mode = ForkMode.ERROR
            self.last_command = command or "<empty>"
            self.error_text = f"invalid_command={self.last_command}"
            self.get_logger().warn(f"Ignoring invalid fork command: {self.last_command}")
            self.publish_state()

    def set_target(self, height: float, command: str, mode: ForkMode) -> None:
        self.target_height = self.clamp_height(height)
        self.last_command = command
        self.mode = mode
        self.error_text = ""
        self.publish_target()
        self.publish_state()
        self.get_logger().info(
            f"{command}: target fork height {self.target_height:.3f} m"
        )

    def joint_state_callback(self, msg: JointState) -> None:
        try:
            index = msg.name.index(self.joint_name)
        except ValueError:
            return

        if index < len(msg.position):
            self.current_height = float(msg.position[index])
            self.update_mode_from_height()

    def update_mode_from_height(self) -> None:
        if self.mode in (ForkMode.ERROR, ForkMode.STOPPED):
            return
        if self.current_height is None:
            return

        error = self.target_height - self.current_height
        if abs(error) <= self.height_tolerance:
            self.mode = ForkMode.HOLDING
        elif error > 0.0:
            self.mode = ForkMode.MOVING_UP
        else:
            self.mode = ForkMode.MOVING_DOWN

    def publish_target(self) -> None:
        target = self.clamp_height(self.target_height)

        gz_msg = Float64()
        gz_msg.data = target
        self.gz_position_pub.publish(gz_msg)

        if self.publish_ros2_control_command and self.ros2_control_pub is not None:
            ros2_control_msg = Float64MultiArray()
            ros2_control_msg.data = [target]
            self.ros2_control_pub.publish(ros2_control_msg)

    def publish_state(self) -> None:
        self.update_mode_from_height()
        height_text = "unknown"
        if self.current_height is not None:
            height_text = f"{self.current_height:.3f}"

        msg = String()
        msg.data = (
            f"{self.mode.value} command={self.last_command} "
            f"height={height_text} target={self.target_height:.3f}"
        )
        if self.error_text:
            msg.data += f" {self.error_text}"
        self.state_pub.publish(msg)

    def clamp_height(self, height: float) -> float:
        return min(max(float(height), self.lower_height), self.lift_height)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ForkController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
