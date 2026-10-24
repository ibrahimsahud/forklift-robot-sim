import glob
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _detect_joystick_device() -> str:
    preferred_patterns = (
        "/dev/input/by-id/*-joystick",
        "/dev/input/by-path/*-joystick",
        "/dev/input/js*",
    )

    for pattern in preferred_patterns:
        candidates = sorted(glob.glob(pattern))
        if candidates:
            return os.path.realpath(candidates[0])

    raise RuntimeError(
        "No USB game controller detected. Connect the controller or pass joy_dev:=/dev/input/jsX."
    )


def _launch_setup(context, *args, **kwargs):
    joy_dev_value = LaunchConfiguration("joy_dev").perform(context).strip()
    use_sim_time_value = LaunchConfiguration("use_sim_time").perform(context)
    cmd_vel_topic_value = LaunchConfiguration("cmd_vel_topic").perform(context)

    joy_dev = joy_dev_value or _detect_joystick_device()

    params_file = os.path.join(
        get_package_share_directory("articubot_one"),
        "config",
        "usb_gamepad_teleop.yaml",
    )

    return [
        LogInfo(msg=f"USB game controller device: {joy_dev}"),
        Node(
            package="joy",
            executable="joy_node",
            name="joy_node",
            parameters=[
                params_file,
                {
                    "dev": joy_dev,
                    "use_sim_time": use_sim_time_value.lower() == "true",
                },
            ],
            output="screen",
        ),
        Node(
            package="teleop_twist_joy",
            executable="teleop_node",
            name="teleop_node",
            parameters=[
                params_file,
                {"use_sim_time": use_sim_time_value.lower() == "true"},
            ],
            remappings=[("/cmd_vel", cmd_vel_topic_value)],
            output="screen",
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "joy_dev",
                default_value="",
                description="Joystick device path. Leave empty for auto-detection.",
            ),
            DeclareLaunchArgument(
                "cmd_vel_topic",
                default_value="/cmd_vel_joy",
                description="Manual teleop output Twist topic.",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation clock if true.",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
