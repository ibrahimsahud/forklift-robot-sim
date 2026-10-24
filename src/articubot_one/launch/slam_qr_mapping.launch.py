import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory("articubot_one")

    world = LaunchConfiguration("world")
    headless = LaunchConfiguration("headless")
    rviz = LaunchConfiguration("rviz")
    rviz_config = LaunchConfiguration("rviz_config")
    use_ros2_control = LaunchConfiguration("use_ros2_control")
    start_joystick = LaunchConfiguration("start_joystick")
    laser_x = LaunchConfiguration("laser_x")
    laser_y = LaunchConfiguration("laser_y")
    laser_z = LaunchConfiguration("laser_z")
    spawn_x = LaunchConfiguration("spawn_x")
    spawn_y = LaunchConfiguration("spawn_y")
    spawn_z = LaunchConfiguration("spawn_z")
    spawn_yaw = LaunchConfiguration("spawn_yaw")

    qr_logger_params = os.path.join(pkg, "config", "qr_logger_params.yaml")

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, "launch", "launch_sim.launch.py")),
        launch_arguments={
            "world": world,
            "headless": headless,
            "use_ros2_control": use_ros2_control,
            "start_slam": "true",
            "start_nav2": "false",
            "start_mission_nodes": "false",
            "start_joystick": start_joystick,
            "laser_x": laser_x,
            "laser_y": laser_y,
            "laser_z": laser_z,
            "spawn_x": spawn_x,
            "spawn_y": spawn_y,
            "spawn_z": spawn_z,
            "spawn_yaw": spawn_yaw,
        }.items(),
    )

    qr_reader = Node(
        package="articubot_one",
        executable="qr_reader",
        name="qr_reader",
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    qr_waypoint_logger = Node(
        package="articubot_one",
        executable="qr_waypoint_logger",
        name="qr_waypoint_logger",
        parameters=[qr_logger_params, {"use_sim_time": True}],
        output="screen",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": True}],
        additional_env={
            "LIBGL_ALWAYS_SOFTWARE": "1",
            "LIBGL_ALWAYS_INDIRECT": "0",
            "MESA_GL_VERSION_OVERRIDE": "3.3",
            "MESA_GLSL_VERSION_OVERRIDE": "330",
            "GALLIUM_DRIVER": "llvmpipe",
        },
        condition=IfCondition(rviz),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "world",
                default_value=os.path.join(pkg, "worlds", "teknofest_arena.world"),
                description="Gazebo world to load",
            ),
            DeclareLaunchArgument(
                "headless",
                default_value="false",
                description="Run Gazebo headless",
            ),
            DeclareLaunchArgument(
                "rviz",
                default_value="true",
                description="Open RViz",
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=os.path.join(pkg, "rviz", "slam_mapping.rviz"),
                description="RViz config file",
            ),
            DeclareLaunchArgument(
                "use_ros2_control",
                default_value="true",
                description="Use ros2_control for simulation",
            ),
            DeclareLaunchArgument(
                "start_joystick",
                default_value="false",
                description="Start joystick teleop bridge",
            ),
            DeclareLaunchArgument(
                "laser_x",
                default_value="0.00",
                description="LiDAR x position in base_link frame",
            ),
            DeclareLaunchArgument(
                "laser_y",
                default_value="0.00",
                description="LiDAR y position in base_link frame",
            ),
            DeclareLaunchArgument(
                "laser_z",
                default_value="0.35",
                description="LiDAR z position in base_link frame",
            ),
            DeclareLaunchArgument(
                "spawn_x",
                default_value="-5.80",
                description="Robot spawn X position",
            ),
            DeclareLaunchArgument(
                "spawn_y",
                default_value="-3.05",
                description="Robot spawn Y position",
            ),
            DeclareLaunchArgument(
                "spawn_z",
                default_value="0.10",
                description="Robot spawn Z position",
            ),
            DeclareLaunchArgument(
                "spawn_yaw",
                default_value="1.5708",
                description="Robot spawn yaw in radians",
            ),
            sim,
            qr_reader,
            qr_waypoint_logger,
            rviz_node,
        ]
    )
