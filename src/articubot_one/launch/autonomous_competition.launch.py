import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory("articubot_one")
    artifacts_root = os.environ.get(
        "TEKNO_ROBOT_ARTIFACTS_DIR",
        os.path.join("/tmp", "tekno_robot_artifacts"),
    )

    default_world = os.path.join(pkg, "worlds", "teknofest_arena.world")
    default_map = os.path.join(pkg, "maps", "teknofest_arena_map_new.yaml")
    line_follower_params = os.path.join(pkg, "config", "line_follower_params.yaml")
    line_sensor_params = os.path.join(pkg, "config", "line_sensor_params.yaml")
    line_segments_file = os.path.join(pkg, "config", "line_segments.yaml")
    route_waypoints_file = os.path.join(pkg, "config", "route_waypoints.yaml")

    route = LaunchConfiguration("route")
    rviz = LaunchConfiguration("rviz")
    line_source = LaunchConfiguration("line_source")
    camera_line_reverse = LaunchConfiguration("camera_line_reverse")
    resume_from_current = LaunchConfiguration("resume_from_current")
    stop_after = LaunchConfiguration("stop_after")
    world = LaunchConfiguration("world")
    map_yaml = LaunchConfiguration("map")
    headless = LaunchConfiguration("headless")
    spawn_x = LaunchConfiguration("spawn_x")
    spawn_y = LaunchConfiguration("spawn_y")
    spawn_z = LaunchConfiguration("spawn_z")
    spawn_yaw = LaunchConfiguration("spawn_yaw")

    localization_nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, "launch", "localization_navigation.launch.py")),
        launch_arguments={
            "world": world,
            "map": map_yaml,
            "use_ros2_control": "true",
            "headless": headless,
            "rviz": rviz,
            "start_mission_nodes": "false",
            "spawn_x": spawn_x,
            "spawn_y": spawn_y,
            "spawn_z": spawn_z,
            "spawn_yaw": spawn_yaw,
        }.items(),
    )

    fork_controller = Node(
        package="articubot_one",
        executable="fork_controller_node",
        name="fork_controller",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "publish_ros2_control_command": False,
        }],
    )

    qr_reader = Node(
        package="articubot_one",
        executable="qr_reader",
        name="qr_reader",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    pose_line_controller = Node(
        package="articubot_one",
        executable="gazebo_pose_line_controller",
        name="gazebo_pose_line_controller",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "line_segments_file": line_segments_file,
            "route_waypoints_file": route_waypoints_file,
        }],
    )

    line_follower = Node(
        package="articubot_one",
        executable="line_follower_node",
        name="line_follower",
        output="screen",
        parameters=[
            line_follower_params,
            {
                "use_sim_time": True,
                "camera_line_reverse": camera_line_reverse,
            },
        ],
    )

    line_sensor_controller = Node(
        package="articubot_one",
        executable="line_sensor_controller",
        name="line_sensor_controller",
        output="screen",
        parameters=[
            line_sensor_params,
            {
                "use_sim_time": True,
            },
        ],
        condition=IfCondition(PythonExpression(["'", line_source, "' == 'line_sensor'"])),
    )

    line_sensor_mock = Node(
        package="articubot_one",
        executable="line_sensor_mock_node",
        name="line_sensor_mock_node",
        output="screen",
        parameters=[
            line_sensor_params,
            {
                "use_sim_time": True,
                "source_mode": "camera",
                "camera_topic": "/camera/image_raw",
            },
        ],
        condition=IfCondition(
            PythonExpression(["'", line_source, "' == 'line_sensor'"])
        ),
    )

    bringup_ready_cmd = [
        "ros2",
        "run",
        "articubot_one",
        "bringup_ready_checker",
        "--route",
        route,
        "--output-dir",
        os.path.join(artifacts_root, "autonomous_competition"),
        "--line-source",
        line_source,
        "--line-follower-params",
        line_follower_params,
        "--line-sensor-params",
        line_sensor_params,
        "--line-segments-file",
        line_segments_file,
        "--route-waypoints-file",
        route_waypoints_file,
    ]

    bringup_ready_cmd_with_stop = list(bringup_ready_cmd)
    stop_after_value = LaunchConfiguration("stop_after")
    bringup_ready_resume = ExecuteProcess(
        cmd=bringup_ready_cmd_with_stop + ["--stop-after", stop_after_value, "--resume-from-current"],
        additional_env={"PYTHONUNBUFFERED": "1"},
        output="screen",
        condition=IfCondition(
            PythonExpression([
                "'",
                resume_from_current,
                "' == 'true' and '",
                stop_after,
                "' != ''",
            ])
        ),
    )

    bringup_ready_resume_no_stop = ExecuteProcess(
        cmd=bringup_ready_cmd_with_stop + ["--resume-from-current"],
        additional_env={"PYTHONUNBUFFERED": "1"},
        output="screen",
        condition=IfCondition(
            PythonExpression([
                "'",
                resume_from_current,
                "' == 'true' and '",
                stop_after,
                "' == ''",
            ])
        ),
    )

    bringup_ready_no_resume = ExecuteProcess(
        cmd=bringup_ready_cmd_with_stop + ["--stop-after", stop_after_value],
        additional_env={"PYTHONUNBUFFERED": "1"},
        output="screen",
        condition=IfCondition(
            PythonExpression([
                "'",
                resume_from_current,
                "' != 'true' and '",
                stop_after,
                "' != ''",
            ])
        ),
    )

    bringup_ready_no_resume_no_stop = ExecuteProcess(
        cmd=bringup_ready_cmd_with_stop,
        additional_env={"PYTHONUNBUFFERED": "1"},
        output="screen",
        condition=IfCondition(
            PythonExpression([
                "'",
                resume_from_current,
                "' != 'true' and '",
                stop_after,
                "' == ''",
            ])
        ),
    )

    shutdown_after_bringup_ready_resume = RegisterEventHandler(
        OnProcessExit(
            target_action=bringup_ready_resume,
            on_exit=[
                EmitEvent(
                    event=Shutdown(
                        reason="bringup_ready_checker/route_executor exited; shutting down autonomous_competition launch",
                    )
                )
            ],
        )
    )

    shutdown_after_bringup_ready_no_resume = RegisterEventHandler(
        OnProcessExit(
            target_action=bringup_ready_no_resume,
            on_exit=[
                EmitEvent(
                    event=Shutdown(
                        reason="bringup_ready_checker/route_executor exited; shutting down autonomous_competition launch",
                    )
                )
            ],
        )
    )

    shutdown_after_bringup_ready_resume_no_stop = RegisterEventHandler(
        OnProcessExit(
            target_action=bringup_ready_resume_no_stop,
            on_exit=[
                EmitEvent(
                    event=Shutdown(
                        reason="bringup_ready_checker/route_executor exited; shutting down autonomous_competition launch",
                    )
                )
            ],
        )
    )

    shutdown_after_bringup_ready_no_resume_no_stop = RegisterEventHandler(
        OnProcessExit(
            target_action=bringup_ready_no_resume_no_stop,
            on_exit=[
                EmitEvent(
                    event=Shutdown(
                        reason="bringup_ready_checker/route_executor exited; shutting down autonomous_competition launch",
                    )
                )
            ],
        )
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("route", default_value="A1_TO_B3"),
            DeclareLaunchArgument("rviz", default_value="false"),
            DeclareLaunchArgument("line_source", default_value="pose_line"),
            DeclareLaunchArgument("camera_line_reverse", default_value="false"),
            DeclareLaunchArgument("resume_from_current", default_value="true"),
            DeclareLaunchArgument("stop_after", default_value=""),
            DeclareLaunchArgument("world", default_value=default_world),
            DeclareLaunchArgument("map", default_value=default_map),
            DeclareLaunchArgument("headless", default_value="false"),
            DeclareLaunchArgument("spawn_x", default_value="-5.80"),
            DeclareLaunchArgument("spawn_y", default_value="-3.05"),
            DeclareLaunchArgument("spawn_z", default_value="0.10"),
            DeclareLaunchArgument("spawn_yaw", default_value="1.5708"),
            localization_nav,
            fork_controller,
            qr_reader,
            line_follower,
            line_sensor_controller,
            line_sensor_mock,
            pose_line_controller,
            bringup_ready_resume,
            bringup_ready_resume_no_stop,
            bringup_ready_no_resume,
            bringup_ready_no_resume_no_stop,
            shutdown_after_bringup_ready_resume,
            shutdown_after_bringup_ready_resume_no_stop,
            shutdown_after_bringup_ready_no_resume,
            shutdown_after_bringup_ready_no_resume_no_stop,
        ]
    )
