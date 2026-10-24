import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.logging import get_logger
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition


def generate_launch_description():
    pkg = get_package_share_directory('articubot_one')

    map_yaml = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    rviz = LaunchConfiguration('rviz')
    rviz_config = LaunchConfiguration('rviz_config')
    with_sim = LaunchConfiguration('with_sim')
    world = LaunchConfiguration('world')
    headless = LaunchConfiguration('headless')
    log_level = LaunchConfiguration('log_level')

    def _debug_paths(context, *args, **kwargs):
        logger = get_logger('saved_map_view')
        map_path = map_yaml.perform(context)
        rviz_path = rviz_config.perform(context)
        world_path = world.perform(context)
        logger.info(f"Resolved map YAML: {map_path}")
        logger.info(f"Resolved RViz config: {rviz_path}")
        logger.info(f"Resolved world: {world_path}")
        logger.info(f"use_sim_time: {use_sim_time.perform(context)}")
        logger.info(f"with_sim: {with_sim.perform(context)}")
        if os.path.isfile(map_path):
            logger.info(f"Map YAML exists: {map_path}")
        else:
            logger.error(f"Map YAML missing: {map_path}")
        if os.path.isfile(rviz_path):
            logger.info(f"RViz config exists: {rviz_path}")
        else:
            logger.error(f"RViz config missing: {rviz_path}")
        if os.path.isfile(world_path):
            logger.info(f"World exists: {world_path}")
        else:
            logger.info(f"World not found (only needed with_sim:=true): {world_path}")
        return []

    declare_map = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(pkg, 'maps', 'teknofest_arena_map.yaml'),
        description='Saved map YAML to load in map_server'
    )

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true'
    )

    declare_autostart = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically transition map_server lifecycle'
    )

    declare_rviz = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Open RViz'
    )

    declare_rviz_config = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(pkg, 'rviz', 'slam_mapping.rviz'),
        description='RViz config file'
    )

    declare_with_sim = DeclareLaunchArgument(
        'with_sim',
        default_value='false',
        description='Start Gazebo simulation'
    )

    declare_world = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(pkg, 'worlds', 'teknofest_arena.world'),
        description='Gazebo world to load when with_sim is true'
    )

    declare_headless = DeclareLaunchArgument(
        'headless',
        default_value='false',
        description='Run Gazebo headless when with_sim is true'
    )

    declare_log_level = DeclareLaunchArgument(
        'log_level',
        default_value='debug',
        description='Logging level for map_server and lifecycle_manager'
    )

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1'
    )

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'launch_sim.launch.py')),
        launch_arguments={
            'world': world,
            'headless': headless,
            'use_ros2_control': 'false',
            'start_slam': 'false',
            'start_nav2': 'false',
        }.items(),
        condition=IfCondition(with_sim)
    )

    map_server = LifecycleNode(
        namespace='',
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
        parameters=[{'use_sim_time': use_sim_time, 'yaml_filename': map_yaml}]
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': autostart},
            {'node_names': ['map_server']}
        ]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        additional_env={
            'LIBGL_ALWAYS_SOFTWARE': '1',
            'LIBGL_ALWAYS_INDIRECT': '0',
            'MESA_GL_VERSION_OVERRIDE': '3.3',
            'MESA_GLSL_VERSION_OVERRIDE': '330',
            'GALLIUM_DRIVER': 'llvmpipe',
        },
        condition=IfCondition(rviz)
    )

    map_server_started = RegisterEventHandler(
        OnProcessStart(
            target_action=map_server,
            on_start=[LogInfo(msg='map_server process started')]
        )
    )

    map_server_exited = RegisterEventHandler(
        OnProcessExit(
            target_action=map_server,
            on_exit=[LogInfo(msg='map_server process exited')]
        )
    )

    lifecycle_started = RegisterEventHandler(
        OnProcessStart(
            target_action=lifecycle_manager,
            on_start=[LogInfo(msg='lifecycle_manager_map process started')]
        )
    )

    lifecycle_exited = RegisterEventHandler(
        OnProcessExit(
            target_action=lifecycle_manager,
            on_exit=[LogInfo(msg='lifecycle_manager_map process exited')]
        )
    )

    map_server_active = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=map_server,
            goal_state='active',
            entities=[LogInfo(msg='map_server is active; /map should be available')]
        )
    )

    return LaunchDescription([
        declare_map,
        declare_use_sim_time,
        declare_autostart,
        declare_rviz,
        declare_rviz_config,
        declare_with_sim,
        declare_world,
        declare_headless,
        declare_log_level,
        stdout_linebuf_envvar,
        LogInfo(msg=['saved_map_view: map=', map_yaml]),
        LogInfo(msg=['saved_map_view: rviz_config=', rviz_config]),
        LogInfo(msg=['saved_map_view: use_sim_time=', use_sim_time]),
        LogInfo(msg=['saved_map_view: with_sim=', with_sim]),
        OpaqueFunction(function=_debug_paths),
        sim,
        map_server,
        lifecycle_manager,
        map_server_started,
        map_server_exited,
        lifecycle_started,
        lifecycle_exited,
        map_server_active,
        rviz_node,
    ])
