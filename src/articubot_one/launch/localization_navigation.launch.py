import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('articubot_one')

    default_world = os.path.join(
        pkg,
        'worlds',
        'teknofest_arena.world',
    )
    default_map = os.path.join(
        pkg,
        'maps',
        'teknofest_arena_map_new.yaml',
    )

    world = LaunchConfiguration('world')
    map_yaml = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz = LaunchConfiguration('rviz')
    headless = LaunchConfiguration('headless')
    use_ros2_control = LaunchConfiguration('use_ros2_control')
    start_mission_nodes = LaunchConfiguration('start_mission_nodes')
    spawn_x = LaunchConfiguration('spawn_x')
    spawn_y = LaunchConfiguration('spawn_y')
    spawn_z = LaunchConfiguration('spawn_z')
    spawn_yaw = LaunchConfiguration('spawn_yaw')

    declare_world = DeclareLaunchArgument(
        'world',
        default_value=default_world,
        description='Gazebo SDF world file to load',
    )

    declare_map = DeclareLaunchArgument(
        'map',
        default_value=default_map,
        description='Saved map YAML to load in map_server',
    )

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time',
    )

    declare_rviz = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Open RViz with the saved-map navigation view',
    )

    declare_headless = DeclareLaunchArgument(
        'headless',
        default_value='false',
        description='Run Gazebo headless',
    )

    declare_use_ros2_control = DeclareLaunchArgument(
        'use_ros2_control',
        default_value='true',
        description='Use ros2_control diff drive chain',
    )

    declare_start_mission_nodes = DeclareLaunchArgument(
        'start_mission_nodes',
        default_value='true',
        description='Start mission manager and perception nodes in sim launch',
    )

    declare_spawn_x = DeclareLaunchArgument(
        'spawn_x',
        default_value='-5.50',
        description='Robot spawn X position',
    )

    declare_spawn_y = DeclareLaunchArgument(
        'spawn_y',
        default_value='-3.05',
        description='Robot spawn Y position',
    )

    declare_spawn_z = DeclareLaunchArgument(
        'spawn_z',
        default_value='0.10',
        description='Robot spawn Z position',
    )

    declare_spawn_yaw = DeclareLaunchArgument(
        'spawn_yaw',
        default_value='1.5708',
        description='Robot spawn yaw',
    )

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'launch_sim.launch.py')),
        launch_arguments={
            'world': world,
            'headless': headless,
            'use_ros2_control': use_ros2_control,
            'start_slam': 'false',
            'start_nav2': 'false',
            'start_mission_nodes': start_mission_nodes,
            'start_joystick': 'false',
            'spawn_x': spawn_x,
            'spawn_y': spawn_y,
            'spawn_z': spawn_z,
            'spawn_yaw': spawn_yaw,
        }.items(),
    )

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'localization_launch.py')),
        launch_arguments={
            'map': map_yaml,
            'use_sim_time': use_sim_time,
            'params_file': os.path.join(pkg, 'config', 'nav2_params.yaml'),
        }.items(),
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': os.path.join(pkg, 'config', 'nav2_params.yaml'),
        }.items(),
    )

    delayed_localization_and_navigation = TimerAction(
        period=15.0,
        actions=[localization, navigation],
    )

    rviz_launch = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(rviz),
        arguments=['-d', os.path.join(pkg, 'rviz', 'slam_nav2.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
        additional_env={
            'LIBGL_ALWAYS_SOFTWARE': '1',
            'LIBGL_ALWAYS_INDIRECT': '0',
            'MESA_GL_VERSION_OVERRIDE': '3.3',
            'MESA_GLSL_VERSION_OVERRIDE': '330',
            'GALLIUM_DRIVER': 'llvmpipe',
        },
    )

    return LaunchDescription([
        declare_world,
        declare_map,
        declare_use_sim_time,
        declare_rviz,
        declare_headless,
        declare_use_ros2_control,
        declare_start_mission_nodes,
        declare_spawn_x,
        declare_spawn_y,
        declare_spawn_z,
        declare_spawn_yaw,
        sim,
        delayed_localization_and_navigation,
        rviz_launch,
    ])
