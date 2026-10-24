import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('articubot_one')

    world = LaunchConfiguration('world')
    headless = LaunchConfiguration('headless')
    rviz = LaunchConfiguration('rviz')
    rviz_config = LaunchConfiguration('rviz_config')
    use_ros2_control = LaunchConfiguration('use_ros2_control')
    start_joystick = LaunchConfiguration('start_joystick')
    laser_x = LaunchConfiguration('laser_x')
    laser_y = LaunchConfiguration('laser_y')
    laser_z = LaunchConfiguration('laser_z')

    declare_world = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(pkg, 'worlds', 'empty.world'),
        description='Gazebo world to load'
    )

    declare_headless = DeclareLaunchArgument(
        'headless',
        default_value='false',
        description='Run Gazebo headless'
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

    declare_use_ros2_control = DeclareLaunchArgument(
        'use_ros2_control',
        default_value='true',
        description='Use ros2_control for odom TF and controller_manager'
    )

    declare_start_joystick = DeclareLaunchArgument(
        'start_joystick',
        default_value='false',
        description='Start joystick teleop bridge'
    )

    declare_laser_x = DeclareLaunchArgument(
        'laser_x',
        default_value='0.00',
        description='LiDAR x position in base_link frame'
    )

    declare_laser_y = DeclareLaunchArgument(
        'laser_y',
        default_value='0.00',
        description='LiDAR y position in base_link frame'
    )

    declare_laser_z = DeclareLaunchArgument(
        'laser_z',
        default_value='0.42',
        description='LiDAR z position in base_link frame'
    )

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'launch_sim.launch.py')),
        launch_arguments={
            'world': world,
            'headless': headless,
            'use_ros2_control': use_ros2_control,
            'start_slam': 'true',
            'start_nav2': 'false',
            'start_mission_nodes': 'false',
            'start_joystick': start_joystick,
            'laser_x': laser_x,
            'laser_y': laser_y,
            'laser_z': laser_z,
        }.items()
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        additional_env={
            'LIBGL_ALWAYS_SOFTWARE': '1',
            'LIBGL_ALWAYS_INDIRECT': '0',
            'MESA_GL_VERSION_OVERRIDE': '3.3',
            'MESA_GLSL_VERSION_OVERRIDE': '330',
            'GALLIUM_DRIVER': 'llvmpipe',
        },
        condition=IfCondition(rviz)
    )

    return LaunchDescription([
        declare_world,
        declare_headless,
        declare_rviz,
        declare_rviz_config,
        declare_use_ros2_control,
        declare_start_joystick,
        declare_laser_x,
        declare_laser_y,
        declare_laser_z,
        sim,
        rviz_node,
    ])
