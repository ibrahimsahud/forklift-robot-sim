import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg = get_package_share_directory('articubot_one')

    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml = LaunchConfiguration('map')
    rviz = LaunchConfiguration('rviz')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation (Gazebo) clock if true')

    declare_map = DeclareLaunchArgument(
        'map', default_value=os.path.join(pkg, 'maps', 'empty_map.yaml'),
        description='Map YAML to load in map_server')

    declare_rviz = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Open RViz with the robot/navigation view')

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'launch_sim.launch.py')),
        launch_arguments={
            'world': os.path.join(pkg, 'worlds', 'empty.world'),
            'headless': 'false',
            'use_ros2_control': 'false',
            'start_slam': 'false',
        }.items()
    )

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'localization_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': map_yaml,
            'params_file': os.path.join(pkg, 'config', 'nav2_params.yaml')
        }.items()
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': os.path.join(pkg, 'config', 'nav2_params.yaml')
        }.items()
    )

    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'rviz_view.launch.py')),
        condition=IfCondition(rviz)
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_map,
        declare_rviz,
        sim,
        localization,
        navigation,
        rviz_launch,
    ])
