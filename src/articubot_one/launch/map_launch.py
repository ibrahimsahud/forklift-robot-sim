import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('articubot_one')

    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock')

    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg, 'config', 'localization_params.yaml'),
        description='SLAM Toolbox localization parameters')

    # SLAM Toolbox in localization mode (loads existing map and localizes)
    # This is simpler than nav2_map_server + amcl + lifecycle_manager
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='localization_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            params_file,
            {'use_sim_time': use_sim_time}
        ]
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_params_file,
        slam_toolbox_node
    ])
