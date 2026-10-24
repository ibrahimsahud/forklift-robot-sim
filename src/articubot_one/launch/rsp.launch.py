import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

import xacro


def generate_launch_description():

    # Check if we're told to use sim time
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_ros2_control = LaunchConfiguration('use_ros2_control')
    fork_lift_min_height = LaunchConfiguration('fork_lift_min_height')
    laser_x = LaunchConfiguration('laser_x')
    laser_y = LaunchConfiguration('laser_y')
    laser_z = LaunchConfiguration('laser_z')

    # Process the URDF file
    pkg_path = os.path.join(get_package_share_directory('articubot_one'))
    xacro_file = os.path.join(pkg_path,'description','robot.urdf.xacro')
    # robot_description_config = xacro.process_file(xacro_file).toxml()
    robot_description_config = Command([
        'xacro ', xacro_file,
        ' use_ros2_control:=', use_ros2_control,
        ' sim_mode:=', use_sim_time,
        ' fork_lift_min_height:=', fork_lift_min_height,
        ' laser_x:=', laser_x,
        ' laser_y:=', laser_y,
        ' laser_z:=', laser_z,
    ])

    # Create a robot_state_publisher node
    params = {'robot_description': robot_description_config, 'use_sim_time': use_sim_time}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )


    # Launch!
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use sim time if true'),
        DeclareLaunchArgument(
            'use_ros2_control',
            default_value='true',
            description='Use ros2_control if true'),
        DeclareLaunchArgument(
            'fork_lift_min_height',
            default_value='0.0',
            description='Minimum simulated fork carriage height'),
        DeclareLaunchArgument(
            'laser_x',
            default_value='0.00',
            description='LiDAR x position in base_link frame'),
        DeclareLaunchArgument(
            'laser_y',
            default_value='0.00',
            description='LiDAR y position in base_link frame'),
        DeclareLaunchArgument(
            'laser_z',
            default_value='0.35',
            description='LiDAR z position in base_link frame'),

        node_robot_state_publisher
    ])
