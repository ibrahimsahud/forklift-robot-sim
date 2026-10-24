import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    package_name = 'articubot_one'

    # Path to RViz config file (if exists)
    rviz_config_file = os.path.join(
        get_package_share_directory(package_name),
        'rviz',
        'robot_only.rviz'
    )

    # RViz Node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': True}],
        additional_env={
            'LIBGL_ALWAYS_SOFTWARE': '1',
            'LIBGL_ALWAYS_INDIRECT': '0',
            'MESA_GL_VERSION_OVERRIDE': '3.3',
            'MESA_GLSL_VERSION_OVERRIDE': '330',
            'GALLIUM_DRIVER': 'llvmpipe',
        }
    )

    return LaunchDescription([
        rviz_node
    ])
