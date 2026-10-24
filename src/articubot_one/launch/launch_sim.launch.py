import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, DeclareLaunchArgument, SetEnvironmentVariable, GroupAction, RegisterEventHandler, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression

from launch_ros.actions import Node



def generate_launch_description():


    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!

    package_name='articubot_one' #<--- CHANGE ME

    world = LaunchConfiguration('world')
    headless = LaunchConfiguration('headless')
    use_ros2_control = LaunchConfiguration('use_ros2_control')
    spawn_fork_ros2_control_controller = LaunchConfiguration('spawn_fork_ros2_control_controller')
    start_slam = LaunchConfiguration('start_slam')
    start_nav2 = LaunchConfiguration('start_nav2')
    start_mission_nodes = LaunchConfiguration('start_mission_nodes')
    start_joystick = LaunchConfiguration('start_joystick')
    nav2_params_file = LaunchConfiguration('nav2_params_file')
    laser_x = LaunchConfiguration('laser_x')
    laser_y = LaunchConfiguration('laser_y')
    laser_z = LaunchConfiguration('laser_z')
    spawn_x = LaunchConfiguration('spawn_x')
    spawn_y = LaunchConfiguration('spawn_y')
    spawn_z = LaunchConfiguration('spawn_z')
    spawn_yaw = LaunchConfiguration('spawn_yaw')

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={
                    'use_sim_time': 'true',
                    'use_ros2_control': use_ros2_control,
                    'laser_x': laser_x,
                    'laser_y': laser_y,
                    'laser_z': laser_z,
                }.items()
    )

    joystick = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','joystick.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items(),
                condition=IfCondition(start_joystick)
    )

    twist_mux_params = os.path.join(get_package_share_directory(package_name),'config','twist_mux.yaml')
    line_segments_file = os.path.join(get_package_share_directory(package_name), 'config', 'line_segments.yaml')
    route_waypoints_file = os.path.join(get_package_share_directory(package_name), 'config', 'route_waypoints.yaml')
    twist_mux_ros2_control = Node(
            package="twist_mux",
            executable="twist_mux",
            parameters=[twist_mux_params, {'use_sim_time': True}],
            remappings=[('/cmd_vel_out','/diff_cont/cmd_vel_unstamped')],
            condition=IfCondition(use_ros2_control)
        )

    twist_stamper_ros2_control = Node(
            package='twist_stamper',
            executable='twist_stamper',
            parameters=[{'use_sim_time': True}],
            remappings=[
                ('/cmd_vel_in', '/diff_cont/cmd_vel_unstamped'),
                ('/cmd_vel_out', '/diff_cont/cmd_vel'),
            ],
            condition=IfCondition(use_ros2_control)
        )

    twist_mux_gz = Node(
            package="twist_mux",
            executable="twist_mux",
            parameters=[twist_mux_params, {'use_sim_time': True}],
            remappings=[('/cmd_vel_out','/diff_cont/cmd_vel_unstamped')],
            condition=UnlessCondition(use_ros2_control)
        )


    default_world = os.path.join(
        get_package_share_directory(package_name),
        'worlds',
        'empty.world'
        )

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=default_world,
        description='World to load'
        )

    headless_arg = DeclareLaunchArgument(
        'headless',
        default_value='false',
        description='Run Gazebo server/headless instead of opening the Gazebo GUI'
        )

    use_ros2_control_arg = DeclareLaunchArgument(
        'use_ros2_control',
        default_value='false',
        description='Use gz_ros2_control and controller_manager instead of Gazebo DiffDrive'
        )

    spawn_fork_ros2_control_controller_arg = DeclareLaunchArgument(
        'spawn_fork_ros2_control_controller',
        default_value='false',
        description='Deprecated: fork lift uses Gazebo native JointPositionController in simulation'
        )

    start_slam_arg = DeclareLaunchArgument(
        'start_slam',
        default_value='false',
        description='Optionally start slam_toolbox for live mapping'
        )

    start_nav2_arg = DeclareLaunchArgument(
        'start_nav2',
        default_value='true',
        description='Start Nav2 using the live SLAM map and /scan costmaps'
        )

    start_mission_nodes_arg = DeclareLaunchArgument(
        'start_mission_nodes',
        default_value='true',
        description='Start mission/FSM related nodes'
        )

    start_joystick_arg = DeclareLaunchArgument(
        'start_joystick',
        default_value='true',
        description='Start joystick teleop bridge'
        )

    nav2_params_file_arg = DeclareLaunchArgument(
        'nav2_params_file',
        default_value=os.path.join(
            get_package_share_directory(package_name),
            'config',
            'nav2_params.yaml'
        ),
        description='Nav2 parameters file used when start_nav2 is true'
        )

    laser_x_arg = DeclareLaunchArgument(
        'laser_x',
        default_value='0.00',
        description='LiDAR x position in base_link frame'
        )

    laser_y_arg = DeclareLaunchArgument(
        'laser_y',
        default_value='0.00',
        description='LiDAR y position in base_link frame'
        )

    laser_z_arg = DeclareLaunchArgument(
        'laser_z',
        default_value='0.35',
        description='LiDAR z position in base_link frame'
        )

    spawn_x_arg = DeclareLaunchArgument(
        'spawn_x',
        default_value='-5.80',
        description='Robot spawn X position'
        )

    spawn_y_arg = DeclareLaunchArgument(
        'spawn_y',
        default_value='-3.05',
        description='Robot spawn Y position'
        )

    spawn_z_arg = DeclareLaunchArgument(
        'spawn_z',
        default_value='0.10',
        description='Robot spawn Z position'
        )

    spawn_yaw_arg = DeclareLaunchArgument(
        'spawn_yaw',
        default_value='1.5708',
        description='Robot spawn yaw; 1.5708 faces north in the arena'
        )

    # Point Gazebo to any available model paths (turtlebot3 meshes, local worlds/models)
    gz_resource_paths = []
    try:
        tb3_models = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'models')
        if os.path.isdir(tb3_models):
            gz_resource_paths.append(tb3_models)
    except Exception:
        # Package may not be installed; skip silently
        pass

    pkg_models = os.path.join(get_package_share_directory(package_name), 'models')
    if os.path.isdir(pkg_models):
        gz_resource_paths.append(pkg_models)

    pkg_worlds = os.path.join(get_package_share_directory(package_name), 'worlds')
    if os.path.isdir(pkg_worlds):
        gz_resource_paths.append(pkg_worlds)

    existing_gz_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    if existing_gz_path:
        gz_resource_paths.insert(0, existing_gz_path)

    gz_resource_env = SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.pathsep.join(gz_resource_paths) if gz_resource_paths else '')

    gz_system_plugin_paths = []
    existing_gz_system_path = os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', '')
    if existing_gz_system_path:
        gz_system_plugin_paths.append(existing_gz_system_path)
    gz_system_plugin_paths.append('/opt/ros/jazzy/lib')
    gz_system_plugin_env = SetEnvironmentVariable(
        'GZ_SIM_SYSTEM_PLUGIN_PATH',
        os.pathsep.join(gz_system_plugin_paths)
    )

    gazebo_headless = ExecuteProcess(
        cmd=['gz', 'sim', '-s', '-r', '--render-engine', 'ogre', '--headless-rendering', world],
        output='screen',
        condition=IfCondition(headless),
        additional_env={
            'LIBGL_ALWAYS_SOFTWARE': '1',
            'LIBGL_ALWAYS_INDIRECT': '0',
            'QT_QPA_PLATFORM': 'offscreen',
            'MESA_GL_VERSION_OVERRIDE': '3.3',
            'MESA_GLSL_VERSION_OVERRIDE': '330',
            'GALLIUM_DRIVER': 'llvmpipe',
            # Match the engine to OGRE to prevent OGRE2/EGL segfaults
            'GZ_SIM_RENDER_ENGINE': 'ogre',
            'GZ_SIM_SYSTEM_PLUGIN_PATH': '/opt/ros/jazzy/lib',
        }
    )

    gazebo_gui = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '--render-engine', 'ogre', world],
        output='screen',
        condition=UnlessCondition(headless),
        additional_env={
            'DISPLAY': os.environ.get('DISPLAY', ':0'),
            'LIBGL_ALWAYS_SOFTWARE': '1',
            'LIBGL_ALWAYS_INDIRECT': '0',
            'MESA_GL_VERSION_OVERRIDE': '3.3',
            'MESA_GLSL_VERSION_OVERRIDE': '330',
            'GALLIUM_DRIVER': 'llvmpipe',
            'QT_QUICK_BACKEND': 'software',
            'QT_X11_NO_MITSHM': '1',
            'GZ_SIM_RENDER_ENGINE': 'ogre',
            'GZ_SIM_SYSTEM_PLUGIN_PATH': '/opt/ros/jazzy/lib',
        }
    )

    # Run the spawner node from the ros_gz_sim package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(package='ros_gz_sim', executable='create',
                        arguments=['-topic', 'robot_description',
                                   '-x', spawn_x,
                                   '-y', spawn_y,
                                   '-z', spawn_z,
                                   '-Y', spawn_yaw],
                        output='screen')

    delayed_spawn_entity = TimerAction(
        period=5.0,
        actions=[spawn_entity],
    )


    controller_params_file = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'my_controllers.yaml'
    )

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(use_ros2_control)
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(use_ros2_control)
    )

    fork_joint_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["fork_joint_controller"],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(spawn_fork_ros2_control_controller)
    )


    bridge_params = os.path.join(get_package_share_directory(package_name), 'config', 'gz_bridge.yaml')
    bridge_params_ros2_control = os.path.join(
        get_package_share_directory(package_name), 'config', 'gz_bridge_ros2_control.yaml'
    )
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            '--ros-args',
            '-p',
            PythonExpression([
                "'config_file:=",
                bridge_params_ros2_control,
                "' if '",
                use_ros2_control,
                "' == 'true' else 'config_file:=",
                bridge_params,
                "'"
            ]),
        ],
        parameters=[{'use_sim_time': True}]
    )

    ros_gz_image_bridge = Node(
        package="ros_gz_image",
        executable="image_bridge",
        name="image_bridge_legacy_camera",
        arguments=["/camera/image_raw"],
        parameters=[{'use_sim_time': True}]
    )

    odom_relay = ExecuteProcess(
        cmd=['python3', '-m', 'articubot_one.odom_relay'],
        output='screen',
        condition=IfCondition(use_ros2_control)
    )

    fork_controller = Node(
        package='articubot_one',
        executable='fork_controller_node',
        name='fork_controller',
        parameters=[{
            'use_sim_time': True,
            'publish_ros2_control_command': False,
        }],
        output='screen'
    )

    qr_reader = Node(
        package='articubot_one',
        executable='qr_reader',
        name='qr_reader',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    pose_line_controller = Node(
        package='articubot_one',
        executable='gazebo_pose_line_controller',
        name='gazebo_pose_line_controller',
        parameters=[{
            'use_sim_time': True,
            'line_segments_file': line_segments_file,
            'route_waypoints_file': route_waypoints_file,
        }],
        output='screen'
    )

    line_detector = Node(
        package='articubot_one',
        executable='line_detector_node',
        name='line_detector',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    front_object_detector = Node(
        package='articubot_one',
        executable='front_object_detector_node',
        name='front_object_detector',
        parameters=[{
            'use_sim_time': True,
            'publish_zero_cmd_on_safety': False,
            'safety_stop_topic': '/camera_detection/safety_stop',
            'safety_status_topic': '/camera_detection/safety_status',
            'safety_cmd_topic': '/camera_detection/cmd_vel_safety',
        }],
        output='screen'
    )

    lidar_obstacle_guard = Node(
        package='articubot_one',
        executable='lidar_obstacle_guard',
        name='lidar_obstacle_guard',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    forklift_mission_manager = Node(
        package='articubot_one',
        executable='forklift_mission_manager',
        name='forklift_mission_manager',
        parameters=[{'use_sim_time': True, 'autostart': False}],
        output='screen'
    )

    mission_nodes = GroupAction(
        condition=IfCondition(start_mission_nodes),
        actions=[
            fork_controller,
            qr_reader,
            pose_line_controller,
            line_detector,
            front_object_detector,
            lidar_obstacle_guard,
            forklift_mission_manager,
        ]
    )

    delayed_controller_spawners = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[
                joint_broad_spawner,
                diff_drive_spawner,
                fork_joint_controller_spawner,
            ],
        )
    )



    slam_params_file = os.path.join(get_package_share_directory(package_name),
                                    'config', 'mapper_params_online_async.yaml')

    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('slam_toolbox'),
            'launch',
            'online_async_launch.py'
        )),
        launch_arguments={
            'use_sim_time': 'true',
            'autostart': 'true',
            'use_lifecycle_manager': 'false',
            'slam_params_file': slam_params_file,
        }.items(),
        condition=IfCondition(start_slam)
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory(package_name),
            'launch',
            'navigation_launch.py'
        )),
        launch_arguments={
            'use_sim_time': 'true',
            'params_file': nav2_params_file,
        }.items(),
        condition=IfCondition(start_nav2)
    )

    # Launch them all!
    return LaunchDescription([
        world_arg,
        headless_arg,
        use_ros2_control_arg,
        spawn_fork_ros2_control_controller_arg,
        start_slam_arg,
        start_nav2_arg,
        start_mission_nodes_arg,
        start_joystick_arg,
        nav2_params_file_arg,
        laser_x_arg,
        laser_y_arg,
        laser_z_arg,
        spawn_x_arg,
        spawn_y_arg,
        spawn_z_arg,
        spawn_yaw_arg,
        rsp,
        joystick,
        twist_mux_ros2_control,
        twist_stamper_ros2_control,
        twist_mux_gz,
        gz_resource_env,
        gz_system_plugin_env,
        gazebo_headless,
        gazebo_gui,
        delayed_spawn_entity,
        delayed_controller_spawners,
        ros_gz_bridge,
        ros_gz_image_bridge,
        odom_relay,
        mission_nodes,
        slam_toolbox,
        navigation,
    ])
