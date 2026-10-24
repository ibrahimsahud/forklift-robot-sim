from setuptools import find_packages, setup


package_name = "articubot_one"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(),
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Oktant Robotics Team",
    maintainer_email="robotics@oktant.local",
    description="ROS 2 Jazzy simulation of the Tekno differential-drive forklift.",
    license="Proprietary",
    entry_points={
        "console_scripts": [
            "qr_reader = articubot_one.qr_reader:main",
            "qr_waypoint_logger = articubot_one.qr_waypoint_logger:main",
            "save_camera_frame = articubot_one.save_camera_frame:main",
            "line_sensor_controller = articubot_one.line_sensor_controller:main",
            "line_sensor_mock_node = articubot_one.line_sensor_mock_node:main",
            "gazebo_pose_line_controller = articubot_one.gazebo_pose_line_controller:main",
            "fork_controller_node = articubot_one.fork_controller_node:main",
            "line_detector_node = articubot_one.line_detector_node:main",
            "line_follower_node = articubot_one.line_follower_node:main",
            "line_hsv_probe = articubot_one.line_hsv_probe:main",
            "mission_path_publisher = articubot_one.mission_path_publisher:main",
            "front_object_detector_node = articubot_one.front_object_detector_node:main",
            "forklift_mission_manager = articubot_one.forklift_mission_manager:main",
            "scenario_state_machine = articubot_one.scenario_state_machine:main",
            "lidar_obstacle_guard = articubot_one.lidar_obstacle_guard:main",
            "route_validator = articubot_one.route_validator:main",
            "route_executor = articubot_one.route_executor:main",
            "bringup_ready_checker = articubot_one.bringup_ready_checker:main",
            "odom_relay = articubot_one.odom_relay:main",
        ],
    },
)
