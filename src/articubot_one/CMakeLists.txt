cmake_minimum_required(VERSION 3.5)
project(articubot_one)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(
    frisby_gamepad_mapping
    test/test_frisby_gamepad_mapping.py
  )
endif()

install(
  DIRECTORY config description launch maps rviz worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_python_install_package(${PROJECT_NAME})

install(
  PROGRAMS
    scripts/scan_frame_fixer.py
    scripts/obstacle_alarm_avoid.py
    scripts/joy_probe.py
    scripts/linux_joystick_node.py
    scripts/joystick_teleop.py
    scripts/keyboard_teleop.py
    scripts/simple_scan_explorer.py
    scripts/stop_robot.py
    scripts/save_map.py
    scripts/point_recorder.py
    scripts/test_qr_image.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  PROGRAMS articubot_one/qr_reader.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME qr_reader
)

install(
  PROGRAMS articubot_one/qr_waypoint_logger.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME qr_waypoint_logger
)

install(
  PROGRAMS articubot_one/save_camera_frame.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME save_camera_frame
)

install(
  PROGRAMS articubot_one/line_sensor_controller.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME line_sensor_controller
)

install(
  PROGRAMS articubot_one/line_sensor_mock_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME line_sensor_mock_node
)

install(
  PROGRAMS articubot_one/gazebo_pose_line_controller.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME gazebo_pose_line_controller
)

install(
  PROGRAMS articubot_one/fork_controller_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME fork_controller_node
)

install(
  PROGRAMS articubot_one/frisby_gamepad_teleop.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME frisby_gamepad_teleop
)

# Standalone import fallback for the installed teleop executable.  The normal
# package installation above remains the primary import path.
install(
  FILES articubot_one/frisby_gamepad_mapping.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  PROGRAMS articubot_one/line_detector_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME line_detector_node
)

install(
  PROGRAMS articubot_one/line_follower_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME line_follower_node
)

install(
  PROGRAMS articubot_one/line_hsv_probe.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME line_hsv_probe
)

install(
  PROGRAMS articubot_one/front_object_detector_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME front_object_detector_node
)

install(
  PROGRAMS articubot_one/forklift_mission_manager.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME forklift_mission_manager
)

install(
  PROGRAMS articubot_one/scenario_state_machine.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME scenario_state_machine
)

install(
  PROGRAMS articubot_one/lidar_obstacle_guard.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME lidar_obstacle_guard
)

install(
  PROGRAMS articubot_one/route_validator.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME route_validator
)

install(
  PROGRAMS articubot_one/route_executor.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME route_executor
)

install(
  PROGRAMS articubot_one/mission_path_publisher.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME mission_path_publisher
)

install(
  PROGRAMS articubot_one/bringup_ready_checker.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME bringup_ready_checker
)

ament_package()
