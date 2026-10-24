# Topics And Nodes

Audit date: 2026-05-12

This file records the static Phase 0 launch/config inspection. Runtime validation still needs to be collected after each implementation phase with `ros2 node list` and `ros2 topic list`.

## Current Working Launch

```bash
ros2 launch articubot_one launch_sim.launch.py world:=$HOME/tekno_robot_simulation/src/articubot_one/worlds/teknofest_arena.world use_ros2_control:=true
```

Important default launch arguments:

- `headless:=false`
- `use_ros2_control:=false`, overridden to `true` by the current command
- `start_slam:=true`
- `spawn_x:=-5.80`
- `spawn_y:=-3.05`
- `spawn_z:=0.10`
- `spawn_yaw:=1.5708`

## Active Nodes From Current Launch Inspection

Expected nodes/processes from `launch_sim.launch.py`:

- `robot_state_publisher`
- `joy_node`
- `teleop_node`
- `twist_stamper`
- `twist_mux`
- `ros_gz_sim/create`
- `controller_manager/spawner` for `diff_cont`
- `controller_manager/spawner` for `joint_broad`
- `ros_gz_bridge/parameter_bridge`
- `ros_gz_image/image_bridge`
- `static_tf_camera`
- `slam_toolbox` when `start_slam:=true`
- Gazebo `gz sim`

Expected controller names:

- `diff_cont`
- `joint_broad`

## Saved-Map Navigation Nodes

Expected additional nodes from `localization_navigation.launch.py`:

- `map_server`
- `amcl`
- `lifecycle_manager_localization`
- `controller_server`
- `smoother_server`
- `planner_server`
- `behavior_server`
- `bt_navigator`
- `waypoint_follower`
- `velocity_smoother`
- `lifecycle_manager_navigation`
- `rviz2` when enabled

## Current Topics From Launch/Config Inspection

Core simulation and navigation topics:

- `/clock`
- `/tf`
- `/tf_static`
- `/robot_description`
- `/joint_states`
- `/odom`
- `/scan`
- `/map`
- `/amcl_pose`

Camera topics:

- `/camera/image_raw`
- `/camera/camera_info`

Command topics:

- `/cmd_vel`
- `/cmd_vel_joy`
- `/cmd_vel_nav`
- `/cmd_vel_nav_smoothed`
- `/cmd_vel_tracker`
- `/cmd_vel_explore`
- `/cmd_vel_keyboard`
- `/diff_cont/cmd_vel`
- `/diff_cont/cmd_vel_unstamped`

Joystick topics:

- `/joy`

Existing QR helper topics:

- `/qr_data`
- `/qr_debug_image`

Real robot / sensor helper topics:

- `/sensors/distance`
- `/sensors/buzzer_cmd`
- `/sensors/led_cmd`
- `/obstacle_guard/status`

Nav2 actions expected when saved-map Nav2 is launched:

- `/navigate_to_pose`
- `/navigate_through_poses`

## Required Compatibility To Preserve

Frames:

- `map`
- `odom`
- `base_link`
- `base_footprint`
- `chassis`
- `laser_frame`
- `camera_link`
- `camera_link_optical`

Topics:

- `/scan`
- `/odom`
- `/tf`
- `/tf_static`
- `/map`
- `/camera/image_raw` for backward compatibility
- `/camera/camera_info` for backward compatibility
- `/diff_cont/cmd_vel_unstamped`

## Target New Topics

Fork:

- `/fork_cmd` (`std_msgs/String`)
- `/fork_state` (`std_msgs/String`)
- `/fork_joint_controller/commands` or equivalent controller command topic

Bottom QR/line camera:

- `/bottom_camera/image_raw`
- `/bottom_camera/camera_info`
- `/qr_code`
- `/qr_detection/debug_image`
- `/line_offset`
- `/line_angle`
- `/line_detection/debug_image`

Front perception camera:

- `/front_camera/image_raw`
- `/front_camera/camera_info`
- `/front_detections`
- `/front_detection/debug_image`

Mission and docking:

- `/forklift_mission/state`
- `/forklift_mission/status`
- `/cmd_vel_docking`

Safety:

- `/forklift_safety/stop`
- `/forklift_safety/status`

## Current Mismatches To Fix

- QR reader currently defaults to `/camera/image_raw`, `/qr_data`, and `/qr_debug_image`; target topics are `/bottom_camera/image_raw`, `/qr_code`, and `/qr_detection/debug_image`.
- There is one current Gazebo camera; the target is a bottom QR/line camera plus a front perception camera, while keeping `/camera/image_raw` backward compatible.
- The static camera TF in `launch_sim.launch.py` publishes an extra transform from `camera_link_optical` to `robot/base_link/camera`; this should be reviewed once two camera frames exist.
- `mission_executor.py` is a simple point sequence, not the requested forklift state machine.
