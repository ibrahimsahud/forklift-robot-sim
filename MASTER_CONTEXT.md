# Master Context

## Current confirmed architecture
- Main ROS 2 package: `articubot_one`.
- Main mapping launch command:
  `ros2 launch articubot_one bringup_all.launch.py world:=$HOME/tekno_robot_simulation/src/articubot_one/worlds/teknofest_arena.world`
- Saved-map localization/navigation launch command:
  `ros2 launch articubot_one localization_navigation.launch.py map:=$HOME/tekno_robot_simulation/src/articubot_one/maps/teknofest_arena_map.yaml world:=$HOME/tekno_robot_simulation/src/articubot_one/worlds/teknofest_arena.world`
- `bringup_all.launch.py` starts Gazebo Sim, `ros_gz_bridge`, SLAM Toolbox, Nav2, `twist_mux`, and RViz.
- `localization_navigation.launch.py` starts Gazebo Sim, `ros_gz_bridge`, `map_server`, AMCL, Nav2, `twist_mux`, and RViz. It passes `start_slam:=false` to `launch_sim.launch.py`.
- Gazebo now runs headless by default through `bringup_all.launch.py` so the GPU LiDAR renderer returns valid ranges on this machine. RViz still starts by default.
- `/scan` is bridged directly by `ros_gz_bridge`; no scan frame fixer is used.
- RViz may log a GLSL Map-display shader error on this machine. Treat that as a visualization issue separate from the verified `/scan` and `/map` topic data.
- Saved map files:
  - `$HOME/tekno_robot_simulation/src/articubot_one/maps/teknofest_arena_map.yaml`
  - `$HOME/tekno_robot_simulation/src/articubot_one/maps/teknofest_arena_map.pgm`
- Mission points file: `$HOME/tekno_robot_simulation/src/articubot_one/config/mission_points.yaml`; each point uses `x`, `y`, `z`, and `yaw` in the `map` frame.
- Mission executor command pattern:
  `ros2 run articubot_one mission_executor.py --pickup 1 --drop 3`

## LiDAR facts
- LiDAR ROS frame: `laser_frame`.
- LiDAR physical link: `laser_frame`.
- Fixed joint: `laser_joint`, parent `base_link`, child `laser_frame`, origin `0.12 0 0.50`.
- `laser_joint` uses Gazebo `<preserveFixedJoint>true</preserveFixedJoint>` so SDFormat keeps the real sensor on `laser_frame` instead of lumping it into `base_link`.
- `laser_frame` has inertial data only and no collision geometry.
- Gazebo sensor: `<sensor name="laser" type="gpu_ray">` with `<ray>`, 720 horizontal samples, range `0.12` to `10.0`, topic `scan`, `gz_frame_id` `laser_frame`.

## Current milestone scope
Saved-map localization/navigation and simple Nav2 mission execution. Do not start QR, PLC, GUI, FSM, line following, web interface, or auto explorer work unless explicitly requested.
