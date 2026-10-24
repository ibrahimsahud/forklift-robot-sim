# Tekno Robot Simulation

ROS 2 Jazzy simulation workspace for the Tekno differential-drive forklift.
It contains the Gazebo arena, robot model, simulated sensors, RViz profiles,
SLAM, Nav2, QR and line perception, and the mission state machine.

## Safety boundary

This repository is simulation-only. It does not contain Arduino firmware,
firmware upload commands, serial motor bridges, physical-device identifiers,
Wi-Fi credentials or real-robot hardware plugins. Use `real_robot_base` for the
physical robot.

## Requirements

- Ubuntu 24.04
- ROS 2 Jazzy
- Gazebo Harmonic ROS packages (`ros_gz_sim`, `ros_gz_bridge`, `ros_gz_image`)
- `gz_ros2_control`, Nav2, SLAM Toolbox, `twist_mux` and `twist_stamper`
- Python OpenCV and NumPy

Install missing ROS dependencies with `rosdep`:

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
```

## Build and test

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select articubot_one
source install/setup.bash
colcon test --packages-select articubot_one
colcon test-result --verbose
```

`build/`, `install/` and `log/` are generated locally and are not stored in
Git.

## Run

Minimal headless smoke test:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch articubot_one launch_sim.launch.py \
  headless:=true \
  use_ros2_control:=true \
  start_slam:=false \
  start_nav2:=false \
  start_mission_nodes:=false \
  start_joystick:=false
```

Arena mission simulation:

```bash
ros2 launch articubot_one autonomous_competition.launch.py \
  route:=A1_TO_B3 \
  headless:=true
```

The maps, worlds and QR textures required by these launch files are stored
under `src/articubot_one`.

## Layout

- `src/articubot_one/description`: Xacro robot model and Gazebo control.
- `src/articubot_one/worlds`: arenas and QR textures.
- `src/articubot_one/maps`: saved maps for localization.
- `src/articubot_one/config`: controller, SLAM, Nav2 and mission settings.
- `src/articubot_one/launch`: simulation, mapping and navigation launch files.
- `src/articubot_one/articubot_one`: perception and mission nodes.
- `src/articubot_one/test`: offline tests.
