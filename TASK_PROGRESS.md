# Task Progress

## Current milestone
Saved-map localization/navigation and simple Nav2 mission execution.

## Exact problem being fixed
`bringup_all.launch.py` is the mapping workflow and starts SLAM again. The saved TEKNOFEST map must be used through a separate localization/navigation launch that starts `map_server` + AMCL and does not start online SLAM.

## Files inspected
- `MASTER_CONTEXT.md`
- `TASK_PROGRESS.md`
- `src/articubot_one/launch/launch_sim.launch.py`
- `src/articubot_one/launch/localization_launch.py`
- `src/articubot_one/launch/navigation_launch.py`
- `src/articubot_one/launch/bringup_sim_localization.launch.py`
- `src/articubot_one/launch/rviz_view.launch.py`
- `src/articubot_one/config/nav2_params.yaml`
- `src/articubot_one/config/gz_bridge.yaml`
- `src/articubot_one/maps/teknofest_arena_map.yaml`
- `src/articubot_one/CMakeLists.txt`
- Existing scripts under `src/articubot_one/scripts/`

## Files changed
- `MASTER_CONTEXT.md`
- `TASK_PROGRESS.md`
- `LOCALIZATION_WORKFLOW.md`
- `src/articubot_one/CMakeLists.txt`
- `src/articubot_one/config/mission_points.yaml`
- `src/articubot_one/config/nav2_params.yaml`
- `src/articubot_one/launch/localization_navigation.launch.py`
- `src/articubot_one/scripts/point_recorder.py`

## Workflow split
- Mapping: `bringup_all.launch.py` remains the mapping workflow and starts SLAM Toolbox.
- Saved-map navigation: `localization_navigation.launch.py` starts Gazebo, robot spawn, `ros_gz_bridge`, `map_server`, AMCL, Nav2, and RViz with `start_slam:=false`.

## Exact commands
Build:

```bash
cd $HOME/tekno_robot_simulation
colcon build --symlink-install
source install/setup.bash
```

Run saved-map navigation:

```bash
ros2 launch articubot_one localization_navigation.launch.py \
  map:=$HOME/tekno_robot_simulation/src/articubot_one/maps/teknofest_arena_map.yaml \
  world:=$HOME/tekno_robot_simulation/src/articubot_one/worlds/teknofest_arena.world
```

Point recording:

```bash
ros2 run articubot_one point_recorder.py
```

## Commands run
- `sed -n ...` on relevant launch/config/CMake/script/map files.
- `python3 -m py_compile src/articubot_one/launch/localization_navigation.launch.py src/articubot_one/scripts/point_recorder.py`
- `chmod +x src/articubot_one/scripts/point_recorder.py`
- `colcon build --symlink-install --packages-select articubot_one`
- `ros2 launch articubot_one localization_navigation.launch.py map:=$HOME/tekno_robot_simulation/src/articubot_one/maps/teknofest_arena_map.yaml world:=$HOME/tekno_robot_simulation/src/articubot_one/worlds/teknofest_arena.world`
- `ROS_DISABLE_DAEMON=1 ros2 node list | grep -E "map_server|amcl|slam"`
- `ROS_DISABLE_DAEMON=1 ros2 topic echo /map --once`
- `ROS_DISABLE_DAEMON=1 ros2 topic info /map -v`
- `ROS_DISABLE_DAEMON=1 ros2 topic echo /amcl_pose --once`
- `ROS_DISABLE_DAEMON=1 ros2 action list | grep navigate`
- `ROS_DISABLE_DAEMON=1 ros2 lifecycle get map_server`
- `ROS_DISABLE_DAEMON=1 ros2 lifecycle get amcl`
- `ROS_DISABLE_DAEMON=1 ros2 lifecycle get bt_navigator`
- `ROS_DISABLE_DAEMON=1 timeout 5s ros2 run tf2_ros tf2_echo map odom`
- Test publish to `/clicked_point` while running `ros2 run articubot_one point_recorder.py`.

## Test results
- `articubot_one` focused build passed.
- `localization_navigation.launch.py` starts `launch_sim.launch.py` with `start_slam:=false`, so online SLAM is not launched.
- `map_server` loaded `$HOME/tekno_robot_simulation/src/articubot_one/maps/teknofest_arena_map.yaml` and `$HOME/tekno_robot_simulation/src/articubot_one/maps/teknofest_arena_map.pgm`.
- `/map` publishes from `map_server` as `nav_msgs/msg/OccupancyGrid`, transient local, reliable.
- `/map` dimensions: `163 x 281`, resolution `0.05`, origin `[-1.034, -12.836, 0]`.
- `amcl` is running and active.
- `map_server` is active.
- `bt_navigator` is active.
- Node check showed `/map_server` and `/amcl`; no `slam_toolbox` node.
- `/amcl_pose` publishes after AMCL initial pose. AMCL now starts with default initial pose `x=0, y=0, yaw=0`; RViz `2D Pose Estimate` can still override it.
- Nav2 actions available: `/navigate_to_pose`, `/navigate_through_poses`.
- TF `map -> odom` is publishing after AMCL initial pose.
- `point_recorder.py` test output: `name: TODO, x: 1.230, y: 4.560, z: 0.000`.
- RViz starts with Fixed Frame `map`; this terminal environment still logs the known GLSL Map-display shader error, so visual RViz map rendering was not honestly confirmed from the terminal.

## Remaining issue
Saved-map localization/navigation data path is working. Remaining visualization-only issue: RViz Map display may log a GLSL shader error on this machine, even though `/map`, AMCL, TF, and Nav2 actions are live.

## Next exact task
Use RViz `2D Pose Estimate` to correct localization, then use `Nav2 Goal` for navigation testing. Use `point_recorder.py` with RViz `Publish Point` to fill `src/articubot_one/config/mission_points.yaml`.

## Mission executor update
- Added `src/articubot_one/scripts/mission_executor.py`.
- Installed it through `src/articubot_one/CMakeLists.txt`.
- Updated `src/articubot_one/config/mission_points.yaml` so each point has `yaw` in radians.
- Mission command:

```bash
ros2 run articubot_one mission_executor.py --pickup 1 --drop 3
```

- Mission sequence: `pickup_X`, wait 5 seconds and print `PICKUP_DONE`; `gate`, wait 10 seconds and print `GATE_PERMISSION_GRANTED`; `drop_Y`, wait 5 seconds and print `DROP_DONE`; `start`, print `MISSION_DONE`.
- Uses `/navigate_to_pose`, `map` frame, and YAML `yaw`.
