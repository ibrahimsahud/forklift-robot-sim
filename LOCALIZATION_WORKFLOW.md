# Saved-Map Localization Workflow

Use this workflow after the arena map has already been created and saved.

Mapping launch stays separate:

```bash
ros2 launch articubot_one bringup_all.launch.py \
  world:=$HOME/tekno_robot_simulation/src/articubot_one/worlds/teknofest_arena.world
```

Saved-map navigation uses `localization_navigation.launch.py` and does not start online SLAM.

## Build

```bash
cd $HOME/tekno_robot_simulation
colcon build --symlink-install
source install/setup.bash
```

## Run Saved-Map Navigation

```bash
ros2 launch articubot_one localization_navigation.launch.py \
  map:=$HOME/tekno_robot_simulation/src/articubot_one/maps/teknofest_arena_map.yaml \
  world:=$HOME/tekno_robot_simulation/src/articubot_one/worlds/teknofest_arena.world
```

This starts Gazebo, the robot, `ros_gz_bridge`, `map_server`, AMCL, Nav2, and RViz.

## RViz

- Fixed Frame: `map`
- AMCL starts with a conservative default initial pose at `x=0, y=0, yaw=0` so Nav2 can activate.
- Use `2D Pose Estimate` to correct or reset the robot initial pose on the saved map.
- Use `Nav2 Goal` to send the robot to a point.
- Use `Publish Point` to record coordinates for mission points.

## Record Points

In another terminal:

```bash
cd $HOME/tekno_robot_simulation
source install/setup.bash
ros2 run articubot_one point_recorder.py
```

Click points in RViz with `Publish Point`. The helper prints:

```text
name: TODO, x: <x>, y: <y>, z: <z>
```

Copy those values into:

```text
src/articubot_one/config/mission_points.yaml
```

Each mission point also has a `yaw` field in radians. Set it manually for the desired final robot heading at that point.

## Run Mission Executor

After filling `src/articubot_one/config/mission_points.yaml`, run a pickup/drop mission while saved-map navigation is active:

```bash
ros2 run articubot_one mission_executor.py --pickup 1 --drop 3
```

## Checks

```bash
ros2 node list | grep -E "map_server|amcl|slam"
ros2 topic echo /map --once
ros2 topic info /map -v
ros2 topic echo /amcl_pose --once
ros2 action list | grep navigate
```

Expected:

- `map_server` publishes the saved map.
- `amcl` is running.
- `slam_toolbox` is not running.
- `/navigate_to_pose` and `/navigate_through_poses` actions are available.
- `/amcl_pose` publishes after the initial pose is set.
