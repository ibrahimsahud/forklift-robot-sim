# Mapping Workflow

Current milestone only: make the existing robot move and create a map.

Do not start QR, PLC, GUI, forklift mission, or competition modules for this workflow.

## 1. Build

From the workspace root:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select articubot_one
source install/setup.bash
```

## 2. Start Simulation, SLAM, Nav2, and RViz

Use the current working command:

```bash
ros2 launch articubot_one bringup_all.launch.py
```

This starts the Gazebo server, RViz, `/scan`, SLAM Toolbox, Nav2, `twist_mux`, and the Gazebo command bridge.

Gazebo runs headless by default so the GPU LiDAR renderer stays stable on this machine while RViz remains available for scan/map visualization. To open the Gazebo GUI on a machine with a working GPU/EGL stack:

```bash
ros2 launch articubot_one bringup_all.launch.py headless:=false
```

### TEKNOFEST Arena World

The default command above still loads `empty.world`.

To run the arena world with outer walls, the middle gate divider, pickup pallets, drop points, internal walls, and visual-only dashed route lines:

```bash
ros2 launch articubot_one bringup_all.launch.py \
  world:=$HOME/tekno_robot_simulation/src/articubot_one/worlds/teknofest_arena.world
```

The arena file is:

```text
src/articubot_one/worlds/teknofest_arena.world
```

The route dashes are visual-only floor markings. They do not have collision geometry and should not block the robot or appear in LiDAR scans.

The arena walls, internal walls, gate posts, pickup pallets, and drop point markers all have collision and visual geometry. Their heights intersect the robot LiDAR scan plane, so they should appear on `/scan`.

To verify `/scan` in RViz:

1. Start the arena world with the launch command above.
2. In RViz, set the fixed frame to `map` or `odom`.
3. Enable or add a `LaserScan` display.
4. Set the LaserScan topic to `/scan`.
5. Drive or rotate the robot with keyboard teleop.
6. Confirm scan points outline the outer walls, middle wall/gate posts, internal walls, pallets, and drop markers.

Recommended RViz LaserScan settings:

```text
Topic: /scan
Reliability Policy: Best Effort
Durability Policy: Volatile
Style: Spheres or Flat Squares
Size (m): 0.08
Color Transformer: FlatColor
Color: yellow
Alpha: 1.0
Decay Time: 0.2
```

Note: on this machine RViz may log a GLSL Map-display shader error. If that happens, trust the CLI checks below for `/scan` and `/map` data first, then debug RViz rendering as a separate visualization issue.

CLI checks:

```bash
ros2 topic echo /scan --once
ros2 topic hz /scan
```

Expected: `/scan` publishes continuously, with finite range values changing as the robot faces nearby walls and pallets. The dashed route lines should not create scan returns.

Gazebo LiDAR check:

- The LiDAR sensor is configured in `src/articubot_one/description/lidar.xacro`.
- Gazebo laser visualization is disabled with `<visualize>false</visualize>`; verify the scan through RViz and CLI data instead of the Gazebo laser fan.
- The scan is a horizontal 2D scan from `laser_frame`; the arena walls, internal walls, gate posts, pickup pallets, and drop markers are tall enough to cross that scan plane.

## 3. Command Routing

The robot is not driven directly by every publisher on `/cmd_vel`.

In simulation, velocity commands go through `twist_mux` and then to:

```text
/diff_cont/cmd_vel_unstamped -> Gazebo DiffDrive cmd_vel
```

Mux inputs:

```text
/cmd_vel                raw/manual test input, priority 5
/cmd_vel_nav_smoothed   Nav2 smoothed output, priority 10
/cmd_vel_tracker        tracker input, priority 20
/cmd_vel_explore        simple mapping explorer, priority 30
/cmd_vel_joy            joystick, priority 100
/cmd_vel_keyboard       keyboard teleop, priority 110
```

Why raw `/cmd_vel` was unreliable:

- Nav2 velocity smoother was also publishing to `/cmd_vel`.
- `twist_mux` was reading `/cmd_vel` as an input.
- That made manual `/cmd_vel` tests compete with Nav2/mux traffic.

Nav2 now publishes smoothed velocity to `/cmd_vel_nav_smoothed`, leaving `/cmd_vel` as a simple low-priority raw test input.

## 4. Keyboard Teleop

In a second terminal:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run articubot_one keyboard_teleop.py
```

Keys:

```text
w/s       forward/back
a/d       rotate left/right
q/e       forward arcs
z/c       reverse arcs
x/space   stop
+/-       speed up/down
Ctrl+C    quit
```

Keyboard teleop publishes to `/cmd_vel_keyboard`, the highest-priority mux input.

## 5. Simple Auto Mapping Explorer

Use this only in open simulation space and keep the stop command ready.

In a second terminal:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run articubot_one simple_scan_explorer.py
```

The explorer subscribes to `/scan` and publishes slow commands to `/cmd_vel_explore`.

It is intentionally simple:

- drive forward when the front sector is clear
- slow down near obstacles
- turn toward the side with more free space when blocked
- publish zero if `/scan` becomes stale

Stop it with `Ctrl+C` or the stop script below.

## 6. Stop Robot

From any terminal with the workspace sourced:

```bash
ros2 run articubot_one stop_robot.py
```

This publishes zero `Twist` commands for a short duration on all active command paths:

```text
/cmd_vel_keyboard
/cmd_vel_explore
/cmd_vel_joy
/cmd_vel_tracker
/cmd_vel
/diff_cont/cmd_vel_unstamped
```

Optional longer stop:

```bash
ros2 run articubot_one stop_robot.py --ros-args -p duration:=3.0
```

## 7. Save Map

After driving enough area for SLAM to build `/map`:

```bash
ros2 run articubot_one save_map.py -f src/articubot_one/maps/my_map
```

Expected output files:

```text
src/articubot_one/maps/my_map.yaml
src/articubot_one/maps/my_map.pgm
```

If no `-f` path is given, the script saves a timestamped map under `src/articubot_one/maps/` when run from the workspace root.

## 8. Quick Checks

Useful checks while mapping:

```bash
ros2 topic echo /scan --once
ros2 topic echo /map --once
ros2 topic echo /diff_cont/cmd_vel_unstamped --once
ros2 node list
```

Expected core topics:

```text
/scan
/map
/odom
/tf
/cmd_vel_keyboard
/cmd_vel_explore
/cmd_vel_nav_smoothed
/diff_cont/cmd_vel_unstamped
```
