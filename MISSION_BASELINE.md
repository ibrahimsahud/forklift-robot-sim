# Mission Baseline

This file is the fixed project baseline. Future mission, navigation, mapping,
perception, and safety changes must read this file first and must not violate
these rules unless the user explicitly changes the baseline.

## 1. Active Map

Use this saved map for route and navigation work:

```text
src/articubot_one/maps/teknofest_arena_map_new.yaml
```

Do not replace or edit the active map unless explicitly requested.

## 2. Movement Chain - Do Not Break

The working command chain is:

```text
twist_mux
-> /diff_cont/cmd_vel_unstamped  geometry_msgs/msg/Twist
-> twist_stamper
-> /diff_cont/cmd_vel            geometry_msgs/msg/TwistStamped
-> diff_cont
```

Do not change this chain, topic type pairing, or controller command topics
unless the user explicitly asks for a controller architecture change.

## 3. Active Route Files

The active measured route data lives in:

```text
src/articubot_one/config/route_waypoints.yaml
src/articubot_one/config/mission_routes.yaml
```

Do not change measured waypoint coordinates unless explicitly requested.

## 4. Active Route Executor

`route_executor` is the active mission execution path.

Deprecated unless explicitly re-enabled:

```text
src/articubot_one/articubot_one/forklift_mission_manager.py
src/articubot_one/articubot_one/scenario_state_machine.py
src/articubot_one/scripts/mission_executor.py
```

Do not add new competing mission executors without replacing this baseline.

## 5. Route Logic

Nav2 handles global road and staging navigation:

```text
START
D1..D6
GATE_OUT
GATE_IN
A*_STAGING
B*_STAGING
```

`DOCK` is not a Nav2 waypoint. `A*_DOCK` and `B*_DOCK` are special actions
after the matching staging point.

`Q1..Q9` are QR checks/reference checkpoints. They are not duplicate
navigation goals.

## 6. Known Working Runtime Proof

The following command was live-tested:

```bash
ros2 run articubot_one route_executor --route A1_TO_B3 --execute --nav-only
```

It reached:

```text
START -> D1 -> A1_STAGING -> D1 -> D2 -> D3 -> D4 -> D6 -> B3_STAGING
```

Dock points were skipped correctly in nav-only mode:

```text
A1_DOCK
B3_DOCK
```

Those dock points were not sent to Nav2.

## 7. Current Unsolved Problem

The robot route is not stable enough for pickup/dropoff execution yet.

Current limitations:

- Robot reaches QR/staging areas but does not properly approach pickup/dropoff.
- Camera-only line following is not reliable enough to be the primary
  competition line-tracking method.
- The project needs physical line-sensor tracking so the robot can follow route
  and docking lines reliably.

## 8. Line Tracking Decision

Primary route and docking line tracking should use a physical line sensor array
under the robot.

Camera remains secondary and is used for:

- QR detection
- visual debug
- optional line verification

For Gazebo-only mission-sequence debugging, a pose/map-based `pose_line` mode is
allowed as a simulation aid. It must not be described as the real
competition-compliant line-following method.

Line sensor array becomes primary for:

- keeping the robot centered on the line
- preventing route deviation
- stable route tracking
- docking alignment support

This decision must not break the movement chain or the proven
`route_executor --execute --nav-only` route.

## 9. Competition Execution Policy

`route_executor --execute --competition` must not pretend success.

Competition mode requires:

- a real active line-tracking controller source
- fresh QR detections at required checkpoints
- a real fork command/state path for lift/drop actions

Allowed controller source modes:

- `camera`: intended real vision-based route following path
- `pose_line`: Gazebo debug-only path using known line-segment geometry
- `line_sensor`: intended real physical line-sensor path

If any of these are missing, competition mode should fail explicitly and report
the blocking subsystem.
