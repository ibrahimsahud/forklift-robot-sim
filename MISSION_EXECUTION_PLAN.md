# Mission Execution Plan

## Active Architecture

The active TEKNOFEST scenario executor is:

```bash
ros2 run articubot_one route_executor --route A1_TO_B3 --execute
```

Route data comes from:

- `src/articubot_one/config/route_waypoints.yaml`
- `src/articubot_one/config/mission_routes.yaml`

Saved-map navigation uses:

- `src/articubot_one/maps/teknofest_arena_map_new.yaml`
- `src/articubot_one/launch/localization_navigation.launch.py`

Nav2 handles only road and staging goals:

- `START`
- `D1` through `D6`
- `GATE_OUT`
- `GATE_IN`
- `A*_STAGING`
- `B*_STAGING`

`A*_DOCK` and `B*_DOCK` are actions, not Nav2 goals. The executor uses them as local docking references and never sends them to `/navigate_to_pose`.

`Q1` through `Q9` are QR/reference checks, not duplicate navigation goals.

## Active Files

- `src/articubot_one/articubot_one/route_executor.py`
- `src/articubot_one/articubot_one/route_config.py`
- `src/articubot_one/articubot_one/route_validator.py`
- `src/articubot_one/config/route_waypoints.yaml`
- `src/articubot_one/config/mission_routes.yaml`
- `src/articubot_one/launch/localization_navigation.launch.py`

## Deprecated Files

These are kept for reference only and are not the active route execution path:

- `src/articubot_one/scripts/mission_executor.py`
- `src/articubot_one/articubot_one/forklift_mission_manager.py`
- `src/articubot_one/articubot_one/scenario_state_machine.py`

`mission_executor.py` is no longer installed by CMake.

## Commands

Launch saved-map navigation:

```bash
ros2 launch articubot_one localization_navigation.launch.py \
  world:=$HOME/tekno_robot_simulation/src/articubot_one/worlds/teknofest_arena.world \
  map:=$HOME/tekno_robot_simulation/src/articubot_one/maps/teknofest_arena_map_new.yaml \
  use_ros2_control:=true \
  headless:=false \
  rviz:=true \
  start_mission_nodes:=false
```

Validate routes:

```bash
ros2 run articubot_one route_validator --show-expanded
```

Navigation-only scenario test:

```bash
ros2 run articubot_one route_executor --route A1_TO_B3 --execute --nav-only
```

Skip-dock scenario test:

```bash
ros2 run articubot_one route_executor --route A1_TO_B3 --execute --no-dock
```

Placeholder full scenario:

```bash
ros2 run articubot_one route_executor --route A1_TO_B3 --execute
```

## Current Behavior

- `--nav-only` visits Nav2 road/staging goals and skips every `*_DOCK`.
- `--no-dock` behaves like navigation-only but explicitly logs each skipped dock action.
- Full execution uses placeholder docking:
  - stop
  - rotate 180 degrees
  - short controlled forward move on `/cmd_vel_docking`
  - stop
  - log fork placeholder without sending `LIFT_UP` or `LIFT_DOWN`
  - reverse
  - rotate back

## Known Limitations

- Line following hooks are not active yet. The current route proof relies on Nav2 for road/staging points and controlled odom motion for placeholder docking.
- QR checks are non-blocking references unless `/qr_code` is already publishing a fresh expected value.
- Real fork lift/drop commands are intentionally not sent by placeholder full execution.
- `lidar_obstacle_guard` can block the old `forklift_mission_manager` path, but it is not part of the active `route_executor` launch when `start_mission_nodes:=false`.

## Latest Live Test

Tested route:

```bash
ros2 run articubot_one route_executor --route A1_TO_B3 --execute --nav-only
```

Result:

- Reached `START`
- Reached `D1`
- Reached `A1_STAGING`
- Skipped `A1_DOCK`; it was not sent to Nav2
- Continued to `D1`
- Reached `D2`
- Reached `D3`
- Reached `D4`
- Reached `D6`
- Reached `B3_STAGING`
- Skipped `B3_DOCK`; it was not sent to Nav2

Runtime artifact:

- `test_artifacts/scenario_A1_TO_B3_clean_nav_only_runtime/`

## Next Steps

- Run `--no-dock` and full placeholder execution after the navigation-only route is accepted.
- Add a reliable QR-confirmation gate once QR reader is included in the route launch.
- Replace placeholder docking with odom/IMU/TFmini-controlled docking and real fork commands after the route path is stable.
