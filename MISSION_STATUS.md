# Mission Status

This file is the live working context for the current TEKNOFEST mission effort.
Read this first before making new mission changes.

## Mission Goal

Complete the full Gazebo mission for route `A1_TO_B3` using:

- `line_source:=line_sensor`
- `real_line_sensor:=false`

Required physical success:

1. Start from `START`
2. Follow the line cleanly to `D1`
3. Reach `A1_STAGING_CALIBRATED`
4. Pass `Q2`
5. Continue to `A1`
6. Enter `A1_DOCK_CALIBRATED`
7. Insert forks correctly into the pallet
8. Start `validation lift`
9. Lift the load visibly off the ground
10. Complete `FULL_LIFT`
11. Carry the load to `B3`
12. Unload safely at `B3`
13. Exit without dropping or dragging the load

Partial milestones are not mission success.

## Fixed Constraints

Do not change these unless the current real blocker proves they are the cause:

- `D1 -> A1_STAGING_CALIBRATED -> Q2` baseline flow
- QR logic
- Nav2
- `B3` logic before `A1` loading succeeds
- pallet mass / geometry / fork PID before lift physics is proven to be the blocker

Waypoint edits are allowed only if full-run evidence proves a point causes bad
alignment. Any such change must record:

- point name
- old coordinates
- why it causes the issue
- proposed new coordinates
- whether the full-run test improved behavior

## Current Proven Baseline

The current useful full-mission baseline is:

- `START -> D1` works with line-follow
- `D1 -> A1_STAGING_CALIBRATED` works with the current hybrid line-sensor path
- `Q2` passes
- the robot continues toward `A1`
- `A1_DOCK_CALIBRATED` starts

## Current Real Blocker

The blocker is inside `A1_DOCK_CALIBRATED`, not `D1` or `Q2`.

Recent proven behaviors:

1. In one clean full run:
   - forks entered
   - support depth became good
   - pre-lift failed because the dock pose was still rejected
2. In another clean full run:
   - the robot oscillated between line-align and insertion states
   - insertion did not fully commit

The current work focus is:

- stabilize `A1` pre-dock / insertion transition
- prevent `retry` from ruining a nearly good inserted pose
- get to a real `validation lift`
- only then diagnose lift physics if lift begins and fails

## Important Retained Changes

These are intentional current changes in the repo:

- `START_TO_D1` line-follow segment exists
- `D1_TO_A1_STAGING_CALIBRATED` can use the branch-preserving fallback
- `route_executor.py` contains staged validation lift logic
- `send_fork_command()` allows near-target success
- `A1` pallet in `teknofest_arena.world` is currently opened for fork entry
- some `A1` inserted-safe lift override logic exists in `route_executor.py`

## Waypoint Experiments Already Tried

Tested and reverted:

- point: `A1_DOCK_CALIBRATED`
- old/current coordinates: `(4.7700, -0.3000, -3.1416)`
- tested coordinates: `(4.7700, -0.3260, -3.1416)`
- result: made `A1` alignment worse
- status: reverted, not retained

Do not repeat this experiment unless a new full-run result proves a different
reason.

## Latest Clean Full-Run Evidence To Trust

Use artifacts under:

- `test_artifacts/autonomous_competition/`
- `test_artifacts/line_debug_A1_STAGING_CALIBRATED/`
- `test_artifacts/line_debug_A1_DOCK_CALIBRATED/`

The most trustworthy current conclusions are:

- `D1 -> Q2` is usable enough to keep frozen
- `A1` is the active blocker
- the next acceptable milestone is not `Q2`; it is:
  - `validation lift started = yes`
  - `load visibly lifted = yes`

## Default Test Command

Use full mission runs by default:

`ros2 launch articubot_one autonomous_competition.launch.py route:=A1_TO_B3 rviz:=false headless:=false line_source:=line_sensor real_line_sensor:=false`

Do not use `stop_after` unless explicitly needed for a focused diagnosis.

## Reporting Rule

Do not report partial software milestones as mission success.

Always report:

- whether the full mission was run without `stop_after`
- first real physical failure
- whether lift actually started
- whether the load visibly rose
- whether the robot continued toward `B3`

