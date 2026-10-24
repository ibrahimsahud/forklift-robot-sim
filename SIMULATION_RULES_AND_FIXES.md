# TEKNOFEST Forklift Simulation Rules and Fix Plan

## Absolute rules

1. DO NOT redesign the map.
2. DO NOT move the gate.
3. DO NOT change gate width.
4. DO NOT reduce distances to make the task easier.
5. DO NOT make QR markers floating in air.
6. DO NOT make QR markers huge.
7. DO NOT start pickup immediately after reading QR.
8. DO NOT skip the green precision line.
9. DO NOT ignore RViz/Nav2 waypoints.
10. DO NOT invent new task logic.

## Correct QR placement

QR markers must be on the floor, not vertical and not floating.

QR requirements:
- position: flat on ground/floor
- orientation: facing upward
- size: small realistic floor marker
- recommended size: 0.20 m x 0.20 m or 0.25 m x 0.25 m
- z height: 0.005 m above floor only to prevent z-fighting
- must be before the precision line, not on top of the pallet
- QR must not be robot-sized

Wrong:
- QR standing vertically like a sign
- QR floating in air
- QR larger than pallet/robot
- QR placed after the line

## Correct pickup logic

Reading QR is NOT pickup.

Correct sequence:

1. Nav2 navigates to pre-pickup waypoint.
2. Camera reads target QR, for example A1.
3. QR only confirms correct pickup zone.
4. Robot continues forward to the green precision line.
5. Robot follows the green line using camera/vision.
6. Robot reaches the pallet approach point.
7. Robot aligns forks with pallet entry.
8. Robot moves slowly forward until forks enter pallet.
9. Then pickup/load action is allowed.

Do not trigger pickup only because QR was read.

## Correct line logic

Green solid line = precision approach line.

Robot must reach and follow this line before pickup/drop.

Black dashed lines = visual/general route only.
They are not the precision pickup line.

The robot must not stop at QR and immediately pick load.
QR is only a localization/confirmation marker.

## Correct one-camera logic

Use one front camera mounted in the fork direction.

After pickup:
1. Robot backs out only a short distance using odometry/IMU.
2. Robot turns 180 degrees.
3. Then camera faces the direction of travel again.
4. Continue Nav2 mission.

Do not drive long distances backward with one camera.

## Correct obstacle logic

Obstacles are detected by:
- LiDAR
- Nav2 costmap

Camera is not the main obstacle detector.

If obstacle appears:
- stop at safe distance
- wait
- continue when obstacle is removed

Do not implement obstacle avoidance unless required.

## Correct robot/pallet dimensions

Use competition dimensions:

Robot:
- body/forklift width: 0.70 m
- total length including forks: 1.00 m
- height: 0.60 m
- fork length: 0.65 m

Load:
- 0.50 m x 0.42 m x 0.36 m

Pallet:
- height: 0.11 m
- fork entry gaps must be visually clear
- use 100/200/100/200/100 mm pattern if possible

## Required fixes now

### Fix 1 — Restore map/gate
Compare current world with original teknofest_arena.world.
Restore:
- original walls
- original gate location
- original gate distance
- original route spacing
- original arena design

Do not simplify the arena.

Test:
Launch Gazebo and confirm the map visually matches the original design.

### Fix 2 — Fix QR models
Replace current QR signs with floor QR markers.

Each QR must be:
- flat on ground
- 0.20–0.25 m square
- z = 0.005
- not floating
- not vertical
- not huge

QR names:
- START
- A1
- A2
- A3
- GATE_OUT
- GATE_IN
- B1
- B2
- B3

Test:
Open Gazebo top/side view.
Confirm all QR markers are flat on floor.

### Fix 3 — Fix pickup/drop zones
For each pickup point A1/A2/A3:
- place QR about 1.5 m before pallet approach
- place green line between QR and pallet
- place pallet/load at the actual pickup point
- robot must drive from QR → line → pallet

For each drop point B1/B2/B3:
- place QR about 1.5 m before drop approach
- place green line between QR and drop zone
- robot must drive from QR → line → drop zone

Test:
Manually drive robot and verify the camera sees:
- QR first
- then green line
- then pallet/drop zone

### Fix 4 — Fix state machine
Implement or update mission logic:

States:
- IDLE
- TASK_RECEIVED
- NAV_TO_PICKUP_STAGING
- QR_PICKUP_CONFIRM
- FOLLOW_LINE_TO_PICKUP
- PRE_DOCK_ALIGN
- STRAIGHT_FORK_INSERT
- VERIFY_FORKS_INSIDE
- VERIFY_SUPPORT_DEPTH
- VERIFY_LATERAL_ALIGNMENT
- VERIFY_HEADING_ALIGNMENT
- LIFT_LOAD
- VERIFY_LOAD_STABLE
- BACK_OUT_FROM_PALLET
- TURN_TO_ROUTE
- NAV_TO_GATE_OUT
- WAIT_GATE_PERMISSION
- PASS_GATE
- NAV_TO_DROPOFF_STAGING
- QR_DROPOFF_CONFIRM
- FOLLOW_LINE_TO_DROPOFF
- PRE_DROPOFF_ALIGN
- STRAIGHT_UNLOAD
- VERIFY_UNLOADED
- BACK_OUT
- NAV_TO_GATE_IN
- WAIT_GATE_PERMISSION_RETURN
- RETURN_TO_START
- MISSION_COMPLETE
- ERROR
- EMERGENCY_STOP

Important:
QR can be read multiple times.
Do not repeat actions from repeated QR readings.
QR meaning depends on state.

Example:
- If state == QR_PICKUP_CONFIRM and qr == target_pickup:
  start FOLLOW_LINE_TO_PICKUP.
- If state == STRAIGHT_FORK_INSERT and the same pickup QR is still visible:
  ignore it.
- If state == VERIFY_LOAD_STABLE and the same pickup QR is still visible:
  ignore it.

Test:
Run simulated mission with repeated QR readings.
Verify pickup does not start at QR.

### Fix 5 — Fix navigation waypoints
Use the existing RViz/Nav2 waypoints.
Do not ignore them.
Robot must navigate to:
- pickup prepoint
- gate point
- dropoff prepoint
- return/start point

The robot must not randomly rotate or create shortcuts.

Test:
Send goal to each waypoint individually.
Verify robot reaches each point before using full mission.

### Fix 6 — Fix motion quality
If robot moves badly, tune control instead of changing the map.

Check:
- wheel joints
- wheel friction
- robot mass/inertia
- cmd_vel topic
- controller parameters
- max velocity
- angular velocity
- acceleration limits
- odom stability
- TF tree

Do not solve motion problems by moving objects closer.

Test:
Robot must drive straight 2 m.
Robot must rotate 180 degrees in place.
Robot must reach a Nav2 goal smoothly.

### Fix 7 — Final full test
Run one full mission:

target pickup = A1
target dropoff = B3

Expected sequence:
1. START
2. Nav2 to A1 prepoint
3. Read A1 QR
4. Follow green line
5. Reach pallet
6. Pick load
7. Back out short distance
8. Turn to route
9. Nav2 to GATE_OUT
10. Stop and wait permission
11. Pass gate
12. Nav2 to B3 prepoint
13. Read B3 QR
14. Follow green line
15. Drop load
16. Nav2 to GATE_IN
17. Wait permission
18. Return START
19. MISSION_COMPLETE

Final report must include:
- files changed
- exact tests run
- screenshots or terminal proof
- failed tests and fixes
- current limitations
