# TEKNOFEST Forklift Scenario State Machine

This state machine separates navigation, QR confirmation, line following, load
handling, gate permission, and return logic. It is the contract for the live
`route_executor` path; the offline simulator is only a reference test harness.

## States

`IDLE`, `TASK_RECEIVED`, `NAV_TO_PICKUP_STAGING`, `QR_PICKUP_CONFIRM`,
`FOLLOW_LINE_TO_PICKUP`, `PRE_DOCK_ALIGN`, `STRAIGHT_FORK_INSERT`,
`VERIFY_FORKS_INSIDE`, `VERIFY_SUPPORT_DEPTH`, `VERIFY_LATERAL_ALIGNMENT`,
`VERIFY_HEADING_ALIGNMENT`, `LIFT_LOAD`, `VERIFY_LOAD_STABLE`,
`BACK_OUT_FROM_PALLET`, `TURN_TO_ROUTE`, `NAV_TO_GATE_OUT`,
`WAIT_GATE_PERMISSION`, `PASS_GATE`, `NAV_TO_DROPOFF_STAGING`,
`QR_DROPOFF_CONFIRM`, `FOLLOW_LINE_TO_DROPOFF`, `PRE_DROPOFF_ALIGN`,
`STRAIGHT_UNLOAD`, `VERIFY_UNLOADED`, `BACK_OUT`, `NAV_TO_GATE_IN`,
`WAIT_GATE_PERMISSION_RETURN`, `RETURN_TO_START`, `MISSION_COMPLETE`, `ERROR`,
`EMERGENCY_STOP`.

## Required A1 to B3 Sequence

```text
MISSION_RECEIVED
-> NAV_TO_PICKUP_STAGING
-> QR_PICKUP_CONFIRM
-> FOLLOW_LINE_TO_PICKUP
-> PRE_DOCK_ALIGN
-> STRAIGHT_FORK_INSERT
-> VERIFY_FORKS_INSIDE
-> VERIFY_SUPPORT_DEPTH
-> VERIFY_LATERAL_ALIGNMENT
-> VERIFY_HEADING_ALIGNMENT
-> LIFT_LOAD
-> VERIFY_LOAD_STABLE
-> BACK_OUT_FROM_PALLET
-> TURN_TO_ROUTE
-> NAV_TO_GATE_OUT
-> WAIT_GATE_PERMISSION
-> PASS_GATE
-> NAV_TO_DROPOFF_STAGING
-> QR_DROPOFF_CONFIRM
-> FOLLOW_LINE_TO_DROPOFF
-> PRE_DROPOFF_ALIGN
-> STRAIGHT_UNLOAD
-> VERIFY_UNLOADED
-> BACK_OUT
-> NAV_TO_GATE_IN
-> WAIT_GATE_PERMISSION_RETURN
-> RETURN_TO_START
-> MISSION_COMPLETE
```

Nav2 is used for road/staging moves. QR confirms the current semantic area.
The green precision line is mandatory before pickup/dropoff docking. Docking is
a local forklift action, not a Nav2 goal.

## QR Rule

QR readings are contextual. A QR message is not an action by itself.

- In `QR_PICKUP_CONFIRM`, only the selected pickup QR (`A1`, `A2`, or `A3`) can advance the mission.
- In `STRAIGHT_FORK_INSERT`, repeated pickup QR readings are ignored.
- In `VERIFY_LOAD_STABLE`, repeated pickup QR readings are ignored; only physical pickup verification can advance the mission.
- In `QR_DROPOFF_CONFIRM`, only the selected dropoff QR (`B1`, `B2`, or `B3`) can advance the mission.
- QR readings in unrelated states are logged and ignored.

## Pickup Verification Rule

`LIFT_LOAD` only sends the fork lift command after the following checks pass:

- `VERIFY_FORKS_INSIDE`: both forks entered the pallet opening.
- `VERIFY_SUPPORT_DEPTH`: the forks support the load center, not only the front lip.
- `VERIFY_LATERAL_ALIGNMENT`: fork and pallet lateral error is within tolerance.
- `VERIFY_HEADING_ALIGNMENT`: fork and pallet heading error is within tolerance.

The mission may only continue to `BACK_OUT_FROM_PALLET` after lift height, load
rise, tilt, and support checks pass. If verification fails, the mission enters
`ERROR` and does not navigate to the gate or dropoff.

## Loaded Exit Rule

After pickup the robot must not turn while the forks are still inside the
pallet pocket. The required order is:

```text
BACK_OUT_FROM_PALLET
-> VERIFY_LOAD_STABLE
-> TURN_TO_ROUTE
```

`TURN_TO_ROUTE` means rotate toward the next planned route segment. It is not a
random or unconditional spin.

## Offline Test

Run:

```bash
ros2 run articubot_one scenario_state_machine --simulate --pickup A1 --dropoff B3
```

Expected result: repeated `A1` and `B3` QR readings are ignored after they have already served their state-specific purpose, and the final printed state is `MISSION_COMPLETE`.

Failed pickup test:

```bash
ros2 run articubot_one scenario_state_machine --simulate-failed-pickup --pickup A1 --dropoff B3
```

Expected result: repeated `A1` readings do not trigger pickup or recovery; the final printed state is `ERROR`.
