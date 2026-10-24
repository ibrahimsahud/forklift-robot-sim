#!/usr/bin/env python3
"""Deprecated offline FSM simulator.

Use route_executor for live measured-route execution. This script is retained
as a reference-only offline state-machine test harness.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass, field
from enum import Enum
from typing import Iterable


class ScenarioState(Enum):
    IDLE = "IDLE"
    TASK_RECEIVED = "TASK_RECEIVED"
    NAV_TO_PICKUP_STAGING = "NAV_TO_PICKUP_STAGING"
    QR_PICKUP_CONFIRM = "QR_PICKUP_CONFIRM"
    FOLLOW_LINE_TO_PICKUP = "FOLLOW_LINE_TO_PICKUP"
    PRE_DOCK_ALIGN = "PRE_DOCK_ALIGN"
    STRAIGHT_FORK_INSERT = "STRAIGHT_FORK_INSERT"
    VERIFY_FORKS_INSIDE = "VERIFY_FORKS_INSIDE"
    VERIFY_SUPPORT_DEPTH = "VERIFY_SUPPORT_DEPTH"
    VERIFY_LATERAL_ALIGNMENT = "VERIFY_LATERAL_ALIGNMENT"
    VERIFY_HEADING_ALIGNMENT = "VERIFY_HEADING_ALIGNMENT"
    LIFT_LOAD = "LIFT_LOAD"
    VERIFY_LOAD_STABLE = "VERIFY_LOAD_STABLE"
    BACK_OUT_FROM_PALLET = "BACK_OUT_FROM_PALLET"
    TURN_TO_ROUTE = "TURN_TO_ROUTE"
    NAV_TO_GATE_OUT = "NAV_TO_GATE_OUT"
    WAIT_GATE_PERMISSION = "WAIT_GATE_PERMISSION"
    PASS_GATE = "PASS_GATE"
    NAV_TO_DROPOFF_STAGING = "NAV_TO_DROPOFF_STAGING"
    QR_DROPOFF_CONFIRM = "QR_DROPOFF_CONFIRM"
    FOLLOW_LINE_TO_DROPOFF = "FOLLOW_LINE_TO_DROPOFF"
    PRE_DROPOFF_ALIGN = "PRE_DROPOFF_ALIGN"
    STRAIGHT_UNLOAD = "STRAIGHT_UNLOAD"
    VERIFY_UNLOADED = "VERIFY_UNLOADED"
    BACK_OUT = "BACK_OUT"
    NAV_TO_GATE_IN = "NAV_TO_GATE_IN"
    WAIT_GATE_PERMISSION_RETURN = "WAIT_GATE_PERMISSION_RETURN"
    RETURN_TO_START = "RETURN_TO_START"
    MISSION_COMPLETE = "MISSION_COMPLETE"
    ERROR = "ERROR"
    EMERGENCY_STOP = "EMERGENCY_STOP"


PICKUP_QR = {"A1", "A2", "A3"}
DROPOFF_QR = {"B1", "B2", "B3"}
PICKUP_PREPOINTS = {
    "A1": "PICKUP_1",
    "A2": "PICKUP_2",
    "A3": "PICKUP_3",
}
DROPOFF_PREPOINTS = {
    "B1": "DROP_1",
    "B2": "DROP_2",
    "B3": "DROP_3",
}


@dataclass
class ScenarioStateMachine:
    pickup: str = "A1"
    dropoff: str = "B3"
    state: ScenarioState = ScenarioState.IDLE
    history: list[str] = field(default_factory=list)
    load_verified: bool = False

    def __post_init__(self) -> None:
        self.pickup = self.pickup.upper()
        self.dropoff = self.dropoff.upper()
        if self.pickup not in PICKUP_QR:
            raise ValueError(f"pickup must be one of {sorted(PICKUP_QR)}")
        if self.dropoff not in DROPOFF_QR:
            raise ValueError(f"dropoff must be one of {sorted(DROPOFF_QR)}")

    def pickup_prepoint(self) -> str:
        return PICKUP_PREPOINTS.get(self.pickup, self.pickup)

    def dropoff_prepoint(self) -> str:
        return DROPOFF_PREPOINTS.get(self.dropoff, self.dropoff)

    def start_task(self) -> None:
        if self.state not in (
            ScenarioState.IDLE,
            ScenarioState.MISSION_COMPLETE,
            ScenarioState.ERROR,
        ):
            self.ignore(f"start ignored while {self.state.value}")
            return
        self.load_verified = False
        self.transition(ScenarioState.TASK_RECEIVED, f"task {self.pickup}->{self.dropoff}")
        self.transition(
            ScenarioState.NAV_TO_PICKUP_STAGING,
            "dispatch Nav2 pickup staging goal",
        )

    def on_nav_reached(self, target: str) -> None:
        target = target.upper()
        if self.state == ScenarioState.NAV_TO_PICKUP_STAGING and target == self.pickup_prepoint():
            self.transition(ScenarioState.QR_PICKUP_CONFIRM, f"near pickup {target}")
        elif (
            self.state == ScenarioState.NAV_TO_DROPOFF_STAGING
            and target == self.dropoff_prepoint()
        ):
            self.transition(ScenarioState.QR_DROPOFF_CONFIRM, f"near dropoff {target}")
        elif self.state == ScenarioState.NAV_TO_GATE_OUT and target == "GATE_OUT":
            self.transition(ScenarioState.WAIT_GATE_PERMISSION, "reached GATE_OUT")
        elif self.state == ScenarioState.NAV_TO_GATE_IN and target == "GATE_IN":
            self.transition(ScenarioState.WAIT_GATE_PERMISSION_RETURN, "reached GATE_IN")
        elif self.state == ScenarioState.RETURN_TO_START and target == "START":
            self.transition(ScenarioState.MISSION_COMPLETE, "returned to START")
        else:
            self.ignore(f"nav reached {target}")

    def on_qr(self, qr_text: str) -> None:
        qr_text = qr_text.strip().upper()
        if not qr_text:
            return

        if self.state == ScenarioState.QR_PICKUP_CONFIRM:
            if qr_text == self.pickup:
                self.transition(
                    ScenarioState.FOLLOW_LINE_TO_PICKUP,
                    f"confirmed pickup QR {qr_text}",
                )
            else:
                self.ignore(f"QR {qr_text}; waiting for pickup {self.pickup}")
            return

        if self.state == ScenarioState.QR_DROPOFF_CONFIRM:
            if qr_text == self.dropoff:
                self.transition(
                    ScenarioState.FOLLOW_LINE_TO_DROPOFF,
                    f"confirmed dropoff QR {qr_text}",
                )
            else:
                self.ignore(f"QR {qr_text}; waiting for dropoff {self.dropoff}")
            return

        self.ignore(f"QR {qr_text}")

    def on_line_complete(self) -> None:
        if self.state == ScenarioState.FOLLOW_LINE_TO_PICKUP:
            self.transition(ScenarioState.PRE_DOCK_ALIGN, "pickup line aligned")
        elif self.state == ScenarioState.FOLLOW_LINE_TO_DROPOFF:
            self.transition(ScenarioState.PRE_DROPOFF_ALIGN, "dropoff line aligned")
        else:
            self.ignore("line complete")

    def on_final_align_complete(self) -> None:
        if self.state == ScenarioState.PRE_DOCK_ALIGN:
            self.transition(ScenarioState.STRAIGHT_FORK_INSERT, "pickup final aligned")
        elif self.state == ScenarioState.PRE_DROPOFF_ALIGN:
            self.transition(ScenarioState.STRAIGHT_UNLOAD, "dropoff final aligned")
        else:
            self.ignore("final align complete")

    def on_pick_complete(self) -> None:
        if self.state == ScenarioState.STRAIGHT_FORK_INSERT:
            self.transition(ScenarioState.VERIFY_FORKS_INSIDE, "forks inserted")
            self.transition(ScenarioState.VERIFY_SUPPORT_DEPTH, "fork support depth ok")
            self.transition(ScenarioState.VERIFY_LATERAL_ALIGNMENT, "fork lateral alignment ok")
            self.transition(ScenarioState.VERIFY_HEADING_ALIGNMENT, "fork heading alignment ok")
            self.transition(ScenarioState.LIFT_LOAD, "lift command allowed")
            self.transition(ScenarioState.VERIFY_LOAD_STABLE, "load lifted; verify stability")
        else:
            self.ignore("pick complete")

    def on_pickup_verified(self, success: bool) -> None:
        if self.state != ScenarioState.VERIFY_LOAD_STABLE:
            self.ignore("pickup verification")
            return
        if not success:
            self.fail("pickup verification failed; mission stopped")
            return
        self.load_verified = True
        self.transition(ScenarioState.BACK_OUT_FROM_PALLET, "pickup physically verified")

    def on_backout_complete(self) -> None:
        if self.state == ScenarioState.BACK_OUT_FROM_PALLET and self.load_verified:
            self.transition(ScenarioState.TURN_TO_ROUTE, "backed out from pallet")
        elif self.state == ScenarioState.BACK_OUT_FROM_PALLET:
            self.fail("backout blocked because pickup was not verified")
        elif self.state == ScenarioState.BACK_OUT:
            self.transition(ScenarioState.NAV_TO_GATE_IN, "backed out after unload")
        else:
            self.ignore("backout complete")

    def on_turn_complete(self) -> None:
        if self.state == ScenarioState.TURN_TO_ROUTE:
            self.transition(ScenarioState.NAV_TO_GATE_OUT, "turned toward gate out")
        else:
            self.ignore("turn complete")

    def on_gate_permission(self, granted: bool = True) -> None:
        if not granted:
            self.ignore("gate permission denied")
            return
        if self.state == ScenarioState.WAIT_GATE_PERMISSION:
            self.transition(ScenarioState.PASS_GATE, "PLC permission granted")
        elif self.state == ScenarioState.WAIT_GATE_PERMISSION_RETURN:
            self.transition(ScenarioState.RETURN_TO_START, "PLC return permission granted")
        else:
            self.ignore("gate permission")

    def on_gate_passed(self) -> None:
        if self.state == ScenarioState.PASS_GATE:
            self.transition(ScenarioState.NAV_TO_DROPOFF_STAGING, "gate passed")
        else:
            self.ignore("gate passed")

    def on_drop_complete(self) -> None:
        if self.state == ScenarioState.STRAIGHT_UNLOAD and self.load_verified:
            self.load_verified = False
            self.transition(ScenarioState.VERIFY_UNLOADED, "load dropped")
            self.transition(ScenarioState.BACK_OUT, "unload verified")
        elif self.state == ScenarioState.STRAIGHT_UNLOAD:
            self.fail("drop blocked because no verified load is carried")
        else:
            self.ignore("drop complete")

    def emergency_stop(self) -> None:
        self.transition(ScenarioState.EMERGENCY_STOP, "emergency stop")

    def fail(self, reason: str) -> None:
        self.transition(ScenarioState.ERROR, reason)

    def transition(self, next_state: ScenarioState, reason: str) -> None:
        if self.state == next_state:
            self.ignore(reason)
            return
        line = f"{self.state.value} -> {next_state.value}: {reason}"
        print(line, flush=True)
        self.history.append(line)
        self.state = next_state

    def ignore(self, reason: str) -> None:
        line = f"{self.state.value}: ignored {reason}"
        print(line, flush=True)
        self.history.append(line)


def simulate(pickup: str, dropoff: str, pickup_success: bool = True) -> ScenarioStateMachine:
    fsm = ScenarioStateMachine(pickup=pickup, dropoff=dropoff)
    fsm.start_task()
    fsm.on_nav_reached(fsm.pickup_prepoint())
    fsm.on_qr(pickup)
    fsm.on_qr(pickup)
    fsm.on_line_complete()
    fsm.on_final_align_complete()
    fsm.on_qr(pickup)
    fsm.on_pick_complete()
    fsm.on_qr(pickup)
    fsm.on_pickup_verified(pickup_success)
    if not pickup_success:
        return fsm
    fsm.on_backout_complete()
    fsm.on_turn_complete()
    fsm.on_qr(pickup)
    fsm.on_nav_reached("GATE_OUT")
    fsm.on_gate_permission(True)
    fsm.on_gate_passed()
    fsm.on_nav_reached(fsm.dropoff_prepoint())
    fsm.on_qr(dropoff)
    fsm.on_qr(dropoff)
    fsm.on_line_complete()
    fsm.on_final_align_complete()
    fsm.on_drop_complete()
    fsm.on_backout_complete()
    fsm.on_nav_reached("GATE_IN")
    fsm.on_gate_permission(True)
    fsm.on_nav_reached("START")
    return fsm


def parse_args(argv: Iterable[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Simulate the TEKNOFEST forklift FSM.")
    parser.add_argument("--simulate", action="store_true", help="Run offline transition test.")
    parser.add_argument(
        "--simulate-failed-pickup",
        action="store_true",
        help="Run offline transition test where pickup verification fails.",
    )
    parser.add_argument("--pickup", default="A1", choices=sorted(PICKUP_QR))
    parser.add_argument("--dropoff", default="B3", choices=sorted(DROPOFF_QR))
    return parser.parse_args(argv)


def main(argv: Iterable[str] | None = None) -> int:
    args = parse_args(argv)
    if args.simulate or args.simulate_failed_pickup:
        fsm = simulate(
            args.pickup,
            args.dropoff,
            pickup_success=not args.simulate_failed_pickup,
        )
        print(f"final_state={fsm.state.value}", flush=True)
        expected = ScenarioState.ERROR if args.simulate_failed_pickup else ScenarioState.MISSION_COMPLETE
        return 0 if fsm.state == expected else 1
    print("Use --simulate or --simulate-failed-pickup for the offline transition test.", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
