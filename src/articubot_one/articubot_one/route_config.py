#!/usr/bin/env python3

from __future__ import annotations

import math
import os
from pathlib import Path
from typing import Any

from ament_index_python.packages import get_package_share_directory

try:
    import yaml
except ImportError as exc:  # pragma: no cover
    raise RuntimeError("PyYAML is required for route configuration support") from exc


EXPECTED_SCENARIOS = (
    "A1_TO_B1",
    "A1_TO_B2",
    "A1_TO_B3",
    "A2_TO_B1",
    "A2_TO_B2",
    "A2_TO_B3",
    "A3_TO_B1",
    "A3_TO_B2",
    "A3_TO_B3",
)

REQUIRED_WAYPOINT_FIELDS = ("x", "y", "yaw")
DOCK_SUFFIX = "_DOCK"

QR_ALIAS_IDS = {
    "Q1": "START",
    "Q2": "A1",
    "Q3": "A2",
    "Q4": "A3",
    "Q5": "GATE_OUT",
    "Q6": "GATE_IN",
    "Q7": "B3",
    "Q8": "B2",
    "Q9": "B1",
}


def _source_path(filename: str) -> Path:
    env_root = os.environ.get("TEKNO_SIM_ROOT")
    candidates = []
    if env_root:
        candidates.append(Path(env_root) / "src" / "articubot_one" / "config" / filename)
        candidates.append(Path(env_root) / "config" / filename)

    package_root = Path(__file__).resolve().parents[1]
    candidates.append(package_root / "config" / filename)

    for candidate in candidates:
        if candidate.is_file():
            return candidate

    return candidates[0]


def config_path(filename: str) -> Path:
    source_path = _source_path(filename)
    if source_path.is_file():
        return source_path
    return Path(get_package_share_directory("articubot_one")) / "config" / filename


def route_waypoints_path() -> Path:
    return config_path("route_waypoints.yaml")


def mission_routes_path() -> Path:
    return config_path("mission_routes.yaml")


def _load_yaml(path: Path) -> dict[str, Any]:
    loaded = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    if not isinstance(loaded, dict):
        raise ValueError(f"{path} must contain a top-level mapping")
    return loaded


def _as_finite_float(value: Any, field_name: str, waypoint_name: str) -> float:
    try:
        numeric = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(
            f"Waypoint '{waypoint_name}' field '{field_name}' must be numeric"
        ) from exc
    if not math.isfinite(numeric):
        raise ValueError(
            f"Waypoint '{waypoint_name}' field '{field_name}' must be finite"
        )
    return numeric


def load_route_waypoints(path: Path | None = None) -> dict[str, Any]:
    route_path = path or route_waypoints_path()
    loaded = _load_yaml(route_path)

    waypoints = loaded.get("waypoints")
    if not isinstance(waypoints, dict) or not waypoints:
        raise ValueError(f"{route_path} must define a non-empty 'waypoints' mapping")

    qr_aliases = loaded.get("qr_aliases", {})
    if not isinstance(qr_aliases, dict):
        raise ValueError(f"{route_path} field 'qr_aliases' must be a mapping")

    normalized_waypoints: dict[str, dict[str, float]] = {}
    for name, pose in waypoints.items():
        if not isinstance(pose, dict):
            raise ValueError(f"Waypoint '{name}' must be a mapping")
        missing = [field for field in REQUIRED_WAYPOINT_FIELDS if field not in pose]
        if missing:
            raise ValueError(
                f"Waypoint '{name}' is missing required fields: {', '.join(missing)}"
            )
        normalized_waypoints[str(name)] = {
            field: _as_finite_float(pose[field], field, str(name))
            for field in REQUIRED_WAYPOINT_FIELDS
        }

    normalized_aliases: dict[str, str] = {}
    for alias, target in qr_aliases.items():
        alias_name = str(alias)
        target_name = str(target)
        if target_name not in normalized_waypoints:
            raise ValueError(
                f"QR alias '{alias_name}' points to unknown waypoint '{target_name}'"
            )
        normalized_aliases[alias_name] = target_name

    map_config = loaded.get("map", {})
    if not isinstance(map_config, dict):
        raise ValueError(f"{route_path} field 'map' must be a mapping")

    return {
        "map": map_config,
        "qr_aliases": normalized_aliases,
        "waypoints": normalized_waypoints,
        "path": route_path,
    }


def load_mission_routes(path: Path | None = None) -> dict[str, list[str]]:
    routes_path = path or mission_routes_path()
    loaded = _load_yaml(routes_path)

    scenarios = loaded.get("scenarios")
    if not isinstance(scenarios, dict) or not scenarios:
        raise ValueError(f"{routes_path} must define a non-empty 'scenarios' mapping")

    normalized: dict[str, list[str]] = {}
    for scenario_name, scenario_data in scenarios.items():
        if not isinstance(scenario_data, dict):
            raise ValueError(f"Scenario '{scenario_name}' must be a mapping")
        route = scenario_data.get("route")
        if not isinstance(route, list) or not route:
            raise ValueError(f"Scenario '{scenario_name}' must define a non-empty route")
        normalized[str(scenario_name)] = [str(item) for item in route]

    return normalized


def expand_route(route: list[str], qr_aliases: dict[str, str]) -> list[str]:
    return [qr_aliases.get(name, name) for name in route]


def is_dock_target(name: str) -> bool:
    target = str(name).upper()
    return target.endswith(DOCK_SUFFIX) or "_DOCK_" in target


def route_actions(route: list[str], qr_aliases: dict[str, str]) -> list[dict[str, str]]:
    actions: list[dict[str, str]] = []
    last_nav_target: str | None = None

    for raw_name in route:
        name = str(raw_name)
        if name in qr_aliases:
            actions.append(
                {
                    "type": "qr_check",
                    "name": name,
                    "target": qr_aliases[name],
                    "qr_id": QR_ALIAS_IDS.get(name, qr_aliases[name]),
                }
            )
            continue

        if is_dock_target(name):
            actions.append({"type": "dock", "name": name, "target": name})
            # A dock changes the physical location even when the preceding NAV
            # target was the station staging point.  Keep a repeated staging
            # point after the dock so it can represent the mandatory back-out.
            last_nav_target = name
            continue

        if name == last_nav_target:
            actions.append({"type": "duplicate_nav_skip", "name": name, "target": name})
            continue

        actions.append({"type": "nav", "name": name, "target": name})
        last_nav_target = name

    return actions


def validate_route_configs(
    waypoints_path_override: Path | None = None,
    routes_path_override: Path | None = None,
) -> dict[str, Any]:
    route_waypoints = load_route_waypoints(waypoints_path_override)
    scenarios = load_mission_routes(routes_path_override)
    waypoints = route_waypoints["waypoints"]
    qr_aliases = route_waypoints["qr_aliases"]

    actual_names = set(scenarios)
    expected_names = set(EXPECTED_SCENARIOS)
    missing_names = sorted(expected_names - actual_names)
    unexpected_names = sorted(actual_names - expected_names)
    if missing_names or unexpected_names:
        problems: list[str] = []
        if missing_names:
            problems.append(f"missing scenarios: {', '.join(missing_names)}")
        if unexpected_names:
            problems.append(f"unexpected scenarios: {', '.join(unexpected_names)}")
        raise ValueError("; ".join(problems))

    expanded_routes: dict[str, list[str]] = {}
    for scenario_name in EXPECTED_SCENARIOS:
        route = scenarios[scenario_name]
        expanded = expand_route(route, qr_aliases)
        missing_refs = [name for name in expanded if name not in waypoints]
        if missing_refs:
            raise ValueError(
                f"Scenario '{scenario_name}' references unknown waypoints: "
                + ", ".join(sorted(set(missing_refs)))
            )
        expanded_routes[scenario_name] = expanded
        route_plan = route_actions(route, qr_aliases)
        missing_action_refs = [
            action["target"]
            for action in route_plan
            if action["type"] in {"nav", "dock"} and action["target"] not in waypoints
        ]
        if missing_action_refs:
            raise ValueError(
                f"Scenario '{scenario_name}' has action references to unknown waypoints: "
                + ", ".join(sorted(set(missing_action_refs)))
            )

    return {
        "waypoints_path": route_waypoints["path"],
        "routes_path": routes_path_override or mission_routes_path(),
        "map": route_waypoints["map"],
        "qr_aliases": qr_aliases,
        "waypoints": waypoints,
        "scenarios": scenarios,
        "expanded_routes": expanded_routes,
        "route_actions": {
            scenario_name: route_actions(scenarios[scenario_name], qr_aliases)
            for scenario_name in EXPECTED_SCENARIOS
        },
    }
