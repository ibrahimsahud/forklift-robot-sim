#!/usr/bin/env python3

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from articubot_one.route_config import validate_route_configs


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Validate TEKNOFEST route waypoint data.")
    parser.add_argument("--waypoints-file", type=Path, default=None)
    parser.add_argument("--routes-file", type=Path, default=None)
    parser.add_argument("--show-expanded", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        validation = validate_route_configs(args.waypoints_file, args.routes_file)
    except (OSError, ValueError) as exc:
        print(f"route_validator error: {exc}", file=sys.stderr, flush=True)
        return 1

    scenarios = validation["scenarios"]
    print(f"Validated {len(validation['waypoints'])} waypoints.", flush=True)
    print(f"Validated {len(scenarios)} scenarios.", flush=True)
    print(f"Waypoints file: {validation['waypoints_path']}", flush=True)
    print(f"Routes file: {validation['routes_path']}", flush=True)
    print(f"Map YAML: {validation['map'].get('yaml', 'n/a')}", flush=True)
    print("Scenario names:", flush=True)
    for name in scenarios:
        print(f"  - {name}", flush=True)

    if args.show_expanded:
        print("Expanded routes:", flush=True)
        for name, route in validation["expanded_routes"].items():
            print(f"  {name}: {' -> '.join(route)}", flush=True)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
