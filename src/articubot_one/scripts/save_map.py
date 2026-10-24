#!/usr/bin/env python3

import argparse
import subprocess
from datetime import datetime
from pathlib import Path


def default_prefix() -> Path:
    workspace_maps = Path.cwd() / "src" / "articubot_one" / "maps"
    if workspace_maps.is_dir():
        base_dir = workspace_maps
    else:
        base_dir = Path.cwd() / "maps"
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return base_dir / f"map_{stamp}"


def normalize_prefix(value: str | None) -> Path:
    prefix = Path(value).expanduser() if value else default_prefix()
    if prefix.suffix in (".yaml", ".pgm", ".png", ".bmp"):
        prefix = prefix.with_suffix("")
    return prefix


def main() -> int:
    parser = argparse.ArgumentParser(description="Save the current /map with Nav2 map_saver_cli.")
    parser.add_argument(
        "-f",
        "--file",
        dest="prefix",
        help="Map output prefix. Do not include .yaml/.pgm suffix.",
    )
    parser.add_argument("-t", "--map-topic", default="/map", help="Map topic to save.")
    parser.add_argument("--fmt", default="pgm", choices=["pgm", "png", "bmp"], help="Image format.")
    parser.add_argument("--occ", default="0.65", help="Occupied threshold.")
    parser.add_argument("--free", default="0.25", help="Free threshold.")
    parser.add_argument("--timeout", default="10.0", help="Map save timeout in seconds.")
    args = parser.parse_args()

    prefix = normalize_prefix(args.prefix)
    prefix.parent.mkdir(parents=True, exist_ok=True)

    command = [
        "ros2",
        "run",
        "nav2_map_server",
        "map_saver_cli",
        "-f",
        str(prefix),
        "-t",
        args.map_topic,
        "--fmt",
        args.fmt,
        "--occ",
        args.occ,
        "--free",
        args.free,
        "--ros-args",
        "-p",
        "map_subscribe_transient_local:=true",
        "-p",
        f"save_map_timeout:={args.timeout}",
    ]

    print(f"Saving map to {prefix}.yaml and {prefix}.{args.fmt}")
    result = subprocess.run(command, check=False)
    return result.returncode


if __name__ == "__main__":
    raise SystemExit(main())
