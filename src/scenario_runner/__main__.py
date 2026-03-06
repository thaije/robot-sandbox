"""CLI entry point — Step 4.4.

Usage:
    python -m scenario_runner --scenario config/scenarios/office_explore_detect.yaml
    arst-run --scenario config/scenarios/office_explore_detect.yaml --gui
"""
from __future__ import annotations

import argparse
import random
import sys
from pathlib import Path

from utils.config_loader import load_scenario, ConfigError
from utils.logging_setup import setup_logging
from scenario_runner.runner import ScenarioRunner


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(prog="arst-run", description="Autonomous Robotics Simulation Testbed")
    p.add_argument("--scenario", required=True, help="Path to scenario YAML config")
    p.add_argument("--headless", action="store_true", default=True, help="Run without GUI (default)")
    p.add_argument("--gui", action="store_true", help="Launch with Gazebo GUI")
    p.add_argument("--seed", type=int, default=None, help="Override random seed")
    p.add_argument("--timeout", type=float, default=None, help="Override timeout (seconds)")
    p.add_argument("--output-dir", default="results", help="Output directory for results")
    p.add_argument("--speed", type=float, default=1.0,
                   help="Simulation real-time factor (e.g. 3.0 = 3× speed). "
                        "Actual RTF depends on CPU; gz-sim degrades gracefully if "
                        "the target cannot be sustained.")
    p.add_argument("--enable-oracle", action="store_true", default=False,
                   help="Bridge the bbox oracle camera to /detections (dev/cheat mode).")
    p.add_argument("--enable-pointcloud", action="store_true", default=False,
                   help="Bridge the RGBD point cloud to /ROBOT_NAME/rgbd/points.")
    return p


def main() -> None:
    setup_logging()
    args = build_parser().parse_args()

    try:
        config = load_scenario(args.scenario)
    except ConfigError as e:
        print(f"Config error: {e}", file=sys.stderr)
        sys.exit(1)

    if args.seed is not None:
        config["scenario"]["random_seed"] = args.seed
    elif config["scenario"].get("random_seed") is None:
        seed = random.randint(0, 2**31 - 1)
        config["scenario"]["random_seed"] = seed
        print(f"[arst] No seed specified — using random seed: {seed}  (replay with --seed {seed})")
    if args.timeout is not None:
        config["scenario"]["timeout_seconds"] = args.timeout
    if args.speed != 1.0:
        config["scenario"]["real_time_factor"] = args.speed

    headless = not args.gui
    runner = ScenarioRunner(
        config,
        headless=headless,
        output_dir=Path(args.output_dir),
        enable_oracle=args.enable_oracle,
        enable_pointcloud=args.enable_pointcloud,
    )
    runner.run()


if __name__ == "__main__":
    main()
