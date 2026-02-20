"""CLI entry point — Step 4.4.

Usage:
    python -m scenario_runner --scenario config/scenarios/office_explore_detect.yaml
    arst-run --scenario config/scenarios/office_explore_detect.yaml --gui
"""
from __future__ import annotations

import argparse
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
    if args.timeout is not None:
        config["scenario"]["timeout_seconds"] = args.timeout

    headless = not args.gui
    runner = ScenarioRunner(config, headless=headless, output_dir=Path(args.output_dir))
    runner.run()


if __name__ == "__main__":
    main()
