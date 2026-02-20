"""
Scenario lifecycle manager — Step 4.3.

Orchestrates: configure → generate world → launch sim → spawn robot →
start metrics → run → evaluate → stop → report.
"""
from __future__ import annotations

import logging
import time
from pathlib import Path
from typing import Any

from utils.config_loader import load_scenario
from utils.logging_setup import setup_logging
from world_manager.world_generator import WorldGenerator
from scenario_runner.launcher import SimulationLauncher
from scenario_runner.reset import ScenarioResetter
from metrics.evaluator import evaluate_criteria
from metrics.scoring import ScoringEngine
from metrics.reporter import render_scorecard, write_results

log = logging.getLogger(__name__)


class ScenarioRunner:
    """Runs a single scenario end-to-end."""

    def __init__(self, config: dict, headless: bool = True, output_dir: Path = Path("results")) -> None:
        self._cfg = config
        self._headless = headless
        self._output_dir = output_dir
        self._launcher = SimulationLauncher(headless=headless)

    def run(self) -> dict[str, Any]:
        """Execute scenario. Returns collected metrics dict."""
        raise NotImplementedError  # TODO: Step 4.3

    def _collect_metrics(self) -> dict[str, Any]:
        raise NotImplementedError

    def _run_until_done(self, timeout: float) -> str:
        """Poll success criteria. Returns 'success' | 'timeout'."""
        raise NotImplementedError
