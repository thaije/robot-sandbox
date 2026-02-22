"""
Scenario lifecycle manager — Step 4.3.

Orchestrates: configure → generate world → launch sim → spawn robot →
start metrics → run → evaluate → stop → report.

Usage (via CLI)
---------------
    python -m scenario_runner --scenario config/scenarios/office_explore_detect.yaml

Or directly:
    from utils.config_loader import load_scenario
    from scenario_runner.runner import ScenarioRunner

    cfg = load_scenario("config/scenarios/office_explore_detect.yaml")
    runner = ScenarioRunner(cfg, headless=True)
    metrics = runner.run()
"""
from __future__ import annotations

import logging
import threading
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

# Repo root: src/scenario_runner/ → src/ → repo root
_REPO_ROOT = Path(__file__).resolve().parent.parent.parent

# How often to check success criteria during a run
_POLL_INTERVAL = 2.0


class ScenarioRunner:
    """Runs a single scenario end-to-end."""

    def __init__(
        self,
        config: dict,
        headless: bool = True,
        output_dir: Path = Path("results"),
    ) -> None:
        self._cfg = config
        self._headless = headless
        self._output_dir = Path(output_dir)
        self._launcher = SimulationLauncher(headless=headless)

    # ── Public API ─────────────────────────────────────────────────────────────

    def run(self) -> dict[str, Any]:
        """Execute the scenario end-to-end.

        Returns
        -------
        dict
            Flat dict of all collected metric values (same as raw_metrics in
            the Scorecard).  Useful for programmatic inspection after the run.
        """
        scenario  = self._cfg["scenario"]
        world_cfg = self._cfg["world"]
        robot_cfg = self._cfg["robot"]

        robot_platform = robot_cfg["platform"]
        robot_name     = f"{robot_platform}_0"
        timeout        = float(scenario["timeout_seconds"])

        # ── 1. Generate world SDF ──────────────────────────────────────────────
        log.info("Generating world SDF …")
        gen       = WorldGenerator(project_root=_REPO_ROOT)
        world_sdf = gen.generate(self._cfg)
        log.info("Generated world: %s", world_sdf)

        # ── 2. Launch Gazebo + robot ───────────────────────────────────────────
        log.info("Launching simulation …")
        self._launcher.launch(
            world_sdf=world_sdf,
            world_name=world_cfg["template"],
            robot_platform=robot_platform,
            spawn_pose=robot_cfg.get("spawn_pose", {}),
            robot_name=robot_name,
        )
        log.info("Simulation ready — robot %r spawned.", robot_name)

        # ── 3. Start ROS 2 metrics collection ─────────────────────────────────
        # rclpy is imported lazily to keep this module importable without ROS 2.
        import rclpy  # noqa: PLC0415
        rclpy.init()
        node    = rclpy.create_node("arst_metrics")
        metrics = self._build_metrics(node, robot_name)

        for m in metrics.values():
            m.start()

        spin_thread = threading.Thread(
            target=rclpy.spin, args=(node,), daemon=True,
        )
        spin_thread.start()
        log.info("Metrics collection started: %s", list(metrics))

        # ── 4. Run until success criteria met or timeout ───────────────────────
        start_time = time.monotonic()
        outcome    = self._run_until_done(timeout, metrics, start_time)
        elapsed    = time.monotonic() - start_time
        log.info("Scenario ended: outcome=%s  elapsed=%.1fs", outcome, elapsed)

        # ── 5. Collect final metric snapshot ──────────────────────────────────
        raw = self._collect_metrics(metrics)
        raw["task_completion_time"] = round(elapsed, 2)

        # ── 6. Evaluate success criteria ───────────────────────────────────────
        success, descriptions = evaluate_criteria(self._cfg["success_criteria"], raw)
        for desc in descriptions:
            log.info("Criterion: %s", desc)

        if outcome == "timeout":
            status = "TIMEOUT"
        elif success:
            status = "SUCCESS"
        else:
            status = "FAILURE"

        # ── 7. Score + display scorecard ──────────────────────────────────────
        engine    = ScoringEngine(self._cfg["scoring"])
        scorecard = engine.compute(raw, self._cfg)
        scorecard.status          = status
        scorecard.elapsed_seconds = elapsed

        print(render_scorecard(scorecard))

        result_path = write_results(scorecard, self._output_dir)
        log.info("Results written to: %s", result_path)

        # ── 8. Cleanup ─────────────────────────────────────────────────────────
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass
        self._launcher.shutdown()

        return raw

    # ── Private helpers ────────────────────────────────────────────────────────

    def _build_metrics(self, node: Any, robot_name: str) -> dict[str, Any]:
        """Instantiate metrics for every name listed in scenario metrics.collect.

        Only metrics with concrete implementations are created; unknown names
        are logged and skipped so the runner doesn't crash on stubs.
        """
        from metrics.meters_traveled import MetersTraveled  # noqa: PLC0415
        from metrics.collision_count import CollisionCount   # noqa: PLC0415
        from metrics.revisit_ratio import RevisitRatio       # noqa: PLC0415

        _registry: dict[str, Any] = {
            "meters_traveled": lambda: MetersTraveled(
                odom_topic=f"/{robot_name}/odom",
                node=node,
            ),
            "collision_count": lambda: CollisionCount(
                bumper_topic=f"/{robot_name}/bumper_contact",
                node=node,
            ),
            "revisit_ratio": lambda: RevisitRatio(
                odom_topic=f"/{robot_name}/odom",
                node=node,
            ),
        }

        requested = self._cfg.get("metrics", {}).get("collect", [])
        built: dict[str, Any] = {}
        for name in requested:
            if name in _registry:
                built[name] = _registry[name]()
                log.debug("Metric registered: %s", name)
            else:
                log.debug("Metric %r not yet implemented — skipping.", name)

        return built

    def _collect_metrics(self, metrics: dict[str, Any]) -> dict[str, Any]:
        """Flatten all metric get_result() dicts into a single dict."""
        combined: dict[str, Any] = {}
        for m in metrics.values():
            combined.update(m.get_result())
        return combined

    def _run_until_done(
        self,
        timeout: float,
        metrics: dict[str, Any],
        start_time: float,
    ) -> str:
        """Poll success criteria every _POLL_INTERVAL seconds.

        Returns
        -------
        str
            ``"success"`` if all criteria passed, ``"timeout"`` otherwise.
        """
        deadline = start_time + timeout
        while time.monotonic() < deadline:
            raw = self._collect_metrics(metrics)
            passed, _ = evaluate_criteria(self._cfg["success_criteria"], raw)
            if passed:
                return "success"
            time.sleep(_POLL_INTERVAL)
        return "timeout"
