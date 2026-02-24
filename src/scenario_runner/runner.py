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
        scenario   = self._cfg["scenario"]
        world_cfg  = self._cfg["world"]
        robots_cfg = self._cfg["robots"]

        timeout = float(scenario["timeout_seconds"])

        # ── 1. Generate world SDF (robots embedded inside) ────────────────────
        log.info("Generating world SDF …")
        gen       = WorldGenerator(project_root=_REPO_ROOT)
        world_sdf = gen.generate(self._cfg)
        log.info("Generated world: %s", world_sdf)

        # ── 2. Launch Gazebo + robot bridges (no spawn — model in world SDF) ──
        log.info("Launching simulation …")
        self._launcher.launch(
            world_sdf=world_sdf,
            world_name=world_cfg["template"],
            robots_cfg=robots_cfg,
        )
        robot_names = [r.get("name", f"{r['platform']}_0") for r in robots_cfg]
        log.info("Simulation ready — robots: %s", robot_names)

        # ── 3. Start ROS 2 metrics collection ─────────────────────────────────
        # rclpy is imported lazily to keep this module importable without ROS 2.
        import rclpy  # noqa: PLC0415
        from rclpy.parameter import Parameter  # noqa: PLC0415
        rclpy.init()
        node = rclpy.create_node(
            "arst_metrics",
            parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)],
        )
        metrics = self._build_metrics(node, robots_cfg)

        for m in metrics.values():
            m.start()

        spin_thread = threading.Thread(
            target=rclpy.spin, args=(node,), daemon=True,
        )
        spin_thread.start()
        # Give DDS time to discover the odom/bumper publishers before the run starts.
        time.sleep(2.0)
        log.info("Metrics collection started: %s", list(metrics))

        raw: dict[str, Any] = {}
        outcome = "interrupted"
        elapsed = 0.0
        try:
            # ── 4. Run until success criteria met or timeout ───────────────────
            start_time = time.monotonic()
            outcome    = self._run_until_done(timeout, metrics, start_time)
            elapsed    = time.monotonic() - start_time
            log.info("Scenario ended: outcome=%s  elapsed=%.1fs", outcome, elapsed)

            # ── 5. Collect final metric snapshot ──────────────────────────────
            raw = self._collect_metrics(metrics)
            raw["task_completion_time"] = round(elapsed, 2)

            # ── 6. Evaluate success criteria ──────────────────────────────────
            success, descriptions = evaluate_criteria(self._cfg["success_criteria"], raw)
            for desc in descriptions:
                log.info("Criterion: %s", desc)

            if outcome == "timeout":
                status = "TIMEOUT"
            elif success:
                status = "SUCCESS"
            else:
                status = "FAILURE"

            # ── 7. Score + display scorecard ──────────────────────────────────
            engine    = ScoringEngine(self._cfg["scoring"])
            scorecard = engine.compute(raw, self._cfg)
            scorecard.status          = status
            scorecard.elapsed_seconds = elapsed

            print(render_scorecard(scorecard))

            result_path = write_results(scorecard, self._output_dir)
            log.info("Results written to: %s", result_path)

        except KeyboardInterrupt:
            log.info("Run interrupted by user (Ctrl-C) — shutting down cleanly.")

        finally:
            # ── 8. Cleanup — always runs even on Ctrl-C ────────────────────────
            node.destroy_node()
            try:
                rclpy.shutdown()
            except Exception:
                pass
            self._launcher.shutdown()

        return raw

    # ── Private helpers ────────────────────────────────────────────────────────

    def _build_metrics(self, node: Any, robots_cfg: list[dict]) -> dict[str, Any]:
        """Instantiate metrics for every robot × every listed metric name.

        Keys in the returned dict use the format ``"{metric_name}__{robot_name}"``
        so that ``_collect_metrics`` can aggregate across robots.

        Only metrics with concrete implementations are created; unknown names
        are logged and skipped so the runner doesn't crash on stubs.
        """
        from metrics.meters_traveled import MetersTraveled  # noqa: PLC0415
        from metrics.collision_count import CollisionCount   # noqa: PLC0415
        from metrics.revisit_ratio import RevisitRatio       # noqa: PLC0415

        def _factory(metric_name: str, robot_name: str) -> Any:
            if metric_name == "meters_traveled":
                return MetersTraveled(odom_topic=f"/{robot_name}/odom", node=node)
            if metric_name == "collision_count":
                return CollisionCount(bumper_topic=f"/{robot_name}/bumper_contact", node=node)
            if metric_name == "revisit_ratio":
                return RevisitRatio(odom_topic=f"/{robot_name}/odom", node=node)
            return None

        _implemented = {"meters_traveled", "collision_count", "revisit_ratio"}
        requested = self._cfg.get("metrics", {}).get("collect", [])
        built: dict[str, Any] = {}

        for robot in robots_cfg:
            robot_name = robot.get("name", f"{robot['platform']}_0")
            for metric_name in requested:
                if metric_name not in _implemented:
                    log.debug("Metric %r not yet implemented — skipping.", metric_name)
                    continue
                key = f"{metric_name}__{robot_name}"
                built[key] = _factory(metric_name, robot_name)
                log.debug("Metric registered: %s", key)

        return built

    def _collect_metrics(self, metrics: dict[str, Any]) -> dict[str, Any]:
        """Collect per-robot results and compute aggregated values.

        For a single robot the result is identical to the previous flat format.
        For multiple robots, sum-aggregates are used for count/distance metrics
        and averages for ratio metrics; per-robot breakdowns are included as
        ``"{metric}__{robot_name}"`` entries.
        """
        # Gather per-robot results: {robot_name: {metric_name: value}}
        per_robot: dict[str, dict] = {}
        for key, m in metrics.items():
            metric_name, robot_name = key.split("__", 1)
            per_robot.setdefault(robot_name, {}).update(m.get_result())

        # Aggregate across robots.
        # Numeric values are summed (or averaged for ratio metrics).
        # List values (e.g. collision_events) are concatenated.
        # Non-numeric / non-list values are passed through as-is.
        _avg_metrics = {"revisit_ratio"}
        sums: dict[str, float] = {}
        lists: dict[str, list] = {}
        counts: dict[str, int] = {}
        for robot_results in per_robot.values():
            for mname, value in robot_results.items():
                if isinstance(value, list):
                    lists.setdefault(mname, []).extend(value)
                elif isinstance(value, (int, float)):
                    sums[mname] = sums.get(mname, 0.0) + value
                    counts[mname] = counts.get(mname, 0) + 1

        combined: dict[str, Any] = {}
        for mname, total in sums.items():
            n = counts[mname]
            combined[mname] = total / n if mname in _avg_metrics else total
        combined.update(lists)

        # Per-robot breakdown for multi-robot runs
        if len(per_robot) > 1:
            for robot_name, robot_results in per_robot.items():
                for mname, value in robot_results.items():
                    combined[f"{mname}__{robot_name}"] = value

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
