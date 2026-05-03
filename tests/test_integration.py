"""End-to-end integration test — verifies all metrics record correctly.

Requires a full ROS 2 + Gazebo Harmonic installation.
Run with:

    pytest -m integration tests/test_integration.py

Skipped in regular CI (no ``-m integration`` flag).

Scripted movement pattern
-------------------------
1. Move forward 1.5 m  → seeds meters_traveled + exploration_coverage
2. Rotate 180°          → turns around
3. Move forward 1.5 m  → retraces outbound path → seeds revisit_ratio > 0
4. Rotate 360°          → expands exploration_coverage via lidar sweep
"""
from __future__ import annotations

import glob
import json
import os
import subprocess
import sys
import time
import urllib.request
from pathlib import Path

import pytest

REPO  = Path(__file__).resolve().parent.parent
PYTHON = "python3.12"

SCENARIO = str(REPO / "config/scenarios/tests/integration_test.yaml")
RESULTS_PATTERN = str(REPO / "results" / "integration_test_001_*.json")
MISSION_URL = "http://localhost:7400/mission"

SIM_READY_TIMEOUT  = 90   # wall-seconds to wait for sim to become ready
RESULT_FILE_TIMEOUT = 120  # wall-seconds to wait for results JSON after movement


def _sim_env() -> dict[str, str]:
    """Environment for subprocess calls — inherits ROS_DOMAIN_ID from shell."""
    env = os.environ.copy()
    env.setdefault("ROS_DOMAIN_ID", "0")
    return env


def _wait_for_mission_server(timeout: float = SIM_READY_TIMEOUT) -> bool:
    """Poll /mission until it responds or timeout expires. Returns True on success."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            with urllib.request.urlopen(MISSION_URL, timeout=2) as resp:
                data = json.loads(resp.read())
                if data.get("status") == "running":
                    return True
        except Exception:
            pass
        time.sleep(2.0)
    return False


def _wait_for_result(timeout: float = RESULT_FILE_TIMEOUT) -> dict | None:
    """Wait for the integration_test_001 result JSON to appear. Returns parsed dict."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        matches = sorted(glob.glob(RESULTS_PATTERN))
        if matches:
            with open(matches[-1]) as f:
                return json.load(f)
        time.sleep(3.0)
    return None


def _rc(*cmd_args: str) -> int:
    """Run a robot_control.py subcommand and return its exit code."""
    result = subprocess.run(
        [PYTHON, str(REPO / "scripts/robot_control.py"), *cmd_args],
        env=_sim_env(),
        cwd=str(REPO),
    )
    return result.returncode


@pytest.mark.integration
def test_metrics_end_to_end():
    """All metric keys are present in the result JSON and have sensible values."""
    sim = None
    try:
        # ── Start sim ──────────────────────────────────────────────────────────
        sim = subprocess.Popen(
            ["./scripts/run_scenario.sh", SCENARIO, "--headless", "--seed", "1"],
            cwd=str(REPO),
            env=_sim_env(),
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )

        assert _wait_for_mission_server(), (
            f"Mission server did not become ready within {SIM_READY_TIMEOUT}s"
        )

        # ── Scripted movement: forward → 180° → back → spin ───────────────────
        # Each robot_control.py call manages its own rclpy lifecycle.
        _rc("move", "1.5")
        _rc("rotate", "180")
        _rc("move", "1.5")   # retraces outbound path → revisit_ratio > 0
        _rc("rotate", "360") # full spin → expands exploration_coverage

        # ── Wait for the scenario to complete (timeout) and write results ──────
        result = _wait_for_result()
        assert result is not None, (
            f"Result JSON not found after {RESULT_FILE_TIMEOUT}s. "
            f"Pattern: {RESULTS_PATTERN}"
        )

        m = result["raw_metrics"]

        # ── Metric presence ────────────────────────────────────────────────────
        for key in ("meters_traveled", "collision_count", "near_miss_count",
                    "exploration_coverage", "revisit_ratio",
                    "found_ratio", "precision", "duplicate_rate"):
            assert key in m, f"Missing metric key: {key}"

        # ── Metric values ──────────────────────────────────────────────────────
        # Robot moved (at least both 1.5m legs partially completed)
        assert m["meters_traveled"] > 1.0, (
            f"meters_traveled={m['meters_traveled']} — robot did not move"
        )

        # Coverage > 0: lidar rays were traced (even a small spin covers some cells)
        assert m["exploration_coverage"] > 0.0, (
            f"exploration_coverage={m['exploration_coverage']} — lidar raycast did not fire"
        )

        # Return leg retraces outbound cells → some cells visited > once
        assert m["revisit_ratio"] > 0.0, (
            f"revisit_ratio={m['revisit_ratio']} — return path produced no revisits"
        )

        # Collision count is a non-negative integer (metric recorded correctly)
        assert isinstance(m["collision_count"], int) and m["collision_count"] >= 0

        # No detections submitted → precision defaults to 1.0, found_ratio = 0
        assert m["found_ratio"] == 0.0, (
            f"found_ratio={m['found_ratio']} — unexpected detections recorded"
        )
        assert m["precision"] == 1.0, (
            f"precision={m['precision']} — unexpected false positives recorded"
        )

    finally:
        if sim is not None and sim.poll() is None:
            sim.terminate()
            try:
                sim.wait(timeout=10)
            except subprocess.TimeoutExpired:
                sim.kill()
