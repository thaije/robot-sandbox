"""
Exploration coverage metric — Step 3.6.

Grid-based raycasting using Gazebo ground-truth pose (SLAM-decoupled).
See plan §0.5 for design rationale.
"""
from __future__ import annotations

from pathlib import Path
from typing import Any

import numpy as np

from metrics.base_metric import BaseMetric


class ExplorationCoverage(BaseMetric):
    """
    Subscribes to /scan + ground-truth pose.
    Traces LiDAR rays via skimage.draw.line() (Bresenham) onto a grid.
    Computes coverage % against a PGM ground-truth mask.
    """

    name = "exploration_coverage"

    def __init__(
        self,
        scan_topic: str = "/scan",
        pose_topic: str = "/model/turtlebot4/pose",
        gt_map_yaml: str | None = None,
        grid_resolution: float = 0.5,
        publish_grid: bool = True,
    ) -> None:
        self._scan_topic = scan_topic
        self._pose_topic = pose_topic
        self._gt_map_yaml = gt_map_yaml
        self._resolution = grid_resolution
        self._publish_grid = publish_grid

        self._gt_mask: np.ndarray | None = None
        self._coverage_grid: np.ndarray | None = None
        self._coverage_pct: float = 0.0

    def _load_gt_map(self, yaml_path: str) -> None:
        """Load PGM + YAML ground truth map into self._gt_mask."""
        raise NotImplementedError  # TODO: Step 3.6

    def start(self) -> None:
        raise NotImplementedError  # TODO: subscribe, load map, init grid

    def _on_scan_and_pose(self, scan: Any, pose: Any) -> None:
        """Vectorized ray tracing — target <2ms per call."""
        raise NotImplementedError  # TODO: Step 3.6

    def _compute_coverage(self) -> float:
        if self._gt_mask is None or self._coverage_grid is None:
            return 0.0
        observed = np.count_nonzero(self._coverage_grid & self._gt_mask)
        explorable = np.count_nonzero(self._gt_mask)
        return 100.0 * observed / explorable if explorable else 0.0

    def update(self) -> None:
        self._coverage_pct = self._compute_coverage()

    def get_result(self) -> dict[str, Any]:
        return {"exploration_coverage": round(self._coverage_pct, 2)}

    def reset(self) -> None:
        self._coverage_grid = None
        self._coverage_pct = 0.0


def main() -> None:
    raise NotImplementedError  # TODO: rclpy node entry point
