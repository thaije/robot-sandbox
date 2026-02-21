"""Revisit ratio metric — Step 3.7."""
from __future__ import annotations

from collections import Counter
from typing import Any

import numpy as np

from metrics.base_metric import BaseMetric


class RevisitRatio(BaseMetric):
    """
    Fraction of visited cells seen more than once.
    ratio = cells_visited_more_than_once / total_cells_visited
    Lower is better (0 = perfectly systematic exploration).
    """

    name = "revisit_ratio"

    def __init__(self, pose_topic: str = "/model/derpbot_0/pose", grid_resolution: float = 0.5) -> None:
        self._pose_topic = pose_topic
        self._resolution = grid_resolution
        self._cell_visits: Counter = Counter()

    def _pose_to_cell(self, x: float, y: float) -> tuple[int, int]:
        return (int(x / self._resolution), int(y / self._resolution))

    def start(self) -> None:
        raise NotImplementedError  # TODO: subscribe to pose topic

    def _on_pose(self, msg: Any) -> None:
        raise NotImplementedError  # extract x, y → _cell_visits[cell] += 1

    def update(self) -> None:
        pass

    def get_result(self) -> dict[str, Any]:
        if not self._cell_visits:
            return {"revisit_ratio": 0.0}
        total = len(self._cell_visits)
        revisited = sum(1 for v in self._cell_visits.values() if v > 1)
        return {"revisit_ratio": round(revisited / total, 4)}

    def reset(self) -> None:
        self._cell_visits.clear()
