"""Object detection tracker — Step 3.4."""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

from metrics.base_metric import BaseMetric


@dataclass
class DetectionEvent:
    class_name: str
    timestamp: float       # seconds since scenario start
    distance: float        # meters from robot at detection time


class ObjectDetectionTracker(BaseMetric):
    """Tracks per-object first-detection events from /detections."""

    name = "object_detection_tracker"

    def __init__(
        self,
        detections_topic: str = "/detections",
        pose_topic: str = "/model/turtlebot4/pose",
        class_map: dict[int, str] | None = None,
    ) -> None:
        self._detections_topic = detections_topic
        self._pose_topic = pose_topic
        self._class_map: dict[int, str] = class_map or {}
        self._first_detections: dict[str, DetectionEvent] = {}
        self._start_time: float | None = None

    def start(self) -> None:
        raise NotImplementedError  # TODO: Step 3.4

    def _on_detection(self, msg: Any) -> None:
        raise NotImplementedError

    def update(self) -> None:
        pass

    def get_result(self) -> dict[str, Any]:
        return {
            "detection_events": [
                {"class": k, "timestamp": v.timestamp, "distance": v.distance}
                for k, v in self._first_detections.items()
            ]
        }

    def reset(self) -> None:
        self._first_detections.clear()
        self._start_time = None
