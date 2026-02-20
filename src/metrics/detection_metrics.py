"""Detection-derived metrics — Step 3.5."""
from __future__ import annotations

from typing import Any

from metrics.base_metric import BaseMetric


class DetectionMetrics(BaseMetric):
    """
    Computes from detection events:
      - object_detection_rate    (detected / total)
      - time_to_all_detections   (timestamp of last first-detection)
      - average_time_per_detection
      - false_positive_rate      (placeholder; 0.0 for ground-truth oracle)
    """

    name = "detection_metrics"

    def __init__(self, total_targets: int = 0) -> None:
        self._total_targets = total_targets
        self._detection_events: list[dict] = []

    def set_total_targets(self, n: int) -> None:
        self._total_targets = n

    def ingest_events(self, events: list[dict]) -> None:
        """Called by reporter with events from ObjectDetectionTracker."""
        self._detection_events = events

    def start(self) -> None:
        pass  # passive — fed by ObjectDetectionTracker

    def update(self) -> None:
        pass

    def get_result(self) -> dict[str, Any]:
        n_detected = len(self._detection_events)
        timestamps = [e["timestamp"] for e in self._detection_events]

        detection_rate = n_detected / self._total_targets if self._total_targets else 0.0
        time_to_all = max(timestamps) if timestamps else 0.0
        avg_time = sum(timestamps) / len(timestamps) if timestamps else 0.0

        return {
            "object_detection_rate": round(detection_rate, 4),
            "time_to_all_detections": round(time_to_all, 2),
            "average_time_per_detection": round(avg_time, 2),
            "false_positive_rate": 0.0,  # ground-truth oracle always 0
        }

    def reset(self) -> None:
        self._detection_events = []
