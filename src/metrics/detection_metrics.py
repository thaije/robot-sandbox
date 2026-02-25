"""Detection-derived metrics — Step 3.5."""
from __future__ import annotations

from typing import Any

from metrics.base_metric import BaseMetric
from metrics.object_detection_tracker import ObjectDetectionTracker


class DetectionMetrics(BaseMetric):
    """
    Wraps ObjectDetectionTracker to compute:
      - object_detection_rate       (detected_classes / total_classes)
      - time_to_all_detections      (timestamp of last first-detection)
      - average_time_per_detection
      - false_positive_rate         (0.0 for ground-truth oracle)

    Parameters
    ----------
    detections_topic:
        ROS 2 topic, e.g. ``/derpbot_0/detections``.
    node:
        An ``rclpy.node.Node`` instance.  Must be provided before calling
        ``start()``.
    total_targets:
        Number of unique object classes in the scenario (e.g. 3 for
        fire_extinguisher + first_aid_kit + hazard_sign).  Used as the
        denominator for object_detection_rate.
    """

    name = "detection_metrics"

    def __init__(
        self,
        detections_topic: str,
        node: Any,
        total_targets: int = 0,
    ) -> None:
        self._tracker = ObjectDetectionTracker(detections_topic, node)
        self._total_targets = total_targets

    def start(self) -> None:
        self._tracker.start()

    def update(self) -> None:
        pass  # reactive — fed by tracker subscription

    def get_result(self) -> dict[str, Any]:
        events = self._tracker.get_events()
        n_detected = len(events)
        timestamps = [e["timestamp"] for e in events]

        detection_rate = n_detected / self._total_targets if self._total_targets else 0.0
        time_to_all = max(timestamps) if timestamps else 0.0
        avg_time = sum(timestamps) / len(timestamps) if timestamps else 0.0

        return {
            "object_detection_rate": round(detection_rate, 4),
            "time_to_all_detections": round(time_to_all, 2),
            "average_time_per_detection": round(avg_time, 2),
            "false_positive_rate": 0.0,  # ground-truth oracle always 0
            "detection_count": n_detected,
            "detection_events": events,
        }

    def reset(self) -> None:
        self._tracker.reset()
