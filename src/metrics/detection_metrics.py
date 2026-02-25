"""Detection-derived metrics — Step 3.5."""
from __future__ import annotations

from typing import Any

from metrics.base_metric import BaseMetric
from metrics.object_detection_tracker import ObjectDetectionTracker


class DetectionMetrics(BaseMetric):
    """
    Wraps ObjectDetectionTracker to compute:
      - object_detection_rate       (detected_instances / total_instances)
      - time_to_all_detections      (timestamp of last first-detection)
      - average_time_per_detection
      - false_positive_rate         (0.0 for ground-truth oracle)
      - detection_by_type           ({type: {detected, total}} per object type)

    Parameters
    ----------
    detections_topic:
        ROS 2 topic, e.g. ``/derpbot_0/detections``.
    node:
        An ``rclpy.node.Node`` instance.  Must be provided before calling
        ``start()``.
    total_targets:
        Total number of object *instances* in the scenario.  Used as the
        denominator for object_detection_rate.
    label_map:
        Optional dict mapping string label IDs → ``{"type": str, "instance": int}``
        dicts, as produced by ``WorldGenerator.label_map``.  When provided,
        enables instance-level detection tracking and ``detection_by_type``.
    """

    name = "detection_metrics"

    def __init__(
        self,
        detections_topic: str,
        node: Any,
        total_targets: int = 0,
        label_map: dict[str, dict] | None = None,
    ) -> None:
        self._label_map = label_map or {}
        self._tracker = ObjectDetectionTracker(detections_topic, node, self._label_map)
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
            "detection_by_type": self._compute_by_type(events),
        }

    def _compute_by_type(self, events: list[dict]) -> dict[str, dict]:
        """Build per-type detection summary from events and label_map."""
        # Count totals per type from label_map
        totals: dict[str, int] = {}
        for entry in self._label_map.values():
            t = entry["type"]
            totals[t] = totals.get(t, 0) + 1

        # Count detected per type from events
        detected: dict[str, int] = {}
        for ev in events:
            cid = ev["class_id"]
            if cid in self._label_map:
                t = self._label_map[cid]["type"]
                detected[t] = detected.get(t, 0) + 1

        # Merge — include all known types even if nothing detected
        result: dict[str, dict] = {}
        for t, total in sorted(totals.items()):
            result[t] = {"detected": detected.get(t, 0), "total": total}
        return result

    def reset(self) -> None:
        self._tracker.reset()
