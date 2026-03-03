"""Detection-derived metrics."""
from __future__ import annotations

from typing import Any

from metrics.base_metric import BaseMetric
from metrics.object_detection_tracker import ObjectDetectionTracker


class DetectionMetrics(BaseMetric):
    """Computes detection quality metrics from ObjectDetectionTracker.

    Metrics returned by get_result():
      found_ratio              – TP instances / total instances (recall)
      precision                – TP / (TP + FP); 1.0 when no detections published
      duplicate_rate           – DP / (TP + DP); tracker quality indicator
      mean_localization_error  – mean XY error (metres) for TP positions;
                                 None when only oracle detections were received
      time_to_all_detections   – elapsed seconds when last required TP confirmed
      average_time_per_detection
      false_positive_count
      duplicate_count
      detection_count          – number of confirmed TPs
      detection_events         – list of TP event dicts
      detection_by_type        – {type: {detected, total}}

    Parameters
    ----------
    detections_topic:
        ROS 2 topic, e.g. ``/derpbot_0/detections``.
    node:
        An ``rclpy.node.Node`` instance.
    total_targets:
        Total object instances in the scenario (denominator for found_ratio).
    label_map:
        Dict mapping string label IDs → ``{"type": str, "instance": int,
        "x": float, "y": float}`` dicts, as produced by WorldGenerator.
    match_threshold:
        Max XY distance (metres) for a detection to match a real object. Default 1.5 m.
    """

    name = "detection_metrics"

    def __init__(
        self,
        detections_topic: str,
        node: Any,
        total_targets: int = 0,
        label_map: dict[str, dict] | None = None,
        match_threshold: float = 1.5,
    ) -> None:
        self._label_map = label_map or {}
        self._tracker = ObjectDetectionTracker(
            detections_topic, node, self._label_map, match_threshold
        )
        self._total_targets = total_targets

    def start(self) -> None:
        self._tracker.start()

    def update(self) -> None:
        pass  # reactive — fed by tracker subscription

    def get_result(self) -> dict[str, Any]:
        events = self._tracker.get_tp_events()
        n_tp = len(events)
        n_fp = self._tracker.get_fp_count()
        n_dp = self._tracker.get_dp_count()
        loc_errors = self._tracker.get_location_errors()
        timestamps = [e["timestamp"] for e in events]

        found_ratio = n_tp / self._total_targets if self._total_targets else 0.0
        precision = n_tp / (n_tp + n_fp) if (n_tp + n_fp) > 0 else 1.0
        duplicate_rate = n_dp / (n_tp + n_dp) if (n_tp + n_dp) > 0 else 0.0
        mean_loc_error = round(sum(loc_errors) / len(loc_errors), 3) if loc_errors else None
        time_to_all = max(timestamps) if timestamps else 0.0
        avg_time = time_to_all / n_tp if n_tp else 0.0

        return {
            "found_ratio": round(found_ratio, 4),
            "precision": round(precision, 4),
            "duplicate_rate": round(duplicate_rate, 4),
            "mean_localization_error": mean_loc_error,
            "time_to_all_detections": round(time_to_all, 2),
            "average_time_per_detection": round(avg_time, 2),
            "false_positive_count": n_fp,
            "duplicate_count": n_dp,
            "detection_count": n_tp,
            "detection_events": events,
            "detection_by_type": self._compute_by_type(events),
        }

    def _compute_by_type(self, events: list[dict]) -> dict[str, dict]:
        totals: dict[str, int] = {}
        for entry in self._label_map.values():
            t = entry["type"]
            totals[t] = totals.get(t, 0) + 1

        detected: dict[str, int] = {}
        for ev in events:
            t = ev["class_type"]
            detected[t] = detected.get(t, 0) + 1

        return {
            t: {"detected": detected.get(t, 0), "total": total}
            for t, total in sorted(totals.items())
        }

    def reset(self) -> None:
        self._tracker.reset()
