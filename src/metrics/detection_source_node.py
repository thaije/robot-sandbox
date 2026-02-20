"""
Detection source abstraction node — Step 3.4.

Subscribes to either ground-truth or real-perception detections and
republishes on a unified /detections topic. All downstream nodes subscribe
only to /detections — swapping sources is a single parameter change.

ROS 2 parameter:
  source: "ground_truth"  →  /ground_truth/detections
  source: "yolo"          →  /yolo/detections
"""
from __future__ import annotations


GROUND_TRUTH_TOPIC = "/ground_truth/detections"
YOLO_TOPIC = "/yolo/detections"
UNIFIED_TOPIC = "/detections"


def main() -> None:
    raise NotImplementedError  # TODO: Step 3.4 — rclpy node
