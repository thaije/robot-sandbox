"""Object detection tracker — Step 3.5 (support class).

Subscribes to the bounding-box camera topic and records the first time each
object class label is detected.  Used by DetectionMetrics.

The boundingbox_camera sensor (gz-sim-sensors-system, Gazebo Harmonic) emits
gz.msgs.AnnotatedAxisAligned2DBox_V, bridged to
vision_msgs/Detection2DArray via ros_gz_bridge.

Each Detection2D.results[0].hypothesis.class_id carries the integer label
assigned by gz-sim-label-system as a string ("1", "2", "3").
  1 = fire_extinguisher
  2 = first_aid_kit
  3 = hazard_sign
"""
from __future__ import annotations

import json
import time
from pathlib import Path
from typing import Any

_LIVE_DETECTIONS_PATH = Path("/tmp/arst_worlds/detections_live.json")

# Maps gz-sim-label-system integer labels → human-readable class names.
# Must match the <label> values in worlds/models/*/model.sdf.
LABEL_NAMES: dict[str, str] = {
    "1": "fire_extinguisher",
    "2": "first_aid_kit",
    "3": "hazard_sign",
}


class ObjectDetectionTracker:
    """Track first-detection events per object label.

    Parameters
    ----------
    detections_topic:
        ROS 2 topic, e.g. ``/derpbot_0/detections``.
    node:
        An ``rclpy.node.Node`` instance.  Must be provided before calling
        ``start()``.  Kept as ``Any`` to avoid a hard rclpy import at module
        load time.
    label_map:
        Optional dict mapping string label IDs to ``{"type": str, "instance": int}``
        dicts, as produced by ``WorldGenerator.label_map``.  When provided,
        each detected label is resolved to a human-readable name like
        ``"fire_extinguisher #1"``.  Falls back to ``LABEL_NAMES`` if absent.
    """

    def __init__(
        self,
        detections_topic: str,
        node: Any,
        label_map: dict[str, dict] | None = None,
    ) -> None:
        self._topic = detections_topic
        self._node = node
        self._label_map = label_map or {}
        self._start_time: float = 0.0
        self._first_detections: dict[str, float] = {}  # class_id → elapsed_s
        self._sub: Any = None

    def start(self) -> None:
        """Subscribe to the detection topic and record start time."""
        from vision_msgs.msg import Detection2DArray  # noqa: PLC0415

        self._start_time = time.monotonic()
        self._sub = self._node.create_subscription(
            Detection2DArray,
            self._topic,
            self._on_detections,
            10,
        )

    def _on_detections(self, msg: Any) -> None:
        elapsed = time.monotonic() - self._start_time
        new_find = False
        for det in msg.detections:
            for hyp in det.results:
                class_id = hyp.hypothesis.class_id
                # Ignore empty or background label ("0")
                if class_id and class_id != "0" and class_id not in self._first_detections:
                    self._first_detections[class_id] = elapsed
                    new_find = True
        if new_find:
            self._write_live_state()

    def _write_live_state(self) -> None:
        """Write found class_ids to disk so agent tools can show live progress."""
        try:
            _LIVE_DETECTIONS_PATH.parent.mkdir(parents=True, exist_ok=True)
            _LIVE_DETECTIONS_PATH.write_text(
                json.dumps({"found": list(self._first_detections.keys())})
            )
        except Exception:
            pass

    def get_events(self) -> list[dict]:
        """Return first-detection events, one entry per detected instance.

        Each event: ``{"class_id": str, "class_name": str, "timestamp": float}``

        When a ``label_map`` was provided, ``class_name`` is resolved to
        ``"<type> #<1-based-instance>"`` (e.g. ``"fire_extinguisher #2"``).
        Falls back to ``LABEL_NAMES`` for type-level labels.
        """
        def _resolve(cid: str) -> str:
            if cid in self._label_map:
                entry = self._label_map[cid]
                return f"{entry['type']} #{entry['instance'] + 1}"
            return LABEL_NAMES.get(cid, f"label_{cid}")

        return [
            {
                "class_id": cid,
                "class_name": _resolve(cid),
                "timestamp": round(ts, 2),
            }
            for cid, ts in sorted(self._first_detections.items(), key=lambda x: int(x[0]))
        ]

    def reset(self) -> None:
        self._first_detections = {}
        self._start_time = 0.0
