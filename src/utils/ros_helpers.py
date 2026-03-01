"""ROS 2 utility helpers."""
from __future__ import annotations

import subprocess
import time
from typing import Any


def wait_for_topic(topic: str, timeout: float = 30.0, poll_interval: float = 0.5) -> bool:
    """Block until a topic appears in `ros2 topic list`. Returns True if found."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        result = subprocess.run(
            ["ros2", "topic", "list"], capture_output=True, text=True
        )
        if topic in result.stdout.splitlines():
            return True
        time.sleep(poll_interval)
    return False

