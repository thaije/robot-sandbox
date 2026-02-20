"""
Abstract base class for all metrics — Step 3.1.

Each metric subscribes to ROS 2 topics, accumulates data, and reports a
final result. Metrics are stateless across resets: call reset() to clear
accumulated data without re-creating the node.
"""
from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Any


class BaseMetric(ABC):

    @abstractmethod
    def start(self) -> None:
        """Subscribe to topics and begin accumulating data."""

    @abstractmethod
    def update(self) -> None:
        """Called periodically if the metric needs a clock tick.
        Most metrics update reactively via topic callbacks instead."""

    @abstractmethod
    def get_result(self) -> dict[str, Any]:
        """Return final computed value(s) as a dict."""

    @abstractmethod
    def reset(self) -> None:
        """Clear accumulated state; ready for a new scenario run."""

    @property
    @abstractmethod
    def name(self) -> str:
        """Unique metric identifier matching the scenario config key."""
