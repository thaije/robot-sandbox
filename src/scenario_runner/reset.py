"""Scenario reset logic — Step 4.5."""
from __future__ import annotations


class ScenarioResetter:
    """Tears down and re-initialises simulation state between runs."""

    def reset(self, launcher: "SimulationLauncher") -> None:  # noqa: F821
        """Full reset: shutdown → relaunch.
        If Gazebo Reset API is available, use it instead."""
        raise NotImplementedError  # TODO: Step 4.5
