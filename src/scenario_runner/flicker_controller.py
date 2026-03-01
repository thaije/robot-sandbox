"""Flickering light controller — Step 3.14.

Toggles named gz-sim lights between ON and OFF states on a configurable period.
Calls the ``/world/<name>/light_config`` **service** directly via gz.transport13
(UserCommands system exposes this as a service, not a pub/sub topic).

Configured via scenario YAML::

    variations:
      flicker:
        - name: "light_corridor_emergency"   # must match a <light name=...> in world SDF
          period_s: 1.0                       # full on/off cycle time
          # on_diffuse / off_diffuse are optional; looked up from lighting preset if omitted

The controller thread runs as a daemon so it dies automatically if the main
process exits without calling stop().
"""
from __future__ import annotations

import subprocess
import sys
import threading
import time
import logging
from typing import Any

log = logging.getLogger(__name__)

# gz.transport13 lives in the system Python path, not in the project venv.
_SYS_PKG = "/usr/lib/python3/dist-packages"
if _SYS_PKG not in sys.path:
    sys.path.insert(0, _SYS_PKG)


class FlickerController:
    """Toggle one or more gz-sim lights at configurable frequencies.

    Parameters
    ----------
    world_name:
        Gazebo world name (e.g. ``"indoor_office"``).  Used to build the
        ``/world/<name>/light_config`` service path.
    flicker_specs:
        List of dicts, one per light.  Required key: ``name``.
        Optional: ``period_s`` (default 1.0), ``on_diffuse`` (RGBA list,
        default [1,1,1,1]), ``off_diffuse`` (RGBA list, default [0,0,0,1]).
    lighting_preset:
        The resolved lighting preset dict (from ``config.yaml``) for the
        current scenario.  Used to auto-fill ``on_diffuse`` from the
        matching room_light entry when not explicitly provided.
    """

    def __init__(
        self,
        world_name: str,
        flicker_specs: list[dict[str, Any]],
        lighting_preset: dict | None = None,
    ) -> None:
        self._service = f"/world/{world_name}/light_config"
        self._specs = self._resolve_specs(flicker_specs, lighting_preset or {})
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None

        # Try to set up gz.transport13 service caller.
        # /world/<world>/light_config is a SERVICE (not a pub/sub topic) exposed
        # by the UserCommands system plugin in Gazebo Harmonic.
        # We call it via Node.request() with a Light request and Boolean response.
        self._gz_node = None
        self._Light = None
        self._Boolean = None
        try:
            from gz.transport13 import Node as _GzNode  # type: ignore[import]
            from gz.msgs10.light_pb2 import Light as _Light  # type: ignore[import]
            from gz.msgs10.boolean_pb2 import Boolean as _Boolean  # type: ignore[import]

            self._gz_node = _GzNode()
            self._Light = _Light
            self._Boolean = _Boolean
            log.debug("FlickerController: gz.transport13 ready, service=%s", self._service)
        except Exception as exc:
            log.warning(
                "FlickerController: gz.transport13 unavailable; "
                "falling back to gz-service CLI: %s",
                exc,
            )

    # ── Public API ──────────────────────────────────────────────────────────────

    def start(self) -> None:
        """Start the flicker daemon thread."""
        if not self._specs:
            return
        self._stop_event.clear()
        self._thread = threading.Thread(
            target=self._run, name="flicker_controller", daemon=True
        )
        self._thread.start()
        log.info("FlickerController started: %d lights", len(self._specs))

    def stop(self) -> None:
        """Signal the thread to stop and restore all lights to ON state."""
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=3.0)
        # Restore all lights to ON
        for spec in self._specs:
            self._call_service(spec["name"], spec["on_diffuse"])

    # ── Internal ────────────────────────────────────────────────────────────────

    def _run(self) -> None:
        # Brief startup delay: give Gazebo time to fully register the
        # light_config service before we start calling it.
        self._stop_event.wait(timeout=3.0)

        while not self._stop_event.is_set():
            for spec in self._specs:
                if self._stop_event.is_set():
                    break
                self._call_service(spec["name"], spec["off_diffuse"])

            half = min(s["period_s"] for s in self._specs) / 2.0
            self._stop_event.wait(timeout=half)

            for spec in self._specs:
                if self._stop_event.is_set():
                    break
                self._call_service(spec["name"], spec["on_diffuse"])

            self._stop_event.wait(timeout=half)

    def _call_service(self, name: str, diffuse: list[float]) -> None:
        """Call the light_config service — gz.transport13 preferred, CLI fallback."""
        r, g, b, a = diffuse

        # ── Fast path: gz.transport13 service call ────────────────────────────
        if self._gz_node is not None and self._Light is not None and self._Boolean is not None:
            try:
                req = self._Light()
                req.name = name
                req.diffuse.r = float(r)
                req.diffuse.g = float(g)
                req.diffuse.b = float(b)
                req.diffuse.a = float(a)
                req.specular.r = float(r * 0.3)
                req.specular.g = float(g * 0.3)
                req.specular.b = float(b * 0.3)
                req.specular.a = 1.0
                # Async service call (timeout=1000ms); result ignored —
                # fire-and-forget is sufficient for visual flickering.
                self._gz_node.request(
                    self._service, req, self._Light, self._Boolean, 1000
                )
                return
            except Exception as exc:
                log.debug("FlickerController gz.transport service error: %s", exc)

        # ── Slow fallback: gz service CLI ─────────────────────────────────────
        req_str = (
            f'name: "{name}" '
            f'diffuse {{r: {r} g: {g} b: {b} a: {a}}} '
            f'specular {{r: {r * 0.3:.3f} g: {g * 0.3:.3f} b: {b * 0.3:.3f} a: 1.0}}'
        )
        try:
            subprocess.run(
                [
                    "gz", "service",
                    "-s", self._service,
                    "--reqtype", "gz.msgs.Light",
                    "--reptype", "gz.msgs.Boolean",
                    "--timeout", "1000",
                    "--req", req_str,
                ],
                timeout=2.0,
                capture_output=True,
            )
        except Exception as exc:
            log.debug("FlickerController CLI service error: %s", exc)

    @staticmethod
    def _resolve_specs(
        flicker_specs: list[dict], lighting_preset: dict
    ) -> list[dict]:
        """Merge flicker YAML entries with on_diffuse values from the lighting preset."""
        room_light_map: dict[str, list] = {}
        for rl in lighting_preset.get("room_lights", []):
            room_light_map[rl["name"]] = rl.get("diffuse", [1.0, 1.0, 1.0, 1.0])

        resolved = []
        for spec in flicker_specs:
            name = spec["name"]
            on_diffuse = spec.get("on_diffuse") or room_light_map.get(name, [1.0, 1.0, 1.0, 1.0])
            off_diffuse = spec.get("off_diffuse", [0.0, 0.0, 0.0, 1.0])
            period_s = float(spec.get("period_s", 1.0))
            resolved.append({
                "name": name,
                "period_s": period_s,
                "on_diffuse": [float(v) for v in on_diffuse],
                "off_diffuse": [float(v) for v in off_diffuse],
            })
        return resolved
