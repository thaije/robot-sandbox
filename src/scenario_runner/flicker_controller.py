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
            self._call_service(spec, is_off=False)

    # ── Internal ────────────────────────────────────────────────────────────────

    def _run(self) -> None:
        # Brief startup delay: give Gazebo time to fully register services.
        self._stop_event.wait(timeout=3.0)
        if self._stop_event.is_set():
            return

        # gz-sim 8.10.0 bug: the light_config service cannot find lights that
        # were baked into the world SDF at load time.  The workaround is to
        # (re-)create each flickerable light via EntityFactory so it gets a
        # proper entity registration that the service handler CAN find.
        self._recreate_lights()

        while not self._stop_event.is_set():
            for spec in self._specs:
                if self._stop_event.is_set():
                    break
                self._call_service(spec, is_off=True)

            half = min(s["period_s"] for s in self._specs) / 2.0
            self._stop_event.wait(timeout=half)

            for spec in self._specs:
                if self._stop_event.is_set():
                    break
                self._call_service(spec, is_off=False)

            self._stop_event.wait(timeout=half)

    def _recreate_lights(self) -> None:
        """Re-create each flicker light via EntityFactory so light_config service can find it.

        gz-sim 8.10.0 bug: the light_config service only works on lights that
        were created dynamically (via the /world/<world>/create service), not on
        lights baked into the world SDF at load time.  Calling this once per
        scenario run registers the light entities so the toggle loop works.
        """
        if self._gz_node is None:
            return
        try:
            from gz.msgs10.entity_factory_pb2 import EntityFactory as _EF  # type: ignore[import]
            from gz.msgs10.boolean_pb2 import Boolean as _Boolean  # type: ignore[import]
        except Exception as exc:
            log.warning("FlickerController: cannot import EntityFactory: %s", exc)
            return

        create_svc = self._service.replace("light_config", "create")
        for spec in self._specs:
            rl = spec.get("light_cfg", {})
            if not rl:
                log.warning("FlickerController: no light_cfg for '%s', skip recreate", spec["name"])
                continue
            name = spec["name"]
            light_type = rl.get("type", "point")
            pose = rl.get("pose", [0, 0, 3, 0, 0, 0])
            pose_str = " ".join(str(float(v)) for v in pose)
            diffuse = spec["on_diffuse"]
            specular = rl.get("specular", [d * 0.3 for d in diffuse])
            att_range = float(rl.get("range", 10.0))
            constant = float(rl.get("constant", 0.5))
            linear = float(rl.get("linear", 0.05))
            quadratic = float(rl.get("quadratic", 0.005))

            sdf = (
                f'<sdf version="1.9">'
                f'<light type="{light_type}" name="{name}">'
                f'<cast_shadows>false</cast_shadows>'
                f'<pose>{pose_str}</pose>'
                f'<diffuse>{diffuse[0]} {diffuse[1]} {diffuse[2]} {diffuse[3]}</diffuse>'
                f'<specular>{specular[0]} {specular[1]} {specular[2]} 1.0</specular>'
                f'<attenuation>'
                f'<range>{att_range}</range>'
                f'<constant>{constant}</constant>'
                f'<linear>{linear}</linear>'
                f'<quadratic>{quadratic}</quadratic>'
                f'</attenuation>'
                f'</light>'
                f'</sdf>'
            )
            try:
                factory = _EF()
                factory.sdf = sdf
                result, rep = self._gz_node.request(
                    create_svc, factory, _EF, _Boolean, 3000
                )
                log.debug(
                    "FlickerController: (re)created light '%s': result=%s rep=%s",
                    name, result, rep.data,
                )
            except Exception as exc:
                log.warning("FlickerController: failed to recreate light '%s': %s", name, exc)

    def _call_service(self, spec: dict, is_off: bool) -> None:
        """Call the light_config service — gz.transport13 preferred, CLI fallback."""
        name = spec["name"]
        diffuse = spec["off_diffuse"] if is_off else spec["on_diffuse"]
        r, g, b, a = diffuse

        rl = spec.get("light_cfg", {})
        att_range = float(rl.get("range", 10.0))
        constant = float(rl.get("constant", 0.5))
        linear = float(rl.get("linear", 0.05))
        quadratic = float(rl.get("quadratic", 0.005))
        _type_map = {"point": 0, "spot": 1, "directional": 2}
        light_type_int = _type_map.get(rl.get("type", "point"), 0)

        # ── Fast path: gz.transport13 service call ────────────────────────────
        if self._gz_node is not None and self._Light is not None and self._Boolean is not None:
            try:
                # see light message here: https://github.com/gazebosim/gz-msgs/blob/main/proto/gz/msgs/light.proto
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
                req.type = light_type_int
                req.cast_shadows = False
                req.attenuation_constant = constant
                req.range = att_range
                req.attenuation_linear = linear
                req.attenuation_quadratic = quadratic
                req.is_light_off = is_off
                req.intensity = 1.0
                req.visualize_visual = True
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
            f'specular {{r: {r * 0.3:.3f} g: {g * 0.3:.3f} b: {b * 0.3:.3f} a: 1.0}} '
            f'range: {att_range} '
            f'attenuation_constant: {constant} '
            f'attenuation_linear: {linear} '
            f'attenuation_quadratic: {quadratic} '
            f'is_light_off: {"true" if is_off else "false"} '
            f'intensity: 1.0'
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
        """Merge flicker YAML entries with on_diffuse values and full light config from the preset."""
        room_light_map: dict[str, dict] = {}
        for rl in lighting_preset.get("room_lights", []):
            room_light_map[rl["name"]] = rl

        resolved = []
        for spec in flicker_specs:
            name = spec["name"]
            rl = room_light_map.get(name, {})
            on_diffuse = spec.get("on_diffuse") or rl.get("diffuse", [1.0, 1.0, 1.0, 1.0])
            off_diffuse = spec.get("off_diffuse", [0.0, 0.0, 0.0, 1.0])
            period_s = float(spec.get("period_s", 1.0))
            resolved.append({
                "name": name,
                "period_s": period_s,
                "on_diffuse": [float(v) for v in on_diffuse],
                "off_diffuse": [float(v) for v in off_diffuse],
                # Full physical config needed to (re-)create the light entity at runtime.
                "light_cfg": rl,
            })
        return resolved
