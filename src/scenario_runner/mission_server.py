"""Mission HTTP server — exposes scenario mission description during a run.

GET http://localhost:7400/mission → JSON (structured) + human-readable description field.

The server stays alive until stop() is called (after the scorecard is printed),
so agents can still query the final status after the scenario ends.
"""
from __future__ import annotations

import json
import logging
import socketserver
import threading
from typing import Any

from http.server import BaseHTTPRequestHandler

log = logging.getLogger(__name__)

MISSION_PORT = 7400
_W = 60  # terminal box width, matches SCORECARD_WIDTH in reporter.py


# ── Data builders ──────────────────────────────────────────────────────────────

def build_mission_data(cfg: dict) -> dict[str, Any]:
    """Build the mission data dict from a loaded scenario config.

    Each target entry carries:
      - type          : object type string
      - count_exact   : True when the exact count is disclosed
      - count         : integer count (only present when count_exact=True)
      - min_count     : minimum guaranteed count (only present when count_exact=False)

    Set ``count_revealed: false`` on an object entry in the scenario YAML to
    withhold the exact count from the mission description.
    """
    scenario = cfg["scenario"]
    objects = cfg.get("world", {}).get("objects", [])

    targets: list[dict[str, Any]] = []
    for obj in objects:
        revealed = obj.get("count_revealed", True)
        if revealed:
            targets.append({
                "type": obj["type"],
                "count": obj.get("count", 1),
                "count_exact": True,
            })
        else:
            targets.append({
                "type": obj["type"],
                "count_exact": False,
                "min_count": 1,
            })

    time_limit = int(scenario.get("timeout_seconds", 600))

    return {
        "scenario": scenario.get("name", "unknown"),
        "goal": (
            "Explore the environment and locate all required target objects. "
            "Report detections before the time limit expires. "
            "Thorough exploration of the full environment earns bonus points."
        ),
        "time_limit_seconds": time_limit,
        "targets": targets,
        "status": "running",
        "description": _build_description(targets, time_limit, "running"),
    }


def _build_description(targets: list[dict], time_limit: int, status: str) -> str:
    parts = []
    for t in targets:
        label = t["type"].replace("_", " ")
        if t["count_exact"]:
            parts.append(f"exactly {t['count']} {label}(s)")
        else:
            parts.append(f"at least {t['min_count']} {label}(s) (exact count unknown)")
    obj_str = ", ".join(parts) if parts else "unspecified targets"
    return (
        f"Explore the environment and locate all target objects within {time_limit}s. "
        f"Targets: {obj_str}. Status: {status}."
    )


# ── Terminal render ────────────────────────────────────────────────────────────

def render_mission_brief(data: dict[str, Any]) -> str:
    """Return an ASCII box summary of the mission, styled like the scorecard."""
    w = _W
    lines = []
    lines.append("╔" + "═" * (w - 2) + "╗")
    lines.append(f"║  MISSION BRIEF{' ' * (w - 17)}║")
    lines.append("╠" + "═" * (w - 2) + "╣")
    scenario = data.get("scenario", "unknown")
    lines.append(f"║  Scenario : {scenario:<{w - 15}}║")
    time_limit = data.get("time_limit_seconds", "?")
    status = data.get("status", "?")
    lines.append(f"║  Time limit: {time_limit}s   Status: {status:<{w - 27}}║")
    lines.append("║" + " " * (w - 2) + "║")
    lines.append(f"║  Targets:{' ' * (w - 11)}║")
    for t in data.get("targets", []):
        label = t["type"].replace("_", " ")
        if t["count_exact"]:
            detail = f"×{t['count']}"
        else:
            detail = f"≥{t['min_count']}  (exact count unknown)"
        row = f"║    • {label:<22} {detail}"
        lines.append(row + " " * (w - 2 - len(row) + 2) + "║")
    lines.append("║" + " " * (w - 2) + "║")
    endpoint = f"http://localhost:{MISSION_PORT}/mission"
    ep_line = f"║  Endpoint: {endpoint}"
    lines.append(ep_line + " " * (w - 2 - len(ep_line) + 2) + "║")
    lines.append("╚" + "═" * (w - 2) + "╝")
    return "\n".join(lines)


# ── HTTP server ────────────────────────────────────────────────────────────────

class _ReusableTCPServer(socketserver.TCPServer):
    allow_reuse_address = True


class MissionServer:
    """Lightweight HTTP server that exposes mission info for the duration of a run.

    Lifecycle::

        server = MissionServer(build_mission_data(cfg))
        server.start()
        # ... scenario runs ...
        server.update_status("completed", outcome="SUCCESS")
        # ... print scorecard ...
        server.stop()
    """

    def __init__(self, mission_data: dict[str, Any], port: int = MISSION_PORT) -> None:
        self._port = port
        self._data: dict[str, Any] = dict(mission_data)
        self._lock = threading.Lock()
        self._server: _ReusableTCPServer | None = None
        self._thread: threading.Thread | None = None

    @property
    def port(self) -> int:
        return self._port

    def update_status(self, status: str, outcome: str | None = None) -> None:
        """Update the status (and optionally outcome) visible via /mission."""
        with self._lock:
            self._data["status"] = status
            if outcome is not None:
                self._data["outcome"] = outcome
            self._data["description"] = _build_description(
                self._data["targets"],
                self._data["time_limit_seconds"],
                status,
            )

    def snapshot(self) -> dict[str, Any]:
        """Return a thread-safe copy of the current mission data."""
        with self._lock:
            return dict(self._data)

    def _make_handler(self):
        server = self

        class _Handler(BaseHTTPRequestHandler):
            def do_GET(self):
                if self.path in ("/mission", "/mission/"):
                    with server._lock:
                        body = json.dumps(server._data, indent=2).encode()
                    self.send_response(200)
                    self.send_header("Content-Type", "application/json")
                    self.send_header("Content-Length", str(len(body)))
                    self.end_headers()
                    self.wfile.write(body)
                else:
                    self.send_response(404)
                    self.end_headers()

            def log_message(self, fmt, *args):  # silence default access log
                pass

        return _Handler

    def start(self) -> None:
        self._server = _ReusableTCPServer(("", self._port), self._make_handler())
        self._thread = threading.Thread(target=self._server.serve_forever, daemon=True)
        self._thread.start()
        log.info("Mission server on http://localhost:%d/mission", self._port)

    def stop(self) -> None:
        if self._server is not None:
            self._server.shutdown()
            self._server = None
