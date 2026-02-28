"""World SDF generator — Step 1.5.

Takes a scenario YAML config, uses ObjectPlacer to get deterministic object
poses, injects them into the base template SDF, applies environment variations
(lighting), and writes a final runnable .sdf to disk.

Usage
-----
    from world_manager.world_generator import WorldGenerator

    gen = WorldGenerator()
    sdf_path = gen.generate(scenario_config)
    # sdf_path is an absolute Path to the written .sdf file
"""
from __future__ import annotations

import copy
import io
import subprocess
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any

from world_manager.template_loader import TemplateLoader
from world_manager.object_placer import ObjectPlacer, PlacedObject

# Project root resolved relative to this file: src/world_manager/ → src/ → repo root.
# Used as the default so WorldGenerator() works regardless of the caller's CWD.
_DEFAULT_ROOT = Path(__file__).resolve().parent.parent.parent


class WorldGenerator:
    """
    Takes a scenario config → generates a complete, launchable .sdf file.
    Combines: base template SDF + placed objects + environment variations.
    """

    #: Path to object model SDFs, relative to project root.
    MODELS_DIR = Path("worlds/models")

    def __init__(
        self,
        output_dir: Path = Path("/tmp/arst_worlds"),
        project_root: Path | None = None,
    ) -> None:
        """
        Parameters
        ----------
        output_dir:
            Directory where generated .sdf files are written.
        project_root:
            Absolute path to the project root (parent of ``worlds/``, ``src/``
            etc.).  Defaults to the current working directory.  Override when
            running from a directory other than the project root.
        """
        self._output_dir = Path(output_dir)
        self._root = Path(project_root) if project_root else _DEFAULT_ROOT
        self._loader = TemplateLoader(self._root / "worlds" / "templates")

    def generate(self, scenario_config: dict) -> Path:
        """Generate a world SDF from a scenario config dict.

        Parameters
        ----------
        scenario_config:
            Parsed scenario YAML (the full dict, not just the ``world:``
            section).  Must contain ``scenario.name``, ``scenario.random_seed``,
            and ``world.template``.

        Returns
        -------
        Path
            Absolute path to the written ``.sdf`` file.
        """
        self.label_map: dict[str, dict] = {}   # populated by _inject_objects
        world_cfg = scenario_config["world"]
        template_name = world_cfg["template"]
        seed = scenario_config["scenario"]["random_seed"]

        # ── Load template config + base SDF ───────────────────────────────────
        template_cfg = self._loader.load(template_name)
        sdf_path = self._loader.sdf_path(template_name)
        if not sdf_path.exists():
            raise FileNotFoundError(f"Template SDF not found: {sdf_path}")
        sdf_text = sdf_path.read_text()

        # ObjectPlacer reads the PGM via a relative path stored in config.yaml.
        # Resolve it to absolute so placer works regardless of CWD.
        _resolve_pgm_path(template_cfg, self._root)

        # ── Place objects ──────────────────────────────────────────────────────
        objects = world_cfg.get("objects", [])
        placer = ObjectPlacer(template_cfg)
        placements: list[PlacedObject] = placer.place(objects, seed) if objects else []

        # ── Build final SDF via XML manipulation ───────────────────────────────
        # NOTE: xml.etree.ElementTree strips XML comments from the template SDF.
        # The generated file is functional but comment-free — that is acceptable
        # for a machine-written output file.
        tree = ET.ElementTree(ET.fromstring(sdf_text))
        world_elem = tree.getroot().find("world")
        if world_elem is None:
            raise ValueError(f"No <world> element found in {sdf_path}")

        # Apply lighting variation
        lighting_preset = world_cfg.get("variations", {}).get("lighting", "normal")
        self._apply_lighting(
            world_elem,
            lighting_preset,
            template_cfg.get("lighting_presets", {}),
        )

        # Apply door states (pass spawn zone for connectivity guarantee)
        door_state = world_cfg.get("variations", {}).get("door_states", "open")
        spawn_zone = _find_spawn_zone(
            scenario_config.get("robots", []),
            template_cfg.get("placement_zones", []),
        )
        self._apply_door_states(
            world_elem,
            door_state,
            template_cfg.get("doors", []),
            seed,
            spawn_zone=spawn_zone,
        )

        # Inject placed objects before </world>
        self._inject_objects(world_elem, placements)

        # Embed robots — convert each URDF to SDF and add as <model>
        robots_cfg = scenario_config.get("robots", [])
        self._save_world_state(template_name, robots_cfg, template_cfg)
        self._embed_robots(world_elem, robots_cfg)

        # ── Write output ───────────────────────────────────────────────────────
        self._output_dir.mkdir(parents=True, exist_ok=True)
        scenario_name = scenario_config["scenario"]["name"]
        out_path = self._output_dir / f"{scenario_name}.sdf"
        _write_sdf(tree, out_path)

        return out_path.resolve()

    # ── Private helpers ────────────────────────────────────────────────────────

    def _apply_lighting(
        self,
        world_elem: ET.Element,
        preset: str,
        lighting_presets: dict,
    ) -> None:
        """Modify ambient/diffuse values in-place for the given lighting preset.

        Affects:
          - ``<world><scene><ambient>``   — overall scene ambient colour
          - ``<world><light><diffuse>``   — every directional/point/spot light

        If the preset defines ``room_lights``, those are injected as new
        ``<light>`` elements in the world.  This is used by localized presets
        that kill the global directional light (diffuse → 0) and replace it
        with per-room point/spot lights.
        """
        if preset not in lighting_presets:
            return  # unknown preset → leave template defaults unchanged

        cfg = lighting_presets[preset]
        ambient_val = cfg.get("ambient")
        diffuse_val = cfg.get("diffuse")

        if ambient_val:
            scene = world_elem.find("scene")
            if scene is not None:
                elem = scene.find("ambient")
                if elem is not None:
                    elem.text = _rgba_str(ambient_val)

        if diffuse_val:
            for light in world_elem.findall("light"):
                d = light.find("diffuse")
                if d is not None:
                    d.text = _rgba_str(diffuse_val)

        for rl in cfg.get("room_lights", []):
            world_elem.append(_build_room_light(rl))

    def _apply_door_states(
        self,
        world_elem: ET.Element,
        door_state: str,
        doors: list[dict],
        seed: int,
        spawn_zone: str = "",
    ) -> None:
        """Inject static door-panel models for closed/random door states.

        Guarantees the world graph remains connected from *spawn_zone* to all
        other zones — doors are reopened (in reverse close order) until full
        connectivity is restored.  This prevents unsolvable scenarios.

        Parameters
        ----------
        door_state:
            ``"open"`` — no panels injected.
            ``"closed"`` — all doors blocked (subject to connectivity guarantee).
            ``"random"`` — each door independently blocked (50 %) using *seed*,
            then connectivity-checked.
        doors:
            List of door dicts from ``config.yaml``
            (id, x, y, orientation, width, connects).
        seed:
            Scenario seed for reproducible ``"random"`` states.
        spawn_zone:
            The placement-zone id containing the robot spawn.  If non-empty and
            ``connects`` metadata is present on the doors, connectivity from
            this zone to all others is guaranteed.
        """
        if door_state == "open" or not doors:
            return

        import random as _random
        rng = _random.Random(seed + 999)  # offset avoids collision with object placement rng

        # Decide which doors to close (ordered list so we can reopen from the end)
        close_order: list[str] = []
        for door in doors:
            if door_state == "closed" or rng.choice([True, False]):
                close_order.append(door["id"])

        # ── Connectivity guarantee ─────────────────────────────────────────────
        # If every door in the template has 'connects' metadata AND we know the
        # spawn zone, restore connectivity by reopening doors as needed.
        if spawn_zone and all("connects" in d for d in doors):
            # Collect all zone ids
            all_zones: set[str] = set()
            for d in doors:
                all_zones.update(d["connects"])

            def _connected(closed_ids: set[str]) -> bool:
                """BFS from spawn_zone over open doors."""
                adj: dict[str, set[str]] = {z: set() for z in all_zones}
                for d in doors:
                    if d["id"] not in closed_ids:
                        a, b = d["connects"][0], d["connects"][1]
                        adj[a].add(b)
                        adj[b].add(a)
                visited: set[str] = set()
                queue = [spawn_zone]
                while queue:
                    node = queue.pop()
                    if node in visited:
                        continue
                    visited.add(node)
                    queue.extend(adj.get(node, []))
                return visited == all_zones

            closed_set = set(close_order)
            # Reopen doors from the end of close_order until graph is connected
            for door_id in reversed(close_order):
                if _connected(closed_set):
                    break
                closed_set.discard(door_id)
            close_order = [d for d in close_order if d in closed_set]
        # ──────────────────────────────────────────────────────────────────────

        PANEL_H = 2.5   # floor-to-ceiling panel height (m)
        PANEL_T = 0.15  # panel thickness (m)

        closed_ids = set(close_order)
        for door in doors:
            if door["id"] not in closed_ids:
                continue

            door_id = door["id"]
            cx, cy = float(door["x"]), float(door["y"])
            width = float(door.get("width", 1.0))
            orientation = door.get("orientation", "ew")

            # NS wall: panel thin in x, wide in y.  EW wall: wide in x, thin in y.
            if orientation == "ns":
                sx, sy, sz = PANEL_T, width, PANEL_H
            else:
                sx, sy, sz = width, PANEL_T, PANEL_H

            panel = ET.fromstring(
                f'<model name="door_panel_{door_id}">'
                f'<static>true</static>'
                f'<pose>{cx} {cy} {PANEL_H / 2:.4f} 0 0 0</pose>'
                f'<link name="link">'
                f'<collision name="collision">'
                f'<geometry><box><size>{sx} {sy} {sz}</size></box></geometry>'
                f'</collision>'
                f'<visual name="visual">'
                f'<geometry><box><size>{sx} {sy} {sz}</size></box></geometry>'
                f'<material>'
                f'<ambient>0.55 0.38 0.18 1</ambient>'
                f'<diffuse>0.65 0.45 0.22 1</diffuse>'
                f'</material>'
                f'</visual>'
                f'</link>'
                f'</model>'
            )
            world_elem.append(panel)

    def _inject_objects(
        self,
        world_elem: ET.Element,
        placements: list[PlacedObject],
    ) -> None:
        """Append one ``<model>`` element per placed object to *world_elem*.

        Assigns a unique integer label (1..N, placement order) to each
        instance by overriding the gz-sim-label-system plugin element.
        Populates ``self.label_map`` with the label → {type, instance} mapping
        so the metrics layer can resolve detected labels back to object types.
        """
        counters: dict[str, int] = {}
        label_counter = 1
        self.label_map: dict[str, dict] = {}
        for obj in placements:
            mt = obj.model_type
            idx = counters.get(mt, 0)
            counters[mt] = idx + 1
            self.label_map[str(label_counter)] = {
                "type": mt, "instance": idx,
                "x": round(obj.x, 3), "y": round(obj.y, 3),
            }
            world_elem.append(self._load_model_element(mt, idx, obj, label_counter))
            label_counter += 1

    def _embed_robots(
        self,
        world_elem: ET.Element,
        robots_cfg: list[dict],
    ) -> None:
        """Convert each robot URDF to SDF via ``gz sdf -p`` and embed in world.

        Robots are embedded at world-generation time (not dynamically spawned)
        so that gz-sim-contact-system's EachNew<ContactSensor> fires at startup.
        This is required because gz-sim 8.10.0 has a bug where EachNew never
        fires for models added via the UserCommands create service.
        """
        for robot in robots_cfg:
            platform = robot["platform"]
            name = robot.get("name", f"{platform}_0")
            pose_cfg = robot.get("spawn_pose", {})

            urdf_path = self._root / "robots" / platform / "urdf" / f"{platform}.urdf"
            if not urdf_path.exists():
                raise FileNotFoundError(
                    f"Robot URDF not found: {urdf_path}\n"
                    f"Expected layout: robots/{platform}/urdf/{platform}.urdf"
                )

            urdf_text = urdf_path.read_text().replace("ROBOT_NAME", name)

            # Write URDF to temp file, convert to SDF via gz sdf -p
            with tempfile.NamedTemporaryFile(
                suffix=".urdf", mode="w", delete=False
            ) as tmp:
                tmp.write(urdf_text)
                tmp_path = Path(tmp.name)

            try:
                result = subprocess.run(
                    ["gz", "sdf", "-p", str(tmp_path)],
                    capture_output=True,
                    text=True,
                    timeout=30,
                )
            finally:
                tmp_path.unlink(missing_ok=True)

            if result.returncode != 0:
                raise RuntimeError(
                    f"gz sdf -p failed for robot '{platform}':\n{result.stderr}"
                )

            sdf_root = ET.fromstring(result.stdout)
            model_elem = sdf_root.find("model")
            if model_elem is None:
                raise ValueError(
                    f"No <model> element in gz sdf -p output for '{platform}'"
                )

            model_elem = copy.deepcopy(model_elem)
            model_elem.set("name", name)

            x   = pose_cfg.get("x",   1.0)
            y   = pose_cfg.get("y",   1.0)
            z   = pose_cfg.get("z",   0.0)
            yaw = pose_cfg.get("yaw", 0.0)
            pose_elem = ET.Element("pose")
            pose_elem.text = f"{x:.4f} {y:.4f} {z:.4f} 0 0 {yaw:.4f}"
            model_elem.insert(0, pose_elem)

            world_elem.append(model_elem)

    def _save_world_state(
        self, template_name: str, robots_cfg: list, template_cfg: dict
    ) -> None:
        """Write /tmp/arst_worlds/world_state.json for agent navigation tools."""
        import json
        spawn: dict = {}
        if robots_cfg:
            sp = robots_cfg[0].get("spawn_pose", {})
            spawn = {
                "x": float(sp.get("x", 1.0)),
                "y": float(sp.get("y", 1.0)),
                "yaw": float(sp.get("yaw", 0.0)),
            }
        pgm = self._root / "worlds" / "templates" / template_name / "ground_truth_map.pgm"
        state = {
            "label_map": self.label_map,
            "map_pgm": str(pgm),
            "map_resolution": 0.5,
            "spawn_pose": spawn,
            "obstacles": template_cfg.get("obstacles", []),
        }
        self._output_dir.mkdir(parents=True, exist_ok=True)
        out = self._output_dir / "world_state.json"
        out.write_text(json.dumps(state, indent=2))
        # Clear live detections from any previous run
        live = self._output_dir / "detections_live.json"
        live.write_text(json.dumps({"found": []}))

    def _load_model_element(
        self, model_type: str, idx: int, obj: PlacedObject, instance_label: int = 0
    ) -> ET.Element:
        """Parse a model SDF and return a renamed, repositioned ``<model>``.

        If *instance_label* > 0, overrides the gz-sim-label-system ``<label>``
        element so each placed instance has a unique integer label.  This
        enables the bounding-box camera to distinguish individual instances
        (e.g. fire_extinguisher #1 vs #2) rather than just object types.
        """
        model_path = self._root / self.MODELS_DIR / model_type / "model.sdf"
        if not model_path.exists():
            raise FileNotFoundError(
                f"Model SDF not found: {model_path}\n"
                f"Expected at worlds/models/{model_type}/model.sdf"
            )

        root = ET.parse(model_path).getroot()
        model = root.find("model")
        if model is None:
            raise ValueError(f"No <model> element found in {model_path}")

        model = copy.deepcopy(model)

        # Unique name so multiple instances of the same type don't conflict
        model.set("name", f"{model_type}_{idx}")

        # Override per-instance label for unique detection tracking
        if instance_label > 0:
            for plugin in model.iter("plugin"):
                if plugin.get("filename") == "gz-sim-label-system":
                    lbl = plugin.find("label")
                    if lbl is not None:
                        lbl.text = str(instance_label)
                    break

        # Insert pose at position 0 (before <static> etc.) so it takes effect
        pose = ET.Element("pose")
        pose.text = f"{obj.x:.4f} {obj.y:.4f} {obj.z:.4f} 0 0 {obj.yaw:.4f}"
        model.insert(0, pose)

        return model


# ── Module-level helpers ──────────────────────────────────────────────────────


def _resolve_pgm_path(template_cfg: dict, project_root: Path) -> None:
    """Rewrite the GT map PGM path to absolute so ObjectPlacer finds it.

    ``config.yaml`` stores the path relative to the project root
    (e.g. ``worlds/templates/indoor_office/ground_truth_map.pgm``).
    ObjectPlacer resolves it with ``Path(pgm)``, which only works when CWD is
    the project root.  This helper makes it absolute so CWD is irrelevant.
    """
    gt = template_cfg.get("ground_truth_map", {})
    pgm = gt.get("pgm")
    if pgm and not Path(pgm).is_absolute():
        gt["pgm"] = str(project_root / pgm)


def _build_room_light(rl: dict) -> ET.Element:
    """Build a ``<light>`` element from a room_lights config dict.

    Expected keys: name, type (point/spot/directional), pose ([x,y,z,r,p,y]),
    diffuse, specular, range, constant, linear, quadratic.
    """
    light_type = rl.get("type", "point")
    name = rl["name"]
    pose = rl.get("pose", [0, 0, 3, 0, 0, 0])
    pose_str = " ".join(str(float(v)) for v in pose)
    diffuse = _rgba_str(rl.get("diffuse", [1, 1, 1, 1]))
    specular = _rgba_str(rl.get("specular", [0.3, 0.3, 0.3, 1]))
    att_range = float(rl.get("range", 10.0))
    constant = float(rl.get("constant", 0.5))
    linear = float(rl.get("linear", 0.05))
    quadratic = float(rl.get("quadratic", 0.005))
    return ET.fromstring(
        f'<light type="{light_type}" name="{name}">'
        f'<cast_shadows>false</cast_shadows>'
        f'<pose>{pose_str}</pose>'
        f'<diffuse>{diffuse}</diffuse>'
        f'<specular>{specular}</specular>'
        f'<attenuation>'
        f'<range>{att_range}</range>'
        f'<constant>{constant}</constant>'
        f'<linear>{linear}</linear>'
        f'<quadratic>{quadratic}</quadratic>'
        f'</attenuation>'
        f'</light>'
    )


def _find_spawn_zone(robots_cfg: list[dict], zones: list[dict]) -> str:
    """Return the placement-zone id that contains the first robot's spawn pose.

    Falls back to ``""`` (connectivity check disabled) if no match is found.
    """
    if not robots_cfg or not zones:
        return ""
    sp = robots_cfg[0].get("spawn_pose", {})
    x, y = float(sp.get("x", 0.0)), float(sp.get("y", 0.0))
    for zone in zones:
        if zone["x_min"] <= x <= zone["x_max"] and zone["y_min"] <= y <= zone["y_max"]:
            return zone["id"]
    return ""


def _rgba_str(rgba: list) -> str:
    """Convert an RGBA list to the space-separated string Gazebo expects."""
    return " ".join(str(float(v)) for v in rgba)


def _write_sdf(tree: ET.ElementTree, path: Path) -> None:
    """Serialize *tree* to *path* with an XML declaration and a notice."""
    buf = io.StringIO()
    tree.write(buf, encoding="unicode", xml_declaration=True)
    content = buf.getvalue()

    # Prepend a human-readable notice after the XML declaration
    notice = "<!-- Auto-generated by WorldGenerator — do not edit manually. -->\n"
    if content.startswith("<?xml"):
        nl = content.index("\n") + 1
        content = content[:nl] + notice + content[nl:]
    else:
        content = notice + content

    path.write_text(content)
