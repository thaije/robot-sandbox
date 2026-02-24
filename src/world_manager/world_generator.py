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

        # Inject placed objects before </world>
        self._inject_objects(world_elem, placements)

        # Embed robots — convert each URDF to SDF and add as <model>
        robots_cfg = scenario_config.get("robots", [])
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

    def _inject_objects(
        self,
        world_elem: ET.Element,
        placements: list[PlacedObject],
    ) -> None:
        """Append one ``<model>`` element per placed object to *world_elem*."""
        counters: dict[str, int] = {}
        for obj in placements:
            mt = obj.model_type
            idx = counters.get(mt, 0)
            counters[mt] = idx + 1
            world_elem.append(self._load_model_element(mt, idx, obj))

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

    def _load_model_element(
        self, model_type: str, idx: int, obj: PlacedObject
    ) -> ET.Element:
        """Parse a model SDF and return a renamed, repositioned ``<model>``."""
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

        # Insert pose at position 0 (before <static> etc.) so it takes effect
        pose = ET.Element("pose")
        pose.text = f"{obj.x:.4f} {obj.y:.4f} 0 0 0 {obj.yaw:.4f}"
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
