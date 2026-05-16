"""Generate COLLADA (.dae) meshes with tiled UVs for basement surfaces.

Creates box meshes where UVs tile at a real-world scale.  Each face is
subdivided into a grid so every quad spans exactly one texture tile (UV 0→1),
preventing GPU mipmapping from blurring the texture.

Usage:
    python3.12 scripts/generate_basement_meshes.py

Output: worlds/models/basement_surfaces/meshes/*.dae
"""
from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from pathlib import Path

OUT = Path(__file__).resolve().parent.parent / "worlds" / "models" / "basement_surfaces" / "meshes"

BRICK_TILE = 0.5
SAND_TILE = 1.0
WOOD_TILE_U = 0.3
WOOD_TILE_V = 0.5
METAL_TILE = 0.3


def _box_mesh(
    name: str,
    sx: float, sy: float, sz: float,
    tile_u: float, tile_v: float,
    tile_side: float = 0.5,
) -> str:
    hx, hy, hz = sx / 2, sy / 2, sz / 2

    faces_def = [
        {"verts": [4, 5, 6, 7], "normal": (0, 0, 1),
         "u_size": sx, "v_size": sy, "u_tile": tile_u, "v_tile": tile_v},
        {"verts": [1, 0, 3, 2], "normal": (0, 0, -1),
         "u_size": sx, "v_size": sy, "u_tile": tile_u, "v_tile": tile_v},
        {"verts": [7, 6, 2, 3], "normal": (0, 1, 0),
         "u_size": sx, "v_size": sz, "u_tile": tile_u, "v_tile": tile_side},
        {"verts": [0, 1, 5, 4], "normal": (0, -1, 0),
         "u_size": sx, "v_size": sz, "u_tile": tile_u, "v_tile": tile_side},
        {"verts": [5, 1, 2, 6], "normal": (1, 0, 0),
         "u_size": sy, "v_size": sz, "u_tile": tile_v, "v_tile": tile_side},
        {"verts": [0, 4, 7, 3], "normal": (-1, 0, 0),
         "u_size": sy, "v_size": sz, "u_tile": tile_v, "v_tile": tile_side},
    ]

    corners = [
        (-hx, -hy, -hz), (hx, -hy, -hz), (hx, hy, -hz), (-hx, hy, -hz),
        (-hx, -hy, hz), (hx, -hy, hz), (hx, hy, hz), (-hx, hy, hz),
    ]

    positions = []
    normals = []
    uvs = []
    triangles = []

    def _lerp(a, b, t):
        return (a[0] + (b[0] - a[0]) * t,
                a[1] + (b[1] - a[1]) * t,
                a[2] + (b[2] - a[2]) * t)

    for face in faces_def:
        vs = face["verts"]
        normal = face["normal"]
        tile_u = face["u_tile"]
        tile_v = face["v_tile"]

        c0 = corners[vs[0]]
        c1 = corners[vs[1]]
        c2 = corners[vs[2]]
        c3 = corners[vs[3]]

        u_size = face["u_size"]
        v_size = face["v_size"]

        n_u_full = int(u_size // tile_u)
        rem_u = u_size - n_u_full * tile_u
        n_v_full = int(v_size // tile_v)
        rem_v = v_size - n_v_full * tile_v

        nu = n_u_full + (1 if rem_u > 0.001 else 0)
        nv = n_v_full + (1 if rem_v > 0.001 else 0)

        for j in range(nv):
            for i in range(nu):
                is_partial_u = (rem_u > 0.001 and i == n_u_full)
                is_partial_v = (rem_v > 0.001 and j == n_v_full)
                quad_u_size = rem_u if is_partial_u else tile_u
                quad_v_size = rem_v if is_partial_v else tile_v

                u_start = i * tile_u
                v_start = j * tile_v

                world_u0 = u_start / u_size
                world_u1 = (u_start + quad_u_size) / u_size
                world_v0 = v_start / v_size
                world_v1 = (v_start + quad_v_size) / v_size

                bl = _lerp(_lerp(c0, c1, world_u0), _lerp(c3, c2, world_u0), world_v0)
                br = _lerp(_lerp(c0, c1, world_u1), _lerp(c3, c2, world_u1), world_v0)
                tr = _lerp(_lerp(c0, c1, world_u1), _lerp(c3, c2, world_u1), world_v1)
                tl = _lerp(_lerp(c0, c1, world_u0), _lerp(c3, c2, world_u0), world_v1)

                base = len(positions) // 3
                for p in (bl, br, tr, tl):
                    positions.extend(p)
                    normals.extend(normal)

                uv_u1_partial = quad_u_size / tile_u
                uv_v1_partial = quad_v_size / tile_v
                uvs.extend([0, 0, uv_u1_partial, 0,
                            uv_u1_partial, uv_v1_partial, 0, uv_v1_partial])

                triangles.append([base, base + 1, base + 2])
                triangles.append([base, base + 2, base + 3])

    return _build_dae(name, positions, normals, uvs, triangles)


def _cylinder_mesh(name: str, radius: float, height: float,
                    u_tile: float, v_tile: float) -> str:
    """Generate a cylinder mesh with subdivided tiled UVs.

    Each sub-quad has UVs in [0,1] range so GPU mipmapping works correctly.
    The cylinder axis is along Z, centred at origin.
    """
    hz = height / 2
    circumference = 2 * math.pi * radius
    n_u = max(24, round(circumference / u_tile))
    n_v = max(1, round(height / v_tile))

    positions = []
    normals = []
    uvs = []
    triangles = []

    uv_span_u = circumference / u_tile
    uv_span_v = height / v_tile

    # Side: grid of n_u columns × n_v rows, UVs proportional to world distance
    for j in range(n_v):
        v0_frac = j / n_v
        v1_frac = (j + 1) / n_v
        z0 = -hz + v0_frac * height
        z1 = -hz + v1_frac * height
        uv_v0 = v0_frac * uv_span_v
        uv_v1 = v1_frac * uv_span_v
        for i in range(n_u):
            u0_frac = i / n_u
            u1_frac = (i + 1) / n_u
            theta0 = u0_frac * 2 * math.pi
            theta1 = u1_frac * 2 * math.pi
            c0, s0 = math.cos(theta0), math.sin(theta0)
            c1, s1 = math.cos(theta1), math.sin(theta1)
            uv_u0 = u0_frac * uv_span_u
            uv_u1 = u1_frac * uv_span_u

            base = len(positions) // 3
            for (ct, st, z, uv_u, uv_v) in [
                (c0, s0, z0, uv_u0, uv_v0), (c1, s1, z0, uv_u1, uv_v0),
                (c1, s1, z1, uv_u1, uv_v1), (c0, s0, z1, uv_u0, uv_v1),
            ]:
                positions.extend([radius * ct, radius * st, z])
                normals.extend([ct, st, 0.0])
                uvs.extend([uv_u, uv_v])

            triangles.append([base, base + 1, base + 2])
            triangles.append([base, base + 2, base + 3])

    cap_scale_u = min(1.0, radius / u_tile)
    cap_scale_v = min(1.0, radius / v_tile)

    # Top cap — fan from centre
    center_top = len(positions) // 3
    positions.extend([0.0, 0.0, hz])
    normals.extend([0.0, 0.0, 1.0])
    uvs.extend([0.5, 0.5])
    for i in range(n_u):
        theta = i / n_u * 2 * math.pi
        ct, st = math.cos(theta), math.sin(theta)
        positions.extend([radius * ct, radius * st, hz])
        normals.extend([0.0, 0.0, 1.0])
        uvs.extend([0.5 + 0.5 * ct * cap_scale_u,
                     0.5 + 0.5 * st * cap_scale_v])
    for i in range(n_u):
        i_next = (i + 1) % n_u
        triangles.append([center_top, center_top + 1 + i, center_top + 1 + i_next])

    # Bottom cap — fan from centre (reversed winding)
    center_bot = len(positions) // 3
    positions.extend([0.0, 0.0, -hz])
    normals.extend([0.0, 0.0, -1.0])
    uvs.extend([0.5, 0.5])
    for i in range(n_u):
        theta = i / n_u * 2 * math.pi
        ct, st = math.cos(theta), math.sin(theta)
        positions.extend([radius * ct, radius * st, -hz])
        normals.extend([0.0, 0.0, -1.0])
        uvs.extend([0.5 - 0.5 * ct * cap_scale_u,
                     0.5 + 0.5 * st * cap_scale_v])
    for i in range(n_u):
        i_next = (i + 1) % n_u
        triangles.append([center_bot, center_bot + 1 + i_next, center_bot + 1 + i])

    return _build_dae(name, positions, normals, uvs, triangles)


def _build_dae(name, positions, normals, uvs, triangles) -> str:
    ns = "http://www.collada.org/2005/11/COLLADASchema"
    ET.register_namespace("", ns)

    root = ET.Element(f"{{{ns}}}COLLADA", version="1.4.1")
    asset = ET.SubElement(root, f"{{{ns}}}asset")
    ET.SubElement(asset, f"{{{ns}}}created").text = "2026-01-01T00:00:00Z"
    ET.SubElement(asset, f"{{{ns}}}modified").text = "2026-01-01T00:00:00Z"
    ET.SubElement(asset, f"{{{ns}}}unit", name="meter", meter="1.0")
    ET.SubElement(asset, f"{{{ns}}}up_axis").text = "Z_UP"

    lib_geom = ET.SubElement(root, f"{{{ns}}}library_geometries")
    geom = ET.SubElement(lib_geom, f"{{{ns}}}geometry", id=f"{name}-geom")
    mesh_el = ET.SubElement(geom, f"{{{ns}}}mesh")

    pos_str = " ".join(f"{v:.6f}" for v in positions)
    norm_str = " ".join(f"{v:.6f}" for v in normals)
    uv_str = " ".join(f"{v:.6f}" for v in uvs)

    src_pos = ET.SubElement(mesh_el, f"{{{ns}}}source", id=f"{name}-positions")
    ET.SubElement(src_pos, f"{{{ns}}}float_array",
                   id=f"{name}-positions-array",
                   count=str(len(positions))).text = pos_str
    tech_pos = ET.SubElement(src_pos, f"{{{ns}}}technique_common")
    acc_pos = ET.SubElement(tech_pos, f"{{{ns}}}accessor",
                             source=f"#{name}-positions-array",
                             count=str(len(positions) // 3), stride="3")

    src_norm = ET.SubElement(mesh_el, f"{{{ns}}}source", id=f"{name}-normals")
    ET.SubElement(src_norm, f"{{{ns}}}float_array",
                   id=f"{name}-normals-array",
                   count=str(len(normals))).text = norm_str
    tech_norm = ET.SubElement(src_norm, f"{{{ns}}}technique_common")
    ET.SubElement(tech_norm, f"{{{ns}}}accessor",
                   source=f"#{name}-normals-array",
                   count=str(len(normals) // 3), stride="3")

    src_uv = ET.SubElement(mesh_el, f"{{{ns}}}source", id=f"{name}-uv")
    ET.SubElement(src_uv, f"{{{ns}}}float_array",
                   id=f"{name}-uv-array",
                   count=str(len(uvs))).text = uv_str
    tech_uv = ET.SubElement(src_uv, f"{{{ns}}}technique_common")
    ET.SubElement(tech_uv, f"{{{ns}}}accessor",
                   source=f"#{name}-uv-array",
                   count=str(len(uvs) // 2), stride="2")

    verts = ET.SubElement(mesh_el, f"{{{ns}}}vertices", id=f"{name}-verts")
    ET.SubElement(verts, f"{{{ns}}}input", semantic="POSITION",
                   source=f"#{name}-positions")
    ET.SubElement(verts, f"{{{ns}}}input", semantic="NORMAL",
                   source=f"#{name}-normals")

    tris_el = ET.SubElement(mesh_el, f"{{{ns}}}triangles",
                            count=str(len(triangles)))
    ET.SubElement(tris_el, f"{{{ns}}}input", semantic="VERTEX",
                   source=f"#{name}-verts", offset="0")
    ET.SubElement(tris_el, f"{{{ns}}}input", semantic="TEXCOORD",
                   source=f"#{name}-uv", offset="1", set="0")
    p = ET.SubElement(tris_el, f"{{{ns}}}p")
    p_data = []
    for tri in triangles:
        for vi in tri:
            p_data.extend([vi, vi])
    p.text = " ".join(str(v) for v in p_data)

    lib_vis = ET.SubElement(root, f"{{{ns}}}library_visual_scenes")
    scene = ET.SubElement(lib_vis, f"{{{ns}}}visual_scene", id="Scene")
    node = ET.SubElement(scene, f"{{{ns}}}node", id=name, name=name)
    ET.SubElement(node, f"{{{ns}}}instance_geometry", url=f"#{name}-geom")

    lib_scene = ET.SubElement(root, f"{{{ns}}}scene")
    ET.SubElement(lib_scene, f"{{{ns}}}instance_visual_scene", url="#Scene")

    ET.indent(ET.ElementTree(root), space="  ")
    return ET.tostring(root, encoding="unicode", xml_declaration=True)


def gen_brick_walls() -> None:
    walls = [
        ("wall_south", 6.2, 0.1, 0.8),
        ("wall_north", 6.2, 0.1, 0.8),
        ("wall_west", 0.1, 8.2, 0.8),
        ("wall_east", 0.1, 8.2, 0.8),
        ("wall_divide_lower", 0.1, 1.1, 0.8),
        ("wall_divide_arch_r2", 0.1, 0.8, 0.25),
        ("wall_divide_mid", 0.1, 3.2, 0.8),
        ("wall_divide_arch_r3", 0.1, 0.8, 0.25),
        ("wall_divide_upper", 0.1, 2.1, 0.8),
        ("wall_r2_r3_divide", 2.0, 0.1, 0.8),
    ]
    for wname, sx, sy, sz in walls:
        dae = _box_mesh(wname, sx, sy, sz, BRICK_TILE, BRICK_TILE, BRICK_TILE)
        (OUT / f"{wname}.dae").write_text(dae)
        print(f"  wrote {wname}.dae")


def gen_floor() -> None:
    dae = _box_mesh("ground_sand", 100, 100, 0.01, SAND_TILE, SAND_TILE, SAND_TILE)
    (OUT / "ground_sand.dae").write_text(dae)
    print("  wrote ground_sand.dae")


def gen_ceilings() -> None:
    ceilings = [
        ("ceiling_r1", 4.1, 8.1, 0.02),
        ("ceiling_r2", 2.1, 3.1, 0.02),
        ("ceiling_r3", 2.1, 5.1, 0.02),
    ]
    for cname, sx, sy, sz in ceilings:
        dae = _box_mesh(cname, sx, sy, sz, WOOD_TILE_V, WOOD_TILE_U, WOOD_TILE_U)
        (OUT / f"{cname}.dae").write_text(dae)
        print(f"  wrote {cname}.dae")


def gen_pillars() -> None:
    for pname in ("pillar_1", "pillar_2"):
        dae = _cylinder_mesh(pname, radius=0.1, height=0.8,
                              u_tile=BRICK_TILE, v_tile=BRICK_TILE)
        (OUT / f"{pname}.dae").write_text(dae)
        print(f"  wrote {pname}.dae")


if __name__ == "__main__":
    OUT.mkdir(parents=True, exist_ok=True)
    print("Generating basement surface meshes...")
    gen_brick_walls()
    gen_floor()
    gen_ceilings()
    gen_pillars()
    print("Done.")