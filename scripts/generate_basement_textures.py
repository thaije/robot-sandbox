"""Generate PBR texture maps for the basement scenario.

Creates normal maps, roughness maps, and metalness maps derived from
existing albedo textures, plus new procedural wood and metal textures.

Output: worlds/media/textures/
"""
from __future__ import annotations

import math
import random
from pathlib import Path

import numpy as np
from PIL import Image

OUT = Path(__file__).resolve().parent.parent / "worlds" / "media" / "textures"
SIZE = 512


def _save(arr: np.ndarray, name: str) -> None:
    img = Image.fromarray(arr)
    img.save(OUT / name)
    print(f"  wrote {name}")


def height_to_normal(height: np.ndarray, strength: float = 2.0) -> np.ndarray:
    """Convert a height map to a normal map (RGB)."""
    h = height.astype(np.float32) / 255.0
    # Sobel-like gradient
    dx = np.zeros_like(h)
    dy = np.zeros_like(h)
    dx[:, 1:] = h[:, 1:] - h[:, :-1]
    dy[1:, :] = h[1:, :] - h[:-1, :]
    nx = -dx * strength
    ny = -dy * strength
    nz = np.ones_like(h)
    length = np.sqrt(nx**2 + ny**2 + nz**2)
    nx /= length
    ny /= length
    nz /= length
    # Encode from [-1,1] to [0,255]
    r = ((nx + 1) * 0.5 * 255).clip(0, 255).astype(np.uint8)
    g = ((ny + 1) * 0.5 * 255).clip(0, 255).astype(np.uint8)
    b = ((nz + 1) * 0.5 * 255).clip(0, 255).astype(np.uint8)
    return np.stack([r, g, b], axis=-1)


def gen_brick_maps() -> None:
    """Generate normal/roughness/metalness maps from brick.png albedo."""
    print("Generating brick PBR maps...")
    albedo = np.array(Image.open(OUT / "brick.png"))

    # Height from luminance
    lum = (0.299 * albedo[..., 0] + 0.587 * albedo[..., 1] + 0.114 * albedo[..., 2]).astype(np.float32)
    height = (lum / lum.max() * 255).astype(np.uint8)

    normal = height_to_normal(height, strength=3.0)
    _save(normal, "brick_normal.png")

    # Roughness: brick surface ~0.9 (rough), mortar ~0.95 (very rough)
    roughness = np.full_like(lum, 0.9 * 255, dtype=np.uint8)
    roughness[lum < 80] = int(0.95 * 255)
    _save(roughness, "brick_roughness.png")

    # Metalness: brick is non-metallic ~0
    metalness = np.zeros((SIZE, SIZE), dtype=np.uint8)
    _save(metalness, "brick_metalness.png")


def gen_sand_maps() -> None:
    """Generate normal/roughness/metalness maps from sand.png albedo."""
    print("Generating sand PBR maps...")
    albedo = np.array(Image.open(OUT / "sand.png"))
    h, w = albedo.shape[:2]

    lum = (0.299 * albedo[..., 0] + 0.587 * albedo[..., 1] + 0.114 * albedo[..., 2]).astype(np.float32)
    height = (lum / lum.max() * 255).astype(np.uint8)

    normal = height_to_normal(height, strength=1.5)
    _save(normal, "sand_normal.png")

    # Sand is very rough ~0.95
    roughness = np.full((h, w), int(0.95 * 255), dtype=np.uint8)
    _save(roughness, "sand_roughness.png")

    metalness = np.zeros((h, w), dtype=np.uint8)
    _save(metalness, "sand_metalness.png")


def gen_wood_planks() -> None:
    """Generate wood plank albedo and PBR maps for ceiling."""
    print("Generating wood_planks PBR maps...")
    rng = random.Random(42)

    # Albedo: brownish wood planks
    albedo = np.zeros((SIZE, SIZE, 3), dtype=np.uint8)
    # Base wood color
    base_r, base_g, base_b = 90, 58, 34

    n_planks = 8
    plank_h = SIZE // n_planks
    for p in range(n_planks):
        y0 = p * plank_h
        y1 = (p + 1) * plank_h
        # Per-plank color variation
        pr = base_r + rng.randint(-10, 10)
        pg = base_g + rng.randint(-8, 8)
        pb = base_b + rng.randint(-6, 6)
        for y in range(y0, min(y1, SIZE)):
            for x in range(SIZE):
                # Wood grain: horizontal streaks
                grain = int(8 * math.sin(x * 0.5 + y * 0.02 + rng.random() * 0.3))
                noise = rng.randint(-4, 4)
                albedo[y, x, 0] = max(0, min(255, pr + grain + noise))
                albedo[y, x, 1] = max(0, min(255, pg + grain // 2 + noise))
                albedo[y, x, 2] = max(0, min(255, pb + grain // 3 + noise))
            # Plank boundary dark line
        if p < n_planks - 1:
            line_y = y1
            if line_y < SIZE:
                albedo[line_y, :, :] = [40, 25, 15]

    # Add knot holes
    for _ in range(4):
        kx = rng.randint(40, SIZE - 40)
        ky = rng.randint(40, SIZE - 40)
        kr = rng.randint(4, 8)
        for yy in range(max(0, ky - kr), min(SIZE, ky + kr)):
            for xx in range(max(0, kx - kr), min(SIZE, kx + kr)):
                if (xx - kx) ** 2 + (yy - ky) ** 2 < kr ** 2:
                    albedo[yy, xx] = [55, 35, 20]

    _save(albedo, "wood_planks.png")

    # Height/normal map: grooves between planks + grain
    height = np.zeros((SIZE, SIZE), dtype=np.float32)
    for p in range(n_planks):
        y0 = p * plank_h
        y1 = (p + 1) * plank_h
        # Plank surface at height 200
        height[y0:y1, :] = 200
        # Darker grain lines
        for y in range(y0, min(y1, SIZE)):
            for x in range(SIZE):
                grain_v = 10 * math.sin(x * 0.5 + y * 0.02)
                height[y, x] += grain_v
        # Groove between planks
        if p < n_planks - 1:
            line_y = y1
            if line_y < SIZE:
                height[line_y, :] = 0
    height = (height / height.max() * 255).clip(0, 255).astype(np.uint8)
    normal = height_to_normal(height, strength=2.0)
    _save(normal, "wood_planks_normal.png")

    # Roughness: wood ~0.6-0.7, grooves ~0.85
    roughness = np.full((SIZE, SIZE), int(0.65 * 255), dtype=np.uint8)
    for p in range(n_planks - 1):
        line_y = (p + 1) * plank_h
        if line_y < SIZE:
            roughness[line_y, :] = int(0.85 * 255)
    _save(roughness, "wood_planks_roughness.png")

    # Metalness: wood is non-metallic
    metalness = np.zeros((SIZE, SIZE), dtype=np.uint8)
    _save(metalness, "wood_planks_metalness.png")


def gen_metal_maps() -> None:
    """Generate PBR maps for metal surfaces (pipes, conduits, hangers).

    Metals don't need an albedo map — the PBR metal workflow uses the
    diffuse color as the albedo tint. We just need roughness + metalness.
    """
    print("Generating metal PBR maps...")

    # Roughness: galvanized/painted metal ~0.4, with variation
    rng_painted = np.random.RandomState(99)
    roughness = np.full((SIZE, SIZE), int(0.4 * 255), dtype=np.int16)
    noise = rng_painted.randint(-10, 11, size=(SIZE, SIZE)).astype(np.int16)
    roughness = np.clip(roughness + noise, 0, 255).astype(np.uint8)
    _save(roughness, "metal_painted_roughness.png")

    # Metalness: 255 = fully metallic
    metalness = np.full((SIZE, SIZE), 255, dtype=np.uint8)
    _save(metalness, "metal_painted_metalness.png")

    # Rougher version for aged/pitted metal (sewer pipes)
    rng_aged = np.random.RandomState(42)
    roughness_dark = np.full((SIZE, SIZE), int(0.55 * 255), dtype=np.int16)
    noise2 = rng_aged.randint(-15, 16, size=(SIZE, SIZE)).astype(np.int16)
    roughness_dark = np.clip(roughness_dark + noise2, 0, 255).astype(np.uint8)
    _save(roughness_dark, "metal_aged_roughness.png")

    metalness_dark = np.full((SIZE, SIZE), 230, dtype=np.uint8)
    _save(metalness_dark, "metal_aged_metalness.png")


def gen_cardboard_maps() -> None:
    """Generate PBR maps for cardboard boxes."""
    print("Generating cardboard PBR maps...")
    rng = random.Random(77)

    # Procedural height map for cardboard
    height = np.full((SIZE, SIZE), 180, dtype=np.float32)
    # Corrugation lines (horizontal)
    for y in range(SIZE):
        wave = 10 * math.sin(y * 0.4)
        height[y, :] += wave
    # Add noise for fiber texture
    for y in range(SIZE):
        for x in range(SIZE):
            height[y, x] += rng.randint(-8, 8)

    height_uint = (height / height.max() * 255).clip(0, 255).astype(np.uint8)
    normal = height_to_normal(height_uint, strength=1.5)
    _save(normal, "cardboard_normal.png")

    # Roughness: very rough ~0.85
    roughness = np.full((SIZE, SIZE), int(0.85 * 255), dtype=np.uint8)
    _save(roughness, "cardboard_roughness.png")

    # Metalness: non-metallic
    metalness = np.zeros((SIZE, SIZE), dtype=np.uint8)
    _save(metalness, "cardboard_metalness.png")


if __name__ == "__main__":
    OUT.mkdir(parents=True, exist_ok=True)
    gen_brick_maps()
    gen_sand_maps()
    gen_wood_planks()
    gen_metal_maps()
    gen_cardboard_maps()
    print("Done — all PBR textures generated.")