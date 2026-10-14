#!/usr/bin/env python3
"""generate.py: writes this directory's world.sdf and texture.png (roadmap item 29, the large dark hall).

A closed hall 80 x 60 m and 20 m high, the vehicle's pad by the west wall and its goal 60 m east of it. The floor
and the walls carry a concrete texture of the location's own statistics in 10 m tiles (about 1 cm a texel); the
ceiling is a plain grey. The
world has no light of its own: the dark materialization keeps it unlit and only the carried light shines (A10).
Deterministic: the same files on every run.
"""
from __future__ import annotations

import struct
import zlib
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
LENGTH_M, WIDTH_M, HEIGHT_M = 80.0, 60.0, 20.0
TILE_M = 10.0
TEXTURE_PX = 1024
PAD_XY_M = (8.0, 30.0)
PAD_SIZE_M = (3.0, 3.0, 0.5)


def value_noise(rng: np.random.Generator, cells: int) -> np.ndarray:
    """Smooth noise in [-1, 1] over the texture with `cells` lattice cells a side, wrapping at the edges."""
    lattice = rng.uniform(-1.0, 1.0, (cells, cells))
    coordinate = np.arange(TEXTURE_PX) * cells / TEXTURE_PX
    low = np.floor(coordinate).astype(int)
    fraction = coordinate - low
    weight = fraction * fraction * (3.0 - 2.0 * fraction)
    high = (low + 1) % cells
    rows = lattice[low] * (1.0 - weight)[:, None] + lattice[high] * weight[:, None]
    return rows[:, low] * (1.0 - weight)[None, :] + rows[:, high] * weight[None, :]


def texture() -> np.ndarray:
    """Concrete as the location's own (Urban's ConcreteDirty: linear albedo 0.09, deviation 0.03): octaves from a
    metre to a centimetre and a fine grain, scaled to a mean albedo of 0.10 and a deviation of 0.03."""
    rng = np.random.default_rng(29)
    field = np.zeros((TEXTURE_PX, TEXTURE_PX))
    for cells, amplitude in ((8, 1.0), (32, 0.8), (128, 0.6), (512, 0.45)):
        field += amplitude * value_noise(rng, cells)
    field += 0.35 * rng.standard_normal((TEXTURE_PX, TEXTURE_PX))
    linear = np.clip(0.10 + 0.03 * (field - field.mean()) / field.std(), 0.01, 0.95)
    # sRGB for the renderer's albedo map.
    srgb = np.where(linear <= 0.0031308, 12.92 * linear, 1.055 * linear ** (1.0 / 2.4) - 0.055)
    grey = np.round(255.0 * srgb).astype(np.uint8)
    return np.stack([grey] * 3, axis=-1)


def write_png(path: Path, rgb: np.ndarray) -> None:
    height, width, _ = rgb.shape
    raw = b"".join(b"\x00" + rgb[row].tobytes() for row in range(height))

    def chunk(kind: bytes, data: bytes) -> bytes:
        return (struct.pack(">I", len(data)) + kind + data
                + struct.pack(">I", zlib.crc32(kind + data) & 0xFFFFFFFF))

    path.write_bytes(b"\x89PNG\r\n\x1a\n"
                     + chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0))
                     + chunk(b"IDAT", zlib.compress(raw, 9)) + chunk(b"IEND", b""))


def tile(name: str, pose: str, size: str) -> str:
    return f"""        <visual name="{name}">
          <pose>{pose}</pose>
          <geometry><box><size>{size}</size></box></geometry>
          <material>
            <diffuse>1 1 1 1</diffuse>
            <specular>0.05 0.05 0.05 1</specular>
            <pbr><metal><albedo_map>texture.png</albedo_map><metalness>0</metalness><roughness>0.9</roughness></metal></pbr>
          </material>
        </visual>
"""


def world() -> str:
    floor = "".join(tile(f"floor_{i}_{j}", f"{TILE_M * (i + 0.5)} {TILE_M * (j + 0.5)} 0.005 0 0 0",
                         f"{TILE_M} {TILE_M} 0.01")
                    for i in range(int(LENGTH_M / TILE_M)) for j in range(int(WIDTH_M / TILE_M)))
    walls = []
    for side, x in (("west", 0.005), ("east", LENGTH_M - 0.005)):
        walls += [tile(f"{side}_{j}_{k}", f"{x} {TILE_M * (j + 0.5)} {TILE_M * (k + 0.5)} 0 0 0",
                       f"0.01 {TILE_M} {TILE_M}")
                  for j in range(int(WIDTH_M / TILE_M)) for k in range(int(HEIGHT_M / TILE_M))]
    for side, y in (("south", 0.005), ("north", WIDTH_M - 0.005)):
        walls += [tile(f"{side}_{i}_{k}", f"{TILE_M * (i + 0.5)} {y} {TILE_M * (k + 0.5)} 0 0 0",
                       f"{TILE_M} 0.01 {TILE_M}")
                  for i in range(int(LENGTH_M / TILE_M)) for k in range(int(HEIGHT_M / TILE_M))]
    shell = [("floor", f"{LENGTH_M / 2} {WIDTH_M / 2} -0.25", f"{LENGTH_M + 1} {WIDTH_M + 1} 0.5"),
             ("ceiling", f"{LENGTH_M / 2} {WIDTH_M / 2} {HEIGHT_M + 0.25}", f"{LENGTH_M + 1} {WIDTH_M + 1} 0.5"),
             ("west_wall", f"-0.25 {WIDTH_M / 2} {HEIGHT_M / 2}", f"0.5 {WIDTH_M + 1} {HEIGHT_M}"),
             ("east_wall", f"{LENGTH_M + 0.25} {WIDTH_M / 2} {HEIGHT_M / 2}", f"0.5 {WIDTH_M + 1} {HEIGHT_M}"),
             ("south_wall", f"{LENGTH_M / 2} -0.25 {HEIGHT_M / 2}", f"{LENGTH_M + 1} 0.5 {HEIGHT_M}"),
             ("north_wall", f"{LENGTH_M / 2} {WIDTH_M + 0.25} {HEIGHT_M / 2}", f"{LENGTH_M + 1} 0.5 {HEIGHT_M}")]
    shell_sdf = "".join(f"""        <collision name="{name}">
          <pose>{pose} 0 0 0</pose>
          <geometry><box><size>{size}</size></box></geometry>
        </collision>
""" for name, pose, size in shell)
    ceiling = f"""        <visual name="ceiling">
          <pose>{LENGTH_M / 2} {WIDTH_M / 2} {HEIGHT_M + 0.25} 0 0 0</pose>
          <geometry><box><size>{LENGTH_M + 1} {WIDTH_M + 1} 0.5</size></box></geometry>
          <material><diffuse>0.1 0.1 0.1 1</diffuse><specular>0.05 0.05 0.05 1</specular></material>
        </visual>
"""
    pad_x, pad_y = PAD_XY_M
    sx, sy, sz = PAD_SIZE_M
    return f"""<?xml version="1.0" ?>
<!-- Generated by generate.py: the large dark hall of roadmap item 29. -->
<sdf version="1.10">
  <world name="dark_hall">
    <gravity>0 0 -9.81</gravity>
    <physics name="default_physics" type="ignored">
      <max_step_size>0.004</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <model name="hall">
      <static>true</static>
      <link name="shell">
{shell_sdf}{''.join([floor] + walls)}{ceiling}      </link>
    </model>

    <model name="pad">
      <static>true</static>
      <link name="pad">
        <pose>{pad_x} {pad_y} {sz / 2} 0 0 0</pose>
        <collision name="collision">
          <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
        </collision>
{tile("visual", "0 0 0 0 0 0", f"{sx} {sy} {sz}")}      </link>
    </model>
  </world>
</sdf>
"""


def main() -> int:
    write_png(HERE / "texture.png", texture())
    (HERE / "world.sdf").write_text(world())
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
