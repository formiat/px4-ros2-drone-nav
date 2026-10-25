#!/usr/bin/env python3
"""room_memory_truth.py RUN [XMIN YMIN ZMIN XMAX YMAX ZMAX] [--truth PATH]: the memory of a room against the truth (roadmap item 23):
from the run's last raw memory snapshot (RAW_SNAPSHOT_BOUNDS_M covers the room), the occupied voxels inside the box
that have no truth occupancy within 0.5 m (phantoms), the truth-occupied voxels the memory observed free (missed
walls), and the observed share of the box; the box is the snapshot's own when none is given. Truth is the 0.25 m
voxelization in log/tools/truth25 (evaluation only), or the one --truth names (the hall's
environments/fixtures/dark_hall/world.occupancy3d for roadmap item 31). Host."""
import glob, json, sys
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from validate_static_cooperative_scenario import CHUNK_SIZE, Occupancy3D  # noqa: E402

arguments = sys.argv[1:]
truth_path = Path('log/tools/truth25/urban_r025.occupancy3d')
if '--truth' in arguments:
    at = arguments.index('--truth'); truth_path = Path(arguments[at + 1]); del arguments[at:at + 2]
run = arguments[0]
box = [float(v) for v in arguments[1:7]] if len(arguments) >= 7 else None
snapshots = sorted(glob.glob(f'log/runs/{run}/raw_snapshot_3d_revision_*.json'),
                   key=lambda s: int(s.rsplit('_', 1)[1].split('.')[0]))
if not snapshots:
    raise SystemExit(f'{run}: no raw snapshot')
d = json.load(open(snapshots[-1])); g = d['grid']; S = g['chunk_size_cells']; res = g['resolution_m']; ox, oy, oz = g['origin_m']
if box is None:
    box = d['selection']['bounds_m']
truth = Occupancy3D.load(truth_path); b = truth.bounds
occ = np.zeros((b.width, b.height, b.depth), dtype=np.uint8)
for (cx, cy, cz), words in truth.chunks.items():
    bits = np.unpackbits(np.frombuffer(np.array(words, dtype='<u8').tobytes(), dtype=np.uint8), bitorder='little')
    block = bits.reshape(CHUNK_SIZE, CHUNK_SIZE, CHUNK_SIZE).transpose(2, 1, 0)
    x0, y0, z0 = cx * CHUNK_SIZE, cy * CHUNK_SIZE, cz * CHUNK_SIZE
    x1, y1, z1 = min(x0 + CHUNK_SIZE, b.width), min(y0 + CHUNK_SIZE, b.height), min(z0 + CHUNK_SIZE, b.depth)
    occ[x0:x1, y0:y1, z0:z1] |= block[:x1 - x0, :y1 - y0, :z1 - z0]
# the truth dilated by 0.5 m (two voxels) for the phantom test
from scipy.ndimage import binary_dilation  # noqa: E402
near = binary_dilation(occ.astype(bool), iterations=2)
def truth_index(x, y, z):
    return int((x - b.origin_x_m) / b.resolution_m), int((y - b.origin_y_m) / b.resolution_m), int((z - b.origin_z_m) / b.resolution_m)
memory_occupied = []; memory_free = []
for c in d['chunks']:
    cx, cy, cz = c['index']
    oc = np.unpackbits(np.array([int(w, 16) for w in c['occupied_words_hex']], dtype=np.uint64).view(np.uint8), bitorder='little').reshape(S, S, S).transpose(2, 1, 0)
    ob = np.unpackbits(np.array([int(w, 16) for w in c['observed_words_hex']], dtype=np.uint64).view(np.uint8), bitorder='little').reshape(S, S, S).transpose(2, 1, 0)
    for (i, j, k) in zip(*np.nonzero(ob)):
        x, y, z = ox + (cx * S + i + 0.5) * res, oy + (cy * S + j + 0.5) * res, oz + (cz * S + k + 0.5) * res
        if not (box[0] <= x <= box[3] and box[1] <= y <= box[4] and box[2] <= z <= box[5]):
            continue
        (memory_occupied if oc[i, j, k] else memory_free).append((x, y, z))
def inside(points, grid):
    out = 0
    for x, y, z in points:
        tx, ty, tz = truth_index(x, y, z)
        if 0 <= tx < b.width and 0 <= ty < b.height and 0 <= tz < b.depth and grid[tx, ty, tz]:
            out += 1
    return out
phantoms = len(memory_occupied) - inside(memory_occupied, near)
missed = inside(memory_free, occ)
nx, ny, nz = [int((box[i + 3] - box[i]) / res) for i in range(3)]
voxels = nx * ny * nz
print(f"{run}: {Path(snapshots[-1]).name} box {box} ({voxels} voxels of {res} m): observed {len(memory_occupied) + len(memory_free)} "
      f"({100 * (len(memory_occupied) + len(memory_free)) / max(voxels, 1):.1f} %), occupied {len(memory_occupied)}, "
      f"phantoms (no truth occupancy within 0.5 m) {phantoms} ({100 * phantoms / max(len(memory_occupied), 1):.1f} % of the occupied), "
      f"truth walls observed free {missed}")
