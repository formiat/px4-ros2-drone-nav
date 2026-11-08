"""body_truth_clearance.py [--grid=PATH] [--map-to-sdf=X,Y,Z] RUN...: the true body's clearance to the true location
over a flight. The body is the stack's own model (cylinder radius 0.55 m, 0.23 m below and 0.35 m above its centre,
upright), its centre the Gazebo model origin plus 0.24 m (base_link over the model's origin at its feet); the location
is the truth voxelization: the urban 0.25 m grid (log/tools/truth25) unless `--grid=` names another (a held-out world's
0.5 m grid of external/environment-candidates/work, in the Gazebo frame: `--map-to-sdf=` then carries the canonical
world's map_to_sdf offset, which the recorded map-frame poses are moved by). Reported after the climb off the pad: the
least clearance, where and when, and the seconds spent under 0.10 and 0.20 m. Evaluation only."""
import csv, sys
from pathlib import Path
import numpy as np
sys.path.insert(0, 'scripts')
from validate_static_cooperative_scenario import CHUNK_SIZE, Occupancy3D
GRID = Path('log/tools/truth25/urban_r025.occupancy3d')
OFFSET = (0.0, 0.0, 0.0)
RUNS = []
for argument in sys.argv[1:]:
    if argument.startswith('--grid='):
        GRID = Path(argument.split('=', 1)[1])
    elif argument.startswith('--map-to-sdf='):
        OFFSET = tuple(float(v) for v in argument.split('=', 1)[1].split(','))
    else:
        RUNS.append(argument)
OCC = Occupancy3D.load(GRID)
B = OCC.bounds; RES = B.resolution_m; R, LO, UP, CENTRE = 0.55, 0.23, 0.35, 0.24
def occupied(i, j, k):
    words = OCC.chunks.get((i // CHUNK_SIZE, j // CHUNK_SIZE, k // CHUNK_SIZE))
    if words is None:
        return False
    bit = i % CHUNK_SIZE + CHUNK_SIZE * (j % CHUNK_SIZE + CHUNK_SIZE * (k % CHUNK_SIZE))
    return (words[bit // 64] >> (bit % 64)) & 1 == 1
def clearance(x, y, z, reach=0.6):
    best = reach
    i0 = int(np.floor((x - R - reach - B.origin_x_m) / RES)); i1 = int(np.floor((x + R + reach - B.origin_x_m) / RES))
    j0 = int(np.floor((y - R - reach - B.origin_y_m) / RES)); j1 = int(np.floor((y + R + reach - B.origin_y_m) / RES))
    k0 = int(np.floor((z - LO - reach - B.origin_z_m) / RES)); k1 = int(np.floor((z + UP + reach - B.origin_z_m) / RES))
    for i in range(i0, i1 + 1):
        bx0 = B.origin_x_m + i * RES
        dx = max(bx0 - x, 0.0, x - (bx0 + RES))
        for j in range(j0, j1 + 1):
            by0 = B.origin_y_m + j * RES
            dy = max(by0 - y, 0.0, y - (by0 + RES))
            radial = max(0.0, np.hypot(dx, dy) - R)
            if radial >= best:
                continue
            for k in range(k0, k1 + 1):
                bz0 = B.origin_z_m + k * RES
                dz = max(bz0 - (z + UP), 0.0, (z - LO) - (bz0 + RES))
                d = np.hypot(radial, dz)
                if d < best and occupied(i, j, k):
                    best = d
    return best
if __name__ == "__main__":
    for run in RUNS:
        rows = [(float(r[0]), float(r[1]) + OFFSET[0], float(r[2]) + OFFSET[1], float(r[3]) + OFFSET[2])
                for r in csv.reader(open(f'log/runs/{run}/gz_pose.csv'))]
        z0 = rows[0][3]
        start = next(n for n, r in enumerate(rows) if r[3] > z0 + 1.0)
        rows = rows[start::5]
        cl = [(clearance(x, y, z + CENTRE), t, x, y, z + CENTRE) for t, x, y, z in rows]
        dt = np.median(np.diff([c[1] for c in cl]))
        m = min(cl)
        print(f"{run}: least {m[0]:.2f} m at t={m[1]:.1f} ({m[2]:.1f},{m[3]:.1f},{m[4]:.1f}); "
              f"under 0.10 m {dt*sum(c[0] < 0.10 for c in cl):.1f} s, under 0.20 m {dt*sum(c[0] < 0.20 for c in cl):.1f} s")
