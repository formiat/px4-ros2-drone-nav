"""sky_above.py [--grid=PATH] [--map-to-sdf=X,Y,Z] [--range=M] RUN|DIRECTORY...: over a flight's true poses, the share
of the time from the climb off the pad in which nothing of the true location stands over the vehicle: no ray of a
cone about the vertical, cast from the sensor's place (the Gazebo model origin plus 0.31 m) to the lidar's range (35
m), meets an occupied voxel of the truth voxelization. Printed for cones of 10, 20, 30, 45 and 60 degrees of half
angle, with the number of changes between "open" and "roofed" along the flight and the longest stretch of each. The
grid is the urban 0.25 m one (log/tools/truth25) unless `--grid=` names another, in the Gazebo frame with the
canonical world's `map_to_sdf` offset by `--map-to-sdf=` (as tools/body_truth_clearance.py). A run is a number of
log/runs or a directory that holds gz_pose.csv. The measure of roadmap item 45: whether "the sky is open over the
vehicle" tells the caves from the open sky. Evaluation only."""
import csv
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, 'scripts')
from validate_static_cooperative_scenario import CHUNK_SIZE, Occupancy3D

HALF_ANGLES_DEG = (10, 20, 30, 45, 60)
SENSOR_OVER_ORIGIN_M = 0.31
AIRBORNE_OVER_START_M = 1.0
SAMPLE_PERIOD_S = 0.1


def dense_chunks(occupancy):
    """The chunk index of every chunk place (-1 where none) and the chunks' voxels as booleans."""
    bounds = occupancy.bounds
    shape = tuple(-(-n // CHUNK_SIZE) for n in (bounds.width, bounds.height, bounds.depth))
    index = np.full(shape, -1, dtype=np.int32)
    voxels = np.zeros((len(occupancy.chunks), CHUNK_SIZE ** 3), dtype=bool)
    for number, (place, words) in enumerate(occupancy.chunks.items()):
        if all(0 <= place[axis] < shape[axis] for axis in range(3)):
            index[place] = number
        voxels[number] = np.unpackbits(np.array(words, dtype='<u8').view(np.uint8), bitorder='little')
    return index, voxels


def cone_directions(half_angle_deg, step_deg=5.0):
    """Unit rays about the vertical, a ring every step_deg of polar angle, as dense in azimuth."""
    rays = [(0.0, 0.0, 1.0)]
    for polar in np.arange(step_deg, half_angle_deg + 1.0e-6, step_deg):
        count = max(6, int(round(360.0 * np.sin(np.radians(polar)) / step_deg)))
        for azimuth in np.arange(count) * (2.0 * np.pi / count):
            rays.append((np.sin(np.radians(polar)) * np.cos(azimuth), np.sin(np.radians(polar)) * np.sin(azimuth),
                         np.cos(np.radians(polar))))
    return np.array(rays)


def main():
    grid = Path('log/tools/truth25/urban_r025.occupancy3d')
    offset = (0.0, 0.0, 0.0)
    reach_m = 35.0
    runs = []
    for argument in sys.argv[1:]:
        if argument.startswith('--grid='):
            grid = Path(argument.split('=', 1)[1])
        elif argument.startswith('--map-to-sdf='):
            offset = tuple(float(v) for v in argument.split('=', 1)[1].split(','))
        elif argument.startswith('--range='):
            reach_m = float(argument.split('=', 1)[1])
        else:
            runs.append(argument)
    occupancy = Occupancy3D.load(grid)
    bounds = occupancy.bounds
    index, voxels = dense_chunks(occupancy)
    origin = np.array((bounds.origin_x_m, bounds.origin_y_m, bounds.origin_z_m))
    size = np.array((bounds.width, bounds.height, bounds.depth))
    steps = np.arange(bounds.resolution_m, reach_m, 0.5 * bounds.resolution_m)

    def any_hit(points, rays):
        """For each point, whether any ray meets an occupied voxel within the range."""
        hit = np.zeros(len(points), dtype=bool)
        for start in range(0, len(points), 64):
            block = points[start:start + 64]
            cells = np.floor((block[:, None, None, :] + rays[None, :, None, :] * steps[None, None, :, None] - origin)
                             / bounds.resolution_m).astype(np.int64)
            inside = np.all((cells >= 0) & (cells < size), axis=-1)
            cells = np.where(inside[..., None], cells, 0)
            chunk = index[cells[..., 0] // CHUNK_SIZE, cells[..., 1] // CHUNK_SIZE, cells[..., 2] // CHUNK_SIZE]
            bit = (cells[..., 0] % CHUNK_SIZE
                   + CHUNK_SIZE * (cells[..., 1] % CHUNK_SIZE + CHUNK_SIZE * (cells[..., 2] % CHUNK_SIZE)))
            occupied = inside & (chunk >= 0) & voxels[np.maximum(chunk, 0), bit]
            hit[start:start + 64] = occupied.any(axis=(1, 2))
        return hit

    for run in runs:
        directory = Path(run) if '/' in run else Path(f'log/runs/{run}')
        rows = np.array([[float(v) for v in row[:4]] for row in csv.reader(open(directory / 'gz_pose.csv'))])
        airborne = rows[np.argmax(rows[:, 3] > rows[0, 3] + AIRBORNE_OVER_START_M):]
        keep = np.concatenate(([True], np.diff(np.floor(airborne[:, 0] / SAMPLE_PERIOD_S)) > 0))
        samples = airborne[keep]
        points = samples[:, 1:4] + np.array(offset) + np.array((0.0, 0.0, SENSOR_OVER_ORIGIN_M))
        print(f'{directory.name}: {len(samples)} samples of {SAMPLE_PERIOD_S} s from the climb, range {reach_m:g} m, '
              f'grid {grid.name}')
        for half_angle in HALF_ANGLES_DEG:
            roofed = any_hit(points, cone_directions(half_angle))
            changes = np.flatnonzero(np.diff(roofed.astype(np.int8)) != 0)
            edges = np.concatenate(([0], changes + 1, [len(roofed)]))
            longest = {True: 0.0, False: 0.0}
            for first, last in zip(edges[:-1], edges[1:]):
                longest[bool(roofed[first])] = max(longest[bool(roofed[first])], (last - first) * SAMPLE_PERIOD_S)
            print(f'  cone {half_angle:2d} deg: open {100.0 * (1.0 - roofed.mean()):6.2f} % of the samples, '
                  f'{len(changes)} changes, longest open {longest[False]:.1f} s, longest roofed {longest[True]:.1f} s')


if __name__ == '__main__':
    main()
