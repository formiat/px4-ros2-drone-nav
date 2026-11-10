#!/usr/bin/env python3
"""memory_layers.py RUN|DIR X0 Y0 X1 Y1 [Z0 Z1] [--truth GRID]: per 0.25 m layer of a flight's last raw snapshot of
the obstacle memory in a region of the plan, the cells observed free, occupied and unknown, and the truth's occupied
cells of the layer: held occupied, observed free. RUN is a run of log/runs, DIR a directory of snapshots (a memory
replay's, tools/replay_memory.sh). Roadmap item 11 read the stereo memory under the hall's floor with it (r3212: free
voxels 0.6 m under a floor held occupied over half the hall). The truth defaults to Urban's at 0.25 m. Host."""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

REPOSITORY = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPOSITORY / "scripts"))

import survey_coverage as sc  # noqa: E402

URBAN_TRUTH = REPOSITORY / "log/tools/truth25/urban_r025.occupancy3d"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("run")
    parser.add_argument("box", nargs="+", type=float, help="X0 Y0 X1 Y1 [Z0 Z1], map frame, metres")
    parser.add_argument("--truth", type=Path, default=URBAN_TRUTH)
    args = parser.parse_args()
    if len(args.box) not in (4, 6):
        parser.error("the region is X0 Y0 X1 Y1 or X0 Y0 X1 Y1 Z0 Z1")
    x0, y0, x1, y1 = args.box[:4]
    z0, z1 = args.box[4:] if len(args.box) == 6 else (0.0, 16.0)
    run = Path(args.run).resolve() if "/" in args.run else REPOSITORY / "log/runs" / args.run
    snapshot = sc.last_snapshot(run)
    if snapshot is None:
        print(f"{run}: no raw snapshot", file=sys.stderr)
        return 1
    revision, res, origin, observed, occupied = sc.load_memory(snapshot, [x0, y0, z0, x1, y1, z1])
    truth = sc.load_truth(args.truth, res, origin, observed.shape) if args.truth.is_file() else None
    print(f"{run.name} {snapshot.name} region x {x0}..{x1} y {y0}..{y1}: cells a layer "
          f"{observed.shape[0] * observed.shape[1]}")
    for k in range(observed.shape[2]):
        o = observed[:, :, k]
        c = occupied[:, :, k]
        line = (f"  z {origin[2] + (k + 0.5) * res:5.2f}: free {int((o & ~c).sum()):5d} occupied {int(c.sum()):5d} "
                f"unknown {int((~o).sum()):5d}")
        if truth is not None:
            t = truth[:, :, k]
            line += (f" | truth occupied {int(t.sum()):5d}, of them held occupied {int((t & c).sum()):5d}, "
                     f"observed free {int((t & o & ~c).sum()):5d}")
        print(line)
    return 0


if __name__ == "__main__":
    sys.exit(main())
