#!/usr/bin/env python3
"""thermal_body_rig.py OUT_DIR NAME... (inside the container, after `make build`): the stack's own thermal readers
over the frames of the rig's levels (tools/smoke_rig.py, `body=T` and `heat=T`; roadmap item 21): for each level
the first saved frame of the thermal core goes through `thermal_frame_bodies`, the tool built from the library the
nodes read with, at the body's true range from the core where the level has a body. One row a level: the source's
reading, the warm regions and their pixels, how many are of a body's size at the true range and by their own size,
and the range the body-sized region's size gives against the truth."""
from __future__ import annotations

import ast
import re
import subprocess
import sys
import tempfile
from pathlib import Path

import numpy as np

REPOSITORY = Path(__file__).resolve().parents[1]
TOOL = REPOSITORY / "build/drone_city_nav/thermal_frame_bodies"
CORE_X_M = 0.32


def main() -> int:
    if len(sys.argv) < 3:
        print(__doc__)
        return 2
    out = Path(sys.argv[1])
    print("| level | body K, at m | source read | warm regions (pixels) | body-sized at the true range | "
          "body-sized by its own size | range by size m |")
    print("|---|---|---|---|---|---|---|")
    for name in sys.argv[2:]:
        data = np.load(out / f"{name}.npz", allow_pickle=True)
        meta = ast.literal_eval(str(data["meta"][0]))
        frame = data["thermal_core"][0].astype("<u2")
        distance = float(meta["bodyx"]) - CORE_X_M if meta.get("body") else 0.0
        with tempfile.NamedTemporaryFile(suffix=".raw") as raw:
            raw.write(frame.tobytes())
            raw.flush()
            arguments = [str(TOOL), raw.name, str(frame.shape[1]), str(frame.shape[0])]
            if distance > 0.0:
                arguments.append(f"{distance:.3f}")
            lines = subprocess.run(arguments, capture_output=True, text=True, check=True).stdout.splitlines()
        source = next(line for line in lines if line.startswith("SOURCE"))
        regions = [dict(re.findall(r"(\w+)=([-\d.]+)", line)) for line in lines if line.startswith("REGION")]
        at_range = [region for region in regions if region["body_at_range"] == "1"]
        by_size = [region for region in regions if region["body_by_size"] == "1"]
        body_text = f"{meta['body']:g}, {distance:.2f}" if meta.get("body") else "none"
        ranges = ", ".join(region["range_by_size_m"] for region in (at_range or by_size)) or "-"
        print(f"| {name} | {body_text} | {'yes' if 'hot=1' in source else 'no'} | "
              f"{len(regions)} ({', '.join(region['warm_pixels'] for region in regions) or '-'}) | "
              f"{len(at_range) if distance > 0.0 else '-'} | {len(by_size)} | {ranges} |")
    return 0


if __name__ == "__main__":
    sys.exit(main())
