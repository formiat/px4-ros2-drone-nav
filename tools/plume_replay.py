#!/usr/bin/env python3
"""plume_replay.py RUN... [NAME=key:value,...]: replays the depth node's plume confirmation (FaceApproach,
drone_city_nav/src/stereo_silent_patches.cpp, specification K44) over the PLUME_CANDIDATE lines of each run's
ros_drone_nav.log under the current settings and the named variants, and judges every laid frame's face against the
run's dense smoke boxes (sigma 0.5/m or more, from the manifest) by the true pose: the face is the candidate's
centroid at its depth in the left camera (HFOV 120 deg), true within 1.5 m of a box. Prints, a variant a line and a
run a column: the first true laid frame (the vehicle's x and the face's depth) and the number of false laid frames.
Keys: lay_cells lay_see front_spread front_trend front_frames drift lost window least_travel shrink t_max grow_min
grow_alone grow_depth_max (none = off). Example: tools/plume_replay.py r1957 r1959 g15=grow_min:0.15
On the host, python3 -I, from the repository's root. Roadmap item 18, the smoke face detector."""
import bisect, csv, json, math, re, sys
from pathlib import Path

FX = 640.0 / math.tan(math.radians(60.0))
CPR = FX / 4.0
CX, CY = 639.5, 479.5
LINE = re.compile(r"\[(\d+\.\d+)\] \[stereo_depth_node\]: PLUME_CANDIDATE along=([-0-9.]+) heading=([-0-9.]+) "
                  r"quartile=([-0-9.na]+) lay=(\d) (.*)$")
SUMMARY = re.compile(r"(\d+)@(\d+),(\d+):(\w+):rim(\d+)@([0-9.na]+)\+([0-9.na]+):trend([0-9.na]+):free(\d+)")


def load(run):
    m = json.loads(Path(f"log/runs/{run}/manifest.json").read_text())
    zones = [[float(a) for a in z.split(",")] for z in m.get("effective_overrides", {}).get("SMOKE_ZONES", "").split(";") if z]
    dense = [z for z in zones if z[6] >= 0.5]
    rows = sorted(([float(a) for a in r[:9]] for r in csv.reader(open(f"log/runs/{run}/gz_pose.csv")) if len(r) >= 9),
                  key=lambda r: r[8])
    walls = [r[8] for r in rows]
    frames = []
    for line in open(f"log/runs/{run}/ros_drone_nav.log"):
        if "PLUME_CANDIDATE" not in line:
            continue
        g = LINE.search(line)
        if not g:
            continue
        w, along, heading, quartile, lay, rest = g.groups()
        k = min(bisect.bisect_left(walls, float(w)), len(rows) - 1)
        pose = rows[k]
        s = SUMMARY.match(rest)
        c = None
        if s:
            cells, col, row, reason, rim, depth, spread, trend, free = s.groups()
            c = dict(cells=int(cells), col=float(col), row=float(row), rim=int(rim), depth=float(depth),
                     spread=float(spread), trend=float(trend), free=int(free), quartile=float(quartile))
        frames.append(dict(t=pose[0], pose=pose, along=float(along), heading=float(heading), lay=int(lay), c=c))
    return dense, frames


def yaw_of(q):
    w, x, y, z = q
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def truth(dense, pose, c):
    yaw = yaw_of(pose[4:8])
    d = c["depth"]
    left = -(4 * c["col"] - CX) / FX * d
    up = -(4 * c["row"] - CY) / FX * d
    p = (pose[1] + math.cos(yaw) * d - math.sin(yaw) * left, pose[2] + math.sin(yaw) * d + math.cos(yaw) * left,
         pose[3] + up)
    best = 99.0
    for b in dense:
        dd = [max(abs(p[i] - b[i]) - b[3 + i] / 2.0, 0.0) for i in range(3)]
        best = min(best, math.sqrt(sum(a * a for a in dd)))
    return best <= 1.5


DEFAULT = dict(lay_cells=15000, lay_see=0.0025, front_spread=0.4, front_trend=0.2, front_frames=3, drift=40.0,
               lost=8, window=64, least_travel=1.2, shrink=0.3, t_max=None, grow_min=0.1, grow_alone=None,
               grow_depth_max=0.1, grow_high=0.3, shrink_high=0.8, climb_max=0.3, climb_explains=None)


def slope_of(points):
    n = len(points)
    sa = sum(a for a, _ in points); sd = sum(d for _, d in points)
    saa = sum(a * a for a, _ in points); sad = sum(a * d for a, d in points)
    spread = n * saa - sa * sa
    if spread <= 1e-9:
        return None
    return (n * sad - sa * sd) / spread


def replay(frames, cfg):
    window = []; crow = ccol = chead = 0.0; lost = 0; front_n = 0; laid = False; laid_lost = 0
    out = []
    for f in frames:
        c = f["c"]
        seen = c is not None and math.isfinite(c["depth"])
        plume_like = seen and c["cells"] >= cfg["lay_cells"] and c["free"] <= cfg["lay_see"] * c["cells"]
        front = plume_like and c["spread"] <= cfg["front_spread"] and c["trend"] < cfg["front_trend"]
        front_n = front_n + 1 if front else 0
        approach = False
        turn = math.remainder(f["heading"] - chead, 2 * math.pi)
        if not seen or (window and math.hypot(c["row"] - crow, c["col"] - (ccol + turn * CPR)) > cfg["drift"]):
            lost += 1
            if lost > cfg["lost"]:
                window = []
        else:
            lost = 0; crow, ccol, chead = c["row"], c["col"], f["heading"]
            window.append((f["along"], c["depth"], math.log(max(c["cells"], 1)), f["pose"][3]))
            while len(window) > cfg["window"] or window[0][0] < f["along"] - 2 * cfg["least_travel"]:
                window.pop(0)
            n = len(window)
            span = f["along"] - window[0][0]
            slope = slope_of([(a, d) for a, d, _, _ in window])
            growth = slope_of([(a, g) for a, _, g, _ in window])
            climb = abs(f["pose"][3] - window[0][3])
            if span >= cfg["least_travel"] and slope is not None:
                approach = slope <= -cfg["shrink"]
                if approach and cfg["t_max"] is not None and n > 2:
                    sa = sum(a for a, _, _, _ in window); sd = sum(d for _, d, _, _ in window)
                    saa = sum(a * a for a, _, _, _ in window)
                    spread = n * saa - sa * sa
                    intercept = (sd - slope * sa) / n
                    rss = sum((d - (intercept + slope * a)) ** 2 for a, d, _, _ in window)
                    se = math.sqrt(rss / (n - 2) / (spread / n)) if spread > 0 else 1e9
                    approach = slope / max(se, 1e-9) <= cfg["t_max"]
                if approach and cfg["grow_min"] is not None:
                    approach = growth is not None and growth >= cfg["grow_min"]
                if approach and cfg["grow_high"] is not None:
                    # A weak growth needs a steep approach of the depth, a strong one not.
                    approach = growth >= cfg["grow_high"] or slope <= -cfg["shrink_high"]
                if approach and cfg["climb_max"] is not None and climb > cfg["climb_max"] * span:
                    # The window climbed or sank too steeply for the depth's fall to be the face's.
                    approach = False
                if approach and cfg["climb_explains"] is not None:
                    # The depth must fall beyond what the climb or sinking explains of a floor's.
                    approach = (-slope * span - cfg["climb_explains"] * climb) >= cfg["shrink"] * span
                if not approach and cfg["grow_alone"] is not None and growth is not None:
                    approach = growth >= cfg["grow_alone"] and slope <= cfg["grow_depth_max"]
        lay = plume_like and (approach or front_n >= cfg["front_frames"])
        if lay:
            laid = True; laid_lost = 0
        elif laid:
            laid_lost += 1
            if laid_lost > cfg["lost"]:
                laid = False
        out.append(lay)
    return out


def main():
    variants = {"current": {}}
    for arg in sys.argv[1:]:
        if "=" in arg:
            name, spec = arg.split("=", 1)
            v = {}
            for kv in spec.split(","):
                k, val = kv.split(":")
                v[k] = None if val == "none" else float(val)
            variants[name] = v
    runs = [a for a in sys.argv[1:] if "=" not in a]
    data = {run: load(run) for run in runs}
    for name, over in variants.items():
        cfg = dict(DEFAULT); cfg.update(over)
        for k in ("lay_cells", "front_frames", "lost", "window"):
            cfg[k] = int(cfg[k])
        line = [f"{name:10s}"]
        for run, (dense, frames) in data.items():
            lays = replay(frames, cfg)
            true_first = None; false = 0
            for f, lay in zip(frames, lays):
                if not lay:
                    continue
                if truth(dense, f["pose"], f["c"]):
                    if true_first is None:
                        true_first = f
                else:
                    false += 1
            first = f"first@x{true_first['pose'][1]:.1f}/d{true_first['c']['depth']:.2f}" if true_first else "no-true"
            line.append(f"{run}:{first},false{false}")
        print(" ".join(line))


if __name__ == "__main__":
    main()
