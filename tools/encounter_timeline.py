#!/usr/bin/env python3
"""encounter_timeline.py RUN [BEFORE_S]: the ten seconds before a flight's closest approach to its walking body, twice
a second, by the truth: the distance to the body's surface, the vehicle's speed, the closing speed and the stopping
margin (as tools/body_encounters.py measures them), the binding speed limiter and the reference, and what the memory
followed at that moment (the followed body nearest the true one and how far from it)."""
import bisect, csv, json, math, re, sys
run = sys.argv[1]; before = float(sys.argv[2]) if len(sys.argv) > 2 else 10.0
R, H, VR = 0.25, 1.7, 0.35
body = [(float(r['sim_s']), float(r['x']), float(r['y']), float(r['z_base'])) for r in csv.DictReader(open(f'log/runs/{run}/moving_bodies.csv'))]
pose = []
for line in open(f'log/runs/{run}/gz_pose.csv'):
    f = line.split(',')
    try: pose.append((float(f[0]), float(f[1]), float(f[2]), float(f[3]), float(f[-1])))
    except ValueError: pass
bt = [b[0] for b in body]; pt = [p[0] for p in pose]; pw = [p[4] for p in pose]
def body_at(t):
    k = min(len(body) - 1, max(0, bisect.bisect_left(bt, t))); return body[k]
def dist(p, b):
    gap_h = max(0.0, math.hypot(p[1] - b[1], p[2] - b[2]) - R); over = p[3] - (b[3] + H); under = b[3] - p[3]
    gap_v = max(0.0, over, under); return math.hypot(gap_h, gap_v), gap_h, over
d = [(dist(p, body_at(p[0]))[0], p[0]) for p in pose if bt[0] <= p[0] <= bt[-1]]
dmin, tstar = min(d)
track = [json.loads(l) for l in open(f'log/runs/{run}/mppi/mppi_track.jsonl')]
tt = [r['stamp_ns'] / 1e9 for r in track]
followed = []
pat = re.compile(r'\[(\d+\.\d+)\].*FOLLOWED_BODIES followed=(\d+)(?: stepping=\d+)? nearest=(.*)')
mem = re.compile(r'\[(\d+\.\d+)\].*LIDAR3D_MEMORY.*moving_voxels=(\d+) followed_bodies=(\d+) swept_voxels=(\d+)(?: warm_voxels=(\d+) body_clusters=(\d+))?')
def sim_of(wall):
    k = min(len(pw) - 1, max(0, bisect.bisect_left(pw, wall))); return pt[k]
scans = []
for line in open(f'log/runs/{run}/ros_drone_nav.log', errors='replace'):
    m = pat.search(line)
    if m:
        nums = re.findall(r'\[(-?[\d.]+) (-?[\d.]+) (-?[\d.]+) r([\d.]+) h([\d.]+) v(-?[\d.]+),(-?[\d.]+) n\d+ a[\d.]+ d[\d.]+ warm', m.group(3))
        followed.append((sim_of(float(m.group(1))), [tuple(map(float, n)) for n in nums]))
    m = mem.search(line)
    if m: scans.append((sim_of(float(m.group(1))), int(m.group(2)), int(m.group(3)), int(m.group(4)), m.group(5) or '?', m.group(6) or '?'))
ft = [f[0] for f in followed]; st = [s[0] for s in scans]
print(f'{run}: closest {dmin:.2f} m at {tstar:.1f} s')
prev = None; t = tstar - before
while t <= tstar + 2.0 + 1e-9:
    k = min(len(pose) - 1, bisect.bisect_left(pt, t)); p = pose[k]; b = body_at(p[0])
    dd, gh, over = dist(p, b)
    k2 = max(0, bisect.bisect_left(pt, t - 0.5)); p2 = pose[k2]
    sp = math.dist(p[1:4], p2[1:4]) / max(1e-6, p[0] - p2[0])
    closing = (prev - dd) / 0.5 if prev is not None else 0.0; prev = dd
    stop = max(0.0, closing) * 0.3 + max(0.0, closing) ** 2 / 8.0
    r = track[min(len(track) - 1, bisect.bisect_left(tt, t))]
    f = followed[max(0, bisect.bisect_right(ft, t) - 1)] if followed else None
    seen = '-'
    if f and abs(f[0] - t) < 1.5 and f[1]:
        e = min(math.hypot(n[0] - b[1], n[1] - b[2]) for n in f[1]); seen = f'{len(f[1])} followed, nearest to the truth {e:.1f} m'
    s = scans[max(0, bisect.bisect_right(st, t) - 1)] if scans else (0, 0, 0, 0, '?', '?')
    print(f'{t - tstar:+5.1f} s veh=({p[1]:.1f},{p[2]:.1f},{p[3]:.1f}) body=({b[1]:.1f},{b[2]:.1f}) d={dd:.2f} (gap {gh:.2f}, over {over:+.2f}) v={sp:.2f} closing={closing:+.2f} margin={dd - VR - stop:+.2f} lim={r["limiter"]} ref={r["reference_speed_mps"]:.2f} | layer {s[1]} voxels, warm {s[4]}, clusters {s[5]}, scan followed {s[2]}, swept {s[3]} | {seen}')
    t += 0.5
