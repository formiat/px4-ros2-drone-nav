"""dark_drift.py DIR: position error of dark-window replays (replay_dark_<from>_<to>.csv) against the truth at the end of
the dark window and 10 s after, beside the uninterrupted replay (replay_new.csv) at the same moments."""
import sys, glob, re, numpy as np
d = sys.argv[1]
def load(p):
    a = np.genfromtxt(p, delimiter=',', names=True)
    return a['stamp_s'], np.c_[a['px'] - a['tx'], a['py'] - a['ty'], a['pz'] - a['tz']], np.c_[a['vx'], a['vy'], a['vz']]
tb, eb, _ = load(f'{d}/replay_new.csv')
def at(t, e, s): return e[min(np.searchsorted(t, s), len(t) - 1)]
for p in sorted(glob.glob(f'{d}/replay_dark_*.csv'), key=lambda x: [float(v) for v in re.findall(r'dark_([\d.]+)_([\d.]+)', x)[0]]):
    a, b = [float(v) for v in re.findall(r'dark_([\d.]+)_([\d.]+)', p)[0]]
    t, e, v = load(p)
    for label, s in (('end', b), ('+10s', b + 10.0)):
        drift = np.linalg.norm(at(t, e, s) - at(tb, eb, s))
        hv = np.linalg.norm(at(t, e, s)[:2] - at(tb, eb, s)[:2])
        print(f'dark {a:6.1f}-{b:6.1f} ({b - a:4.0f} s) at {label:4s}: drift from the lit replay {drift:6.2f} m (horizontal {hv:5.2f})')
