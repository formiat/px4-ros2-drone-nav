"""drag_report.py: horizontal position and velocity error at the end of each dark window of the replays written by
drag_bench.sh (replay_dk_<drag>_<from>_<to>.csv), with and without the rotor drag fusion; the lit replay's final error."""
import glob, re, numpy as np
for run in ['r779', 'r795']:
    d = f'log/tools/replay/{run}'
    for drag in ['0', '0.106']:
        try:
            a = np.genfromtxt(f'{d}/replay_lit_drag{drag}.csv', delimiter=',', names=True)
            e = np.hypot(a['px'] - a['tx'], a['py'] - a['ty'])
            print(f"{run} lit drag={drag}: final horizontal error {e[-1]:.2f} m, p50 {np.median(e):.2f}, max {e.max():.2f}; rows {len(e)}")
        except Exception as ex:
            print(run, drag, 'lit missing', ex)
    for p in sorted(glob.glob(f'{d}/replay_dk_0_*.csv')):
        a_, b_ = re.findall(r'dk_0_(\d+)_(\d+)', p)[0]
        out = []
        for drag in ['0', '0.106']:
            q = f'{d}/replay_dk_{drag}_{a_}_{b_}.csv'
            lit = np.genfromtxt(f'{d}/replay_lit_drag{drag}.csv', delimiter=',', names=True)
            x = np.genfromtxt(q, delimiter=',', names=True)
            i = min(np.searchsorted(x['stamp_s'], float(b_)), len(x) - 1); j = min(np.searchsorted(lit['stamp_s'], float(b_)), len(lit) - 1)
            drift = np.hypot((x['px'][i] - x['tx'][i]) - (lit['px'][j] - lit['tx'][j]), (x['py'][i] - x['ty'][i]) - (lit['py'][j] - lit['ty'][j]))
            dv = np.hypot(x['vx'][i] - lit['vx'][j], x['vy'][i] - lit['vy'][j])
            out.append(f"drag={drag}: drift {drift:5.2f} m, velocity off {dv:4.2f} m/s")
        print(f"{run} dark {a_}-{b_}: " + ' | '.join(out))
