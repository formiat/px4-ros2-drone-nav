"""lost_inspect.py RUN...: a light-lost flight by phase: dead reckoning start, the landing command, the landing or contact,
truth at each, the drift between the dead reckoning's start and the touchdown, and the mission check's verdict lines."""
import csv, re, sys
import numpy as np
for run in sys.argv[1:]:
    log = open(f'log/runs/{run}/ros_drone_nav.log', errors='ignore').read()
    g = np.array([[float(r[0]), float(r[1]), float(r[2]), float(r[3]), float(r[8])] for r in csv.reader(open(f'log/runs/{run}/gz_pose.csv'))])
    def at(pat):
        m = re.search(r'\[(\d+\.\d+)\] [^\n]*' + pat, log)
        return float(m.group(1)) if m else None
    def truth(t):
        i = np.argmin(abs(g[:, 4] - t)); return g[i, 1:4]
    t0 = at('MISSION_READINESS ready=true')
    ev = [('dr', at('DEAD_RECKONING started=true')), ('judged', at('GOAL_UNREACHABLE trigger')),
          ('land_cmd', at('DEAD_RECKONING_LANDING commanded=true')), ('landed', at('VEHICLE_LANDED')),
          ('destroyed', at(r'collision_crash_node\]: VEHICLE_DESTROYED'))]
    print(f'== {run}')
    for name, t in ev:
        if t is not None:
            p = truth(t); print(f'   {name:9s} t={t - t0:6.1f} truth=({p[0]:.2f},{p[1]:.2f},{p[2]:.2f})')
    dr = dict(ev)['dr']; end = dict(ev)['landed'] or dict(ev)['destroyed']
    if dr and end:
        d = truth(end) - truth(dr); print(f'   drift dr->end {np.hypot(d[0], d[1]):.2f} m horizontal in {end - dr:.1f} s')
    n_land = len(re.findall('DEAD_RECKONING_LANDING commanded=true', log))
    m = re.search(r'VEHICLE_LANDED[^\n]*', log)
    print(f'   landing commands {n_land}; ' + (m.group(0)[:230] if m else 'no VEHICLE_LANDED'))
    val = open(f'log/tools/run_{run}.log', errors='ignore').read()
    for l in val.splitlines():
        if l.startswith('FAIL') or re.match(r'^OK: (no crash|the vehicle is whole)', l):
            print('  ', l[:160])
