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
    def all_at(pat):
        return [float(m.group(1)) for m in re.finditer(r'\[(\d+\.\d+)\] [^\n]*' + pat, log)]
    def truth(t):
        i = np.argmin(abs(g[:, 4] - t)); return g[i, 1:4]
    t0 = at('MISSION_READINESS ready=true')
    # The dead reckoning that ended in the landing: a short one on the pad before the take-off
    # is not it (r1345: the first one, at -10 s, read 24 m of drift).
    end_t = at('VEHICLE_LANDED') or at(r'collision_crash_node\]: VEHICLE_DESTROYED')
    starts = [t for t in all_at('DEAD_RECKONING started=true') if end_t is None or t <= end_t]
    descents = [t for t in all_at('DEAD_RECKONING_LANDING descending=true') if not starts or t >= starts[-1]]
    ev = [('dr', starts[-1] if starts else None), ('judged', at('GOAL_UNREACHABLE trigger')),
          ('land_cmd', descents[0] if descents else None), ('landed', at('VEHICLE_LANDED')),
          ('destroyed', at(r'collision_crash_node\]: VEHICLE_DESTROYED'))]
    print(f'== {run}')
    for name, t in ev:
        if t is not None:
            p = truth(t); print(f'   {name:9s} t={t - t0:6.1f} truth=({p[0]:.2f},{p[1]:.2f},{p[2]:.2f})')
    dr = dict(ev)['dr']; end = dict(ev)['landed'] or dict(ev)['destroyed']
    if dr and end:
        d = truth(end) - truth(dr); print(f'   drift dr->end {np.hypot(d[0], d[1]):.2f} m horizontal in {end - dr:.1f} s')
    n_land = len(re.findall('DEAD_RECKONING_LANDING descending=true', log))
    m = re.search(r'VEHICLE_LANDED[^\n]*', log)
    print(f'   landing commands {n_land}; ' + (m.group(0)[:230] if m else 'no VEHICLE_LANDED'))
    val = open(f'log/tools/run_{run}.log', errors='ignore').read()
    for l in val.splitlines():
        if l.startswith('FAIL') or re.match(r'^OK: (no crash|the vehicle is whole)', l):
            print('  ', l[:160])
