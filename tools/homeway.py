"""homeway.py RUN...: the way home of a returned flight: seconds of simulation and metres from the give-up to the
result, the trail objectives published and the revocations on the way, beside the check's lines."""
import csv, re, sys
import numpy as np
for run in sys.argv[1:]:
    log = open(f'log/runs/{run}/ros_drone_nav.log', errors='ignore').read()
    t0 = float(re.search(r'\[(\d+\.\d+)\] .*MISSION_READINESS ready=true', log).group(1))
    j = re.search(r'\[(\d+\.\d+)\] .*GOAL_UNREACHABLE trigger=(\w+)[^\n]*elapsed_s=([\d.]+) flown_path_m=([\d.]+)', log)
    e = re.search(r'\[(\d+\.\d+)\] .*MISSION_RESULT[^\n]*reason=\'(\w+)\'', log)
    val = open(f'log/tools/run_{run}.log', errors='ignore').read()
    lines = [l[:120] for l in val.splitlines() if l.startswith('FAIL') or re.match(r'^OK: (the true position|no crash)', l)]
    if not j:
        print(f'{run}: no give-up; ' + ' | '.join(lines)); continue
    g = np.array([[float(r[0]), float(r[1]), float(r[2]), float(r[3]), float(r[8])] for r in csv.reader(open(f'log/runs/{run}/gz_pose.csv'))])
    tj = float(j.group(1)); te = float(e.group(1)) if e else g[-1, 4]
    i0 = np.argmin(abs(g[:, 4] - tj)); i1 = np.argmin(abs(g[:, 4] - te))
    path = np.sum(np.linalg.norm(np.diff(g[i0:i1:10, 1:4], axis=0), axis=1)); dt = g[i1, 0] - g[i0, 0]
    n = len(re.findall('RETURN_TRAIL', log)); rev = sum(1 for m in re.finditer(r'\[(\d+\.\d+)\] .*EXECUTION_REVOCATION', log) if float(m.group(1)) > tj)
    print(f"{run}: {j.group(2)} at {float(j.group(3)):.0f} s ({float(j.group(4)):.0f} m); home {dt:.0f} s, {path:.0f} m, {path / max(dt, 1):.2f} m/s, {n} trail points, {rev} revocations; {e.group(2) if e else 'no result'}; " + ' | '.join(lines))
