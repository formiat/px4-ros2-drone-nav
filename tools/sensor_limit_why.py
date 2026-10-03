"""sensor_limit_why.py RUN...: ticks whose sensor braking limit is low (< 1.5 m/s) with the measured range high (>= 6 m): which inputs differ."""
import re, sys, collections
for run in sys.argv[1:]:
    log = open(f'log/runs/{run}/ros_drone_nav.log', errors='ignore').read()
    t0 = float(re.search(r'\[(\d+\.\d+)\] .*MISSION_READINESS ready=true', log).group(1))
    rows = []
    for m in re.finditer(r'\[(\d+\.\d+)\] \[production_mppi_node\]: PRODUCTION_MPPI_TICK ([^\n]+)', log):
        t = float(m.group(1))
        if t < t0:
            continue
        d = dict(re.findall(r'([a-zA-Z0-9_]+)=([^\s]+)', m.group(2)))
        try:
            r = float(d['sensor_measured_range_m']); l = float(d['sensor_braking_speed_limit_mps'])
        except (KeyError, ValueError):
            continue
        if r >= 6.0 and l < 1.5:
            rows.append((t - t0, d))
    print(f'{run}: {len(rows)} ticks')
    keys = ['sensor_evidence_age_s', 'sensor_braking_guaranteed_detection_range_m', 'sensor_braking_total_latency_s',
            'sensor_braking_reserve_m', 'sensor_braking_assessed_speed_mps', 'unfaced_observed_range_m', 'planning_state']
    for k in keys:
        vals = [d.get(k, '?') for _, d in rows]
        c = collections.Counter(v if not v.replace('.', '').replace('-', '').isdigit() else round(float(v), 1) for v in vals)
        print('   ', k, c.most_common(6))
    print('    times', [round(t) for t, _ in rows[::max(1, len(rows)//15)]])
