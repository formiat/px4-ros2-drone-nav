"""sensor_range_compare.py RUN...: the sensor contract's measured range and the speed it allowed over a flight's ready..end ticks."""
import re, sys
for run in sys.argv[1:]:
    log = open(f'log/runs/{run}/ros_drone_nav.log', errors='ignore').read()
    t0 = float(re.search(r'\[(\d+\.\d+)\] .*MISSION_READINESS ready=true', log).group(1))
    t1 = float(re.search(r'\[(\d+\.\d+)\] .*MISSION_RESULT success=true', log).group(1))
    rng, lim, sb = [], [], 0
    for m in re.finditer(r'\[(\d+\.\d+)\] \[production_mppi_node\]: PRODUCTION_MPPI_TICK ([^\n]+)', log):
        t = float(m.group(1))
        if not t0 <= t <= t1:
            continue
        d = dict(re.findall(r'([a-zA-Z0-9_]+)=([^\s]+)', m.group(2)))
        r = float(d.get('sensor_measured_range_m', 'nan'))
        if r == r and r > 0:
            rng.append(r)
            lim.append(float(d['sensor_braking_speed_limit_mps']))
        sb += d.get('active_speed_limiter') == 'sensor_braking'
    rng.sort(); lim.sort(); k = len(rng)
    q = lambda a, p: a[int(p * (len(a) - 1))]
    print(f'{run}: ticks {k} range p10 {q(rng,.1):.2f} p25 {q(rng,.25):.2f} p50 {q(rng,.5):.2f} p75 {q(rng,.75):.2f}; '
          f'share <6.0: {100*sum(x<6.0 for x in rng)/k:.0f}% <4: {100*sum(x<4 for x in rng)/k:.0f}%; '
          f'limit p25 {q(lim,.25):.2f} p50 {q(lim,.5):.2f}; sensor_braking active {100*sb/k:.0f}%')
