"""room_churn.py RUN...: inside the shaft's room (x 46..56, y -9..-4): route generations issued, no-route holds,
limiters, and time, from mppi_track.jsonl."""
import json, sys, collections
for run in sys.argv[1:]:
    rows = [json.loads(l) for l in open(f'log/runs/{run}/mppi/mppi_track.jsonl')]
    ins = [r for r in rows if 46 <= r['p'][0] <= 56 and -9 <= r['p'][1] <= -4]
    if not ins:
        print(run, 'never in the room'); continue
    gens = {r['route_generation'] for r in ins if r['route_generation']}
    holds = sum(r['planning_state'] != 'planned' for r in ins)
    lim = collections.Counter(r['limiter'] for r in ins)
    span = (ins[-1]['stamp_ns'] - ins[0]['stamp_ns']) * 1e-9
    spd = sum((r['v'][0]**2 + r['v'][1]**2 + r['v'][2]**2) ** .5 for r in ins) / len(ins)
    print(f"{run}: {span:.1f} s span, {len(ins)} ticks, generations {len(gens)}, hold ticks {holds} ({100*holds/len(ins):.0f} %), "
          f"mean speed {spd:.2f}, limiters {dict(lim.most_common(4))}")
