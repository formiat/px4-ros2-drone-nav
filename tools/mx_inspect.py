"""mx_inspect.py RUN...: what the item-17 acceptance asks of a flight: FAILs, speed, truth, return trigger and moment,
light judgments, contacts and landings, the vehicle's least distance to each zone's centre and the least light share it
flew in, gaze overshoots, tick wall time, the real-time factor and the resource record's largest gap."""
import csv, json, math, re, subprocess, sys
for run in sys.argv[1:]:
    d = f'log/runs/{run}'
    val = open(f'log/tools/run_{run}.log', errors='ignore').read()
    ros = open(f'{d}/ros_drone_nav.log', errors='ignore').read()
    m = json.load(open(f'{d}/manifest.json'))
    o = m.get('effective_overrides', {})
    print(f"== {run} commit {m['repository']['commit'][:8]} zones={o.get('ANOMALY_ZONES')} faults={o.get('LIGHT_FAULTS')} "
          f"stream={o.get('STREAM_FAULTS')} goals={o.get('MISSION_GOALS_XYZ_M')} battery={o.get('LIGHT_BATTERY_S')}")
    for l in val.splitlines():
        if l.startswith('FAIL') or re.match(r'^(OK|NOTE): (mean flight|the true position|the goal was given up|the vehicle returned|the vehicle is whole|production tick wall time|real-time factor|no crash|crash)', l):
            print('  ', l[:170])
    for pat in ['GOAL_UNREACHABLE trigger', 'GOAL_UNREACHABLE_HELD', 'START_UNREACHABLE', 'CARRIED_LIGHT_JUDGMENT unreliable',
                'VEHICLE_LANDED', 'VEHICLE_DESTROYED', 'LIGHT_BATTERY']:
        hits = [l for l in ros.splitlines() if pat in l]
        if hits:
            print(f'   {pat}: {len(hits)}x; first: ' + re.sub(r'^.*?\]: ', '', hits[0])[:190])
    pose = [(float(r[0]), float(r[1]), float(r[2]), float(r[3])) for r in csv.reader(open(f'{d}/gz_pose.csv'))]
    for z in filter(None, (o.get('ANOMALY_ZONES') or '').split(';')):
        zx, zy, zz, core, fall = map(float, z.split(','))
        dmin = min(math.dist((x, y, zv), (zx, zy, zz)) for _, x, y, zv in pose)
        print(f'   zone ({zx:g},{zy:g},{zz:g}) core {core:g} falloff {fall:g}: least distance {dmin:.2f} m')
    try:
        shares = [float(r['share']) for r in csv.DictReader(open(f'{d}/carried_light.csv'))]
        print(f'   light share least {min(shares):.3f}; time under 0.35: {sum(s < 0.35 for s in shares) / max(1, len(shares)) * 100:.1f} % of samples')
    except FileNotFoundError:
        pass
    try:
        out = subprocess.run(['python3', 'tools/yaw_overshoot.py', run], capture_output=True, text=True).stdout.strip()
        print('  ', out)
    except Exception as e:
        print('   overshoot n/a', e)
    stamps = sorted({float(r['stamp_s']) for r in csv.DictReader(open(f'{d}/resources.csv'))})
    gap = max((b - a for a, b in zip(stamps, stamps[1:])), default=0)
    print(f'   resource record largest gap {gap:.1f} s')
