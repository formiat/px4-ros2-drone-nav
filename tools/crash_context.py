"""crash_context.py RUN [before_s]: state, readiness, executor events and offboard lines before VEHICLE_DESTROYED."""
import re, sys
run = sys.argv[1]; before = float(sys.argv[2]) if len(sys.argv) > 2 else 8.0
log = open(f'log/runs/{run}/ros_drone_nav.log', errors='ignore').read()
m = re.search(r'\[(\d+\.\d+)\] \[collision_crash_node\]: VEHICLE_DESTROYED ([^\n]*)', log)
tc = float(m.group(1)); print('CRASH', m.group(2)[:400])
t0 = float(re.search(r'\[(\d+\.\d+)\] \[production_mppi_node\]: PRODUCTION_MPPI_TICK', log).group(1))
print(f'crash at t={tc - t0:.1f}s after first tick')
print('--- ticks')
for mm in re.finditer(r'\[(\d+\.\d+)\] \[production_mppi_node\]: PRODUCTION_MPPI_TICK ([^\n]+)', log):
    t = float(mm.group(1))
    if tc - before <= t <= tc + 0.3:
        d = dict(re.findall(r'([a-zA-Z0-9_]+)=([^\s]+)', mm.group(2)))
        v = [float(x) for x in d['state_velocity'].strip('()').split(',')]
        print(f'  {t - tc:+5.1f}s v={(v[0]**2+v[1]**2+v[2]**2)**0.5:4.2f} vel={d["state_velocity"]} ref={float(d.get("reference_speed_mps", 0)):4.2f} pos={d.get("state_position")} lim={d.get("active_speed_limiter")} mode={d.get("execution_mode")} reason={d.get("execution_reason")} unk={d.get("route_unknown_exposure")}')
print('--- readiness')
for mm in re.finditer(r'\[(\d+\.\d+)\] \[production_mppi_node\]: OBSERVED_FOOTPRINT_READINESS ([^\n]*)', log):
    t = float(mm.group(1))
    if tc - before <= t <= tc + 0.5:
        d = dict(re.findall(r'([a-zA-Z0-9_]+)=([^\s]+)', mm.group(2))); print(f'  {t - tc:+5.1f}s status={d.get("status")} strict={d.get("strict_status")} pos={d.get("position")} failure={d.get("failure_point")}')
print('--- executor events')
for mm in re.finditer(r'\[(\d+\.\d+)\] \[production_mppi_node\]: ((?:TRAJECTORY_COLLISION_CELLS|TRAJECTORY_COLLISION_CANDIDATE|TRAJECTORY_COLLISION_REPLAN|EXECUTION_HORIZON_COMMIT|STOP_EXECUTION|EXECUTION_HOLD|EXECUTION_RETENTION|EXECUTION_HORIZON |ROUTE_GEOMETRY|STALLED)[^\n]*)', log):
    t = float(mm.group(1))
    if tc - before <= t <= tc + 0.2: print(f'  {t - tc:+5.2f}s {mm.group(2)[:260]}')
print('--- offboard')
for mm in re.finditer(r'\[(\d+\.\d+)\] \[mppi_offboard_node\]: ([^\n]*)', log):
    t = float(mm.group(1))
    if tc - before <= t <= tc: print(f'  {t - tc:+5.2f}s {mm.group(2)[:200]}')
# What the stack commanded, what the airframe did and what mode the autopilot was in, on the simulation clock: a pose
# change before a contact is then a commanded motion, a lost mode or something else. The idea is naipi11's
# crash_window.py (c651926d, docs/forks.md), which reads the stack's bag; here it is read from the records every
# headless flight keeps: tracking.npz (the setpoints the autopilot was sent and its local position, north-east-down,
# with their wall stamps), gz_pose.csv (the true pose on both clocks) and pilot.csv (the autopilot's mode and arming).
import csv, os
import numpy as np
directory = f'log/runs/{run}'
if os.path.exists(f'{directory}/tracking.npz') and os.path.exists(f'{directory}/gz_pose.csv'):
    pose = np.array([[float(r[0]), float(r[1]), float(r[2]), float(r[3]), float(r[8])]
                     for r in csv.reader(open(f'{directory}/gz_pose.csv')) if len(r) >= 9])
    crash_s = float(np.interp(tc, pose[:, 4], pose[:, 0]))
    track = np.load(f'{directory}/tracking.npz')
    sp, lp = track['sp'], track['lp']
    sp_s = np.interp(track['sp_received_s'], pose[:, 4], pose[:, 0])
    lp_s = np.interp(track['lp_received_s'], pose[:, 4], pose[:, 0])
    print(f'--- commanded and flown (simulation clock; the contact at {crash_s:.2f} s)')
    print('    t        truth x,y,z            true speed  | local position x,y,z (NED), down speed '
          '| setpoint position x,y,z; velocity x,y,z')
    for t in np.arange(crash_s - before, crash_s + 0.01, 0.5):
        i = min(np.searchsorted(pose[:, 0], t), len(pose) - 1); j = max(0, i - 12)
        speed = np.linalg.norm(pose[i, 1:4] - pose[j, 1:4]) / max(1e-3, pose[i, 0] - pose[j, 0])
        a = min(np.searchsorted(lp_s, t), len(lp) - 1); b = min(np.searchsorted(sp_s, t), len(sp) - 1)
        print(f'  {t - crash_s:+5.1f}s ({pose[i, 1]:7.2f},{pose[i, 2]:7.2f},{pose[i, 3]:6.2f}) {speed:5.2f} m/s '
              f'| ({lp[a, 2]:7.2f},{lp[a, 3]:7.2f},{lp[a, 4]:6.2f}) {lp[a, 7]:+5.2f} '
              f'| ({sp[b, 1]:7.2f},{sp[b, 2]:7.2f},{sp[b, 3]:6.2f}); ({sp[b, 4]:5.2f},{sp[b, 5]:5.2f},{sp[b, 6]:5.2f})')
    if os.path.exists(f'{directory}/pilot.csv'):
        print('--- the autopilot (pilot.csv)')
        for row in csv.DictReader(open(f'{directory}/pilot.csv')):
            if float(row['clock_s']) >= crash_s - before - 30.0:
                print(f"  {float(row['clock_s']) - crash_s:+7.1f}s {row['what']} {row['detail']}")
