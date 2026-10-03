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
