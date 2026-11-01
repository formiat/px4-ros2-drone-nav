"""gnss_gates.py RUN...: the distribution of the trust law's readings over the honest flights (the GNSS_TRUST lines
of the estimator, one a second), and the gates the law gets from them: p99 x 2 of every reading (Р1.3)."""
import re, sys
PATTERN = re.compile(r"GNSS_TRUST trusted=(true|false) agreeing=(true|false) agreeing_s=([0-9.]+) reason=(\w+) residual_m=([0-9.]+) normalised=([0-9.]+) velocity_mps=([0-9.]+) velocity_normalised=([0-9.]+) self_mps=([0-9.]+) step_m=([0-9.]+) clock_s=([-0-9.]+) fix_type=(\d+) eph=([0-9.]+) fused=(\d+)")
names = {"residual_m": 4, "normalised": 5, "velocity_mps": 6, "velocity_normalised": 7, "self_mps": 8, "step_m": 9, "clock_s": 10}
series = {name: [] for name in names}
per_run = {}
for run in sys.argv[1:]:
    text = open(f"log/runs/{run}/ros_drone_nav.log", errors="replace").read()
    rows = PATTERN.findall(text)
    per_run[run] = len(rows)
    for row in rows:
        for name, index in names.items():
            series[name].append(float(row[index]))
def q(values, share):
    values = sorted(values); return values[min(len(values) - 1, int(share * (len(values) - 1)))]
print("lines per run:", per_run)
print(f"{'reading':22s} {'p50':>8s} {'p90':>8s} {'p99':>8s} {'max':>8s} {'gate p99x2':>11s}")
for name in names:
    v = series[name]
    if not v: continue
    print(f"{name:22s} {q(v,0.5):8.3f} {q(v,0.9):8.3f} {q(v,0.99):8.3f} {max(v):8.3f} {2*q(v,0.99):11.3f}")
