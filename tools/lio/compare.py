"""compare.py RUN_DIR REPLAY_CSV [T0 T1]: horizontal error of a lidar-inertial replay (replay_lio output, NED from
the spawn) against the flight's true pose (RUN_DIR/gz_pose.csv, the map frame; its first sample is the spawn), with
the mission check's own measurement; prints the error over the flight in ten-second windows. Run from the repository
root."""
import sys
sys.path.insert(0, "scripts")
import numpy as np
from pathlib import Path
from controller_dynamics_evidence import position_estimate_error, load_truth_csv
run = Path(sys.argv[1]); r = np.loadtxt(sys.argv[2], delimiter=",")
r = r[r[:, 4] == 1]
r = r[np.concatenate([[True], np.diff(r[:, 0]) > 0])]
spawn = load_truth_csv(run / "gz_pose.csv")[0]
t = r[:, 0] * 1e-9; x = r[:, 2] + spawn[1]; y = r[:, 1] + spawn[2]; z = -r[:, 3] + spawn[3]
if len(sys.argv) > 4:
    m = (t >= float(sys.argv[3])) & (t <= float(sys.argv[4])); t, x, y, z = t[m], x[m], y[m], z[m]
speed = np.hypot(np.gradient(x, t), np.gradient(y, t))
truth = load_truth_csv(run / "gz_pose.csv")
truth = truth[(truth[:, 0] >= t[0] - 5.0) & (truth[:, 0] <= t[-1] + 5.0)]
e = position_estimate_error(np.column_stack([t, x, y, speed]), truth)
print("replay cross-track p95 %.3f m, along-track %+.3f s, total p95 %.3f m (%d samples)" % (e.cross_track_p95_m, e.along_track_offset_s, e.total_p95_m, e.samples))
gx = np.interp(t, truth[:, 0], truth[:, 1]); gy = np.interp(t, truth[:, 0], truth[:, 2])
err = np.hypot(x - gx, y - gy)
for a in np.arange(t[0], t[-1], 10.0):
    w = (t >= a) & (t < a + 10)
    if w.any(): print("  t %5.0f truth (%6.1f,%6.1f) horizontal error max %.2f end %.2f" % (a, gx[w][-1], gy[w][-1], err[w].max(), err[w][-1]))
