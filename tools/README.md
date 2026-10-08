# Flight Tools

The scripts the acceptance flights are launched and read with. They are
evaluation tools: nothing here is built into the stack or run by a gate, and
none of them is a criterion (the mission check in
[`docs/testing.md`](../docs/testing.md) is). Run them from the repository
root on the host unless a line says "inside the container". What they write
(run logs, flags, series tables, recordings, replay binaries) goes to
`log/tools/`, which git ignores.

`log/tools/` is pruned like every log (`scripts/prune_sim_logs.sh`, a week):
an entry directly inside it goes when nothing in it was touched for seven
days. A directory holding a `.keep` file at its top is never deleted, and
that is how anything worth more than a week is kept:

| Kept directory | What it holds |
|---|---|
| `log/tools/journal/` | the goal journals (`goal_journal_*.md`) and the series tables |
| `log/tools/diagnostics/` | one-off scripts and data of single investigations |
| `log/tools/vio/`, `log/tools/replay/` | recorded flights for the offline replay (r575, r576; r779, r795) and the replay binaries |
| `log/tools/truth25/`, `log/tools/proof_leak/`, `log/tools/texture/`, `log/tools/stereo/` | truth grids and measurement data the documents cite |

A loose file cannot be kept: check logs (`run_rNNN.log`), gate logs and
flags go after a week. A new recording or result that has to stay goes into
a kept directory, or into a new directory with its own `.keep`.

Runs are named `rNNN`; a flight's artifacts are in `log/runs/rNNN/` and its
mission check's output in `log/tools/run_rNNN.log`.

## Launching

| Tool | What it does | Use |
|---|---|---|
| `gates.sh` | The whole pre-commit gate in the dev container, detached: format, build, unit tests, script tests, quality. Verdict in `log/tools/gate_flag.txt` (`X=0` is green), output in `log/tools/gate_all.log`. | `./tools/gates.sh`, then poll the flag. Do not edit tracked files while it runs. |
| `series2.sh` | Acceptance flights one at a time on the current commit. Prunes logs older than a week, waits until the host has been quiet for 60 s, stops any simulation before and after, appends the check's key lines to `log/tools/journal/series_<commit>_<profile>.txt`. | `./tools/series2.sh r950 r951` (cameras); `./tools/series2.sh --lidar r954` (`--lidar` first); another scenario with `TARGET=sim-urban-light-lost-headless SMOKE_DURATION_S=400 ./tools/series2.sh r939`. |
| `host_verdict.py` | Whether a flight counts, by the host it was flown on (specification A7): `HOST valid` or `HOST VOID` with the reason, from the real-time factor at the median (0.82 on the stereo set, 0.97 on the lidar, of the factor the run asked for), the resource sampler's largest gap (2 s) and the age of the estimator's poses on their way to the autopilot (300 ms at the 95th percentile). The series launcher appends it to the series table after every flight. | `python3 tools/host_verdict.py r981` |
| `rtf_watchdog.py` | Whether a flight is being flown under a load that will void it, read while it flies: the median of the simulator's real-time factor over the last 30 sampled seconds (after the flight has got going) against the verdict's floor of the factor the run asked for. `WATCHDOG ok`, `slow` (exit 1) or `waiting`. | `python3 tools/rtf_watchdog.py r1098 --lidar --factor 1.0` |
| `fly_until_valid.sh` | One flight that counts, on a host another task may load: a `series2.sh` flight watched every 20 s by the watchdog, stopped at once when slowed, judged by `host_verdict.py` after, and flown again under the next number when stopped or void, at most the stated number of times. A flight that counts is the result whatever its check says. `TARGET`, `SMOKE_DURATION_S` and `REAL_TIME_FACTOR` pass through. Lines in `log/tools/fly_until_valid.txt` (`VALID`, `VOID reason`, `WATCHDOG reason`), then `END`. | `./tools/fly_until_valid.sh --lidar 1130 4`; `TARGET=sim-urban-low-battery-headless SMOKE_DURATION_S=1800 ./tools/fly_until_valid.sh 1140 4` |
| `mx.sh` | One flight of roadmap item 17's acceptance matrix by line number (1 cameras, 2 lidar, 3 long failures, 4 light lost, 5 severe failure, 7 low battery, 8 goal outside). Refuses to start while a flight runs or the run exists; marks its end in `log/tools/mx_flag.txt`. | `./tools/mx.sh 5 r944 &` |
| `mxw.sh` | Waits up to 9.5 minutes for that flight; prints `DONE` or `RUNNING`. Never stops a flight. | `./tools/mxw.sh r944` |

Flights run strictly one at a time. A flight `host_verdict.py` calls void is
flown again, and a crash in it is not a defect. On a host that another task
loads from time to time, every flight goes through `fly_until_valid.sh` or
`record_until_pass.sh`: the load is noticed while the flight flies, the flight
is stopped, and it is flown again when the host is quiet; the foreign
processes are never touched (specification A4).

## Recording

| Tool | What it does | Use |
|---|---|---|
| `record_flight.sh` | One flight of a named scenario (`point-to-point`, `light-lost`, `light-failure`, `low-battery`, `long-failures`, `return-home`; the stereo set only) flown as its headless flight with the Gazebo window and RViz open, at a real-time factor of 0.6 unless `REAL_TIME_FACTOR` says otherwise, and recorded: two split videos in the directory, the world on the left and RViz on the right, third-person and top-down, each the whole flight at the flight's own pace. A recording whose window stood still, whose half was a slideshow while the vehicle moved, that is off the flight's clock by more than a second or shorter than a minute is `BAD`. The desktop is kept from going idle meanwhile. The recorder's output is `log/tools/record_RUN.log`. | `./tools/record_flight.sh light-lost r1020 log/videos/2026-10-04` |
| `record_batch.sh` | Every scenario of roadmap item 17 on the stereo set round robin, all once, then all again, for the stated number of passes; a flight that failed or whose recording is not one is flown again at the end of its pass. Videos, `index.md` and `batch.log` in the directory, which it keeps from the pruning. | `./tools/record_batch.sh log/videos/2026-10-04 3 1020 &` |
| `record_index.py` | The index row of one recorded flight (the batch calls it). | `python3 tools/record_index.py light-lost r1020 log/videos/2026-10-04` |
| `record_until_pass.sh` | Records one scenario until a flight passes its check with a recording that is one, at most the stated number of times; a flight that stands still for a minute of simulation (the long flight under failures does that below real time, the register's P10), one the watchdog finds slowed by the host's load, or one the verdict does not count is stopped and flown again. One line a flight in `log/tools/record_until_pass.txt` (`PASS`, `STALLED`, `WATCHDOG`, `VOID`, `FAILED`), then `END`. | `./tools/record_until_pass.sh long-failures 1121 5 log/videos/2026-10-05 &` |
| `video_check.py` | A finished split video with no run beside it: its length, the different frames a second of each half in windows of 5 s (a slideshow shows about one, a smooth recording 19 to 24), and the sampled frames in which the world half holds the green of Gazebo's light gizmo. `VIDEO ok` or `VIDEO BAD` with the reasons. | `python3 tools/video_check.py log/videos/2026-10-05/*_follow.mp4` |
| `window_rig.sh` | The Gazebo window of a recording over a small test world, with no flight, inside the container: `rate SECONDS [NAME=value ...]` prints the window's redraws a second (24 is sound, 1 is a window waiting for a blank screen; the assignments try a remedy in its environment), `frame [SDF]` counts the bright green pixels of one frame (the gizmo of a light the window is told to visualize; the SDF goes inside the light). Both causes of the first batch's defects were found with it. | `bash /workspace/tools/window_rig.sh rate 20` |

A recorded flight the host's verdict counts is a flight like any other
(specification A11); a row that says `HOST VOID` is a demonstration. Lidar
flights are not recorded: the location is dark and the lidar carries no
light, so the world half of the picture shows nothing. A slowed flight of
any kind is `REAL_TIME_FACTOR=0.5 ./tools/series2.sh r1100`.

## Reading A Flight

| Tool | What it prints | Use |
|---|---|---|
| `mx_inspect.py` | Everything the acceptance asks of one flight: failing lines, speed, truth at the goal, the return's trigger and moment, light judgments, the least light flown in, gaze overshoots, tick time, real-time factor. | `python3 tools/mx_inspect.py r949 r950` |
| `mxs.sh` | The same, cut to the acceptance lines, with the count of failing lines. | `./tools/mxs.sh r949` |
| `homeway.py` | The way home of a returned flight: when and where the goal was given up, seconds and metres home, trail points, revocations. | `python3 tools/homeway.py r974` |
| `lost_inspect.py` | A light-lost flight by phase: dead reckoning, judgment, landing or contact, truth at each, the drift between. | `python3 tools/lost_inspect.py r939` |
| `crash_context.py` | State, readiness, executor and offboard lines in the seconds before `VEHICLE_DESTROYED`. | `python3 tools/crash_context.py r932 8` |
| `body_truth_clearance.py` | The true body's clearance to the true location over a flight (needs the 0.25 m truth grid in `log/tools/truth25`). | `python3 tools/body_truth_clearance.py r958` |
| `yaw_overshoot.py` | Gaze overshoots: the heading swinging more than 45 degrees past its target. | `python3 tools/yaw_overshoot.py r949` |
| `room_time.py`, `room_churn.py` | Seconds a flight spends in the shaft room at (53, -7), and the routes, holds and limiters inside it. Location-specific by design: they measure one register entry. | `python3 tools/room_time.py r958` |
| `sensor_range_compare.py`, `sensor_limit_why.py` | The braking contract's measured range and the speed it allowed; the ticks where the limit is low with the range high. | `python3 tools/sensor_range_compare.py r949` |

## Replaying The Visual-Inertial Estimator Offline (`vio/`)

A recorded flight (stereo frames, IMU, truth) replayed through the
repository's filter with no simulator: a change of the estimator is measured
on recordings before any flight.

| Tool | What it does | Use |
|---|---|---|
| `vio/record_flight.sh` | One flight with the recorders attached; the record lands in `log/tools/vio/RUN/`. Needs `build_recorder.sh` once. | `./tools/vio/record_flight.sh r575` |
| `vio/build_recorder.sh`, `vio/record_frames.cpp`, `vio/record_vio.py`, `vio/trim_record.py`, `vio/truth_from_run.py` | The recorder's parts: stereo frames from Gazebo transport, IMU and estimates from ROS, the trim to the flight, the truth from the run's pose capture. | inside the container: `bash /workspace/tools/vio/build_recorder.sh` |
| `vio/build_replay_src.sh`, `vio/replay_vio.cpp` | Builds the replay against the tree's `visual_inertial_odometry.cpp` and a tracker source. Options are `key=value`: `dark_from_s`, `dark_to_s` (frames blanked), `baro=1`, `drag=<1/s>`, `tag=<name>`; output `replay_<tag>.csv` beside the record. | inside the container: `bash /workspace/tools/vio/build_replay_src.sh /workspace/drone_city_nav/src/stereo_feature_tracker.cpp replay_vio` |
| `vio/drag_bench.sh`, `vio/drag_report.py` | Dark-window replays of two recorded flights with and without the rotor drag fusion (specification K21), and the drift table. | inside the container the bench, then `python3 tools/vio/drag_report.py` |
| `vio/dark_drift.py` | Position error of dark-window replays against the lit replay. | `python3 tools/vio/dark_drift.py log/tools/replay/r779` |

One-off diagnostics written for a single investigation stay in
`log/tools/diagnostics/` and are not tracked; the goal journals in
`log/tools/journal/` say which was used for what.
