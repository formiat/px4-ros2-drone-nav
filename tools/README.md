# Flight Tools

The scripts the acceptance flights are launched and read with. They are
evaluation tools: nothing here is built into the stack or run by a gate, and
none of them is a criterion (the mission check in
[`docs/testing.md`](../docs/testing.md) is). Run them from the repository
root on the host unless a line says "inside the container". What they write
(run logs, flags, series tables, recordings, replay binaries) goes to
`log/tools/`, which git ignores.

**A script a tracked document cites lives here, under git** (C++ built
against the package in `drone_city_nav/tools/`), with a line on this page:
a figure in a document has to be reproducible from the repository.
`log/tools/` holds drafts, data and what the tools write, never the code a
document points at; a draft that a document comes to cite moves here first
(the owner, 2026-10-13). `scripts/tests/test_tools_tracked.py` holds every
tool to a line on this page and to parsing.

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
| `log/tools/part1/`, `log/tools/part2/` | the smoke-and-light goal: the smoke rig's levels (`final/`; the light's calibration in part 1's, the rest at the pair's final noise in part 2's) and the estimator's replays over the rig's records (`vio/`, their frames removed) the smoke documents cite |
| `log/tools/part3/`, `log/tools/part6/`, `log/tools/part8/`, `log/tools/part11/`, `log/tools/stall/`, `log/tools/trials/` | the technical-debt goal's and the owner's trials' drafts and data; the scripts the documents cite from them moved here (`blind_phase.py`, `k27_side.py`, `speed_limiters.py`, `airframe_size.py`, `drone_city_nav/tools/mppi_cpu_rollout_benchmark.cpp`) |

A loose file cannot be kept: check logs (`run_rNNN.log`), gate logs and
flags go after a week. A new recording or result that has to stay goes into
a kept directory, or into a new directory with its own `.keep`.

Runs are named `rNNN`; a flight's artifacts are in `log/runs/rNNN/` and its
mission check's output in `log/tools/run_rNNN.log`.

## Launching

| Tool | What it does | Use |
|---|---|---|
| `gates.sh` | The whole pre-commit gate in the dev container, detached: format, build, unit tests, script tests, quality. Verdict in `log/tools/gate_flag.txt` (`X=0` is green), output in `log/tools/gate_all.log`. `--docs`: for a change to Markdown files alone, the script tests only (the ones that read the documents), about a minute; refused with `X=1` when anything else differs from HEAD. | `./tools/gates.sh`, then poll the flag; `./tools/gates.sh --docs` for a documentation-only commit. Do not edit tracked files while it runs. |
| `series2.sh` | Acceptance flights one at a time on the current commit, on the flight's core set (`FLIGHT_CPUSET`, `0-11` of this host unless set; empty for the whole host; specification A4). Prunes logs older than a week, waits until the host has been quiet for 60 s, stops any simulation before and after, appends the check's key lines to `log/tools/journal/series_<commit>_<profile>.txt`. | `./tools/series2.sh r950 r951` (cameras); `./tools/series2.sh --lidar r954` (`--lidar` first); another scenario with `TARGET=sim-urban-light-lost-headless SMOKE_DURATION_S=400 ./tools/series2.sh r939`. |
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
| `record_flight.sh` | One flight of a named scenario (`point-to-point`, `light-lost`, `light-failure`, `low-battery`; the stereo set only; `long-failures` and `return-home` are set aside by the owner since 2026-10-08 and refused without `FLY_SET_ASIDE=1`, the owner's word) flown as its headless flight with the Gazebo window and RViz open, at a real-time factor of 0.6 unless `REAL_TIME_FACTOR` says otherwise, and recorded: two split videos in the directory, the world on the left and RViz on the right, third-person and top-down, each the whole flight at the flight's own pace. A recording whose window stood still, whose half was a slideshow while the vehicle moved, that is off the flight's clock by more than a second, that shows a frame further than half a second from its moment or that is shorter than a minute is `BAD`. The desktop is kept from going idle meanwhile; the windows open below the others and minimize themselves once they draw (the owner, 2026-10-08), and keep drawing minimized. The recorder's output is `log/tools/record_RUN.log`. | `./tools/record_flight.sh light-lost r1020 log/videos/2026-10-04` |
| `render_flight.sh` (the replay draws the run's smoke zones as boxes and keeps the vehicle's left frame for the follow view's inset, `scripts/render_camera_frames.py`) | One flight of a named scenario (the stereo set; not the set-aside ones; `dark-hall` for roadmap item 29's hall) flown as its ordinary headless flight at a real-time factor of 1.0 with no window, recording what RViz shows (`RENDER_BAG=1`), then its pictures drawn afterwards (`scripts/render_flight_video.py`: the world paused with the vehicle moved through its true poses and its light, the record played into both RViz views) and joined into the two split files a live recording makes, checked by the same rules. Specification A11. | `./tools/render_flight.sh point-to-point 1420 log/videos/2026-10-13` |
| `record_batch.sh` | Every scenario of roadmap item 17 on the stereo set round robin, all once, then all again, for the stated number of passes; a flight that failed or whose recording is not one is flown again at the end of its pass. Videos, `index.md` and `batch.log` in the directory, which it keeps from the pruning. | `./tools/record_batch.sh log/videos/2026-10-04 3 1020 &` |
| `record_index.py` | The index row of one recorded flight (the batch calls it). | `python3 tools/record_index.py light-lost r1020 log/videos/2026-10-04` |
| `record_until_pass.sh` | Records one scenario until a flight passes its check with a recording that is one, at most the stated number of times; a flight that stands still for a minute of simulation (the long flight under failures does that below real time, the register's P10), one the watchdog finds slowed by the host's load, or one the verdict does not count is stopped and flown again. One line a flight in `log/tools/record_until_pass.txt` (`PASS`, `STALLED`, `WATCHDOG`, `VOID`, `FAILED`), then `END`. | `./tools/record_until_pass.sh long-failures 1121 5 log/videos/2026-10-05 &` |
| `video_check.py` | A finished split video with no run beside it: its length, the different frames a second of each half in windows of 5 s (a slideshow shows about one, a smooth recording 19 to 24, the dark hall's world half 10 to 13), and the sampled frames in which the world half holds the green of Gazebo's light gizmo. `VIDEO ok` or `VIDEO BAD` with the reasons. | `python3 tools/video_check.py log/videos/2026-10-05/*_follow.mp4` |
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
| `log_window.py` | The ROS log lines of a run between two simulation times, the simulation time in front (the wall stamps put on the simulation clock through the true pose record); by default the estimator's, the landing's and the mission's events. The other tools import its `events`. | `python3 tools/log_window.py r1459 60 90 DEAD_RECKONING` |
| `body_truth_clearance.py` | The true body's clearance to the true location over a flight (needs the 0.25 m truth grid in `log/tools/truth25`). | `python3 tools/body_truth_clearance.py r958` |
| `room_memory_truth.py` | The memory of a smoky room against the truth (roadmap item 23): the phantoms among the occupied voxels of the room's box (no truth occupancy within 0.5 m), the truth walls the memory observed free, the observed share; from the run's last raw memory snapshot, whose bounds the scenario sets to the room. | `python3 tools/room_memory_truth.py r1660` |
| `smoke_marks_truth.py` | Every region whose marks the depth node laid (the `SMOKE_FACE_MARKS` lines, specification K44) projected into the world by its centroid and rim depth at the true pose and judged against the run's dense smoke boxes (within 1.5 m: true). Roadmap item 18. | `python3 -I tools/smoke_marks_truth.py r1947 r1952` |
| `plume_replay.py` | The depth node's plume confirmation (K44) replayed offline over a run's `PLUME_CANDIDATE` lines under the current settings and named variants, each laid frame judged against the dense boxes: the first true laid frame and the false ones a run. Roadmap item 18. | `python3 -I tools/plume_replay.py r1957 r1959 g15=grow_min:0.15` |
| `smoke_series.sh` | One acceptance group of the smoke detector (K44): five counted flights of `blocked`, `ordinary`, `clear` or `lidar` through `fly_until_valid.sh`, the verdict lines gathered in log/tools/smoke_detector/part3/series.txt. Roadmap item 18. Not run since 2026-10-24: no smoke flight is flown (the owner's rule, F4); its `clear` group is the ordinary flight. | `ATTEMPTS=10 ./tools/smoke_series.sh blocked 2046` |
| `phantom_blocks.py` | Blocked-route episodes of 5 s and more (the controller held by the blocked route's law) and every cell the trajectory validator named in a block, looked up in the 0.25 m truth grid: a cell with no truth within 0.5 m is a phantom (roadmap item 18; needs `log/tools/truth25`). | `python3 tools/phantom_blocks.py r1213 r1214` |
| `blind_phase.py` | The blind phase of a flight that lost its light: the plan flown on dead reckoning to the light's verdict, the descent (its height, its entry speed, the estimate's velocity error, its drift), the drift to the ground, the body's least clearance (a floor included) and the walls' (the documents' blind-descent figures, specification K19). Needs `log/tools/truth25`. | `python3 tools/blind_phase.py r1459 r1460` |
| `dead_reckoning_light.py` | Every dead-reckoning episode of a camera flight while the planner plans: its span, the share of its range the frame alone granted (`sensor_frame_range_m`, the measure of K12 and of the dark hold, K19), whether the offboard held level, and the light's own share from the evaluation record. Whether the dark hold stays out of daylight. | `python3 tools/dead_reckoning_light.py r1463` |
| `speed_limiters.py` | What holds a flight under 1 m/s: the time under it, each limiter's share, the 5th percentile of the clearance laws' limits where they bind. | `python3 tools/speed_limiters.py r1449` |
| `cores_series.sh` | A group of flights that count on the flight's core set (`FLIGHT_CPUSET`, `0-11` unless set): `cameras` or `lidar`, one at a time through `fly_until_valid.sh`, the host's temperature and clocks sampled every 10 s beside each, and every flight's verdict lines, host verdict, cost by process and core set appended to `log/tools/flight_cores/series_<tag>.txt`. | `./tools/cores_series.sh cameras 2086 5` |
| `process_cores.py` | What each process of a flight cost: cores at p50 and p95 per process from `resources.csv`, the onboard set's sum as the mission check counts it, the real-time factor and the GPU at p50, and the flight's core set from the manifest. | `python3 -I tools/process_cores.py r2070 r2083` |
| `k27_side.py` | At the slow ticks the clearance laws limit, which of K27's vertical shifts brings the truth within half a metre of the body (below, above, neither), by the truth grid. Needs `log/tools/truth25`. | `python3 tools/k27_side.py r1449 r1425` |
| `airframe_size.py` | What the routes flown ask of the vehicle's size (the distance from the body's axis to the nearest wall at its height) and a what-if of a body smaller by 0.1 to 0.3 m on the same flights (roadmap item 30). Needs `log/tools/truth25`. | `python3 tools/airframe_size.py r1430` |
| `closure.py` | Where a raw snapshot of the memory closes the way between two points: the widest path's clearance (unknown is free) and its bottleneck. | `python3 tools/closure.py log/runs/r1365/raw_snapshot_3d_revision_2353.json 62.9,30.7,12.6 63.4,5.1,14.7` |
| `stall_report.py` | A B-and-back flight's legs and its stall near B, and the snapshot nearest the stall measured from it to the trail behind it (fly it with `RAW_SNAPSHOT_BOUNDS_M` on B's area). | `python3 tools/stall_report.py r1365` |
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
| `vio/euroc_to_replay.py` | An EuRoC MAV sequence (ASL format) as a replay record: frames, the IMU turned into a forward-left-up vehicle frame, the reference, `calibration.txt` (roadmap item 25). The data stay in `/data/gazebo/datasets/` (the owner's rule), the replay sees them with `DATASETS_DIR=/data/gazebo/datasets` as `/datasets`. | `python3 -I tools/vio/euroc_to_replay.py /data/gazebo/datasets/euroc/V1_01_easy /data/gazebo/datasets/euroc/V1_01_easy/replay`, then `DATASETS_DIR=/data/gazebo/datasets ./scripts/container_run.sh bash -lc "log/tools/vio/replay_vio /datasets/euroc/V1_01_easy/replay imu=gz calibration=/datasets/euroc/V1_01_easy/replay/calibration.txt start_s=1.1 tag=k33"` |
| `vio/euroc_report.py` | A replay of a real recording against its reference: ATE after a 4-DOF alignment and without one, RTE over 10 m, the drift at the end in percent of the path. | `python3 tools/vio/euroc_report.py /data/gazebo/datasets/euroc/V1_0*/replay/replay_k33.csv` |
| `vio/dark_drift.py` | Position error of dark-window replays against the lit replay. | `python3 tools/vio/dark_drift.py log/tools/replay/r779` |

## Smoke In The Simulator

What the vehicle's sensors see through the smoke of roadmap item 18
(specification A13; [`obstacle_mapping.md`](../docs/obstacle_mapping.md),
"Smoke In The Simulator").

| Tool | What | How |
|---|---|---|
| `smoke_box_bench.py` | How a see-through smoke box renders in a dark world under the vehicle's own light, material by material (the recording's boxes, K26): a 3 m box, a spot light, one gz camera frame a variant; the glow of 0.1 + 0.3 x the opacity rests on it. | inside the container: `python3 tools/smoke_box_bench.py log/tools/release/v0.6.0/box_bench` |
| `smoke_rig.py` | A world of its own with the vehicle's sensor sets on a carrier floating 1.5 m over a textured floor, a textured wall ahead and a box of smoke between them, read through `simulation_smoke_sensor_node` as in a flight (or Gazebo's particles, or its scene fog, for the comparison; or a matte panel, for the light's calibration). Each level saves every sensor's frames decoded; `record=` also records the pair as the camera driver passes it on, still or flying through the smoke (`move=1`), for the estimator's replay. The level's keys are in its header. Needs `make build`. | inside the container: `python3 tools/smoke_rig.py log/tools/part1/final w0,wall=5 w05,wall=5,sigma=0.5 v05,sigma=0.5,record=30 f05,sigma=0.5,record=1,move=1`; a record is replayed with `log/tools/vio/replay_vio log/tools/part1/vio/f05_vio imu=gz drag=0 tag=smoke` |
| `smoke_rig_report.py` | The rig's levels as the vehicle's camera path would see them (the driver's gain, the depth node's matcher and noise mask): the wall's grey and contrast, the share matched on the wall and in the smoke, the front time-of-flight unit's zones, the lidar's forward rays from the smoke and the wall, the thermal camera; `--calibrate` the light's response from the panel levels; `--noise` the pair's noise by brightness; `--vio` the estimator's replays over the records. | on the host: `python3 tools/smoke_rig_report.py log/tools/part1/final w0 w05 --baseline w0` |
| `smoke_signature_report.py` | The signature of smoke in the pair's frame from the rig's levels (the smoke face detector, K44): on the depth node's stride grid, the share of silent cells, their raw brightness against the matched cells', the silent patches judged as K36 judges them (closed: cells, rim depth and spread, raw grey, the rim's contrast, the matches the patch encloses beyond its rim) and the front time-of-flight unit; the centre of the frame beside the whole. The corridor levels (`corridor=4.4,ceiling=3`) are where the faces close; the high levels (`height=8`) are a hall's floor out of reach, a plume floating (`smokez=`) against the lamp's cone on a veil. | on the host: `python3 -I tools/smoke_signature_report.py log/tools/smoke_detector/part0 c_s3x4 c_t01l12` |
| `light_lines.py` | The carried light's own reading against a line (roadmap item 29, specification K12), from the injector's own schedules: the longest stretch the moderate flicker (F1) keeps the light under the line over a thousand seeds and its 99th percentile, and when the severe failure and the light lost are first under it for the judgment's 3.5 s, in seconds after take-off. | `python3 tools/light_lines.py 0.25 0.35` |
| `panel_memory.py` | What the memory held of a blank panel (roadmap item 18 stage 0b): from a flight's render bag (`RENDER_BAG=1 ENABLE_LIDAR_DEBUG=true`), the occupied and darkness voxels in the panel's slab when the vehicle had been within 6 m for 8 s and at the end, the share of the face covered, and the vehicle's nearest true approach to the panel. Inside the container. | `python3 tools/panel_memory.py r1498 8 21 9.5 1.5708 3 2` |
| `stereo_depth_accuracy.sh` | Specification K7's measurement: an ordinary camera flight on Urban whose pair and evaluation depth camera are recorded from its 60th second for 420 s, given the camera driver's gain and matched as the depth node matches; prints by depth band the share matched, the share within 0.25 m and the depth and disparity errors, and the confident depth the largest disparity error at the 90th percentile from 2 to 8 m gives. Records in `log/tools/stereo/RUN` and `RUNg`, the table in `RUN_accuracy.txt`. Not an acceptance flight. | `./tools/stereo_depth_accuracy.sh r1480` |
| `stereo_record_frames.py` | The recorder `stereo_depth_accuracy.sh` runs inside the container: the pair and the evaluation depth camera over gz-transport, every KEEP-th frame of the pair's clock. | (by `stereo_depth_accuracy.sh`) |
| `stereo_gain_copy.py` | A copy of such a record with the camera driver's gain applied to each pair (95th percentile to 200, 1 to 8). | `python3 tools/stereo_gain_copy.py log/tools/stereo/r1480 log/tools/stereo/r1480g` |
| `media/fog/` | The particle texture of Fuel's Fog Emitter (OpenRobotics, CC BY 4.0) for the rig's `particles=` levels. | |

## Replaying The Lidar-Inertial Estimator Offline (`lio/`)

A lidar flight's scans and the autopilot's IMU recorded as a rosbag inside the
simulation container, extracted once, and replayed through the repository's
lidar-inertial estimator with no simulator; a change of that estimator is
measured on the record before any flight (roadmap item 13 set its thresholds
this way). The record lands under `log/tools/replay/RUN/`, a directory kept by
its `.keep`.

| Tool | What it does | Use |
|---|---|---|
| `lio/record_flight.sh` | One flight on the lidar profile through `fly_until_valid.sh` (one attempt) with `ros2 bag record` of `/lidar_3d/points` and the IMU attached inside the container, then the extraction. | `./tools/lio/record_flight.sh r1230` |
| `lio/extract_bag.py` | The bag's scans (body FRD, the extrinsic of `urban_mvp.yaml` applied) to `scans.bin` and the IMU to `imu.csv`, record by record, so a bag cut off without its footer still yields its data. | inside the container, called by `record_flight.sh` |
| `lio/build_replay.sh`, `lio/replay_lio.cpp` | Builds the replay against the tree's localization library (after `make build`) and runs the estimator over a record; options are `key=value` overrides of the estimator's configuration (`corr`, `recovery`, `keyframes`, `range`, ...), `lag` in ms and `out`. Writes the trajectory and the registration times. | `./tools/lio/build_replay.sh`, then inside the container `log/tools/replay/replay_lio log/tools/replay/r1230 HEADING out=log/tools/replay/r1230/base.csv` |
| `lio/lio_cost.py` | The lidar-inertial estimator's cost in flight from its once-a-second line: `scan_ms` at p50, p95 and most, the scans let go (specification K23), the pose age's 95th percentile (A7). | `python3 tools/lio/lio_cost.py r1410 r1411` |
| `lio/compare.py` | Horizontal error of a replay against the flight's truth, the mission check's own measurement, in ten-second windows. | `python3 tools/lio/compare.py log/runs/r1230 log/tools/replay/r1230/base.csv` |

The heading argument is the vehicle's initial heading in NED radians (the
`initial_heading_rad` the flight ran with).

One-off diagnostics written for a single investigation stay in
`log/tools/diagnostics/` and are not tracked, as long as no document cites
them; the goal journals in `log/tools/journal/` say which was used for what.

The stereo pair's depth accuracy (specification K7) is built with the
package: `build/drone_city_nav/stereo_depth_accuracy DIR` inside the
container matches a gained record as the depth node does
(`drone_city_nav/tools/stereo_depth_accuracy.cpp`, its noise mask a copy of
the node's) and holds it against the evaluation depth camera.

The CPU rollout benchmark (roadmap item 28) is built with the package:
`build/drone_city_nav/mppi_cpu_rollout_benchmark` inside the container
prints what one MPPI rollout costs on a CPU core, in open space and in a
corridor.
