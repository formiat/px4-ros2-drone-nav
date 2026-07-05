# Logging And Diagnostics

Diagnostics are a first-class part of the project. Planner and controller
behavior should be debugged from logs and artifacts, not only from RViz.

## Main Log Locations

- `log/offboard_blackbox.jsonl`
- `log/final_trajectory_samples/`
- `log/corridor_samples/`
- `log/lidar_debug/`
- `log/gazebo_scene_debug/`
- `log/gz_city_mvp.log`
- `log/gz_gui_city_mvp.log`

## Offboard Blackbox

`log/offboard_blackbox.jsonl` mirrors runtime telemetry. It includes:

- path ids and accepted path stamp;
- current pose and velocity;
- cross-track error;
- heading error;
- commanded target motion;
- desired and smoothed velocity setpoints;
- attitude and tilt;
- nearest prohibited-cell information;
- terminal state data.

Use this file for control-quality analysis.

## Final Trajectory Dumps

`log/final_trajectory_samples/` contains timestamped trajectory CSV files and a
`latest.csv` shortcut. Summary JSON files sit next to the CSV files.

These files are useful for:

- geometry inspection;
- speed-profile constraints;
- curvature spikes;
- accepted trajectory id and stamp;
- reproducing visual artifacts seen in RViz.

## Corridor Dumps

`log/corridor_samples/` contains corridor samples for each rebuild. Use it to
debug narrow passages, asymmetric corridors, and centerline recovery behavior.

## Trajectory Diagnostics JSON

The planner publishes `/drone_city_nav/trajectory_diagnostics`. Diagnostics are
matched to the accepted path by `path_stamp_ns`, not by assuming topic delivery
order. The accepted planner id is confirmed from matching diagnostics.

Important fields:

- total build duration;
- grid/corridor/optimizer/turn-smoothing timings;
- active windows;
- candidate counts;
- speed-profile top constraints;
- fingerprints for speed profile construction and runtime policy/control.

Only construction fingerprint mismatch is a warning. Runtime mismatches are
context because offboard-only control settings can legitimately differ from
planner settings.

## Replan Diagnostics

Planner logs include replan reasons and blockers, for example:

- prohibited intersection;
- path stale/invalid;
- A* failure;
- no valid trajectory;
- diagnostics mismatch;
- invalid or rejected refined trajectory.

## Important Metrics

For flight quality:

- speed;
- tilt;
- roll/pitch;
- cross-track error;
- signed cross-track error;
- normal velocity;
- commanded normal velocity;
- desired-to-setpoint velocity error;
- jerk limiter activity;
- terminal state.

For planner quality:

- total path build wall-time;
- grid build time;
- A* time;
- corridor time;
- trajectory optimizer time;
- turn smoothing time;
- speed profile time;
- replan count.

## Last Run Analysis Checklist

1. Check whether the mission completed or failed.
2. Check replan count and reasons.
3. Check accepted trajectory geometry.
4. Check top speed constraints.
5. Check cross-track and normal velocity around bad segments.
6. Check terminal state separately from cruise flight.
7. Compare RViz colors to trajectory CSV and diagnostics JSON.
