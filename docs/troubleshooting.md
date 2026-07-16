# Troubleshooting

This page lists common failure modes and first checks.

## Simulation Does Not Start

Check:

- Docker permissions: `docker ps`;
- stale simulator processes: `./scripts/stop_sim.sh --dry-run`;
- full cleanup: `./scripts/stop_sim.sh`;
- container image availability;
- host display access for GUI runs.

## Build Cannot Find `px4_msgs`

Use the repository wrappers:

```bash
./scripts/build.sh
./scripts/test.sh
```

They start the dev container and source `/opt/px4_msgs_ws/install/setup.bash`
automatically. If `px4_msgs` is still missing, rebuild or refresh the dev image:

```bash
./scripts/build_dev_image.sh
```

Only set `PX4_MSGS_SETUP_FILE` manually when intentionally using a custom
container image or external `px4_msgs` install.

## Drone Takes Off And Hovers

Check:

- whether `/drone_city_nav/path` is published;
- planner logs for A* or trajectory failure;
- offboard logs for path acceptance/rejection;
- PX4 offboard mode and arming state;
- mission start/goal/origin consistency.

## RViz Does Not Show The Path

Check:

- `enable_rviz:=true`;
- RViz fixed frame is `map`;
- `/drone_city_nav/final_trajectory_path`;
- `/drone_city_nav/path`;
- whether the planner published an empty hold path;
- `log/final_trajectory_samples/latest.csv`.

## Path Flickers Or Disappears

Likely causes:

- invalid new trajectory;
- failed replan;
- diagnostics/path stamp mismatch;
- empty hold path after planning failure;
- stale pose or path update rejection.

Inspect planner logs and offboard trajectory update logs.

## Replan Loop

Check:

- prohibited-grid intersection logs;
- raw obstacle sources;
- current lidar overlay quality;
- obstacle memory scoring;
- inflation and planning clearance settings;
- whether the drone is actually crossing the prohibited grid or only the
  planning-clearance margin.

## Gazebo Or PX4 Processes Remain After Closing GUI

Run:

```bash
./scripts/stop_sim.sh --dry-run
./scripts/stop_sim.sh
```

The standard workflow does not support multiple simultaneous Gazebo instances.

## Docker / X11 Problems

Check:

- Docker group membership;
- `DISPLAY`;
- Xauthority or Wayland/XWayland setup;
- whether host security policy blocks GUI clients from containers.

For headless validation, use `./scripts/sim_headless.sh`.

## Lidar Or Memory Is Shifted

Check:

- `px4_local_origin_x_m`, `px4_local_origin_y_m`;
- `use_px4_heading_for_scan`;
- `scan_yaw_offset_rad`;
- lidar mount roll/pitch/yaw;
- `lidar_pose_latency_s`;
- attitude compensation settings.

Use lidar debug snapshots and the static map analyzer script.

If downward lidar returns cause false replans, inspect both
`Obstacle memory lidar decisions` and `Planner current lidar decisions`:

- `expected_ground` should rise when the tilted lidar sees the physical ground;
- `closer_retained` means a return was materially before the expected ground or
  known static surface and was intentionally kept;
- `ground_unavailable` indicates invalid ground configuration, missing altitude,
  or missing attitude required by compensated 3D projection;
- `non_ground_altitude_rejected` identifies the legacy altitude gate rather than
  ground rejection.

Compare the bounded sample's measured range, expected range, endpoint Z, and
ray direction. Do not add a global tilt cutoff to hide the symptom: ordinary
high-speed flight can use the same roll/pitch angles. Verify that
`ground_lidar_altitude_m` matches the physical ground surface and that both
nodes log the same effective tolerances.

Every memory-sourced prohibited replan should report
`memory_provenance[status=matched ...]` on the same log line. The atomic
`/drone_city_nav/obstacle_memory_snapshot` subscription does not accept a grid
without its exact provenance. The matched record contains endpoint XYZ,
attitude, measured/expected ranges, delta, and selected ingestion surface.

If a memory blocker reports `not_received`, `cell_missing`, `pending`, or
`history_expired`, treat it as an invariant failure rather than normal callback
ordering. Check for `Ignoring invalid atomic obstacle memory snapshot`, compare
the nested grid/provenance identity, and verify that the planner subscribes to
the atomic snapshot topic rather than the standalone debug topics.

For a suspected transport-induced replan or clearance failure, inspect the
`memory_snapshot_transport[...]` block on that exact replan line. Its sequence,
stamp, apply age, current age, callback time, receive/apply rates, pending
sequence, replacements, and rejects describe the authoritative planner input at
the event. Cross-check the active sequence against `Obstacle memory snapshot
published`, `queued`, and `applied`. `dds_sequence_gaps` means DDS replaced a
message before the dedicated callback received it. `pending_replacements` is
expected when multiple valid snapshots arrive while one planning cycle is
running; the newest complete pair wins. A reject means a delivered pair failed
identity or nested consistency checks. Budget warnings identify oversized
messages, slow assembly, stale adoption, slow parsing, excessive apply delay, or
inadequate effective apply rate.

## A* Does Not Find A Path

Check:

- prohibited grid coverage;
- start and goal positions;
- grid bounds;
- static map path;
- inflation and planning clearance;
- whether current lidar or memory creates a blocking wall.

## Trajectory Diagnostics Do Not Match

Diagnostics should match accepted trajectories by `path_stamp_ns`. If they do
not:

- check `/drone_city_nav/path` header stamp;
- check `/drone_city_nav/trajectory_diagnostics`;
- check offboard logs for accepted planner id confirmation;
- check whether a candidate trajectory was rejected after diagnostics arrived.

## Drone Takes Too Long To Position At The Finish

Check terminal logs:

- terminal state;
- velocity terminal capture activation distance;
- position capture reason;
- final hold speed and radius;
- current speed near the final point.

Position capture should take over near the end. If it does not, check terminal
thresholds and speed.

## Drone Oscillates On A Nearly Straight Segment

Check:

- signed cross-track zero crossings;
- normal velocity zero crossings;
- desired versus smoothed normal velocity;
- projection smoothing mode;
- curvature feedforward context scale;
- P gain factor near the path;
- effective D gain factor;
- smoother jerk limit activity.

Likely causes:

- local tangent noise is being followed too literally;
- curvature feedforward is active on sign-changing micro-curvature;
- near-path P gain is too weak or too strong;
- D damping is insufficient at high speed;
- smoother lag makes the command arrive late.

Do not assume the published trajectory is broken only because it is slightly
wavy. The drone should track reasonable smooth waves without visible
left-right oscillation.

## Drone Misses A Turn

Separate scalar speed from velocity direction:

- Was speed consistent with the turn radius?
- Was actual normal velocity outward before the turn?
- Did desired normal velocity point into the turn early enough?
- Did the smoother clip setpoint rotation?
- Did projection smoothing suppress curvature too much?
- Did the turn radius shrink quickly over a short distance?

If scalar speed was too high for the radius, inspect speed profile and
lookahead. If scalar speed was reasonable but direction was wrong, inspect
projection, feedforward, normal velocity, and smoother lag.

## Trajectory Looks Too Angular

Check:

- corridor width near the angular segment;
- optimizer active windows;
- minimum radius and radius-shortfall cost;
- curvature-jump cost;
- turn-smoothing detected and attempted corners;
- rejected smoothing candidate reasons;
- prohibited-grid intersections for nicer candidates.

If the corridor is narrow, the planner may not have space to create a large
radius. If the corridor is wide, tune trajectory optimizer and turn-smoothing
logic before changing runtime control.

## Planning Time Regressed

Check stage wall time first:

- grid and clearance;
- A*;
- corridor;
- trajectory optimizer;
- turn smoothing;
- speed profile;
- diagnostics.

Then inspect aggregate candidate timing. A wall-time regression in optimizer or
turn smoothing often comes from too many active samples, too many candidates,
or strict rejection rules that force long searches.

If a diagnostic-only feature became expensive, disable or sample it before
changing path selection.

## New Trajectory Is Rejected

Check:

- candidate path validity;
- stale pose handling;
- projection jump;
- tangent jump;
- curvature jump;
- speed-limit jump;
- tangent-speed command jump;
- accepted path stamp and diagnostics stamp.

A rejected candidate should not delete the active trajectory. If the path
disappears after rejection, inspect publication and fallback behavior.
