# Changelog

Code releases are tagged `vMAJOR.MINOR.PATCH` on `main`. Environment asset
bundles are released separately under `environment-assets-*` tags; each code
release names the asset tags it was validated with.

## Unreleased

- The navigation stack reads the autopilot through one contract
  (`autopilot_state.hpp`, `AutopilotStateSource`); the PX4 adapter
  (`px4_autopilot_adapter.cpp`, the offboard node and the setpoint builders)
  is the only code that includes `px4_msgs`, and a script test pins that
  boundary. The NED-to-map frame, the altitude sign, the heading convention
  and the payload fingerprint live in the adapter alone. The wire is unchanged.
- Every headless flight records the setpoints against the local position and
  the true Gazebo pose, and the mission check holds four controller-dynamics
  measurements to the assumptions of the navigation laws: lateral tracking
  error, descent arrest, position-estimate error and lidar evidence age
  (`scripts/controller_dynamics_evidence.py`, `docs/testing.md`).
- `scripts/prune_sim_logs.sh` deletes simulation logs older than a week
  (`DRONE_GAZEBO_PRUNE_LOGS_DAYS`) from the places the stack writes them:
  every entry of `log/` except `log/tools`, every run directory, and the PX4
  flight logs under `rootfs/<n>/log`. An entry with a `.keep` file on top
  stays; symbolic links and anything outside the repository are never
  touched. The sim wrappers run it before every run; `--dry-run` previews and
  `DRONE_GAZEBO_PRUNE_LOGS=false` disables it. The logs of one month filled
  118 GB before this.
- `stop_sim.sh` stops every container of this repository (the dev image or
  the repository mount) that runs any Makefile `sim-*` target; the former
  pattern named only `sim-gui` and `sim-headless`, so a container left
  behind by `make sim-urban-point-to-point-gui` survived every stop. Stopped
  containers are removed. Every `sim_*.sh` wrapper now runs that cleanup
  before its run and again when the run ends (`scripts/run_sim_wrapped.sh`).

## v0.2.1 (2026-09-14)

- `scripts/bootstrap.sh` prepares a fresh clone with one command: host
  checks, the dev image, the PX4 checkout and its `px4_sitl` build inside the
  container, the workspace build and the urban environment assets, then the
  urban point-to-point simulation (`--headless`, `--no-run`). The 3D-lidar
  profile launches the PX4 binary directly, so the SITL build was a manual,
  undocumented step before. Verified on a fresh clone of v0.2.0: the
  preparation took about 25 minutes with the dev image already present, and
  the headless flight that followed completed the mission at 3.162 m/s
  (389.8 m in 123.3 s) without a crash.

## v0.2.0 (2026-09-14)

First tagged release of the 3D navigation stack: point-to-point flight without
a static map through a complex 3D urban location, from revisioned 3D-lidar
evidence alone. Roadmap items 1 to 8 are complete; item 12 (persistent full-3D
strategic navigation) is the substance of this release and stays in progress.

### Validated scenario

`make sim-urban-point-to-point-headless` on the Urban Circuit Practice 01
location (`environment-assets-urban-v1`), no static map, 3D lidar 360 by 181
beams at 10 Hz. Five consecutive flights on commit `9ab94040` (r288 to r292),
nothing changed between them:

| Flight | Path | Duration | Mean speed | Crash | Route availability | Planner p95 |
|---|---|---|---|---|---|---|
| r288 | 399.9 m | 137.8 s | 2.901 m/s | none | 95.1 % | 154 ms |
| r289 | 384.6 m | 148.8 s | 2.584 m/s | none | 93.1 % | 152 ms |
| r290 | 401.5 m | 134.4 s | 2.986 m/s | none | 92.0 % | 157 ms |
| r291 | 371.5 m | 123.9 s | 2.998 m/s | none | 91.2 % | 154 ms |
| r292 | 417.2 m | 134.7 s | 3.097 m/s | none | 88.9 % | 155 ms |

The mean flight speed counts every hold, stop and replan from mission
readiness to the successful result. The headless mission check requires at
least 2.5 m/s and no crash; it still requires 97 percent route availability,
which these flights do not reach (see the limitations).

Other scenarios (`sim-headless` Manhattan, cooperative traffic, interception)
build and pass their contract tests but were last flown before the September
navigation changes listed below; they are not part of this release's
validation.

### Navigation laws in this release

- Contact is the depth the body already has in the evidence, judged voxel by
  voxel along the departure chain and never carried to another wall; one law
  for the planner and the executor, with a regression reproducing the r216
  contact deadlock (`061e2532`, `4bf8a4dd`).
- A rest hold keeps its anchor while the vehicle still stands within the hold
  tolerance of it, and the offboard continues a lapsed stationary hold at its
  own position (`d746bdb8`).
- A vehicle is at rest only while it is not accelerating: the rest rearm and
  the hold certification require the measured acceleration within 1 m/s^2
  (`ebadf243`).
- The vertical dynamics are the descent arrest the airframe delivers, 2 m/s^2
  for both the guaranteed vertical deceleration and the planned vertical
  acceleration (`a44cc7e0`).
- The tracking-error tube law prices the envelope's clearance with a 0.075 s
  response horizon and a 3 m/s progress floor; the physical body's own
  clearance bounds that floor, down to 1 m/s for leaving a contact
  (`d187e249`, `9ab94040`).
- A blocked route's replacement is taken as soon as it is raw-valid; the
  grace a worse replacement used to earn is nil (`5d5dd9ea`, `cb45e5fb`).
- A full raw snapshot recovers the transport admission from deltas that
  outran it (`b116f7fe`).
- The 3D lidar cloud is registered with per-source time offsets, the position
  source at -120 ms and the attitude source at 0 (`a00368b2`); the memory is
  no longer smeared by the vehicle's own roll and pitch.
- Braking contract: 14 m guaranteed detection range, 2 m physical margin,
  0.6 s evidence age, 4 m/s^2 horizontal and 8 m/s^2 lateral acceleration,
  12 m/s^3 jerk; the translational limit is 5.14 m/s.
- Mean flight speed is a mission-check metric (`03632053`), 2.5 m/s in this
  release.

### Known limitations

- Route availability after bootstrap is 88 to 95 percent on the urban
  location; the 97 percent threshold of the mission check is not met. The
  remaining unavailability is replacement search after walls revealed a few
  metres ahead in unknown space, which stays traversable without penalty by
  design.
- Speed next to walls is bound by the body's own clearance: with the 0.55 m
  body model (0.16 m beyond the rotor tips) and a lateral tracking error of
  0.09 to 0.15 m at p99, the reference falls to 1 m/s where the body keeps
  less than 0.075 m.
- The position estimate is not corrected against the map: PX4's local
  position sits 0.1 to 0.3 m from the true pose, and walls are mapped in the
  estimate's frame at observation time.
- Descents are planned at 2 m/s^2 and arrested at the same; passages that
  open below the vehicle are entered more slowly than before.
- The speed profile, the lidar model and the braking contract are tuned on
  this simulation (PX4 SITL, Gazebo Harmonic, ROS 2 Jazzy). Nothing here is
  validated for real aircraft.

### Compatibility

- Environment assets: `environment-assets-urban-v1` (Urban Circuit Practice
  01), `environment-assets-v1` (Finals Prize Round World 07, Cave Circuit
  Practice 01).
- Runtime: ROS 2 Jazzy, Gazebo Harmonic, PX4 from `external/PX4-Autopilot`,
  the container workflow in `docker/`.
- The runtime manifest of every flight (`log/runs/<id>/manifest.json`) records
  the package version, `git describe`, the commit and the effective overrides.

## v0.1.0

The initial `package.xml` version; never tagged. Roadmap items 1 to 8:
interceptor missions, radar-derived tracking, target prediction, multi-drone
interception, cooperative air traffic, advanced 3D passages and the 3D
perception and raw-world foundation.
