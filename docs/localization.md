# Localization

Where the vehicle's position and heading come from, per localization
profile, and how the lidar-inertial estimator that replaces GNSS and the
magnetometer works, is initialised, reports its health and is measured.
The autopilot's EKF2 remains the owner of the estimate in every profile:
obstacle memory, the controller and the offboard node read
`/fmu/out/vehicle_local_position_v1` and `/fmu/out/vehicle_attitude`, and
the NED to map transform is fixed by the scenario's start pose. Gazebo's
true pose is read by the evaluation and the mission check only; it never
enters the estimator or the control path.

## Profiles

`LOCALIZATION_PROFILE` selects the profile in `scripts/run_drone_nav_sim.sh`
and reaches the launch file as `localization_profile`; the runtime manifest
records it.

| Profile | EKF2 fuses | Heading | Estimator node |
|---|---|---|---|
| `gnss` (default) | IMU, barometer, simulated GNSS position and velocity (the default height reference), the simulation heading source's attitude through external vision (`EKF2_EV_CTRL 8`) | the simulator's true attitude with a wandering bias and noise (`simulation_heading_source_node`, `ENABLE_SIMULATION_HEADING_SOURCE`) | not run |
| `gnss_shadow` | as `gnss` | as `gnss` | run, publishes its estimate to the diagnostic topic only; the mission check compares it with the true pose |
| `lidar_inertial` | IMU and the estimator's odometry: position, height (`EKF2_HGT_REF 3`) and yaw (`EKF2_EV_CTRL 11`, no velocity); `EKF2_GPS_CTRL 0`, `EKF2_MAG_TYPE 5` | the estimator's | run, publishes `VehicleOdometry` to `/fmu/in/vehicle_visual_odometry`; the heading source is forced off |

The magnetometer is not fused in any profile: the simulated one sits five to
six degrees off. In `lidar_inertial` nothing the autopilot fuses comes from
the simulator's truth. `scripts/px4_parameter_runtime.sh` streams the
parameters and shows the four the mission check reads back.

## The Estimator

`lidar_inertial_odometry_node` (`src/lidar_inertial_odometry_node.cpp`) is
the ROS shell around `LidarInertialOdometry`
(`include/drone_city_nav/lidar_inertial_odometry.hpp`, the localization
layer, which depends on Eigen only and on nothing of the planner or the
obstacle memory; `tests/test_navigation_dependency_contract.py` holds that).

Inputs: `/lidar_3d/points` (the 360 x 181 scan at 10 Hz, stamped on the
simulation clock) and `/fmu/out/sensor_combined` (the IMU at 110 Hz on the
transport's clock), with `/fmu/out/timesync_status` for the clock mapping
(`Px4RosTimeMapper`, the same resolution the obstacle memory applies to the
autopilot's position stamps). The lidar extrinsic is the mounting the
obstacle memory projects with.

Per scan:

1. the IMU is integrated from the last registered scan to this scan's stamp
   (gravity, the gyroscope bias as a state, the accelerometer at rest for
   the initial level); the stretch after the last sample before the stamp
   is integrated with the sample that spans it;
2. the scan is thinned to one point per 0.4 m cell and coarser until it
   holds at most 4000 points, so the submap keeps no holes;
3. point-to-plane Gauss-Newton registration against a sliding submap of
   the last 40 keyframes in a voxel hash, normals fitted over two rings,
   the IMU pose as the starting guess, a robust width of 0.2 m; a failed
   registration is retried from the last registered pose with the
   correspondence widened three times, and until one registers the
   position holds there while the attitude follows the gyroscope;
4. the registered position updates position and velocity in a Kalman step:
   the IMU's motion is the prior with white acceleration noise, the
   registration the measurement, its variance from the registration's
   information along each translational axis floored at 3.2 cm. An axis
   with under 0.01 of information per matched point is degenerate (a bare
   corridor leaves its own axis free) and carries no measurement; an axis
   whose registered position lies more than five standard deviations of
   the prior and the measurement from the prior is gated for that scan;
5. a keyframe is inserted every 0.5 m or 0.17 rad at the registered
   position along the observed axes, so the submap follows what the scans
   saw and not the filter's blend with the IMU.

Scans wait for the IMU up to their own stamp and keep their order; the
subscription queues hold 100 IMU samples and 3 scans, because a
registration takes 40 to 80 ms at p50 and p95 on the workstation and
longer at times. The published odometry carries the scan's stamp mapped
into the autopilot's synchronised clock, the position and velocity
variances from the filter and the orientation variance from the
registration's rotational information; a scan that did not register is
not published, so the autopilot sees no estimate rather than a wrong one.

The parameters are in `config/urban_mvp.yaml` under
`lidar_inertial_odometry_node`, each with the measurement that set it.
The thresholds of the filter were set by replaying the estimator offline
over the lidar and IMU recorded on seven flights, three times each with
the IMU delivered 0, 4 and 8 ms early, against the true pose; the flights
confirmed them.

## Initialisation And Frames

The declared initial pose is the scenario's start: the autopilot's local
origin and the start heading (`initial_heading_rad`, set by the launch file
from the scenario's `yaw_rad`). The estimator starts at the origin of the
NED frame with that heading and the level the accelerometer shows at rest,
and its odometry is published in `POSE_FRAME_NED`. Nothing is taken from
the simulator's truth. The goal capture and every map-frame consumer read the autopilot's local position through the
scenario's fixed NED to map transform, unchanged from the GNSS profile.

The autopilot aligns its estimator to the odometry at start, which resets
its position by centimetres; the controller measures each reset's shift on
the published estimate and treats a shift within 0.33 m as the same frame.
A larger shift, which a restart of the external-vision fusion produces
(0.8 to 2.3 m on r389, r393 and r398, before the queues were deepened),
closes the navigation for the flight.

## Health

Every scan yields the share of the scan that matched the submap, the
registration residual, the information along the weakest translational
axis per matched point, the count of degenerate and gated axes and the
correction along the track; the node logs them once a second
(`LIDAR_INERTIAL_ODOMETRY`) and publishes the estimate's pose to
`/drone_city_nav/lidar_inertial_odometry/pose` with the frame `map` or
`map_unhealthy`. An unhealthy scan (matched share under 0.3, residual over
0.5 m, or no convergence) is not published to the autopilot. Without
odometry for 200 ms the autopilot's external-vision fusion stops, and after
`EKF2_NOAID_TOUT` (5 s) of dead reckoning it withdraws its horizontal
estimate: the local position arrives at the stack with `xy_valid` false,
the controller revokes execution on the stale pose and the offboard node
holds. No new latch and no restriction of motion is added for this; the
chain is the pose-age contour that exists for every profile
(`maximum_pose_age_ms`, `production_mppi_node_planning_tick.cpp`), and the
link from the withdrawn estimate to an invalid position is pinned by
`px4_autopilot_adapter_test.cpp`.

## Measurement

The mission check ([testing.md](testing.md)) reports on every flight:

- the position estimate against the true pose (cross-track p95 and
  along-track offset), the same check as on the GNSS profile;
- the lidar-inertial estimate against the true pose, from the estimate the
  node published (`lio_estimate.csv`,
  `scripts/capture_lidar_inertial_estimate.py`);
- `localization profile`: on `lidar_inertial`, `EKF2_GPS_CTRL 0`,
  `EKF2_MAG_TYPE 5`, `EKF2_EV_CTRL 11` and `EKF2_HGT_REF 3` shown in the
  autopilot log, no `simulation_heading_source_node` in the ROS log, and the
  odometry published at 5 Hz or more over the flight;
- `lidar-inertial estimator health`: the matched share, residual and
  weakest-axis information at p50 and their worst, and the scans that left
  the autopilot without an estimate, which must be none in a clean flight.

Measured on the urban point-to-point mission with the 3D lidar and no
static map, the r430 to r434 series on commit 5a113bc5 (the acceptance
series in `docs/roadmap.md`): the autopilot's estimate 0.15 to 0.22 m from
the true pose across the track at p95, the estimator's own 0.06 to 0.17 m,
no crash, 2.57 to 2.97 m/s, the estimator at 0.5 cores at p50 and 59 MiB
([resource_budget.md](resource_budget.md)). The GNSS profile measured 0.19
to 0.25 m over r340 to r356. Along the track the check's own resolution is
its 0.02 s alignment grid and the pose age at the tick: the GNSS flights
read -0.013 to +0.022 s and the lidar-inertial ones -0.029 to +0.013 s
(r415 to r434),
while the autopilot's position and the odometry it fuses agree to 5 ms in
every flight.

Known limits, measured: the flights that failed while the estimator was
being set (r386, r387, r399) all lost the track along the corridor at
x 30 to 62, y -15, where the walls leave the corridor's axis with 0.002
to 0.05 of information per point and the IMU carries it; the estimate
drifts 0.08 to 0.22 m there on the accepted flights. Loop closure is not
implemented: over the 400 m mission the drift does not grow with the
flight's length (the error at the goal, 0.1 to 0.2 m, is the corridor's,
not the distance's), so it stays the next line of roadmap item 13.
