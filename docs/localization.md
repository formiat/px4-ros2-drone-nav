# Localization

Where the vehicle's position and heading come from, per localization
profile, and how the two estimators that replace GNSS and the magnetometer,
lidar-inertial odometry (LIO) and visual-inertial odometry (VIO, a stereo
MSCKF), work, are initialised, report their health and are measured. Both
correct their drift against a map of their own, which makes the pair a SLAM
system in the broad sense: localization and mapping at once, with map-based
relocalization and no pose graph or global optimization.
The autopilot's EKF2 remains the owner of the estimate in every profile:
obstacle memory, the controller and the offboard node read
`/fmu/out/vehicle_local_position_v1` and `/fmu/out/vehicle_attitude`, and
the NED to map transform is fixed by the scenario's start pose. Gazebo's
true pose is read by the evaluation and the mission check only; it never
enters the estimator or the control path.

## Profiles

`LOCALIZATION_PROFILE` selects the profile in `scripts/run_drone_nav_sim.sh`
and reaches the launch file as `localization_profile`; the runtime manifest
records it. No single-vehicle flight flies on GNSS unless it is asked to: the
default is the estimator the navigation sensors feed, `visual_inertial` on the
stereo sensor set (the repository's default sensors, roadmap items 14 and 16)
and `lidar_inertial` on the lidar (roadmap item 13). `gnss` remains for
comparison and for the multi-vehicle missions: their launches run no estimator
and the vehicles share no frame yet (roadmap item 15), so they refuse the
other profiles and say so. `NAVIGATION_SENSOR_PROFILE=stereo_tof` refuses
`lidar_inertial` (no lidar to register), and the visual-inertial profiles
require `CAMERA_PROFILE=stereo_tof`.

| Profile | EKF2 fuses | Heading | Estimator node |
|---|---|---|---|
| `gnss` | IMU, barometer, simulated GNSS position and velocity (the default height reference), the simulation heading source's attitude through external vision (`EKF2_EV_CTRL 8`) | the simulator's true attitude with a wandering bias and noise (`simulation_heading_source_node`, `ENABLE_SIMULATION_HEADING_SOURCE`) | not run |
| `gnss_shadow` | as `gnss` | as `gnss` | run, publishes its estimate to the diagnostic topic only; the mission check compares it with the true pose |
| `lidar_inertial` (default on the lidar) | IMU and the estimator's odometry: position, height (`EKF2_HGT_REF 3`) and yaw (`EKF2_EV_CTRL 11`, no velocity); `EKF2_GPS_CTRL 0`, `EKF2_MAG_TYPE 5` | the estimator's | run, publishes `VehicleOdometry` to `/fmu/in/vehicle_visual_odometry`; the heading source is forced off |
| `visual_inertial_shadow` | as `gnss` | as `gnss` | the visual-inertial estimator runs inside the stereo process and publishes its estimate to the diagnostic topic only; the mission check compares it with the true pose |
| `visual_inertial` (default on the stereo set) | IMU and the visual-inertial estimator's odometry: position, height (`EKF2_HGT_REF 3`) and yaw (`EKF2_EV_CTRL 11`, no velocity), with a fixed noise (`EKF2_EV_NOISE_MD 1`, 0.3 m, 0.05 rad); `EKF2_GPS_CTRL 0`, `EKF2_MAG_TYPE 5` | the estimator's | run inside the stereo process, publishes `VehicleOdometry` to `/fmu/in/vehicle_visual_odometry` every 40 ms |

The magnetometer is not fused in any profile: the simulated one sits five to
six degrees off. In `lidar_inertial` and `visual_inertial` nothing the
autopilot fuses comes from the simulator's truth. `scripts/px4_parameter_runtime.sh` streams the
parameters and shows the four the mission check reads back.

## The Lidar-Inertial Estimator

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
2. the scan is read to the sensor's whole 35 m (read to 30 m, the ends of a
   55 m street were not in the scan, the registration had no measurement
   along it, and the estimate slid 0.06 to 0.35 m there and once 2.0 m,
   r461) and thinned to one point per 0.4 m cell and coarser until it holds
   at most 4000 points, so the submap keeps no holes;
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

## The Visual-Inertial Estimator

Roadmap item 16. `VisualInertialOdometry`
(`include/drone_city_nav/visual_inertial_odometry.hpp`, the localization
layer) is a stereo multi-state constraint Kalman filter. It reads Eigen and
nothing else: `tests/test_navigation_dependency_contract.py` holds every
system include of its header and source to the standard library and Eigen, so
the filter is replayed offline (`log/tools/vio`) and carries no image library
and no middleware. The images are followed outside it, by
`StereoFeatureTracker` (`src/stereo_feature_tracker.cpp`, OpenCV behind its
implementation), and reach the filter as normalized coordinates under stable
ids.

The state is the body pose, velocity and the two IMU biases with a sliding
window of twelve cloned body poses, one per frame; no point is a state. The
IMU propagates state and covariance between frames. A feature is used once,
when its track ends or reaches the pose about to leave the window: it is
triangulated from all its stereo observations, its residuals are projected
onto the left null space of its own Jacobian, gated by chi-square, stacked,
compressed by QR and applied in one Kalman step. Jacobians and the transition
are evaluated at first estimates: the heading and the position of the whole
scene, which no camera observes, are never learned from features, and the
filter does not report a certainty it does not have (unit test).

The tracker follows corners of the left image from frame to frame and into
the right image of the same frame: pyramidal Lucas-Kanade both ways (a track
survives when the way back ends within a pixel of where it started), started
from the positions the gyroscope predicts (a turn of ten degrees between
frames moves a corner sixty pixels), an epipolar RANSAC between frames, and
the row between the rectified cameras. New corners fill a grid, up to 200.

It follows the frame's texture, not its grey level: the frame smoothed over
a pixel less its mean over some 32 pixels (taken on a frame an eighth the
size). The vehicle's own light (roadmap item 17) moves with the cameras, so
a surface brightens as the vehicle nears it, the light's falloff sweeps the
scene and the gain scales the frame; following the grey level, the filter
gated twice the features and drifted twice as far per 100 m with the light
on (r775 against r774), and dark flights ended 2.1 to 2.3 m from their goal
in truth. Replayed on two recorded flights (`log/tools/replay/record_vio.sh`
records the pair after the gain as the estimator receives it), the texture
ended the estimate 0.22 m from the truth in the dark world against 2.96 m,
and 0.34 m in the lit one against 1.38 m; the first dark flight on it
drifted at most 0.53 m per 100 m (r781).

`visual_inertial_odometry_node` lives in the process that owns the pair's
images (`gazebo_stereo_depth_node` or `stereo_depth_node`, switch
`--visual-inertial-odometry`), so two 1.2 MB frames 7.5 times a second reach
it without a copy. A frame waits for the autopilot's IMU up to its own stamp,
in order. The estimator and the camera perception read the same frames; of
the perception's products the estimator reads the depth cloud alone, for a
map of its own (below), and never the obstacle memory: an estimator that took
its pose from a map built from that pose would hide its own drift.

What the recorded flights set (`log/tools/vio`: a recorder of every frame,
both IMU streams and the true pose on one clock; a replay of the tracker and
the filter; reports by manoeuvre):

- the frame stamp leads the IMU's clock by 4 ms: the share of gated features
  over a sweep of the offset has its minimum at +4.1 ms (r547) and +3.8 ms
  (r550) and doubles 10 ms either side (`frame_stamp_offset_s`);
- the estimator runs at the matcher's 7.5 Hz: every second frame (3.75 Hz)
  flies the same record as well, so the pair is not rendered at 15 or 30 Hz,
  which costs the simulation its real time;
- the accelerometer's noise density is 0.2 m/s^2/sqrt(Hz), not the sensor's
  0.02: a tilt error of 0.2 degrees leaks 0.03 m/s^2 of gravity, and with 0.02
  the filter refused a quarter of the features and measured every
  displacement 1.6 percent short;
- the gyroscope's is its own, 1.0e-4 rad/s/sqrt(Hz), with a bias walk of
  2.0e-6 and the initial bias known to its scatter at rest: between frames the
  heading is the gyroscope's, and told a gyroscope ten times noisier the
  filter let every update's noise walk the heading 2 to 4 degrees over a
  flight, 1.6 to 2.1 m at a goal 62 m away; with its own noise the heading
  ends within a degree and the position 0.25 to 0.89 m from the truth on six
  records;
- samples farther apart than 50 ms are a hole in the IMU stream (the
  autopilot's IMU crosses a best-effort transport at 83 to 92 Hz and loses
  bursts when the host stalls): the uncertainty grows over a hole by what the
  vehicle can do, and a point the cloned poses cannot agree on is placed by
  the newest frame's own pair. Without these one 0.52 s hole refused every
  feature for the rest of a flight;
- the time-of-flight ranges are not fused: climbing a shaft a metre from its
  wall the estimate loses 0.09 m over 3.4 m (0.28 m over 3.9 m on the record
  with IMU holes), the vertical being the best-held axis.

Over four seconds, the time a surface stays in view, the estimate loses
0.11 to 0.18 m at the median and 0.25 to 0.45 m at p95, and 0.2 to 0.3
degrees of heading, in every manoeuvre class (hover, shaft, stop and start,
straight, turning flight): inside what the map tolerates.

On `visual_inertial` the node hands the autopilot a pose every 40 ms, the
last frame's estimate carried through the IMU samples received since
(`predicted`): the pair gives a pose 7.5 times a second and the autopilot ends
its external-vision fusion after 200 ms without one, so one late frame would
end it. The autopilot fuses the poses with a fixed noise (0.3 m, 0.05 rad),
not the estimator's variances: an odometry's honest variance of its position
in the world grows without bound, and the autopilot has no other position.
0.3 m is what a frame's update moves the pose by (up to 0.22 m between
consecutive frames); with 0.1 m the autopilot's gate threw the odometry away
after a 0.4 m correction, flew 1.8 s on its IMU alone and reset its position
by 1.19 m, which closed the navigation for good then (r561; a reset up to
3 m is flown on since roadmap item 19).

Health: the estimate is healthy while features corrected it within a second
and the IMU reaches the frame. The node prints once a second the tracked,
offered, used, gated and untriangulated features, the residual in observation
deviations, the standard deviation of the velocity along its least certain
direction, the frame's cost, the IMU's lag and largest gap, and the frames
without a healthy estimate. A hole of seconds in the IMU stream is not
bridged: under a host frozen by another task's disk writes the filter came out
of holes of 1.9 and 3.6 s at 17 m/s, refused every feature afterwards and
published two diverging frames as healthy, and the autopilot, fusing them,
flew both vehicles into walls (r720, r723). Since 2026-09-27 a hole longer
than the unaided timeout leaves the estimate unhealthy for the rest of the
flight, and so does a velocity less certain than 1 m/s along any direction
while it lasts (`maximum_velocity_sigma_mps`; the flying filter holds 0.08
to 0.18): the estimator falls silent, the autopilot ends its external-vision
fusion and its own failsafe lands the vehicle. The filter is not initialised
again in flight; that is the register's remaining entry. The mission check reports these and the estimate
against the true pose as notes; what the estimate answers for is the
programme's first requirement, checked by the truth: at every goal
acknowledgement the true position is inside the 2.0 m capture radius
([testing.md](testing.md)).

Cost: 0.53 core beside the depth matcher (the stereo process 1.69 to 2.22
cores), a frame 44 to 60 ms at the median and 72 to 82 ms at most, inside the
0.5 to 1.5 cores stage 0 reserved.

### Relocalization Against Its Own Map (Visual-Inertial SLAM)

Roadmap item 19 flew the mission out and back and measured the drift of a
doubled path: 0.40 to 4.10 m at the end of 854 to 1467 m on eighteen camera
flights, and r689 flew into the launch platform with the estimate 4.07 m off.
Where the vehicle returns to ground it has seen, the drift is now bounded by
registering the depth against a map the estimator keeps.

The map is the lidar-inertial estimator's: cells of points with a plane fitted
through each and a point-to-plane registration against them, moved into one
component with two consumers (`PointPlaneMap3D`,
`include/drone_city_nav/point_plane_map_3d.hpp`; the lidar replays are
byte-identical after the move). The camera estimator thins the depth cloud to
surfaces within 6 m in 0.4 m cells, at most 3000 points, and lays it into the
map by its corrected pose; every 0.5 s it registers the current depth against
the cells laid more than 20 s before, never against what it has just laid,
which carries the drift of the pose it was laid from. A registration counts
when most of the depth matched old cells (0.3 and at least 150 points), the
fit is tight (0.25 m) and an axis carries information per point above the
lidar estimator's floor.

The correction is an offset from the filter's frame to the map's, kept
outside the filter: the filter's position is certain to a few centimetres
while its drift is metres (clone deviation 0.05 m against innovations of 0.4
to 1.0 m on r691), and a Kalman step against it passed only innovations under
half a metre. A registration moves the target offset by a fifth of what it
measured along the axes it observes, by at most 0.1 m, within 1.5 m of its
prior; where the depth fixes the heading with the translation left free, the
target heading moves by a fifth too, by at most 0.2 degrees, within 5
degrees. The offset the autopilot and the navigation receive follows the
target at 0.2 m/s and 1 degree per second, turned about the vehicle: the
autopilot's fusion of the external position takes motion and may refuse a
jump. At a half of the measurement per registration the target moved 0.2 to
0.35 m and half a degree between registrations a second apart (r709), where
the drift it corrects accrues millimetres per second.

Measured on the acceptance of 2026-09-27 (bf92e952, r721 to r726): the error
at the end of 1014 to 1141 m is 0.20 to 0.68 m, the true position at the start
0.40 to 1.06 m, and the heading's drift is taken back to about zero; the frame
costs 53 ms at the median (54 before), the stereo process 0.1 to 0.2 more
cores and 90 MB of map. A first pass through new ground drifts as before, 0.1
to 0.4 percent of the path: nothing old lies there to register against. The
lidar estimator is unchanged: its own submap bounds it, 0.15 to 0.30 m at the
end of the doubled path (r727 to r731).
