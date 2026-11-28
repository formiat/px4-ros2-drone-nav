# Related Work

A review of the open stacks nearest to this one (2026-11-13, by their READMEs
and documentation, not by their code: "not found in the README" is not "not in
the code"), what sets this project apart, where it lags, and what is worth
taking from whom.

## The Nearest Full Stacks

| Project | ROS, autopilot | What it has | Flown for real | Licence |
|---|---|---|---|---|
| Unified Autonomy Stack (NTNU) | ROS 1 and ROS 2 mixed; PX4 or ArduPilot | lidar, radar and IMU SLAM, Voxblox, a graph planner of exploration and inspection, NMPC, a CBF safety filter; smoke, fog and darkness in its paper | mines, tunnels | BSD-3 |
| MRS UAV System (CTU Prague) | ROS 1 stable, ROS 2 Jazzy in transition; PX4 | redundant estimators switched by health, an MPC tracker, a staged failsafe, a safety area | DARPA SubT, MBZIRC | BSD-3 |
| KR Autonomous Flight (UPenn) | ROS 1 | VIO or LIO, a map, planning, control, a state machine | yes | non-commercial research |
| SUPER (HKU MaRS) | ROS 1; PX4 | the ROG-Map and a fast safe planner with a backup trajectory | yes | none |
| CERLAB-UAV-Autonomy (CMU), Agipix (its ROS 2 port) | ROS 1 / ROS 2; PX4 | a map, planning, exploration, a detector of moving obstacles | yes | MIT / unclear |
| Fast-Drone-250 (ZJU) | ROS 1; PX4 v1.11 | VINS-Fusion, EGO-Planner, a 250 mm frame | yes | no commercial use |
| Agilicious (UZH) | ROS 1 | estimation, MPC/INDI, a safety module | yes | academic only |

Frameworks rather than autonomy: Aerostack2, AirStack (CI on a live
simulation), aerial-autonomy-stack (Gazebo Harmonic faster than real time),
px4-ros2-interface-lib (external PX4 modes over uXRCE-DDS). Components:
FAST_LIO, ROG-Map, EGO-Planner, FUEL, GBPlanner, voxblox, MIGHTY.

Tools beside the stacks (read on 2026-11-20, by their pages, not run here):
LidarView (Kitware; Apache-2.0, on ParaView) receives, records and shows
lidar data, reads `.mcap` recordings since 5.0 and subscribes to ROS 2
topics and publishes them since 5.3. A flight's record is opened here by
Lichtblick, which shows the whole flight and not the lidar alone (roadmap
item 42); LidarView was not weighed for it. Kitware's lidar SLAM (LidarSlam,
`gitlab.kitware.com/keu-computervision/slam`; a BSD licence by its authors'
post of 2023-02-17, the README names none) has a ROS 2 wrapper (Humble,
Iron; images for Humble and Jazzy), takes an IMU, GPS and cameras beside the
lidar, and closes loops where it is built with TEASER++ and optimises a pose
graph where it is built with g2o; its README says the loop closure detection
is not yet supported in the ROS 2 wrapper. A candidate reference for the
lidar set's half of roadmap item 41, and since the measurement of
2026-11-26 (the lidar set needs a closure in the held-out worlds:
[localization.md](localization.md)) the second of the three ways named
there, behind a closure of our own over the graph the stereo set has.

Read on 2026-11-28, by their pages, none built or run here:

- **lidar_slam** (`rsasaki0109/lidar_slam`; BSD-2-Clause, its default
  build without GPL parts; C++ with Python entry points; Humble and Jazzy):
  a bag turned into a point-cloud map. The front end is RKO-LIO (MIT), the
  back end `graph_based_slam` on g2o with loop closures by descriptors (Scan
  Context built in; BEV, SOLiD and triangle descriptors; 3D-BBS to verify,
  optional). Shown on Ouster, Livox and Velodyne bags (HILTI 2022, Oxford
  Spires, NCLT, KITTI, GEODE's tunnels, where by its page it did not fail
  and was "not the most accurate"). For this project: a second candidate
  reference for the lidar set's half of roadmap item 41 beside LidarSlam,
  and the nearest reading for the recognition such a closure needs: K93
  found the estimator's own health rule no gate (it accepts 95 to 100
  percent of pairs of different places), and a scan's descriptor verified
  by a registration is the usual answer; its licence allows code to be
  taken.
- **PteroSim** (PteroLabs; proprietary, by its author free for personal,
  academic and non-commercial use, sessions of two hours on the free tier;
  open beta): Unreal Engine 5 with JSBSim's dynamics as a physics and
  rendering backend for PX4 and ArduPilot SITL (the simulator MAVLink API in
  lockstep), a Python API over gRPC; IMU, magnetometer, barometer, GPS and
  a camera, a lidar and a radar "in development" by its author's post; PX4
  lists it as community supported. Not for this project today: no lidar, no
  ROS 2 interface named, and a licence under which the worlds and the
  acceptance could not be published for others to run (roadmap item 57). A
  renderer to remember if photorealistic frames are ever needed for the
  estimator.
- **TFD-SfM** (Chui, Andrade-Loarca, Cremers; arXiv 2505.20858, for WACV
  2027; code "coming soon"): structure from motion over an unordered,
  uncalibrated set of pictures from a cold start, every dense match a 3D
  Gaussian whose variance is optimised with the geometry. Offline (tens of
  thousands of iterations) and built on learned matchers (LoFTR, DKM). Not
  for the vehicle: the estimators here are online, calibrated and without a
  network; a possible offline tool for a map from a recorded flight's
  pictures, which nothing here needs yet.
- **ros2_rust** (rclrs 0.8): roadmap item 56.

Read on 2026-11-22, by their READMEs, neither built here:

- **visloc-rs** (`rsasaki0109/visloc-rs`; MIT or Apache-2.0; Rust, begun in
  2026-05): structure from motion, a stereo-inertial SLAM that is a port of
  Basalt with a keyframe mapper that closes loops, a pose graph,
  localisation against a saved map, and Gaussian splatting from photos and
  360-degree video. No C++, OpenCV or CUDA to build; the GPU through wgpu,
  optional. Places are recognised without a network (VLAD with LSH over
  SIFT or ORB descriptors); learned features are an option. Three ROS 2
  nodes over RustDDS, by its README tested RustDDS to RustDDS and "not yet
  against a ROS 2 install", and "not yet tried on a real robot"; no lidar.
  Its own figures, not a leaderboard's: better than ORB-SLAM3 on 9 of 11
  EuRoC sequences, 0.057 m and 0.072 m on MH_03 and MH_05 with the closure.
  For this project: a candidate third reference for the stereo set's half
  of roadmap item 41 and a reading for its stage 1; an example for item 56.
  Its "street walked with a camera into a simulator" is a splat that
  pictures are rendered from, with no collisions: the sensors here (a
  lidar, a depth pair, time of flight) need geometry, and Gazebo does not
  render splats, so it is not a source of worlds.
- **GSeg3D** (`dfki-ric/ground_segmentation` and its wrapper
  `ground_segmentation_ros2`; DFKI Robotics Innovation Center; BSD-3; C++
  on PCL and nanoflann; Humble and Jazzy): a lidar scan split into ground
  and the rest on a grid in two passes, coarse and then fine, with an
  optional IMU for the gravity's direction; on the processor alone. Its
  authors' figures on SemanticKITTI: a precision of 96 to 99 percent, a
  recall of 85 to 91, about 48 ms a scan; dense vegetation is its named
  weakness. Not for the vehicle that flies: the memory here is
  three-dimensional and a floor is an obstacle like a wall, and 48 ms is
  most of a planner's tick. A candidate for the support layer of a ground
  platform (roadmap item 60).

## What Sets This Project Apart

1. A full stack on ROS 2 with PX4 over uXRCE-DDS: nearly every stack that flies
   without GNSS on its own estimation and planning is a ROS 1 one.
2. MPPI on the GPU (and on the CPU and Vulkan) in a flying stack.
3. A persistent incremental 3D planner of the D* Lite kind.
4. Estimator failures, darkness, smoke, the return home without GNSS.
5. Statistical acceptance by series of flights in the simulator.
6. A permissive licence (MIT).

## Where It Lags

- No real flights: every stack above has flown; this one is proven in
  simulation and on recorded data (EuRoC, NTU VIRAL).
- No reviewed publication behind it (roadmap item 58), and no acceptance
  another stack can be flown through (roadmap item 57).
- No radar for smoke (cameras, time of flight, a thermal core).
- No swarm (roadmap item 15).

The honest wording: the most complete open stack of GNSS-denied autonomy on
ROS 2 and PX4, proven in simulation.

## The Licence Rule

This project is MIT. Code may be taken, with its copyright notice kept, from
BSD and MIT projects (MRS, NTNU, Aerostack2, AirStack, CERLAB,
px4-ros2-interface-lib, voxblox, FASTER, MIGHTY, GSeg3D, visloc-rs). From GPL projects (FAST_LIO,
ROG-Map, EGO-Planner, FUEL), non-commercial ones (KR, Fast-Drone-250,
Agilicious) and those without a licence (SUPER, RACER) only ideas are taken,
in an implementation of our own. GBPlanner's "civilian use only" is not a
clean BSD: no code from it. Even from a permissive project an idea and a test
are usually the better thing to take than code.

## What To Take, In Order

1. The stack registered as an external PX4 mode (px4-ros2-interface-lib): the
   mode in QGroundControl, the pilot's switch, PX4's failsafes aware of it
   (roadmap item 44).
2. A staged failsafe (hover, emergency landing, landing without an estimator,
   motors off) and a manager of estimators (MRS) (items 44 and 39).
3. A safety filter between the MPPI horizon and the offboard that corrects a
   command by the distance field instead of refusing the horizon (NTNU's CBF)
   (item 44).
4. A short flight in CI (AirStack) (item 46).
5. A backup trajectory that always stops inside known free space while the
   main one may enter the unknown (SUPER, the idea alone) (item 53).
6. This page, linked from the README.
