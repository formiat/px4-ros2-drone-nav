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
lidar set's half of roadmap item 41.

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
px4-ros2-interface-lib, voxblox, FASTER, MIGHTY). From GPL projects (FAST_LIO,
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
