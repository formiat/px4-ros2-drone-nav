# Step 3. The Mission And The Pipeline

What happens between a sensor's reading and a command to the autopilot, in
the order the data travels. Each stage names its node; the
[map of the architecture](architecture_map.md) gives the files, and
[navigation_pipeline.md](../navigation_pipeline.md) the full reference.

## The Mission

A mission is a start and one or more goals, a scenario file
([missions.md](../missions.md), [scenarios.md](../scenarios.md)). The
mission monitor (`mission_monitor_node`) hands the planner the current goal,
acknowledges a goal when the vehicle is inside its capture radius (V6), and
ends the mission with a landing (K30). Two missions are flown as acceptance
(V4): point to point, and cooperative traffic of several vehicles. A third
kind chooses its own goals: the survey (`survey_mission_node`, F20, K59)
looks at a volume frontier by frontier until nothing unseen is left, and
its memory can be saved as a map (K67).

An operator can redirect the vehicle on one channel (K31): a new goal, home,
land. The pilot arms and chooses the mode; the stack never does (K29).

## 1. Where Am I

The estimators turn the sensors into a pose
([localization.md](../localization.md)):

- on the stereo set, a visual-inertial filter tracks corners in the pair
  and fuses them with the IMU; a magnetometer holds its heading (K60), its
  own map holds its position where it sees old ground again (K57), and a
  place seen again closes a loop (K92);
- on the lidar set, each scan is registered against a submap of the last
  scans, with the IMU between (K51); where a tunnel gives the scans nothing
  to hold on to along its axis, the estimator says so (K70).

The pose goes to the autopilot as external odometry, and the stack flies by
the autopilot's fused pose.

## 2. What Is Around Me

The obstacle memory (`obstacle_memory_3d_node`) writes what the sensors
measured into one grid of voxels: a hit closes a voxel, a ray that passed
through opens the ones before it
([obstacle_mapping.md](../obstacle_mapping.md)). What was never looked at
stays unknown, which is not the same as dark or as empty (F6), and the
memory does not forget with time (K10). Warm bodies that move are kept in a
layer of their own and followed (K65, K66).

## 3. Which Way

The planner (`production_mppi_node`) runs one tick at a time
([trajectory_optimization.md](../trajectory_optimization.md),
[replanning.md](../replanning.md)):

1. **A route** through the memory's free and unknown space to the goal, kept
   and repaired as the memory changes rather than searched anew.
2. **A controller** that samples many short futures along the route and
   keeps the best one (MPPI); the same rollout model runs on a CPU, on CUDA
   and on Vulkan (K54).
3. **A speed** the laws allow: no faster than the vehicle can stop inside
   what the forward sensor has seen (K9), slower near a surface, slower by a
   warm body.

## 4. A Piece Of Trajectory At A Time

The planner does not steer the vehicle. It publishes an **execution
horizon**: a short, finite path that ends at rest, checked once more against
what the sensors say now. The offboard node (`mppi_offboard_node`) flies an
accepted horizon and nothing else ([drone_control.md](../drone_control.md));
when no new horizon comes, the vehicle is already on a path that stops.
[Step 4](04_safety.md) is about what follows from that.

## Following One Tick In A Flight's Log

Every stage prints one line per event with a fixed name, so a flight can be
read with `grep` ([diagnostics.md](../diagnostics.md)). In the stack's log
of a run, `log/runs/RUN/ros_drone_nav.log`:

- `LIDAR3D_CURRENT_SCAN` or `STEREO_DEPTH`: a scan or a depth frame arrived;
- `PRODUCTION_MPPI_TICK`: one tick of the planner, with the ages of the
  pose and of the observation it used;
- `EXECUTION_HORIZON published=true` and, from the offboard node,
  `EXECUTION_HORIZON accepted=true`: a horizon handed over and taken;
- `SURVEY_FRONTIER`, `SURVEY_DONE`: the survey's decisions.

[Step 5](05_testing_debugging.md) reads a real flight this way.

Next: [what safety means here](04_safety.md).
