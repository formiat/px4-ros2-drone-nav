# A Map Of The Architecture

Every component of the stack, where its code lives and which row of the
[specification](../specification.md) sets it. The map follows the data: what
the sensors and the simulator give, how the vehicle estimates its pose, what
it remembers of the obstacles, how it plans, how the plan is executed and
kept safe, what the mission asks, and the tools round it. The steps of the
guide start at [01_basics.md](01_basics.md); the reference behind this map is
[architecture.md](../architecture.md) and
[navigation_pipeline.md](../navigation_pipeline.md).

The idea of a map whose every component links to its code is hundong2's
([forks.md](../forks.md)); the text is written here, on today's code.

The names in `code type` are executables of the package
([CMakeLists.txt](../../drone_city_nav/CMakeLists.txt)) or nodes the
single-vehicle launch starts
([city_nav.launch.py](../../drone_city_nav/launch/city_nav.launch.py)). A
script test (`scripts/tests/test_guide_links.py`) holds every link of this
page to a file that exists, every row it names to the specification, and
every executable and launched node to this page.

## 1. The Sensors And The Simulator

What flies in the simulator is PX4's software-in-the-loop autopilot and a
Gazebo model with one of two sensor sets (V1): the stereo pair with the
time-of-flight ring, or the 3D lidar. Nothing below this layer exists on a
real vehicle; everything above it is the stack.

| Component | Code | Specification |
|---|---|---|
| The launch: which nodes start for which sensor set and localization profile | [city_nav.launch.py](../../drone_city_nav/launch/city_nav.launch.py), [launch_arguments.py](../../drone_city_nav/launch/launch_arguments.py), [sensor_profile.py](../../drone_city_nav/launch/sensor_profile.py), [lidar_profile.py](../../drone_city_nav/launch/lidar_profile.py), [localization_profile.py](../../drone_city_nav/launch/localization_profile.py) | V1, V2 |
| The flight script: the world, the autopilot, the bridges, the stack, the records of a run | [run_drone_nav_sim.sh](../../scripts/run_drone_nav_sim.sh), [container_run.sh](../../scripts/container_run.sh), [Makefile](../../Makefile) | A5, A8 |
| The bridges between Gazebo and ROS 2 (`parameter_bridge` of `ros_gz_bridge`) and the fixed frames (`static_transform_publisher`) | [city_nav.launch.py](../../drone_city_nav/launch/city_nav.launch.py), [sensor_extrinsics.py](../../drone_city_nav/launch/sensor_extrinsics.py), [sensor_extrinsics.yaml](../../drone_city_nav/config/sensor_extrinsics.yaml) | K22, K61 |
| The stereo pair read from Gazebo: `gazebo_stereo_depth_node`, one process for the depth and one for the estimator | [gazebo_stereo_depth_main.cpp](../../drone_city_nav/src/gazebo_stereo_depth_main.cpp), [stereo_depth_node.cpp](../../drone_city_nav/src/stereo_depth_node.cpp) | K33, A12 |
| The contact and the vibration plugins of the vehicle model | [drone_contact_system.cpp](../../drone_city_nav/src/drone_contact_system.cpp), [drone_vibration_system.cpp](../../drone_city_nav/src/drone_vibration_system.cpp), [models](../../drone_city_nav/models) | A9, K52 |
| `collision_crash_node`: a contact of the vehicle with a surface is a crash | [collision_crash_node.cpp](../../drone_city_nav/src/collision_crash_node.cpp) | R1, A9 |
| `simulation_truth_adapter_node`: the simulator's truth, for the checks alone | [simulation_truth_adapter_node.cpp](../../drone_city_nav/src/simulation_truth_adapter_node.cpp) | I5 |
| `simulation_smoke_sensor_node`: smoke as a model the sensors read | [simulation_smoke_sensor_node.cpp](../../drone_city_nav/src/simulation_smoke_sensor_node.cpp), [simulation_smoke_model.cpp](../../drone_city_nav/src/simulation_smoke_model.cpp) | A13 |
| `simulation_heading_source_node`: a heading for the profiles that take one | [simulation_heading_source_node.cpp](../../drone_city_nav/src/simulation_heading_source_node.cpp) | V2 |
| The real vehicle's imperfections: the IMU stream's jitter, the cameras' faults, the light | [imu_stream_faults.py](../../scripts/imu_stream_faults.py), [camera_stream_faults.py](../../scripts/camera_stream_faults.py) | K52, F21, K28 |
| `budget_relay_node`: each node's output delayed by its measured budget | [budget_relay_node.cpp](../../drone_city_nav/src/budget_relay_node.cpp), [budget_relay.py](../../drone_city_nav/launch/budget_relay.py) | K53 |
| The autopilot's parameters, the simulation's and the vehicle's | [px4](../../drone_city_nav/config/px4), [setup_px4_autopilot.sh](../../scripts/setup_px4_autopilot.sh) | K34, V5 |

## 2. The Estimate Of The Pose

Two estimators, one a sensor set; neither reads a satellite signal by
default (V2). Their poses go to the autopilot as external odometry, and the
stack flies by the autopilot's fused pose.

| Component | Code | Specification |
|---|---|---|
| The stereo visual-inertial filter (a stereo MSCKF) | [visual_inertial_odometry.hpp](../../drone_city_nav/include/drone_city_nav/visual_inertial_odometry.hpp), [visual_inertial_odometry.cpp](../../drone_city_nav/src/visual_inertial_odometry.cpp), [stereo_feature_tracker.cpp](../../drone_city_nav/src/stereo_feature_tracker.cpp) | K8, K21, K28 |
| Its node, inside the camera process | [visual_inertial_odometry_node.cpp](../../drone_city_nav/src/visual_inertial_odometry_node.cpp) | K24 |
| The magnetometer's heading and the barometer's height as priors | [visual_inertial_odometry.cpp](../../drone_city_nav/src/visual_inertial_odometry.cpp) | K60, K58 |
| The registration of the depth against the vehicle's own map | [stereo_map_registration.cpp](../../drone_city_nav/src/stereo_map_registration.cpp) | K57 |
| The loop closure: keyframes, the recognition's gates, the pose graph | [stereo_loop_closure.cpp](../../drone_city_nav/src/stereo_loop_closure.cpp), [pose_graph_4dof.cpp](../../drone_city_nav/src/pose_graph_4dof.cpp) | K92 |
| The void: a frame with nothing in the light's reach | [stereo_void_frame.cpp](../../drone_city_nav/src/stereo_void_frame.cpp), [visual_inertial_void.hpp](../../drone_city_nav/include/drone_city_nav/visual_inertial_void.hpp) | K46, K47 |
| The trust in the satellite signal on the `hybrid` profile | [gnss_trust.cpp](../../drone_city_nav/src/gnss_trust.cpp), [visual_inertial_gnss.cpp](../../drone_city_nav/src/visual_inertial_gnss.cpp) | K56 |
| `lidar_inertial_odometry_node`: scans registered against a submap of planes, the IMU between | [lidar_inertial_odometry.cpp](../../drone_city_nav/src/lidar_inertial_odometry.cpp), [lidar_inertial_odometry.hpp](../../drone_city_nav/include/drone_city_nav/lidar_inertial_odometry.hpp), [point_plane_map_3d.hpp](../../drone_city_nav/include/drone_city_nav/point_plane_map_3d.hpp), [lidar_inertial_odometry_node.cpp](../../drone_city_nav/src/lidar_inertial_odometry_node.cpp) | K51, K45, K70, K23 |
| Whether the lidar set needs a loop closure: a measurement | [localization.md](../localization.md) | K93 |

## 3. The Memory Of The Obstacles

One grid of voxels a quarter of a metre wide, written by measurement alone:
unknown space is free to fly (I1), a prohibition comes only from what a
sensor saw (I2), and nothing is forgotten by time (K10).

| Component | Code | Specification |
|---|---|---|
| `obstacle_memory_3d_node`: the scans and the depth frames into the grid, the snapshots and the deltas out | [obstacle_memory_3d_node.cpp](../../drone_city_nav/src/obstacle_memory_3d_node.cpp), [obstacle_memory_3d.cpp](../../drone_city_nav/src/obstacle_memory_3d.cpp), [observed_occupancy_grid_3d.cpp](../../drone_city_nav/src/observed_occupancy_grid_3d.cpp) | I1, I2, K10, K81 |
| The depth of the stereo pair and the time-of-flight ring | [stereo_depth_node_match.cpp](../../drone_city_nav/src/stereo_depth_node_match.cpp), [stereo_depth_returns.cpp](../../drone_city_nav/src/stereo_depth_returns.cpp), [tof_zone_returns.cpp](../../drone_city_nav/src/tof_zone_returns.cpp) | K7, K35, K72 |
| Darkness, smoke and the silent patch: what a frame that sees nothing means | [carried_light_judgment.cpp](../../drone_city_nav/src/carried_light_judgment.cpp), [stereo_void_frame.cpp](../../drone_city_nav/src/stereo_void_frame.cpp) | F6, K12, K14, K36, K44 |
| `thermal_core_node` and the moving layer: warm bodies, followed and looked at | [thermal_core_node.cpp](../../drone_city_nav/src/thermal_core_node.cpp), [obstacle_memory_3d_moving.cpp](../../drone_city_nav/src/obstacle_memory_3d_moving.cpp), [obstacle_memory_3d_look.cpp](../../drone_city_nav/src/obstacle_memory_3d_look.cpp), [moving_body_tracker_3d.cpp](../../drone_city_nav/src/moving_body_tracker_3d.cpp) | K39, K64, K65, K66, K87 |
| A survey's map as the memory's first content | [obstacle_mapping.md](../obstacle_mapping.md) | K67, K71 |
| The distance field the planner reads | [observed_esdf_3d.cpp](../../drone_city_nav/src/observed_esdf_3d.cpp) | K74 |

## 4. Planning

`production_mppi_node` holds the whole planning tick: the route through the
memory, the sampled controller along it, and the speed the laws allow.

| Component | Code | Specification |
|---|---|---|
| `production_mppi_node` and its tick | [production_mppi_main.cpp](../../drone_city_nav/src/runtime/ros/production_mppi_main.cpp), [production_mppi_node.cpp](../../drone_city_nav/src/runtime/ros/production_mppi_node.cpp), [production_mppi_node_planning_tick.cpp](../../drone_city_nav/src/runtime/ros/production_mppi_node_planning_tick.cpp), [planning_cycle_coordinator_3d.cpp](../../drone_city_nav/src/runtime/planning_cycle_coordinator_3d.cpp) | K24, K53 |
| The route: an incremental search over the memory's lattice | [persistent_dstar_lite_planner_3d.cpp](../../drone_city_nav/src/persistent_dstar_lite_planner_3d.cpp), [production_mppi_node_route_planning.cpp](../../drone_city_nav/src/runtime/ros/production_mppi_node_route_planning.cpp) | I1, I3, I4 |
| The controller: MPPI, one rollout model on three engines (CPU, CUDA, Vulkan) | [mppi_controller_3d.cpp](../../drone_city_nav/src/runtime/mppi_controller_3d.cpp), [mppi_cpu_engine.cpp](../../drone_city_nav/src/mppi/mppi_cpu_engine.cpp), [mppi_engine.cu](../../drone_city_nav/src/mppi/mppi_engine.cu), [mppi_vulkan_engine.cpp](../../drone_city_nav/src/mppi/mppi_vulkan_engine.cpp) | K48, K54 |
| The speed laws: how fast the sensor's range, the clearance and a body allow | [mppi_speed_policy.cpp](../../drone_city_nav/src/mppi_speed_policy.cpp), [mppi_speed_policy_stopping.cpp](../../drone_city_nav/src/mppi_speed_policy_stopping.cpp), [mppi_speed_policy_moving_body.cpp](../../drone_city_nav/src/mppi_speed_policy_moving_body.cpp) | V3, K9, K18, K27, K68, K69, K78 |
| The proof that a goal cannot be reached | [goal_reachability_proof_3d.cpp](../../drone_city_nav/src/goal_reachability_proof_3d.cpp) | K16, F15 |
| The configuration | [urban_mvp.yaml](../../drone_city_nav/config/urban_mvp.yaml), [production_mppi_config.cpp](../../drone_city_nav/src/runtime/production_mppi_config.cpp) | V3, V6 |

## 5. Execution And Safety

A plan becomes motion only as a horizon the offboard node accepted: a finite
path that ends at rest in space the memory has seen, with a promise of when
it will be renewed.

| Component | Code | Specification |
|---|---|---|
| The horizon: assembled, certified, committed against what the sensors say now | [execution_horizon_assembler_3d.cpp](../../drone_city_nav/src/runtime/execution_horizon_assembler_3d.cpp), [execution_supervisor_3d.cpp](../../drone_city_nav/src/execution_supervisor_3d.cpp), [production_mppi_execution_control.cpp](../../drone_city_nav/src/runtime/production_mppi_execution_control.cpp) | K82, K86, K89 |
| `mppi_offboard_node`: the horizon flown through the autopilot's offboard interface, or as a mode of PX4's own | [mppi_offboard_node.cpp](../../drone_city_nav/src/mppi_offboard_node.cpp), [mppi_offboard_node_horizon.cpp](../../drone_city_nav/src/mppi_offboard_node_horizon.cpp), [mppi_offboard_node_external_mode.hpp](../../drone_city_nav/src/mppi_offboard_node_external_mode.hpp) | K17, K29, K91 |
| The failsafe ladder: what the vehicle does as it loses the planner, the estimate, the autopilot's position | [mppi_offboard_node_state.cpp](../../drone_city_nav/src/mppi_offboard_node_state.cpp), [navigation_health_supervisor.cpp](../../drone_city_nav/src/navigation_health_supervisor.cpp) | K90, K19, K1, K77 |
| The landing at the mission's end | [mppi_offboard_node_state.cpp](../../drone_city_nav/src/mppi_offboard_node_state.cpp) | K30 |
| The two requirements everything is judged by | [specification.md](../specification.md) | R1, R2, I6 |

## 6. The Mission

| Component | Code | Specification |
|---|---|---|
| `mission_monitor_node`: the goals, their acknowledgement, the way home, the operator's commands | [mission_monitor_node.cpp](../../drone_city_nav/src/mission_monitor_node.cpp), [mission_monitor_node_return.cpp](../../drone_city_nav/src/mission_monitor_node_return.cpp), [mission_monitor_node_operator.cpp](../../drone_city_nav/src/mission_monitor_node_operator.cpp) | V6, K20, K26, K31, K84 |
| `survey_mission_node`: a volume looked at frontier by frontier | [survey_mission_node.cpp](../../drone_city_nav/src/survey_mission_node.cpp), [survey_mission.cpp](../../drone_city_nav/src/survey_mission.cpp), [survey_reading.cpp](../../drone_city_nav/src/survey_reading.cpp) | F20, K59, R9 |
| The scenarios: a start and its goals, proven inside the world | [config](../../drone_city_nav/config), [point_to_point_scenario.py](../../drone_city_nav/launch/point_to_point_scenario.py) | K63, V4 |
| Cooperative traffic: `cooperative_traffic_referee_node`, `cooperative_traffic_agent_node` | [cooperative_traffic_referee_node.cpp](../../drone_city_nav/src/cooperative_traffic_referee_node.cpp), [cooperative_traffic_agent_main.cpp](../../drone_city_nav/src/cooperative_traffic_agent_main.cpp), [cooperative_traffic.launch.py](../../drone_city_nav/launch/cooperative_traffic.launch.py) | V4, A2 |
| Several vehicles at once: `multi_vehicle_spectator_node`, `multi_vehicle_diagnostics_mux_node` | [multi_vehicle_spectator_main.cpp](../../drone_city_nav/src/multi_vehicle_spectator_main.cpp), [multi_vehicle_diagnostics_mux_main.cpp](../../drone_city_nav/src/multi_vehicle_diagnostics_mux_main.cpp), [multi_vehicle.launch.py](../../drone_city_nav/launch/multi_vehicle.launch.py) | A2 |

## 7. What A Person Sees

| Component | Code | Specification |
|---|---|---|
| `world_visualization_node` and `rviz2`: what the vehicle sees, never the world's truth | [world_visualization_main.cpp](../../drone_city_nav/src/world_visualization_main.cpp), [diagnostics.md](../diagnostics.md) | I5 |
| A flight's record opened without the stack | [diagnostics.md](../diagnostics.md) | K73 |
| A recorded flight as a video | [record_flight.sh](../../tools/record_flight.sh), [virtual_display.sh](../../tools/virtual_display.sh) | A11 |

## 8. The Tools

The tools read a flight's records; none of them is in the flight. The whole
list is [tools/README.md](../../tools/README.md).

| Component | Code | Specification |
|---|---|---|
| The gate before a commit | [gates.sh](../../tools/gates.sh), [testing.md](../testing.md) | A5 |
| Continuous integration | [ci.yml](../../.github/workflows/ci.yml) | A5 |
| One flight that counts, and a series | [fly_until_valid.sh](../../tools/fly_until_valid.sh), [world_series.sh](../../tools/world_series.sh), [fix_series.sh](../../tools/fix_series.sh), [host_verdict.py](../../tools/host_verdict.py) | A1, A3, A4, A7, A14, A15 |
| The mission check: what a flight must show | [validate_drone_nav_headless.py](../../scripts/validate_drone_nav_headless.py), [headless_runtime_evidence.py](../../scripts/headless_runtime_evidence.py) | R1, R2, R3, R4 |
| Reading a flight: the sweep, the estimate against the truth, the budget | [flight_sweep.py](../../tools/flight_sweep.py), [estimate_error.py](../../tools/estimate_error.py), [node_budget.py](../../tools/node_budget.py) | K53 |
| A flight's bag and its replay | [replay_stack.sh](../../tools/replay_stack.sh), [stack_bag_topics.py](../../scripts/stack_bag_topics.py) | K62 |
| A survey's map and its check against the world | [survey_map.py](../../tools/survey_map.py), [survey_map_check.py](../../tools/survey_map_check.py) | K67 |
| The loop closures against the truth | [loop_closures.py](../../tools/loop_closures.py), [ideal_closure.py](../../tools/ideal_closure.py) | K92, K93 |
| The estimators on real recordings | [tools/vio](../../tools/vio), [tools/lio](../../tools/lio) | K50 |
| The package's own tools: `vulkan_probe`, `mppi_cuda_benchmark`, `mppi_cpu_rollout_benchmark`, `mppi_engine_benchmark`, `generate_static_esdf_cache`, `free_space_topology_compiler`, `gnss_trust_replay`, `thermal_frame_bodies`, `stereo_depth_accuracy`, `voxelize_sdf_collisions`, `persistent_planner_probe` | [drone_city_nav/tools](../../drone_city_nav/tools) | K48, K54 |
| The container | [Dockerfile](../../docker/Dockerfile), [Dockerfile.tools](../../docker/Dockerfile.tools) | A5 |
