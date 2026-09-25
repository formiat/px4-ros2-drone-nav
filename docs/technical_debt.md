# Technical Debt

The one register of what is known to be wrong or unfinished and has been set
aside. The project owner's rule of 2026-09-19: whatever a series shows to be
wrong is fixed with its measured cause, and debt may hold only what is

- **(a)** very hard,
- **(b)** in need of a deep rework of the code, or
- **(c)** in need of the project owner's decision,

and every entry says which. An entry leaves this file when it is repaired or
when the owner decides it is not wanted; the roadmap item that found it keeps
the history. Figures are from the flights named; none of them fails a flight
unless the entry says so ([testing.md](testing.md)).

## Localization

| Debt | Measured | Class | Found by |
|---|---|---|---|
| The visual-inertial estimate drifts without bound: an odometry with no loop closure. The goal-in-truth check leaves 0.6 m of margin on the worst accepted flight, and a mission several times longer will exceed the 2.0 m capture radius. The remedy is long-lived points in the filter's state or relocalization against a map. | 0.1 to 0.4 percent of the path; the true position 0.47 to 1.42 m from the goal at its acknowledgement (r579 to r583), 1.97 and 2.31 m before the gyroscope noise was corrected (r572, r575) | (b) | item 16 |
| The controller closes the navigation for the rest of the flight when the autopilot resets its position by more than 0.33 m (the rule roadmap item 13 made). With an odometry as the only position such a reset is possible; the vehicle then holds until the flight's window ends, which fails "always reaches its goal". Not seen again since the autopilot fuses the poses with 0.3 m of noise. What the stack should do after such a reset (hold and re-anchor, discard the map near the vehicle, land) is a policy. | r561: a 0.4 m correction, the odometry rejected, 1.8 s on the IMU alone, a reset of 1.19 m, mission incomplete | (c) | item 16 |
| The autopilot's position estimate follows the odometry, so its error across the track against the true pose is the odometry's drift; the 0.35 m reference figure was measured on GNSS and is printed as a note on every camera flight. Whether the figure is re-derived for the odometry profiles or dropped is a decision. | 0.67 to 1.19 m at p95 in four of five flights (r579 to r582) | (c) | item 16 |
| No reference visual-inertial system has been run on the recorded flights. OpenVINS without ROS needs Ceres, which builds from source in the container, and Boost.Filesystem, which the container image does not carry, and then a runner for these records. It is a tool and no criterion; adding the package to the image is a decision about the image. | attempted 2026-09-20, stopped at configuration | (c) | item 16 |
| The autopilot's IMU reaches the estimators over a best-effort transport at 83 to 92 Hz and loses bursts when the host stalls. The filter grows its uncertainty over a hole and recovers; a reliable or higher-rate IMU path is a change of the autopilot's transport configuration. | holes of 0.33 to 0.69 s under a stalled host (r547, r549), at most 0.1 s otherwise (r550) | (b) | item 16 |

## Speed

| Debt | Measured | Class | Found by |
|---|---|---|---|
| The lidar profile's mean flight speed has little margin over its requirement; a flight under it fails. Most of what it loses is no-route holds at surfaces. | 2.42 m/s against 2.4 (r587); 2.42 to 2.67 over r584 to r588 | (a) | items 9 and 16 |
| The mean flight speed is measured on the wall clock, so a simulation slower than real time, from foreign load on the host, lowers it by as much; such flights are excluded by hand (r553, r554, r577, r578). Measuring it on the simulation clock changes what the requirement measures, which the owner fixed. | real-time factor 0.78 under a browser and a GPU job against 0.93: 1.22 m/s on the wall clock for 1.53 m/s of simulation time (r577) | (c) | item 16 |
| The heaviest configuration, the stereo pair rendered beside the lidar, does not hold real time on the reference workstation, so its speeds are not comparable with either sensor set's. | real-time factor 0.83; 1.9 to 2.4 m/s (r566 to r570) | (a) | item 16 |

## Camera Perception

| Debt | Measured | Class | Found by |
|---|---|---|---|
| A faced motion whose elevation lies between the pair's vertical field and a time-of-flight cone is seen by no sensor and is flown at the unobserved speed. Closing the gap is a sensor on the vehicle or a wider field. | 52.4 to 67.5 degrees of elevation, flown at 1.0 m/s | (c) | item 14 |
| Lateral deviations of the horizon from the route enter unknown space at the validators' speed; under the lidar that space had already been seen. | not measured as a separate figure | (b) | item 14 |
| The sensor evidence age exceeds the 600 ms the braking contract charges on a small share of ticks of the camera profile. | up to 628 ms on one flight of five after stage 0 (r541), 624 to 948 ms before it | (a) | items 14 and 16 |
| The other half of the entry above: the contract charges that age as a constant and never reads the real one, although the stack computes it. `latestSensorEvidenceFreshness` hands the measured age and a freshness flag to the validator, which drops a stale scan from the points it checks the body against, so "will I hit what I can see" is answered on measurement and "how fast may I fly" on a configured 600 ms. The two consumers of the one pair therefore time out asymmetrically: the estimator goes unhealthy 1.0 s after the last visual update (`maximum_unaided_s`) and stops publishing, which withdraws the autopilot's position, while the speed law goes on admitting what 6.4 m and 600 ms admit. The window between the two, where the charged age is already exceeded and the pose is dead reckoning that is still declared healthy, has never been flown on purpose. | 0.6 s charged against a 1.0 s unaided timeout; 2.4 m of travel at 2.45 m/s against a 2.0 m margin | (b) | item 17 |
| The nearest pass to true occupancy is closer on the vision memory than on the lidar's. | 0.41 to 0.75 m centre to centre of 0.5 m voxels against 0.84 m | (a) | item 14 |
| The braking contract's forward range is a configured constant (6.4 m in `sensor_profile.py`) and nothing lowers it when the pair returns nothing. The one reduction in the code, `min(guaranteed_detection_range_m, observed_range_m)`, applies to a motion the vehicle does not face and reads memory, not the sensor. A blind pair is therefore flown at the speed a healthy one admits, which is a hole in the first requirement at full illumination, not only in the dark. The repair makes the range a measurement of the recent frames, judged for the frame as a whole and not per ray, and moves the contract's central input, so both acceptance series are re-flown and the speed figures of items 14 and 16 re-measured. | the pair returns 14 172 to 74 099 points per frame within one lit flight (r579, r583) and the admitted speed does not move | (b) | item 17 |
| The simulated cameras carry no noise model and the location carries no lamp (the urban world has no `<light>` element; all of its illumination is `<scene><ambient>0.1 0.1 0.1</ambient>`), so every flight so far was lit by a constant that no failure can touch. Adding the noise lowers the measured confident depth and moves the speed baseline with it, which is why it waits for a series of its own. | the confident depth 6.4 m was measured with no noise on the imager (item 14 stage 1) | (c) | item 17 |
| The confident depth of 6.4 m was measured on the photogrammetric surfaces of one location, the only textured world the project renders; the collision-only world carries no textures and a camera sees nothing on it, so the two extremes have been flown and nothing between them. A surface a matcher cannot match — blank paint, poured concrete, a large uniform panel — is fully lit, carries all the signal an imager wants and returns no depth or an interpolated surface that is not there. What the confident range becomes on such a surface has never been measured, and measuring it is likely to lower the range and move the speed baseline with it. | not measured | (c) | item 17 |

## Planning, Execution And Control

| Debt | Measured | Class | Found by |
|---|---|---|---|
| Route availability and ordinary no-route holds miss their reference figures on the lidar profile; what remains is the recovery after a physical block at a surface, 0.3 to 1.2 s without a route each time. | availability 93.4 to 96.7 percent against 97, holds 3.0 to 6.8 percent against 3 (r584 to r588) | (a) | items 10 and 12 |
| The planning loop runs near 43 Hz with most ticks over the 20 ms deadline and no single bottleneck left; the planner spends its whole 150 ms budget, so its p95 measures the configuration. | tick 22 to 23 ms at p50, 30 to 37 ms at p95 | (b) | item 10 |
| A holding vehicle drifts farther than the clearance its rest pose keeps beyond the body; three laws that released the margin were flown and reverted. | 0.37 m at p95 against 0.27 m | (a) | item 10 |
| A path validation beside a wall the envelope touches still costs several times the median, and the assembly's 12 ms budget is checked between attempts, not inside one. | 15 ms for one path against a thinned scan (487 ms before), the longest tick 226 ms (r548) | (b) | item 16 |
| The persistent planner can livelock while the world keeps changing. On r596 (2026-09-25, camera profile, f4c45394) the vehicle stood 106 s at (5.5, 21, 17) m under the blocked-route law of a stale route (generation 271 never replaced) while every update reported `repair_processed=0`, `expansions=0`, `repair_pending=681`, the feasibility search restarting (13 restarts, 100 684 records, 61 375 labels invalidated per pass, closest approach to the goal 36 m) and the queued continuation retired after 0.03 ms, because the raw world advanced a revision on nearly every tick (3161 to 4132 over the stall) and no stage ever got a stable world to finish on; the flight timed out 45 m from the goal after an 816 m excursion around the southern pocket near it. The same telemetry on the ten comparable flights (r579 to r583, r589 to r595) shows zero-progress streaks of at most 0.7 s, so the mechanism is old and its trigger rare: a closest-approach incumbent, a large label set after a long excursion, and occupancy churn while hovering. The repair is the planner's budget scheduling between repair, feasibility search and world changes, and route stall recovery (`route_stall_recovery_enabled`, off) is the existing lever to measure against it. | one flight in eleven; 106 s of zero progress; mission incomplete | (b) | speed goal 2026-09-24 |
| The obstacle memory clears an occupied voxel only when a free ray passes through it again (`miss_weight`, the Schmitt trigger at `free_score`), so a voxel never re-observed stays occupied for the rest of the flight, and nothing distinguishes an occupancy confirmed by a thousand scans from one that collected two hits and vanished: the planner treats both as a wall. A person crossing the frame is a route replacement; a trail left where the vehicle does not return is a wall until landing; smoke is the case rays cannot clear at all while it lasts. Found from two sides by two readers of v0.4.0 (a walking person, smoke). `forgetDynamicVolumes` covers only the cooperative peers whose positions arrive by intent. | not measured as a figure; the stack has never flown a moving body or a plume | (b) | item 18 |

## Repository

| Debt | Measured | Class | Found by |
|---|---|---|---|
| The 2D obstacle memory node is still selectable by the launch files. | one node and its launch branch | (c) | item 10 |
| Fourteen sources sit near the 1000-line cap and most of the package lies flat in `src/`. | `swept_footprint.cpp` at 994 non-blank lines | (b) | item 10 |

## Not Flight-Verified

Not debt by the rule above, but unproven and recorded here so that a green
series does not hide it: the multi-vehicle missions have not been flown on the
camera defaults, nor without GNSS, nor since the interception missions were
removed (roadmap item 15 waits for them); the cave and the finals locations
are imported and load but have not been flown; the point-to-point mission has
been flown from one start to one goal.

The chain that a blinded vehicle would take has never been exercised deliberately: the estimator reports itself unhealthy and stops publishing, the autopilot ends its external-vision fusion after 200 ms, its position ages out, the controller revokes the execution authority and the offboard path holds. r561 walked part of it by accident (the autopilot rejected the odometry, dead-reckoned 1.8 s and reset its position by 1.19 m) and the flight did not recover. Until roadmap item 17 injects the failure, no flight has tested degraded or absent illumination at all.
