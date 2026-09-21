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
| The nearest pass to true occupancy is closer on the vision memory than on the lidar's. | 0.41 to 0.75 m centre to centre of 0.5 m voxels against 0.84 m | (a) | item 14 |
| The braking contract's forward range is a configured constant (6.4 m in `sensor_profile.py`) and nothing lowers it when the pair returns nothing. The one reduction in the code, `min(guaranteed_detection_range_m, observed_range_m)`, applies to a motion the vehicle does not face and reads memory, not the sensor. A blind pair is therefore flown at the speed a healthy one admits, which is a hole in the first requirement at full illumination, not only in the dark. The repair makes the range a measurement of the recent frames, judged for the frame as a whole and not per ray, and moves the contract's central input, so both acceptance series are re-flown and the speed figures of items 14 and 16 re-measured. | the pair returns 14 172 to 74 099 points per frame within one lit flight (r579, r583) and the admitted speed does not move | (b) | item 17 |
| The simulated cameras carry no noise model and the location carries no lamp (the urban world has no `<light>` element; all of its illumination is `<scene><ambient>0.1 0.1 0.1</ambient>`), so every flight so far was lit by a constant that no failure can touch. Adding the noise lowers the measured confident depth and moves the speed baseline with it, which is why it waits for a series of its own. | the confident depth 6.4 m was measured with no noise on the imager (item 14 stage 1) | (c) | item 17 |

## Planning, Execution And Control

| Debt | Measured | Class | Found by |
|---|---|---|---|
| Route availability and ordinary no-route holds miss their reference figures on the lidar profile; what remains is the recovery after a physical block at a surface, 0.3 to 1.2 s without a route each time. | availability 93.4 to 96.7 percent against 97, holds 3.0 to 6.8 percent against 3 (r584 to r588) | (a) | items 10 and 12 |
| The planning loop runs near 43 Hz with most ticks over the 20 ms deadline and no single bottleneck left; the planner spends its whole 150 ms budget, so its p95 measures the configuration. | tick 22 to 23 ms at p50, 30 to 37 ms at p95 | (b) | item 10 |
| A holding vehicle drifts farther than the clearance its rest pose keeps beyond the body; three laws that released the margin were flown and reverted. | 0.37 m at p95 against 0.27 m | (a) | item 10 |
| A path validation beside a wall the envelope touches still costs several times the median, and the assembly's 12 ms budget is checked between attempts, not inside one. | 15 ms for one path against a thinned scan (487 ms before), the longest tick 226 ms (r548) | (b) | item 16 |

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
