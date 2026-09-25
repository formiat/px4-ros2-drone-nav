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

## Decisions Of 2026-09-25

The project owner took the decisions the (c) entries below waited for. An
entry keeps its class until it is repaired; the decision is written into it
and here, with where the work lands.

| Entry | Decision | Lands in |
|---|---|---|
| Navigation closed after an autopilot position reset | The repair as seen: the map anchor shifts by the reset the autopilot reports (`delta_xy`, `delta_z` of `vehicle_local_position`), a hold of a tick or two, then the flight goes on; landing rejected as a policy because it fails "always reaches its goal"; proof by an injected reset of the external-vision pose. | roadmap item 17, stage 6 |
| The 0.35 m cross-track reference | A reference per localization profile: 0.35 m on GNSS (measured), 1.0 m on the odometry profiles, half the capture radius the note guards. | mission check |
| A reference visual-inertial system | Not now; the drift is already measured against truth. Built as a tool image (not the production one) when the unbounded-drift work starts. | with the drift rework |
| The uncovered elevation band | The survey turn (3df703ce) stays; route shaping is not pursued, because the band lives in shafts, where a diagonal has no room; what remains is a sensor. | done; remainder (c) |
| Camera noise and a lamp | Together, in item 17 stage 1: one move of the speed baseline, not two. | roadmap item 17, stage 1 |
| Surfaces the matcher cannot match | The measurement first: the panel world, one flight, the confident range against it; the behaviour is decided on the number. | roadmap item 17, stage 2 |
| The mean flight speed's clock | The simulation clock replaces the wall clock as the requirement's measurement; the thresholds 2.4 and 1.2 m/s stand, and the base series are re-expressed in simulation time when the check changes. Flights under foreign host load stay excluded regardless, by the quiet-host gate before the flight rather than by hand after it. | mission check, [testing.md](testing.md) |
| The chords of a wide fillet | Not on its own; `static_route_corner_curve_samples` 4 to 8 rides with the next change that needs a lidar series. | next lidar series |
| The 2D obstacle memory node | Removal of the node, its launch branch and its tests, once a grep shows no scenario selects it. | repository |
| The route-volume crossing heuristic | The check stays; as seen, one pass spans the first entry to the last exit, with a unit test on the r600 geometry. | mission check |

## Assessment Of 2026-09-25

Every entry, rated on the day the owner reviewed the register, so that the
picture as a whole can be read at once. The codes name the entries of the
tables below in their order (L localization, S speed, C camera perception, P
planning, R repository, N not flight-verified). "Repair as seen" is the
repair as it looked on that day, not an instruction: by the time an entry
is worked on, the view of it may have changed, and the register does not
bind it.

| Entry | Urgent | Important | Risk of the change | Risk of leaving it | Repair as seen on 2026-09-25 | Size | What it touches, what to expect |
|---|---|---|---|---|---|---|---|
| L1 unbounded odometry drift | not for 400 to 600 m missions; yes as they lengthen | high: the first requirement on long missions | high: the estimator's core, the frame already costs 55 ms | low now, high for a mission two or three times longer | in concept: long-lived points in the filter's state, or relocalization against a map; no design | XL | the estimator, the tick budget, both series; the error at the goal under 0.5 m whatever the length |
| L2 navigation closed after a position reset | no, not seen since the 0.3 m EV noise | high: on a repeat the flight holds to its timeout | medium: a recovery policy reaches the controller and the memory | medium: rare and fatal for the first requirement | decided, see above | S to M | the controller; a camera series |
| L3 the 0.35 m cross-track reference | no | low: noise in the reports | low | low | decided, see above | S | the mission check only |
| L4 a reference visual-inertial system | no | medium: without one, nobody knows how good the estimator is | low | low | decided, see above | M | the container image; a tool, not a criterion |
| L5 the IMU over a best-effort transport | no | medium: a real vehicle has another link | medium: the uXRCE transport and its QoS | low in simulation on a quiet host | in part: a reliable QoS or a higher rate | M | the estimators, both series; holes under 0.1 s under any load |
| S1 the lidar profile's small speed margin | no | medium: not the target profile, but its series flake | high: what remains is the physical blocks at surfaces, three laws already reverted | medium: any lidar series may fall on one flight | not known | L | the clearance laws, the route beside walls; a series minimum at 2.6 |
| S2 the speed measured on the wall clock | no | low | low | low: the quiet-host gate already covers it | decided, see above | S | the mission check |
| S3 the pair beside the lidar below real time | no | low: not a target configuration | low | low | not known: the host's power | none | comparability of the figures only |
| C1 the uncovered elevation band | no | medium: 2 percent directly, the shafts behind it | medium | low: slow but safe | decided, see above | done | the shafts; the remainder is a sensor |
| C2 the horizon's lateral deviations into unknown space | no | medium | medium: a tube about the route for the horizon | low: the validators hold the speed | not known, only the direction | L | the controller and the tube; the effect on speed unmeasured |
| C3 the evidence age above the charged 600 ms | medium | high: a hole in the first requirement, 0.12 m of 2.0 | low if done with C4 | low to medium | the measured age read into the contract | with C4 | the ticks with old evidence a little slower |
| C4 the contract never reads the measured age | medium to high: item 17 stage 0 | high: a window of 2.4 m of travel against a 2.0 m margin | medium: the contract's central input | medium: not seen in a lit world, to be seen in item 17 | item 17 stage 0 | M | both series anew, the camera speed a little lower |
| C5 the nearest pass closer on the vision memory | no | medium: the margin to a wall | medium | low to medium: no contact in twenty flights | not known | M to L | the camera profile's clearance |
| C6 the contract's range a constant, a blind pair flown as a healthy one | **high**: a hole at full illumination | **high**: the first requirement | medium to high: the contract's central input, the speed baseline moves | **high**: the pair can return nothing and the speed does not move | the range as a measurement of the recent frames (item 17 stage 0) | M to L | both series anew; the camera speed lower on ticks of weak depth |
| C7 no camera noise, no lamp | no | medium: realism | medium: the speed baseline moves | low in simulation | decided, see above | M plus series | the confident depth below 6.4 m, the camera speed lower |
| C8 textureless surfaces unmeasured | no | medium in simulation, high in reality | low for the measurement | medium in the real world | decided, see above | M to measure, XL to remedy | the contract's range |
| P1 lidar availability and holds off their figures | no | medium | high: physical blocks, three laws reverted | low | not known | L | with S1 |
| P2 the tick past its 20 ms, the planner spending its whole budget | no | low: it works | high: the tick's architecture | low | not known | XL | all of execution |
| P3 the hover drifts past the rest clearance | no | medium | high: three laws reverted | low: no contacts | not known | M to L | the rest clearance |
| P4 a path validation beside a wall, the budget checked between attempts | no | low | medium | low | the check inside an attempt | S to M | the horizon assembly |
| P5 the planner's stages unscheduled, the livelock closed by the probe deadline only | medium | high: the first requirement (a mission incomplete) | high: the planner | medium: a rare trigger, none in twenty flights after the repair | in part: a deadline on every stage, stall recovery as the lever | L | the planner, both series |
| P6 the recovery at 2 m/s^2 against the 4 admitted | no | medium to high: 150 to 200 s of the budget, 10 to 15 percent on cameras | medium | low | not known: the cause not found, the seed excluded | M of investigation | the camera speed |
| P7 the curvature law reading a wide fillet as chords | no | low: the lidar above 2.8 m/s only | low: the samples and a lidar series | low | decided, see above | S plus a series | a longer sweep per fillet; the lidar 0.05 to 0.1 m/s |
| P8 the memory forgets nothing, a transient is a wall | not in a static world | high for item 18 and for reality | medium to high: the memory is the planner's input | low in simulation, high in reality | in concept: item 18 stage 0 (transient occupancy, decay) | L | the memory, the planner, both series |
| R1 the 2D memory node still selectable | no | low | low | low | decided, see above | S | the launch files |
| R2 the route-volume heuristic | no | low | low: a script and a test | low | decided, see above | S | the mission check |
| R3 sources near the cap, a flat `src/` | no | medium: maintenance | medium: moves, the gates | low | mechanical | L | the structure |
| N1 multi-vehicle missions on the camera defaults | no | high for item 15 | none | medium: item 15 may uncover defects | none | series | the cooperative missions |
| N2 the cave and the finals locations | no | medium | none | medium: location-independent code unproved | none | series | none |
| N3 one start, one goal | no | medium | none | medium | none | series | none |
| N4 the blinded chain never exercised | medium: item 17 | high: the first requirement | none | high outside the simulator | item 17's injected failure | series | the estimator, the offboard path |

Read together: first C6, then C4 with C3 in one step, all three item 17
stage 0, all three moving the speed baseline and needing both series; then
P5, dear and with a trigger that has not returned since its repair. The
cheap and safe ones, R2, R1, L3, P4 and P7, close in one pass of scripts
and documents plus one lidar series. S1 with P1, P3, C5 and P6 are not to be
touched before a cause is measured: their laws have been flown and reverted
before. P8, C8, L5 and N4 do not hurt in the simulator and are each a hole on
a real vehicle.

### The Options Weighed For The Decided Entries

For each entry the owner decided on, the two ways it could go, as they were
laid out on 2026-09-25, with the effort and the quality each was given.

- **L2.** Minimal: the anchor shift by the autopilot's reported delta (M,
  medium risk, needs an injected reset to prove; removes the fatal latch of
  item 13's rule; a sound solution). Architectural: navigation in the
  estimator's frame, setpoints translated into the autopilot's by a live
  offset, so that resets stop existing for the memory (L to XL, every
  consumer of the pose, both series; clean, but a rework of the whole
  stack's pose). Landing as the policy was set aside: it keeps the first
  requirement and fails the second by construction. Chosen: the minimal one.
- **L3.** Drop the note for the odometry profiles (S, the drift loses its
  watchman), or a reference per profile tied to the capture radius (S).
  Chosen: the reference per profile.
- **L4.** Not now, the drift is measured against truth already; or a tool
  image with Ceres and OpenVINS and a runner on the records r550, r571 to
  r576 (M). Chosen: not now, the tool image when the drift rework starts,
  where a reference says how much of the drift is the algorithm's and how
  much the data's.
- **C1.** A route and horizon shape that climbs and descends at 50 degrees
  or less, inside the pair's field, at 2.45 m/s instead of 1.0 (M, medium
  risk; the vertical stays free by the invariant, what is charged is the
  unobservability of a motion, which the speed law charges already); or a
  sensor, a third time-of-flight unit or a wider cone (L, a decision about
  the real vehicle, and outside the speed goal's rule on sensor models). The
  measurement that came first showed the band lives in shafts, where a
  diagonal has no room, and what the band hid was the walls a climb never
  faced; the survey turn answered that. Chosen: the survey; the sensor
  remains.
- **C7.** The noise now, on a series of its own, with the confident depth
  re-measured and a new baseline (M plus series); or the noise together with
  the lamp in item 17, since the lamp is needed there anyway. Chosen:
  together, one move of the baseline.
- **C8.** The measurement alone, a panel the matcher cannot match on the
  route, one flight, the confident range against it (S to M); or the
  behaviour at once, a matcher confidence by texture and a matched nothing at
  short range read as observed unobservability (L, and without the number
  nobody knows whether it is needed). Chosen: the measurement, inside item
  17.
- **S2.** The wall clock kept, with the quiet-host gate moved into
  `scripts/` and the simulation-time figure printed beside it as a note (S,
  the metric unchanged); or the simulation clock as the requirement's
  measurement, stable and independent of the host, at the price of
  re-expressing the thresholds' base and of no longer measuring whether the
  stack keeps real time. The agent recommended the first; the owner chose
  the second, and kept the rule that no flight under foreign load counts.
- **P7.** `static_route_corner_curve_samples` 4 to 8 with a lidar series (S
  plus five flights, 0.05 to 0.1 m/s on the lidar, a sweep twice as long per
  fillet); or the curvature read from the fillet itself through arc metadata
  in the route samples (M, across modules). Chosen: the first, and not on
  its own, with the next change that needs a lidar series.
- **R1.** Removal after a grep shows no scenario selects the node (S to M),
  or leaving it. Chosen: removal; dead code since the 3D memory of item 10.
- **R2.** One pass counted from the first entry to the last exit, with a unit
  test on the r600 geometry (S); or dropping the check, which was set aside
  because it certifies the physical passage through the observed 3D volume
  of a scenario. Chosen: the first.

## Localization

| Debt | Measured | Class | Found by |
|---|---|---|---|
| The visual-inertial estimate drifts without bound: an odometry with no loop closure. The goal-in-truth check leaves 0.6 m of margin on the worst accepted flight, and a mission several times longer will exceed the 2.0 m capture radius. The remedy is long-lived points in the filter's state or relocalization against a map. | 0.1 to 0.4 percent of the path; the true position 0.47 to 1.42 m from the goal at its acknowledgement (r579 to r583), 1.97 and 2.31 m before the gyroscope noise was corrected (r572, r575) | (b) | item 16 |
| The controller closes the navigation for the rest of the flight when the autopilot resets its position by more than 0.33 m (the rule roadmap item 13 made). With an odometry as the only position such a reset is possible; the vehicle then holds until the flight's window ends, which fails "always reaches its goal". Not seen again since the autopilot fuses the poses with 0.3 m of noise. What the stack should do after such a reset (hold and re-anchor, discard the map near the vehicle, land) is a policy. Decided 2026-09-25: a re-anchor by the autopilot's reported delta, as seen then (item 17, stage 6). | r561: a 0.4 m correction, the odometry rejected, 1.8 s on the IMU alone, a reset of 1.19 m, mission incomplete | (c) | item 16 |
| The autopilot's position estimate follows the odometry, so its error across the track against the true pose is the odometry's drift; the 0.35 m reference figure was measured on GNSS and is printed as a note on every camera flight. Whether the figure is re-derived for the odometry profiles or dropped is a decision. Decided 2026-09-25: re-derived, 1.0 m on the odometry profiles. | 0.67 to 1.19 m at p95 in four of five flights (r579 to r582) | (c) | item 16 |
| No reference visual-inertial system has been run on the recorded flights. OpenVINS without ROS needs Ceres, which builds from source in the container, and Boost.Filesystem, which the container image does not carry, and then a runner for these records. It is a tool and no criterion; adding the package to the image is a decision about the image. Decided 2026-09-25: not now; a tool image when the drift rework starts. | attempted 2026-09-20, stopped at configuration | (c) | item 16 |
| The autopilot's IMU reaches the estimators over a best-effort transport at 83 to 92 Hz and loses bursts when the host stalls. The filter grows its uncertainty over a hole and recovers; a reliable or higher-rate IMU path is a change of the autopilot's transport configuration. | holes of 0.33 to 0.69 s under a stalled host (r547, r549), at most 0.1 s otherwise (r550) | (b) | item 16 |

## Speed

| Debt | Measured | Class | Found by |
|---|---|---|---|
| The lidar profile's mean flight speed has little margin over its requirement; a flight under it fails. Most of what it loses is no-route holds at surfaces. | 2.42 m/s against 2.4 (r587); 2.42 to 2.67 over r584 to r588; 2.49 to 2.72 over r612 to r616, and 2.29 on r605 before the curvature law's inputs were corrected | (a) | items 9 and 16 |
| The mean flight speed is measured on the wall clock, so a simulation slower than real time, from foreign load on the host, lowers it by as much; such flights are excluded by hand (r553, r554, r577, r578, r592, r593, r619). The rule that the clock could not change was the agent's reading of the owner's "thresholds are not tuned", not the owner's word. Decided 2026-09-25: the requirement is measured on the simulation clock; the thresholds stand; flights under foreign load stay excluded by the quiet-host gate. | real-time factor 0.78 under a browser and a GPU job against 0.93: 1.22 m/s on the wall clock for 1.53 m/s of simulation time (r577) | (c) | item 16 |
| The heaviest configuration, the stereo pair rendered beside the lidar, does not hold real time on the reference workstation, so its speeds are not comparable with either sensor set's. | real-time factor 0.83; 1.9 to 2.4 m/s (r566 to r570) | (a) | item 16 |

## Camera Perception

| Debt | Measured | Class | Found by |
|---|---|---|---|
| A faced motion whose elevation lies between the pair's vertical field and a time-of-flight cone is seen by no sensor and is flown at the vertical contract's speed. Measured on the camera series r607 to r611 the band holds 1.7 percent of the moving time and 0.9 percent of the path, 28 s over five flights at 1.0 m/s, every episode 0.2 to 0.8 s long and inside a shaft, so its own price is about 2 percent of the mean speed. What the band hides is larger and sits in the shafts: climbing one, the pair faced one or two of eight heading sectors per half metre, the walls it did not see stayed unknown, and the planner went on hoping for exits in them; the shaft at (53, -7) m, 30 m from the goal, was tried on five of the eight camera flights r599 to r611 and cost 27 to 50 s each (two climbs through 34 to 39 routes, three abandonments with an 80 m detour), about 34 s a flight. 3df703ce turns the gaze at 1.5 rad/s while a climb or a descent has no heading to face (`gaze_survey_yaw_rate_radps`), which gives the pair 99 to 100 percent of a shaft's walls per half metre against 60 to 78 percent before (r617 to r622 against r608 to r611 in the shaft at (54, -25)); none of the six survey flights chose the shaft at (53, -7), so its price under the survey is not measured. Seeing an exit from below the level it opens at remains a sensor on the vehicle or a wider field. Decided 2026-09-25: the survey stays, route shaping is not pursued (the band lives in shafts, where a diagonal has no room). | band 52.4 to 67.5 degrees, 1.74 percent of the moving time at 1.0 m/s; the shaft 27 to 50 s on five of eight flights before the survey | (c) | item 14, C1 follow-up 2026-09-25 |
| Lateral deviations of the horizon from the route enter unknown space at the validators' speed; under the lidar that space had already been seen. | not measured as a separate figure | (b) | item 14 |
| The sensor evidence age exceeds the 600 ms the braking contract charges on a small share of ticks of the camera profile. | up to 628 ms on one flight of five after stage 0 (r541), 624 to 948 ms before it; 648 ms on one of five (r611) | (a) | items 14 and 16 |
| The other half of the entry above: the contract charges that age as a constant and never reads the real one, although the stack computes it. `latestSensorEvidenceFreshness` hands the measured age and a freshness flag to the validator, which drops a stale scan from the points it checks the body against, so "will I hit what I can see" is answered on measurement and "how fast may I fly" on a configured 600 ms. The two consumers of the one pair therefore time out asymmetrically: the estimator goes unhealthy 1.0 s after the last visual update (`maximum_unaided_s`) and stops publishing, which withdraws the autopilot's position, while the speed law goes on admitting what 6.4 m and 600 ms admit. The window between the two, where the charged age is already exceeded and the pose is dead reckoning that is still declared healthy, has never been flown on purpose. | 0.6 s charged against a 1.0 s unaided timeout; 2.4 m of travel at 2.45 m/s against a 2.0 m margin | (b) | item 17 |
| The nearest pass to true occupancy is closer on the vision memory than on the lidar's. | 0.41 to 0.75 m centre to centre of 0.5 m voxels against 0.84 m | (a) | item 14 |
| The braking contract's forward range is a configured constant (6.4 m in `sensor_profile.py`) and nothing lowers it when the pair returns nothing. The one reduction in the code, `min(guaranteed_detection_range_m, observed_range_m)`, applies to a motion the vehicle does not face and reads memory, not the sensor. A blind pair is therefore flown at the speed a healthy one admits, which is a hole in the first requirement at full illumination, not only in the dark. The repair makes the range a measurement of the recent frames, judged for the frame as a whole and not per ray, and moves the contract's central input, so both acceptance series are re-flown and the speed figures of items 14 and 16 re-measured. | the pair returns 14 172 to 74 099 points per frame within one lit flight (r579, r583) and the admitted speed does not move | (b) | item 17 |
| The simulated cameras carry no noise model and the location carries no lamp (the urban world has no `<light>` element; all of its illumination is `<scene><ambient>0.1 0.1 0.1</ambient>`), so every flight so far was lit by a constant that no failure can touch. Adding the noise lowers the measured confident depth and moves the speed baseline with it, which is why it waits for a series of its own. Decided 2026-09-25: with the lamp, in item 17 stage 1. | the confident depth 6.4 m was measured with no noise on the imager (item 14 stage 1) | (c) | item 17 |
| The confident depth of 6.4 m was measured on the photogrammetric surfaces of one location, the only textured world the project renders; the collision-only world carries no textures and a camera sees nothing on it, so the two extremes have been flown and nothing between them. A surface a matcher cannot match — blank paint, poured concrete, a large uniform panel — is fully lit, carries all the signal an imager wants and returns no depth or an interpolated surface that is not there. What the confident range becomes on such a surface has never been measured, and measuring it is likely to lower the range and move the speed baseline with it. Decided 2026-09-25: measure first, in item 17 stage 2. | not measured | (c) | item 17 |

## Planning, Execution And Control

| Debt | Measured | Class | Found by |
|---|---|---|---|
| Route availability and ordinary no-route holds miss their reference figures on the lidar profile; what remains is the recovery after a physical block at a surface, 0.3 to 1.2 s without a route each time. | availability 93.4 to 96.7 percent against 97, holds 3.0 to 6.8 percent against 3 (r584 to r588) | (a) | items 10 and 12 |
| The planning loop runs near 43 Hz with most ticks over the 20 ms deadline and no single bottleneck left; the planner spends its whole 150 ms budget, so its p95 measures the configuration. | tick 22 to 23 ms at p50, 30 to 37 ms at p95 | (b) | item 10 |
| A holding vehicle drifts farther than the clearance its rest pose keeps beyond the body; three laws that released the margin were flown and reverted. | 0.37 m at p95 against 0.27 m | (a) | item 10 |
| A path validation beside a wall the envelope touches still costs several times the median, and the assembly's 12 ms budget is checked between attempts, not inside one. | 15 ms for one path against a thinned scan (487 ms before), the longest tick 226 ms (r548) | (b) | item 16 |
| The persistent planner's update has no bound on its own stages but the deadline the departure and goal probing now respect. On r596 (2026-09-25, camera profile, f4c45394) the vehicle stood 106 s at (5.5, 21, 17) m under the blocked-route law of a stale route: every update spent 4.4 s (search_ms p50 4382, raw sweeps 4276 ms, repair and feasibility 0) in the departure refinement, 512 probes at 8 ms of raw sweep each, and the queued continuation was retired every tick, so the route was never replaced while the raw world advanced a revision on nearly every tick; the flight timed out 45 m from the goal after an 816 m excursion. f0d64d86 stops both probings at the update's deadline (telemetry `departure_deadline_hit`, never hit on the twenty acceptance flights r597 to r616, planner p95 152 to 163 ms). What remains is that the planner's budget between repair, feasibility search and world changes is not scheduled: a stage that finds no stable world can still spend the whole update, and route stall recovery (`route_stall_recovery_enabled`, off) is the existing lever to measure against it. | one flight in eleven before the repair; none in twenty after | (b) | speed goal 2026-09-24 |
| The recovery after a stop reaches about 2 m/s^2 with the heading already aligned, against the 4 m/s^2 the profile admits. It is not the rest-to-rest cubic connector of the route-directed seed (a guide seed for straight non-terminal intervals was flown on r594 and reverted: recovery 1.89 against 2.03 m/s^2, holds and availability worse) and not the arrival limits; the executed horizon's first step itself implies 2.0 to 2.1 m/s^2 on aligned recoveries. The cause has not been found. | 1.9 to 2.1 m/s^2 at p50 on aligned recoveries (r591, r594) | (a) | speed goal 2026-09-24 |
| The curvature law reads a corner over 2 m of route. A fillet in open space is materialized as four curve samples and resampled, so the law reads its chords: a right angle with a 6 m control distance is admitted at 2.8 m/s where the arc supports 3.9. Only the lidar profile cruises above 2.8. Reading the corner over 5 m admitted 2.5 m/s at a right angle in a passage whose fillet supports 1.6, which is what cut the corners on r605; more curve samples would let the law read the arc, at the price of a longer swept validation per fillet. Decided 2026-09-25: 4 to 8 samples with the next lidar series. | not measured as a figure on the lidar series r612 to r616 (2.49 to 2.72 m/s) | (c) | speed goal 2026-09-25 |
| The obstacle memory clears an occupied voxel only when a free ray passes through it again (`miss_weight`, the Schmitt trigger at `free_score`), so a voxel never re-observed stays occupied for the rest of the flight, and nothing distinguishes an occupancy confirmed by a thousand scans from one that collected two hits and vanished: the planner treats both as a wall. A person crossing the frame is a route replacement; a trail left where the vehicle does not return is a wall until landing; smoke is the case rays cannot clear at all while it lasts. Found from two sides by two readers of v0.4.0 (a walking person, smoke). `forgetDynamicVolumes` covers only the cooperative peers whose positions arrive by intent. | not measured as a figure; the stack has never flown a moving body or a plume | (b) | item 18 |

## Repository

| Debt | Measured | Class | Found by |
|---|---|---|---|
| The 2D obstacle memory node is still selectable by the launch files. Decided 2026-09-25: to be removed. | one node and its launch branch | (c) | item 10 |
| The mission check's route-volume crossing (`headless_topology_validation.py`, the box 4..16 x 20..32 x 9..18 m) counts one pass as two when the vehicle grazes the box's side inside the pass, and prints a NOTE for a flight that crossed the volume end to end (r600: y 19.6 to 19.8 m at x 14.7 to 15.4; r609, r610 the same at the far side). Whether the heuristic should tolerate a side excursion is a decision about what it certifies. Decided 2026-09-25: it stays; as seen, one pass from the first entry to the last exit. | three of twenty acceptance flights | (c) | speed goal 2026-09-25 |
| Fourteen sources sit near the 1000-line cap and most of the package lies flat in `src/`. | `swept_footprint.cpp` at 994 non-blank lines | (b) | item 10 |

## Not Flight-Verified

Not debt by the rule above, but unproven and recorded here so that a green
series does not hide it: the multi-vehicle missions have not been flown on the
camera defaults, nor without GNSS, nor since the interception missions were
removed (roadmap item 15 waits for them); the cave and the finals locations
are imported and load but have not been flown; the point-to-point mission has
been flown from one start to one goal.

The chain that a blinded vehicle would take has never been exercised deliberately: the estimator reports itself unhealthy and stops publishing, the autopilot ends its external-vision fusion after 200 ms, its position ages out, the controller revokes the execution authority and the offboard path holds. r561 walked part of it by accident (the autopilot rejected the odometry, dead-reckoned 1.8 s and reset its position by 1.19 m) and the flight did not recover. Until roadmap item 17 injects the failure, no flight has tested degraded or absent illumination at all.
