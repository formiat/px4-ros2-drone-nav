# GPU MPPI Trajectory Optimization

The production local planner is Model Predictive Path Integral control running
on CUDA. There is no corridor-constrained post-processing optimizer.

## Persistent Engine

`MppiController3D` is the sole production owner of `MppiCudaEngine`, the
nominal-reseed lifecycle, and the controller-reference cache. The engine owns
persistent CUDA buffers, the resident ESDF texture, and the nominal control
sequence. Per-tick planning does not recreate CUDA allocations. A generic
known-solid API remains in the engine, but the production 3D-world path does not
populate it; canonical physical solids are encoded in Occupancy3D.

## State And Control

The benchmark and production engine use a 3D translational state with yaw:

```text
x, y, z, vx, vy, vz, yaw, yaw_rate
```

Controls are:

```text
ax, ay, az, yaw_accel
```

Dynamics apply acceleration, velocity, vertical-speed, yaw-rate, drag, and jerk
limits from the active static or no-static profile.

## Tick Sequence

```text
shift nominal controls by elapsed time
-> generate counter-based control noise
-> simulate rollouts on CUDA
-> query local ESDF3D against physical Occupancy3D
-> reject only physical collisions and flight-envelope violations
-> compute MPPI weights
-> update controls
-> constrain first control against applied feedback
-> reconstruct nominal horizon
-> classify and validate
-> shape an arrival-to-rest profile within the existing finite horizon
-> validate the complete finite path against raw physical obstacles
```

The arrival-to-rest profile occupies existing path samples and is part of the
published path contract. It is not a second execution phase. If a
later optimization tick fails, the unchanged remaining trajectory continues
only when both its geometry and its remaining controls from the measured state
are physically valid. Otherwise
the remaining path may be rebuilt from the measured state, with its arrival
profile shaped again and the complete rebuilt path physically validated.
The previous validity deadline is never extended.

The warm start is shifted by real elapsed time, including fractional
interpolation. This keeps the nominal sequence aligned with commands already
executed by PX4.

## Feasibility And Soft Clearance Cost

Rollout feasibility has a narrow physical contract:

1. every state remains inside the flight envelope and is dynamically recoverable
   before either altitude boundary;
2. the swept physical footprint does not intersect raw occupancy;
3. the generic known-solid collision contract is not violated.

Clearance is priced by two laws, each answering a different physical
question, and by one mild preference:

- Beside the motion, the **tracking-error tube law**: the error the controller
  can accumulate within its response time must fit inside the body clearance.
  A wall the vehicle flies along never gets nearer, so no braking distance is
  owed to it. This is the law that certifies a route's speed profile and that
  the speed policy's `clearance` limiter applies. A sealed profile stays
  certified on a world it is admissible on: nowhere may a sealed ceiling
  exceed the ceiling the world now permits. A world that only loosened around
  the route still matches — flying under a lower ceiling than the world allows
  is safe — while one that tightened anywhere is refused and the route is
  re-sealed by a fresh search. Read as bit-for-bit equality instead, the rule
  made every certified route perishable on every scan: a voxel that opened a
  little clearance beside the route changed one ceiling by a hundredth of a
  metre a second, and the successor sealed a tick earlier was refused for it
  while the vehicle held with nothing else to fly.
- Ahead on the motion, the **stopping law**: the path the body covers while it
  reacts and brakes must fit inside the free path to the point where its
  envelope enters occupied evidence. It is charged along the rollout's own
  simulated future, on every state before the contact, so a control that will
  turn the vehicle into a wall is priced before the wall is beside it.
- The **clearance preference**: squared normalised depth into the preferred
  band, integrated over time. It prices position, never motion, so it nudges
  the horizon toward the middle of a passage without making standing still
  cheaper than progress.

These terms rank safe rollouts together with route deviation, mission
progress, early/head progress, altitude error, speed tracking, overspeed
(speed above the dynamics caps or above the reference speed the speed policy
derives from its stopping laws, priced so that shedding it outweighs the
progress it buys), acceleration, jerk, yaw motion, terminal error, and control
effort. Low clearance alone cannot make a rollout unreachable or force a
position hold.

Conservative ESDF distance classifies the critical and planning bands for the
risk tier and diagnostics and feeds the soft terms. Hard raw collision is
reported only when the swept oriented physical footprint intersects a raw
occupied cell; there is no additional prohibited inflation layer.

## Static And No-Static Geometry Profiles

World profile may select different horizon, target-lookahead, and
distance-evidence-window geometry. Cruise speed, absolute speed, and dynamics
limits are map-independent explicit parameters.

In no-static 3D mode, unknown space remains distinct from raw occupied space but
has the same strategic traversability and base cost as confirmed free space.
Sensor range, physical stopping capability, route reserve, and the finite
zero-speed braking fallback bound motion before unobserved obstacles can become
unavoidable. The production speed cap is the largest speed satisfying

```text
speed * (maximum_sensor_evidence_age + reaction_latency)
  + jerk_limited_stopping_distance
  + physical_margin
  <= guaranteed_lidar_detection_range
```

The stopping calculation starts with the worst permitted forward 3D
acceleration, reverses it under the configured jerk limit, and uses the weaker
guaranteed horizontal or vertical deceleration. Stale direct evidence fails
closed; a fresh `Unknown` voxel has the same strategic cost and admission rule
as a fresh `Free` voxel. The solved limit bounds the full translational velocity
norm in both host and CUDA integration, not independent XY and Z components.
Strategic ETA and compiled route speed profiles use that same limit. If measured
motion is already above it, the reference speed becomes zero while the dynamics
preserve the inherited velocity for physically continuous braking instead of
teleporting it down to the cap.

Exact defaults live in `config/urban_mvp.yaml`.

## Post-Update Validation

The weighted control update can produce a nominal horizon different from every
individual sampled rollout. The reconstructed horizon is therefore classified
again for physical occupied-cell collision and risk exposure.

Collision results reject the horizon. Route unavailability is handled by a
separate typed position hold. An expired finite path already ends at rest, and
offboard holds that path's terminal point. Post-update classification remains
available in diagnostics without hiding physical invalidity inside aggregate
cost.

The published planned path always ends at a terminal rest state. Its speed
profile reaches zero inside the configured path duration. If the resulting
physical path is invalid, arrival shaping is retried from an earlier path sample;
an old or colliding path is never
executed as a fallback.

The deterministic route-directed candidate connects a straight route interval
with a cubic rest-to-rest maneuver solved in the shortest duration whose peak
acceleration, peak speed, and deceleration of the current speed fit the
dynamics; it then holds the terminal rest for the remainder of the horizon.
A connector stretched over the whole horizon would be re-solved every tick
for "rest at the horizon end" and creep toward a near route end at ever
smaller accelerations without reaching it.

Arrival shaping uses
`finite_path_arrival_maximum_horizontal_deceleration_mps2`, a conservative contract
separate from the higher acceleration available to ordinary manoeuvres. This
prevents a finite path from claiming stopping performance that PX4 cannot track.
That contract bounds the arrival profile's *amplitude* only. The profile is
integrated, jerk-limited and validated under the one canonical dynamics
configuration, so the states the builder records are the states every later
stage reconstructs from the same controls. Integrating the arrival under
reduced limits while the certificate re-integrated it under the full ones made
the builder's own output dynamically inconsistent whenever the control already
applied exceeded the guaranteed deceleration.

The terminal state is the one the integrator produced, never a velocity forced
to zero afterwards. The publisher derives each point's acceleration from the
velocity step between neighbouring states, so a fabricated jump reads as a
terminal acceleration the wire contract rejects. `kTerminalRestVelocityToleranceMps`
and `kTerminalRestControlToleranceMps2` in `control_contracts_3d.hpp` are the
single definition of rest, shared by the builder, the physical path validator,
the execution certificate and the ROS contract.

`motionStepDynamicallyConsistent3D` and `finiteMotionHorizonDynamicsConsistency3D`
in `motion_dynamics_3d.hpp` are the single dynamics law: acceleration envelope,
jerk relative to the previously applied control, and agreement between the
recorded state and the integrated one. The finite-path builder checks what it
emits with it and the execution certificate admits a horizon by it, so neither
can refuse what the other accepted. A rejection carries the reason it broke
(`acceleration_limit`, `jerk_limit`, `state_mismatch`), reported as
`validation=dynamics_inconsistent dynamics=<reason>` in
`EXECUTION_HORIZON_ASSEMBLY`.

The host sampler and the CUDA rollouts share one admissible-control law,
`limitMotionControlStep3D` in `mppi/mppi_control_limits.hpp`. Clamping the
horizontal axes independently, as the device kernel once did, can leave the
acceleration disk during a direction change and produce candidates the host
validator then refuses.

## Reference Speed

The reference speed the controller aims for is capped by several limiters, and
the tightest one wins. Two of them are shaped by what the vehicle is actually
doing:

- The `clearance` limiter reads the horizon **execution currently owns**,
  measured against the world as it stands now (`executed_horizon_clearance_3d`).
  A controller candidate that was never published moved nothing, so sizing the
  reference from its clearance makes the reference chase trajectories the
  vehicle never flew.
- It applies the tube law at **every** sample of that horizon whose body
  clearance falls below cruise times the tube response time — the clearance
  at which the tube admits exactly cruise, so nothing else defines "close" —
  and lets the stopping law decide what the vehicle may carry on the way to
  each; the tightest answer wins, so a mild constraint nearby cannot hide a
  tight one behind it. Treating the minimum clearance over the whole horizon
  as an immediate cap is what made the reference oscillate: a grazing sample
  far ahead dropped the reference, the horizon shortened out of reach of the
  obstacle, the reference jumped back, and the longer horizon found the
  obstacle again.
- The stopping law answers for the vehicle as it flies. It is the exact
  jerk-limited slowdown to the speed admitted at the point: the reaction
  latency, the ramp from the **measured** forward acceleration to the full
  deceleration under `maximum_control_jerk_mps3`, then constant deceleration.
  A law of instantaneous deceleration falls along its own curve at 4 m/s^2
  once the vehicle rides it, while the vehicle beneath it may still be
  accelerating: in the flight r440 the clearance limit fell from 7.9 to
  3.8 m/s in 1.3 s, the vehicle climbed from 3.8 to 5.6 m/s under it, crossed
  it a second before a wall and needed 0.64 s to turn its acceleration round.
  The flights before the change carry 7 to 15 such crossings each; after it 0
  to 4. The laws that stop the vehicle short of evidence or of the end of what
  was certified (clearance, route clearance, blocked route, route endpoint)
  also charge `reference_tracking_lag_s`, the measured delay with which the
  loop follows a falling reference (0.30 s at the median, 0.41 s at p75 over
  r457 to r459, of which 0.10 s is the reaction latency already charged); the
  curvature and goal laws do not, since arriving late at their speed meets
  nothing. The sensor-braking contract keeps its own worst-case ramp.
- Evidence beside the motion owes no braking distance. The horizon is
  validated by the body against the raw world and ends at rest, so "stop
  within the lateral clearance" is not a physical requirement; asking for it
  pinned every corridor to the limiter's progress floor.
- The `unobserved_frontier` limiter reads the same executed horizon for where
  the evidence ends. The sensor-braking contract bounds the speed by the range
  the lidar is guaranteed to have seen, which holds in the open; the inside of
  an opening the vehicle is about to turn into has not been seen at all until
  the lidar looks in. Along the executed horizon the evidence reaches only as
  far as the first sample whose body envelope enters unobserved space, and the
  contract is read with that range in place of the guaranteed one — the same
  latency, stopping distance and physical margin. Unobserved space stays
  traversable and carries no penalty: inside the margin the limiter is the
  tube limiter's progress floor, the speed the raw validators certify every
  horizon for, and the range grows back as the view opens. Without it a jamb
  the lidar revealed late was met at four to five metres per second, the
  finite path was rejected too late for anything but a physical stop, and the
  stop left the body beside the evidence, where every entry into the next route
  collided.
- The executed horizon alone is not enough to find that frontier. The horizon
  ends where the vehicle can come to rest, so the frontier it reaches is never
  beyond the stopping path: it reports one only once the vehicle can no longer
  stop before it, and over a whole urban flight it reported none at all while
  walls the lidar had not looked at were met one to three metres ahead. The
  followed route is where the vehicle goes next, so the same measurement is
  taken along the route from the vehicle's projection, as far as the latest
  scan is checked along it — the distance the vehicle needs to react at its
  absolute speed limit — and the limiter reads the contract with the shorter
  of the two ranges. Only the speed changes: the route through unobserved
  space is flown as planned, at the speed the vehicle can stop from before
  the space it has not seen.

On the default stereo sensor set the contract is directional. Its range and
margin are those of the sensor whose field holds the motion: the pair's 6.4 m
and 2.0 m inside 60 degrees of the heading and 52.4 degrees of the horizon
(2.452 m/s), a time-of-flight sensor's 2.8 m and 1.0 m inside 22.5 degrees of
the vertical (1.29 m/s). Two cases have no sensor behind them:

- a motion the vehicle does not face, at any elevation outside the vertical
  cones. The direction is the route's tangent where there is a route: the
  velocity of a vehicle correcting its track swings through tens of degrees.
  Memory answers for it. `measureObservedRangeAlong3D` reads how far along
  the direction the raw world has observed every voxel the body sweeps
  (occupied counts as observed; the clearance laws own it), the contract is
  read with that range, capped by the sensor's, and where memory has observed
  no more than the physical margin nothing is admitted. The distance field
  cannot supply the range: free and unknown are one input to it, and along a
  route that entered unseen space 1 m ahead the frontier limiter read 20 m,
  the edge of the field's grid (r500: a wall met at 1.5 m/s under a fixed
  1 m/s rule). Space the vehicle has flown through or seen before stays
  flyable without facing it: 40 percent of the planned ticks of r505 were
  unfaced, at a median admitted speed of 2.36 m/s, and 4.7 percent of all
  ticks waited;
- a faced motion between the pair's field and a cone (52.4 to 67.5 degrees of
  elevation), which no heading brings into view: `unobserved_motion_speed_mps`
  (1 m/s). A wall across such a motion is seen by the pair, which is why it
  must be faced; an edge the size of the gap is not, and this remains a known
  limit.

Waiting for the gaze is not a latch because the gaze turns a vehicle at rest:
`applyGazeYawControls` turns the heading to where the horizon moves over 1.5 s
at a rate proportional to the remaining angle (gain 1.5 1/s, inside the yaw
limits), and where the horizon goes nowhere to the route's tangent at the
vehicle's station; a position hold other than a goal capture carries the same
heading (`gazeRestHeading`), so a held vehicle looks where its route leaves
and the space ahead becomes observed. A lidar that sees all around enters none
of this (`forward_detection_horizontal_half_angle_deg` 180).

The same rule binds every published horizon, not only the reference of the
tick. A horizon owns the vehicle for its whole lease, and the lease outlives
the tick: when nothing can be committed (r518: the autopilot's position was
rejected for 1.7 s while its timestamps were reacquired) the resident horizon
flies on, through whatever turn it holds. The horizon assembler's candidate
validator therefore refuses a candidate whose states carry speed along a
motion no sensor sees at that state's own planned heading, faster than memory
admits along it (`firstUnseenMotionState3D`; states no faster than the
stationary hold tolerance carry nothing), and the arrival search shortens the
refused candidate as it does for any other refusal, so the horizon ends at
rest before the unseen motion. Measured cost: the mean speed of the stereo
profile fell from 1.48 to 1.67 m/s (r506 to r510) to 1.17 to 1.57 m/s (r523
to r527), the tick from 21 to 23 ms at p50 and from 31 to 37 ms at p95.

The reference may fall as fast as any limiter asks — a cap is always allowed to
bite at once — but it may only climb at `reference_speed_rise_mps2`, the
vehicle's own horizontal acceleration. A limit that lifts as the horizon shifts
therefore cannot snap the reference back up, because the controller answers
each step with a fresh burst of acceleration.

## Clearance Costs And Control Selection

A rollout carries one obstacle approach charge, `obstacle_approach_weight`,
made of two laws that share the weight and no parameters of their own:

- the tracking-error tube law beside the motion, read from
  `tracking_error_tube_response_time_s` — the same configuration the route
  certification uses and the speed policy's `clearance` limiter reads as a
  speed cap;
- the stopping law ahead of the motion, read from the speed policy's stopping
  capability (`speed_reaction_latency_s`, the horizontal acceleration), which
  the same policy applies to a blocked route, a route endpoint and the goal.

Each law is one expression read in two directions: the rollout cost reads it
as a shortfall, the reference speed as a cap. Two copies of one law drift
apart, and the reference speed then admits what the optimiser prices as too
fast; that is why neither law is tunable here separately.

Charges that used to sit here, and why they are gone:

- A flat price per metre *travelled* inside the critical band priced motion
  itself. In a corridor narrower than the band the whole section is inside it,
  and a metre of flight cost two orders of magnitude more than the progress it
  earned; the weighted update converged on standing still.
- An isotropic stopping law priced the nearest surface, whichever way it lay:
  a wall beside the vehicle demanded the braking distance a wall ahead would
  need. In a 4.4 m corridor the law admitted 1 m/s, the reference sat on the
  limiter's progress floor for more than half of every flight, and the same
  law in the optimiser made every faster rollout more expensive than the
  crawl. The tube law is what the passage was certified for.

Position is still priced, mildly, by `clearance_preference_weight`: squared
normalised depth into the preferred band per second, the same for a rollout
that hovers beside a wall and one that flies past it. Distance inside the
bands is still measured for the risk tier and diagnostics.

The softmax temperature is regulated, not configured. A fixed temperature
against a population spread over thousands of cost units collapses the weights
onto the single best sample, and the weighted update degenerates into "best of
N random"; scaling the temperature by a share of the mean cost excess tracked
the spread's scale but not how many samples actually carried weight, which is
the quantity that matters. The regulator measures the effective sample size,
`(sum w)^2 / (N sum w^2)`, from the weights each tick produced and nudges the
temperature toward `mppi_target_effective_sample_fraction` for the next one,
bounded below by `mppi_temperature`, above by `mppi_maximum_temperature_growth`
and per tick by a factor of two, so the loop settles within a few ticks and one
odd tick cannot swing it. `effective_sample_fraction` reports what it achieved.

Two sources can own the update: the weighted update and the deterministic
route-directed candidate. They produce visibly different first controls, so a
preference that flips tick to tick is felt as a jerk. The candidate takes the
update over only after staying preferable for
`route_directed_candidate_switch_ticks` consecutive ticks; handing it back is
immediate, because the weighted update is the default owner. A candidate that
is itself the best feasible rollout, and a liveness-forced candidate, still win
at once.

## Reference Speed And The Route Profile

The controller does not track a bare scalar reference. Every route sample
carries its own `reference_speed_mps`, and the rollout cost tracks
`min(scalar reference, route profile at the rollout's own station)` — for the
speed-tracking term and for the overspeed term alike.

The scalar remains, and deliberately. It carries the limits that depend on
evidence the compiled route profile cannot see: the clearance of the horizon
the vehicle is executing right now, the sensor-braking range, a route blocked
since activation, and the goal. A profile compiled when the route was activated
answers for the route's geometry; it cannot answer for what the lidar saw two
seconds ago.

The route profile itself is a continuous acceleration profile over the dense
route samples: each sample's speed is bounded by what the previous sample can
accelerate to and what the next can be braked to over the whole path between
them, and the jerk limit is charged once per acceleration phase, at the point
where the phase ends. The same profile prices routes in the planner's ranking.
It used to be built from one jerk-limited transition per sample, each starting
and ending at zero acceleration, so a straight metre sampled every half metre
cost about a quarter more than the same metre sampled once, and the reference
the optimiser was offered on a dense route sat well below what the geometry
allowed.

## Continuity And Liveness

- The first command is bounded relative to applied-control feedback.
- Early predicted progress is measured separately from terminal progress.
- Neighboring horizons are compared through warm-start continuity metrics.
- Persistent prediction without actual displacement triggers reseeding and can,
  when recovery is enabled, release and repair a stalled persistent route.

## Performance

The supported benchmark is:

```bash
make mppi-benchmark \
  MPPI_BENCHMARK_ARGS="--scenario urban_blocks --rollouts 8192 --steps 80"
```

Production diagnostics separate noise generation, rollout simulation, risk
reduction, control update, horizon reconstruction, and total host/GPU time.
ESDF build latency is measured separately from the control tick.
