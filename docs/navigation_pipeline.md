# Navigation Pipeline

The production pipeline converts current vehicle state and obstacle evidence
into a timestamped local trajectory horizon.

## 1. Raw World Snapshot

The obstacle-memory node integrates accepted 3D lidar returns into sparse
observed `Occupancy3D`.
`/drone_city_nav/obstacle_memory_status` carries the producer and sequence
heartbeat without copying the grid. The 3D profile publishes a revisioned
`/drone_city_nav/raw_obstacle_snapshot_3d` base plus
`/drone_city_nav/raw_obstacle_delta_3d` dirty chunks. RViz and provenance
representations are debug-only and are never planner inputs.

In no-static mode the reconstructed raw state is the planning world. The 3D
pipeline publishes the current physical scan first, integrates persistent memory
on a latest-value worker, and transports immutable world revisions at a bounded
rate. Base snapshots are adaptive and intervening updates are cumulative dirty
deltas, so skipped superseded messages do not invalidate the newest state. In
static mode the planner loads canonical Occupancy3D directly and does not merge
the sensor grid into the static 3D map.

Sensor liveness, raw-map content, and local planning state have independent
identities. `LatestObservation` is refreshed by the lightweight heartbeat even
when the scene is unchanged. `RawMapVersion` changes only with reconstructed map
content. A `LocalWorldGeneration` binds one raw version, the pose used to select
the local window, the CPU ESDF, the active GPU ESDF revision, and compatible
topology evidence. A heartbeat never rewrites ESDF content timestamps or map
revisions.

## 2. ESDF Preparation

The production MPPI node prepares a mode-specific occupied-distance field
asynchronously. Static mode extracts a local dense distance window from the
precomputed chunked cache associated with canonical Occupancy3D. Fingerprint or format
mismatch falls back to the exact runtime EDT. No-static mode reconstructs the
revisioned observed occupancy and builds a recentered local
`KnownObstacleDistance3D` window. Distances are defined in both free and unknown
voxels from confirmed occupied sources only; a missing or out-of-window value is
neutral.

The no-static cache stores immutable sparse 8-cubed source and distance chunks.
An exact occupied-source index covers the capped influence halo around the local
output window. A dirty update verifies occupied changes against the dirty-chunk
lineage, structurally shares unaffected chunks, and recomputes only output chunks
within the inserted or removed source's exact distance cap. Broad, reset,
incompatible, over-budget, or incomplete-lineage work falls back to a full sparse
cache rebuild, not the retired dense three-pass observed EDT. Free/unknown
relabeling still refreshes the local observed classification while reusing the
same distance cache and dense GPU buffer. Only the immutable controller upload
boundary materializes a dense float projection; no dense nearest-source array is
retained. MPPI continues
using the last complete immutable field until a newer coherent generation is
ready.

Coherent-world publication and route activation must serialize through explicit
world and execution authorities. A completed asynchronous ESDF build replaces
only immutable world resources and cannot copy, clear, or restore route state.
The resident world, planner transaction, materialized route, compilation
candidate, admission report, and pipeline telemetry are separate immutable or
value-stage artifacts. Their remaining migration into a sealed trajectory and
single execution manager is tracked by
[`navigation_architecture_remediation.md`](navigation_architecture_remediation.md).

Rate-limited work is retained by a latest-wins deferred scheduler. The newest
pending state is processed when the rate deadline arrives even if no later
sensor message appears. A pose-driven recenter request is urgent and may build a
new local generation before that deadline. Planning starts only when the
captured CPU generation names the ESDF revision active in the GPU engine.
Observed generations additionally carry a coverage certificate naming their
exact raw source, the occupied source fingerprint, maximum distance, and whether
the dense distance transform was rebuilt or reused.

Distance classifications are:

- raw collision: hard reject;
- critical band: highest non-collision risk;
- planning band: elevated risk;
- preferred space: normal risk.

The current defaults are defined in `config/urban_mvp.yaml`; documentation must
not duplicate YAML as a second parameter source of truth.

## 3. Persistent Full-3D Route Planning

Production point-to-point navigation has one strategic route producer:
`PersistentDStarLitePlanner3D`. Static and no-static profiles both search a sparse
`(x, y, z)` motion graph. Its level-zero graph is a complete minimum-resolution
26-connected lattice. World-aligned power-of-two overlays add 26-connected long
edges at higher levels; lazy swept-footprint validation accepts them in open
volume and the unchanged level-zero graph supplies local resolution around
obstacles. The authoritative raw occupied set and the physical
swept footprint are the only hard collision constraints; unknown space remains
traversable and has neither a penalty nor an eligibility gate.

Launch support is separate, anchored contact evidence: the pose the vehicle
rested at before takeoff, whose occupied cells its body overlapped there. It is
anchored only while the land detector reports contact and the vehicle is at
rest, so a navigation stack that starts after takeoff anchors none rather than
exempting evidence the body never rested on. It is released once the body is
clear without it and the vehicle has left its envelope in any direction, a
departure sideways included. Its anchor seed is its own: a world derivation
never requires the vehicle's current pose to match it, or an unreleased support
would reject every observed world once the vehicle moved and leave the stack
without evidence at all.

The vehicle's own pose is proprioceptive contact evidence. The footprint has
two radii: the validation envelope (`physical_footprint_radius_m`, the body
plus the clearance every motion keeps) and the physical body itself
(`physical_footprint_body_radius_m`, rotor tips included). The airframe
tilts with the acceleration it commands, so the body every validator answers
to is the configured hull at every tilt the dynamics reach
(`tiltEnvelopedFootprint`, `maximumBodyTiltRad`: the thrust axis at the
horizontal and vertical acceleration limits together), enveloped once at
configuration and stood upright by every validator, planner and execution
alike. With the hull above, the body reaches 0.65 m from the axis, 0.50 m
below and 0.60 m above the centre, inside the 0.82 m envelope. Tilting the
body per command in the execution validators while the planner validated the
upright hull laid routes a quarter of a metre above roofs, where every
accelerating horizon and every stop swept the roof once tilted; one recorded
flight rested beside such a roof for a quarter of its mission with every
route certified from where it stood rejected the same way.

The envelope is the tilt's business, the body is the hull's: the hull's own
extents stay on the body (`body_lower_extent_m`, `body_upper_extent_m`), so
contact evidence is judged against what the vehicle physically occupies. A
vehicle leaves a tight spot the way it entered it, at hover and upright, so a
departure is validated with the hull as configured (`hull_footprint`,
`PersistentPlannerConfig3D::departure_footprint`): the legs leaving the
vehicle, the departure waypoints and every step of the escape fill. The route
records the station its departure ends at
(`CompiledTrajectory3D::departure_end_station_m`), and every validator that
holds the route, the route compiler, the certification, the executed-horizon
validators and the continuation checks, gives its proprioceptive seed that
departure as a chain (`ProprioceptiveFreeSpaceSeed3D::departure_chain`,
`departureChain3D`): the contact volume is swept along it, so evidence the
envelope cannot clear while the vehicle threads its way out is contact for the
whole departure and an obstacle nowhere else, and the physical body stays a
hard rule at every pose of it. A slot 1.25 m tall under an awning, entered
while the awning was unknown, kept one recorded flight resting for two
hundred seconds once the awning was mapped: no lattice node fitted the
enveloped body, the fill found no way out for it either, and the vehicle,
which fitted at hover, was never asked to leave at hover. Occupied evidence
that overlaps the envelope at the proprioceptive seed (the current pose,
widened by half an observed voxel) is contact rather than an obstacle: the
vehicle demonstrably stands there. Every raw validator (planner departure,
route candidates, certification, the finite execution path, hold validation)
exempts exactly that evidence from the envelope, for every candidate pose of
that validation, with no condition whatsoever on where the body then moves. A
rule of the form "no closer than it already is", "only away from the surface",
or "not along the wall" is a prohibition on moving through free space and is
forbidden here: free space stays traversable in every layer. What the
exemption never removes is the body: evidence in the margin between the body
and the envelope at the seed stays binding for the body, so a candidate pose
whose physical body reaches it is a collision like any other, while evidence
the body itself overlaps at the seed is contact through and through, allowed
exactly as far in as the vehicle already is: an occupied voxel says nothing
about where inside it the surface lies, so pressing deeper into it is not
moving through free space, while holding, departing and every free direction
stay open. Evidence
the envelope does not touch at the seed binds exactly as it always does. The
half-voxel tolerance widens what counts as contact and nothing else. The seed is transient evidence
like the newest lidar returns: each validation builds it from the pose its own
execution input reports, never from the pose a resident route was certified
from. A seed frozen into a certified world would stop exempting evidence that
closed in on the vehicle since, and a moving vehicle would lose every validated
motion, including the braking tail that stops it. Departing from a wall that closed
in on the body therefore validates; any further approach is a collision, and
evidence the body does not overlap at the seed (the rest of that wall, any
other obstacle) is never exempt. The resident search graph never sees the
seed, so pose refreshes do not invalidate cached edges or D* labels.

The planner retains its D* Lite state across compatible world revisions. Occupied
voxel deltas update only affected vertices, while unchanged raw occupancy reuses
the existing search state. An occupied delta invalidates only incident cached
edges and already resident D* states inside the maximum-edge sweep reach. Search
that reaches its per-tick budget remains running and resumes through the typed
planner-session queue independently from whether the same update produced a
publishable complete incumbent. An incomplete prefix is not published as a
substitute mission route.

Every edge uses the shared `FlightTimeModel3D`, so horizontal and vertical speed,
acceleration, jerk, and stationary turn limits contribute to one time objective.
The materialized result is then geometry-optimized, assigned a speed-dependent
tracking tube, compiled into immutable execution geometry, and certified against
the exact raw world lineage before activation.

`RouteExecutionManager3D` retains the accepted mission intent and route identity
across ordinary world updates and owns the pending successor and resident plan
under one lock. Successor search starts from a
certified future station and must preserve stopping distance, measured p99
planning latency, and overlap reserve. A newer occupied observation repairs only
the affected suffix or hands the vehicle to a certified stop; it never clears a
still-valid prefix.

Offline `FreeSpaceTopology3D` remains optional static evidence for passage
identities and constrained spans. It is not an online route producer, does not
arbitrate against the persistent planner, and cannot make unknown space hard or
costly. The retired 2D/3D risk lattices, online incremental topology/frontier
planner, strategic lattice adapter, and plan-level route arbitration are not part
of the production graph.

Route activation is then a single optimistic transaction over that immutable
resident world plus the jointly captured pose and
`CommittedExecutionAuthority3D`. The
current-pose connector and remaining suffix are swept against the raw occupancy
from the same producer lineage at or beyond the resident ESDF source revision;
this lets final validation consume newer obstacle evidence without mixing the
resident CPU ESDF, GPU ESDF, and topology generation used for planning.
Collisions in an already passed prefix do not reject the route. The same pose
and applied control drive the dynamic handoff simulation. Publication is
abandoned if the resident world, objective, or captured raw snapshot changes
before the execution manager commits it. Unknown voxels remain traversable
during this raw check.

The acceleration a horizon point carries is the acceleration of the trajectory
the offboard is asked to fly, which it feeds forward to the vehicle: the
command where the integrator let it through, and the command plus the clamp's
share where a speed cap shed the excess the command would have added, so the
vehicle never accelerates past a cap the model believes it is braking under.

The offboard's applied-control feedback names the horizon it executes. The
owner installs it as the applied control when it names the owner's horizon
or, in planned mode, the immediate predecessor: the owner republishes its
horizon every control interval, so the feedback of the previous publication
is the control being applied now. The admission, the authority's
applied-control contract and the freshness rule of the execution input all
apply that one predecessor rule, and a new lease inherits the witnessed
control of its predecessor horizon instead of starting unwitnessed until the
next feedback lands. Feedback of an older or unknown horizon
revokes the latch, and the planner then falls back to the measured
acceleration for control continuity; that fallback anchors the jerk limit on
the vehicle's response rather than on the command, so it is meant for gaps,
not for steady operation.

The production execution boundary has one authority owner. The manager
atomically replaces a captured pending successor and resident authority; a
successor that loses an optimistic race leaves both unchanged and requests a
fresh read. Plan, horizon owner, pointer-identical versioned input, and matching
applied-control evidence are read through one immutable atomic pointer. Every
feedback, lease, revocation, and transition update validates the exact captured
authority before publishing a complete successor revision.

Initial search heading uses a cascade:

1. velocity heading at normal speed;
2. previous accepted-route tangent at low speed;
3. mission direction when no accepted route exists.

## 4. Target And Speed Policy

The planner selects a lookahead point on the active persistent route. World
profile selects distance-evidence preparation and observation limits, while
cruise speed, absolute speed, and acceleration are explicit map-independent
parameters.

Reference speed is bounded by:

- mode cruise and absolute limits;
- curvature preview;
- sensor-observation range and physical stopping capability;
- goal approach;
- the finite certified route endpoint and the reserve needed to stop before it;
- the first route sample the persistent raw world blocks, braked toward as
  toward a route end while the replacement is searched;
- the first route sample the latest lidar scan touches within the distance the
  vehicle needs to react at its absolute speed limit (the sensor braking
  contract's required detection range), braked toward in the same way. The
  persistent memory integrates a hit only after its own confidence stages;
  until then the route is clear to everything but the horizon's own
  validation, which meets the hit at the end of the horizon and stops the
  vehicle hard. The scan bounds the route ahead as it bounds the horizon; the
  persistent memory stays the authority on the route itself, so a single scan
  slows the vehicle and never releases the route;
- constrained-route span limits.

When no executable route exists in either mode, direct flight to the distant
mission goal is forbidden. The planner publishes a typed stationary hold while
route search continues. Direct interception remains valid without a persistent route
only when the current target is visible and the direct swept path is physically
validated.

Route availability and local-horizon executability are separate contracts. If
the active route or validated direct interception exists but MPPI produces no
physically executable next horizon, the planner first validates the unchanged
remaining trajectory of the previously published finite path against the
current raw world. If that trajectory became invalid, the planner may rebuild
the remaining control intent from the measured state, embed endpoint
deceleration inside the remaining control slots, and validate the complete
rebuilt path. Either continuation retains the previous validity deadline and
ends at rest. Only when neither path is executable does the planner publish a
`no_executable_horizon` position hold. A newly validated finite path supersedes
that hold immediately;
recovery does not wait for the vehicle to become stationary. Loss of the
persistent route first preserves any still-executable finite path, and clearance tiers do
not trigger this hold.

## 5. Constrained Route Spans

Static air passages are ordinary physical free space. The separately loaded
`FreeSpaceTopology3D` index provides optional passage identities, volumes, and
cooperative conflict evidence after the persistent planner produces raw-safe
route geometry. It does not add strategic successors or compete with D* Lite.
Intersection of the certified route with derived passage volume creates a typed
constrained station interval; local clearance analysis remains validation rather
than a nearest-opening selector or separate route lifecycle.

## 6. GPU MPPI

Each planning tick:

1. shifts the previous nominal controls by elapsed time;
2. generates CUDA control perturbations;
3. simulates thousands of dynamic-state rollouts;
4. queries the 3D ESDF with the swept oriented physical footprint;
5. rejects physically infeasible rollouts and applies strong soft clearance
   exposure costs to the remaining rollouts;
6. computes the weighted control update;
7. limits the first control relative to applied-control feedback;
8. reconstructs the selected nominal horizon on the host;
9. uses the final control slots of that same path for dynamically bounded
   deceleration, so the endpoint has zero translational and yaw velocity without
   adding samples beyond the configured duration;
10. validates the complete finite path before publication.

MPPI optimizes a short receding horizon. It does not publish a long mission
path for open-loop execution.

## 7. Post-Update And Route-Availability Checks

The reconstructed horizon is validated against the physical occupancy and the
configured flight envelope. Every state must retain enough vertical stopping
room under the configured acceleration and jerk limits, not merely lie inside
the numeric altitude interval. In no-static mode, the newest timestamp-aligned
raw lidar returns additionally validate the complete finite path, independently
of persistent-memory integration latency. A path that intersects raw occupancy
or cannot remain inside the flight envelope is not published. The planner moves
the start of the in-path deceleration earlier until the path validates; it never
appends motion after the finite endpoint. Critical or planning clearance exposure
affects cost and diagnostics, but cannot independently reject motion or latch a
hold.

A failed replacement update does not invalidate the previous path by itself.
The planner first checks its remaining timed trajectory using raw occupancy, the
full swept footprint, and fresh direct lidar returns. The already published path
continues without republishing when that trajectory remains valid. If it does
not, the planner may re-simulate the unexecuted controls from the current state,
reshape the arrival profile in the remaining duration, and validate the
complete rebuilt path. The arrival profile starts from the control being
applied now: a jerk-limited ramp from that control to the deceleration, a hold,
and a ramp back to zero, so a profile rebuilt every tick carries the
deceleration already reached forward instead of releasing it and ramping in
again from zero. The rebuilt command starts at the current timestamp, but
its deadline never exceeds the previous `valid_until`.

Route validity is checked separately. Every non-terminal execution plan includes
a certified braking tail and admits motion only while its remaining route
reserve covers stopping, measured planning latency, and required overlap. The
braking tail is the earliest stop along the command horizon the world admits:
the stop that begins at once where it clears, and otherwise the first stop
that follows the horizon's own turn away from the evidence a straight stop
would run into — the last candidate is the horizon itself, which ends at
rest. A plan whose horizon was certified is therefore never refused for the
straight stop it would not fly; that refusal used to turn a valid turn beside
a wall into a certified stop and a hold. If no
physically executable route remains, the planner latches the current admissible
position and publishes `no_executable_route` hold horizons until a replacement
route is atomically accepted.

### Stopping A Moving Vehicle

A stop is the vehicle's own physics rather than a route action: a finite braking
trajectory that starts at the exact current state, ends at rest, and is
validated against the flight envelope, the dynamics, and the swept body versus
raw occupancy with the live contact exemption. It carries no route, no adherence
corridor, and no route certificate, which is exactly why it stays available when
the route machinery can produce nothing. `ExecutionSupervisor3D::prepareStop` is
its single entry point; the plan state it installs is `kStopping`.

Every situation that would otherwise leave a moving vehicle without a plan asks
for a stop: physical evidence against the resident path, a lifecycle event that
ends that path's claim on the vehicle, and the loss of a lease that can no
longer be continued. Physical evidence that forces a stop also requests the
resident route's successor at that moment, latched once per route generation:
the search anchors at the vehicle while it is still braking, and the successor
is ready when the stop is finished instead of being requested only then. One
recorded run spent a full second holding after every such stop for exactly
that reason. Retention keeps its one job, continuing a path that is
still executable, and delegates braking to the stop
(`braking_delegated_to_stop`) instead of rebuilding a braking tail out of the
path that was just invalidated. Only the offboard's local position latch remains
behind the stop, and only for a genuine loss of the planner.

A stop never survives its own completion. It is finite by construction: once
the vehicle rests, the execution is revoked exactly as it is at a captured
goal, the offboard holds the position the vehicle is at, and the next certified
route activates from the revoked plan. A vehicle that is already at rest when
physical evidence ends its path's claim has no stop to fly, and a revocation
cannot be committed while the route stays resident — the plan a suspension
leaves is not publishable. Nothing then reached the offboard, which kept
flying the last horizon it had accepted, certified before the evidence that
had just invalidated it, to that horizon's rest point; one recorded flight
ended against a wall that way. The rest position is held explicitly instead
(`EXECUTION_HOLD rest=true`): a stationary hold supersedes the stale horizon
on the wire, and the next certified route takes the vehicle back from it. That
hold carries the trajectory revision of the execution it took over, and the
route's trajectory is numbered after whatever owns the vehicle
(`ExecutionPlan3D::ownerTrajectoryRevision`): the execution it replaces, the
stop it takes the vehicle back from, or the hold's source. Numbering it as if
nothing had owned the vehicle kept one recorded flight resting for half a
minute with a certified route pending (`finite_execution_conflict`). A route
may also take the vehicle back while the stop is still braking, so no approach
to any obstacle is ever withheld. Stopping is not a route state, so the stop's length follows the
vehicle's state and its guaranteed deceleration rather than the controller's
control count, and its profile is shaped against the same integrator that
validates it, including the shedding of an inherited excess above the speed
cap.

The swept body a stop is certified against is the policy's clearance envelope
first. That envelope keeps clearance the vehicle may already have lost: the
evidence that ended its path's claim was confirmed beside that path, or ahead
of it, and a braking sweep that starts there cannot always keep the margin. A
stop is the last motion the vehicle can be given, so when the envelope's sweep
meets occupied evidence (`raw_collision`, `latest_lidar_raw_collision`) the
same trajectory is certified again against the physical body alone
(`physicalBodyFootprint`, the body radius with the axial extents kept). Braking
with the body clear is strictly safer than the horizon it replaces; a stop the
envelope refused and nothing replaced is how one recorded flight, still moving,
flew its stale horizon into a wall. A body-certified stop records the body it
answers to (`StopExecution3D::validation_footprint`, `physical_body_only`) and
stays resident, and is revalidated on the wire, against that same body rather
than the envelope, so the envelope's failure does not derive a new stop every
tick. The stop log carries `body_only=true|false`. Other verdicts, a broken
envelope or dynamics law, are not retried: no smaller body satisfies them.

The liveness monitor compares predicted and actual full-3D route progress.
Persistent prediction without real movement can reseed the MPPI nominal controls
and, when explicitly enabled, request release and repair of a stalled route.

## 8. Horizon Publication And Execution

The production node publishes the execution horizon immediately after planning
and safety evaluation. RViz, INFO summaries, and JSONL diagnostics run outside
the control-critical publication path. JSONL writes have a configurable rate and
batched flush period; a bounded recent-record ring is dumped when a collision
episode begins.

Offboard consumes the fresh horizon, interpolates by timestamp, and emits PX4
trajectory setpoints. The finite path reaches zero velocity by its deadline;
offboard then holds its terminal point instead of extrapolating motion.

## Removed Legacy Stages

The production runtime no longer contains:

- grid A* path publication;
- corridor construction;
- post-corridor trajectory optimization;
- separate turn smoothing;
- partial-replan races;
- legacy partial-path prefix/suffix races;
- uncertified safe truncation;
- planner/prohibited inflated grids;
- inflation relaxation or escape tunnels;
- the legacy speed planner and terminal-capture path lifecycle;
- 2D/3D risk-lattice, online-topology, semantic-frontier, and strategic
  arbitration route producers.
