# Logging And Diagnostics

Production diagnostics are designed to observe control without delaying
execution-horizon publication.

## Main Artifacts

- `log/mppi/`: MPPI JSONL summaries.
- `log/lidar_debug/`: synchronized lidar/grid/path snapshots.
- `log/lidar_memory_hits/`: accepted and classified memory-hit records.
- `log/gz_drone_nav.log`: Gazebo server and orchestration log.
- `log/gz_gui_drone_nav.log`: Gazebo GUI log.
- `log/gazebo_scene_debug/`: bounded GUI scene and camera diagnostics.
- ROS logs under `log/latest/` and timestamped run directories.

The runner may place artifacts in a per-run directory. Use the path printed by
the script rather than assuming that `log/latest` belongs to the intended run.

## MPPI Tick Diagnostics

`PRODUCTION_MPPI_TICK` reports:

- pose, obstacle, memory, and ESDF revisions and ages;
- target source and target position;
- active route generation, planner status, and remaining length;
- static/no-static speed-policy limits;
- constrained-route phase, route generation, and span index;
- GPU and host stage timings;
- selected tier and risk exposure;
- raw collision and compatibility known-solid flags; the latter remains false
  in the canonical Occupancy3D production path;
- head and terminal progress;
- horizon stability and first-control delta;
- liveness action;
- post-update classification;
- typed execution mode and reason, including `no_executable_route` hold and
  `no_executable_horizon` hold;
- finite-path nominal and arrival-profile control counts, terminal-rest
  confirmation, arrival-shaping attempts, and whether the current tick retained
  a revalidated previous finite path;
- latest raw lidar sequence, age, hit count, freshness, and whether it forced an
  earlier in-path deceleration start;
- dropped diagnostic snapshots.

`active_speed_limiter` names the speed limit that set the reference speed
(`cruise`, `curvature`, `sensor_braking`, `goal`, `route_endpoint`,
`route_constraint`, `blocked_route`, `clearance`, `unobserved_frontier`), and
the `*_speed_limit_mps` fields carry each limit; `clearance_speed_limit_mps` is
the tracking-tube speed the executed horizon's body clearance admits, folded
with the stopping law on the way to each constrained sample, and
`unobserved_frontier_speed_limit_mps` is the sensor-braking contract read with
the range the evidence covers ahead of the vehicle — along the executed horizon
and along the followed route, whichever ends first in space the lidar has not
observed — and `unobserved_frontier_range_m` is that range (`-1` when both stay
observed).

The JSONL record carries the same data in machine-readable form. It is written
at `diagnostics_file_rate_hz`, well below the tick rate, because the record is
large. It additionally itemises the decision:

- `executed_horizon_*`: the clearance the speed policy answered to — the
  minimum over the motion under execution, the first constrained sample's
  clearance and distance, `executed_horizon_distance_to_unobserved_m`, the
  path length to the first sample whose body envelope reaches unobserved
  space (`inf` when the whole motion is observed), and
  `executed_horizon_constrained_samples`, every constrained sample as
  `[distance_m, clearance_m]` in path order;
- `selected_costs` and `route_directed_candidate_costs`: the weighted cost
  terms of the sequence the tick selected and of the route-directed candidate
  (`null` when none was injected), reported by the same kernel that ranked the
  population — `head_progress`, `progress`, `route_progress_integral`,
  `speed_tracking`, `overspeed`, `guide_deviation`, `altitude_tracking`,
  `acceleration`, `jerk`, `yaw_change`, `dynamic_aircraft`,
  `maneuver_preference`, `clearance_preference`, `obstacle_approach`,
  `stopping_deficit`, `terminal`, their sum `soft_cost`, plus the rollout's
  `minimum_clearance_m`, `contact_distance_m` (path length to the first
  envelope contact, `null` when clear), `head_speed_mps` and its collision and
  altitude-envelope flags. A tick that preferred the slower sequence names the
  term that made the faster one dear.

`mppi_track.jsonl` carries one compact line per tick regardless of that rate:
position, velocity, the first control, the control-selection source, the
reference speed before and after its rise limit, the active limiter, the
minimum ESDF distance, the selected sequence's head speed and contact distance,
the executed horizon's first constrained clearance and its distance (`-1` when
unconstrained), the distance to its first unobserved sample (`-1` when the
motion is observed throughout), the risk tier, the route generation, the planning state and
the execution reason. A throttled record cannot answer how often the reference
speed flips, how often the first control opposes the velocity, or how long a
stall lasted — those are properties of the ticks it skips.

## Timing

Relevant stages include:

- input snapshot;
- ESDF build and upload latency;
- noise generation;
- rollout simulation;
- risk reduction;
- weight and control update;
- horizon reconstruction;
- safety evaluation;
- total GPU and host tick time;
- asynchronous RViz/JSONL work.

Refusals name the rule they broke rather than the stage they happened in:

- `EXECUTION_HORIZON_COMMIT committed=false stage=<payload status>` reports
  which payload rule the horizon failed, not a bare `invalid_payload`. These
  are the most frequent commit refusals a run records.
- `STOP_EXECUTION published=false` carries `certification=`,
  `certification_dynamics=` and `certification_path=` beside the transition
  status. A stop is the last thing a vehicle with no executable route can do,
  so `next_plan_invalid` on its own left nothing to act on.
- `EXECUTION_HORIZON_ASSEMBLY` carries `dynamics=` beside `validation=`.

Every log of a run lands in that run's own directory (`log/runs/<id>/`) beside
its manifest and raw snapshots. Writing them to a fixed path meant the next run
overwrote the previous one's evidence, so two runs could never be compared
after the fact. The manifest records the launch overrides that were actually in
force (`effective_overrides`): the configuration file's hash alone does not
identify a run, because the launch takes overrides that change speed limits,
accelerations, the map mode and the duration without touching the file.

`RAW_SNAPSHOT_BOUNDS_M` places the raw Occupancy3D capture volume
independently of `OBSERVED_3D_ROUTE_VOLUME_BOUNDS_M`. The latter is the
acceptance volume and sits near the start, because that is the geometry the
crossing check is about; a pocket the vehicle gets stuck in is somewhere else,
and snapshots taken at the start cannot reproduce it offline.

`ROUTE_GEOMETRY` records, for every activated route, its generation, sample
count and a strided list of its points. Without it a log shows that a route
changed and how long it is, but not where it goes, and a reversal between two
passages cannot be told from a local adjustment after the fact.

`PRODUCTION_MPPI_SUMMARY` reports two route-availability figures.
`post_bootstrap_route_availability_ratio` counts ticks where a route was
resident and *something* owned execution — a stationary hold counts, which is
how a vehicle stuck in a pocket for eleven minutes reported 99.8% availability.
`post_bootstrap_route_executable_ratio` counts only ticks where an owner was
actually flying the route, and is the figure that describes progress.

`PRODUCTION_MPPI_SUMMARY` reports two deadline counters. `deadline_misses`
counts ticks whose whole cycle — snapshot, cycle preparation, controller,
publication and commit — overran the planning period: this is the rate at which
the vehicle actually receives a fresh executable horizon.
`controller_deadline_misses` counts the optimiser's own share of that overrun.
A controller comfortably inside its budget with a cycle several times the period
shows up as zero controller misses and a high full-cycle count.

ESDF build latency is asynchronous and must not be interpreted as part of
`gpu_total_ms`.

For no-static observed worlds, `PRODUCTION_MPPI_ESDF3D_ONLINE` reports the
effective occupied-source count, sparse source and distance chunks, finite
distance voxels, inserted and removed sources, recomputed/reused/changed chunks,
queried voxels, source-index time, and distance-query time. These are sparse
`KnownObstacleDistance3D` measurements. Legacy dense x/y/z pass,
nearest-source, dependency-invalidation, and lowering counters are not valid for
this path and must not be inferred from them. `mode=reused` can still include a
new raw revision and refreshed free/unknown classification without a GPU upload.

For static worlds, `PRODUCTION_MPPI_SUMMARY` reports service-owned
`static_esdf_builds`, `static_esdf_cpu_reuses`, `static_esdf_gpu_reuses`, and
`static_esdf_refreshes`. A proactive refresh may increment both reuse counters
while still publishing a fresh local-world generation; it does not imply a
redundant controller upload.

## Persistent Planner And Route Diagnostics

Inspect:

- independent `planner_input` and `planner_progress` values, plus whether this
  update published an improved candidate and whether any incumbent is resident;
- planner mission/world provenance, search and repair generations;
- changed occupied voxels, affected lattice states, records, open entries, and
  expansions;
- total lattice-edge queries, raw edge validations, adaptive-edge queries,
  adaptive edges retained in the extracted lattice path, and maximum queried
  resolution level;
- whether search state was reused, the occupied world was unchanged, or an
  incumbent was retained;
- path length, remaining goal distance, execution-time estimate, and split
  translation/stationary-turn estimates;
- planner world-update, planner-search, route-search, and end-to-end
  route-planning latency. Two different questions are measured separately.
  `planner_p95_ms`/`planner_p99_ms` describe one planner update's search step:
  whether the planner returns within its compute budget.
  `route_lead_time_p95_ms`/`route_lead_time_p99_ms` describe the lead time of a
  route *request* — queue, every search continuation and the activation attempt
  — measured from the request stamp the transaction carries, and longer by
  orders of magnitude. The lookahead extension policy is sized from the lead
  time; it used to be sized from `route_planning_ms`, the slice one update
  spent applying a result that had already come back, which is why it
  underestimated the margin it needed;
- route fingerprint, generation, mission-target identity, and release reason;
- current route station and remaining distance;
- certified-reserve, compilation, validation, publication, and activation
  status.

Planner evidence describes the search that produced the resident route.
Candidate validation and activation fields describe later contracts and must
not be inferred from `planner_executable` alone.

`OBSERVED_FOOTPRINT_READINESS` reports the vehicle's own pose against observed
occupancy once per second. `status` is what every raw validator sees, including
proprioceptive contact suppression; `strict_status` ignores it. A
`strict_status=raw_collision` with `status=clear` means observed evidence has
closed in on the body (`failure_point` names the voxel) and the validators are
letting the vehicle depart without approaching it; `contact_tolerance_m` is the
half-voxel tolerance in force. A persistent `planner_input=start_unavailable`
together with `status=raw_collision` means no departure from the pose validates
even with contact suppression. `PERSISTENT_PLANNER3D` says why on every update:
`departure_nodes=valid/candidates` counts the lattice nodes in the connector
radius and those whose own body clears the raw world,
`departure_rejected_legs` the legs from the vehicle to a valid node that the
swept body could not fly, `departure_failure` the first point such a leg met
evidence at, `departure_probes=reachable/tried` the finer departure refinement
probes and how many the body reached, and `seed_distance_m` /
`seed_tolerance_m` where the proprioceptive seed the departure exemption holds
lies relative to the start (negative without one) and its contact tolerance.

`PRODUCTION_MPPI_SUMMARY` also reports the planning-service lifecycle counters:
`route_planning_queued`, `route_planning_processed`,
`route_planning_displaced`, `route_planning_busy_rejections`,
`route_planning_invalid_rejections`, `route_planning_stopped_rejections`,
`route_planning_processing_failures`, and
`route_planning_handler_failures`. A displaced request means a newer immutable
world request replaced an older pending request; it does not mean that the
currently executing planner call was interrupted.

For no-static 3D runs, `PRODUCTION_MPPI_ROUTE3D` also reports
the same persistent-planner result together with certified-route reserve,
publication, activation, raw-connector, and raw-suffix evidence. Acceptance
uses these fields together with measured `state_position` samples; it does not
depend on a planner-defined passage event.

`COMPILED_TRAJECTORY valid=false reason=invalid_tracking_error_tube_*` names the
rule the tracking-error tube refused a route on: `_route` for the route
geometry, `_world` for the evidence fingerprint the profile is bound to,
`_contract` for the footprint, tube configuration or speed ceiling, `_segment`
for a segment whose limit came out non-finite, `_profile` for the assembled
profile's own validity. `missing_tracking_world` means the compiler had no
evidence to bind the profile to at all. One recorded flight held for three
seconds with candidate after candidate refused as a bare invalid tube.

`PENDING_ROUTE3D adopted=false eligibility=<verdict>` means a successor is
sealed and the plan cannot take it: `owner_epoch_mismatch` and
`base_generation_mismatch` say the plan has moved past the base the successor
was sealed against, `base_owner_mismatch` that the base is still there but not
in the shape the successor's kind requires. One recorded flight held for one
and three quarter seconds with a certified successor published and nothing in
the log to act on.

`activation_status=route_certification_rejected` means the candidate passed the
activation assessment and the dynamic handoff but the route certification
refused it; `certification=<verdict>` names the rule
(`RouteCertificationStatus3D`: `raw_evidence_not_current`,
`tracking_tube_world_mismatch`, `passage_geometry_mismatch`,
`assessment_*`, `raw_connector_not_validated`, ...). The assessment verdicts
name the rule the activation assessment refused on inside the certification —
publication, objective, projection, cross-track, raw-world compatibility or
raw validation — because a route the admission's own assessment accepted can
still be refused there, the certification measuring it against the evidence
and the body it will be executed with. A handoff the
validator itself refused stays `dynamic_handoff_rejected`. Before the verdict
was reported, one recorded flight rested for three seconds beside a wall with
candidate after candidate logged as a handoff rejection and nothing to act on.

## Liveness Diagnostics

Compare:

- actual displacement;
- predicted head progress;
- predicted terminal progress;
- observation age;
- reseed generation;
- route stall generation and action.

High terminal progress with no actual displacement indicates an ineffective
horizon, not successful navigation.

### Stop Execution

`STOP_EXECUTION published=true trajectory_revision=<n> snapshot_version=<v>
speed_mps=<s> stop_distance_m=<d> controls=<n> rest=(x,y,z)
replacement_failure=<reason>` reports that no route-directed horizon could be
published while the vehicle was moving, so a certified braking trajectory from
the exact current state took the vehicle to rest. It is normal after physical
evidence against the resident path; a run with none of these lines never lost
its plan while moving.

`STOP_EXECUTION published=false status=<status>` names why a stop could not be
derived: `validation_world_unavailable` (no usable raw/lidar evidence),
`horizon_unavailable` (the dynamics admit no braking horizon of that length),
`transition_rejected` or `publication_commit_rejected` (the plan or lease moved
under the preparation). `at_rest` and `resident_stop_current` are not failures
and are not logged: the vehicle is already standing, or the stop that owns it is
still executable.

`STOP_EXECUTION completed=true speed_mps=<s> position=(x,y,z)
action=revoke_and_hold_locally` reports that the stop has brought the vehicle to
rest: the execution is revoked, the offboard holds the position, and the next
certified route activates from the revoked plan, exactly as after a captured
goal.

`EXECUTION_RETENTION ... stage=braking_delegated_to_stop` reports that a
lifecycle event ended the resident path's claim on the vehicle. Retention only
continues a path that is still executable, so it hands braking to the stop
instead of rebuilding a braking tail out of the invalidated path.

### Execution Plan Transition Rejections

`EXECUTION_HORIZON_ASSEMBLY ... transition=<status> detail=<detail>`,
`EXECUTION_HOLD ... transition=<status> detail=<detail>` and
`ROUTE_CERTIFICATION_RELEASE ... transition=<status> detail=<detail>` name the
plan transition that rejected a certified candidate. The status is the reducer
outcome (`invalid_candidate`, `finite_execution_conflict`, ...); the detail
names the contract check behind it (`activation_binding_invalid`,
`next_plan_invalid`, `progress_composition_mismatch`, ...), so a repeated
rejection can be traced to one predicate without re-running the reducer.

`PENDING_ROUTE_RETIRED route_generation=<n> base_route_generation=<m>
transition=<status> detail=<detail> acknowledged=<bool>` reports a pending
successor whose activation the reducer rejected on the candidate's own contract:
every `invalid_candidate`, and a `certificate_regression` whose detail names
the successor's own certificate (`successor_certificate_older`,
`successor_world_content_mismatch` — two certificates at one raw revision
whose persistent occupancy differs; the transient free-space seed and launch
support are not part of that comparison —, `successor_producer_mismatch`,
`successor_certificate_kind_mismatch`, `successor_validation_policy_mismatch`)
rather than the finite execution it was offered with
(`successor_execution_evidence_older`, `successor_execution_input_older`).
Such a proposal cannot activate by being offered again,
and while it stays pending the lifecycle measures every newer successor against
it and the selection offers it instead of the resident route's own candidates.
The node acknowledges it so the lifecycle plans a fresh successor from the
vehicle. Transient outcomes (a moved snapshot version, a finite-execution
ordering conflict) keep the pending route and are retried.

## Static Constrained Route Diagnostics

`ROUTE_CONSTRAINT_EVENT` is emitted on observable lifecycle transitions:

```text
approach -> traversal -> departure -> unconstrained
```

The event and `mppi_ticks.jsonl` expose:

- route generation plus constrained-span index and count;
- current, entry, and exit route stations;
- signed distance to entry and exit;
- entry and exit positions;
- actual, reference, minimum, and maximum Z;
- free distance left/right plus lateral width and vertical height;
- whether lateral clearance, vertical clearance, or both caused classification;
- vertical-window validity, vertical error, and horizontal cross-track error;
- actual horizontal/vertical speed and constrained reference speed;
- execution mode and reason at every lifecycle transition.

Selected traversals retain a geometry-stable `PassageTraversalId` plus the
ordered `PassageSegmentId` resources used by that route. Constrained spans are
correlated by `route_generation + passage_id + span_index`; the serialized ROS
field named `passage_id` carries the traversal identity. Entry and exit
coordinates use traversable anchors on the automatically extracted arbitrary
portal voxel patches. `approach` is not proof of entry; only `traversal` means
the measured 3D route station crossed the span boundary.

`PASSAGE_TRAVERSAL_EVENT` is evaluated on every planning tick, independently of
the throttled `mppi_ticks.jsonl` sample rate. It emits `entered`, `completed`, or
`aborted` with the stable traversal ID, route generation, span index, measured
position and station, traversal duration, observation count, maximum cross-track
and vertical errors, and whether every observed traversal sample remained inside
the vertical envelope. A completed event for the same identity as its entered
event is the durable evidence that the vehicle crossed the full passage span.

`PASSAGE_GEOMETRY_EVENT` provides independent physical-path evidence when a
static route passes through geometry represented by the optional static
topology index. It projects the
measured vehicle position onto every runtime static traversal candidate on each
planning tick and requires continuous entry-to-exit progression inside the
extracted clearance tube.
`PASSAGE_GEOMETRY_PROXIMITY` reports the nearest entry, projection station,
cross-track error, and extracted clearance while the vehicle is within 8 m of
an entry. A `completed` geometry event together with raw-safe horizon validation
proves physical passage traversal regardless of which search backend produced
the active route.

## Offboard Diagnostics

Important events:

- horizon rejection reason;
- `MPPI_HORIZON_DEADLINE_MISSED`;
- completed-path or unavailable-path hold activation;
- route-unavailable, cooperative, or mission hold activation;
- applied-control feedback age;
- PX4 mode/arming state.

An expired planned path must already be at its terminal rest state. Offboard
then holds that terminal point and never extrapolates the path beyond its
deadline.

## Lidar Diagnostics

For the 3D profile, use `LIDAR3D_SCAN`, `ONLINE_OCCUPANCY3D_UPDATE`, and
`PRODUCTION_MPPI_ESDF3D_ONLINE` to verify nonzero hit and miss beams, monotonic
occupancy revisions, base/delta publication, known/free/occupied/unknown counts,
local ROI dimensions, and ESDF build/upload cadence. Use lidar snapshots to
verify:

- heading was accepted before projection;
- scan and pose timestamps align;
- raw returns and map-frame points agree;
- memory contains plausible retained evidence;
- static/no-static source selection is correct;
- no-static 3D route activation is independent of static topology metadata.

## Run Analysis Order

1. Confirm mission outcome, typed destruction cause, disarm/hold settlement, and
   final pose.
2. Confirm pose, heading, raw snapshot, and ESDF freshness.
3. Inspect the active route, persistent-planner provenance, and target source.
4. Inspect selected MPPI tier and collision flags.
5. Inspect head progress, actual motion, and liveness.
6. Inspect constrained-span evidence only when the certified route intersects a
   derived static passage volume.
7. Inspect the finite-path deadline, in-path arrival profile, and final hold.
8. Only then tune costs or dynamics.

For constrained no-static 3D-lidar acceptance, begin with the per-run
`manifest.json`. Its hashes identify the source, parameters, generated world,
and retained raw volume before log interpretation. The final
`PRODUCTION_MPPI_SUMMARY` reports `post_bootstrap_route_availability_ratio`,
post-bootstrap no-route holds, ownership gaps, and persistent-planner p95/p99.
An admitted `PRODUCTION_MPPI_ROUTE3D` entry must report either `sufficient` or
`terminal_exempt` reserve; a sufficient continuation must have
`reserve_available_m >= reserve_required_m`.
