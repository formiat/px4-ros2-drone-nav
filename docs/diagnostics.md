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
`route_constraint`, `blocked_route`, `clearance`), and the
`*_speed_limit_mps` fields carry each limit; `clearance_speed_limit_mps` is
the stopping-limited speed within the executed horizon's body clearance.

The JSONL record carries the same data in machine-readable form.

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
  route-planning latency;
- route fingerprint, generation, mission-target identity, and release reason;
- current route station and remaining distance;
- certified-reserve, compilation, validation, publication, and activation
  status.

Planner evidence describes the search that produced the resident route.
Candidate validation and activation fields describe later contracts and must
not be inferred from `planner_executable` alone.

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
successor whose activation the reducer rejected on the candidate's own contract
(`invalid_candidate`). Such a proposal cannot activate by being offered again,
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
