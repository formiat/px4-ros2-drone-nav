# Performance

Planner performance is measured through wall-time logs and trajectory
diagnostics. Focus on wall-time for user-visible latency and aggregate timings
for CPU cost inside parallel candidate evaluation.

## Main Build Stages

Typical stages:

- grid / inflation;
- A* rough route;
- clearance field;
- corridor construction;
- trajectory optimizer;
- turn smoothing;
- speed profile construction;
- diagnostics and dumps.

The current dominant stages are usually trajectory optimizer and turn
smoothing, but the exact bottleneck depends on map, obstacle state, and active
windows.

## Reading Wall-Time

Planner summary logs report total and stage timings. Use wall-time values when
asking whether the drone had to wait for a new trajectory.

Aggregate candidate timings can exceed wall-time because candidate evaluation
is parallel or repeated over many candidates. Use aggregate timings to find CPU
hotspots, not to estimate publication latency directly.

## Grid / Inflation

Grid cost depends on:

- map size;
- resolution;
- static map caching;
- dynamic lidar/memory overlay size;
- inflation radius;
- planning clearance.

The grid builder uses distance fields and static/dynamic reuse where possible.

## A*

A* is usually not the main bottleneck after heuristic tuning. It can become
expensive when the map is large, blocked, or overly constrained.

Important parameters:

- `astar_heuristic_weight`
- turn cost;
- initial heading bias;
- evasive mode.

Changing A* search behavior can change routes, so optimize it carefully.

## Corridor

Corridor cost depends on sample count, raycasts, clearance-field reuse, and
lateral limit calculations. Diagnostics include sample build, raycast,
lateral-limit, and clearance-field timings.

## Trajectory Optimizer

Optimizer cost depends on:

- active samples;
- candidate evaluations;
- DP states and transitions;
- collision checks;
- scoring cost;
- worker configuration.

Useful diagnostics:

- `trajectory_optimizer_candidate_evaluations`
- `trajectory_optimizer_active_window_count`
- `trajectory_optimizer_window_eval_duration_ms`
- `trajectory_optimizer_dp_duration_ms`
- `trajectory_optimizer_candidate_batch_wall_duration_ms`

Reducing false active windows is often safer than cutting candidate quality.

## Turn Smoothing

Turn smoothing cost depends on:

- detected corners;
- attempted corners;
- candidate attempts;
- relaxed candidate attempts;
- collision checks;
- shape diagnostics.

If it becomes too expensive, prefer better acceptance/scoring and bounded
attempts over broad candidate explosion.

## What Can Be Optimized Without Changing Behavior

Usually safer:

- buffer reuse;
- exact cache hits;
- avoiding duplicate diagnostics;
- thread-pool overhead removal;
- static cache reuse;
- top-level logging cleanup.

Potentially behavior-changing:

- reducing candidate space;
- changing active-window rules;
- changing A* heuristic/search strategy;
- changing speed policy;
- changing trajectory smoothing acceptance;
- local repair instead of full replan.

## Profiling Checklist

1. Extract total trajectory build wall-time.
2. Rank stages by wall-time.
3. Check whether candidate aggregate cost explains optimizer wall-time.
4. Check active sample count and candidate count.
5. Check whether turn smoothing attempted too many candidates.
6. Confirm whether any expensive work is diagnostics-only.
7. Compare previous and current runs with the same map and config.
8. Avoid optimizing a non-dominant stage first.

## Optimization Principles

Performance work in this project is constrained by trajectory quality and
safety. A faster planner that produces sharper turns, worse curvature jumps, or
more controller oscillation is not a successful optimization.

Classify every optimization before implementing it:

- exact optimization, expected to preserve the chosen trajectory;
- diagnostic optimization, expected to reduce logging cost only;
- shadow-mode optimization, not yet allowed to affect behavior;
- behavior-changing optimization, allowed only when quality tradeoffs are
  understood.

Examples of exact optimizations include buffer reuse, exact cache hits, worker
pool reuse, and avoiding repeated computation of diagnostics that are no longer
used. Examples of behavior-changing optimizations include adaptive candidate
space, top-N candidate scoring, pruning by approximate lower bounds, and
coarse-to-fine candidate search.

## Wall-Time Versus Aggregate Time

Parallel candidate evaluation can make aggregate time look alarming. If 16
workers each spend 100 ms, aggregate time can report 1600 ms while wall time is
near 100 ms. Both numbers matter, but they answer different questions.

Use wall time for mission responsiveness:

- how long before a trajectory is published;
- how long a replan blocks improvement;
- whether a local repair target is realistic.

Use aggregate time for CPU efficiency:

- whether candidate scoring is doing too much repeated work;
- whether a diagnostic is expensive across many candidates;
- whether a cache would reduce total CPU load.

## Current High-Value Targets

The usual high-value planning targets are:

- trajectory optimizer candidate evaluation;
- turn smoothing candidate generation and validation;
- grid and clearance-field construction when obstacle sources are large;
- diagnostics that compute expensive metrics in hot loops.

Before optimizing, confirm from the latest run that the target is still a
bottleneck. This project has changed substantially: grid and speed-profile
candidate cost were once larger issues than they are after later refactors.

## Trajectory Optimizer Performance

The optimizer cost is driven mainly by:

- number of active samples;
- number of offset candidates;
- full scoring frequency;
- collision and traversability checks;
- curvature and radius metric evaluation;
- worker scheduling overhead.

The safest improvements are exact:

- reuse worker buffers;
- reserve candidate vectors;
- cache exact collision checks;
- cache exact segment geometry;
- avoid unused speed-profile scoring for candidates;
- keep deterministic result order in parallel evaluation.

Riskier improvements should stay in shadow mode first:

- adaptive candidate space;
- lower-bound pruning;
- local speed-profile approximation;
- top-N full scoring.

Top-N full scoring is a known cautionary example. It can reduce work but can
also select a worse-looking trajectory if the cheap score misses the true best
candidate.

## Turn Smoothing Performance

Turn smoothing becomes expensive when many candidates are generated for each
corner or strict acceptance rules force long searches. The right goal is not to
evaluate every possible local curve. The goal is to find a good enough local
repair without harming global shape.

Useful controls:

- number of corners attempted per pass;
- maximum candidates per corner;
- early acceptance when a candidate clearly improves radius and curvature jump;
- scoring valid candidates instead of hard-rejecting too many;
- skipping speed/time computations in the hot loop when they are diagnostic
  only.

If turn smoothing takes more time but does not visibly improve trajectory
quality, check whether it is searching for a perfect candidate after already
finding a good one.

## Grid And Clearance Performance

Grid build is safety-critical and should be optimized carefully. Static map
data is a natural cache candidate because buildings do not change during a run.
Dynamic lidar and memory overlays change more often.

A future optimized model can split:

- static raw grid;
- static inflated grid;
- dynamic raw grid;
- dynamic inflated grid;
- final hard prohibited grid;
- planning-clearance grid.

The merge must be exact and invalidation must be conservative. A grid-cache bug
can become a safety bug, so this is a medium-risk optimization even if the idea
is straightforward.

An EDT-based distance field can improve inflation and clearance reuse, but it
can change boundary cells. It should be introduced with tests that compare
expected prohibited cells near the radius threshold.

## Local Repair Performance Target

A future local repair should aim to beat full replan wall time when only a
small future span is blocked. The realistic target is not "always under
500 ms"; it depends on window length, grid size, and whether the repair needs
full validation. The useful target is:

- faster than full replan for small blocked spans;
- no worse than full replan for quality;
- never accepted without whole-trajectory validation;
- full replan still available as fallback.

Performance logs for local repair should report both local-repair time and
full-replan time for the same trigger when they run in parallel.
