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
