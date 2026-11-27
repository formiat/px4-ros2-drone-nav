# Step 5. Testing A Change And Reading A Flight

Two things a contributor does every day: prove a change did not break the
build, and find out what a flight did. The references are
[testing.md](../testing.md), [diagnostics.md](../diagnostics.md),
[troubleshooting.md](../troubleshooting.md) and
[tools/README.md](../../tools/README.md).

## The Gate Before A Commit

Everything a commit must pass is one script: the format, the build, the
unit tests, the script tests and the static analysis, in the container.

```bash
./tools/gates.sh
cat log/tools/gate_flag.txt          # X=0 when all of it passed
grep -c 'warning:' log/tools/gate_all.log   # must print 0
```

The script returns at once and works in the background; wait for the flag.
A commit that changes Markdown files alone needs only the script tests,
which read the documents:

```bash
./tools/gates.sh --docs
```

One script test can be run by itself on the host, for instance the one that
holds this guide's links to files that exist:

```bash
python3 -m pytest -q scripts/tests/test_guide_links.py
```

The same checks run as continuous integration
([ci.yml](../../.github/workflows/ci.yml)).

## A Flight That Counts

A flight proves something only when the simulator ran in real time and
nothing else loaded the host (A4, A7). The tool that flies one such flight,
and flies it again when the host spoiled it, is

```bash
tools/fly_until_valid.sh --lidar 4283 1
```

(the sensor set, the run's number, the attempts allowed; without `--lidar`
the stereo set). A series of the acceptance is
`tools/fix_series.sh FIRST_RUN`, five flights on each set
([autonomous_development.md](../autonomous_development.md), section 5).
Flights go strictly one at a time (A3), and nothing heavy runs beside one:
on the day this page was written an analysis started beside a flight cost
that flight.

A run leaves its records in `log/runs/RUN/` and the mission check's verdict
in `log/tools/run_RUN.log`.

## Reading One Flight

The flight below is r4283, the second lidar flight of an acceptance series
on Urban.

**1. Did it count, and did it pass?**

```bash
python3 tools/host_verdict.py r4283
grep -E '^(OK|FAIL|NOTE)' log/tools/run_r4283.log
```

```text
HOST valid r4283
OK: no vehicle collided with a static world obstacle
OK: PX4 instances report Gazebo ready (1)
OK: 3D obstacle memory receives timestamped hit/miss scans
...
```

Sixty-two lines of `OK` and none of `FAIL`. A `FAIL` on contact or on the
goal is a failed requirement (R1, R2); a `NOTE` is a measurement outside its
reference, never a failure (R4). A flight the host spoiled prints `VOID`
and is flown again (A7, A15).

**2. The whole flight in one row.**

```bash
python3 tools/flight_sweep.py r4283
```

```text
| run | mean m/s | goal by truth m | stops published / refused / in a row / commits refused | revocations moving / fastest m/s | scans with a body / two on one place / two apart | margin m / stale cells | problems |
| r4283 | 2.263 | 0.06 | 12 / 7 / 2 / 10 | 8 / 3.32 | 0 / 0 / 0 | - / - | none |
```

The mean speed, how far from the goal the vehicle truly was when it
acknowledged it, how often the planner asked for a stop and how often the
stop was refused at its commit, and a last column that names a problem when
the sweep sees one. Give it many runs at once (`--jobs N`,
`--problems-only`) after a series.

**3. How good was the estimate of the pose?**

```bash
python3 tools/estimate_error.py r4283
```

```text
r4283: n=7836 path 534 m | horizontal p50 0.12 p95 0.29 max 0.48 end 0.07 m | vertical |p50| 0.10 |p95| 0.17 min -0.25 max +0.05 m | frame turn -0.03 deg scale 0.9999 remainder rms 0.08 m
```

The pose the planner flew by against the simulator's truth: here within
0.29 m for 95 percent of the flight and 0.07 m at its end. A goal missed
"in truth" (R2) with a clean flight otherwise is almost always this figure.

**4. The stack's own words.** Every stage logs one line per event
([step 3](03_mission_and_pipeline.md)); `grep` is the debugger:

```bash
grep -c 'EXECUTION_HORIZON accepted=true' log/runs/r4283/ros_drone_nav.log
```

printed 7401: the horizons the offboard node took in that flight.

## When A Flight Goes Wrong

- **A contact** is a crash and is traced to its cause, never flown again to
  make it pass. Start from the line `VEHICLE_DESTROYED` in the stack's log
  (which parts touched, where, how fast) and walk back through the
  planner's track, `log/runs/RUN/mppi/mppi_track.jsonl`: one line per tick
  with the position, the speed, the limiter that set it and the planning
  state.
- **The vehicle stands still**: the track's `planning_state` and `limiter`
  say why, and the lines `EXECUTION_HORIZON_ASSEMBLY` and
  `EXECUTION_HORIZON_COMMIT` say what was refused.
- **The autopilot**: its log is `log/runs/RUN/px4_drone_nav.log`.
- **Slow computer or slow host**: `python3 tools/node_budget.py RUN` gives
  each node's time against the budget of record (K53).
- **The same flight again without the simulator**: a flight recorded with
  `RECORD_STACK_BAG=true` is replayed by `tools/replay_stack.sh RUN` (K62).

A picture of what the vehicle saw is in [diagnostics.md](../diagnostics.md)
(RViz shows only what the vehicle's sensors gave it, never the world's
truth); windows are opened on a virtual display,
[virtual_display.sh](../../tools/virtual_display.sh).

Next: [extending the stack](06_extending.md).
