# Goal Approach And Final Hold

The production MPPI stack does not use the legacy terminal-capture state
machine.

## Goal Approach

The speed policy computes a goal-limited reference speed from remaining
distance, braking acceleration, reaction latency, and goal margin. MPPI
therefore reduces speed before the mission goal instead of flying at cruise
speed until the last horizon.

The global guide also has a mission-goal hold distance. Near the goal, the
target is the mission goal rather than another distant lattice frontier.

## Final Hold

When MPPI produces negligible motion at the goal, offboard follows the horizon.
If horizons stop arriving or expire, offboard's generic fallback decelerates
and then holds the current position.

Mission completion is determined by `mission_monitor_node`, using:

- goal radius;
- low-speed threshold;
- stable hold duration;
- absence of a latched crash.

Mission monitoring does not send control commands.

## Distinguishing Holds

- **Goal hold**: normal low-speed MPPI behavior near the mission goal.
- **No-guide hold**: braking/hold because no safe no-static guide exists.
- **Passage hold**: explicit XY position hold while capturing passage altitude.
- **Deadline hold**: offboard fallback after the execution horizon expires.

These states should be distinguished in logs before changing goal parameters.
