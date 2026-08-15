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

Every published planned horizon is a complete finite execution path. Its final
samples contain the deceleration required to reach zero translational velocity
and zero yaw rate before the path deadline; no motion is appended after that
deadline. A newer validated path atomically replaces it. If no replacement is
available, offboard may finish the still-valid path and then holds its terminal
position. If no executable path has ever been accepted, offboard holds the
captured current position instead of extrapolating motion.

Mission completion is determined by `mission_monitor_node`, using:

- goal radius;
- low-speed threshold;
- stable hold duration;
- absence of a latched `VehicleDestroyed` event.

Mission monitoring does not send control commands.

## Distinguishing Holds

- **Goal hold**: normal low-speed MPPI behavior near the mission goal.
- **No-executable-route hold**: explicit position hold because no physically
  valid route is available; low clearance alone cannot activate it.
- **Completed-path hold**: stationary hold at the terminal point already
  contained in the last accepted finite path.
- **Mission-command hold**: explicit lifecycle command after a terminal mission
  event for a surviving vehicle.
- **Unavailable-path hold**: stationary hold at the current admissible position
  when no finite path has supplied a terminal point.

These states should be distinguished in logs before changing goal parameters.
