# Terminal Capture

Terminal capture handles the final meters of the mission. It exists because
pure trajectory-following velocity control is not the best tool for precise
final positioning and hover.

## State Machine

The terminal state machine uses these states:

```text
cruise
  -> velocity_terminal_capture
  -> position_capture
  -> final_hold
```

`position_capture` is intentionally preserved. It is the mode that commands PX4
to go to the final point and hold there, and it is important for stable mission
completion.

## Temporary Replan Hold

Lidar-only runs can enable `safe_trajectory_truncation_enabled`. When the
planner finds a prohibited cell on the currently accepted path, it immediately
publishes `/drone_city_nav/replan_blocker`. Offboard retains only the old
executable prefix ending `safe_trajectory_truncation_margin_m` before the
blocker, then uses the same velocity terminal capture and position capture
states as it does at the mission goal.

The margin is fixed along the executable trajectory; it is deliberately not a
speed-dependent stopping-distance calculation. If the requested terminal
station is no longer ahead of the drone, offboard enters
`temporary_replan_hold` at its current position.

`temporary_replan_hold` differs from `final_hold` only in mission semantics:

- it never marks the mission complete;
- an empty path after a failed replan preserves the already-safe prefix;
- the first accepted replacement path clears the temporary state and resumes
  normal flight.

The launch file enables this feature only with the lidar-only no-static policy.
Static-map runs retain their existing trajectory behavior.

## Velocity Terminal Capture

Velocity terminal capture starts before the final point. It computes a
terminal speed limit using braking distance and remaining distance.

Important parameters:

- `terminal_capture_radius_m`
- `terminal_capture_gain_1ps`
- `terminal_capture_max_speed_mps`
- `terminal_capture_decel_mps2`
- `terminal_capture_braking_margin_m`

The terminal speed law prevents the controller from relying on a sudden zero
speed exactly at the goal.

## Position Capture

Position capture switches PX4 offboard setpoint mode to a position target near
the final goal. It is entered only when terminal conditions are safe enough,
including speed and distance thresholds.

Important parameters:

- `terminal_position_capture_max_entry_speed_mps`
- `terminal_stuck_speed_mps`
- `acceptance_radius_m`
- `final_hold_max_speed_mps`

## Final Hold

Final hold is latched when the drone is within the final acceptance condition
and slow enough. It prevents the controller from repeatedly leaving and
re-entering terminal behavior.

## Diagnostics

Offboard telemetry logs include:

- terminal state;
- velocity terminal capture active flag;
- terminal distance and signed-along distance;
- terminal speed limits;
- braking distance;
- position capture active flag and reason;
- final hold flag.

When analyzing final behavior, separate normal trajectory-following metrics
from terminal metrics. Final positioning intentionally uses a different mode.

## Why Position Capture Stays

Position capture is intentionally kept as the final precision mode. Earlier
velocity-only terminal behavior could leave the drone spending a long time near
the goal without settling cleanly. PX4 position setpoint mode is designed to
drive to a point and hold it, so it is the correct last step after the fast
trajectory-following phase has slowed down.

The project should not remove position capture merely to make the architecture
look simpler. The right simplification is to make it an explicit state in one
terminal state machine. That keeps the working behavior and removes the
ambiguity of having two independent terminal decision paths.

## State Transition Intent

`cruise` means the normal executable trajectory follower owns horizontal
motion.

`velocity_terminal_capture` means the trajectory follower still publishes
velocity setpoints, but scalar speed is now dominated by braking distance and
terminal remaining distance.

`position_capture` means the final goal point is now the primary target and PX4
position control should settle the vehicle.

`final_hold` means the drone has satisfied final distance and speed conditions.
It should be sticky so small pose noise does not cause mode bouncing.

## Entry Thresholds

Position capture thresholds should be configured and logged because they affect
visible behavior at the end of the mission. Important thresholds include:

- maximum speed for normal position-capture entry;
- speed below which the drone is considered stuck near the terminal region;
- terminal capture radius;
- final acceptance radius;
- final hold maximum speed.

If position capture starts too early, the drone may leave the planned terminal
curve abruptly. If it starts too late, the drone can overshoot and then slowly
return. The goal is to enter position capture after velocity terminal capture
has already made the final point reachable with a small, stable correction.

## Debugging Terminal Behavior

A clean terminal run should show:

- terminal capture activation before the goal, not after a large overshoot;
- terminal speed limit decreasing smoothly;
- position capture entering with a clear reason;
- final hold latching only after distance and speed are both acceptable;
- no repeated switching between velocity and position modes.

If final distance becomes small at high speed and then grows again, braking
started too late or the terminal speed cap was too high. If the drone reaches a
few meters from the goal and waits for a long time, position capture may be
entering too late or the terminal velocity mode may be too conservative.
