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
