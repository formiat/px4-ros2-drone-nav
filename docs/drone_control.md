# Drone Control

The offboard controller tracks the accepted executable trajectory with PX4
offboard setpoints. Normal flight uses velocity setpoints. Final positioning
uses terminal position capture when the terminal state machine enters that
state.

## Why Velocity Setpoints

Velocity setpoints are used because the planner produces a continuous
trajectory and speed profile, not a PX4 waypoint mission. Velocity control lets
the stack:

- follow a smoothed trajectory;
- react to replans;
- apply custom lateral control;
- keep speed decisions tied to curvature and terminal state;
- preserve a clear separation between planner geometry and runtime control.

## Executable Trajectory Follower

The offboard node receives `/drone_city_nav/path`, builds trajectory samples,
and computes a runtime speed profile. The current control point is projected
onto these samples.

The controller uses a predicted position:

```text
predicted_position = current_position + current_velocity * tracking_prediction_horizon_s
```

This makes the projection speed-aware: at higher speed the projection point is
farther ahead in meters for the same time horizon.

## Lateral Control

The lateral command is based on:

- cross-track P control;
- cross-track D control;
- curvature feedforward.

Conceptually:

```text
lateral_control =
  cross_track_feedback
  + cross_track_derivative_damping
  + curvature_feedforward
```

P control pulls the drone toward the trajectory. The P gain is scheduled by
cross-track error:

```text
feedback = error * cross_track_gain * cross_track_p_gain_factor
```

D control damps lateral velocity relative to the trajectory normal. The D gain
has a speed-aware factor:

```text
effective_D = cross_track_derivative_gain * cross_track_d_gain_factor
```

Curvature feedforward anticipates real turns. Deadband/full-angle settings
reduce feedforward on nearly straight or sign-changing micro-curvature.

## Projection Smoothing

Control projection smoothing can smooth the tangent and curvature used by the
controller. This is control-only smoothing; it does not change the published
trajectory geometry.

There are two practical cases:

- straight-ish windows, where tangent noise should not produce lateral
  oscillation;
- curve windows, where tangent and curvature should remain consistent.

Diagnostics include whether smoothing was applied and which mode was used.

## Speed Policy

Runtime scalar speed is based on:

- cruise speed;
- curvature limits;
- minimum turn speed;
- lookahead;
- acceleration/deceleration policy;
- terminal capture state.

The planner also emits speed diagnostics, but offboard runtime speed is
authoritative for actual control.

## Velocity Smoother

The velocity smoother is the final setpoint-shaping layer before PX4. It limits
velocity-vector changes and jerk. It is the main runtime layer that prevents
abrupt setpoint changes from reaching PX4.

Relevant parameters include:

- `setpoint_forward_accel_mps2`
- `setpoint_forward_decel_mps2`
- `setpoint_lateral_response_accel_mps2`
- `max_velocity_jerk_mps3`
- `max_lateral_velocity_jerk_mps3`

## Trajectory Updates

When a new trajectory arrives, the offboard node evaluates continuity:

- projection jump;
- tangent jump;
- curvature jump;
- speed-limit jump;
- tangent-speed command jump.

The decision can preserve smoother history, reset the smoother, or reject the
candidate trajectory. Rejected trajectories do not delete the current accepted
trajectory.

## Terminal Mode

Near the goal, control transitions into the terminal state machine described in
`terminal_capture.md`.
