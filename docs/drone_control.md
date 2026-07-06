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

## Control Loop Sequence

The normal horizontal loop is:

1. read current PX4 local position and velocity;
2. predict a short-horizon control position;
3. project that predicted position onto the accepted executable trajectory;
4. optionally smooth the projection frame for control only;
5. read runtime scalar speed from the speed policy;
6. build forward velocity along the trajectory tangent;
7. build lateral correction from P, scheduled D, and curvature feedforward;
8. apply the lateral angle safety cap;
9. pass the desired velocity through the velocity smoother;
10. publish the PX4 offboard setpoint.

This sequence explains why the same trajectory can be visually smooth but still
produce a tracking problem. The trajectory is only one input. Projection,
prediction, lateral control, speed policy, smoother history, and PX4 response
all shape the actual motion.

## P, D, And Feedforward Roles

P control reacts to position error. If the drone is left or right of the
trajectory, P generates a normal velocity component toward the trajectory. The
P gain schedule changes how strong that pull is near the line versus farther
away.

D control reacts to normal velocity. If the drone is already moving toward the
line quickly, D damps the command so it does not overshoot as much. If the
drone is moving away from the line, D helps reverse that motion. The
speed-aware D factor makes the damping stronger at high speed when a small
normal velocity error can grow quickly.

Curvature feedforward is different. It does not wait for cross-track error. It
adds lateral command because the path ahead is curved. This is useful on real
turns and harmful on tiny sign-changing curvature noise. That is why the
feedforward has deadband and context suppression for nearly straight or
S-shaped windows.

## Projection Smoothing Rationale

The controller can see a local tangent that changes every few meters even when
the visible path looks mostly straight. If the velocity vector follows every
small tangent change, the drone can roll left and right on a straight segment.

Projection smoothing reduces this problem by stabilizing the control frame. It
does not rewrite the trajectory. RViz still shows the real executable path.
Only the tangent and curvature used by the command planner can be adjusted.

The smoothing mode matters:

- straight-ish smoothing should make micro-waves quieter;
- curve smoothing should keep tangent and curvature consistent on real arcs;
- if smoothing suppresses curvature too much, turn entry can become late;
- if smoothing is too weak, the controller can chase path noise.

The logs should show which mode was applied so that a flight issue can be tied
to a specific smoothing decision.

## Runtime Speed Versus Setpoint Dynamics

Runtime speed policy and velocity smoothing solve different problems. Runtime
speed policy chooses a target scalar speed along the trajectory. It looks at
curvature, lookahead, terminal braking, and acceleration/deceleration policy.

The velocity smoother shapes the transition from the previous setpoint to the
new desired setpoint. It limits command change and jerk before PX4 sees the
command.

This split is important. If the drone enters a turn too fast because the speed
profile is optimistic, the speed policy should be fixed. If the desired command
is correct but the setpoint cannot rotate quickly enough, the smoother or
trackability logic is involved. If the setpoint is correct but actual velocity
lags, the physical PX4 response or vehicle limits are involved.

## Reading Lateral Oscillation Logs

For left-right oscillation on a nearly straight segment, compare:

- signed cross-track error and zero crossings;
- normal velocity and normal velocity zero crossings;
- desired normal velocity;
- smoothed setpoint normal velocity;
- actual normal velocity;
- P factor and effective D factor;
- curvature feedforward and its context scale;
- projection smoothing mode.

If desired normal velocity changes sign often, the issue is upstream of the
smoother. Look at local tangent, curvature feedforward, and P gain near the
path. If desired normal velocity is stable but setpoint normal velocity lags or
clips, the smoother is the active limiting layer. If setpoint is stable but
actual velocity oscillates, PX4 or vehicle response is the likely bottleneck.

## Reading Turn Misses

For a large departure in a turn, separate module speed from velocity direction.
A correct scalar speed can still fail if the velocity vector points poorly at
turn entry or if the drone carries outward normal velocity from the previous
segment.

Useful questions:

- Was the commanded scalar speed consistent with curvature radius?
- Was desired normal velocity pointing into the turn early enough?
- Was actual normal velocity still outward at turn entry?
- Did the smoother prevent the setpoint from rotating quickly enough?
- Did projection smoothing or feedforward suppress the curve signal?
- Did the path radius shrink quickly over a short distance?

This distinction prevents blaming only the speed profile when the real problem
is vector direction or setpoint lag.
