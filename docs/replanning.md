# Receding-Horizon Updates And Guide Replacement

The current stack does not perform legacy full/partial path replanning.

## Local Horizon Updates

GPU MPPI recomputes a local horizon at the configured planning frequency. It
uses:

- current pose and velocity;
- the latest complete ESDF revision;
- the active global lattice guide;
- the previous control sequence as a warm start;
- the latest applied-control feedback.

Only a fresh timestamped horizon is executable. Offboard never continues an
expired horizon as though it were a long accepted route.

## Sticky Global Guide

The global lattice guide is persistent across ordinary ESDF revisions. A newly
calculated guide does not replace it merely because it has a slightly different
score.

The active guide can be released when it is:

- blocked by current raw occupancy;
- exhausted below the mode-specific remaining-distance threshold;
- too far from the current vehicle position;
- stalled according to along-guide progress;
- superseded by mission-goal completion.

Heading for a replacement guide comes from velocity at speed, the previous
accepted-guide tangent at low speed, or yaw/goal direction as the final
fallback.

## Liveness

The liveness monitor distinguishes predicted progress from actual vehicle
motion. A horizon that repeatedly predicts useful terminal motion while the
vehicle remains stationary is considered ineffective.

Recovery can:

- reseed the local MPPI nominal control sequence;
- release a stalled active guide so the lattice can build another guide;
- brake or hold when no executable local continuation exists.

The current lattice still has limited recovery primitives and no persistent
topological memory. A dead end may therefore end in hold rather than a complete
route around the obstacle.

## Safety During Updates

There is no prefix/suffix stitch or safe truncation. Safety comes from:

- short overlapping horizons;
- ESDF and known-solid collision evaluation;
- first-control continuity relative to the command actually sent;
- time-to-collision and braking fallback;
- horizon validity deadlines.

If a replacement horizon is unsafe, the system publishes braking behavior. It
does not preserve a physically blocked old trajectory just because a new plan
failed.

## Removed Protocols

The following concepts are not part of production runtime:

- partial A* repair;
- repair margins and parallel repair races;
- blocked-span suffix stitching;
- safe truncation;
- truncation generation or fingerprints;
- moving/after-hold successor negotiation;
- path-id ACK/coalescing lifecycle.
