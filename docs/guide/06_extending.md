# Step 6. Extending The Stack

Where a change goes, what it must not break, and how it is accepted. The
rules of work are in [CONTRIBUTING.md](../../CONTRIBUTING.md) and
[development.md](../development.md); the C++ rules in
[cpp_best_practices.md](../cpp_best_practices.md).

## Before You Write

1. Find the component on the [map of the architecture](architecture_map.md)
   and read the specification's row beside it: the row says what is fixed,
   who fixed it and by which measurement.
2. Read the [roadmap](../roadmap.md): the item may exist, with its
   prerequisites and the criterion it is complete by.
3. Read the [register of debt](../technical_debt.md): the defect may be
   known, with the options already weighed.

## The Invariants A Change Keeps

- Unknown space stays free to plan through; a prohibition comes from a
  measurement; nothing latches on history (I1, I2, I3).
- Vertical motion is free in the planner's time model (I4).
- No world, opening or coordinate in the flight code (I5).
- The simulator's truth is for the checks and the analysis tools alone.
- No code in the flight that exists only for a test.

A point in dispute takes the most conservative option that passes the
acceptance.

## Where Things Go

| You want to | It goes in | Read first |
|---|---|---|
| read a new sensor | a node of its own that publishes what the memory or an estimator already takes; its mount in [sensor_extrinsics.yaml](../../drone_city_nav/config/sensor_extrinsics.yaml) (K61) | [camera_perception.md](../camera_perception.md), [obstacle_mapping.md](../obstacle_mapping.md) |
| change how obstacles are remembered | the obstacle memory | [obstacle_mapping.md](../obstacle_mapping.md) |
| change the route or the controller | the planner's tick | [trajectory_optimization.md](../trajectory_optimization.md), [replanning.md](../replanning.md) |
| change how fast the vehicle may fly | the speed laws, never the contract's checks | [navigation_pipeline.md](../navigation_pipeline.md) |
| add a mission | a node that chooses goals and sends them on the operator's channel (K31), as the survey does | [missions.md](../missions.md) |
| add a world | the environment's manifest and a scenario proven inside the world (K63) | [environment_candidates.md](../environment_candidates.md), [scenarios.md](../scenarios.md) |
| add a parameter | the node's declaration and [urban_mvp.yaml](../../drone_city_nav/config/urban_mvp.yaml) together: the file overrides the default | [configuration.md](../configuration.md) |
| add a tool that reads flights | [tools](../../tools), with a row in [tools/README.md](../../tools/README.md) | [tools/README.md](../../tools/README.md) |

## From A Change To An Accepted Change

1. A unit test beside the code ([testing.md](../testing.md)); for anything
   a recorded flight can prove, the replay of its bag before any flight
   (K62).
2. The gate ([step 5](05_testing_debugging.md)); every warning of the build
   is repaired in the same commit.
3. The flights the change owes (A14): the scenario it acts on, then the
   acceptance of five and five (A1).
4. The records in the same commit: the specification's row with a line in
   its change log, the roadmap's item, the register of debt (a debt repaired
   is removed), the changelog.

A change that makes the stack heavier is measured before it is kept: the
onboard budget is a measurement on the host of record, never a figure set by
hand (K53).

## Other People's Work

What is taken from a fork or another project is signed with its author and
listed in [forks.md](../forks.md) and [related_work.md](../related_work.md);
a fork's commit is read, gated and flown before it is trusted.

Back to [step 1](01_basics.md) or to the
[map of the architecture](architecture_map.md).
