# Step 1. What This Is

A guide in six steps for someone who meets the project for the first time.
Each step explains and leads; the numbers and the rules live in the
reference documents it links to. The idea of such a course, and of the
[map of the architecture](architecture_map.md) beside it, is hundong2's
([forks.md](../forks.md)); the text is written here, on today's code.

1. What this is (this page).
2. [Install it and fly once](02_install_first_flight.md).
3. [The mission and the pipeline from a sensor to a command](03_mission_and_pipeline.md).
4. [What safety means here](04_safety.md).
5. [Testing a change and reading a flight](05_testing_debugging.md).
6. [Extending the stack](06_extending.md).

## The Vehicle And Its Task

A quadcopter flies by itself through places it has never seen: a town with
buildings to fly through, a cave, tunnels, the open sky. It is given a goal
and nothing else: no map, no satellite signal by default (V2), no list of
what is forbidden. It builds its own picture of the obstacles from what its
sensors measure, plans through it, and flies.

Two requirements judge every flight, and both are read against the
simulator's truth, never against what the vehicle believes:

- **it never touches anything** (R1; a contact of any kind is a crash, A9);
- **it reaches its goal** (R2; inside the capture radius, V6).

Speed comes third and is measured, not promised (R3). Everything else a
flight's check prints is a note (R4).

## What It Is Made Of

- **PX4** is the autopilot. It keeps the vehicle level and flies the
  setpoints it is given; it runs here as software in the loop.
- **Gazebo** is the simulator: the world, the vehicle's body and its sensors.
- **ROS 2** carries the messages between the stack's nodes.
- **The stack** is this repository's package, `drone_city_nav`: it estimates
  the vehicle's pose, remembers obstacles, plans, and hands the autopilot a
  short piece of trajectory at a time. The
  [map of the architecture](architecture_map.md) names every part.

The vehicle carries one of two sensor sets (V1): a stereo pair of cameras
with a ring of time-of-flight sensors and a light of its own, or a 3D lidar.
The first is the default; the acceptance of a change flies both (A1).

## Three Ideas To Hold On To

1. **Unknown is not forbidden.** Space no sensor has looked at is free to
   plan through (I1); space is closed only by a measurement (I2); nothing is
   closed because of what happened before (I3). The vehicle stays safe not
   by avoiding the unknown but by never moving faster than it can stop
   inside what it has seen ([step 4](04_safety.md)).
2. **The code knows no place.** Nothing in the flight code names a world, a
   doorway or a coordinate (I5): a world the stack has never flown must be
   flyable with nothing adapted.
3. **A measurement decides.** A rule, a threshold or a claim in the
   documents stands on a flight or a recording that anyone can repeat; the
   [specification](../specification.md) says, row by row, who set a rule and
   why.

## Where To Read More

- [overview.md](../overview.md): the reference documents, by subject.
- [specification.md](../specification.md): the requirements, the
  invariants and the numbers the work relies on.
- [roadmap.md](../roadmap.md): what is done and what is next.
- [related_work.md](../related_work.md): how other open projects solve the
  same problems, and what was taken or left.

Next: [install it and fly once](02_install_first_flight.md).
