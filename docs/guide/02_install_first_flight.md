# Step 2. Install It And Fly Once

From an empty directory to a flight that passed its check, with one command.
Everything is built and run inside a container, so the host needs little;
the reference is [installation.md](../installation.md) and
[build_and_run.md](../build_and_run.md).

## What The Host Needs

A Linux host with Docker and git, and an NVIDIA GPU with its container
toolkit for the default engine of the controller (the stack also flies on a
CPU alone, K54). The exact list, and what not to install on the host, is in
[installation.md](../installation.md).

## One Command

```bash
git clone https://github.com/formiat/px4-ros2-drone-nav.git
cd px4-ros2-drone-nav
./scripts/bootstrap.sh --headless
```

The script does, in order, and skips whatever is already in place:

1. checks the host;
2. builds the development image, or takes the one that exists;
3. clones PX4 at the version the project flies, with its submodules, and
   builds its software-in-the-loop binary;
4. fetches the library that lets the stack fly as a mode of PX4's own (K91);
5. builds the workspace;
6. installs the Urban location's assets;
7. flies the urban point-to-point mission without windows and runs the
   mission check on it.

On the day this page was written the whole of it, in a fresh clone on a
machine that already held the image, took 28 minutes, the flight included,
and left a tree of 9.4 GB.

## What A Passed Flight Looks Like

The check prints one line per thing it verified. The first flight of that
fresh clone ended with 64 lines of `OK`, none of `FAIL`, among them:

```text
OK: mean flight speed is 1.392 m/s: 464.0 m in 333.3 s of simulation time (1.064 m/s on the wall clock, 436.2 s)
OK: the true position is 0.51 m from the goal at its acknowledgement (capture radius 2.0 m)
OK: no crash was reported
OK: the vehicle landed and disarmed at the mission's end
```

The vehicle took off inside a building on the stereo cameras alone, with no
map and no satellite signal, flew 464 m through the town to its goal and
landed. "In truth" means against the simulator's own knowledge of where the
vehicle was, which the vehicle never reads ([step 1](01_basics.md)). Your
figures will differ a little from flight to flight: the simulated sensors
carry noise and the simulator is not deterministic.

## Flying Again

```bash
./scripts/sim_urban_point_to_point_headless.sh         # the stereo set
./scripts/sim_urban_point_to_point_lidar_headless.sh   # the 3D lidar
./scripts/stop_sim.sh                                  # stop whatever still runs
```

One flight at a time: a second simulation on the same checkout is refused,
and `stop_sim.sh` stops only what its own checkout started. A run's records
are kept under `log/runs/` for a week
([step 5](05_testing_debugging.md) reads them).

The same flights with Gazebo's and RViz's windows, the other worlds (a cave,
a tunnel world, the open sky) and the cooperative mission are in
[build_and_run.md](../build_and_run.md) and [missions.md](../missions.md).

## When It Does Not Work

[troubleshooting.md](../troubleshooting.md) lists what goes wrong most
often: Docker's access, the GPU inside the container, a simulation left
running.

Next: [the mission and the pipeline](03_mission_and_pipeline.md).
