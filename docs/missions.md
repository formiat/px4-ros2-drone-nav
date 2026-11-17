# Missions

A mission is work the vehicle does for somebody: it is given an order, flies
without a pilot and comes back with the result the order asked for. A
scenario ([`scenarios.md`](scenarios.md)) is a flight that tests the
navigation: its goal point is the test's, and its result is a verdict on the
stack. Until roadmap item 34 every flight of this repository was a scenario:
the flight to a point, the return home, the failures on the way, the smoke,
the bodies that walk. They prove what a mission stands on (the vehicle does
not crash, R1; it arrives, R2), and none of them leaves anything behind but a
verdict.

The survey is the first mission (the owner, 2026-11-17). The missions to come
on the same core (a patrol, the inspection of an object, a search by heat, the
following of a target) are roadmap item 47.

## The Survey Of A Location

**The order.** A volume: a box in the map frame, the part of a building, a
cave or a tunnel system somebody wants a map of (`survey_volume_m` of the
mission's file, for example
`drone_city_nav/config/urban_circuit_practice_01_survey_scenario.json`).
Nothing else: no goal point, no map of the place, no satellite signal. The
vehicle knows no location (the same code flew the three worlds below with
nothing adapted).

**The flight.** The vehicle takes off and flies to the frontiers of its own
map, the boundary between the free space it has seen and the unknown inside
the volume; it looks at each from a place it can reach, and chooses the next
(specification F20 and K59; how the layer works is
[`navigation_pipeline.md`](navigation_pipeline.md), section 9). When no
frontier is left that leads into the volume, it comes home along its own
track, lands and disarms. It comes home earlier when the battery's time left
falls under 1.1 times the way home (K32); the map is then of what it saw.
The mission chooses goals for the one planner and adds no law of its own: in
transit nothing prefers the unknown, a prohibition comes from a measurement
only, and every contact is a crash, as in every flight.

**The result.** The map. The vehicle's obstacle memory at the end of the
flight is persisted as the place's static map (`tools/survey_map.py`, K67):
the occupied cells at 0.25 m in the stack's own format, the cells the
vehicle observed, and a manifest (the place and its geometry, the run and
the commit, the frame, the resolution, the volume, the coverage). In the
simulator, where the truth is known, `tools/survey_map_check.py` reads the
map against it: coverage, alignment, displaced surfaces, and cells the map
calls flyable that are not.

**What the map is for.** The next flight in the same place starts with it as
the first content of its memory (`MEMORY_PRIOR_OCCUPANCY_3D_PATH`, K71): the
route is planned through what the survey saw from the first second, and what
the flight sees still decides wherever the two differ. The record of the
flight opens in a viewer without the stack (`tools/view_flight.sh`).

**Done** (R9) when the map covers at least 0.95 of the truth's flyable cells
of the volume that the body reaches, with no contact, the vehicle home within
the capture radius, landed and disarmed. The mean speed is a figure of a
survey, not a requirement.

**Measured** (2026-11-10, commit 00d1e1c6, each flight one that counts; the
whole record is roadmap item 11's):

| Place, volume | Sensors | Flights | Coverage | Flown, m | Home and landed | Contact |
|---|---|---|---|---|---|---|
| Urban, the start's building and the west corridor (52 x 46 x 16 m) | cameras | 3 | 0.998 to 0.999 | 600 to 1156 | 3 of 3 | none |
| the same | 3D lidar | 3 | 1.00 | 611 to 695 | 3 of 3 | none |
| Finals, the staging area (30 x 60 x 12 m) | 3D lidar | 2 | 0.994, 1.00 | 948, 1446 | 2 of 2 | none |
| Cave, the start cavern (30 x 60 x 12 m) | 3D lidar | 2 | 1.00 | 1420, 2161 | 2 of 2 | none |

Six maps made from those flights passed their check with no cell called
flyable by the flown path that the truth holds occupied; the lidar's are
aligned with the truth to 0.00 to 0.25 m at the median, the cameras' to
0.25 m. Six flights to a goal inside the Urban volume started from a map,
none with a contact.

**To fly it in the simulator:**

```bash
./scripts/sim_urban_survey_headless.sh          # the camera set; _gui.sh shows it
./scripts/sim_urban_survey_lidar_headless.sh    # the 3D lidar
python3 tools/survey_map.py rNNNN --out log/tools/maps/urban --truth log/tools/truth25/urban_r025.occupancy3d
MEMORY_PRIOR_OCCUPANCY_3D_PATH=log/tools/maps/urban/occupancy.occupancy3d \
  ./scripts/sim_urban_point_to_point_in_volume_headless.sh
```

The finals world's and the cave's surveys are `./scripts/sim_finals_survey_*`
and `./scripts/sim_cave_survey_*`; the terminal shows `SURVEY   GOAL N`, then
`SURVEY DONE -> HOME`, `SURVEY COMPLETE`, `LANDED`.

**What the mission does not do yet:**

- It has flown in the simulator only; no real vehicle has flown it.
- The order is a file read at the start. There is no ground station and no
  link to an operator: the volume cannot be given or changed in flight, and
  the map is read after the landing (roadmap item 47).
- A volume the size of a building is held; a whole district is not (345 x
  525 x 40 m was refused by the reading's budget: the technical debt).
- One vehicle. A survey divided between several is roadmap items 15 and 48.
- The map is proven against the simulator's truth and by flights that also
  see. A flight that trusts the map alone, its sensors off, has not been
  flown (roadmap item 54).
- A grating the lidar sees through and a way to a vantage round the outside
  of a building are known weaknesses (the technical debt's rows on the
  survey).
- On the development host the camera set surveys at half of real time.
