# Open Sky: Baylands

The open-sky world of roadmap item 39 (stage 0): the autopilot's own
`baylands` world (`PX4-Autopilot/Tools/simulation/gz/worlds/baylands.sdf`),
a park by the water with trees, paths and a few buildings under the sun and a
sky, carried as a fixture of the environment pipeline so that the pipeline
materialises it as it does every world: the two Fuel models it includes
(`OpenRobotics/baylands` version 3, `OpenRobotics/Coast Water` version 3) are
the manifest's visual dependencies and are downloaded into the installed
environment's `visual_resources`; the water's pose is written in the world
frame (the materializer reads no `relative_to`). The world's spherical
coordinates are the autopilot's (37.412 N, 121.999 W), the satellite sensor
of the vehicle reads them. The truth grid is built from the materialised
collisions as for every environment ([`docs/environment_candidates.md`](../../../docs/environment_candidates.md)).
