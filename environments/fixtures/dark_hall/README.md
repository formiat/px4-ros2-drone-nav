# Dark Hall

The large dark space of roadmap item 29: a closed hall 80 x 60 m and 20 m
high with no light of its own, the vehicle's pad 4 m from its west wall at
(4, 30), the vehicle facing the wall as it stands (its estimator starts on
what it sees there, as on any start), and the goal 64 m east. Flown at 6 m
above the floor, the
carried light reaches the floor only at the bottom edge of the pair's
frame and nothing ahead of it: the space a camera vehicle cannot tell from
a failed light by its frames alone. The floor and the walls carry a concrete
texture of the location's own statistics (Urban Circuit Practice 01's dirty
concrete: linear albedo 0.10, deviation 0.03) with its aggregate, stones of
one to four centimetres whose edges give the feature tracker its corners (a
smooth noise alone left the estimator nothing to start on, r1468), in 10 m
tiles; the ceiling is a plain dark grey.

`generate.py` writes `world.sdf` and `texture.png`, the same files on every
run. The grids are built from the world as for every environment
([`docs/environment_candidates.md`](../../../docs/environment_candidates.md)),
inside the project container after `make build`:

```bash
python3 scripts/materialize_sdf_collisions.py --world environments/fixtures/dark_hall/world.sdf \
  --output-sdf environments/fixtures/dark_hall/world.collisions.sdf --report /tmp/dark_hall.json
./build/drone_city_nav/voxelize_sdf_collisions --sdf environments/fixtures/dark_hall/world.collisions.sdf \
  --output environments/fixtures/dark_hall/world.occupancy3d --report /tmp/dark_hall_r025.json \
  --resolution-m 0.25 --margin-m 2
./build/drone_city_nav/generate_static_esdf_cache --occupancy environments/fixtures/dark_hall/world.occupancy3d \
  --output environments/fixtures/dark_hall/world.esdf3d --maximum-distance-m 8 --workers 8
python3 scripts/compile_environment_topology.py --environment dark_hall_fixture --static-map r025 \
  --output environments/fixtures/dark_hall/world.topology3d
```

`world.occupancy3d` is also the truth the flights' checks measure against.
The world and its generated files are covered by the repository MIT
license.
