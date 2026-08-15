# External Environment Candidates

## Scope

This document records the environment-selection and import-feasibility work that
precedes roadmap items 7, 8, and 9. It does not claim that any imported world is
already integrated into a mission or that full-mission validation is complete.

The current generated city remains the deterministic regression fixture. The
external candidates are intended to expose the planner to irregular collision
meshes, non-axis-aligned walls, tunnels, caverns, ramps, and vertical shafts
before the generic passage work starts.

Downloaded source assets and generated candidate artifacts are stored under
`external/environment-candidates/`. The complete local working set is about
2.0 GB and is intentionally ignored by Git.

The distribution contract is committed as
`environments/environment_manifest.yaml`. It contains all three confirmed-fit
external worlds and one compact repository fixture. Candidate evaluation data
is broader than the supported distribution set.

## Selection Criteria

A primary candidate must satisfy all of the following:

- redistribution and local use are permitted by an explicit license;
- collision geometry represents the openings rather than closing them with a
  coarse bounding box;
- the world has enough metric clearance for the configured swept vehicle
  footprint;
- geometry can be resolved deterministically without silently dropping models;
- collision meshes can be converted into raw Occupancy3D without artificial
  clearance inflation;
- the resulting map has bounded dimensions and a practical ESDF representation;
- the environment exercises genuine 3D navigation rather than only a planar
  maze.

The classification below concerns suitability for the next environment-import
and passage-planning stage. It is not evidence that current missions can already
fly through these worlds.

## Classification

| Candidate | Source revision | Classification | Reason |
|---|---:|---|---|
| Finals Prize Round World 07 | Fuel world v1 | **Confirmed fit** | Large hybrid SubT environment combining tunnel, urban, and cave geometry. Collision import, 0.5 m Occupancy3D, ESDF3D generation, and a collision-only Gazebo Harmonic server load all succeeded. This is the primary candidate. |
| Cave Circuit Practice 01 | Fuel world v2 | **Confirmed fit** | Contains elevation pieces, caverns, and multiple explicit vertical-shaft assets. Both 0.5 m and 1.0 m static maps were generated. The 0.5 m map is expensive, so this is the vertical-passage stress world rather than the default world. |
| Urban Circuit Practice 01 | Fuel world v1 | **Confirmed fit** | Multi-level rooms, corridors, bends, and elevation changes. Collision import and a compact 0.5 m static map succeeded. It is a useful secondary indoor/urban candidate. |
| Tunnel Circuit Practice 01 | Fuel world v1 | **Uncertain** | Import and static-map generation succeeded, but the world is mostly a wide, repetitive, near-planar tunnel network. It tests scale and mesh complexity better than arbitrary 3D passages. |
| AWS RoboMaker Hospital, three floors | commit `7161eb8448f5cba6a469da7a79ae5d660b0b7f58` | **Uncertain** | Visually realistic and multi-floor, but it is a legacy Gazebo Classic world with tight indoor clearance. Several disabled props are hidden at approximately `z=-757768997 m`; strict map generation therefore rejects the unmodified world instead of guessing which geometry to discard. A typed migration/filter policy is required. |
| Cave World | Fuel world v1 | **Rejected as primary** | The irregular cave collision mesh imports correctly, but the physical extent is only about 21 x 25 x 8.5 m. Retained locally as a compact mesh-import smoke fixture. |
| AWS RoboMaker Small Warehouse | commit `ee0af733315e78432408c3cd98d378ecee5f767c` | **Rejected as primary** | Only about 18 x 25 x 15 m and has no useful complex 3D passage. Static import is valid, so it remains a small regression candidate. |
| Industrial-Warehouse | Fuel world v4 | **Rejected as primary** | Similar small footprint and no meaningful vertical passage. One dynamic object is correctly omitted from the static map. |

The three confirmed candidates come from the Open Robotics DARPA Subterranean
Challenge environment set. Upstream describes the set as tile-based Tunnel,
Urban, and Cave worlds containing vertical pits and shafts, with larger worlds
spanning hundreds to thousands of metres:
<https://www.openrobotics.org/blog/2022/2/3/subt-part-2-robots-and-environments>.

## Static Map Evidence

All successful maps below were generated from collision geometry at 0.5 m
resolution, except the explicitly listed 1.0 m Cave variant. The ESDF uses the
existing sparse, chunked project format.

| Candidate | Grid dimensions | Dense voxel domain | Collision triangles | Occupied voxels | Occupancy3D | ESDF3D |
|---|---:|---:|---:|---:|---:|---:|
| Finals Prize Round World 07 | 892 x 843 x 136 | 102,266,016 | 2,100,372 | 747,205 | 1.1 MB | 39 MB |
| Cave Circuit Practice 01, 0.5 m | 1203 x 956 x 458 | 526,731,144 | 1,335,543 | 1,638,744 | 2.3 MB | 100 MB |
| Cave Circuit Practice 01, 1.0 m | 602 x 478 x 230 | 66,183,880 | 1,335,543 | 377,852 | 620 KB | 11 MB |
| Urban Circuit Practice 01 | 605 x 528 x 73 | 23,319,120 | 1,811,856 | 356,853 | 624 KB | 9.9 MB |
| Tunnel Circuit Practice 01 | 668 x 668 x 56 | 24,988,544 | 2,586,956 | 1,271,276 | 1.5 MB | 12 MB |
| Cave World | 42 x 50 x 17 | 35,700 | 74,353 | 1,684 | 4 KB | 24 KB |
| AWS Small Warehouse | 36 x 50 x 30 | 54,000 | 1,612 | 9,005 | 8 KB | 8 KB |
| Industrial-Warehouse | 36 x 50 x 30 | 54,000 | 920 | 6,675 | 8 KB | 12 KB |

The Cave 0.5 m build intentionally required an explicit
`--maximum-voxel-count 600000000`. This makes the memory cost visible rather
than silently lowering map resolution. Its 1.0 m derivative is the practical
initial integration artifact, while 0.5 m remains available for geometry and
clearance studies.

The Hospital collision materialization produced 540 collision instances from
69 mesh files, but voxelization stopped at a computed
`208 x 208 x 1515538028` grid. No partial map was accepted. The outlier geometry
must be removed by an explicit source migration based on model identity, not by
an arbitrary coordinate clamp.

## Sources And Licenses

The local source set uses these upstream resources:

- Open Robotics Fuel worlds: [Finals Prize Round World 07 v1](https://fuel.gazebosim.org/1.0/OpenRobotics/worlds/Finals%20Prize%20Round%20World%2007/1),
  [Cave Circuit Practice 01 v2](https://fuel.gazebosim.org/1.0/OpenRobotics/worlds/Cave%20Circuit%20Practice%2001/2),
  [Urban Circuit Practice 01 v1](https://fuel.gazebosim.org/1.0/OpenRobotics/worlds/Urban%20Circuit%20Practice%2001/1),
  [Tunnel Circuit Practice 01 v1](https://fuel.gazebosim.org/1.0/OpenRobotics/worlds/Tunnel%20Circuit%20Practice%2001/1),
  [Cave World v1](https://fuel.gazebosim.org/1.0/OpenRobotics/worlds/Cave%20World/1),
  and [Industrial-Warehouse v4](https://fuel.gazebosim.org/1.0/OpenRobotics/worlds/Industrial-warehouse/4).
  World-level Fuel metadata reported CC BY 4.0 during retrieval; distributed
  transitive model licenses are pinned separately.
- [AWS RoboMaker Hospital World](https://github.com/aws-robotics/aws-robomaker-hospital-world) and [AWS RoboMaker Small Warehouse World](https://github.com/aws-robotics/aws-robomaker-small-warehouse-world), both under the MIT license in their checked-out repositories.
- [OSRF Gazebo Models](https://github.com/osrf/gazebo_models) at commit `8163eb4b5e7e21985c6591d1c0bfb56468c0093f`, sparse-checkout of `sun` and `ground_plane`, under Apache-2.0.

The AWS repositories are archived legacy assets. Their source geometry is useful
for comparison, but their plugins, resource URIs, and rendering materials must
not be treated as native Gazebo Harmonic content.

## Versioned Distribution Contract

Large environment binaries are not stored in normal Git history. The manifest
pins, for every distributed environment:

- upstream URL and version;
- world SPDX license identifier, license URL, and attribution;
- a committed SHA-pinned license inventory for every transitive Fuel resource;
- classification and project role;
- artifact filename, byte size, and SHA-256;
- map resolution, origin, dimensions, collision triangles, occupied voxels,
  and occupied chunks;
- independent SHA-256 contracts for Occupancy3D and ESDF3D.

The release registry contains six deterministic bundles split between the
immutable core and Urban releases:

| Release | Artifact | Approximate compressed size |
|---|---|---:|
| `environment-assets-v1` | Finals Prize Round World 07 source | 565 MB |
| `environment-assets-v1` | Finals Prize Round World 07 0.5 m static map | 39 MB |
| `environment-assets-v1` | Cave Circuit Practice 01 source | 99 MB |
| `environment-assets-v1` | Cave Circuit Practice 01 1.0 m static map | 11 MB |
| `environment-assets-urban-v1` | Urban Circuit Practice 01 source | 32 MB |
| `environment-assets-urban-v1` | Urban Circuit Practice 01 0.5 m static map | 9.9 MB |

Each source bundle contains the selected Fuel world and the exact transitive
model versions needed by its physical geometry. Each map bundle contains the
Occupancy3D, ESDF3D, normalized generation reports, attribution, and a
per-member checksum manifest. Every bundle also carries `LICENSES.json`; the
Finals inventory contains 60 CC-BY-4.0 and 23 CC0-1.0 resources, while the Cave
inventory contains 40 CC-BY-4.0 and 5 CC0-1.0 resources. The Urban inventory
contains 24 CC-BY-4.0 and 8 CC0-1.0 resources. Archives use stable ordering,
timestamps, ownership, permissions, and gzip metadata.

Every published release and its URL are recorded in `artifact_releases`; each
external environment references one release by typed ID. Verified local mirrors
remain under `external/environment-artifacts/releases/` and are ignored by Git.
With the selected release marked `published`, a fresh checkout can download the
exact SHA-pinned files when no local mirror is available.

The repository-owned compact fixture is stored under
`environments/fixtures/compact_3d_passage/`. Its horizontal corridor, roof
opening, and vertical shaft use only SDF boxes and have no external resources.
Its source, collision-only SDF, Occupancy3D, and ESDF3D are small enough to keep
in Git and are checked by both Python contract tests and C++ loaders.

List and verify the complete local contract:

```bash
./scripts/dev_shell.sh python3 scripts/manage_environment_assets.py list
./scripts/dev_shell.sh python3 scripts/manage_environment_assets.py verify
```

Install one artifact from the verified local mirror, or from the published
release after `published` becomes `true`:

```bash
./scripts/dev_shell.sh python3 scripts/manage_environment_assets.py fetch \
  --environment finals_prize_round_world_07 --artifact static_r050
```

Rebuild one release environment after intentionally changing source inputs or
map generation. Omitting `--environment` rebuilds every configured release:

```bash
./scripts/dev_shell.sh python3 scripts/build_environment_release.py \
  --environment urban_circuit_practice_01 --update-manifest
```

Refresh the pinned Fuel metadata only when source dependency versions change:

```bash
./scripts/dev_shell.sh python3 scripts/resolve_environment_licenses.py \
  --environment finals_prize_round_world_07
./scripts/dev_shell.sh python3 scripts/resolve_environment_licenses.py \
  --environment cave_circuit_practice_01
./scripts/dev_shell.sh python3 scripts/resolve_environment_licenses.py \
  --environment urban_circuit_practice_01
```

The builder updates all artifact and repository-fixture hashes atomically in the
typed manifest. Publishing is intentionally a separate authenticated operation;
the repository workflow never pushes a release implicitly.

## Reproducible Import Pipeline

Build through the project container first:

```bash
./scripts/build.sh
```

Materialize one source world into an include-free, collision-only SDF. Every
mesh URI becomes relative to the generated SDF, nested poses are composed, static
collision instances are flattened, and unsupported or unresolved geometry is a
hard error:

```bash
./scripts/dev_shell.sh python3 scripts/materialize_sdf_collisions.py \
  --world "external/environment-candidates/fuel/fuel.gazebosim.org/openrobotics/worlds/finals%20prize%20round%20world%2007/1/final_event_07.sdf" \
  --fuel-cache external/environment-candidates/fuel \
  --model-path external/environment-candidates/sources/gazebo_models \
  --output-sdf external/environment-candidates/work/final_event_07_collisions.sdf \
  --report external/environment-candidates/work/final_event_07_materialization.json
```

Voxelize the physical collision triangles without clearance inflation:

```bash
./scripts/dev_shell.sh ./build/drone_city_nav/voxelize_sdf_collisions \
  --sdf external/environment-candidates/work/final_event_07_collisions.sdf \
  --output external/environment-candidates/work/final_event_07_r050.occupancy3d \
  --report external/environment-candidates/work/final_event_07_r050_voxelization.json \
  --resolution-m 0.5 --margin-m 2
```

Generate the normal project ESDF cache:

```bash
./scripts/dev_shell.sh ./build/drone_city_nav/generate_static_esdf_cache \
  --occupancy external/environment-candidates/work/final_event_07_r050.occupancy3d \
  --output external/environment-candidates/work/final_event_07_r050.esdf3d \
  --maximum-distance-m 26 --workers 8
```

`voxelize_sdf_collisions` supports collision meshes, boxes, and planes. It uses
exact triangle-versus-voxel intersection tests rather than marking every cell in
a triangle AABB. Output dimensions and the total dense domain are checked before
allocation. The map fingerprint covers both the path-independent materialized
SDF and every unique collision mesh. Rebuilding the same unpacked source in a
different root therefore produces byte-identical Occupancy3D and ESDF3D. The
sparse Occupancy3D writer emits raw physical voxels only. Previously published
candidate bundles use the accepted legacy v4 raw-only encoding; regenerated
maps use raw-only v5.

## Current Limits And Next Decision

- The imported environments do not yet have separate FreeSpaceTopology3D
  artifacts. Generic portal and free-volume extraction belongs to roadmap item 7.
- Candidate visuals are not yet a production Harmonic world. The Finals
  collision-only world loads in a Harmonic server, but the original legacy
  materials and resource URIs still need a visual migration pass.
- Mission starts, goals, map-to-SDF transforms, world launch selection, and
  flight-envelope bounds have not been defined for any candidate.
- No navigation, interception, or cooperative mission was run in an imported
  environment during this selection stage.
- A static map proves deterministic collision conversion, not route
  reachability. Passage width and swept-footprint feasibility must be measured
  after generic free-volume extraction.

Proceed with Finals Prize Round World 07 as the primary integration target,
Cave Circuit Practice 01 as the vertical-shaft stress target, and Urban Circuit
Practice 01 as the smaller secondary target. Do not replace the current city
fixture until at least one complete mission is reproducibly validated in the
new environment.
