# Compact 3D Passage Fixture

This repository-owned world is a deterministic integration fixture for the SDF
collision materializer, collision voxelizer, Occupancy3D serializer, static
ESDF loader, and generalized FreeSpaceTopology3D compiler. It is intentionally
small and is not a visual-quality mission environment.

The fixture contains a roofed horizontal corridor and a 4 x 4 metre opening
into a vertical shaft. All geometry uses SDF boxes, so the world has no external
model or texture dependencies.

`world.topology3d` is compiled from `world.occupancy3d` and `world.esdf3d` with
the profile pinned in `environments/environment_manifest.yaml`. It contains two
arbitrarily oriented portal voxel patches joined by one sparse medial passage
segment. Rebuild and verify it through the project container:

```bash
python3 scripts/compile_environment_topology.py \
  --environment compact_3d_passage_fixture --static-map r025
```

The source and generated artifacts are covered by the repository MIT license.
