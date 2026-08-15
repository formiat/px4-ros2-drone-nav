#include "drone_city_nav/occupancy_grid_3d.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <filesystem>
#include <fstream>

namespace drone_city_nav {
namespace {

void convertToLegacyRawOnlyArtifact(const std::filesystem::path& path,
                                    const std::uint32_t embedded_region_count) {
  std::fstream stream{path, std::ios::binary | std::ios::in | std::ios::out};
  if (!stream) {
    throw std::runtime_error{"failed to reopen temporary Occupancy3D fixture"};
  }
  constexpr std::uint32_t legacy_version{4U};
  constexpr std::uint32_t embedded_traversal_count{0U};
  stream.seekp(8, std::ios::beg);
  // Binary fixture construction intentionally exposes scalar storage.
  // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
  stream.write(reinterpret_cast<const char*>(&legacy_version),
               static_cast<std::streamsize>(sizeof(legacy_version)));
  stream.seekp(0, std::ios::end);
  // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
  stream.write(reinterpret_cast<const char*>(&embedded_region_count),
               static_cast<std::streamsize>(sizeof(embedded_region_count)));
  // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
  stream.write(reinterpret_cast<const char*>(&embedded_traversal_count),
               static_cast<std::streamsize>(sizeof(embedded_traversal_count)));
  if (!stream) {
    throw std::runtime_error{"failed to create temporary legacy Occupancy3D"};
  }
}

TEST(OccupancyGrid3D, StoresSparseVoxelsAcrossChunks) {
  OccupancyGrid3D grid{GridBounds3D{0.0, 0.0, 0.0, 0.5, 40, 40, 40}, 42U};
  grid.setOccupied({0, 0, 0});
  grid.setOccupied({17, 18, 19});
  grid.setOccupied({17, 18, 19});

  EXPECT_TRUE(grid.isOccupied({0, 0, 0}));
  EXPECT_TRUE(grid.isOccupied({17, 18, 19}));
  EXPECT_FALSE(grid.isOccupied({16, 18, 19}));
  EXPECT_EQ(grid.occupiedVoxelCount(), 2U);
  EXPECT_EQ(grid.occupiedChunkCount(), 2U);
  EXPECT_EQ(grid.fingerprint(), 42U);
}

TEST(OccupancyGrid3D, ConvertsWorldCoordinates) {
  OccupancyGrid3D grid{GridBounds3D{-1.0, -2.0, 3.0, 0.5, 4, 6, 8}};
  const std::optional<GridIndex3D> cell = grid.worldToCell({-0.74, -1.26, 4.26});
  if (!cell.has_value()) {
    FAIL() << "expected in-bounds cell";
  }
  const GridIndex3D index = cell.value_or(GridIndex3D{});
  EXPECT_EQ(index, (GridIndex3D{0, 1, 2}));
  const Point3 center = grid.cellCenter(index);
  EXPECT_DOUBLE_EQ(center.x, -0.75);
  EXPECT_DOUBLE_EQ(center.y, -1.25);
  EXPECT_DOUBLE_EQ(center.z, 4.25);
  EXPECT_FALSE(grid.worldToCell({10.0, 0.0, 0.0}).has_value());
}

TEST(OccupancyGrid3D, WritesAndReloadsSparseArtifact) {
  const std::filesystem::path path =
      std::filesystem::temp_directory_path() /
      "drone_city_nav_occupancy_grid_3d_round_trip.occupancy3d";
  std::filesystem::remove(path);
  OccupancyGrid3D original{GridBounds3D{-2.0, 3.0, -4.0, 0.5, 40, 50, 60}, 123456U};
  original.setOccupied({0, 0, 0});
  original.setOccupied({17, 34, 51});

  original.write(path);
  const OccupancyGrid3D loaded = OccupancyGrid3D::load(path);
  std::filesystem::remove(path);

  EXPECT_EQ(loaded.bounds(), original.bounds());
  EXPECT_EQ(loaded.fingerprint(), 123456U);
  EXPECT_EQ(loaded.occupiedVoxelCount(), 2U);
  EXPECT_TRUE(loaded.isOccupied({0, 0, 0}));
  EXPECT_TRUE(loaded.isOccupied({17, 34, 51}));
}

TEST(OccupancyGrid3D, LoadsLegacyArtifactOnlyWhenEmbeddedTopologyIsEmpty) {
  const std::filesystem::path path = std::filesystem::temp_directory_path() /
                                     "drone_city_nav_legacy_raw_only.occupancy3d";
  std::filesystem::remove(path);
  OccupancyGrid3D original{GridBounds3D{-2.0, 3.0, -4.0, 0.5, 40, 50, 60}, 123456U};
  original.setOccupied({17, 34, 51});
  original.write(path);
  convertToLegacyRawOnlyArtifact(path, 0U);

  const OccupancyGrid3D loaded = OccupancyGrid3D::load(path);
  std::filesystem::remove(path);

  EXPECT_EQ(loaded.bounds(), original.bounds());
  EXPECT_EQ(loaded.fingerprint(), original.fingerprint());
  EXPECT_TRUE(loaded.isOccupied({17, 34, 51}));
}

TEST(OccupancyGrid3D, RejectsLegacyArtifactWithEmbeddedTopology) {
  const std::filesystem::path path =
      std::filesystem::temp_directory_path() /
      "drone_city_nav_legacy_embedded_topology.occupancy3d";
  std::filesystem::remove(path);
  const OccupancyGrid3D original{GridBounds3D{-2.0, 3.0, -4.0, 0.5, 40, 50, 60},
                                 123456U};
  original.write(path);
  convertToLegacyRawOnlyArtifact(path, 1U);

  EXPECT_THROW(static_cast<void>(OccupancyGrid3D::load(path)), std::runtime_error);
  std::filesystem::remove(path);
}

TEST(OccupancyGrid3D, LoadsCommittedCompactPassageFixture) {
  const OccupancyGrid3D grid =
      OccupancyGrid3D::load(TEST_COMPACT_PASSAGE_OCCUPANCY3D_PATH);
  const GridBounds3D& bounds = grid.bounds();

  EXPECT_DOUBLE_EQ(bounds.origin_x, -13.0);
  EXPECT_DOUBLE_EQ(bounds.origin_y, -7.0);
  EXPECT_DOUBLE_EQ(bounds.origin_z, -1.5);
  EXPECT_DOUBLE_EQ(bounds.resolution_m, 0.25);
  EXPECT_EQ(bounds.width_cells, 104);
  EXPECT_EQ(bounds.height_cells, 56);
  EXPECT_EQ(bounds.depth_cells, 60);
  EXPECT_EQ(grid.occupiedVoxelCount(), 32604U);
  EXPECT_EQ(grid.occupiedChunkCount(), 88U);
}

} // namespace
} // namespace drone_city_nav
