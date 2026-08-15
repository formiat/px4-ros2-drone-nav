#include "drone_city_nav/collision_voxelizer.hpp"

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <string>
#include <utility>
#include <vector>

namespace drone_city_nav {
namespace {

TEST(CollisionVoxelizer, DetectsTriangleBoxIntersectionWithoutAabbFalsePositive) {
  const CollisionTriangle3D intersecting{
      {-0.25, -0.25, 0.0}, {0.25, -0.25, 0.0}, {0.0, 0.25, 0.0}};
  const CollisionTriangle3D separated{
      {0.6, 0.6, 0.0}, {0.9, 0.6, 0.0}, {0.6, 0.9, 0.0}};

  EXPECT_TRUE(
      triangleIntersectsAxisAlignedBox(intersecting, Point3{}, Vec3{0.5, 0.5, 0.5}));
  EXPECT_FALSE(
      triangleIntersectsAxisAlignedBox(separated, Point3{}, Vec3{0.5, 0.5, 0.5}));
}

TEST(CollisionVoxelizer, MarksOnlyPhysicallyIntersectedVoxel) {
  OccupancyGrid3D occupancy{GridBounds3D{0.0, 0.0, 0.0, 1.0, 4, 4, 4}};
  const std::vector<CollisionTriangle3D> triangles{
      CollisionTriangle3D{{1.1, 1.1, 1.5}, {1.8, 1.1, 1.5}, {1.1, 1.8, 1.5}}};

  const CollisionVoxelizationStats stats =
      voxelizeCollisionTriangles(triangles, occupancy);

  EXPECT_EQ(stats.triangles, 1U);
  EXPECT_EQ(stats.newly_occupied_voxels, 1U);
  EXPECT_EQ(occupancy.occupiedVoxelCount(), 1U);
  EXPECT_TRUE(occupancy.isOccupied({1, 1, 1}));
  EXPECT_FALSE(occupancy.isOccupied({0, 1, 1}));
  EXPECT_FALSE(occupancy.isOccupied({2, 1, 1}));
}

TEST(CollisionVoxelizer, FingerprintChangesWithCollisionMeshContent) {
  const std::filesystem::path directory = std::filesystem::temp_directory_path() /
                                          "drone_city_nav_collision_fingerprint_test";
  std::filesystem::create_directories(directory);
  const std::filesystem::path sdf = directory / "world.sdf";
  const std::filesystem::path mesh = directory / "collision.dae";
  {
    std::ofstream{sdf} << "materialized world";
    std::ofstream{mesh} << "mesh revision one";
  }

  const std::uint64_t original = fingerprintCollisionInputs(sdf, {mesh, mesh});
  std::ofstream{mesh} << "mesh revision two";
  const std::uint64_t changed = fingerprintCollisionInputs(sdf, {mesh});

  std::filesystem::remove_all(directory);
  EXPECT_NE(original, changed);
}

TEST(CollisionVoxelizer, FingerprintDoesNotDependOnInstallationRoot) {
  const std::filesystem::path directory =
      std::filesystem::temp_directory_path() /
      "drone_city_nav_collision_fingerprint_root_test";
  const auto create_inputs = [&](const std::string& root_name) {
    const std::filesystem::path root = directory / root_name;
    const std::filesystem::path work = root / "work";
    const std::filesystem::path fuel = root / "fuel";
    std::filesystem::create_directories(work);
    std::filesystem::create_directories(fuel);
    std::ofstream{work / "world.sdf"}
        << "<mesh><uri>../fuel/collision.dae</uri></mesh>";
    std::ofstream{fuel / "collision.dae"} << "identical collision mesh";
    return std::pair{work / "world.sdf", fuel / "collision.dae"};
  };
  std::filesystem::remove_all(directory);
  const auto [first_sdf, first_mesh] = create_inputs("first");
  const auto [second_sdf, second_mesh] = create_inputs("second");

  const std::uint64_t first = fingerprintCollisionInputs(first_sdf, {first_mesh});
  const std::uint64_t second = fingerprintCollisionInputs(second_sdf, {second_mesh});

  std::filesystem::remove_all(directory);
  EXPECT_EQ(first, second);
}

} // namespace
} // namespace drone_city_nav
