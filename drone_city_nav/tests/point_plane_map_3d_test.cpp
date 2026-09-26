#include "drone_city_nav/point_plane_map_3d.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <limits>
#include <vector>

namespace drone_city_nav {
namespace {

// The corner of a room: the floor and two walls, a point every 0.1 m over
// 4 m by 4 m, in the world frame.
std::vector<Eigen::Vector3d> roomCorner() {
  std::vector<Eigen::Vector3d> points;
  for (int i = 0; i < 40; ++i) {
    for (int j = 0; j < 40; ++j) {
      const double a = 0.05 + 0.1 * static_cast<double>(i);
      const double b = 0.05 + 0.1 * static_cast<double>(j);
      points.emplace_back(a, b, 0.0);
      points.emplace_back(0.0, a, b);
      points.emplace_back(a, 0.0, b);
    }
  }
  return points;
}

PointPlaneRegistrationConfig3D registrationConfig(const std::int64_t born_by_ns) {
  return PointPlaneRegistrationConfig3D{.maximum_correspondence_m = 1.0,
                                        .maximum_iterations = 20U,
                                        .convergence_translation_m = 1.0e-4,
                                        .convergence_rotation_rad = 1.0e-5,
                                        .robust_width_m = 0.2,
                                        .born_by_ns = born_by_ns};
}

TEST(PointPlaneMap3D, TheRegistrationRecoversTheBodyPoseFromAnOffsetPrior) {
  const std::vector<Eigen::Vector3d> world = roomCorner();
  PointPlaneMap3D map{
      PointPlaneMapConfig3D{.cell_m = 0.4, .maximum_points_per_cell = 6U}};
  map.insert(world, 1U, 0);
  // The body stands at (1, 1, 1), level: its points are the world's less that.
  const Eigen::Vector3d body_position{1.0, 1.0, 1.0};
  std::vector<Eigen::Vector3d> body;
  for (std::size_t index = 0U; index < world.size(); index += 7U) {
    body.push_back(world[index] - body_position);
  }
  const PointPlaneRegistration3D registration = registerPointsToPlanes(
      map, body, Eigen::Vector3d{1.2, 0.85, 1.1}, Eigen::Quaterniond::Identity(),
      registrationConfig(std::numeric_limits<std::int64_t>::max()));
  EXPECT_TRUE(registration.converged);
  EXPECT_GT(registration.matched_fraction, 0.7);
  EXPECT_LT((registration.position - body_position).norm(), 0.03)
      << registration.position.transpose() << " matched "
      << registration.matched_fraction << " iterations " << registration.iterations;
}

TEST(PointPlaneMap3D, CellsBornAfterTheLimitAreNotMatched) {
  const std::vector<Eigen::Vector3d> world = roomCorner();
  PointPlaneMap3D map{
      PointPlaneMapConfig3D{.cell_m = 0.4, .maximum_points_per_cell = 6U}};
  map.insert(world, 1U, 100);
  std::vector<Eigen::Vector3d> body;
  for (std::size_t index = 0U; index < world.size(); index += 7U) {
    body.push_back(world[index]);
  }
  const PointPlaneRegistration3D registration =
      registerPointsToPlanes(map, body, Eigen::Vector3d::Zero(),
                             Eigen::Quaterniond::Identity(), registrationConfig(50));
  EXPECT_EQ(registration.matched_fraction, 0.0);
  EXPECT_FALSE(registration.converged);
}

TEST(PointPlaneMap3D, AnOwnerTakesItsOwnPointsBackOut) {
  PointPlaneMap3D map{
      PointPlaneMapConfig3D{.cell_m = 0.4, .maximum_points_per_cell = 6U}};
  const std::vector<Eigen::Vector3d> first{{0.1, 0.1, 0.1}, {1.1, 0.1, 0.1}};
  const std::vector<Eigen::Vector3d> second{{0.2, 0.1, 0.1}};
  map.insert(first, 1U, 0);
  map.insert(second, 2U, 0);
  EXPECT_EQ(map.pointCount(), 3U);
  map.remove(first, 1U);
  EXPECT_EQ(map.pointCount(), 1U);
  EXPECT_FALSE(map.empty());
  map.remove(second, 2U);
  EXPECT_TRUE(map.empty());
}

} // namespace
} // namespace drone_city_nav
