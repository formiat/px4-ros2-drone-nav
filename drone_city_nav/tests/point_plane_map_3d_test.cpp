#include "drone_city_nav/point_plane_map_3d.hpp"

#include <gtest/gtest.h>

#include <cmath>
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

// The tilt prior: the room's corner registered from a level prior with an
// outside level that says the body is pitched one degree. Without a share the
// registration stays level (the map's own level); at a share of one the pose
// pitches part of the way (more than half: the points' hold on the pitch is
// less than their diagonal information once the translation they share it
// with is free); at ten the prior nearly has its way. The heading is never
// the prior's.
TEST(PointPlaneMap3D, TheTiltPriorPricesRollAndPitchByItsShareAndLeavesTheHeading) {
  const std::vector<Eigen::Vector3d> world = roomCorner();
  PointPlaneMap3D map{
      PointPlaneMapConfig3D{.cell_m = 0.4, .maximum_points_per_cell = 6U}};
  map.insert(world, 1U, 0);
  const Eigen::Vector3d body_position{1.0, 1.0, 1.0};
  std::vector<Eigen::Vector3d> body;
  for (std::size_t index = 0U; index < world.size(); index += 7U) {
    body.push_back(world[index] - body_position);
  }
  const double one_degree = M_PI / 180.0;
  const Eigen::Quaterniond level_says{
      Eigen::AngleAxisd{one_degree, Eigen::Vector3d::UnitY()} *
      Eigen::AngleAxisd{5.0 * one_degree, Eigen::Vector3d::UnitZ()}};
  const auto pitch_of = [](const Eigen::Quaterniond& q) {
    return std::asin(2.0 * (q.w() * q.y() - q.z() * q.x()));
  };
  const auto yaw_of = [](const Eigen::Quaterniond& q) {
    return std::atan2(2.0 * (q.w() * q.z() + q.x() * q.y()),
                      1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
  };
  double pitches[3];
  const double shares[3] = {0.0, 1.0, 10.0};
  for (int index = 0; index < 3; ++index) {
    PointPlaneRegistrationConfig3D config =
        registrationConfig(std::numeric_limits<std::int64_t>::max());
    config.tilt_prior = true;
    config.tilt_prior_rotation = level_says;
    config.tilt_prior_share = shares[index];
    const PointPlaneRegistration3D registration = registerPointsToPlanes(
        map, body, body_position, Eigen::Quaterniond::Identity(), config);
    EXPECT_TRUE(registration.converged) << shares[index];
    pitches[index] = pitch_of(registration.rotation);
    EXPECT_LT(std::abs(yaw_of(registration.rotation)), 0.2 * one_degree)
        << shares[index];
  }
  EXPECT_LT(std::abs(pitches[0]), 0.05 * one_degree);
  EXPECT_GT(pitches[1], 0.25 * one_degree);
  EXPECT_LT(pitches[1], 0.95 * one_degree);
  EXPECT_GT(pitches[2], 0.9 * one_degree);
  EXPECT_LT(pitches[2], 1.05 * one_degree);
}

// The height prior: the room's corner registered from the true pose with an
// outside height half a metre above it, the level held by the tilt prior as
// in flight (without it the solve tilts the body to meet the height); at a
// share of nought the pose stays, at one it rises part of the way, at ten
// nearly the whole way, and x and y stay where the points put them.
TEST(PointPlaneMap3D, TheHeightPriorPricesZByItsShareAndLeavesTheRest) {
  const std::vector<Eigen::Vector3d> world = roomCorner();
  PointPlaneMap3D map{
      PointPlaneMapConfig3D{.cell_m = 0.4, .maximum_points_per_cell = 6U}};
  map.insert(world, 1U, 0);
  const Eigen::Vector3d body_position{1.0, 1.0, 1.0};
  std::vector<Eigen::Vector3d> body;
  for (std::size_t index = 0U; index < world.size(); index += 7U) {
    body.push_back(world[index] - body_position);
  }
  double rises[3];
  const double shares[3] = {0.0, 1.0, 10.0};
  for (int index = 0; index < 3; ++index) {
    PointPlaneRegistrationConfig3D config =
        registrationConfig(std::numeric_limits<std::int64_t>::max());
    config.height_prior = true;
    config.height_prior_z_m = body_position.z() + 0.5;
    config.height_prior_share = shares[index];
    config.tilt_prior = true;
    config.tilt_prior_rotation = Eigen::Quaterniond::Identity();
    config.tilt_prior_share = 10.0;
    const PointPlaneRegistration3D registration = registerPointsToPlanes(
        map, body, body_position, Eigen::Quaterniond::Identity(), config);
    EXPECT_TRUE(registration.converged) << shares[index];
    rises[index] = registration.position.z() - body_position.z();
    EXPECT_LT((registration.position.head<2>() - body_position.head<2>()).norm(), 0.05)
        << shares[index];
  }
  EXPECT_LT(std::abs(rises[0]), 0.02);
  EXPECT_GT(rises[1], 0.1);
  EXPECT_LT(rises[1], 0.48);
  EXPECT_GT(rises[2], 0.4);
}

// One wall at x = 0, seen from a prior 0.8 m (two cells) off along x: the
// one ring of cells the search covers by default finds no point of it and
// the registration matches nothing; two rings reach the wall and the offset
// goes (the other axes stay at the prior, which is the truth there).
TEST(PointPlaneMap3D, TheSearchReachesAsManyRingsOfCellsAsItIsGiven) {
  std::vector<Eigen::Vector3d> wall;
  for (int i = 0; i < 40; ++i) {
    for (int j = 0; j < 40; ++j) {
      wall.emplace_back(0.0, 0.05 + 0.1 * static_cast<double>(i),
                        0.05 + 0.1 * static_cast<double>(j));
    }
  }
  PointPlaneMap3D map{
      PointPlaneMapConfig3D{.cell_m = 0.4, .maximum_points_per_cell = 6U}};
  map.insert(wall, 1U, 0);
  const Eigen::Vector3d body_position{1.0, 1.0, 1.0};
  std::vector<Eigen::Vector3d> body;
  for (std::size_t index = 0U; index < wall.size(); index += 7U) {
    body.push_back(wall[index] - body_position);
  }
  const Eigen::Vector3d prior = body_position + Eigen::Vector3d{0.8, 0.0, 0.0};
  PointPlaneRegistrationConfig3D config =
      registrationConfig(std::numeric_limits<std::int64_t>::max());
  config.maximum_correspondence_m = 3.0;
  const PointPlaneRegistration3D one_ring =
      registerPointsToPlanes(map, body, prior, Eigen::Quaterniond::Identity(), config);
  EXPECT_EQ(one_ring.matched_fraction, 0.0);
  EXPECT_FALSE(one_ring.converged);
  config.search_ring_cells = 2;
  const PointPlaneRegistration3D two_rings =
      registerPointsToPlanes(map, body, prior, Eigen::Quaterniond::Identity(), config);
  EXPECT_TRUE(two_rings.converged);
  EXPECT_GT(two_rings.matched_fraction, 0.9);
  EXPECT_NEAR(two_rings.position.x(), body_position.x(), 0.03)
      << two_rings.position.transpose() << " matched " << two_rings.matched_fraction;
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
