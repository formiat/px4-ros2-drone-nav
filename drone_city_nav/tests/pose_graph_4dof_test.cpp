#include "drone_city_nav/pose_graph_4dof.hpp"

#include <gtest/gtest.h>

#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <numbers>
#include <random>
#include <vector>

namespace drone_city_nav {
namespace {

constexpr double kDegree{std::numbers::pi / 180.0};

[[nodiscard]] Eigen::Matrix3d aboutVertical(const double yaw_rad) {
  return Eigen::AngleAxisd{yaw_rad, Eigen::Vector3d::UnitZ()}.toRotationMatrix();
}

// The node `to` from the node `from`, as an edge reads it.
[[nodiscard]] PoseGraphEdge between(const std::size_t from_index,
                                    const PoseGraphPose& from,
                                    const std::size_t to_index, const PoseGraphPose& to,
                                    const double shift_sigma_m,
                                    const double turn_sigma_rad) {
  return PoseGraphEdge{
      .from = from_index,
      .to = to_index,
      .shift = aboutVertical(-from.yaw_rad) * (to.position - from.position),
      .turn_rad = std::remainder(to.yaw_rad - from.yaw_rad, 2.0 * std::numbers::pi),
      .shift_sigma_m = shift_sigma_m,
      .turn_sigma_rad = turn_sigma_rad};
}

// A square flown once round, a keyframe every metre, back to its start: the
// true poses.
[[nodiscard]] std::vector<PoseGraphPose> square(const int side_m) {
  std::vector<PoseGraphPose> poses;
  PoseGraphPose pose;
  for (int leg = 0; leg < 4; ++leg) {
    for (int step = 0; step < side_m; ++step) {
      poses.push_back(pose);
      pose.position += aboutVertical(pose.yaw_rad) * Eigen::Vector3d{1.0, 0.0, 0.01};
    }
    pose.yaw_rad += 90.0 * kDegree;
  }
  pose.yaw_rad = 0.0;
  poses.push_back(pose);
  return poses;
}

// The graph of a route as an odometry that drifts lays it: each step read
// with noise, a scale error and a heading bias, the nodes put where the
// readings add up to.
struct DriftedRoute {
  PoseGraph4Dof graph;
  std::vector<PoseGraphPose> laid;
};

[[nodiscard]] DriftedRoute drifted(const std::vector<PoseGraphPose>& truth) {
  std::mt19937 generator{7U};
  std::normal_distribution<double> shift_noise{0.0, 0.01};
  std::normal_distribution<double> turn_noise{0.0, 0.05 * kDegree};
  DriftedRoute route;
  PoseGraphPose pose = truth.front();
  route.graph.addNode(pose);
  route.laid.push_back(pose);
  for (std::size_t i = 1U; i < truth.size(); ++i) {
    PoseGraphEdge edge =
        between(i - 1U, truth[i - 1U], i, truth[i], 0.02, 0.1 * kDegree);
    edge.shift = 1.01 * edge.shift + Eigen::Vector3d{shift_noise(generator),
                                                     shift_noise(generator),
                                                     shift_noise(generator)};
    edge.turn_rad += 0.02 * kDegree + turn_noise(generator);
    pose.position += aboutVertical(pose.yaw_rad) * edge.shift;
    pose.yaw_rad += edge.turn_rad;
    route.graph.addNode(pose);
    route.graph.addEdge(edge);
    route.laid.push_back(pose);
  }
  return route;
}

TEST(PoseGraph4DofTest, AnEdgeReadsNothingWhereTheNodesAgreeWithIt) {
  PoseGraph4Dof graph;
  const PoseGraphPose first{{1.0, 2.0, -3.0}, 0.4};
  const PoseGraphPose second{{4.0, -1.0, -2.5}, -2.9};
  graph.addNode(first);
  graph.addNode(second);
  const PoseGraphEdge edge = between(0U, first, 1U, second, 0.1, 0.01);
  EXPECT_LT(graph.residual(edge).position.norm(), 1.0e-12);
  EXPECT_NEAR(graph.residual(edge).yaw_rad, 0.0, 1.0e-12);
  PoseGraphEdge shifted = edge;
  shifted.shift.x() += 0.5;
  shifted.turn_rad -= 0.2;
  EXPECT_NEAR(graph.residual(shifted).position.x(), -0.5, 1.0e-12);
  EXPECT_NEAR(graph.residual(shifted).yaw_rad, 0.2, 1.0e-12);
}

TEST(PoseGraph4DofTest, ALoopOfNoisyOdometryIsClosedByItsClosure) {
  const std::vector<PoseGraphPose> truth = square(40);
  DriftedRoute route = drifted(truth);
  const std::size_t last = truth.size() - 1U;
  // The odometry alone comes home well off: the loop is worth closing.
  const double drift_m = (route.laid[last].position - truth[last].position).norm();
  ASSERT_GT(drift_m, 1.0);
  // The closure: the last keyframe saw the first one again, where it truly is.
  route.graph.addEdge(between(0U, truth[0U], last, truth[last], 0.1, 0.5 * kDegree));
  const int iterations = route.graph.optimise(10);
  EXPECT_LT(iterations, 10);
  EXPECT_LT((route.graph.node(last).position - truth[last].position).norm(), 0.15);
  EXPECT_NEAR(route.graph.node(last).yaw_rad, truth[last].yaw_rad, 0.5 * kDegree);
  // The anchor stands, and the correction is spread over the loop: no node
  // lies as far from the truth as the odometry had left the last one, and
  // the route as a whole lies nearer (its scale error no closure removes).
  EXPECT_TRUE(route.graph.node(0U).position.isApprox(truth[0U].position));
  double worst_m = 0.0;
  double mean_before_m = 0.0;
  double mean_after_m = 0.0;
  for (std::size_t i = 0U; i < truth.size(); ++i) {
    const double after_m = (route.graph.node(i).position - truth[i].position).norm();
    worst_m = std::max(worst_m, after_m);
    mean_after_m += after_m / static_cast<double>(truth.size());
    mean_before_m += (route.laid[i].position - truth[i].position).norm() /
                     static_cast<double>(truth.size());
  }
  EXPECT_LT(worst_m, drift_m);
  EXPECT_LT(mean_after_m, 0.7 * mean_before_m);
}

TEST(PoseGraph4DofTest, AClosureMovesNothingBeforeTheKeyframeItReturnsTo) {
  const std::vector<PoseGraphPose> truth = square(40);
  DriftedRoute route = drifted(truth);
  const std::size_t returned_to = 30U;
  const std::size_t last = truth.size() - 1U;
  // The last keyframe sees the thirtieth again, as that one was laid: what
  // was laid there stays, and the route after it is brought to agree.
  PoseGraphEdge closure =
      between(returned_to, truth[returned_to], last, truth[last], 0.1, 0.5 * kDegree);
  route.graph.addEdge(closure);
  route.graph.optimise(10);
  for (std::size_t i = 0U; i <= returned_to; ++i) {
    EXPECT_LT((route.graph.node(i).position - route.laid[i].position).norm(), 1.0e-6);
    EXPECT_NEAR(route.graph.node(i).yaw_rad, route.laid[i].yaw_rad, 1.0e-8);
  }
  EXPECT_LT(route.graph.residual(closure).position.norm(), 0.1);
  EXPECT_GT((route.graph.node(last).position - route.laid[last].position).norm(), 0.5);
}

TEST(PoseGraph4DofTest, AGraphWithoutAnEdgeIsLeftAsItIs) {
  PoseGraph4Dof graph;
  EXPECT_EQ(graph.optimise(5), 0);
  graph.addNode(PoseGraphPose{{1.0, 0.0, 0.0}, 0.1});
  graph.addNode(PoseGraphPose{{2.0, 0.0, 0.0}, 0.2});
  EXPECT_EQ(graph.optimise(5), 0);
  EXPECT_EQ(graph.node(1U).position.x(), 2.0);
  EXPECT_EQ(graph.nodeCount(), 2U);
  EXPECT_EQ(graph.edgeCount(), 0U);
}

} // namespace
} // namespace drone_city_nav
