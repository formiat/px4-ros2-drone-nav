#include "drone_city_nav/pose_graph_4dof.hpp"

#include <Eigen/Geometry>
#include <Eigen/SparseCholesky>
#include <Eigen/SparseCore>
#include <algorithm>
#include <array>
#include <cmath>
#include <numbers>
#include <utility>
#include <vector>

namespace drone_city_nav {

namespace {

[[nodiscard]] double wrapped(const double angle_rad) {
  return std::remainder(angle_rad, 2.0 * std::numbers::pi);
}

[[nodiscard]] Eigen::Matrix3d headingFrame(const double yaw_rad) {
  return Eigen::AngleAxisd{yaw_rad, Eigen::Vector3d::UnitZ()}.toRotationMatrix();
}

} // namespace

std::size_t PoseGraph4Dof::addNode(const PoseGraphPose& pose) {
  nodes_.push_back(pose);
  return nodes_.size() - 1U;
}

void PoseGraph4Dof::addEdge(const PoseGraphEdge& edge) {
  edges_.push_back(edge);
}

PoseGraphPose PoseGraph4Dof::residual(const PoseGraphEdge& edge) const {
  const PoseGraphPose& from = nodes_.at(edge.from);
  const PoseGraphPose& to = nodes_.at(edge.to);
  return PoseGraphPose{headingFrame(from.yaw_rad).transpose() *
                               (to.position - from.position) -
                           edge.shift,
                       wrapped(to.yaw_rad - from.yaw_rad - edge.turn_rad)};
}

int PoseGraph4Dof::optimise(const int maximum_iterations) {
  constexpr double kPositionStepM{1.0e-3};
  constexpr double kYawStepRad{1.7e-5};
  if (nodes_.size() < 2U || edges_.empty()) {
    return 0;
  }
  // The anchor is not a variable: node n's four live at 4 (n - 1).
  const auto unknowns = static_cast<Eigen::Index>(4U * (nodes_.size() - 1U));
  const auto column = [](const std::size_t node) {
    return static_cast<Eigen::Index>(4U * (node - 1U));
  };
  int iterations = 0;
  while (iterations < maximum_iterations) {
    ++iterations;
    std::vector<Eigen::Triplet<double>> entries;
    entries.reserve(edges_.size() * 64U);
    Eigen::VectorXd gradient = Eigen::VectorXd::Zero(unknowns);
    for (const PoseGraphEdge& edge : edges_) {
      const PoseGraphPose& from = nodes_[edge.from];
      const PoseGraphPose& to = nodes_[edge.to];
      const Eigen::Matrix3d to_heading = headingFrame(from.yaw_rad).transpose();
      const Eigen::Vector3d seen = to_heading * (to.position - from.position);
      Eigen::Vector4d error;
      error << seen - edge.shift, wrapped(to.yaw_rad - from.yaw_rad - edge.turn_rad);
      // The residual by the two nodes' (x, y, z, yaw).
      Eigen::Matrix4d by_from = Eigen::Matrix4d::Zero();
      Eigen::Matrix4d by_to = Eigen::Matrix4d::Zero();
      by_from.topLeftCorner<3, 3>() = -to_heading;
      // d(R^T v)/d(yaw) = (y, -x, 0) of the turned vector.
      by_from.topRightCorner<3, 1>() = Eigen::Vector3d{seen.y(), -seen.x(), 0.0};
      by_from(3, 3) = -1.0;
      by_to.topLeftCorner<3, 3>() = to_heading;
      by_to(3, 3) = 1.0;
      Eigen::Vector4d weight;
      weight << Eigen::Vector3d::Constant(1.0 /
                                          (edge.shift_sigma_m * edge.shift_sigma_m)),
          1.0 / (edge.turn_sigma_rad * edge.turn_sigma_rad);
      const std::array<std::pair<std::size_t, const Eigen::Matrix4d*>, 2> sides{
          {{edge.from, &by_from}, {edge.to, &by_to}}};
      for (const auto& [row_node, row_jacobian] : sides) {
        if (row_node == 0U) {
          continue;
        }
        const Eigen::Matrix4d weighted =
            row_jacobian->transpose() * weight.asDiagonal();
        gradient.segment<4>(column(row_node)) += weighted * error;
        for (const auto& [column_node, column_jacobian] : sides) {
          if (column_node == 0U) {
            continue;
          }
          const Eigen::Matrix4d block = weighted * *column_jacobian;
          for (int row = 0; row < 4; ++row) {
            for (int col = 0; col < 4; ++col) {
              entries.emplace_back(column(row_node) + row, column(column_node) + col,
                                   block(row, col));
            }
          }
        }
      }
    }
    Eigen::SparseMatrix<double> normal(unknowns, unknowns);
    normal.setFromTriplets(entries.begin(), entries.end());
    const Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> solver{normal};
    if (solver.info() != Eigen::Success) {
      // A node no edge reaches: nothing to solve for it, and nothing is moved.
      return iterations;
    }
    const Eigen::VectorXd step = solver.solve(-gradient);
    double largest_shift = 0.0;
    double largest_turn = 0.0;
    for (std::size_t node = 1U; node < nodes_.size(); ++node) {
      const Eigen::Vector4d change = step.segment<4>(column(node));
      nodes_[node].position += change.head<3>();
      nodes_[node].yaw_rad = wrapped(nodes_[node].yaw_rad + change(3));
      largest_shift = std::max(largest_shift, change.head<3>().norm());
      largest_turn = std::max(largest_turn, std::abs(change(3)));
    }
    if (largest_shift < kPositionStepM && largest_turn < kYawStepRad) {
      break;
    }
  }
  return iterations;
}

} // namespace drone_city_nav
