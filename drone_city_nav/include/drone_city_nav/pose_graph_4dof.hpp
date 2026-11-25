#pragma once
// A pose graph over the flight's keyframes (roadmap item 41 stage 2): each
// node is where a keyframe was and which way it faced about the vertical; an
// edge says how one keyframe lay from another, by the odometry between
// neighbours or by a loop closure between a keyframe and one it saw again.
// Four degrees of freedom: roll and pitch are observed by gravity in the
// filter and are not the graph's to move. The first node is the anchor and
// stays where it was put. Solved by Gauss-Newton on a sparse Cholesky
// factorisation; a flight holds a few thousand keyframes at most.

#include <Eigen/Core>
#include <cstddef>
#include <vector>

namespace drone_city_nav {

struct PoseGraphPose {
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  double yaw_rad{0.0};
};

// The node `to` as seen from the node `from`: the shift in `from`'s heading
// frame and the turn, each with the standard deviation the edge is known to.
struct PoseGraphEdge {
  std::size_t from{0U};
  std::size_t to{0U};
  Eigen::Vector3d shift{Eigen::Vector3d::Zero()};
  double turn_rad{0.0};
  double shift_sigma_m{1.0};
  double turn_sigma_rad{1.0};
};

class PoseGraph4Dof {
public:
  // A node at its first estimate; its index.
  std::size_t addNode(const PoseGraphPose& pose);
  void addEdge(const PoseGraphEdge& edge);

  // What the edge would read at the nodes' present poses, less what it says:
  // the shift's difference in `from`'s heading frame and the turn's.
  [[nodiscard]] PoseGraphPose residual(const PoseGraphEdge& edge) const;

  // Gauss-Newton until the largest step is under a millimetre and a
  // thousandth of a degree, `maximum_iterations` at most; the iterations run.
  int optimise(int maximum_iterations);

  [[nodiscard]] const PoseGraphPose& node(const std::size_t index) const {
    return nodes_.at(index);
  }

  [[nodiscard]] std::size_t nodeCount() const noexcept {
    return nodes_.size();
  }

  [[nodiscard]] std::size_t edgeCount() const noexcept {
    return edges_.size();
  }

private:
  std::vector<PoseGraphPose> nodes_;
  std::vector<PoseGraphEdge> edges_;
};

} // namespace drone_city_nav
