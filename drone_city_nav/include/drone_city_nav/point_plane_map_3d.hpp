#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <span>
#include <vector>

// A map of depth points in cells, each cell with the plane through it and its
// neighbours, and the point-to-plane registration of a scan against it: what
// the lidar-inertial estimator registers every scan against, and what the
// visual-inertial estimator registers its depth against where it has been
// before. Every point carries the id of whatever inserted it, so an owner can
// take its own points back out, and every cell the moment its first point
// arrived, so a query can be limited to what was mapped long enough ago.

namespace drone_city_nav {

struct PointPlaneMapConfig3D {
  double cell_m{0.4};
  // The first points of a cell stay; later ones are not kept.
  std::size_t maximum_points_per_cell{6U};
};

class PointPlaneMap3D {
public:
  explicit PointPlaneMap3D(const PointPlaneMapConfig3D& config);
  ~PointPlaneMap3D();
  PointPlaneMap3D(const PointPlaneMap3D&) = delete;
  PointPlaneMap3D& operator=(const PointPlaneMap3D&) = delete;

  [[nodiscard]] bool empty() const noexcept;
  [[nodiscard]] std::size_t pointCount() const noexcept;

  void insert(std::span<const Eigen::Vector3d> points_world, std::uint64_t owner,
              std::int64_t stamp_ns);
  // Removes the points `owner` inserted at these positions.
  void remove(std::span<const Eigen::Vector3d> points_world, std::uint64_t owner);

  // The nearest point to `query` within `maximum_distance_m`, with its cell's
  // plane normal, among the cells whose first point arrived no later than
  // `born_by_ns`; false when there is none or its cell has no plane.
  [[nodiscard]] bool nearest(const Eigen::Vector3d& query, double maximum_distance_m,
                             std::int64_t born_by_ns, Eigen::Vector3d& point,
                             Eigen::Vector3d& normal);

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

struct PointPlaneRegistrationConfig3D {
  double maximum_correspondence_m{1.0};
  std::size_t maximum_iterations{10U};
  double convergence_translation_m{1.0e-3};
  double convergence_rotation_rad{1.0e-4};
  // Huber width of the point-to-plane residual.
  double robust_width_m{0.2};
  // Only cells born by this moment are matched.
  std::int64_t born_by_ns{std::numeric_limits<std::int64_t>::max()};
};

struct PointPlaneRegistration3D {
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond rotation{Eigen::Quaterniond::Identity()};
  // Information of the left perturbation [rotation; translation].
  Eigen::Matrix<double, 6, 6> information{Eigen::Matrix<double, 6, 6>::Zero()};
  double matched_fraction{0.0};
  double residual_rms_m{0.0};
  double information_per_point{0.0};
  std::size_t iterations{0U};
  bool converged{false};
};

// Point-to-plane registration of body-frame points against the map from the
// prior pose: Gauss-Newton over the left perturbation [rotation;
// translation] of the world pose with a Huber weight on each residual.
[[nodiscard]] PointPlaneRegistration3D registerPointsToPlanes(
    PointPlaneMap3D& map, std::span<const Eigen::Vector3d> points_body,
    const Eigen::Vector3d& prior_position, const Eigen::Quaterniond& prior_rotation,
    const PointPlaneRegistrationConfig3D& config);

} // namespace drone_city_nav
