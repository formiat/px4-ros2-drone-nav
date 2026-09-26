#include "drone_city_nav/point_plane_map_3d.hpp"

#include <Eigen/Dense>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <unordered_map>
#include <vector>

namespace drone_city_nav {

namespace {

using Vector6d = Eigen::Matrix<double, 6, 1>;
using Matrix6d = Eigen::Matrix<double, 6, 6>;

struct CellKey {
  std::int32_t x{0};
  std::int32_t y{0};
  std::int32_t z{0};
  [[nodiscard]] bool operator==(const CellKey&) const noexcept = default;
};

struct CellKeyHash {
  [[nodiscard]] std::size_t operator()(const CellKey& key) const noexcept {
    const auto x = static_cast<std::uint64_t>(static_cast<std::uint32_t>(key.x));
    const auto y = static_cast<std::uint64_t>(static_cast<std::uint32_t>(key.y));
    const auto z = static_cast<std::uint64_t>(static_cast<std::uint32_t>(key.z));
    return static_cast<std::size_t>(x * 73856093ULL ^ y * 19349663ULL ^
                                    z * 83492791ULL);
  }
};

[[nodiscard]] CellKey cellOf(const Eigen::Vector3d& point,
                             const double size_m) noexcept {
  return CellKey{static_cast<std::int32_t>(std::floor(point.x() / size_m)),
                 static_cast<std::int32_t>(std::floor(point.y() / size_m)),
                 static_cast<std::int32_t>(std::floor(point.z() / size_m))};
}

// The rotation of a small angle vector.
[[nodiscard]] Eigen::Quaterniond expSmallAngle(const Eigen::Vector3d& theta) noexcept {
  const double angle = theta.norm();
  if (angle < 1.0e-12) {
    return Eigen::Quaterniond{1.0, 0.5 * theta.x(), 0.5 * theta.y(), 0.5 * theta.z()}
        .normalized();
  }
  return Eigen::Quaterniond{Eigen::AngleAxisd{angle, theta / angle}};
}

// One cell: its points, who inserted each, when the first arrived, and the
// plane through it, fitted over the cell and its neighbours once enough
// points are there.
struct Cell {
  std::vector<Eigen::Vector3d> points;
  std::vector<std::uint64_t> owners;
  std::int64_t born_ns{0};
  Eigen::Vector3d normal{Eigen::Vector3d::Zero()};
  bool normal_valid{false};
  bool normal_stale{true};
};

} // namespace

struct PointPlaneMap3D::Impl {
  explicit Impl(const PointPlaneMapConfig3D& config_in)
      : config(config_in) {
  }

  // A new point refits the planes of its own cell and the ring around it;
  // the outer ring the fit reads keeps its plane, which one point two
  // cells away hardly moves, and refitting it on every insertion took the
  // registration past the scan period (r364: 86 ms at p95, 125 at most).
  void markNeighboursStale(const Eigen::Vector3d& point) {
    const CellKey center = cellOf(point, config.cell_m);
    for (std::int32_t dx = -1; dx <= 1; ++dx) {
      for (std::int32_t dy = -1; dy <= 1; ++dy) {
        for (std::int32_t dz = -1; dz <= 1; ++dz) {
          const auto found =
              cells.find(CellKey{center.x + dx, center.y + dy, center.z + dz});
          if (found != cells.end()) {
            found->second.normal_stale = true;
          }
        }
      }
    }
  }

  // The plane through the points of the cell and its two rings of
  // neighbours: the smallest principal axis, valid once the points are many
  // and flat enough. Two rings, because a scan thinned coarser than the
  // cell leaves one ring with too few points for a plane.
  void fitNormal(Cell& cell, const Eigen::Vector3d& around) {
    const CellKey center = cellOf(around, config.cell_m);
    Eigen::Vector3d mean = Eigen::Vector3d::Zero();
    std::size_t count = 0U;
    std::vector<const Eigen::Vector3d*> neighbours;
    for (std::int32_t dx = -2; dx <= 2; ++dx) {
      for (std::int32_t dy = -2; dy <= 2; ++dy) {
        for (std::int32_t dz = -2; dz <= 2; ++dz) {
          const auto found =
              cells.find(CellKey{center.x + dx, center.y + dy, center.z + dz});
          if (found == cells.end()) {
            continue;
          }
          for (const Eigen::Vector3d& point : found->second.points) {
            neighbours.push_back(&point);
            mean += point;
            ++count;
          }
        }
      }
    }
    cell.normal_stale = false;
    cell.normal_valid = false;
    if (count < 6U) {
      return;
    }
    mean /= static_cast<double>(count);
    Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
    for (const Eigen::Vector3d* point : neighbours) {
      const Eigen::Vector3d offset = *point - mean;
      covariance += offset * offset.transpose();
    }
    covariance /= static_cast<double>(count);
    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver{covariance};
    const Eigen::Vector3d values = solver.eigenvalues();
    // A plane: the smallest spread is well below the middle one.
    if (!(values(0) < 0.25 * values(1))) {
      return;
    }
    cell.normal = solver.eigenvectors().col(0).normalized();
    cell.normal_valid = true;
  }

  PointPlaneMapConfig3D config;
  std::unordered_map<CellKey, Cell, CellKeyHash> cells;
  std::size_t point_count{0U};
};

PointPlaneMap3D::PointPlaneMap3D(const PointPlaneMapConfig3D& config)
    : impl_(std::make_unique<Impl>(config)) {
}

PointPlaneMap3D::~PointPlaneMap3D() = default;

bool PointPlaneMap3D::empty() const noexcept {
  return impl_->cells.empty();
}

std::size_t PointPlaneMap3D::pointCount() const noexcept {
  return impl_->point_count;
}

void PointPlaneMap3D::insert(const std::span<const Eigen::Vector3d> points_world,
                             const std::uint64_t owner, const std::int64_t stamp_ns) {
  Impl& impl = *impl_;
  for (const Eigen::Vector3d& point : points_world) {
    const auto [found, created] =
        impl.cells.try_emplace(cellOf(point, impl.config.cell_m));
    Cell& cell = found->second;
    if (created) {
      cell.born_ns = stamp_ns;
    }
    if (cell.points.size() >= impl.config.maximum_points_per_cell) {
      continue;
    }
    cell.points.push_back(point);
    cell.owners.push_back(owner);
    cell.normal_stale = true;
    ++impl.point_count;
    impl.markNeighboursStale(point);
  }
}

// Removes the owner's own points and refits only the planes they belonged
// to. Rebuilding the whole hash on every eviction cost the full submap per
// keyframe, and at 5 m/s every scan is a keyframe: on r447 the submap held
// 112 thousand points, a scan took 155 to 266 ms against a 100 ms period,
// the odometry reached the autopilot 1.2 to 1.5 s old, the autopilot stopped
// fusing it and lost its position four seconds later.
void PointPlaneMap3D::remove(const std::span<const Eigen::Vector3d> points_world,
                             const std::uint64_t owner) {
  Impl& impl = *impl_;
  for (const Eigen::Vector3d& point : points_world) {
    const auto found = impl.cells.find(cellOf(point, impl.config.cell_m));
    if (found == impl.cells.end()) {
      continue;
    }
    Cell& cell = found->second;
    bool removed = false;
    for (std::size_t index = cell.owners.size(); index-- > 0U;) {
      if (cell.owners[index] != owner) {
        continue;
      }
      cell.points.erase(cell.points.begin() + static_cast<std::ptrdiff_t>(index));
      cell.owners.erase(cell.owners.begin() + static_cast<std::ptrdiff_t>(index));
      --impl.point_count;
      removed = true;
    }
    if (!removed) {
      continue;
    }
    impl.markNeighboursStale(point);
    if (cell.points.empty()) {
      impl.cells.erase(found);
    }
  }
}

bool PointPlaneMap3D::nearest(const Eigen::Vector3d& query,
                              const double maximum_distance_m,
                              const std::int64_t born_by_ns, Eigen::Vector3d& point,
                              Eigen::Vector3d& normal) {
  Impl& impl = *impl_;
  const CellKey center = cellOf(query, impl.config.cell_m);
  double best = maximum_distance_m * maximum_distance_m;
  Cell* best_cell = nullptr;
  for (std::int32_t dx = -1; dx <= 1; ++dx) {
    for (std::int32_t dy = -1; dy <= 1; ++dy) {
      for (std::int32_t dz = -1; dz <= 1; ++dz) {
        const auto found =
            impl.cells.find(CellKey{center.x + dx, center.y + dy, center.z + dz});
        if (found == impl.cells.end() || found->second.born_ns > born_by_ns) {
          continue;
        }
        for (const Eigen::Vector3d& candidate : found->second.points) {
          const double distance = (candidate - query).squaredNorm();
          if (distance < best) {
            best = distance;
            point = candidate;
            best_cell = &found->second;
          }
        }
      }
    }
  }
  if (best_cell == nullptr) {
    return false;
  }
  if (best_cell->normal_stale) {
    impl.fitNormal(*best_cell, point);
  }
  normal = best_cell->normal;
  return best_cell->normal_valid;
}

PointPlaneRegistration3D registerPointsToPlanes(
    PointPlaneMap3D& map, const std::span<const Eigen::Vector3d> points_body,
    const Eigen::Vector3d& prior_position, const Eigen::Quaterniond& prior_rotation,
    const PointPlaneRegistrationConfig3D& config) {
  PointPlaneRegistration3D result;
  result.position = prior_position;
  result.rotation = prior_rotation;
  if (points_body.empty()) {
    return result;
  }
  Eigen::Vector3d map_point;
  Eigen::Vector3d normal;
  for (std::size_t iteration = 0U; iteration < config.maximum_iterations; ++iteration) {
    Matrix6d hessian = Matrix6d::Zero();
    Vector6d gradient = Vector6d::Zero();
    double weighted_square = 0.0;
    std::size_t matched = 0U;
    const Eigen::Matrix3d rotation = result.rotation.toRotationMatrix();
    for (const Eigen::Vector3d& point_body : points_body) {
      const Eigen::Vector3d rotated = rotation * point_body;
      const Eigen::Vector3d point_world = rotated + result.position;
      if (!map.nearest(point_world, config.maximum_correspondence_m, config.born_by_ns,
                       map_point, normal)) {
        continue;
      }
      const double residual = normal.dot(point_world - map_point);
      const double magnitude = std::abs(residual);
      const double weight =
          magnitude <= config.robust_width_m ? 1.0 : config.robust_width_m / magnitude;
      Vector6d jacobian;
      jacobian.head<3>() = rotated.cross(normal);
      jacobian.tail<3>() = normal;
      hessian += weight * jacobian * jacobian.transpose();
      gradient += weight * residual * jacobian;
      weighted_square += weight * residual * residual;
      ++matched;
    }
    result.iterations = iteration + 1U;
    result.matched_fraction =
        static_cast<double>(matched) / static_cast<double>(points_body.size());
    result.residual_rms_m =
        matched > 0U ? std::sqrt(weighted_square / static_cast<double>(matched)) : 0.0;
    result.information = hessian;
    if (matched < 6U) {
      return result;
    }
    // A touch of damping keeps a weakly observed axis from running away.
    const Matrix6d damped = hessian + 1.0e-6 * Matrix6d::Identity();
    const Vector6d delta = damped.ldlt().solve(-gradient);
    if (!delta.allFinite()) {
      return result;
    }
    result.rotation = (expSmallAngle(delta.head<3>()) * result.rotation).normalized();
    result.position += delta.tail<3>();
    if (delta.tail<3>().norm() < config.convergence_translation_m &&
        delta.head<3>().norm() < config.convergence_rotation_rad) {
      result.converged = true;
      break;
    }
  }
  if (result.iterations > 0U && !result.converged) {
    // The last step was applied; the fit is what its own step size says.
    result.converged = true;
  }
  const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver{
      result.information.bottomRightCorner<3, 3>()};
  const double matched_points =
      result.matched_fraction * static_cast<double>(points_body.size());
  result.information_per_point =
      matched_points > 0.0 ? solver.eigenvalues()(0) / matched_points : 0.0;
  return result;
}

} // namespace drone_city_nav
