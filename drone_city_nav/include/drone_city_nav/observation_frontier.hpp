#pragma once

#include "drone_city_nav/observed_occupancy_grid_3d.hpp"
#include "drone_city_nav/swept_footprint.hpp"
#include "drone_city_nav/types.hpp"

#include <array>
#include <compare>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

namespace drone_city_nav {

struct ObservationFrontierId {
  std::uint64_t value{0U};

  [[nodiscard]] auto operator<=>(const ObservationFrontierId&) const noexcept = default;
};

enum class ObservationFrontierStatus : std::uint8_t {
  kAccepted,
  kOutsideMap,
  kFootprintNotObserved,
  kRawCollision,
  kNoUnknownBoundary,
  kInsufficientRaySupport,
  kInsufficientInformationGain,
};

struct SensorObservabilityConfig {
  SweptFootprintConfig footprint{};
  double maximum_observation_range_m{8.0};
  double minimum_known_free_ray_m{1.0};
  double minimum_observation_pose_advance_m{0.5};
  double frontier_identity_resolution_m{8.0};
  double horizontal_min_angle_rad{-3.141592653589793};
  double horizontal_max_angle_rad{3.141592653589793};
  double vertical_min_angle_rad{-1.3962634015954636};
  double vertical_max_angle_rad{1.3962634015954636};
  double directional_cluster_half_angle_rad{0.7853981633974483};
  std::size_t horizontal_samples{240U};
  std::size_t vertical_samples{17U};
  std::size_t minimum_supporting_rays{2U};
  std::size_t minimum_information_gain_voxels{4U};
};

struct SensorObservabilityEvidence {
  ObservationFrontierStatus status{ObservationFrontierStatus::kOutsideMap};
  Vec3 observation_direction{};
  Point3 boundary_centroid{};
  std::size_t tested_rays{0U};
  std::size_t supporting_rays{0U};
  std::size_t information_gain_voxels{0U};
  std::size_t required_information_gain_voxels{0U};
  double minimum_known_free_ray_m{0.0};
  bool footprint_observed_free{false};

  [[nodiscard]] bool accepted() const noexcept {
    return status == ObservationFrontierStatus::kAccepted;
  }
};

struct ObservationFrontier {
  ObservationFrontierId id{};
  Point3 observation_pose{};
  Point3 boundary_centroid{};
  Vec3 observation_direction{};
  std::uint64_t supporting_map_revision{0U};
  std::size_t supporting_rays{0U};
  std::size_t information_gain_voxels{0U};
  std::size_t required_information_gain_voxels{0U};
  double minimum_known_free_ray_m{0.0};
};

struct ObservationFrontierEvaluation {
  ObservationFrontier frontier{};
  SensorObservabilityEvidence evidence{};

  [[nodiscard]] bool accepted() const noexcept {
    return evidence.accepted();
  }
};

struct ObservationFrontierSetEvaluation {
  std::vector<ObservationFrontier> frontiers;
  SensorObservabilityEvidence evidence{};

  [[nodiscard]] bool accepted() const noexcept {
    return !frontiers.empty();
  }
};

struct ObservationFrontierDiscovery {
  std::vector<ObservationFrontier> frontiers;
  std::size_t sampled_free_voxels{0U};
  std::size_t boundary_candidates{0U};
  std::size_t evaluated_candidates{0U};
  std::uint64_t evaluation_sample_fingerprint{0U};
  std::array<std::size_t, 7U> evaluation_status_counts{};
  bool evaluation_budget_exhausted{false};
};

struct ObservationFrontierDiscoveryRegion {
  Point3 center{};
  double maximum_distance_m{0.0};
  // Discovery remains complete within this region, but a bounded evaluation
  // budget should first sample cells that can reveal a route toward the
  // mission goal. Other cells are deterministically rotated as a fallback.
  std::optional<Point3> mission_goal;
};

[[nodiscard]] ObservationFrontierEvaluation
evaluateObservationFrontier(const ObservedOccupancyGrid3D& occupancy,
                            const Point3& observation_pose, std::uint64_t map_revision,
                            const SensorObservabilityConfig& config = {});

[[nodiscard]] ObservationFrontierSetEvaluation
evaluateObservationFrontiers(const ObservedOccupancyGrid3D& occupancy,
                             const Point3& observation_pose, std::uint64_t map_revision,
                             const SensorObservabilityConfig& config = {});

[[nodiscard]] bool observationPoseHasSupportedUnknownBoundary(
    const ObservedOccupancyGrid3D& occupancy, GridIndex3D observation_cell,
    const SensorObservabilityConfig& config) noexcept;

[[nodiscard]] ObservationFrontierDiscovery discoverObservationFrontiers(
    const ObservedOccupancyGrid3D& occupancy, std::uint64_t map_revision,
    const SensorObservabilityConfig& config, std::size_t cell_stride,
    std::size_t maximum_evaluations,
    std::optional<ObservationFrontierDiscoveryRegion> region = std::nullopt);

[[nodiscard]] bool
sensorObservabilityConfigIsValid(const SensorObservabilityConfig& config) noexcept;

[[nodiscard]] const char*
observationFrontierStatusName(ObservationFrontierStatus status) noexcept;

} // namespace drone_city_nav
