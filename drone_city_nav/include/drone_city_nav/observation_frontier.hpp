#pragma once

#include "drone_city_nav/observed_occupancy_grid_3d.hpp"
#include "drone_city_nav/swept_footprint.hpp"
#include "drone_city_nav/types.hpp"

#include <compare>
#include <cstddef>
#include <cstdint>

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
};

struct SensorObservabilityConfig {
  SweptFootprintConfig footprint{};
  double maximum_observation_range_m{8.0};
  double minimum_known_free_ray_m{1.0};
  std::size_t minimum_supporting_rays{2U};
  std::size_t minimum_information_gain_voxels{4U};
};

struct SensorObservabilityEvidence {
  ObservationFrontierStatus status{ObservationFrontierStatus::kOutsideMap};
  Vec3 observation_direction{};
  std::size_t tested_rays{0U};
  std::size_t supporting_rays{0U};
  std::size_t information_gain_voxels{0U};
  double minimum_known_free_ray_m{0.0};
  bool footprint_observed_free{false};

  [[nodiscard]] bool accepted() const noexcept {
    return status == ObservationFrontierStatus::kAccepted;
  }
};

struct ObservationFrontier {
  ObservationFrontierId id{};
  Point3 observation_pose{};
  Vec3 observation_direction{};
  std::uint64_t supporting_map_revision{0U};
  std::size_t supporting_rays{0U};
  std::size_t information_gain_voxels{0U};
  double minimum_known_free_ray_m{0.0};
};

struct ObservationFrontierEvaluation {
  ObservationFrontier frontier{};
  SensorObservabilityEvidence evidence{};

  [[nodiscard]] bool accepted() const noexcept {
    return evidence.accepted();
  }
};

[[nodiscard]] ObservationFrontierEvaluation
evaluateObservationFrontier(const ObservedOccupancyGrid3D& occupancy,
                            const Point3& observation_pose, std::uint64_t map_revision,
                            const SensorObservabilityConfig& config = {});

[[nodiscard]] const char*
observationFrontierStatusName(ObservationFrontierStatus status) noexcept;

} // namespace drone_city_nav
