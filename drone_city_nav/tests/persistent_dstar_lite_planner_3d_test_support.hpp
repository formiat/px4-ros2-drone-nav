#pragma once

#include "drone_city_nav/persistent_dstar_lite_planner_3d.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <span>
#include <stdexcept>
#include <utility>
#include <vector>

namespace drone_city_nav {
namespace {

[[nodiscard]] inline PersistentPlannerConfig3D testConfig() {
  PersistentPlannerConfig3D config;
  config.minimum_horizontal_step_m = 1.0;
  config.minimum_vertical_step_m = 1.0;
  config.time_model.maximum_horizontal_speed_mps = 5.0;
  config.time_model.maximum_vertical_speed_mps = 2.0;
  config.goal_tolerance_m = 0.01;
  config.feasibility_first_enabled = false;
  config.maximum_compute_time_ms = 1000.0;
  config.maximum_no_route_compute_time_ms = 1000.0;
  config.maximum_expansions_per_update = 100000U;
  config.physical_footprint.radius_m = 0.0;
  config.physical_footprint.lower_extent_m = 0.0;
  config.physical_footprint.upper_extent_m = 0.0;
  config.physical_footprint.perimeter_samples = 0U;
  config.physical_footprint.radial_rings = 0U;
  config.physical_footprint.axial_samples = 0U;
  config.physical_footprint.sweep_step_m = 0.2;
  config.flight_envelope.minimum_target_z_m = 0.0;
  config.flight_envelope.maximum_target_z_m = 20.0;
  return config;
}

[[nodiscard]] inline PersistentPlannerWorld3D
world(std::shared_ptr<const ObservedOccupancyGrid3D> occupancy,
      const std::uint64_t revision,
      std::vector<OccupancyChunkIndex3D> dirty_chunks = {},
      const bool full_reset = false) {
  const OccupancyGrid3D occupied = occupancy->occupiedSnapshot();
  return PersistentPlannerWorld3D{
      .observed_occupancy = std::move(occupancy),
      .static_occupancy = nullptr,
      .proprioceptive_free_space_seed = std::nullopt,
      .launch_support_contact = std::nullopt,
      .dirty_chunks = std::move(dirty_chunks),
      .producer_instance_id = 17U,
      .revision = revision,
      .incremental_parent_revision = revision > 1U ? revision - 1U : 0U,
      .occupied_fingerprint = occupied.contentFingerprint(),
      .full_reset = full_reset,
  };
}

[[nodiscard]] inline PersistentPlannerRequest3D
request(const Point3& start, const Point3& goal, PersistentPlannerWorld3D raw_world,
        const std::uint64_t mission_epoch = 3U) {
  return PersistentPlannerRequest3D{
      .start = start,
      .velocity = {},
      .mission_goal = goal,
      .mission_epoch = mission_epoch,
      .world = std::move(raw_world),
  };
}

[[nodiscard]] inline const SpatialRouteCandidate3D&
candidate(const PlannerUpdate3D& update) {
  if (!update.improved_incumbent.has_value()) {
    throw std::logic_error{"planner update has no improved incumbent"};
  }
  return *update.improved_incumbent;
}

inline void expectSamePath(const std::vector<Point3>& first,
                           const std::vector<Point3>& second) {
  ASSERT_EQ(first.size(), second.size());
  for (std::size_t index = 0U; index < first.size(); ++index) {
    EXPECT_NEAR(first[index].x, second[index].x, 1.0e-12);
    EXPECT_NEAR(first[index].y, second[index].y, 1.0e-12);
    EXPECT_NEAR(first[index].z, second[index].z, 1.0e-12);
  }
}

inline void expectRawValid(const std::vector<Point3>& path,
                           const ObservedOccupancyGrid3D& occupancy,
                           const SweptFootprintConfig& footprint) {
  ASSERT_GE(path.size(), 2U);
  for (std::size_t index = 1U; index < path.size(); ++index) {
    EXPECT_TRUE(validateRawSweptFootprint(occupancy, path[index - 1U],
                                          FootprintBodyAxis{}, path[index],
                                          FootprintBodyAxis{}, footprint)
                    .accepted());
  }
}

} // namespace
} // namespace drone_city_nav
