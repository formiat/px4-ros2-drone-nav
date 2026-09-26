#pragma once

#include "drone_city_nav/persistent_dstar_lite_planner_3d.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cmath>
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

// The path clears the body as the planner accepts it: the departure swept
// whole, every later segment in the pieces the route sampler cuts it into.
inline void expectRawValid(const std::vector<Point3>& path,
                           const ObservedOccupancyGrid3D& occupancy,
                           const SweptFootprintConfig& footprint,
                           const double route_sampling_step_m = 0.5) {
  ASSERT_GE(path.size(), 2U);
  EXPECT_TRUE(validateRawSweptFootprint(occupancy, path[0], FootprintBodyAxis{},
                                        path[1], FootprintBodyAxis{}, footprint)
                  .accepted());
  for (std::size_t index = 2U; index < path.size(); ++index) {
    const Point3& first = path[index - 1U];
    const Point3& second = path[index];
    const auto pieces = std::max<std::size_t>(
        1U, static_cast<std::size_t>(
                std::ceil(distance3D(first, second) / route_sampling_step_m)));
    Point3 previous = first;
    for (std::size_t piece = 1U; piece <= pieces; ++piece) {
      const double ratio = static_cast<double>(piece) / static_cast<double>(pieces);
      const Point3 next{std::lerp(first.x, second.x, ratio),
                        std::lerp(first.y, second.y, ratio),
                        std::lerp(first.z, second.z, ratio)};
      EXPECT_TRUE(validateRawSweptFootprint(occupancy, previous, FootprintBodyAxis{},
                                            next, FootprintBodyAxis{}, footprint)
                      .accepted());
      previous = next;
    }
  }
}

} // namespace
} // namespace drone_city_nav
