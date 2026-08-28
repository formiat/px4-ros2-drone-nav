#pragma once

#include "drone_city_nav/observed_occupancy_grid_3d.hpp"
#include "drone_city_nav/types.hpp"

#include <cstdint>
#include <memory>
#include <optional>
#include <string_view>
#include <vector>

namespace drone_city_nav::test {

enum class PersistentPlannerAcceptanceKind : std::uint8_t {
  kUnobstructed,
  kLowerOpening,
  kUpperOpening,
  kLateralOpening,
  kVerticalShaft,
  kInclinedPassage,
  kAwayFromGoalWall,
  kTJunction,
  kXJunction,
  kLoop,
  kCulDeSacReturn,
  kBeyondLocalDistanceCache,
};

enum class RequiredRouteMotion : std::uint8_t {
  kNone,
  kDescend,
  kClimb,
  kLateral,
  kVertical,
  kInclined,
  kInitiallyAwayFromGoal,
  kBeyondLocalDistanceCache,
};

struct PersistentPlannerAcceptanceMission {
  Point3 start{};
  Point3 goal{};
  RequiredRouteMotion required_motion{RequiredRouteMotion::kNone};
};

struct PersistentPlannerAcceptanceFixture {
  PersistentPlannerAcceptanceKind kind{PersistentPlannerAcceptanceKind::kUnobstructed};
  std::string_view name;
  std::shared_ptr<ObservedOccupancyGrid3D> occupancy;
  std::vector<PersistentPlannerAcceptanceMission> missions;
  std::optional<GridBounds3D> local_distance_cache_bounds;
};

[[nodiscard]] std::vector<PersistentPlannerAcceptanceKind>
persistentPlannerAcceptanceKinds();

[[nodiscard]] PersistentPlannerAcceptanceFixture
buildPersistentPlannerAcceptanceFixture(PersistentPlannerAcceptanceKind kind);

} // namespace drone_city_nav::test
