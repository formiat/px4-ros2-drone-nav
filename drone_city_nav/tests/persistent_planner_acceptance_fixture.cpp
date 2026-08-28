#include "persistent_planner_acceptance_fixture.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <stdexcept>
#include <utility>

namespace drone_city_nav::test {
namespace {

constexpr GridBounds3D kStandardBounds{0.0, 0.0, 0.0, 1.0, 24, 16, 12};

class FixtureVolume final {
public:
  explicit FixtureVolume(const GridBounds3D bounds = kStandardBounds)
      : occupancy_{std::make_shared<ObservedOccupancyGrid3D>(bounds)} {
  }

  void fillOccupied() {
    const GridBounds3D& bounds = occupancy_->bounds();
    for (int z = 0; z < bounds.depth_cells; ++z) {
      for (int y = 0; y < bounds.height_cells; ++y) {
        for (int x = 0; x < bounds.width_cells; ++x) {
          static_cast<void>(
              occupancy_->setState({x, y, z}, ObservedVoxelState::kOccupied));
        }
      }
    }
  }

  void occupyWallX(const int wall_x,
                   const std::function<bool(const GridIndex3D&)>& opening) {
    const GridBounds3D& bounds = occupancy_->bounds();
    for (int z = 0; z < bounds.depth_cells; ++z) {
      for (int y = 0; y < bounds.height_cells; ++y) {
        const GridIndex3D index{wall_x, y, z};
        if (!opening(index)) {
          static_cast<void>(occupancy_->setState(index, ObservedVoxelState::kOccupied));
        }
      }
    }
  }

  void occupySlabZ(const int slab_z,
                   const std::function<bool(const GridIndex3D&)>& opening) {
    const GridBounds3D& bounds = occupancy_->bounds();
    for (int y = 0; y < bounds.height_cells; ++y) {
      for (int x = 0; x < bounds.width_cells; ++x) {
        const GridIndex3D index{x, y, slab_z};
        if (!opening(index)) {
          static_cast<void>(occupancy_->setState(index, ObservedVoxelState::kOccupied));
        }
      }
    }
  }

  void carveBox(const GridIndex3D minimum, const GridIndex3D maximum) {
    for (int z = minimum.z; z <= maximum.z; ++z) {
      for (int y = minimum.y; y <= maximum.y; ++y) {
        for (int x = minimum.x; x <= maximum.x; ++x) {
          static_cast<void>(
              occupancy_->setState({x, y, z}, ObservedVoxelState::kUnknown));
        }
      }
    }
  }

  void carveSphere(const Point3& center, const double radius_m) {
    const GridBounds3D& bounds = occupancy_->bounds();
    const double squared_radius_m2 = radius_m * radius_m;
    for (int z = 0; z < bounds.depth_cells; ++z) {
      for (int y = 0; y < bounds.height_cells; ++y) {
        for (int x = 0; x < bounds.width_cells; ++x) {
          const Point3 point = occupancy_->cellCenter({x, y, z});
          const double dx = point.x - center.x;
          const double dy = point.y - center.y;
          const double dz = point.z - center.z;
          if (dx * dx + dy * dy + dz * dz <= squared_radius_m2) {
            static_cast<void>(
                occupancy_->setState({x, y, z}, ObservedVoxelState::kUnknown));
          }
        }
      }
    }
  }

  void carvePolyline(const std::vector<Point3>& points, const double radius_m) {
    if (points.size() < 2U) {
      throw std::invalid_argument{"acceptance polyline needs two points"};
    }
    for (std::size_t segment = 1U; segment < points.size(); ++segment) {
      const Point3& first = points[segment - 1U];
      const Point3& second = points[segment];
      const double length_m = distance3D(first, second);
      const std::size_t steps = std::max<std::size_t>(
          1U, static_cast<std::size_t>(std::ceil(length_m / 0.5)));
      for (std::size_t step = 0U; step <= steps; ++step) {
        const double ratio = static_cast<double>(step) / static_cast<double>(steps);
        carveSphere(Point3{std::lerp(first.x, second.x, ratio),
                           std::lerp(first.y, second.y, ratio),
                           std::lerp(first.z, second.z, ratio)},
                    radius_m);
      }
    }
  }

  [[nodiscard]] std::shared_ptr<ObservedOccupancyGrid3D> finish() {
    return std::move(occupancy_);
  }

private:
  std::shared_ptr<ObservedOccupancyGrid3D> occupancy_;
};

[[nodiscard]] PersistentPlannerAcceptanceFixture
openingFixture(const PersistentPlannerAcceptanceKind kind) {
  FixtureVolume volume;
  const bool lower = kind == PersistentPlannerAcceptanceKind::kLowerOpening;
  const bool upper = kind == PersistentPlannerAcceptanceKind::kUpperOpening;
  volume.occupyWallX(11, [lower, upper](const GridIndex3D index) {
    const bool centered_laterally = index.y >= 6 && index.y <= 9;
    if (lower) {
      return centered_laterally && index.z >= 1 && index.z <= 4;
    }
    if (upper) {
      return centered_laterally && index.z >= 7 && index.z <= 10;
    }
    return index.y >= 1 && index.y <= 4 && index.z >= 3 && index.z <= 7;
  });
  std::string_view name = "lateral_opening";
  double altitude_m = 5.5;
  RequiredRouteMotion required_motion = RequiredRouteMotion::kLateral;
  if (lower) {
    name = "lower_opening";
    altitude_m = 7.5;
    required_motion = RequiredRouteMotion::kDescend;
  } else if (upper) {
    name = "upper_opening";
    altitude_m = 3.5;
    required_motion = RequiredRouteMotion::kClimb;
  }
  return PersistentPlannerAcceptanceFixture{
      .kind = kind,
      .name = name,
      .occupancy = volume.finish(),
      .missions = {{.start = {3.5, 8.5, altitude_m},
                    .goal = {20.5, 8.5, altitude_m},
                    .required_motion = required_motion}},
      .local_distance_cache_bounds = std::nullopt,
  };
}

[[nodiscard]] PersistentPlannerAcceptanceFixture verticalShaftFixture() {
  FixtureVolume volume;
  volume.occupySlabZ(6, [](const GridIndex3D index) {
    return index.x >= 5 && index.x <= 8 && index.y >= 6 && index.y <= 9;
  });
  return PersistentPlannerAcceptanceFixture{
      .kind = PersistentPlannerAcceptanceKind::kVerticalShaft,
      .name = "vertical_shaft",
      .occupancy = volume.finish(),
      .missions = {{.start = {6.5, 7.5, 2.5},
                    .goal = {6.5, 7.5, 10.5},
                    .required_motion = RequiredRouteMotion::kVertical}},
      .local_distance_cache_bounds = std::nullopt,
  };
}

[[nodiscard]] PersistentPlannerAcceptanceFixture inclinedPassageFixture() {
  const std::vector<Point3> centerline{
      {2.5, 7.5, 2.5}, {8.5, 7.5, 4.5}, {14.5, 7.5, 7.5}, {21.5, 7.5, 9.5}};
  FixtureVolume volume;
  volume.fillOccupied();
  volume.carvePolyline(centerline, 2.25);
  volume.carveSphere(centerline.front(), 3.0);
  volume.carveSphere(centerline.back(), 3.0);
  return PersistentPlannerAcceptanceFixture{
      .kind = PersistentPlannerAcceptanceKind::kInclinedPassage,
      .name = "inclined_passage",
      .occupancy = volume.finish(),
      .missions = {{.start = centerline.front(),
                    .goal = centerline.back(),
                    .required_motion = RequiredRouteMotion::kInclined}},
      .local_distance_cache_bounds = std::nullopt,
  };
}

[[nodiscard]] PersistentPlannerAcceptanceFixture awayFromGoalWallFixture() {
  FixtureVolume volume;
  volume.occupyWallX(11, [](const GridIndex3D index) { return index.y >= 14; });
  return PersistentPlannerAcceptanceFixture{
      .kind = PersistentPlannerAcceptanceKind::kAwayFromGoalWall,
      .name = "away_from_goal_wall",
      .occupancy = volume.finish(),
      .missions = {{.start = {7.5, 7.5, 5.5},
                    .goal = {15.5, 7.5, 5.5},
                    .required_motion = RequiredRouteMotion::kInitiallyAwayFromGoal}},
      .local_distance_cache_bounds = std::nullopt,
  };
}

[[nodiscard]] PersistentPlannerAcceptanceFixture junctionFixture(const bool x_shape) {
  FixtureVolume volume;
  volume.fillOccupied();
  volume.carveBox({2, 6, 3}, {21, 9, 7});
  volume.carveBox({10, x_shape ? 2 : 6, 3}, {13, 13, 7});
  std::vector<PersistentPlannerAcceptanceMission> missions{
      {.start = {3.5, 7.5, 5.5}, .goal = {11.5, 12.5, 5.5}},
      {.start = {20.5, 7.5, 5.5}, .goal = {11.5, 12.5, 5.5}},
  };
  if (x_shape) {
    missions.push_back({.start = {11.5, 3.5, 5.5}, .goal = {11.5, 12.5, 5.5}});
  }
  return PersistentPlannerAcceptanceFixture{
      .kind = x_shape ? PersistentPlannerAcceptanceKind::kXJunction
                      : PersistentPlannerAcceptanceKind::kTJunction,
      .name = x_shape ? "x_junction" : "t_junction",
      .occupancy = volume.finish(),
      .missions = std::move(missions),
      .local_distance_cache_bounds = std::nullopt,
  };
}

[[nodiscard]] PersistentPlannerAcceptanceFixture loopFixture() {
  FixtureVolume volume;
  volume.fillOccupied();
  volume.carveBox({2, 2, 3}, {21, 5, 7});
  volume.carveBox({2, 10, 3}, {21, 13, 7});
  volume.carveBox({2, 2, 3}, {5, 13, 7});
  volume.carveBox({18, 2, 3}, {21, 13, 7});
  return PersistentPlannerAcceptanceFixture{
      .kind = PersistentPlannerAcceptanceKind::kLoop,
      .name = "loop_with_incremental_repair",
      .occupancy = volume.finish(),
      .missions = {{.start = {3.5, 3.5, 5.5}, .goal = {20.5, 12.5, 5.5}}},
      .local_distance_cache_bounds = std::nullopt,
  };
}

[[nodiscard]] PersistentPlannerAcceptanceFixture culDeSacFixture() {
  FixtureVolume volume;
  volume.fillOccupied();
  volume.carveBox({4, 9, 3}, {15, 12, 7});
  volume.carveBox({4, 3, 3}, {7, 12, 7});
  volume.carveBox({4, 3, 3}, {20, 6, 7});
  volume.carveBox({18, 3, 3}, {21, 12, 7});
  return PersistentPlannerAcceptanceFixture{
      .kind = PersistentPlannerAcceptanceKind::kCulDeSacReturn,
      .name = "cul_de_sac_return",
      .occupancy = volume.finish(),
      .missions = {{.start = {14.5, 10.5, 5.5},
                    .goal = {19.5, 10.5, 5.5},
                    .required_motion = RequiredRouteMotion::kInitiallyAwayFromGoal}},
      .local_distance_cache_bounds = std::nullopt,
  };
}

[[nodiscard]] PersistentPlannerAcceptanceFixture beyondCacheFixture() {
  const GridBounds3D bounds{0.0, 0.0, 0.0, 1.0, 32, 12, 8};
  FixtureVolume volume{bounds};
  return PersistentPlannerAcceptanceFixture{
      .kind = PersistentPlannerAcceptanceKind::kBeyondLocalDistanceCache,
      .name = "beyond_local_distance_cache",
      .occupancy = volume.finish(),
      .missions = {{.start = {1.5, 5.5, 3.5},
                    .goal = {29.5, 5.5, 3.5},
                    .required_motion = RequiredRouteMotion::kBeyondLocalDistanceCache}},
      .local_distance_cache_bounds = GridBounds3D{0.0, 0.0, 0.0, 1.0, 12, 12, 8},
  };
}

} // namespace

std::vector<PersistentPlannerAcceptanceKind> persistentPlannerAcceptanceKinds() {
  return {
      PersistentPlannerAcceptanceKind::kUnobstructed,
      PersistentPlannerAcceptanceKind::kLowerOpening,
      PersistentPlannerAcceptanceKind::kUpperOpening,
      PersistentPlannerAcceptanceKind::kLateralOpening,
      PersistentPlannerAcceptanceKind::kVerticalShaft,
      PersistentPlannerAcceptanceKind::kInclinedPassage,
      PersistentPlannerAcceptanceKind::kAwayFromGoalWall,
      PersistentPlannerAcceptanceKind::kTJunction,
      PersistentPlannerAcceptanceKind::kXJunction,
      PersistentPlannerAcceptanceKind::kLoop,
      PersistentPlannerAcceptanceKind::kCulDeSacReturn,
      PersistentPlannerAcceptanceKind::kBeyondLocalDistanceCache,
  };
}

PersistentPlannerAcceptanceFixture
buildPersistentPlannerAcceptanceFixture(const PersistentPlannerAcceptanceKind kind) {
  switch (kind) {
    case PersistentPlannerAcceptanceKind::kUnobstructed: {
      FixtureVolume volume;
      return PersistentPlannerAcceptanceFixture{
          .kind = kind,
          .name = "unobstructed",
          .occupancy = volume.finish(),
          .missions = {{.start = {2.5, 2.5, 3.5}, .goal = {21.5, 13.5, 8.5}}},
          .local_distance_cache_bounds = std::nullopt,
      };
    }
    case PersistentPlannerAcceptanceKind::kLowerOpening:
    case PersistentPlannerAcceptanceKind::kUpperOpening:
    case PersistentPlannerAcceptanceKind::kLateralOpening:
      return openingFixture(kind);
    case PersistentPlannerAcceptanceKind::kVerticalShaft:
      return verticalShaftFixture();
    case PersistentPlannerAcceptanceKind::kInclinedPassage:
      return inclinedPassageFixture();
    case PersistentPlannerAcceptanceKind::kAwayFromGoalWall:
      return awayFromGoalWallFixture();
    case PersistentPlannerAcceptanceKind::kTJunction:
      return junctionFixture(false);
    case PersistentPlannerAcceptanceKind::kXJunction:
      return junctionFixture(true);
    case PersistentPlannerAcceptanceKind::kLoop:
      return loopFixture();
    case PersistentPlannerAcceptanceKind::kCulDeSacReturn:
      return culDeSacFixture();
    case PersistentPlannerAcceptanceKind::kBeyondLocalDistanceCache:
      return beyondCacheFixture();
  }
  throw std::invalid_argument{"unsupported persistent planner acceptance fixture"};
}

} // namespace drone_city_nav::test
