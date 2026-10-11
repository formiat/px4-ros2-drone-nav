#pragma once

#include "drone_city_nav/swept_footprint.hpp"

#include <memory>
#include <optional>

#include "production_mppi_route_world.hpp"
#include "route_activation_preparation_3d.hpp"

namespace drone_city_nav::route_activation_preparation_detail {

// What the preparation's stages share: the exact vehicle state and the
// activation footprint, and the two assessments defined in
// route_activation_preparation_3d_assessment.cpp.

template<typename T>
[[nodiscard]] const T* optionalAddress(const std::optional<T>& value) noexcept {
  if (!value.has_value()) {
    return nullptr;
  }
  return std::addressof(value.value());
}

[[nodiscard]] inline VehicleState3D
exactVehicleState3D(const ProductionMppiNavigation& navigation) noexcept {
  return VehicleState3D{
      .identity =
          VehicleStateIdentity3D{
              .revision = navigation.revision,
              .source_timestamp_us = navigation.source_timestamp_us,
              .receive_stamp_ns = navigation.receive_stamp_ns,
          },
      .position = Point3{navigation.state.x, navigation.state.y, navigation.state.z},
      .velocity = Vec3{navigation.state.vx, navigation.state.vy, navigation.state.vz},
      .yaw_rad = navigation.state.yaw,
      .yaw_rate_radps = navigation.state.yaw_rate,
  };
}

[[nodiscard]] inline SweptFootprintConfig
activationFootprint(const RouteActivationCoordinatorConfig3D& config) noexcept {
  return SweptFootprintConfig{
      .radius_m = config.physical_footprint.radius_m,
      .lower_extent_m = config.physical_footprint.lower_extent_m,
      .upper_extent_m = config.physical_footprint.upper_extent_m,
      .body_radius_m = config.physical_footprint.body_radius_m,
      .body_lower_extent_m = config.physical_footprint.body_lower_extent_m,
      .body_upper_extent_m = config.physical_footprint.body_upper_extent_m,
      .perimeter_samples = config.physical_footprint.perimeter_samples,
      .radial_rings = config.physical_footprint.radial_rings,
      .axial_samples = config.physical_footprint.axial_samples,
      .sweep_step_m = config.physical_footprint.sweep_step_m,
  };
}

struct RouteActivationPreparationState3D final {
  RouteActivationPreparationState3D() = default;
  RouteActivationPreparationState3D(const RouteActivationPreparationState3D&) = delete;
  RouteActivationPreparationState3D&
  operator=(const RouteActivationPreparationState3D&) = delete;
  RouteActivationPreparationState3D(RouteActivationPreparationState3D&&) = default;
  RouteActivationPreparationState3D&
  operator=(RouteActivationPreparationState3D&&) = default;
  ~RouteActivationPreparationState3D() = default;

  PreparedRouteActivation3D prepared{};
  std::shared_ptr<const PlannerSearchTransaction3D> transaction;
  StaticRouteReplacementPolicy replacement_policy{
      StaticRouteReplacementPolicy::kRequireEndpointImprovement};
  StaticRoutePlanningLatencyStats planning_latency{};
  NavigationWorldCertificate3D planned_world_certificate{};
  std::shared_ptr<const VersionedObservedRawWorld3D> activation_raw_owner;
  // The seed the candidate is validated with: the world's seed stood where
  // the vehicle now stands, carrying the candidate's own departure.
  std::optional<ProprioceptiveFreeSpaceSeed3D> departure_seed;
  std::optional<CertifiedRouteSuffix3D> certified_route;
  std::uint64_t base_generation{0U};
  bool raw_validation_required{false};
  bool overlap_search{false};
  bool overlap_base_matches{false};
  bool splice_ready{false};
  bool successor_improvement_cleared{false};
  double blocked_replacement_held_s{0.0};
};

// The grace a blocked route's worse replacement gets, as a share of what the
// replacement would cost over the route it replaces. Waiting is a bet that the
// block clears or the search finds better. At half the loss the bet was
// measured on the urban flights r261 to r265: 34 to 43 held replacements per
// flight, 1.8 to 2.6 s each, and the vehicle braked for the block under the
// blocked-route limiter through every one of them (9 to 21 s lost per flight
// against 4.5 m/s), while the better route the wait was for replaced the
// taken one in 1 of 108 successor assessments. No grace: the first raw-valid
// replacement is taken, and a better one replaces it under the successor
// improvement rule as the search converges.
constexpr double kBlockedReplacementGraceLossRatio{0.0};

[[nodiscard]] RouteActivationPreparationState3D
assessAdmission(RouteActivationPreparationState3D state,
                const RouteActivationCoordinatorConfig3D& config);

[[nodiscard]] RouteActivationPreparationState3D
assessReplacement(RouteActivationPreparationState3D state,
                  const RouteActivationCoordinatorConfig3D& config);

} // namespace drone_city_nav::route_activation_preparation_detail
