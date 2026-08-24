#pragma once

#include "drone_city_nav/route_planning_3d.hpp"

#include <cstdint>
#include <limits>
#include <string_view>

namespace drone_city_nav {

enum class RouteEndpointSemantics3D : std::uint8_t {
  kContinuation,
  kObservationStop,
  kMissionStop,
  kEmergencyBrakeTail,
};

struct RouteContinuityLineage3D {
  std::uint64_t mission_epoch{0U};
  std::uint64_t assignment_generation{0U};
  std::uint64_t target_detection_id{0U};
  std::uint64_t target_track_id{0U};
};

[[nodiscard]] RouteEndpointSemantics3D
routeEndpointSemantics3D(const RouteIntent3D& intent, bool reaches_intent_target,
                         bool reaches_mission_goal) noexcept;

[[nodiscard]] RouteEndpointSemantics3D
effectiveRouteEndpointSemantics3D(RouteEndpointSemantics3D planned_semantics,
                                  bool raw_invalidation_active,
                                  bool finite_braking_tail_active) noexcept;

[[nodiscard]] std::uint64_t
routeContinuityId3D(const RouteIntent3D& intent,
                    const RouteContinuityLineage3D& lineage = {}) noexcept;

[[nodiscard]] double routeSpeed3D(const Vec3& velocity) noexcept;

[[nodiscard]] std::string_view
routeEndpointSemantics3DName(RouteEndpointSemantics3D semantics) noexcept;

struct RollingRouteTelemetryConfig3D {
  double continuation_boundary_distance_m{1.0};
  double stationary_speed_tolerance_mps{0.25};
};

struct RollingRouteTelemetryObservation3D {
  std::uint64_t route_generation{0U};
  std::uint64_t continuity_id{0U};
  std::uint64_t geometry_revision{0U};
  RouteEndpointSemantics3D endpoint_semantics{RouteEndpointSemantics3D::kContinuation};
  double route_remaining_m{std::numeric_limits<double>::infinity()};
  double speed_mps{0.0};
  bool resident_route_available{false};
  bool execution_owner_available{false};
  bool endpoint_limiter_active{false};
  bool raw_invalidation_active{false};
  bool finite_braking_tail_active{false};
  bool nominal_reseeded{false};
  bool continuity_preserving_update{false};
};

struct RollingRouteTelemetrySnapshot3D {
  std::uint64_t observations{0U};
  std::uint64_t continuation_boundary_ticks{0U};
  std::uint64_t continuation_zero_speed_ticks{0U};
  std::uint64_t continuation_endpoint_limited_ticks{0U};
  std::uint64_t continuity_transition_ticks{0U};
  std::uint64_t continuity_transition_zero_speed_ticks{0U};
  std::uint64_t ownership_gap_ticks{0U};
  std::uint64_t ownership_gap_episodes{0U};
  std::uint64_t maximum_consecutive_ownership_gap_ticks{0U};
  std::uint64_t moving_raw_invalidation_ticks{0U};
  std::uint64_t moving_raw_invalidation_without_braking_tail_ticks{0U};
  std::uint64_t finite_braking_tail_activations{0U};
  std::uint64_t nominal_reseed_ticks{0U};
  std::uint64_t continuity_preserving_reseed_ticks{0U};
  std::uint64_t route_generation_changes{0U};
  std::uint64_t continuity_preserving_generation_changes{0U};
  std::uint64_t geometry_revision_changes{0U};
  double minimum_continuation_boundary_speed_mps{
      std::numeric_limits<double>::infinity()};
  double minimum_continuity_transition_speed_mps{
      std::numeric_limits<double>::infinity()};
  double maximum_continuity_transition_speed_drop_mps{0.0};

  [[nodiscard]] bool regressionFree() const noexcept;
};

class RollingRouteTelemetry3D final {
public:
  explicit RollingRouteTelemetry3D(RollingRouteTelemetryConfig3D config = {});

  void observe(const RollingRouteTelemetryObservation3D& observation) noexcept;
  [[nodiscard]] const RollingRouteTelemetrySnapshot3D& snapshot() const noexcept;
  void reset() noexcept;

private:
  RollingRouteTelemetryConfig3D config_{};
  RollingRouteTelemetrySnapshot3D snapshot_{};
  RollingRouteTelemetryObservation3D previous_{};
  std::uint64_t consecutive_ownership_gap_ticks_{0U};
  bool previous_available_{false};
  bool previous_ownership_gap_{false};
  bool previous_braking_tail_active_{false};
};

} // namespace drone_city_nav
