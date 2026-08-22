#pragma once

#include "drone_city_nav/observed_occupancy_grid_3d.hpp"
#include "drone_city_nav/route_3d.hpp"
#include "drone_city_nav/route_planning_3d.hpp"
#include "drone_city_nav/static_route_extension.hpp"
#include "drone_city_nav/swept_footprint.hpp"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <string_view>

namespace drone_city_nav {

struct NavigationWorldCertificate3D {
  std::uint64_t producer_instance_id{0U};
  std::uint64_t esdf_fingerprint{0U};
  std::uint64_t esdf_source_raw_revision{0U};
  std::uint64_t esdf_source_occupied_fingerprint{0U};
  std::uint64_t raw_validated_through_revision{0U};

  [[nodiscard]] bool valid() const noexcept;
};

struct MaterializedRouteProposal3D {
  NavigationWorldCertificate3D planned_world{};
  NavigationWorldCertificate3D validated_world{};
  StaticRouteObjective objective{};
  RouteIntent3D intent{};
  SegmentEvidence3D evidence{};
  std::uint64_t route_fingerprint{0U};
  std::size_t route_sample_count{0U};
  bool reaches_mission_goal{false};
  bool activation_eligible{false};
};

struct ActivatedRouteIdentity3D {
  std::uint64_t generation{0U};
  MaterializedRouteProposal3D proposal{};
};

enum class RawRouteSuffixStatus3D : std::uint8_t {
  kValid,
  kInvalidRoute,
  kInvalidProjection,
  kRawCollision,
};

struct RawRouteSuffixValidation3D {
  RawRouteSuffixStatus3D status{RawRouteSuffixStatus3D::kInvalidRoute};
  std::size_t first_validated_route_segment{0U};
  std::size_t failure_route_segment{0U};
  Point3 failure_point{};
  bool connector_validated{false};

  [[nodiscard]] bool accepted() const noexcept;
};

enum class RouteExecutionStatus3D : std::uint8_t {
  kUsable,
  kNoActiveRoute,
  kWorldLineageMismatch,
  kObjectiveMismatch,
  kInvalidRoute,
  kInvalidProjection,
  kExcessiveCrossTrack,
  kRawCollision,
};

struct RouteExecutionObservation3D {
  StaticRouteObjective current_objective{};
  std::uint64_t minimum_tracking_sample_sequence{0U};
  std::uint64_t previously_validated_through_raw_revision{0U};
  Point3 position{};
  double minimum_station_m{0.0};
  double maximum_cross_track_m{0.0};
  const ObservedOccupancyGrid3D* latest_raw_occupancy{nullptr};
  std::uint64_t latest_raw_producer_instance_id{0U};
  std::uint64_t latest_raw_revision{0U};
  SweptFootprintConfig footprint{};
  const ProprioceptiveFreeSpaceSeed3D* proprioceptive_free_space_seed{nullptr};
  const LaunchSupportContact3D* launch_support_contact{nullptr};
};

struct RouteExecutionAssessment3D {
  RouteExecutionStatus3D status{RouteExecutionStatus3D::kNoActiveRoute};
  RouteProjection3D projection{};
  RawRouteSuffixValidation3D raw_validation{};
  std::uint64_t validated_through_raw_revision{0U};

  [[nodiscard]] bool usable() const noexcept;
};

struct RouteExecutionState3D {
  std::uint64_t generation{0U};
  std::uint64_t raw_validated_through_revision{0U};
  double station_m{0.0};
};

[[nodiscard]] std::optional<ActivatedRouteIdentity3D>
activateRouteProposal3D(const MaterializedRouteProposal3D& proposal,
                        std::uint64_t generation) noexcept;

[[nodiscard]] RawRouteSuffixValidation3D validateRawRouteSuffix3D(
    std::span<const RouteSample3D> route, const Point3& position,
    const RouteProjection3D& projection, const ObservedOccupancyGrid3D& occupancy,
    const SweptFootprintConfig& footprint,
    const ProprioceptiveFreeSpaceSeed3D* proprioceptive_free_space_seed = nullptr,
    const LaunchSupportContact3D* launch_support_contact = nullptr) noexcept;

[[nodiscard]] RouteExecutionAssessment3D
assessRouteExecution3D(const ActivatedRouteIdentity3D* active_route,
                       std::span<const RouteSample3D> route,
                       const RouteExecutionObservation3D& observation) noexcept;

[[nodiscard]] std::string_view
rawRouteSuffixStatus3DName(RawRouteSuffixStatus3D status) noexcept;
[[nodiscard]] std::string_view
routeExecutionStatus3DName(RouteExecutionStatus3D status) noexcept;

} // namespace drone_city_nav
