#pragma once

#include "drone_city_nav/flight_envelope.hpp"
#include "drone_city_nav/observed_occupancy_grid_3d.hpp"
#include "drone_city_nav/route_3d.hpp"
#include "drone_city_nav/swept_footprint.hpp"
#include "drone_city_nav/types.hpp"

#include <cstddef>
#include <cstdint>
#include <limits>
#include <span>

namespace drone_city_nav {

enum class RouteIntentSource3D : std::uint8_t {
  kDirect,
  kTopology,
  kLaunchDeparture,
  kPersistentPlanner,
};

enum class RouteIntentPurpose3D : std::uint8_t {
  kMissionTransit,
  kLaunchDeparture,
  kObservationFrontier,
  kTopologicalBacktrack,
};

struct RouteIntent3D {
  std::uint64_t id{0U};
  std::uint64_t strategic_plan_id{0U};
  std::uint64_t planned_on_revision{0U};
  std::uint64_t target_identity{0U};
  Point3 mission_target{};
  Point3 intent_target{};
  Point3 segment_target{};
  RouteIntentSource3D source{RouteIntentSource3D::kDirect};
  RouteIntentPurpose3D purpose{RouteIntentPurpose3D::kMissionTransit};
  bool segment_reaches_intent_target{false};
  bool observation_stop_required{true};
  bool valid{false};
};

enum class SegmentEvidenceStatus3D : std::uint8_t {
  kValid,
  kEmptyRoute,
  kPlannerRejected,
  kInvalidWorld,
  kOutsideFlightEnvelope,
  kOutsideGrid,
  kUnknownRejected,
  kInvalidEsdf,
  kRawCollision,
};

struct SegmentEvidence3D {
  std::uint64_t planned_on_revision{0U};
  std::uint64_t validated_through_revision{0U};
  SegmentEvidenceStatus3D status{SegmentEvidenceStatus3D::kInvalidWorld};
  Point3 failure_point{};
  std::size_t failure_segment_index{0U};
  double route_length_m{0.0};
  double endpoint_displacement_m{0.0};
  double mission_progress_m{0.0};
  double net_coordinate_progress_m{std::numeric_limits<double>::quiet_NaN()};
  double objective_cost{std::numeric_limits<double>::infinity()};
  double minimum_known_clearance_m{std::numeric_limits<double>::infinity()};
  bool materialized{false};
  bool planner_executable{false};
  bool physical_executable{false};
  bool reaches_segment_target{false};
  bool reaches_intent_target{false};
  bool reaches_mission_target{false};
  bool raw_collision{false};
  bool outside_grid_exposure{false};
  bool unknown_exposure{false};
  bool invalid_esdf_exposure{false};
  bool known_clearance_observed{false};
};

// Measures endpoint displacement and signed radial progress in the same 3D
// mission metric. Preparatory vertical and lateral motion therefore remains
// visible to strategic diagnostics instead of collapsing onto XY.
[[nodiscard]] double
routeNetCoordinateProgress3D(const Point3& start, const Point3& endpoint,
                             const Point3& mission_target) noexcept;

struct SegmentEvidenceWorld3D {
  const mppi::EsdfGrid* grid{nullptr};
  std::span<const float> esdf_m;
  const ObservedOccupancyGrid3D* latest_observed_occupancy{nullptr};
  const OccupancyGrid3D* static_occupancy{nullptr};
  const ProprioceptiveFreeSpaceSeed3D* proprioceptive_free_space_seed{nullptr};
  const LaunchSupportContact3D* launch_support_contact{nullptr};
  SweptFootprintConfig footprint{};
  FlightEnvelopeConfig flight_envelope{};
  std::uint64_t validated_through_revision{0U};
  bool require_known_free_space{false};
  bool reject_invalid_esdf{false};
};

[[nodiscard]] std::uint64_t
makeRouteIntentId3D(RouteIntentSource3D source, RouteIntentPurpose3D purpose,
                    const Point3& mission_target, const Point3& intent_target,
                    std::uint64_t target_identity = 0U,
                    bool observation_stop_required = true) noexcept;

[[nodiscard]] SegmentEvidence3D evaluateSegmentEvidence3D(
    const RouteIntent3D& intent, std::span<const RouteSample3D> route,
    const Point3& search_start, bool planner_executable, bool reaches_segment_target,
    bool reaches_mission_target, double objective_cost,
    const SegmentEvidenceWorld3D& world) noexcept;

[[nodiscard]] const char* routeIntentSource3DName(RouteIntentSource3D source) noexcept;
[[nodiscard]] const char*
routeIntentPurpose3DName(RouteIntentPurpose3D purpose) noexcept;
[[nodiscard]] const char*
segmentEvidenceStatus3DName(SegmentEvidenceStatus3D status) noexcept;

} // namespace drone_city_nav
