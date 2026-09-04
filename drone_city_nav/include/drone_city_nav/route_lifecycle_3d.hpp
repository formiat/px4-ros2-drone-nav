#pragma once

#include "drone_city_nav/execution_evidence_3d.hpp"
#include "drone_city_nav/observed_occupancy_grid_3d.hpp"
#include "drone_city_nav/occupied_collision_oracle_3d.hpp"
#include "drone_city_nav/route_3d.hpp"
#include "drone_city_nav/route_planning_3d.hpp"
#include "drone_city_nav/static_route_extension.hpp"
#include "drone_city_nav/swept_footprint.hpp"
#include "drone_city_nav/versioned_world_evidence_3d.hpp"

#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <span>
#include <string_view>

namespace drone_city_nav {

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

// Stable mission-level intent owned by the route lifecycle. Geometry may be
// appended or repaired while this identity remains unchanged.
struct ActiveIntent3D {
  std::uint64_t route_intent_id{0U};
  std::uint64_t mission_epoch{0U};
  std::uint64_t assignment_generation{0U};
  std::uint64_t target_detection_id{0U};
  std::uint64_t target_track_id{0U};
  Point3 mission_target{};
  bool continuous_tracking{false};

  [[nodiscard]] bool valid() const noexcept;
};

struct RouteOwnerIdentity3D {
  std::uint64_t id{0U};
  ActiveIntent3D active_intent{};

  [[nodiscard]] bool valid() const noexcept;
};

[[nodiscard]] std::optional<ActiveIntent3D>
activeIntent3D(const MaterializedRouteProposal3D& proposal) noexcept;

[[nodiscard]] bool sameActiveIntent3D(const ActiveIntent3D& first,
                                      const ActiveIntent3D& second,
                                      double target_tolerance_m = 1.0e-6) noexcept;

enum class RoutePublicationStatus3D : std::uint8_t {
  kNotAssessed,
  kCompatible,
  kInvalidProposalWorld,
  kInvalidResidentWorld,
  kWorldLineageMismatch,
  kResidentWorldPredatesPlan,
};

struct RoutePublicationAssessment3D {
  RoutePublicationStatus3D status{RoutePublicationStatus3D::kNotAssessed};

  [[nodiscard]] bool compatible() const noexcept;
};

struct ActivatedRouteIdentity3D {
  std::uint64_t generation{0U};
  MaterializedRouteProposal3D proposal{};
};

enum class RouteLifecycleEventKind3D : std::uint8_t {
  kCompleted,
  kRawInvalidated,
  kLatestLidarInvalidated,
  kObjectiveSuperseded,
  kControlCandidateRejected,
  kCrossTrackExceeded,
  kTrackingTubeExceeded,
};

struct RouteLifecycleEvent3D {
  RouteLifecycleEventKind3D kind{RouteLifecycleEventKind3D::kCompleted};
  std::uint64_t generation{0U};
  std::uint64_t raw_producer_instance_id{0U};
  std::uint64_t raw_revision{0U};
  LatestLidarEvidenceId3D latest_lidar_evidence{};
};

enum class RouteProposalReplacementStatus3D : std::uint8_t {
  kReplace,
  kRetainEquivalentActiveSegment,
  kRejectIntentConflict,
};

struct RouteProposalReplacementObservation3D {
  double mission_target_tolerance_m{1.0e-6};
  bool safety_replan_requested{false};
  bool continuity_preserving_successor{false};
  bool materially_improved_point_to_point_successor{false};
};

struct RouteProposalReplacementAssessment3D {
  RouteProposalReplacementStatus3D status{RouteProposalReplacementStatus3D::kReplace};

  [[nodiscard]] bool replacementAllowed() const noexcept;
};

enum class RawRouteSuffixStatus3D : std::uint8_t {
  kValid,
  kInvalidRoute,
  kInvalidProjection,
  kOutsideFlightEnvelope,
  kRawCollision,
  kInvalidCollisionWorld,
};

struct RawRouteSuffixValidation3D {
  RawRouteSuffixStatus3D status{RawRouteSuffixStatus3D::kInvalidRoute};
  std::size_t first_validated_route_segment{0U};
  std::size_t failure_route_segment{0U};
  Point3 failure_point{};
  double validated_from_station_m{0.0};
  bool connector_validated{false};
  bool suffix_validated{false};

  [[nodiscard]] bool accepted() const noexcept;
};

struct RouteActivationObservation3D {
  NavigationWorldCertificate3D resident_world{};
  StaticRouteObjective current_objective{};
  std::uint64_t minimum_tracking_sample_sequence{0U};
  Point3 position{};
  double maximum_cross_track_m{0.0};
  const ObservedOccupancyGrid3D* latest_raw_occupancy{nullptr};
  std::uint64_t latest_raw_producer_instance_id{0U};
  std::uint64_t latest_raw_revision{0U};
  SweptFootprintConfig footprint{};
  const LaunchSupportContact3D* launch_support_contact{nullptr};
  const ProprioceptiveFreeSpaceSeed3D* proprioceptive_free_space_seed{nullptr};
  std::optional<FlightEnvelopeConfig> flight_envelope{};
  bool raw_validation_required{false};
};

struct RouteActivationAssessment3D {
  RoutePublicationAssessment3D publication{};
  RouteProjection3D projection{};
  RawRouteSuffixValidation3D raw_validation{};
  std::uint64_t raw_validated_through_revision{0U};
  bool objective_matches{false};
  bool cross_track_accepted{false};
  bool raw_world_compatible{false};

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
  kTrackingTubeViolation,
  kRawCollision,
};

struct RouteExecutionObservation3D {
  StaticRouteObjective current_objective{};
  std::uint64_t minimum_tracking_sample_sequence{0U};
  std::uint64_t previously_validated_through_raw_revision{0U};
  Point3 position{};
  double minimum_station_m{0.0};
  double maximum_station_m{std::numeric_limits<double>::infinity()};
  double maximum_cross_track_m{0.0};
  const ObservedOccupancyGrid3D* latest_raw_occupancy{nullptr};
  std::uint64_t latest_raw_producer_instance_id{0U};
  std::uint64_t latest_raw_revision{0U};
  SweptFootprintConfig footprint{};
  const LaunchSupportContact3D* launch_support_contact{nullptr};
  const ProprioceptiveFreeSpaceSeed3D* proprioceptive_free_space_seed{nullptr};
  std::optional<FlightEnvelopeConfig> flight_envelope{};
};

struct RouteExecutionAssessment3D {
  RouteExecutionStatus3D status{RouteExecutionStatus3D::kNoActiveRoute};
  RouteProjection3D projection{};
  RawRouteSuffixValidation3D raw_validation{};
  std::uint64_t validated_through_raw_revision{0U};

  [[nodiscard]] bool usable() const noexcept;
  [[nodiscard]] bool replacementRequired() const noexcept;
};

struct RouteSegmentCompletionConfig3D {
  double capture_radius_m{2.0};
  double terminal_station_tolerance_m{0.5};
};

struct RouteSegmentCompletionObservation3D {
  std::uint64_t route_generation{0U};
  Point3 position{};
  double minimum_station_m{0.0};
};

struct RouteSegmentCompletionAssessment3D {
  RouteProjection3D projection{};
  double endpoint_distance_m{0.0};
  double monotonic_station_m{0.0};
  bool generation_matches{false};
  bool terminal_station_reached{false};
  bool captured{false};
};

[[nodiscard]] std::optional<ActivatedRouteIdentity3D>
activateRouteProposal3D(const MaterializedRouteProposal3D& proposal,
                        std::uint64_t generation) noexcept;

[[nodiscard]] RouteProposalReplacementAssessment3D assessRouteProposalReplacement3D(
    const ActivatedRouteIdentity3D* active_route,
    const MaterializedRouteProposal3D& candidate,
    const RouteProposalReplacementObservation3D& observation) noexcept;

[[nodiscard]] RoutePublicationAssessment3D
assessRoutePublication3D(const MaterializedRouteProposal3D& proposal,
                         const NavigationWorldCertificate3D& resident_world) noexcept;

[[nodiscard]] RouteActivationAssessment3D
assessRouteActivation3D(const MaterializedRouteProposal3D& proposal,
                        std::span<const RouteSample3D> route,
                        const RouteActivationObservation3D& observation) noexcept;

[[nodiscard]] RawRouteSuffixValidation3D
validateRawRouteSuffix3D(std::span<const RouteSample3D> route, const Point3& position,
                         const RouteProjection3D& projection,
                         const OccupiedCollisionWorld3D& collision_world) noexcept;

[[nodiscard]] RouteExecutionAssessment3D
assessRouteExecution3D(const ActivatedRouteIdentity3D* active_route,
                       std::span<const RouteSample3D> route,
                       const RouteExecutionObservation3D& observation) noexcept;

[[nodiscard]] RouteSegmentCompletionAssessment3D
assessRouteSegmentCompletion3D(std::span<const RouteSample3D> route,
                               std::uint64_t expected_generation,
                               const RouteSegmentCompletionObservation3D& observation,
                               const RouteSegmentCompletionConfig3D& config) noexcept;

[[nodiscard]] std::string_view
routeLifecycleEventKind3DName(RouteLifecycleEventKind3D kind) noexcept;
[[nodiscard]] std::string_view
routePublicationStatus3DName(RoutePublicationStatus3D status) noexcept;
[[nodiscard]] std::string_view
rawRouteSuffixStatus3DName(RawRouteSuffixStatus3D status) noexcept;
[[nodiscard]] std::string_view
routeExecutionStatus3DName(RouteExecutionStatus3D status) noexcept;
} // namespace drone_city_nav
