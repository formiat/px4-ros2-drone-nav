#pragma once

#include "drone_city_nav/execution_route_store_3d.hpp"
#include "drone_city_nav/flight_envelope.hpp"
#include "drone_city_nav/mppi/finite_execution_path.hpp"
#include "drone_city_nav/mppi/mppi_finite_horizon.hpp"
#include "drone_city_nav/offboard_session_admission.hpp"
#include "drone_city_nav/swept_footprint.hpp"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <span>
#include <string_view>

#include "production_mppi_execution_control.hpp"
#include "production_mppi_node_execution_types.hpp"
#include "production_mppi_node_types.hpp"

namespace drone_city_nav {

struct EvidenceSnapshot3D {
  std::shared_ptr<const ProductionNavigationObjective> objective;
  std::shared_ptr<const VersionedExecutionInput3D> execution_input;
  std::shared_ptr<const VersionedLatestLidarEvidence3D> latest_lidar_evidence;
  OffboardSessionAdmissionState offboard_session{};
  std::int64_t offboard_session_receive_stamp_ns{0};
  mppi::State exact_initial_state{};
  mppi::Control exact_previous_control{};
  std::uint64_t target_offboard_instance_id{0U};
  std::int64_t lidar_validation_now_ns{0};
  std::shared_ptr<const VersionedObservedRawWorld3D> direct_observed_world;
  std::shared_ptr<const VersionedStaticWorld3D> direct_static_world;
  std::shared_ptr<const VersionedExecutionValidationPolicy3D> selected_policy;
  double latest_lidar_obstacle_age_ms{-1.0};
  bool latest_lidar_obstacle_fresh{false};
  bool latest_lidar_obstacle_receive_time_fallback{false};
  std::span<const Point3> latest_lidar_obstacle_points;
  std::uint64_t latest_lidar_obstacle_sequence{0U};
  bool exact_snapshot_world{false};
  const FlightEnvelopeConfig* execution_flight_envelope{nullptr};
  const mppi::DynamicsConfig* execution_dynamics{nullptr};
  const mppi::AltitudeEnvelopeConfig* execution_altitude_envelope{nullptr};
  const SweptFootprintConfig* execution_footprint{nullptr};
  mppi::FiniteExecutionPathWorld execution_path_world{};
};

struct RouteDecision3D {
  ProductionRouteExecutionSelection3D execution{};
  Point3 mission_goal{};
  const CertifiedRouteSuffix3D* selected_snapshot_route{nullptr};
  ProductionMppiPlanningState planning_state{ProductionMppiPlanningState::kPlanned};
  bool direct_tracking_requested{false};
  bool publication_route_constrained{false};
};

struct ControllerCycle3D {
  const mppi::MppiTickInput* input{nullptr};
  const mppi::MppiTickResult* result{nullptr};
  const WorldSnapshot3D* world{nullptr};
  std::int64_t now_ns{0};
  std::size_t arrival_search_step_controls{0U};
  std::int64_t finite_path_control_interval_ns{0};
  std::uint64_t latest_obstacle_revision{0U};

  [[nodiscard]] const mppi::MppiTickInput& inputRef() const noexcept {
    return *input;
  }

  [[nodiscard]] const mppi::MppiTickResult& resultRef() const noexcept {
    return *result;
  }

  [[nodiscard]] const WorldSnapshot3D& worldRef() const noexcept {
    return *world;
  }
};

struct ProductionMppiExecutionCycle {
  EvidenceSnapshot3D evidence{};
  RouteDecision3D route{};
  ControllerCycle3D controller{};
  ProductionMppiExecutionPublication* publication{nullptr};

  [[nodiscard]] bool validForAssembly() const noexcept {
    return controller.input != nullptr && controller.result != nullptr &&
           controller.world != nullptr;
  }

  [[nodiscard]] bool valid() const noexcept {
    return validForAssembly() && publication != nullptr;
  }

  [[nodiscard]] ProductionMppiExecutionPublication& publicationRef() const noexcept {
    return *publication;
  }
};

enum class HorizonCandidateStatus3D : std::uint8_t {
  kPlanned,
  kExplicitHold,
  kNoExecutableRoute,
  kMissingExactEvidence,
  kInvalidControllerOutput,
  kStaleExecutionSnapshot,
  kFinitePathRejected,
  kCertificationRejected,
  kTransitionRejected,
};

[[nodiscard]] std::string_view
horizonCandidateStatus3DName(HorizonCandidateStatus3D status) noexcept;

enum class HorizonCandidateObstacleSource3D : std::uint8_t {
  kNone,
  kPersistentRaw,
  kLatestLidar,
};

struct HorizonCandidatePhysicalRejection3D {
  std::shared_ptr<const VersionedObservedRawWorld3D> observed_raw_world;
  std::uint64_t route_generation{0U};
  HorizonCandidateObstacleSource3D source{HorizonCandidateObstacleSource3D::kNone};
};

struct HorizonCandidate3D {
  HorizonCandidateStatus3D status{HorizonCandidateStatus3D::kMissingExactEvidence};
  ProductionMppiExecutionReason failure_reason{
      ProductionMppiExecutionReason::kNoExecutableHorizon};
  Point3 explicit_hold_position{};
  ProductionMppiExecutionReason explicit_hold_reason{
      ProductionMppiExecutionReason::kNone};
  std::shared_ptr<const ExecutionPlan3D> committed_snapshot;
  std::optional<ExecutionRouteTransitionResult3D> transition;
  // Status and detail of a rejected plan transition, for diagnostics.
  ExecutionRouteTransitionStatus3D transition_status{
      ExecutionRouteTransitionStatus3D::kApplied};
  ExecutionRouteTransitionDetail3D transition_detail{
      ExecutionRouteTransitionDetail3D::kNone};
  std::optional<HorizonCandidatePhysicalRejection3D> physical_rejection;
  std::size_t arrival_shaping_attempts{0U};
  mppi::FiniteExecutionPathStatus validation_status{
      mppi::FiniteExecutionPathStatus::kInvalidContract};
  mppi::FiniteExecutionPathStatus first_failed_validation_status{
      mppi::FiniteExecutionPathStatus::kValid};
  const char* finite_path_rejected_precondition{"none"};
  // Segment at which the finite path validation failed, for diagnostics.
  std::size_t validation_failure_segment{0U};
  std::size_t validation_first_remaining_point{0U};
  // Where the first (longest) rejected candidate failed, for diagnostics.
  std::size_t first_failed_validation_segment{0U};
  Point3 first_failed_validation_point{};
  // Route certification verdict of the last candidate, for diagnostics.
  FiniteExecutionCertificationStatus3D certification_status{
      FiniteExecutionCertificationStatus3D::kInvalidInput};
  FiniteExecutionCertificationStatus3D braking_tail_certification_status{
      FiniteExecutionCertificationStatus3D::kInvalidInput};
  FiniteExecutionRouteAdherenceStatus3D route_adherence_status{
      FiniteExecutionRouteAdherenceStatus3D::kNotEvaluated};
  double route_adherence_failure_distance_m{-1.0};
  bool nominal_candidate_degraded{false};
  bool path_validation_backoff{false};
  bool latest_lidar_path_validation_backoff{false};

  [[nodiscard]] bool planned() const noexcept {
    return status == HorizonCandidateStatus3D::kPlanned &&
           committed_snapshot != nullptr && transition.has_value();
  }
};

struct ExecutionHorizonAssemblerConfig3D {
  FlightEnvelopeConfig flight_envelope{};
  mppi::FiniteHorizonConfig finite_horizon{};
  std::shared_ptr<const VersionedExecutionValidationPolicy3D>
      direct_tracking_validation_policy;
};

// Produces an immutable execution-plan candidate without ROS dependencies.
// Serialization and the sole atomic ExecutionSupervisor3D commit remain in the
// node adapter.
class ExecutionHorizonAssembler3D final {
public:
  explicit ExecutionHorizonAssembler3D(ExecutionHorizonAssemblerConfig3D config);

  [[nodiscard]] HorizonCandidate3D
  assemble(const ProductionMppiExecutionCycle& cycle,
           const std::shared_ptr<const ExecutionPlan3D>& resident_plan) const;

private:
  ExecutionHorizonAssemblerConfig3D config_{};
};

} // namespace drone_city_nav
