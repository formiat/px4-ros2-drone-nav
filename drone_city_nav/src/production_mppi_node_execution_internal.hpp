#pragma once

#include <builtin_interfaces/msg/time.hpp>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <span>

#include "production_mppi_node.hpp"

namespace drone_city_nav {

enum class ProductionMppiHorizonCommitKind : std::uint8_t {
  kPublishSnapshotTransition,
  kConfirmSnapshotUnchanged,
  kCommitPendingSnapshotTransition,
};

enum class ProductionMppiHorizonCommitStatus : std::uint8_t {
  kPublished,
  kDeferredResidentOwner,
  kRejected,
};

struct ProductionMppiHorizonCommit {
  ProductionMppiHorizonCommitKind kind{
      ProductionMppiHorizonCommitKind::kConfirmSnapshotUnchanged};
  std::shared_ptr<const CommittedExecutionAuthority3D> expected_authority;
  std::shared_ptr<const ExecutionPlan3D> expected_snapshot;
  std::shared_ptr<const ExecutionPlan3D> certification_snapshot;
  std::shared_ptr<const ExecutionRouteTransitionResult3D> progress_preparation;
  const ExecutionRouteTransitionResult3D* transition{nullptr};
  std::shared_ptr<const PendingCertifiedRoute3D> expected_pending;
  bool latest_evidence_revalidated{false};
};

struct ProductionMppiExecutionCycle {
  const mppi::MppiTickInput& input;
  const mppi::MppiTickResult& result;
  const WorldSnapshot3D& world;
  const ProductionRouteExecutionSelection3D& route_execution;
  const std::shared_ptr<const ProductionNavigationObjective>& objective;
  const std::shared_ptr<const VersionedExecutionInput3D>& execution_input;
  const std::shared_ptr<const VersionedLatestLidarEvidence3D>& latest_lidar_evidence;
  OffboardSessionAdmissionState offboard_session{};
  std::int64_t offboard_session_receive_stamp_ns{0};
  ProductionMppiPlanningState planning_state;
  std::int64_t now_ns;
  ProductionMppiExecutionPublication& publication;
  const mppi::State& exact_initial_state;
  const mppi::Control& exact_previous_control;
  std::uint64_t target_offboard_instance_id;
  std::int64_t lidar_validation_now_ns;
  const Point3& mission_goal;
  bool direct_tracking_requested;
  const CertifiedRouteSuffix3D* selected_snapshot_route;
  const std::shared_ptr<const VersionedObservedRawWorld3D>& direct_observed_world;
  const std::shared_ptr<const VersionedStaticWorld3D>& direct_static_world;
  std::shared_ptr<const VersionedExecutionValidationPolicy3D> selected_policy;
  double latest_lidar_obstacle_age_ms;
  bool latest_lidar_obstacle_fresh;
  bool latest_lidar_obstacle_receive_time_fallback;
  std::span<const Point3> latest_lidar_obstacle_points;
  std::uint64_t latest_lidar_obstacle_sequence;
  bool exact_snapshot_world;
  bool publication_route_constrained;
  const std::optional<mppi::FiniteExecutionPathTerminalBoundary>&
      route_terminal_boundary;
  const FlightEnvelopeConfig* execution_flight_envelope;
  const mppi::DynamicsConfig* execution_dynamics;
  const mppi::AltitudeEnvelopeConfig* execution_altitude_envelope;
  const SweptFootprintConfig* execution_footprint;
  const mppi::FiniteExecutionPathWorld& execution_path_world;
  std::size_t arrival_search_step_controls;
  std::int64_t finite_path_control_interval_ns;
  std::uint64_t latest_obstacle_revision;
};

namespace production_mppi_execution_detail {

[[nodiscard]] builtin_interfaces::msg::Time
timeFromNanoseconds(std::int64_t nanoseconds);

[[nodiscard]] std::int64_t
timeToNanoseconds(const builtin_interfaces::msg::Time& time) noexcept;

[[nodiscard]] std::optional<std::int64_t>
canonicalHorizonEndTime(std::int64_t valid_from_ns, std::int64_t duration_ns) noexcept;

[[nodiscard]] bool sameState(const mppi::State& first,
                             const mppi::State& second) noexcept;

[[nodiscard]] bool sameControl(const mppi::Control& first,
                               const mppi::Control& second) noexcept;

[[nodiscard]] LatestLidarEvidenceFreshness3D latestLidarEvidenceFreshness(
    const std::shared_ptr<const VersionedLatestLidarEvidence3D>& evidence,
    std::int64_t now_ns, double maximum_age_ms) noexcept;

[[nodiscard]] bool bindHorizonRouteMetadata(msg::MppiTrajectoryHorizon& horizon,
                                            const CertifiedRouteSuffix3D& route);

void appendStationaryHoldPoint(msg::MppiTrajectoryHorizon& horizon,
                               const Point3& hold_position,
                               std::int64_t time_from_start_ns, float yaw_rad);

[[nodiscard]] bool appendFiniteExecutionPoints(
    msg::MppiTrajectoryHorizon& horizon, std::span<const mppi::State> states,
    std::span<const mppi::Control> controls,
    const mppi::Control& previous_applied_control, std::int64_t control_interval_ns);

} // namespace production_mppi_execution_detail

} // namespace drone_city_nav
