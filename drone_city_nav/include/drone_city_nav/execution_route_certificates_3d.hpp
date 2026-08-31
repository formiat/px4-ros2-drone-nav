#pragma once

#include "drone_city_nav/compiled_trajectory_3d.hpp"
#include "drone_city_nav/execution_route_model_3d.hpp"
#include "drone_city_nav/finite_execution_path_3d.hpp"
#include "drone_city_nav/route_decorations_3d.hpp"
#include "drone_city_nav/route_execution_contract_3d.hpp"

#include <cstdint>
#include <memory>
#include <optional>
#include <string_view>
#include <variant>

namespace drone_city_nav {

struct ExecutionPlan3D;

struct StaticRouteCertificate3D {
  RouteInstanceId3D route_instance_id{};
  std::uint64_t route_generation{0U};
  std::uint64_t geometry_revision{0U};
  std::uint64_t physical_route_fingerprint{0U};
  std::uint64_t static_occupancy_content_fingerprint{0U};
  std::uint64_t validation_policy_fingerprint{0U};
  std::uint64_t execution_validation_policy_fingerprint{0U};
  std::uint64_t route_decorations_revision{0U};
  std::uint64_t passage_volume_config_fingerprint{0U};
  std::uint64_t geometry_derivation_occupancy_content_fingerprint{0U};
  NavigationWorldCertificate3D world_certificate{};
  double suffix_start_station_m{0.0};
  double certified_end_station_m{0.0};

  [[nodiscard]] bool validFor(RouteInstanceId3D expected_route_instance_id,
                              const ActivatedRouteIdentity3D& identity,
                              std::uint64_t expected_geometry_revision,
                              std::uint64_t expected_physical_route_fingerprint,
                              double route_end_station_m) const noexcept;
};

struct ObservedRawRouteCertificate3D {
  RouteInstanceId3D route_instance_id{};
  std::uint64_t route_generation{0U};
  std::uint64_t geometry_revision{0U};
  std::uint64_t physical_route_fingerprint{0U};
  std::uint64_t producer_instance_id{0U};
  std::uint64_t validated_through_revision{0U};
  std::uint64_t validation_policy_fingerprint{0U};
  std::uint64_t execution_validation_policy_fingerprint{0U};
  std::uint64_t observed_world_content_fingerprint{0U};
  std::uint64_t route_decorations_revision{0U};
  std::uint64_t passage_volume_config_fingerprint{0U};
  std::uint64_t geometry_derivation_occupancy_content_fingerprint{0U};
  double suffix_start_station_m{0.0};
  double certified_end_station_m{0.0};

  [[nodiscard]] bool validFor(RouteInstanceId3D expected_route_instance_id,
                              const ActivatedRouteIdentity3D& identity,
                              std::uint64_t expected_geometry_revision,
                              std::uint64_t expected_physical_route_fingerprint,
                              double route_end_station_m) const noexcept;
};

using RouteSuffixCertificate3D =
    std::variant<StaticRouteCertificate3D, ObservedRawRouteCertificate3D>;

struct CertifiedRouteSuffix3D {
  RouteInstanceId3D route_instance_id{};
  RouteOwnerIdentity3D owner{};
  std::optional<RouteInstanceId3D> parent_route_instance_id;
  ActivatedRouteIdentity3D identity{};
  std::shared_ptr<const CompiledTrajectory3D> geometry;
  std::shared_ptr<const RouteDecorations3D> decorations;
  RouteSuffixCertificate3D certificate{StaticRouteCertificate3D{}};
  CertifiedRouteProgress3D progress{};
  RouteContinuityLineage3D continuity_lineage{};
  std::uint64_t continuity_id{0U};
  std::shared_ptr<const VersionedObservedRawWorld3D> observed_raw_world;
  std::shared_ptr<const VersionedStaticWorld3D> static_world;
  std::shared_ptr<const VersionedExecutionValidationPolicy3D> validation_policy;
  RouteEndpointSemantics3D planned_endpoint_semantics{
      RouteEndpointSemantics3D::kContinuation};

  [[nodiscard]] bool valid() const noexcept;
  [[nodiscard]] double endStationM() const noexcept;
  [[nodiscard]] double remainingM() const noexcept;
};

enum class FiniteExecutionKind3D : std::uint8_t {
  kNominal,
  kRetained,
  kEmergencyBrakeTail,
};

struct FiniteRouteTerminalBoundary3D {
  Point3 endpoint{};
  Vec3 forward{};
  double tolerance_m{0.5};
  double activation_distance_m{0.0};
  double maximum_cross_track_m{0.0};
  double initial_route_station_m{0.0};
  double activation_route_station_m{0.0};

  [[nodiscard]] bool valid(double route_end_station_m) const noexcept;
};

struct CertifiedStopBoundary3D {
  RouteInstanceId3D route_instance_id{};
  std::uint64_t route_generation{0U};
  std::uint64_t geometry_revision{0U};
  std::uint64_t physical_route_fingerprint{0U};
  std::uint64_t trajectory_revision{0U};
  std::uint64_t raw_validated_through_revision{0U};
  double station_m{0.0};
  double position_tolerance_m{0.25};
  Point3 position{};
};

struct StaticFiniteExecutionValidationLineage3D {
  NavigationWorldCertificate3D world_certificate{};
  std::uint64_t static_occupancy_content_fingerprint{0U};
  std::uint64_t validation_policy_fingerprint{0U};
};

struct ObservedRawFiniteExecutionValidationLineage3D {
  std::uint64_t producer_instance_id{0U};
  std::uint64_t validated_through_raw_revision{0U};
  std::uint64_t validation_policy_fingerprint{0U};
  std::uint64_t observed_world_content_fingerprint{0U};
};

using FiniteExecutionValidationLineage3D =
    std::variant<StaticFiniteExecutionValidationLineage3D,
                 ObservedRawFiniteExecutionValidationLineage3D>;

struct FiniteExecutionValidationProof3D {
  std::uint64_t artifact_fingerprint{0U};
  std::uint64_t validation_contract_fingerprint{0U};
  FiniteExecutionValidationLineage3D lineage{
      StaticFiniteExecutionValidationLineage3D{}};
};

struct FiniteExecutionState3D {
  std::uint64_t trajectory_revision{0U};
  std::uint64_t source_snapshot_version{0U};
  std::uint64_t source_navigation_revision{0U};
  RouteInstanceId3D source_route_instance_id{};
  std::uint64_t source_route_generation{0U};
  std::uint64_t source_geometry_revision{0U};
  std::uint64_t source_physical_route_fingerprint{0U};
  RouteSuffixCertificate3D certificate{StaticRouteCertificate3D{}};
  std::shared_ptr<const FiniteMotionHorizon3D> horizon;
  std::shared_ptr<const VersionedObservedRawWorld3D> observed_raw_world;
  std::shared_ptr<const VersionedStaticWorld3D> static_world;
  std::shared_ptr<const VersionedExecutionValidationPolicy3D> validation_policy;
  std::shared_ptr<const VersionedExecutionInput3D> execution_input;
  std::shared_ptr<const VersionedLatestLidarEvidence3D> latest_lidar_evidence;
  std::optional<FiniteRouteTerminalBoundary3D> terminal_boundary;
  CertifiedStopBoundary3D stop_boundary{};
  double begin_route_station_m{0.0};
  std::int64_t valid_from_ns{0};
  std::int64_t valid_until_ns{0};
  std::int64_t control_interval_ns{0};
  FiniteExecutionKind3D kind{FiniteExecutionKind3D::kNominal};
  FiniteExecutionValidationProof3D validation_proof{};
  bool revalidation_required{false};

  [[nodiscard]] bool validFor(const CertifiedRouteSuffix3D* route) const noexcept;
};

// Raw-world and acquisition-aligned lidar changes invalidate only an occupied
// intersection with the still-active part of an already-published finite path.
[[nodiscard]] FiniteExecutionPathValidation3D
validateRemainingFiniteExecutionAgainstObservedWorld3D(
    const FiniteExecutionState3D& execution,
    const VersionedExecutionInput3D& current_input,
    const VersionedObservedRawWorld3D& current_world,
    std::int64_t validation_stamp_ns) noexcept;

[[nodiscard]] FiniteExecutionPathValidation3D
validateRemainingFiniteExecutionAgainstLatestLidar3D(
    const FiniteExecutionState3D& execution,
    const VersionedExecutionInput3D& current_input,
    const VersionedLatestLidarEvidence3D& current_lidar,
    std::int64_t validation_stamp_ns) noexcept;

struct FiniteExecutionPlan3D {
  FiniteExecutionState3D command_horizon{};
  FiniteExecutionState3D braking_tail{};

  [[nodiscard]] bool validFor(const CertifiedRouteSuffix3D& route) const noexcept;
};

[[nodiscard]] std::string_view
finiteExecutionKind3DName(FiniteExecutionKind3D kind) noexcept;

} // namespace drone_city_nav
