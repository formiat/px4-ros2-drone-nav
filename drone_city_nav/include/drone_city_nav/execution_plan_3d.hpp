#pragma once

#include "drone_city_nav/execution_route_certificates_3d.hpp"

#include <cstdint>
#include <memory>
#include <string_view>
#include <variant>

namespace drone_city_nav {

// A stop is the vehicle's own physics rather than a route action: a finite
// braking trajectory that starts at the exact current state and ends at rest.
// It carries no route, no adherence corridor and no route certificate, so it
// stays available in exactly the situations where the route machinery can
// produce nothing. It is finite by construction: once flown, the plan rests
// and the next certified route activates from where the vehicle stands, so it
// never becomes a state that withholds movement.
struct StopExecution3D {
  std::uint64_t trajectory_revision{0U};
  std::uint64_t source_snapshot_version{0U};
  std::uint64_t source_navigation_revision{0U};
  Point3 rest_position{};
  std::shared_ptr<const FiniteMotionHorizon3D> horizon;
  std::shared_ptr<const VersionedObservedRawWorld3D> observed_raw_world;
  std::shared_ptr<const VersionedStaticWorld3D> static_world;
  std::shared_ptr<const VersionedExecutionValidationPolicy3D> validation_policy;
  std::shared_ptr<const VersionedExecutionInput3D> execution_input;
  std::shared_ptr<const VersionedLatestSensorEvidence3D> latest_sensor_evidence;
  std::int64_t valid_from_ns{0};
  std::int64_t valid_until_ns{0};
  std::int64_t control_interval_ns{0};
  FiniteExecutionValidationProof3D validation_proof{};
  // The swept body this stop was certified with and stays executable against:
  // the policy's clearance envelope, or the physical body alone when the
  // envelope could not clear the evidence the vehicle must brake through.
  SweptFootprintConfig validation_footprint{};
  // How much of the policy envelope's clearance this stop gave up to sweep
  // clear; 0 is the envelope, 1 the physical body.
  double clearance_reduction{0.0};

  [[nodiscard]] bool valid() const noexcept;
};

inline constexpr double kStationaryExecutionHoldPositionToleranceM{0.25};
inline constexpr double kStationaryExecutionHoldSpeedToleranceMps{0.25};
inline constexpr double kStationaryExecutionHoldYawRateToleranceRadps{0.25};
// A vehicle at rest is not accelerating: measured on PX4's local position in
// the urban flights r268 to r270, the acceleration at a sustained rest is
// 0.1 to 0.4 m/s^2 at p99 (0.42 at most), while at the turning point of a
// braking overshoot, where the speed passes through zero, it is 2.6 to
// 7.9 m/s^2 (r281). A rest read from the speed alone anchored a hold at
// that turning point, 0.3 m inside a wall by the body model, and the hold
// pulled the vehicle back into the wall.
inline constexpr double kStationaryExecutionHoldAccelerationToleranceMps2{1.0};

enum class StationaryExecutionHoldOrigin3D : std::uint8_t {
  kTerminalExecution,
  kStationaryCaptureRearm,
};

struct StationaryExecutionHold3D {
  std::uint64_t hold_id{0U};
  std::uint64_t source_trajectory_revision{0U};
  StationaryExecutionHoldOrigin3D origin{
      StationaryExecutionHoldOrigin3D::kTerminalExecution};
  Point3 position{};
  std::shared_ptr<const VersionedExecutionInput3D> terminal_execution_input;
  std::shared_ptr<const VersionedObservedRawWorld3D> observed_raw_world;
  std::shared_ptr<const VersionedStaticWorld3D> static_world;
  std::shared_ptr<const VersionedExecutionValidationPolicy3D> validation_policy;
  std::shared_ptr<const VersionedLatestSensorEvidence3D> latest_sensor_evidence;

  [[nodiscard]] bool valid() const noexcept;
};

enum class ExecutionRoutePhase3D : std::uint8_t {
  kFollowing,
  kAwaitingSuccessor,
  kStopping,
  kStopped,
  kRevoked,
};

struct FollowingPlan3D {
  CertifiedRouteSuffix3D route{};
  FiniteExecutionPlan3D execution{};
};

struct StopPlan3D {
  StopExecution3D execution{};
};

struct CertifiedTerminalHoldPlan3D {
  CertifiedRouteSuffix3D route{};
  FiniteExecutionPlan3D execution{};
};

struct StationaryHoldPlan3D {
  std::variant<CertifiedTerminalHoldPlan3D, StationaryExecutionHold3D> owner{
      CertifiedTerminalHoldPlan3D{}};
};

struct EmptyAwaitingSuccessorPlan3D final {};

struct SuspendedRoutePlan3D {
  CertifiedRouteSuffix3D route{};
};

struct ContinuationStopPlan3D {
  CertifiedRouteSuffix3D route{};
  FiniteExecutionPlan3D execution{};
};

struct AwaitingSuccessorPlan3D {
  std::variant<EmptyAwaitingSuccessorPlan3D, SuspendedRoutePlan3D,
               ContinuationStopPlan3D>
      owner{EmptyAwaitingSuccessorPlan3D{}};
};

struct RevokedPlan3D final {};

using ExecutionPlanState3D =
    std::variant<FollowingPlan3D, StopPlan3D, StationaryHoldPlan3D,
                 AwaitingSuccessorPlan3D, RevokedPlan3D>;

struct ExecutionPlan3D {
  std::uint64_t version{0U};
  ExecutionPlanState3D state{AwaitingSuccessorPlan3D{}};
  std::uint64_t execution_owner_epoch{0U};
  std::uint64_t route_generation_high_water{0U};

  [[nodiscard]] bool valid() const noexcept;
  [[nodiscard]] bool publishable() const noexcept;
  [[nodiscard]] std::uint64_t routeGenerationHighWater() const noexcept;
  [[nodiscard]] ExecutionRoutePhase3D phase() const noexcept;
  [[nodiscard]] const CertifiedRouteSuffix3D* route() const noexcept;
  [[nodiscard]] const FiniteExecutionState3D* finiteExecution() const noexcept;
  [[nodiscard]] const FiniteExecutionState3D* brakingFallback() const noexcept;
  [[nodiscard]] const StopExecution3D* stopExecution() const noexcept;
  [[nodiscard]] const StationaryExecutionHold3D* stationaryHold() const noexcept;
  // The highest trajectory revision the resident owner carries: a finite or
  // stop execution's own revision, or the revision a stationary hold took the
  // vehicle over from. Every trajectory that
  // succeeds the owner must carry a higher one, so this is where a successor
  // derives its revision from. Zero when no owner carries one.
  [[nodiscard]] std::uint64_t ownerTrajectoryRevision() const noexcept;
};

[[nodiscard]] bool
executionRouteAcceptsCertifiedReplacement3D(const ExecutionPlan3D& snapshot) noexcept;

[[nodiscard]] std::shared_ptr<const ExecutionPlan3D>
makeInitialExecutionRouteSnapshot3D();

[[nodiscard]] RouteEndpointSemantics3D
executionRouteEndpointSemantics3D(const ExecutionPlan3D& snapshot) noexcept;

[[nodiscard]] std::string_view
executionRoutePhase3DName(ExecutionRoutePhase3D phase) noexcept;

} // namespace drone_city_nav
