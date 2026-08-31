#pragma once

#include "drone_city_nav/esdf_grid_3d.hpp"
#include "drone_city_nav/flight_envelope.hpp"
#include "drone_city_nav/route_3d.hpp"
#include "drone_city_nav/route_execution_contract_3d.hpp"
#include "drone_city_nav/route_progress_3d.hpp"
#include "drone_city_nav/stopping_capability.hpp"
#include "drone_city_nav/swept_footprint.hpp"
#include "drone_city_nav/types.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <span>
#include <string_view>
#include <vector>

namespace drone_city_nav {

struct StaticRouteExtensionConfig {
  double minimum_remaining_m{45.0};
  double required_certified_overlap_m{8.0};
  double latency_margin_s{0.5};
  double maximum_latency_s{8.0};
  double maximum_horizontal_acceleration_mps2{4.0};
  double maximum_vertical_acceleration_mps2{3.0};
  double maximum_control_jerk_mps3{12.0};
  StoppingCapability stopping_capability{};
  double minimum_retry_progress_m{15.0};
  double minimum_retry_interval_s{1.0};
  double minimum_endpoint_improvement_m{5.0};
  double protected_departure_m{5.0};
};

[[nodiscard]] bool
staticRouteExtensionConfigValid(const StaticRouteExtensionConfig& config) noexcept;

struct StaticRoutePlanningLatencyStats {
  std::size_t sample_count{0U};
  double planning_p95_ms{0.0};
  double planning_p99_ms{0.0};
  double build_and_planning_p99_ms{0.0};
};

class StaticRoutePlanningLatencyTracker final {
public:
  static constexpr std::size_t kMaximumSamples{128U};

  void record(double planning_latency_ms, double world_build_latency_ms) noexcept;
  [[nodiscard]] StaticRoutePlanningLatencyStats stats() const noexcept;
  void clear() noexcept;

private:
  std::array<double, kMaximumSamples> planning_latency_ms_{};
  std::array<double, kMaximumSamples> build_and_planning_latency_ms_{};
  std::size_t next_index_{0U};
  std::size_t sample_count_{0U};
};

[[nodiscard]] double jerkLimitedHorizontalStoppingDistanceM(
    double horizontal_speed_mps, double forward_acceleration_mps2,
    const StoppingCapability& capability, double maximum_horizontal_acceleration_mps2,
    double maximum_control_jerk_mps3) noexcept;

struct JerkLimitedStoppingDistance3D {
  double horizontal_m{0.0};
  double vertical_m{0.0};
  // Safe upper bound for travelled 3D station while both components stop.
  double route_station_m{0.0};

  [[nodiscard]] bool valid() const noexcept;
};

[[nodiscard]] JerkLimitedStoppingDistance3D jerkLimitedStoppingDistance3D(
    double horizontal_speed_mps, double forward_horizontal_acceleration_mps2,
    double vertical_speed_mps, double forward_vertical_acceleration_mps2,
    const StaticRouteExtensionConfig& config) noexcept;

struct StaticRouteObjective {
  Point3 goal{};
  std::uint64_t mission_epoch{0U};
  std::uint64_t sample_sequence{0U};
  std::uint64_t assignment_generation{0U};
  std::uint64_t target_detection_id{0U};
  std::uint64_t target_track_id{0U};
  bool continuous_tracking{false};
  bool available{false};
};

[[nodiscard]] bool
staticRouteAssignmentMatches(const StaticRouteObjective& first,
                             const StaticRouteObjective& second) noexcept;

struct StaticRouteSearchRetryConfig {
  double minimum_pose_change_m{2.0};
  double minimum_objective_change_m{5.0};
  double minimum_retry_interval_s{1.0};
};

struct StaticRouteSearchContext {
  std::uint64_t base_route_generation{0U};
  Point3 search_start{};
  StaticRouteObjective objective{};
  std::uint64_t minimum_tracking_sample_sequence{0U};
  std::int64_t stamp_ns{0};
};

enum class StaticRouteSearchRetryTrigger : std::uint8_t {
  kNoFailure,
  kRouteGenerationChanged,
  kObjectiveChanged,
  kPoseChanged,
  kRetryIntervalElapsed,
  kSuppressed,
};

struct StaticRouteSearchRetryDecision {
  bool allow{false};
  StaticRouteSearchRetryTrigger trigger{StaticRouteSearchRetryTrigger::kSuppressed};
  double pose_change_m{0.0};
  double objective_change_m{0.0};
  double elapsed_s{0.0};
};

class StaticRouteFailedSearchLatch final {
public:
  [[nodiscard]] StaticRouteSearchRetryDecision
  evaluate(const StaticRouteSearchRetryConfig& config,
           const StaticRouteSearchContext& context) const noexcept;
  void recordFailure(const StaticRouteSearchContext& context) noexcept;
  void clear() noexcept;
  [[nodiscard]] bool latched() const noexcept;

private:
  std::optional<StaticRouteSearchContext> failure_;
};

struct StaticRouteExtensionObservation {
  std::uint64_t route_generation{0U};
  double route_station_m{0.0};
  double route_remaining_m{0.0};
  double horizontal_speed_mps{0.0};
  double forward_acceleration_mps2{0.0};
  double vertical_speed_mps{0.0};
  double forward_vertical_acceleration_mps2{0.0};
  double planning_latency_p95_ms{0.0};
  double planning_latency_p99_ms{0.0};
  double build_and_planning_latency_p99_ms{0.0};
  bool route_reaches_mission_goal{false};
  bool next_planning_goal_inside_esdf{true};
  bool request_in_flight{false};
  std::uint64_t last_request_generation{0U};
  double last_request_station_m{0.0};
  std::int64_t request_stamp_ns{0};
  std::int64_t last_request_stamp_ns{0};
};

struct StaticRouteExtensionDecision {
  bool valid{false};
  bool request_extension{false};
  bool request_roi_refresh{false};
  double planning_p95_trigger_remaining_m{0.0};
  double extension_trigger_remaining_m{0.0};
  double roi_refresh_trigger_remaining_m{0.0};
  double braking_path_m{0.0};
  double horizontal_braking_path_m{0.0};
  double vertical_braking_path_m{0.0};
  double required_certified_overlap_m{0.0};
};

enum class CertifiedRouteReserveStatus3D : std::uint8_t {
  kSufficient,
  kTerminalExempt,
  kInsufficient,
  kInvalid,
};

struct CertifiedRouteReserveAssessment3D {
  CertifiedRouteReserveStatus3D status{CertifiedRouteReserveStatus3D::kInvalid};
  double available_m{0.0};
  double required_m{0.0};
  double shortfall_m{0.0};

  [[nodiscard]] bool accepted() const noexcept;
};

[[nodiscard]] CertifiedRouteReserveAssessment3D
assessCertifiedRouteReserve3D(const StaticRouteExtensionDecision& decision,
                              double available_route_m,
                              RouteEndpointSemantics3D endpoint_semantics) noexcept;

[[nodiscard]] std::string_view
certifiedRouteReserveStatus3DName(CertifiedRouteReserveStatus3D status) noexcept;

class StaticRouteReplanGate {
public:
  [[nodiscard]] bool tryBegin(std::uint64_t route_generation) noexcept;
  void finish(std::uint64_t route_generation) noexcept;
  [[nodiscard]] std::optional<std::uint64_t>
  finishIfSupersededBy(std::uint64_t resident_route_generation) noexcept;
  [[nodiscard]] bool inFlight() const noexcept;
  [[nodiscard]] std::uint64_t generation() const noexcept;

private:
  std::optional<std::uint64_t> generation_;
};

[[nodiscard]] std::uint64_t
staticRouteSearchGeneration(bool snapshot_owned_execution,
                            std::uint64_t prepared_route_generation,
                            std::uint64_t committed_route_generation) noexcept;

enum class StaticRouteSearchRequestKind : std::uint8_t {
  kInvalid,
  kInitial,
  kInitialRetry,
  kResidentRefresh,
  kExtension,
  kReplan,
};

struct StaticRouteSearchRequestIdentity {
  StaticRouteSearchRequestKind kind{StaticRouteSearchRequestKind::kInvalid};
  std::uint64_t base_route_generation{0U};

  [[nodiscard]] bool valid() const noexcept;
};

enum class StaticRouteSearchCurrencyStatus : std::uint8_t {
  kCurrent,
  kInvalidRequest,
  kSupersededByResidentRoute,
  kResidentRoutePredatesRequest,
};

struct StaticRouteSearchCurrencyAssessment {
  StaticRouteSearchCurrencyStatus status{
      StaticRouteSearchCurrencyStatus::kInvalidRequest};
  StaticRouteSearchRequestIdentity request{};
  std::uint64_t resident_route_generation{0U};

  [[nodiscard]] bool current() const noexcept;
};

[[nodiscard]] StaticRouteSearchRequestIdentity identifyStaticRouteSearchRequest(
    std::uint64_t world_route_generation, bool extension_request,
    std::uint64_t extension_base_generation, bool replan_request,
    std::uint64_t replan_base_generation) noexcept;

[[nodiscard]] StaticRouteSearchCurrencyAssessment
assessStaticRouteSearchCurrency(const StaticRouteSearchRequestIdentity& request,
                                std::uint64_t resident_route_generation) noexcept;

[[nodiscard]] bool
staticRouteSearchFailureLatchEligible(const StaticRouteSearchRequestIdentity& request,
                                      std::uint64_t resident_route_generation) noexcept;

[[nodiscard]] std::string_view
staticRouteSearchRequestKindName(StaticRouteSearchRequestKind kind) noexcept;

[[nodiscard]] std::string_view
staticRouteSearchCurrencyStatusName(StaticRouteSearchCurrencyStatus status) noexcept;

struct StaticRouteDeferredReplan {
  RouteReleaseReason3D reason{RouteReleaseReason3D::kNone};
  std::uint64_t route_generation{0U};
};

class StaticRouteDeferredReplanLatch final {
public:
  void defer(StaticRouteDeferredReplan request) noexcept;
  [[nodiscard]] std::optional<StaticRouteDeferredReplan>
  finishExtension(std::uint64_t route_generation, bool extension_activated) noexcept;
  [[nodiscard]] std::optional<StaticRouteDeferredReplan>
  finishReplan(std::uint64_t route_generation, bool route_activated) noexcept;
  [[nodiscard]] bool pending() const noexcept;

private:
  std::optional<StaticRouteDeferredReplan> request_;
};

enum class StaticRouteCandidateStatus : std::uint8_t {
  kAccepted,
  kEmpty,
  kInvalidInput,
  kRawWorldUnavailable,
  kRawCollision,
  kOutsideFlightEnvelope,
  kInvalidPassageSpan,
  kProtectedConstrainedSuffix,
  kInvalidCertifiedReserve,
  kInsufficientCertifiedReserve,
  kNoEndpointImprovement,
};

enum class StaticRouteReplacementPolicy : std::uint8_t {
  kRequireEndpointImprovement,
  kAllowSafetyReplan,
  kAllowSuccessorProgress,
};

enum class StaticRouteActivationStatus : std::uint8_t {
  kNotAttempted,
  kActivated,
  kCandidateNotExecutable,
  kCandidateValidationRejected,
  kWorldPublicationRejected,
  kActivationSnapshotSuperseded,
  kActivationCommitRejected,
  kStaleRouteGeneration,
  kStaleObjective,
  kInvalidExecutionGeometry,
  kDynamicHandoffRejected,
  kCertifiedSpliceRejected,
  kEquivalentActiveSegmentRetained,
  kInsufficientSuccessorImprovement,
  kCertifiedPending,
};

struct StaticRouteCandidateValidation {
  StaticRouteCandidateStatus status{StaticRouteCandidateStatus::kEmpty};
  double endpoint_improvement_m{0.0};
  std::size_t failure_segment_index{0U};
  Point3 failure_point{};
  bool accepted{false};
};

struct StaticRouteCandidate {
  std::uint64_t search_revision{0U};
  std::uint64_t base_route_generation{0U};
  std::uint64_t candidate_route_generation{0U};
  std::uint64_t fingerprint{0U};
  bool executable{false};
  bool reaches_mission_goal{false};
  StaticRouteCandidateValidation validation{};
  std::shared_ptr<const std::vector<RouteSample3D>> route;
  std::shared_ptr<const std::vector<ConstrainedRouteSpan>> constrained_spans;
};

[[nodiscard]] StaticRouteExtensionDecision evaluateStaticRouteExtension(
    const StaticRouteExtensionConfig& config,
    const StaticRouteExtensionObservation& observation) noexcept;

[[nodiscard]] bool
deferStaticRouteReleaseDuringExtension(bool request_in_flight,
                                       RouteReleaseReason3D reason) noexcept;

[[nodiscard]] Point3 staticRoutePlanningGoal(const Point3& start,
                                             const Point3& mission_goal,
                                             double planning_distance_m) noexcept;

[[nodiscard]] bool staticRoutePointInsideEsdf(const EsdfGrid3D& grid,
                                              const Point3& point,
                                              double margin_m = 0.0) noexcept;

[[nodiscard]] bool
staticRouteObjectiveMatches(const StaticRouteObjective& route_objective,
                            const StaticRouteObjective& current_objective,
                            std::uint64_t minimum_tracking_sample_sequence,
                            double maximum_tracking_goal_error_m) noexcept;

[[nodiscard]] bool staticRouteHasProtectedConstrainedSuffix(
    std::span<const RouteSample3D> route,
    std::span<const ConstrainedRouteSpan> constrained_spans,
    const Point3& current_position, double protected_departure_m) noexcept;

[[nodiscard]] bool
staticRouteReplacementProtected(std::span<const RouteSample3D> route,
                                std::span<const ConstrainedRouteSpan> constrained_spans,
                                const Point3& current_position,
                                const StaticRouteObjective& route_objective,
                                const StaticRouteObjective& search_objective,
                                double protected_departure_m) noexcept;

[[nodiscard]] StaticRouteCandidateValidation validateStaticRouteCandidate(
    std::span<const RouteSample3D> active_route,
    std::span<const RouteSample3D> candidate_route, const Point3& mission_goal,
    double minimum_endpoint_improvement_m, bool reaches_mission_goal,
    const FlightEnvelopeConfig& flight_envelope,
    StaticRouteReplacementPolicy replacement_policy =
        StaticRouteReplacementPolicy::kRequireEndpointImprovement) noexcept;

[[nodiscard]] std::string_view
staticRouteReplacementPolicyName(StaticRouteReplacementPolicy policy) noexcept;

[[nodiscard]] std::string_view
staticRouteCandidateStatusName(StaticRouteCandidateStatus status) noexcept;

[[nodiscard]] std::string_view
staticRouteActivationStatusName(StaticRouteActivationStatus status) noexcept;

[[nodiscard]] std::string_view
staticRouteSearchRetryTriggerName(StaticRouteSearchRetryTrigger trigger) noexcept;

} // namespace drone_city_nav
