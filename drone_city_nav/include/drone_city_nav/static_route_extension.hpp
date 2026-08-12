#pragma once

#include "drone_city_nav/active_global_guide.hpp"
#include "drone_city_nav/flight_envelope.hpp"
#include "drone_city_nav/mppi/mppi_types.hpp"
#include "drone_city_nav/route_3d.hpp"
#include "drone_city_nav/swept_footprint.hpp"
#include "drone_city_nav/types.hpp"

#include <atomic>
#include <cstdint>
#include <memory>
#include <optional>
#include <span>
#include <string_view>
#include <vector>

namespace drone_city_nav {

struct StaticRouteExtensionConfig {
  double minimum_remaining_m{45.0};
  double latency_margin_s{0.5};
  double maximum_latency_s{8.0};
  double minimum_retry_progress_m{15.0};
  double minimum_retry_interval_s{1.0};
  double minimum_endpoint_improvement_m{5.0};
  double protected_departure_m{5.0};
};

struct StaticRouteObjective {
  Point3 goal{};
  std::uint64_t mission_epoch{0U};
  std::uint64_t sample_sequence{0U};
  bool continuous_tracking{false};
  bool available{false};
};

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
  double guide_search_latency_ms{0.0};
  double esdf_build_latency_ms{0.0};
  bool route_reaches_mission_goal{false};
  bool next_planning_goal_inside_esdf{true};
  bool request_in_flight{false};
  std::uint64_t last_request_generation{0U};
  double last_request_station_m{0.0};
  std::int64_t request_stamp_ns{0};
  std::int64_t last_request_stamp_ns{0};
};

struct StaticRouteExtensionDecision {
  bool request_extension{false};
  bool request_roi_refresh{false};
  double extension_trigger_remaining_m{0.0};
  double roi_refresh_trigger_remaining_m{0.0};
};

class StaticRouteReplanGate {
public:
  [[nodiscard]] bool tryBegin(std::uint64_t route_generation) noexcept;
  void finish(std::uint64_t route_generation) noexcept;
  [[nodiscard]] bool inFlight() const noexcept;
  [[nodiscard]] std::uint64_t generation() const noexcept;

private:
  std::optional<std::uint64_t> generation_;
};

struct StaticRouteDeferredReplan {
  GlobalGuideReleaseReason reason{GlobalGuideReleaseReason::kNone};
  std::uint64_t route_generation{0U};
};

class StaticRouteDeferredReplanLatch final {
public:
  void defer(StaticRouteDeferredReplan request) noexcept;
  [[nodiscard]] std::optional<StaticRouteDeferredReplan>
  finishExtension(std::uint64_t route_generation, bool extension_activated) noexcept;
  [[nodiscard]] bool pending() const noexcept;

private:
  std::optional<StaticRouteDeferredReplan> request_;
};

struct StaticRouteRoiRefreshRequest {
  std::uint64_t sequence{0U};
  std::uint64_t base_route_generation{0U};
  enum class Purpose : std::uint8_t {
    kRouteExtension,
    kTrackingObjective,
  } purpose{Purpose::kRouteExtension};
};

class StaticRouteRoiRefreshLifecycle final {
public:
  [[nodiscard]] StaticRouteRoiRefreshRequest
  queue(std::uint64_t base_route_generation,
        StaticRouteRoiRefreshRequest::Purpose purpose =
            StaticRouteRoiRefreshRequest::Purpose::kRouteExtension) noexcept;
  [[nodiscard]] StaticRouteRoiRefreshRequest latest() const noexcept;
  [[nodiscard]] bool
  pending(const StaticRouteRoiRefreshRequest& request) const noexcept;
  void complete(std::uint64_t sequence) noexcept;

private:
  std::atomic<std::uint64_t> next_sequence_{0U};
  std::atomic<std::uint64_t> requested_sequence_{0U};
  std::atomic<std::uint64_t> requested_base_route_generation_{0U};
  std::atomic<StaticRouteRoiRefreshRequest::Purpose> requested_purpose_{
      StaticRouteRoiRefreshRequest::Purpose::kRouteExtension};
  std::atomic<std::uint64_t> completed_sequence_{0U};
};

enum class StaticRouteCandidateStatus : std::uint8_t {
  kAccepted,
  kEmpty,
  kOutsideEsdf,
  kInvalidEsdf,
  kRawCollision,
  kOutsideFlightEnvelope,
  kInvalidChannelSpan,
  kProtectedConstrainedSuffix,
  kNoEndpointImprovement,
};

enum class StaticRouteReplacementPolicy : std::uint8_t {
  kRequireEndpointImprovement,
  kAllowSafetyReplan,
};

enum class StaticRouteActivationStatus : std::uint8_t {
  kNotAttempted,
  kActivated,
  kCandidateNotExecutable,
  kCandidateValidationRejected,
  kStaleWorldRevision,
  kStaleRouteGeneration,
  kStaleObjective,
};

struct StaticRouteCandidateValidation {
  StaticRouteCandidateStatus status{StaticRouteCandidateStatus::kEmpty};
  double endpoint_improvement_m{0.0};
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
                                       GlobalGuideReleaseReason reason) noexcept;

[[nodiscard]] Point3 staticRoutePlanningGoal(const Point3& start,
                                             const Point3& mission_goal,
                                             double planning_distance_m) noexcept;

[[nodiscard]] bool staticRoutePointInsideEsdf(const mppi::EsdfGrid& grid,
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

[[nodiscard]] StaticRouteCandidateValidation validateStaticRouteCandidate(
    std::span<const RouteSample3D> active_route,
    std::span<const RouteSample3D> candidate_route, const mppi::EsdfGrid& grid,
    std::span<const float> esdf_m, const Point3& mission_goal,
    double minimum_endpoint_improvement_m, bool reaches_mission_goal,
    const FlightEnvelopeConfig& flight_envelope,
    StaticRouteReplacementPolicy replacement_policy =
        StaticRouteReplacementPolicy::kRequireEndpointImprovement,
    const SweptFootprintConfig& footprint_config = {}) noexcept;

[[nodiscard]] std::string_view
staticRouteReplacementPolicyName(StaticRouteReplacementPolicy policy) noexcept;

[[nodiscard]] std::string_view
staticRouteCandidateStatusName(StaticRouteCandidateStatus status) noexcept;

[[nodiscard]] std::string_view
staticRouteActivationStatusName(StaticRouteActivationStatus status) noexcept;

[[nodiscard]] std::string_view
staticRouteSearchRetryTriggerName(StaticRouteSearchRetryTrigger trigger) noexcept;

} // namespace drone_city_nav
