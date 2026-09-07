#pragma once

#include "drone_city_nav/flight_envelope.hpp"
#include "drone_city_nav/navigation_recovery_episode_tracker.hpp"
#include "drone_city_nav/route_risk_annotation_3d.hpp"
#include "drone_city_nav/static_route_extension.hpp"

#include <cstddef>
#include <cstdint>
#include <exception>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string_view>

#include "production_mppi_node_types.hpp"
#include "production_mppi_raw_world.hpp"
#include "production_route_pipeline_artifacts_3d.hpp"
#include "route_activation_coordinator_3d.hpp"
#include "route_materializer_3d.hpp"
#include "route_planning_coordinator_3d.hpp"

namespace drone_city_nav {

enum class RouteLifecycleWorldRefreshPurpose3D : std::uint8_t {
  kRouteExtension,
  kTrackingObjective,
};

struct RouteLifecycleWorldRefreshResult3D {
  std::uint64_t sequence{0U};
  std::uint64_t base_route_generation{0U};

  [[nodiscard]] bool valid() const noexcept {
    return sequence != 0U && base_route_generation != 0U;
  }
};

struct RouteLifecycleTrackingContext3D {
  std::shared_ptr<const ProductionNavigationObjective> objective;
  std::uint64_t minimum_route_mission_epoch{0U};
  std::uint64_t minimum_route_sample_sequence{0U};
};

struct RouteLifecycleReplanSnapshot3D {
  ProductionMppiNavigation navigation{};
  std::shared_ptr<const ProductionNavigationObjective> objective;
  std::shared_ptr<const WorldSnapshot3D> resident_world;
  std::shared_ptr<const PersistentPlannerWorld3D> resident_planner_world;
  std::shared_ptr<const ProductionMppiRawWorld3D> latest_raw_world;
  ProductionWorldBuildTelemetry3D world_telemetry{};
  std::uint64_t committed_route_generation{0U};
  std::uint64_t blocked_raw_revision{0U};
  std::uint64_t minimum_route_mission_epoch{0U};
  std::uint64_t minimum_route_sample_sequence{0U};
  std::int64_t stamp_ns{0};
};

struct RouteLifecycleCandidateSummary3D {
  SpatialRouteCandidateSource3D source{
      SpatialRouteCandidateSource3D::kFeasibilitySearch};
  std::size_t point_count{0U};
  double estimated_execution_time_s{0.0};
  double estimated_translation_time_s{0.0};
  double estimated_stationary_turn_time_s{0.0};
  bool available{false};
};

struct RouteLifecycleTrackingFollowup3D {
  StaticRouteObjective resident_objective{};
  std::uint64_t required_mission_epoch{0U};
  std::uint64_t required_sample_sequence{0U};
  bool required{false};
};

enum class RouteLifecycleAdvanceStatus3D : std::uint8_t {
  kInvalidEvent,
  kContinuationQueued,
  kCompleted,
};

// What the lifecycle must do with a candidate after one activation attempt.
// The candidate is owned by the lifecycle until it reaches a terminal
// disposition, so a transient loss of currentness costs one retry instead of
// discarding a route the search already proved.
enum class RouteCandidateDisposition3D : std::uint8_t {
  // Activation committed the candidate.
  kActivated,
  // The optimistic commit base moved. The candidate itself is untouched, so it
  // is materialized and committed again against the current snapshot.
  kRetrySameCandidate,
  // The current world contradicts the world the search ran on. Neither this
  // candidate nor the search that produced it can be trusted.
  kRetireSearchAndReplan,
  // Activation declined this candidate, but the running search may still find a
  // better one against the same world.
  kContinueForImprovement,
  // Activation rejected the candidate on grounds a retry cannot change.
  kTerminalReject,
};

[[nodiscard]] const char*
routeCandidateDisposition3DName(RouteCandidateDisposition3D disposition) noexcept;

struct RouteLifecycleUpdate3D {
  RouteLifecycleAdvanceStatus3D status{RouteLifecycleAdvanceStatus3D::kInvalidEvent};
  RoutePlanningRequest3D request{};
  RoutePlannerVehicleState3D vehicle_state{};
  RoutePlannerUpdate3D planner_update{};
  RouteLifecycleCandidateSummary3D candidate{};
  ProductionRouteActivationResult3D activation{};
  std::optional<RouteRiskAnnotationResult3D> geometry_optimization_fallback;
  RouteLifecycleTrackingFollowup3D tracking_followup{};
  Point3 search_start{};
  // Wall time this update spent applying an already-delivered planner result:
  // candidate materialisation, validation and activation.
  double route_planning_ms{0.0};
  // Wall time from the moment the search was first enqueued to this update,
  // covering the queue, every search continuation and the activation attempt.
  // This is the lead time a route request actually needs, and the quantity the
  // lookahead extension policy is sized from; route_planning_ms sees only the
  // last slice of it.
  double route_search_latency_ms{0.0};
  // True when this update ends the search: it activated a route, failed, or
  // was retired. Only then is route_search_latency_ms a complete measurement.
  bool search_latency_complete{false};
  RouteCandidateDisposition3D candidate_disposition{
      RouteCandidateDisposition3D::kContinueForImprovement};
  std::size_t activation_attempts{0U};
  bool search_running{false};
  bool search_superseded_by_activation{false};
  // The search's gate was cleared by a release that queued a fresh search;
  // its continuation is dropped instead of displacing that search.
  bool search_retired{false};
  bool continuation_queued{false};
  // Activation rejected the delivered incumbent for a vehicle-relative reason;
  // the continuation asks the planner to drop it and search afresh.
  bool incumbent_rejected{false};
  bool failed_search_latched{false};

  [[nodiscard]] bool valid() const noexcept {
    return status != RouteLifecycleAdvanceStatus3D::kInvalidEvent &&
           request.transaction != nullptr;
  }
};

struct RouteLifecycleExtensionRequest3D {
  std::shared_ptr<const WorldSnapshot3D> world;
  ProductionWorldBuildTelemetry3D world_telemetry{};
  std::shared_ptr<const CertifiedRouteSuffix3D> active_route;
  ProductionMppiNavigation navigation{};
  std::shared_ptr<const ProductionNavigationObjective> objective;
  RouteProgressProjection3D route_projection{};
  std::int64_t stamp_ns{0};
  bool pending_successor{false};
};

enum class RouteLifecycleExtensionStatus3D : std::uint8_t {
  kNotRequired,
  kInvalidRequest,
  kInvalidTransaction,
  kRouteQueueBusy,
  kWorldRefreshUnavailable,
  kSearchQueued,
  kWorldRefreshQueued,
};

[[nodiscard]] std::string_view
routeLifecycleExtensionStatus3DName(RouteLifecycleExtensionStatus3D status) noexcept;

struct RouteLifecycleExtensionOutcome3D {
  RouteLifecycleExtensionStatus3D status{RouteLifecycleExtensionStatus3D::kNotRequired};
  StaticRouteExtensionDecision decision{};
  StaticRoutePlanningLatencyStats planning_latency{};
  RoutePlanningEnqueueStatus3D enqueue_status{
      RoutePlanningEnqueueStatus3D::kInvalidRequest};
  std::uint64_t route_generation{0U};
  std::uint64_t world_refresh_sequence{0U};
  double station_m{0.0};
  double remaining_m{0.0};
  bool observed_world{false};

  [[nodiscard]] bool queued() const noexcept {
    return status == RouteLifecycleExtensionStatus3D::kSearchQueued ||
           status == RouteLifecycleExtensionStatus3D::kWorldRefreshQueued;
  }
};

enum class RouteLifecycleReplanStatus3D : std::uint8_t {
  kObjectiveUnavailable,
  kDeferredDuringExtension,
  kDeferredReplanInFlight,
  kInvalidResidentWorld,
  kWaitingInitialSearch,
  kGenerationMismatch,
  kWaitingRawSnapshot,
  kInvalidStart,
  kSuppressedFailedSearch,
  kInvalidTransaction,
  kRouteQueueBusy,
  kQueued,
};

enum class RouteLifecycleReplanOrigin3D : std::uint8_t {
  kRequested,
  kDeferredExtensionReplay,
  kDeferredReplanReplay,
};

[[nodiscard]] std::string_view
routeLifecycleReplanStatus3DName(RouteLifecycleReplanStatus3D status) noexcept;

struct RouteLifecycleReplanOutcome3D {
  RouteLifecycleReplanStatus3D status{
      RouteLifecycleReplanStatus3D::kObjectiveUnavailable};
  RouteLifecycleReplanOrigin3D origin{RouteLifecycleReplanOrigin3D::kRequested};
  RouteReleaseReason3D reason{RouteReleaseReason3D::kNone};
  StaticRouteSearchRetryDecision retry{};
  RoutePlanningEnqueueStatus3D enqueue_status{
      RoutePlanningEnqueueStatus3D::kInvalidRequest};
  std::optional<std::uint64_t> cleared_gate_generation;
  std::uint64_t requested_route_generation{0U};
  std::uint64_t replay_completed_generation{0U};
  std::uint64_t committed_route_generation{0U};
  std::uint64_t search_generation{0U};
  std::uint64_t deferred_route_generation{0U};
  std::uint64_t in_flight_generation{0U};
  std::uint64_t resident_world_revision{0U};
  std::uint64_t resident_source_raw_revision{0U};
  std::uint64_t latest_raw_revision{0U};
  std::uint64_t blocked_raw_revision{0U};
  std::uint64_t dispatched_raw_revision{0U};
  Point3 search_start{};
  bool raw_search_overlay_used{false};

  [[nodiscard]] bool queued() const noexcept {
    return status == RouteLifecycleReplanStatus3D::kQueued;
  }
};

struct RouteLifecycleTrackingRefreshRequest3D {
  std::shared_ptr<const WorldSnapshot3D> world;
  ProductionMppiNavigation navigation{};
  std::shared_ptr<const ProductionNavigationObjective> objective;
  std::int64_t stamp_ns{0};
};

enum class RouteLifecycleTrackingRefreshStatus3D : std::uint8_t {
  kNotRequired,
  kInvalidRequest,
  kLifecycleBusy,
  kWorldRefreshUnavailable,
  kQueued,
};

[[nodiscard]] std::string_view routeLifecycleTrackingRefreshStatus3DName(
    RouteLifecycleTrackingRefreshStatus3D status) noexcept;

struct RouteLifecycleTrackingRefreshOutcome3D {
  RouteLifecycleTrackingRefreshStatus3D status{
      RouteLifecycleTrackingRefreshStatus3D::kNotRequired};
  std::uint64_t sequence{0U};
  std::uint64_t base_route_generation{0U};
  std::uint64_t objective_mission_epoch{0U};
  std::uint64_t objective_sample_sequence{0U};
  std::int64_t stamp_ns{0};
};

using RouteActivationCommitOperation3D = std::function<RouteActivationCommitResult3D(
    PreparedRouteActivation3D, const RouteActivationCommitContext3D&)>;
using RouteActivationCommitBoundary3D = std::function<RouteActivationCommitResult3D(
    PreparedRouteActivation3D, const RouteActivationCommitOperation3D&)>;

struct RouteLifecycleCoordinatorConfig3D {
  RoutePlannerConfig3D planner{};
  RouteMaterializerConfig3D materializer{};
  RouteActivationCoordinatorConfig3D activation{};
  StaticRouteExtensionConfig extension{};
  StaticRouteSearchRetryConfig search_retry{};
  FlightEnvelopeConfig flight_envelope{};
  double static_route_lookahead_m{0.0};
  double tracking_world_refresh_margin_m{0.0};
  bool observed_world{false};

  std::function<RoutePlannerVehicleState3D()> vehicle_state_provider;
  std::function<ProductionRouteActivationSnapshot3D()> activation_snapshot_provider;
  RouteActivationCommitBoundary3D activation_commit_boundary;
  std::function<RouteLifecycleTrackingContext3D()> tracking_context_provider;
  std::function<RouteLifecycleReplanSnapshot3D()> replan_snapshot_provider;
  std::function<RouteLifecycleWorldRefreshResult3D(std::uint64_t,
                                                   RouteLifecycleWorldRefreshPurpose3D)>
      world_refresh_requester;
  std::function<std::int64_t()> stamp_provider;

  std::function<void(RouteLifecycleUpdate3D)> update_handler;
  std::function<void(const RoutePlanningRejection3D&)> rejection_handler;
  std::function<void(const RouteLifecycleReplanOutcome3D&)> replan_outcome_handler;
  std::function<void(const RouteLifecycleTrackingFollowup3D&)>
      tracking_followup_handler;
  std::function<void(std::exception_ptr)> failure_handler;
};

class RouteLifecycleCoordinator3D final {
public:
  RouteLifecycleCoordinator3D(ExecutionSupervisor3D& execution_supervisor,
                              RouteLifecycleCoordinatorConfig3D config);
  ~RouteLifecycleCoordinator3D();

  RouteLifecycleCoordinator3D(const RouteLifecycleCoordinator3D&) = delete;
  RouteLifecycleCoordinator3D& operator=(const RouteLifecycleCoordinator3D&) = delete;
  RouteLifecycleCoordinator3D(RouteLifecycleCoordinator3D&&) = delete;
  RouteLifecycleCoordinator3D& operator=(RouteLifecycleCoordinator3D&&) = delete;

  void start();
  void stop() noexcept;

  [[nodiscard]] RoutePlanningEnqueueResult3D
  enqueue(RoutePlanningRequest3D request,
          RoutePlanningQueuePolicy3D policy = RoutePlanningQueuePolicy3D::kKeepPending);
  [[nodiscard]] std::optional<RoutePlanningRequest3D> cancelPending();
  [[nodiscard]] bool pending() const noexcept;
  [[nodiscard]] RoutePlanningCoordinatorStatistics3D statistics() const noexcept;

  [[nodiscard]] RouteLifecycleUpdate3D advance(RoutePlanningUpdateEvent3D event);
  [[nodiscard]] RouteLifecycleExtensionOutcome3D
  requestExtension(RouteLifecycleExtensionRequest3D request);
  [[nodiscard]] RouteLifecycleReplanOutcome3D
  requestReplan(RouteReleaseReason3D reason, std::uint64_t route_generation);
  [[nodiscard]] RouteLifecycleTrackingRefreshOutcome3D
  requestTrackingWorldRefresh(RouteLifecycleTrackingRefreshRequest3D request);

  void finishWorldRefresh(std::uint64_t base_generation,
                          RouteLifecycleWorldRefreshPurpose3D purpose,
                          bool route_activated = false);
  void finishSearch(const PlannerSearchTransaction3D& transaction,
                    bool route_activated = false);

  // Lead time of a route request: what the lookahead extension is sized from.
  [[nodiscard]] StaticRoutePlanningLatencyStats
  planningLatencyStatistics() const noexcept;
  // Duration of one planner update's search step: whether the planner returns
  // within its compute budget.
  [[nodiscard]] StaticRoutePlanningLatencyStats
  plannerUpdateLatencyStatistics() const noexcept;
  [[nodiscard]] std::uint64_t recoverySequence() const noexcept;

private:
  [[nodiscard]] std::uint64_t nextRouteGeneration() const noexcept;
  [[nodiscard]] RouteLifecycleReplanOutcome3D
  requestReplanImpl(RouteReleaseReason3D reason, std::uint64_t route_generation,
                    RouteLifecycleReplanOrigin3D origin,
                    std::uint64_t replay_completed_generation = 0U);
  // Whether the transaction's gate no longer names it as the in-flight
  // search: a release retired it and queued a replacement.
  [[nodiscard]] bool
  searchRetired(const PlannerSearchTransaction3D& transaction) const noexcept;
  [[nodiscard]] bool queueContinuation(const RouteLifecycleUpdate3D& update);
  std::uint64_t incumbent_rejection_sequence_{0U};
  // Rebases a continuation request onto the newest coherent world when it is
  // strictly newer than the session world.
  void refreshContinuationWorld(RoutePlanningRequest3D& request,
                                const PlannerTelemetry3D& telemetry) const;
  // Moves a vehicle-anchored continuation start to the current vehicle state.
  void refreshContinuationStart(RoutePlanningRequest3D& request) const;
  void handleWorkerUpdate(RoutePlanningUpdateEvent3D event);
  void handleWorkerRejection(const RoutePlanningRejection3D& rejection);
  void observeRecoveryEpisode(std::uint64_t mission_epoch) noexcept;
  void finishExtension(std::uint64_t base_generation, bool route_activated);
  void finishReplan(std::uint64_t base_generation, bool route_activated);
  void replayDeferredReplan(const StaticRouteDeferredReplan& replay,
                            RouteLifecycleReplanOrigin3D origin,
                            std::uint64_t completed_generation) noexcept;
  void notifyReplanOutcome(const RouteLifecycleReplanOutcome3D& outcome) const;
  void handleFailure(std::exception_ptr failure) const noexcept;

  ExecutionSupervisor3D& execution_supervisor_;
  RouteLifecycleCoordinatorConfig3D config_{};
  RouteMaterializer3D route_materializer_;
  RouteActivationCoordinator3D route_activation_coordinator_;
  std::unique_ptr<RoutePlanningCoordinator3D> route_planning_coordinator_;

  mutable std::mutex lifecycle_mutex_;
  bool extension_request_in_flight_{false};
  std::uint64_t extension_in_flight_generation_{0U};
  std::uint64_t extension_last_request_generation_{0U};
  double extension_last_request_station_m_{0.0};
  std::int64_t extension_last_request_stamp_ns_{0};
  double latest_route_search_ms_{0.0};
  // Two different questions, two trackers.
  //
  // `planning_latency_tracker_` holds the lead time of a route request: queue,
  // every search continuation and the activation attempt. That is what the
  // lookahead extension policy has to plan around, and sizing it from
  // anything shorter is why the extension underestimated the margin it needed.
  //
  // `planner_update_latency_tracker_` holds how long one planner update's
  // search step took. That is the question "does the planner return within its
  // compute budget", which is a property of one update and not of a request.
  StaticRoutePlanningLatencyTracker planning_latency_tracker_{};
  StaticRoutePlanningLatencyTracker planner_update_latency_tracker_{};
  StaticRouteDeferredReplanLatch deferred_replan_latch_{};
  StaticRouteReplanGate replan_gate_{};
  // Mission epoch of the objective the in-flight replan search serves.
  std::uint64_t replan_in_flight_mission_epoch_{0U};
  // The in-flight replan search already delivered a candidate; what it still
  // runs for are improvements, which a release request does not wait for.
  bool replan_in_flight_published_{false};
  StaticRouteFailedSearchLatch failed_search_latch_{};
  NavigationRecoveryEpisodeTracker recovery_episodes_{};
};

} // namespace drone_city_nav
