#include "drone_city_nav/distance_field.hpp"
#include "drone_city_nav/mppi/mppi_control_sequence.hpp"
#include "drone_city_nav/navigation_state_prediction.hpp"
#include "drone_city_nav/ros_conversions.hpp"

#include <algorithm>
#include <chrono>
#include <cinttypes>
#include <cmath>
#include <limits>
#include <span>
#include <tuple>
#include <utility>

#include "production_mppi_node.hpp"
#include "production_mppi_route_helpers.hpp"

namespace drone_city_nav {
namespace {

[[nodiscard]] double distance3(const mppi::State& first, const mppi::State& second) {
  return std::hypot(std::hypot(static_cast<double>(first.x - second.x),
                               static_cast<double>(first.y - second.y)),
                    static_cast<double>(first.z - second.z));
}

[[nodiscard]] mppi::State interpolateState(const mppi::State& first,
                                           const mppi::State& second,
                                           const double ratio) {
  const float clamped = static_cast<float>(std::clamp(ratio, 0.0, 1.0));
  return mppi::State{
      .x = std::lerp(first.x, second.x, clamped),
      .y = std::lerp(first.y, second.y, clamped),
      .z = std::lerp(first.z, second.z, clamped),
      .vx = std::lerp(first.vx, second.vx, clamped),
      .vy = std::lerp(first.vy, second.vy, clamped),
      .vz = std::lerp(first.vz, second.vz, clamped),
      .yaw = std::lerp(first.yaw, second.yaw, clamped),
      .yaw_rate = std::lerp(first.yaw_rate, second.yaw_rate, clamped),
  };
}

[[nodiscard]] mppi::State sampleState(const std::span<const mppi::State> states,
                                      const double offset_steps) {
  if (states.empty()) {
    return {};
  }
  const double source =
      std::clamp(offset_steps, 0.0, static_cast<double>(states.size() - 1U));
  const std::size_t lower = static_cast<std::size_t>(std::floor(source));
  const std::size_t upper = std::min(lower + 1U, states.size() - 1U);
  return interpolateState(states[lower], states[upper],
                          source - static_cast<double>(lower));
}

[[nodiscard]] int pendingGuideStatusRank(const LatticePlanStatus status) noexcept {
  switch (status) {
    case LatticePlanStatus::kReachedPlanningGoal:
      return 0;
    case LatticePlanStatus::kViableFrontier:
      return 1;
    case LatticePlanStatus::kRawSafeDetourPrefix:
      return 2;
    case LatticePlanStatus::kSearchIncomplete:
    case LatticePlanStatus::kMotionGraphExhausted:
    case LatticePlanStatus::kInvalidInput:
      return 3;
  }
  return 3;
}

[[nodiscard]] bool betterPendingGuide(
    const ProductionPendingGlobalGuide& candidate,
    const std::optional<ProductionPendingGlobalGuide>& current) noexcept {
  if (!current.has_value()) {
    return true;
  }
  return std::tuple{pendingGuideStatusRank(candidate.status),
                    candidate.remaining_goal_distance_m, candidate.cost,
                    candidate.fingerprint} <
         std::tuple{pendingGuideStatusRank(current->status),
                    current->remaining_goal_distance_m, current->cost,
                    current->fingerprint};
}

} // namespace

void ProductionMppiNode::guideWorker(const std::stop_token stop_token) {
  std::size_t active_guide_expansions = 0U;
  double active_guide_cost = 0.0;
  std::shared_ptr<const std::vector<mppi::RouteSample3D>> mppi_route;
  std::shared_ptr<const std::vector<Point2>> route_source;
  std::shared_ptr<const ProductionMppiPreparedEsdf> world;
  RiskAwareLatticeSearchSession search_session;
  std::optional<ProductionMppiNavigation> search_navigation;
  std::optional<GlobalGuideHeading> search_heading;
  std::size_t continuation_attempt = 0U;
  bool search_session_in_progress = false;
  std::chrono::steady_clock::time_point search_session_started{};
  std::int64_t adaptive_search_until_ns{0};
  while (!stop_token.stop_requested()) {
    {
      std::unique_lock lock{guide_queue_mutex_};
      if (!world) {
        guide_queue_condition_.wait(
            lock, stop_token, [this]() { return pending_guide_world_ != nullptr; });
      }
      if (stop_token.stop_requested()) {
        return;
      }
      if (!world && pending_guide_world_) {
        world = std::exchange(pending_guide_world_, nullptr);
        search_session.reset();
        search_navigation.reset();
        search_heading.reset();
        continuation_attempt = 0U;
        search_session_in_progress = false;
        search_session_started = std::chrono::steady_clock::now();
      }
    }
    if (!world || !world->distances_m) {
      world.reset();
      continue;
    }
    if (use_static_map_ && world->grid.depth > 1) {
      ProductionMppiNavigation navigation;
      {
        const std::scoped_lock lock{input_mutex_};
        navigation = navigation_;
      }
      if (!navigation.valid) {
        if (world->static_route_extension_request) {
          finishStaticRouteExtension(world->static_route_extension_base_generation);
        }
        if (world->static_route_replan_request) {
          finishStaticRouteReplan(world->static_route_replan_base_generation);
        }
        world.reset();
        continue;
      }
      processStaticGuideSearch(*world, navigation);
      world.reset();
      search_navigation.reset();
      search_heading.reset();
      continuation_attempt = 0U;
      continue;
    }
    const mppi::EsdfGrid& grid = world->grid;
    const std::shared_ptr<const std::vector<float>>& host_distances =
        world->distances_m;
    RiskAwareLatticeConfig search_config = lattice_config_;
    const std::int64_t worker_now_ns = get_clock()->now().nanoseconds();
    if (worker_now_ns < adaptive_search_until_ns) {
      search_config.minimum_frontier_reachable_depth_m =
          std::max(search_config.minimum_frontier_reachable_depth_m,
                   no_static_adaptive_reachable_depth_m_);
      search_config.minimum_frontier_guide_length_m =
          std::max(search_config.minimum_frontier_guide_length_m,
                   no_static_adaptive_minimum_guide_length_m_);
      search_config.minimum_frontier_endpoint_displacement_m =
          std::max(search_config.minimum_frontier_endpoint_displacement_m,
                   no_static_adaptive_minimum_endpoint_displacement_m_);
      search_config.frontier_validation_maximum_states =
          std::max(search_config.frontier_validation_maximum_states,
                   no_static_adaptive_validation_states_);
      search_config.maximum_expansions = std::max(
          search_config.maximum_expansions, lattice_config_.maximum_expansions * 2U);
    }
    if (!search_navigation.has_value()) {
      const std::scoped_lock lock{input_mutex_};
      search_navigation = navigation_;
    }
    const ProductionMppiNavigation navigation = *search_navigation;
    ActiveGlobalGuideUpdate guide_update;
    GlobalGuideHeading guide_heading;
    RiskAwareLatticeResult lattice_observation;
    bool lattice_search_performed = false;
    GlobalGuideAcceptanceResult guide_acceptance;
    std::shared_ptr<const std::vector<Point2>> guide;
    std::shared_ptr<const ProductionMppiPreparedEsdf> publication_world = world;
    RawGuideValidationResult raw_validation;
    ProductionGuideCandidateValidationStatus candidate_validation_status{
        ProductionGuideCandidateValidationStatus::kNotAttempted};
    std::uint64_t validation_revision = 0U;
    bool latest_world_rejected_candidate = false;
    if (navigation.valid && active_guide_lifecycle_) {
      const Point2 position{navigation.state.x, navigation.state.y};
      const std::int64_t blacklist_now_ns = get_clock()->now().nanoseconds();
      std::erase_if(frontier_blacklist_,
                    [blacklist_now_ns](const LatticeFrontierBlacklistEntry& entry) {
                      return entry.expires_at_ns <= blacklist_now_ns;
                    });
      Point2 candidate_validation_position = position;
      const auto validate_candidate_on_latest_world =
          [&](const std::shared_ptr<const std::vector<Point2>>& candidate,
              const bool reaches_mission_goal) {
            std::shared_ptr<const ProductionMppiPreparedEsdf> validation_world;
            {
              const std::scoped_lock lock{esdf_state_mutex_};
              if (prepared_esdf_.has_value()) {
                validation_world =
                    std::make_shared<const ProductionMppiPreparedEsdf>(*prepared_esdf_);
              }
            }
            ProductionMppiNavigation validation_navigation;
            {
              const std::scoped_lock lock{input_mutex_};
              validation_navigation = navigation_;
            }
            if (!validation_world || !validation_world->distances_m ||
                !validation_world->raw_occupancy || !validation_navigation.valid) {
              candidate_validation_status =
                  ProductionGuideCandidateValidationStatus::kUnavailableLatestWorld;
              latest_world_rejected_candidate = true;
              return false;
            }
            validation_revision = validation_world->revision;
            candidate_validation_position =
                Point2{validation_navigation.state.x, validation_navigation.state.y};
            const GlobalGuideProjection candidate_projection =
                projectOntoGlobalGuide(*candidate, candidate_validation_position);
            if (!candidate_projection.valid) {
              candidate_validation_status =
                  ProductionGuideCandidateValidationStatus::kInvalidProjection;
              latest_world_rejected_candidate = true;
              return false;
            }
            if (candidate_projection.cross_track_m >
                active_guide_config_.maximum_cross_track_m) {
              candidate_validation_status =
                  ProductionGuideCandidateValidationStatus::kExcessiveCrossTrack;
              latest_world_rejected_candidate = true;
              return false;
            }
            raw_validation = validateGuideAgainstRawOccupancy(
                *candidate, *validation_world->raw_occupancy,
                active_guide_config_.validation_sample_step_m,
                candidate_projection.station_m);
            if (!raw_validation.accepted) {
              candidate_validation_status =
                  ProductionGuideCandidateValidationStatus::kRawValidationRejected;
              latest_world_rejected_candidate = true;
              return false;
            }
            ActiveGlobalGuideLifecycle validator{active_guide_config_};
            if (!validator
                     .accept(candidate, reaches_mission_goal, validation_world->grid,
                             *validation_world->distances_m,
                             candidate_validation_position)
                     .accepted) {
              candidate_validation_status =
                  ProductionGuideCandidateValidationStatus::kLifecycleRejected;
              latest_world_rejected_candidate = true;
              return false;
            }
            candidate_validation_status =
                ProductionGuideCandidateValidationStatus::kAccepted;
            publication_world = std::move(validation_world);
            return true;
          };
      const std::shared_ptr<const std::vector<Point2>> previous_active_guide =
          active_guide_lifecycle_->guide();
      guide_update = active_guide_lifecycle_->update(
          grid, *host_distances, position,
          guide_release_generation_.load(std::memory_order_acquire),
          guide_release_reason_.load(std::memory_order_relaxed));
      if (frontier_blacklist_enabled_ && !guide_update.active &&
          previous_active_guide && previous_active_guide->size() >= 2U &&
          (guide_update.release_reason == GlobalGuideReleaseReason::kStalled ||
           guide_update.release_reason ==
               GlobalGuideReleaseReason::kPersistentSafetyRejection)) {
        const GlobalGuideProjection failure_projection =
            projectOntoGlobalGuide(*previous_active_guide, position);
        const Point2 failure_point =
            failure_projection.valid ? failure_projection.point : position;
        const Point2 failure_tangent =
            failure_projection.valid ? failure_projection.tangent
                                     : Point2{navigation.state.vx, navigation.state.vy};
        frontier_blacklist_.push_back(LatticeFrontierBlacklistEntry{
            .failure_point = failure_point,
            .approach_heading_rad = std::atan2(failure_tangent.y, failure_tangent.x),
            .expires_at_ns = blacklist_now_ns + static_cast<std::int64_t>(
                                                    frontier_blacklist_ttl_s_ * 1.0e9),
            .soft_penalty_cost = 0.0,
        });
        constexpr std::size_t kMaximumFrontierBlacklistEntries{8U};
        if (frontier_blacklist_.size() > kMaximumFrontierBlacklistEntries) {
          frontier_blacklist_.erase(frontier_blacklist_.begin());
        }
      }
      if (guide_update.active) {
        guide = active_guide_lifecycle_->guide();
        guide_heading.source = GlobalGuideHeadingSource::kActiveGuide;
        if (guide_update.requires_replan || search_session_in_progress) {
          if (!search_heading.has_value()) {
            search_heading = active_guide_lifecycle_->selectPlanningHeading(
                navigation.state, Point2{mission_goal_.x, mission_goal_.y});
          }
          guide_heading = *search_heading;
          lattice_observation = planRiskAwareMotionPrimitiveGuide(
              grid, *host_distances, position, guide_heading.heading_rad,
              Point2{mission_goal_.x, mission_goal_.y}, search_config,
              frontier_blacklist_, &search_session, planning_worker_pool_.get());
          lattice_search_performed = true;
          const auto candidate =
              std::make_shared<const std::vector<Point2>>(lattice_observation.guide);
          const bool reaches_mission_goal =
              lattice_observation.status == LatticePlanStatus::kReachedPlanningGoal &&
              lattice_observation.reached_mission_goal &&
              lattice_observation.exact_terminal_connector && !candidate->empty() &&
              distance(candidate->back(), Point2{mission_goal_.x, mission_goal_.y}) <=
                  1.0e-6;
          const bool executable =
              lattice_observation.status == LatticePlanStatus::kReachedPlanningGoal ||
              lattice_observation.status == LatticePlanStatus::kViableFrontier ||
              lattice_observation.status == LatticePlanStatus::kRawSafeDetourPrefix;
          if (executable &&
              validate_candidate_on_latest_world(candidate, reaches_mission_goal)) {
            if (guide_update.release_reason == GlobalGuideReleaseReason::kExhausted) {
              const double session_age_ms =
                  std::chrono::duration<double, std::milli>(
                      std::chrono::steady_clock::now() - search_session_started)
                      .count();
              if (!lattice_observation.search_session_complete &&
                  previous_active_guide &&
                  session_age_ms < lattice_search_session_maximum_ms_) {
                const ProductionPendingGlobalGuide pending{
                    .guide = candidate,
                    .reaches_mission_goal = reaches_mission_goal,
                    .status = lattice_observation.status,
                    .remaining_goal_distance_m =
                        lattice_observation.remaining_goal_distance_m,
                    .cost = lattice_observation.cost,
                    .fingerprint = routeFingerprint(*candidate),
                };
                if (betterPendingGuide(pending, pending_global_guide_)) {
                  pending_global_guide_ = pending;
                }
              } else {
                guide_acceptance = active_guide_lifecycle_->accept(
                    candidate, reaches_mission_goal, publication_world->grid,
                    *publication_world->distances_m, candidate_validation_position);
                if (guide_acceptance.accepted) {
                  guide = active_guide_lifecycle_->guide();
                  guide_update = active_guide_lifecycle_->status();
                  active_guide_expansions = lattice_observation.expansions;
                  active_guide_cost = lattice_observation.cost;
                }
              }
            } else {
              const ProductionPendingGlobalGuide pending{
                  .guide = candidate,
                  .reaches_mission_goal = reaches_mission_goal,
                  .status = lattice_observation.status,
                  .remaining_goal_distance_m =
                      lattice_observation.remaining_goal_distance_m,
                  .cost = lattice_observation.cost,
                  .fingerprint = routeFingerprint(*candidate),
              };
              if (betterPendingGuide(pending, pending_global_guide_)) {
                pending_global_guide_ = pending;
              }
            }
          }
        }
      } else {
        if (pending_global_guide_ && pending_global_guide_->guide) {
          if (validate_candidate_on_latest_world(
                  pending_global_guide_->guide,
                  pending_global_guide_->reaches_mission_goal)) {
            guide_acceptance = active_guide_lifecycle_->accept(
                pending_global_guide_->guide,
                pending_global_guide_->reaches_mission_goal, publication_world->grid,
                *publication_world->distances_m, candidate_validation_position);
          }
          pending_global_guide_.reset();
        }
        if (guide_acceptance.accepted) {
          guide = active_guide_lifecycle_->guide();
          guide_update = active_guide_lifecycle_->status();
        } else {
          if (!search_heading.has_value()) {
            search_heading = active_guide_lifecycle_->selectPlanningHeading(
                navigation.state, Point2{mission_goal_.x, mission_goal_.y});
          }
          guide_heading = *search_heading;
          lattice_observation = planRiskAwareMotionPrimitiveGuide(
              grid, *host_distances, position, guide_heading.heading_rad,
              Point2{mission_goal_.x, mission_goal_.y}, search_config,
              frontier_blacklist_, &search_session, planning_worker_pool_.get());
          lattice_search_performed = true;
          const auto candidate = std::make_shared<const std::vector<Point2>>(
              std::move(lattice_observation.guide));
          const bool reaches_mission_goal =
              lattice_observation.status == LatticePlanStatus::kReachedPlanningGoal &&
              lattice_observation.reached_mission_goal &&
              lattice_observation.exact_terminal_connector && !candidate->empty() &&
              distance(candidate->back(), Point2{mission_goal_.x, mission_goal_.y}) <=
                  1.0e-6;
          const bool executable =
              lattice_observation.status == LatticePlanStatus::kReachedPlanningGoal ||
              lattice_observation.status == LatticePlanStatus::kViableFrontier ||
              lattice_observation.status == LatticePlanStatus::kRawSafeDetourPrefix;
          if (executable &&
              validate_candidate_on_latest_world(candidate, reaches_mission_goal)) {
            guide_acceptance = active_guide_lifecycle_->accept(
                candidate, reaches_mission_goal, publication_world->grid,
                *publication_world->distances_m, candidate_validation_position);
          }
          if (guide_acceptance.accepted) {
            guide = active_guide_lifecycle_->guide();
            guide_update = active_guide_lifecycle_->status();
            active_guide_expansions = lattice_observation.expansions;
            active_guide_cost = lattice_observation.cost;
          } else {
            active_guide_expansions = 0U;
            active_guide_cost = 0.0;
          }
        }
      }
    } else if (active_guide_lifecycle_) {
      guide = active_guide_lifecycle_->guide();
      guide_update = active_guide_lifecycle_->status();
      guide_update.retained = guide != nullptr;
    }
    const ActiveGlobalGuideUpdate active_status =
        active_guide_lifecycle_ ? active_guide_lifecycle_->status()
                                : ActiveGlobalGuideUpdate{};
    NoStaticRouteCycleResult cycle_result;
    if (guide && guide->size() >= 2U && no_static_cycle_detector_) {
      const Point2 endpoint = guide->back();
      const Point2 before_endpoint = (*guide)[guide->size() - 2U];
      cycle_result = no_static_cycle_detector_->observe(NoStaticRouteCycleObservation{
          .guide_generation = active_status.generation,
          .stamp_ns = worker_now_ns,
          .vehicle_position = Point2{navigation.state.x, navigation.state.y},
          .guide_endpoint = endpoint,
          .approach_heading_rad = std::atan2(endpoint.y - before_endpoint.y,
                                             endpoint.x - before_endpoint.x),
          .mission_distance_m = distance(Point2{navigation.state.x, navigation.state.y},
                                         Point2{mission_goal_.x, mission_goal_.y}),
      });
      if (cycle_result.cycle_detected) {
        const std::int64_t expires_at_ns =
            worker_now_ns +
            static_cast<std::int64_t>(frontier_blacklist_ttl_s_ * 1.0e9);
        for (const NoStaticDirectedTabuSample& sample : sampleNoStaticDirectedTabu(
                 *guide, no_static_soft_tabu_sample_spacing_m_)) {
          frontier_blacklist_.push_back(LatticeFrontierBlacklistEntry{
              .failure_point = sample.point,
              .approach_heading_rad = sample.approach_heading_rad,
              .expires_at_ns = expires_at_ns,
              .soft_penalty_cost = no_static_soft_tabu_penalty_,
          });
        }
        constexpr std::size_t kMaximumFailureMemoryEntries{64U};
        if (frontier_blacklist_.size() > kMaximumFailureMemoryEntries) {
          frontier_blacklist_.erase(
              frontier_blacklist_.begin(),
              frontier_blacklist_.begin() +
                  static_cast<std::ptrdiff_t>(frontier_blacklist_.size() -
                                              kMaximumFailureMemoryEntries));
        }
        adaptive_search_until_ns =
            worker_now_ns + static_cast<std::int64_t>(
                                no_static_cycle_config_.observation_window_s * 1.0e9);
        requestGuideRelease(GlobalGuideReleaseReason::kStalled,
                            active_status.generation);
      }
    }
    if (!guide) {
      mppi_route.reset();
      route_source.reset();
    } else if (guide.get() != route_source.get()) {
      mppi_route = makeMppiRoute2D(*guide, mission_goal_.z,
                                   speed_policy_config_.cruise_speed_mps);
      route_source = guide;
    }
    ProductionMppiPreparedEsdf prepared = *publication_world;
    prepared.mppi_route = mppi_route;
    prepared.route_2d_projection = guide;
    prepared.global_guide_expansions = active_guide_expansions;
    prepared.global_guide_cost = active_guide_cost;
    prepared.global_guide_generation = active_status.generation;
    prepared.global_guide_reused = guide_update.retained;
    prepared.global_guide_reaches_mission_goal = active_status.reaches_mission_goal;
    prepared.global_guide_release_reason = guide_update.release_reason;
    prepared.global_guide_heading_source = guide_heading.source;
    prepared.global_guide_risk = active_status.current_risk;
    prepared.global_guide_acceptance_reason = guide_acceptance.reason;
    prepared.global_guide_projection = active_status.projection;
    if (lattice_search_performed) {
      prepared.planning_search_kind = ProductionPlanningSearchKind::kLattice2D;
      prepared.planning_search_start =
          Point3{navigation.state.x, navigation.state.y, navigation.state.z};
      prepared.planning_search_goal =
          Point3{lattice_observation.planning_goal.x,
                 lattice_observation.planning_goal.y, mission_goal_.z};
      prepared.planning_candidate_endpoint =
          lattice_observation.guide.empty()
              ? prepared.planning_search_start
              : Point3{lattice_observation.guide.back().x,
                       lattice_observation.guide.back().y, mission_goal_.z};
      prepared.planning_search_direction =
          Vec3{std::cos(guide_heading.heading_rad), std::sin(guide_heading.heading_rad),
               0.0};
      prepared.planning_candidate_points = lattice_observation.guide.size();
      prepared.planning_candidate_samples = lattice_observation.guide.size();
    }
    prepared.lattice_search_performed = lattice_search_performed;
    prepared.lattice_executable = lattice_observation.valid;
    prepared.lattice_status = lattice_observation.status;
    prepared.lattice_termination = lattice_observation.termination;
    prepared.lattice_planning_goal_reached = lattice_observation.planning_goal_reached;
    prepared.lattice_achieved_progress_m = lattice_observation.achieved_progress_m;
    prepared.lattice_guide_length_m = lattice_observation.guide_length_m;
    prepared.lattice_remaining_goal_distance_m =
        lattice_observation.remaining_goal_distance_m;
    prepared.lattice_terminal_successor_count =
        lattice_observation.terminal_successor_count;
    prepared.lattice_risk_stage = lattice_observation.risk_stage;
    prepared.lattice_stale_queue_pops = lattice_observation.stale_queue_pops;
    prepared.lattice_open_peak = lattice_observation.open_peak;
    prepared.lattice_records_peak = lattice_observation.records_peak;
    prepared.lattice_continuation_reachable_states =
        lattice_observation.continuation_reachable_states;
    prepared.lattice_reachable_depth_m = lattice_observation.reachable_depth_m;
    prepared.lattice_frontier_endpoint_displacement_m =
        lattice_observation.frontier_endpoint_displacement_m;
    prepared.lattice_frontier_selection_score =
        lattice_observation.frontier_selection_score;
    prepared.lattice_frontier_candidates_considered =
        lattice_observation.frontier_candidates_considered;
    prepared.continuation_validation_ms =
        lattice_observation.continuation_validation_ms;
    prepared.lattice_successor_diagnostics = lattice_observation.successor_diagnostics;
    prepared.lattice_continuation_attempt = continuation_attempt;
    prepared.lattice_search_session_age_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() -
                                                  search_session_started)
            .count();
    prepared.no_static_cycle_detected = cycle_result.cycle_detected;
    prepared.no_static_adaptive_search = worker_now_ns < adaptive_search_until_ns;
    prepared.no_static_soft_tabu_entries =
        static_cast<std::size_t>(std::ranges::count_if(
            frontier_blacklist_, [](const LatticeFrontierBlacklistEntry& entry) {
              return entry.soft_penalty_cost > 0.0;
            }));
    prepared.lattice_search_session_resumed =
        lattice_observation.search_session_resumed;
    prepared.lattice_search_session_complete =
        lattice_observation.search_session_complete;
    prepared.lattice_search_revision = world->revision;
    prepared.lattice_validation_revision = validation_revision;
    prepared.lattice_raw_validation_status = raw_validation.status;
    prepared.guide_candidate_validation_status = candidate_validation_status;
    prepared.route_fingerprint = guide ? routeFingerprint(*guide) : 0U;
    bool activated = false;
    {
      const std::scoped_lock lock{esdf_state_mutex_};
      if (prepared_esdf_.has_value() && prepared_esdf_->revision == prepared.revision) {
        prepared_esdf_ = prepared;
        activated = true;
      }
    }
    RCLCPP_INFO(
        get_logger(),
        "PRODUCTION_MPPI_GUIDE revision=%" PRIu64 " search_revision=%" PRIu64
        " validation_revision=%" PRIu64
        " candidate_validation=%s raw_validation=%s activated=%s "
        "continuation_attempt=%zu "
        "search_session_resumed=%s search_session_complete=%s "
        "dropped_guide_worlds=%" PRIu64
        " guide_valid=%s guide_points=%zu guide_expansions=%zu guide_cost=%.2f "
        "search_start=(%.2f,%.2f,%.2f) planning_goal=(%.2f,%.2f,%.2f) "
        "endpoint=(%.2f,%.2f,%.2f) direction=(%.3f,%.3f,%.3f) "
        "guide_generation=%" PRIu64
        " guide_reused=%s guide_reaches_mission_goal=%s guide_release=%s "
        "guide_heading_source=%s guide_risk=%s guide_acceptance=%s "
        "guide_station_m=%.2f "
        "guide_remaining_m=%.2f guide_cross_track_m=%.2f "
        "lattice_search_performed=%s lattice_executable=%s lattice_status=%s "
        "lattice_termination=%s lattice_risk_stage=%s "
        "lattice_expansion_limit=%zu lattice_deadline_ms=%.2f "
        "lattice_planning_goal_reached=%s "
        "lattice_achieved_progress_m=%.2f lattice_guide_length_m=%.2f "
        "lattice_remaining_goal_distance_m=%.2f "
        "lattice_stale_pops=%zu lattice_open_peak=%zu lattice_records_peak=%zu "
        "lattice_continuation_states=%zu lattice_reachable_depth_m=%.2f "
        "lattice_frontier_endpoint_displacement_m=%.2f "
        "lattice_frontier_selection_score=%.2f "
        "lattice_frontier_candidates=%zu lattice_terminal_successors=%zu "
        "successor_generated=%zu "
        "successor_accepted=%zu successor_reject_roi=%zu "
        "successor_reject_grid=%zu successor_reject_invalid=%zu "
        "successor_reject_collision=%zu "
        "successor_reject_risk=%zu successor_reject_blacklist=%zu "
        "successor_reject_cost=%zu successor_soft_tabu=%zu "
        "search_session_age_ms=%.2f cycle_detected=%s adaptive_search=%s "
        "soft_tabu_entries=%zu",
        prepared.revision, prepared.lattice_search_revision,
        prepared.lattice_validation_revision,
        productionGuideCandidateValidationStatusName(
            prepared.guide_candidate_validation_status),
        rawGuideValidationStatusName(prepared.lattice_raw_validation_status),
        activated ? "true" : "false", continuation_attempt,
        prepared.lattice_search_session_resumed ? "true" : "false",
        prepared.lattice_search_session_complete ? "true" : "false",
        dropped_guide_worlds_.load(std::memory_order_relaxed),
        guide && guide->size() >= 2U ? "true" : "false", guide ? guide->size() : 0U,
        prepared.global_guide_expansions, prepared.global_guide_cost,
        prepared.planning_search_start.x, prepared.planning_search_start.y,
        prepared.planning_search_start.z, prepared.planning_search_goal.x,
        prepared.planning_search_goal.y, prepared.planning_search_goal.z,
        prepared.planning_candidate_endpoint.x, prepared.planning_candidate_endpoint.y,
        prepared.planning_candidate_endpoint.z, prepared.planning_search_direction.x,
        prepared.planning_search_direction.y, prepared.planning_search_direction.z,
        prepared.global_guide_generation,
        prepared.global_guide_reused ? "true" : "false",
        prepared.global_guide_reaches_mission_goal ? "true" : "false",
        globalGuideReleaseReasonName(prepared.global_guide_release_reason),
        globalGuideHeadingSourceName(prepared.global_guide_heading_source),
        globalGuideRiskTierName(prepared.global_guide_risk),
        globalGuideAcceptanceReasonName(prepared.global_guide_acceptance_reason),
        prepared.global_guide_projection.station_m,
        prepared.global_guide_projection.remaining_m,
        prepared.global_guide_projection.cross_track_m,
        prepared.lattice_search_performed ? "true" : "false",
        prepared.lattice_executable ? "true" : "false",
        latticePlanStatusName(prepared.lattice_status),
        latticeSearchTerminationName(prepared.lattice_termination),
        latticeRiskStageName(prepared.lattice_risk_stage),
        search_config.maximum_expansions, search_config.maximum_search_time_ms,
        prepared.lattice_planning_goal_reached ? "true" : "false",
        prepared.lattice_achieved_progress_m, prepared.lattice_guide_length_m,
        prepared.lattice_remaining_goal_distance_m, prepared.lattice_stale_queue_pops,
        prepared.lattice_open_peak, prepared.lattice_records_peak,
        prepared.lattice_continuation_reachable_states,
        prepared.lattice_reachable_depth_m,
        prepared.lattice_frontier_endpoint_displacement_m,
        prepared.lattice_frontier_selection_score,
        prepared.lattice_frontier_candidates_considered,
        prepared.lattice_terminal_successor_count,
        prepared.lattice_successor_diagnostics.generated,
        prepared.lattice_successor_diagnostics.accepted,
        prepared.lattice_successor_diagnostics.rejected_outside_roi,
        prepared.lattice_successor_diagnostics.rejected_outside_grid,
        prepared.lattice_successor_diagnostics.rejected_invalid_clearance,
        prepared.lattice_successor_diagnostics.rejected_raw_collision,
        prepared.lattice_successor_diagnostics.rejected_risk_stage,
        prepared.lattice_successor_diagnostics.rejected_blacklisted_failure,
        prepared.lattice_successor_diagnostics.rejected_no_cost_improvement,
        prepared.lattice_successor_diagnostics.soft_tabu_penalties_applied,
        prepared.lattice_search_session_age_ms,
        prepared.no_static_cycle_detected ? "true" : "false",
        prepared.no_static_adaptive_search ? "true" : "false",
        prepared.no_static_soft_tabu_entries);

    const bool search_incomplete =
        lattice_search_performed && !lattice_observation.search_session_complete;
    const double search_session_age_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() -
                                                  search_session_started)
            .count();
    if (search_incomplete && !latest_world_rejected_candidate &&
        search_session_age_ms < lattice_search_session_maximum_ms_) {
      ++continuation_attempt;
      search_session_in_progress = true;
      continue;
    }
    world.reset();
    search_navigation.reset();
    search_heading.reset();
    continuation_attempt = 0U;
    search_session_in_progress = false;
  }
}

mppi::State ProductionMppiNode::selectTarget(const ProductionMppiPreparedEsdf& esdf,
                                             const double current_station_m,
                                             const double lookahead_m,
                                             std::string& target_source,
                                             double& target_station_m) const {
  mppi::State target{static_cast<float>(mission_goal_.x),
                     static_cast<float>(mission_goal_.y),
                     static_cast<float>(mission_goal_.z)};
  target_source = "mission_goal_direct";
  target_station_m = 0.0;
  if (esdf.mppi_route && !esdf.mppi_route->empty()) {
    const float desired_station =
        static_cast<float>(current_station_m + std::max(0.0, lookahead_m));
    const auto selected = std::ranges::lower_bound(*esdf.mppi_route, desired_station,
                                                   {}, &mppi::RouteSample3D::station_m);
    const mppi::RouteSample3D& sample =
        selected == esdf.mppi_route->end() ? esdf.mppi_route->back() : *selected;
    target.x = sample.x_m;
    target.y = sample.y_m;
    target.z = sample.z_m;
    target_station_m = sample.station_m;
    target_source = "global_route_3d";
    return target;
  }
  return target;
}

void ProductionMppiNode::planningTick() {
  if (!engine_) {
    return;
  }
  const auto snapshot_started = std::chrono::steady_clock::now();
  ProductionMppiNavigation navigation;
  ProductionMppiPredictionError prediction;
  ProductionMppiAppliedControl applied_control;
  std::uint64_t memory_sequence{0U};
  {
    const std::scoped_lock lock{input_mutex_};
    navigation = navigation_;
    prediction = latest_prediction_error_;
    applied_control = applied_control_;
    memory_sequence = memory_sequence_;
  }
  std::optional<ProductionMppiPreparedEsdf> esdf;
  {
    const std::scoped_lock lock{esdf_state_mutex_};
    esdf = prepared_esdf_;
  }
  const std::int64_t now_ns = get_clock()->now().nanoseconds();
  const double pose_age_ms =
      static_cast<double>(now_ns - navigation.receive_stamp_ns) / 1.0e6;
  const double esdf_age_ms =
      esdf.has_value() ? static_cast<double>(now_ns - esdf->ready_stamp_ns) / 1.0e6
                       : std::numeric_limits<double>::infinity();
  const double control_feedback_age_ms =
      applied_control.valid
          ? static_cast<double>(now_ns - applied_control.receive_stamp_ns) / 1.0e6
          : std::numeric_limits<double>::infinity();
  if (!navigation.valid || pose_age_ms < 0.0) {
    return;
  }
  bool pose_predicted = false;
  if (pose_age_ms > maximum_pose_age_ms_) {
    const NavigationStatePredictionResult predicted =
        predictNavigationState(navigation.state, pose_age_ms / 1000.0,
                               maximum_pose_prediction_age_ms_ / 1000.0);
    if (!predicted.valid) {
      return;
    }
    navigation.state = predicted.state;
    pose_predicted = predicted.predicted;
  }
  if (!esdf.has_value() || esdf_age_ms < 0.0 ||
      esdf_age_ms > maximum_esdf_age_ms_ + stale_esdf_execution_window_ms_) {
    const ProductionMppiPreparedEsdf stale_esdf =
        esdf.value_or(ProductionMppiPreparedEsdf{});
    mppi::MppiTickInput input{
        .initial_state = navigation.state,
        .target = navigation.state,
        .pose_revision = navigation.revision,
        .obstacle_revision = stale_esdf.revision,
        .planning_stamp_ns = now_ns,
        .previous_applied_control = std::nullopt,
        .nominal_reseed_generation = 0U,
        .reference_speed_mps = 0.0F,
        .route = std::nullopt,
    };
    const MppiHorizonSafetyResult fallback =
        buildMppiBrakingFallback(input.initial_state, safety_config_);
    mppi::MppiTickResult result;
    result.horizon = fallback.fallback_horizon;
    result.controls = fallback.fallback_controls;
    result.selected_tier = mppi::RiskTier::kPreferred;
    result.raw_collision = false;
    result.known_solid_collision = false;
    result.esdf_revision = stale_esdf.revision;
    result.timings.host_total_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() -
                                                  snapshot_started)
            .count();
    const ProductionMppiPlanningState planning_state =
        ProductionMppiPlanningState::kUnavailableWorldBrakingHold;
    ++tick_sequence_;
    recordTickStatistics(result, planning_state, false);
    ProductionMppiExecutionPublication execution =
        publishExecutionHorizon(input, result, stale_esdf, planning_state, now_ns);
    std::optional<ProductionMppiRvizSnapshot> rviz;
    if (now_ns - last_rviz_stamp_ns_ >= rviz_period_ns_) {
      rviz = ProductionMppiRvizSnapshot{
          .candidate_horizon = result.horizon,
          .previous_horizon = previous_result_.has_value() ? previous_result_->horizon
                                                           : std::vector<mppi::State>{},
          .execution_horizon = execution.horizon,
          .route = stale_esdf.mppi_route,
          .channel_edges = stale_esdf.channel_edges,
          .selected_channel_ids = stale_esdf.selected_channel_ids,
      };
      last_rviz_stamp_ns_ = now_ns;
    }
    ProductionMppiPreparedEsdf diagnostic_esdf = stale_esdf;
    diagnostic_esdf.distances_m.reset();
    diagnostic_esdf.mppi_route.reset();
    enqueueDiagnostics(ProductionMppiDiagnosticsSnapshot{
        .input = input,
        .result = result,
        .esdf = std::move(diagnostic_esdf),
        .stability = {},
        .prediction = prediction,
        .liveness = {},
        .speed_policy = {},
        .guide_progress = {},
        .goal_capture = {},
        .execution = std::move(execution),
        .planning_state = planning_state,
        .rviz = std::move(rviz),
        .target_source = "unavailable_world_braking",
        .tick_sequence = tick_sequence_,
        .memory_sequence = memory_sequence,
        .pose_age_ms = pose_age_ms,
        .esdf_age_ms = esdf_age_ms,
        .control_feedback_age_ms = control_feedback_age_ms,
        .snapshot_ms = result.timings.host_total_ms,
        .stability_ms = 0.0,
        .liveness_reseed_requested = false,
        .pose_predicted = pose_predicted,
        .maximum_eligible_risk_tier = maximum_eligible_risk_tier_,
    });
    previous_result_ = std::move(result);
    return;
  }
  if (!engine_->ready()) {
    return;
  }
  const std::span<const Point2> guide =
      esdf->route_2d_projection ? std::span<const Point2>{*esdf->route_2d_projection}
                                : std::span<const Point2>{};
  if (tracked_route_generation_ != esdf->global_guide_generation) {
    tracked_route_generation_ = esdf->global_guide_generation;
    tracked_route_station_m_ = 0.0;
  }
  const RouteProjection3D measured_route_3d =
      esdf->route_3d ? projectOntoRoute3D(*esdf->route_3d,
                                          Point3{navigation.state.x, navigation.state.y,
                                                 navigation.state.z},
                                          tracked_route_station_m_)
                     : RouteProjection3D{};
  const GlobalGuideProjection measured_route_projection{
      .valid = measured_route_3d.valid,
      .station_m = measured_route_3d.station_m,
      .total_length_m = esdf->route_3d && !esdf->route_3d->empty()
                            ? esdf->route_3d->back().station_m
                            : 0.0,
      .remaining_m = measured_route_3d.remaining_m,
      .cross_track_m = measured_route_3d.distance_m,
      .point = Point2{measured_route_3d.point.x, measured_route_3d.point.y},
  };
  if (measured_route_projection.valid) {
    tracked_route_station_m_ =
        std::max(tracked_route_station_m_, measured_route_projection.station_m);
  }
  GlobalGuideProjection route_projection = measured_route_projection;
  if (route_projection.valid) {
    route_projection.station_m = tracked_route_station_m_;
    route_projection.remaining_m =
        std::max(0.0, route_projection.station_m + route_projection.remaining_m -
                          tracked_route_station_m_);
  }
  if (use_static_map_ && route_projection.valid) {
    maybeRequestStaticRouteExtension(*esdf, navigation, route_projection, now_ns);
  }
  const std::span<const RouteSample3D> route_3d =
      esdf->route_3d ? std::span<const RouteSample3D>{*esdf->route_3d}
                     : std::span<const RouteSample3D>{};
  const std::span<const ConstrainedRouteSpan> constrained_spans =
      esdf->constrained_spans
          ? std::span<const ConstrainedRouteSpan>{*esdf->constrained_spans}
          : std::span<const ConstrainedRouteSpan>{};
  const ConstrainedRouteObservation route_constraint = observeConstrainedRoute(
      route_3d, constrained_spans, esdf->global_guide_generation,
      route_projection.station_m,
      Point3{navigation.state.x, navigation.state.y, navigation.state.z},
      Vec3{navigation.state.vx, navigation.state.vy, navigation.state.vz},
      route_envelope_config_, lattice_3d_config_.planning_goal_distance_m);
  const ConstrainedRouteControl route_control = constrained_route_coordinator_.update(
      route_constraint, speed_policy_config_.cruise_speed_mps,
      constrained_route_control_config_);
  const MissionGoalCaptureResult goal_capture =
      mission_goal_capture_latch_
          ? mission_goal_capture_latch_->update(MissionGoalCaptureObservation{
                .mission_goal = mission_goal_,
                .state = navigation.state,
                .terminal_route_available = esdf->global_guide_reaches_mission_goal,
            })
          : MissionGoalCaptureResult{};
  MppiSpeedPolicyResult speed_policy = evaluateMppiSpeedPolicy(
      speed_policy_config_,
      MppiSpeedPolicyInput{
          .state = navigation.state,
          .mission_goal = mission_goal_,
          .guide = guide,
          .route_constraint_speed_limit_mps =
              route_control.active
                  ? std::optional<double>{route_control.speed_limit_mps}
                  : std::nullopt,
      });
  std::string target_source;
  double target_station_m = 0.0;
  mppi::State target =
      selectTarget(*esdf, tracked_route_station_m_, speed_policy.target_lookahead_m,
                   target_source, target_station_m);
  if (route_control.active) {
    target.z = static_cast<float>(route_control.reference_z_m);
    if (route_control.hold_xy) {
      target_source = "channel_vertical_alignment_hold";
    } else if (route_control.vertical_ready) {
      target_source = "channel_traversal";
    } else {
      target_source = "channel_vertical_alignment";
    }
    if (route_control.hold_xy) {
      target.x = navigation.state.x;
      target.y = navigation.state.y;
      speed_policy.reference_speed_mps = 0.0;
      speed_policy.target_lookahead_m = 0.0;
    }
  }
  ProductionMppiPlanningState planning_state = ProductionMppiPlanningState::kPlanned;
  if (goal_capture.latched) {
    planning_state = ProductionMppiPlanningState::kMissionGoalPositionHold;
    target = mppi::State{
        .x = static_cast<float>(mission_goal_.x),
        .y = static_cast<float>(mission_goal_.y),
        .z = static_cast<float>(mission_goal_.z),
        .yaw = navigation.state.yaw,
    };
    speed_policy.reference_speed_mps = 0.0;
    speed_policy.target_lookahead_m = 0.0;
    target_source = "mission_goal_position_hold";
  } else if (target_source == "mission_goal_direct") {
    planning_state = ProductionMppiPlanningState::kNoGuideBrakingHold;
    target = navigation.state;
    speed_policy.reference_speed_mps = 0.0;
    speed_policy.target_lookahead_m = 0.0;
    target_source = "no_guide_braking_hold";
  }
  const bool control_feedback_fresh =
      applied_control.valid && control_feedback_age_ms >= 0.0 &&
      control_feedback_age_ms <= maximum_control_feedback_age_ms_;
  MppiLivenessResult liveness;
  if (liveness_supervisor_) {
    liveness = liveness_supervisor_->evaluate(MppiLivenessObservation{
        .stamp_ns = now_ns,
        .actual_state = navigation.state,
        .controller_active = control_feedback_fresh &&
                             planning_state == ProductionMppiPlanningState::kPlanned &&
                             !route_control.hold_xy && route_projection.valid,
        .emergency_braking =
            control_feedback_fresh && applied_control.emergency_braking,
        .predicted_head_progress_m =
            previous_result_.has_value() ? previous_result_->head_progress_m : 0.0,
        .predicted_terminal_progress_m =
            previous_result_.has_value() ? previous_result_->terminal_progress_m : 0.0,
        .route_generation = esdf->global_guide_generation,
        .route_station_m = route_projection.station_m,
        .route_station_valid = route_projection.valid,
    });
  }
  GlobalGuideProgressUpdate guide_progress;
  if (guide_progress_tracker_) {
    const GlobalGuideProjection& projection = route_projection;
    guide_progress = guide_progress_tracker_->evaluate(GlobalGuideProgressObservation{
        .stamp_ns = now_ns,
        .guide_generation =
            projection.valid && planning_state == ProductionMppiPlanningState::kPlanned
                ? esdf->global_guide_generation
                : 0U,
        .station_m = projection.station_m,
        .predicted_head_progress_m =
            previous_result_.has_value() ? previous_result_->head_progress_m : 0.0,
        .controller_active = control_feedback_fresh && !route_control.hold_xy,
        .emergency_braking =
            control_feedback_fresh && applied_control.emergency_braking,
    });
    if (guide_progress.stalled) {
      requestGuideRelease(guide_progress.persistent_safety_rejection
                              ? GlobalGuideReleaseReason::kPersistentSafetyRejection
                              : GlobalGuideReleaseReason::kStalled,
                          esdf->global_guide_generation);
    }
  }
  const MppiNominalReseedUpdate nominal_reseed =
      nominal_reseed_tracker_.update(MppiNominalReseedObservation{
          .guide_generation = esdf->global_guide_generation,
          .local_liveness_generation = liveness.reseed_generation,
          .guide_liveness_generation = guide_progress.local_reseed_generation,
          .safety_rejection_generation = guide_progress.persistent_safety_rejection
                                             ? guide_progress.stall_generation
                                             : 0U,
      });
  if (risk_escalation_) {
    const bool stable_progress = previous_result_.has_value() &&
                                 previous_result_->eligible_risk_contract.available &&
                                 guide_progress.progress_m > 1.0e-3;
    maximum_eligible_risk_tier_ =
        risk_escalation_
            ->update(MppiRiskEscalationObservation{
                .reseed_generation =
                    liveness.reseed_generation + guide_progress.local_reseed_generation,
                .no_eligible_recovery_generation =
                    nominal_reseed.no_eligible_recovery_generation,
                .stable_progress = stable_progress,
            })
            .maximum_eligible_tier;
  }
  mppi::RiskTier route_required_risk_tier = mppi::RiskTier::kPreferred;
  if (esdf->mppi_route && route_projection.valid) {
    const double horizon_distance_m = std::max(
        target_station_m - route_projection.station_m,
        speed_policy.reference_speed_mps * static_cast<double>(mppi_config_.steps) *
            static_cast<double>(mppi_config_.dynamics.dt_s));
    route_required_risk_tier = mppi::maximumRequiredRiskTier(
        *esdf->mppi_route, static_cast<float>(route_projection.station_m),
        static_cast<float>(route_projection.station_m +
                           std::max(0.0, horizon_distance_m)));
    maximum_eligible_risk_tier_ = static_cast<mppi::RiskTier>(
        std::max(static_cast<std::uint8_t>(maximum_eligible_risk_tier_),
                 static_cast<std::uint8_t>(route_required_risk_tier)));
  }
  mppi::MppiTickInput input{
      .initial_state = navigation.state,
      .target = target,
      .pose_revision = navigation.revision,
      .obstacle_revision = esdf->revision,
      .planning_stamp_ns = now_ns,
      .previous_applied_control =
          control_feedback_fresh ? std::optional<mppi::Control>{applied_control.control}
                                 : std::nullopt,
      .nominal_reseed_generation = nominal_reseed.generation,
      .reference_speed_mps = speed_policy.enabled
                                 ? static_cast<float>(speed_policy.reference_speed_mps)
                                 : -1.0F,
      .maximum_eligible_risk_tier = maximum_eligible_risk_tier_,
      .route =
          esdf->mppi_route && route_projection.valid && !route_control.hold_xy
              ? std::optional<mppi::RouteReference>{mppi::RouteReference{
                    .points = esdf->mppi_route,
                    .generation = esdf->global_guide_generation,
                    .initial_station_m = static_cast<float>(route_projection.station_m),
                }}
              : std::nullopt,
  };
  const double snapshot_ms = std::chrono::duration<double, std::milli>(
                                 std::chrono::steady_clock::now() - snapshot_started)
                                 .count();
  mppi::MppiTickResult result;
  MppiEligibleRolloutUpdate no_eligible_recovery{
      .no_eligible_recovery_generation = nominal_reseed.no_eligible_recovery_generation,
      .phase = nominal_reseed.no_eligible_phase,
  };
  if (planning_state == ProductionMppiPlanningState::kNoGuideBrakingHold) {
    const MppiHorizonSafetyResult fallback =
        buildMppiBrakingFallback(input.initial_state, safety_config_);
    result.horizon = fallback.fallback_horizon;
    result.controls = fallback.fallback_controls;
    result.selected_tier = mppi::RiskTier::kPreferred;
    result.raw_collision = false;
    result.esdf_revision = esdf->revision;
    result.timings.host_total_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() -
                                                  snapshot_started)
            .count();
  } else if (planning_state == ProductionMppiPlanningState::kMissionGoalPositionHold) {
    result.horizon = {target, target};
    result.controls = {mppi::Control{}};
    result.selected_tier = mppi::RiskTier::kPreferred;
    result.raw_collision = false;
    result.known_solid_collision = false;
    result.esdf_revision = esdf->revision;
    result.timings.host_total_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() -
                                                  snapshot_started)
            .count();
  } else {
    try {
      result = engine_->plan(input);
    } catch (const std::exception& error) {
      RCLCPP_ERROR(get_logger(), "PRODUCTION_MPPI_TICK failed: %s", error.what());
      return;
    }
    no_eligible_recovery = nominal_reseed_tracker_.observeEligibleRolloutResult(
        result.eligible_risk_contract.available, result.nominal_reseeded);
    if (no_eligible_recovery.guide_replan_requested) {
      requestGuideRelease(GlobalGuideReleaseReason::kNoEligibleRollouts,
                          esdf->global_guide_generation);
    }
  }
  ++tick_sequence_;
  recordTickStatistics(result, planning_state,
                       liveness.reseed_requested ||
                           guide_progress.local_reseed_requested);
  ProductionMppiExecutionPublication execution =
      publishExecutionHorizon(input, result, *esdf, planning_state, now_ns);

  const auto stability_started = std::chrono::steady_clock::now();
  const ProductionMppiStability stability = compareWithPrevious(result);
  const double stability_ms = std::chrono::duration<double, std::milli>(
                                  std::chrono::steady_clock::now() - stability_started)
                                  .count();
  std::optional<ProductionMppiRvizSnapshot> rviz;
  if (now_ns - last_rviz_stamp_ns_ >= rviz_period_ns_) {
    rviz = ProductionMppiRvizSnapshot{
        .candidate_horizon = result.horizon,
        .previous_horizon = previous_result_.has_value() ? previous_result_->horizon
                                                         : std::vector<mppi::State>{},
        .execution_horizon = execution.horizon,
        .route = esdf->mppi_route,
        .channel_edges = esdf->channel_edges,
        .selected_channel_ids = esdf->selected_channel_ids,
    };
    last_rviz_stamp_ns_ = now_ns;
  }

  mppi::MppiTickResult diagnostic_result;
  diagnostic_result.eligible_risk_contract = result.eligible_risk_contract;
  diagnostic_result.post_update_classification = result.post_update_classification;
  diagnostic_result.post_update_repair = result.post_update_repair;
  diagnostic_result.post_update_backtrack_ratio = result.post_update_backtrack_ratio;
  diagnostic_result.selected_tier = result.selected_tier;
  diagnostic_result.raw_collision = result.raw_collision;
  diagnostic_result.known_solid_collision = result.known_solid_collision;
  diagnostic_result.critical_exposure_m = result.critical_exposure_m;
  diagnostic_result.planning_exposure_m = result.planning_exposure_m;
  diagnostic_result.minimum_esdf_distance_m = result.minimum_esdf_distance_m;
  diagnostic_result.head_progress_m = result.head_progress_m;
  diagnostic_result.terminal_progress_m = result.terminal_progress_m;
  diagnostic_result.maximum_acceleration_mps2 = result.maximum_acceleration_mps2;
  diagnostic_result.maximum_jerk_mps3 = result.maximum_jerk_mps3;
  diagnostic_result.first_control_delta = result.first_control_delta;
  diagnostic_result.warm_start_shift_s = result.warm_start_shift_s;
  diagnostic_result.nominal_reseeded = result.nominal_reseeded;
  diagnostic_result.esdf_revision = result.esdf_revision;
  diagnostic_result.timings = result.timings;
  ProductionMppiPreparedEsdf diagnostic_esdf = *esdf;
  diagnostic_esdf.distances_m.reset();
  if (!rviz.has_value()) {
    diagnostic_esdf.mppi_route.reset();
  }
  enqueueDiagnostics(ProductionMppiDiagnosticsSnapshot{
      .input = input,
      .result = std::move(diagnostic_result),
      .esdf = std::move(diagnostic_esdf),
      .stability = stability,
      .prediction = prediction,
      .liveness = liveness,
      .speed_policy = speed_policy,
      .guide_progress = guide_progress,
      .no_eligible_recovery = no_eligible_recovery,
      .goal_capture = goal_capture,
      .execution = std::move(execution),
      .planning_state = planning_state,
      .rviz = std::move(rviz),
      .target_source = target_source,
      .tick_sequence = tick_sequence_,
      .memory_sequence = memory_sequence,
      .pose_age_ms = pose_age_ms,
      .esdf_age_ms = esdf_age_ms,
      .control_feedback_age_ms = control_feedback_age_ms,
      .route_station_m = route_projection.station_m,
      .route_remaining_m = route_projection.remaining_m,
      .snapshot_ms = snapshot_ms,
      .stability_ms = stability_ms,
      .route_projection_valid = route_projection.valid,
      .liveness_reseed_requested = liveness.reseed_requested,
      .pose_predicted = pose_predicted,
      .route_required_risk_tier = route_required_risk_tier,
      .maximum_eligible_risk_tier = maximum_eligible_risk_tier_,
  });
  {
    const std::scoped_lock lock{input_mutex_};
    if (result.horizon.size() > 1U) {
      previous_predicted_next_state_ = result.horizon[1U];
      previous_prediction_stamp_ns_ = now_ns;
    }
  }
  previous_result_ = result;
}

void ProductionMppiNode::requestGuideRelease(const GlobalGuideReleaseReason reason,
                                             const std::uint64_t guide_generation) {
  if (use_static_map_) {
    std::shared_ptr<ProductionMppiPreparedEsdf> request;
    std::scoped_lock lifecycle_lock{static_route_extension_mutex_};
    const bool replan_in_flight = static_route_replan_gate_.inFlight();
    if (deferStaticRouteReleaseDuringExtension(
            static_route_extension_request_in_flight_ || replan_in_flight, reason)) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                           "STATIC_ROUTE_REPLAN_REQUEST status=deferred_active_request "
                           "in_flight_generation=%" PRIu64
                           " requested_generation=%" PRIu64 " reason=%s",
                           static_route_replan_gate_.generation(), guide_generation,
                           globalGuideReleaseReasonName(reason));
      return;
    }
    if (replan_in_flight) {
      RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "STATIC_ROUTE_REPLAN_REQUEST status=coalesced_replan_in_flight "
          "in_flight_generation=%" PRIu64 " requested_generation=%" PRIu64 " reason=%s",
          static_route_replan_gate_.generation(), guide_generation,
          globalGuideReleaseReasonName(reason));
      return;
    }
    {
      const std::scoped_lock esdf_lock{esdf_state_mutex_};
      if (!prepared_esdf_ || prepared_esdf_->global_guide_generation == 0U ||
          (guide_generation != 0U &&
           prepared_esdf_->global_guide_generation != guide_generation)) {
        RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "STATIC_ROUTE_REPLAN_REQUEST status=rejected_generation_mismatch "
            "resident_generation=%" PRIu64 " requested_generation=%" PRIu64
            " reason=%s",
            prepared_esdf_ ? prepared_esdf_->global_guide_generation : 0U,
            guide_generation, globalGuideReleaseReasonName(reason));
        return;
      }
      request = std::make_shared<ProductionMppiPreparedEsdf>(*prepared_esdf_);
      request->global_guide_release_reason = reason;
      request->static_route_replan_request = true;
      request->static_route_replan_base_generation =
          prepared_esdf_->global_guide_generation;
      request->static_route_replan_reason = reason;
    }
    {
      const std::scoped_lock queue_lock{guide_queue_mutex_};
      if (pending_guide_world_) {
        RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "STATIC_ROUTE_REPLAN_REQUEST status=deferred_guide_queue_busy "
            "generation=%" PRIu64 " reason=%s",
            request->static_route_replan_base_generation,
            globalGuideReleaseReasonName(reason));
        return;
      }
      if (!static_route_replan_gate_.tryBegin(
              request->static_route_replan_base_generation)) {
        RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "STATIC_ROUTE_REPLAN_REQUEST status=coalesced_gate_rejected "
            "generation=%" PRIu64 " in_flight_generation=%" PRIu64 " reason=%s",
            request->static_route_replan_base_generation,
            static_route_replan_gate_.generation(),
            globalGuideReleaseReasonName(reason));
        return;
      }
      pending_guide_world_ = request;
    }
    guide_queue_condition_.notify_all();
    RCLCPP_INFO(get_logger(),
                "STATIC_ROUTE_REPLAN_REQUEST status=queued generation=%" PRIu64
                " resident_esdf_revision=%" PRIu64 " reason=%s",
                request->static_route_replan_base_generation, request->revision,
                globalGuideReleaseReasonName(reason));
    return;
  }
  guide_release_reason_.store(reason, std::memory_order_relaxed);
  guide_release_generation_.fetch_add(1U, std::memory_order_release);
}

ProductionMppiStability
ProductionMppiNode::compareWithPrevious(const mppi::MppiTickResult& result) const {
  ProductionMppiStability stability;
  if (!previous_result_.has_value() || result.controls.empty() ||
      previous_result_->controls.size() < 2U || result.horizon.empty() ||
      previous_result_->horizon.size() < 2U) {
    return stability;
  }
  const mppi::Control& current = result.controls.front();
  const double offset_steps =
      result.warm_start_shift_s / static_cast<double>(mppi_config_.dynamics.dt_s);
  const std::vector<mppi::Control> shifted_controls =
      mppi::shiftControlSequence(previous_result_->controls, mppi_config_.dynamics.dt_s,
                                 result.warm_start_shift_s);
  const mppi::Control& previous = shifted_controls.front();
  stability.first_control_delta =
      std::hypot(std::hypot(current.ax - previous.ax, current.ay - previous.ay),
                 current.az - previous.az);
  const std::size_t count = result.horizon.size();
  double squared_sum = 0.0;
  for (std::size_t index = 0U; index < count; ++index) {
    const mppi::State previous_state = sampleState(
        previous_result_->horizon, offset_steps + static_cast<double>(index));
    const double difference = distance3(result.horizon[index], previous_state);
    squared_sum += difference * difference;
    stability.position_max_m = std::max(stability.position_max_m, difference);
  }
  stability.position_rms_m = std::sqrt(squared_sum / static_cast<double>(count));
  stability.terminal_shift_m =
      distance3(result.horizon.back(), previous_result_->horizon.back());
  stability.valid = true;
  return stability;
}

} // namespace drone_city_nav
