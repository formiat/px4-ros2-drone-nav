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
#include <utility>

#include "production_mppi_node.hpp"
#include "production_mppi_route_helpers.hpp"

namespace drone_city_nav {
namespace {

constexpr std::size_t kMaximumFailureMemoryEntries{64U};

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
  StaticRouteObjective active_route_objective;
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
    search_config.minimum_frontier_guide_length_m = std::max(
        search_config.minimum_frontier_guide_length_m,
        active_guide_config_.minimum_remaining_m + search_config.primitive_length_m);
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
    const std::shared_ptr<const ProductionNavigationObjective> current_objective =
        navigationObjective();
    const Point3 mission_goal = world->search_objective.available
                                    ? world->search_objective.goal
                                    : mission_goal_;
    const std::uint64_t required_objective_epoch =
        minimum_tracking_route_mission_epoch_.load(std::memory_order_acquire);
    const std::uint64_t required_objective_sample =
        current_objective &&
                current_objective->mission_epoch == required_objective_epoch
            ? minimum_tracking_route_sample_sequence_.load(std::memory_order_acquire)
            : 0U;
    const bool world_objective_matches =
        current_objective &&
        staticRouteObjectiveMatches(
            world->search_objective, makeStaticRouteObjective(*current_objective),
            required_objective_sample, std::numeric_limits<double>::infinity());
    const auto candidate_objective_matches =
        [&](const GlobalGuideCandidate& candidate) {
          return current_objective &&
                 staticRouteObjectiveMatches(
                     StaticRouteObjective{
                         .goal = candidate.objective_goal,
                         .mission_epoch = candidate.objective_mission_epoch,
                         .sample_sequence = candidate.objective_sample_sequence,
                         .continuous_tracking = candidate.objective_continuous_tracking,
                         .available = candidate.objective_available},
                     makeStaticRouteObjective(*current_objective),
                     required_objective_sample,
                     std::numeric_limits<double>::infinity());
        };
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
    std::optional<GlobalGuideCandidate> activated_candidate;
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
            ProductionGuideCandidateValidation validation =
                validateGuideCandidateOnLatestWorld(candidate, reaches_mission_goal);
            raw_validation = validation.raw_validation;
            candidate_validation_status = validation.status;
            validation_revision = validation.validation_revision;
            candidate_validation_position = validation.validation_position;
            latest_world_rejected_candidate = !validation.accepted;
            if (validation.accepted) {
              publication_world = std::move(validation.publication_world);
            }
            return validation.accepted;
          };
      const std::shared_ptr<const std::vector<Point2>> previous_active_guide =
          active_guide_lifecycle_->guide();
      guide_update = active_guide_lifecycle_->update(
          grid, *host_distances, position,
          guide_release_generation_.load(std::memory_order_acquire),
          guide_release_reason_.load(std::memory_order_relaxed));
      if (!guide_update.active && previous_active_guide &&
          previous_active_guide->size() >= 2U &&
          guide_update.release_reason == GlobalGuideReleaseReason::kBlocked &&
          world->raw_occupancy) {
        const std::shared_ptr<const ProductionMppiRawWorld2D> latest_raw_world =
            latest_raw_world_.load(std::memory_order_acquire);
        const std::shared_ptr<const OccupancyGrid2D> validation_occupancy =
            latest_raw_world && latest_raw_world->occupancy
                ? latest_raw_world->occupancy
                : world->raw_occupancy;
        const GlobalGuideProjection release_projection =
            projectOntoGlobalGuide(*previous_active_guide, position);
        const double validation_station_m =
            release_projection.valid ? release_projection.station_m : 0.0;
        const RawGuideValidationResult blocked_validation =
            validateGuideAgainstRawOccupancy(
                *previous_active_guide, *validation_occupancy,
                active_guide_config_.validation_sample_step_m, validation_station_m);
        const Point2 failure_point =
            blocked_validation.accepted ? position : blocked_validation.failure_point;
        const GlobalGuideProjection failure_projection =
            projectOntoGlobalGuide(*previous_active_guide, failure_point);
        const Point2 failure_tangent =
            failure_projection.valid ? failure_projection.tangent
                                     : Point2{navigation.state.vx, navigation.state.vy};
        frontier_blacklist_.push_back(LatticeFrontierBlacklistEntry{
            .failure_point = failure_point,
            .approach_heading_rad = std::atan2(failure_tangent.y, failure_tangent.x),
            .expires_at_ns = blacklist_now_ns + static_cast<std::int64_t>(
                                                    frontier_blacklist_ttl_s_ * 1.0e9),
            .soft_penalty_cost = no_static_soft_tabu_penalty_,
        });
      }
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
      if (frontier_blacklist_.size() > kMaximumFailureMemoryEntries) {
        frontier_blacklist_.erase(
            frontier_blacklist_.begin(),
            frontier_blacklist_.begin() +
                static_cast<std::ptrdiff_t>(frontier_blacklist_.size() -
                                            kMaximumFailureMemoryEntries));
      }
      if (guide_update.active) {
        guide = active_guide_lifecycle_->guide();
        guide_heading.source = GlobalGuideHeadingSource::kActiveGuide;
        if (guide_update.requires_replan || search_session_in_progress) {
          if (!search_heading.has_value()) {
            search_heading = active_guide_lifecycle_->selectPlanningHeading(
                navigation.state, Point2{mission_goal.x, mission_goal.y});
          }
          guide_heading = *search_heading;
          lattice_observation = planRiskAwareMotionPrimitiveGuide(
              grid, *host_distances, position, guide_heading.heading_rad,
              Point2{mission_goal.x, mission_goal.y}, search_config,
              frontier_blacklist_, &search_session, planning_worker_pool_.get());
          lattice_search_performed = true;
          const auto candidate =
              std::make_shared<const std::vector<Point2>>(lattice_observation.guide);
          const bool reaches_mission_goal =
              lattice_observation.status == LatticePlanStatus::kReachedPlanningGoal &&
              lattice_observation.reached_mission_goal &&
              lattice_observation.exact_terminal_connector && !candidate->empty() &&
              distance(candidate->back(), Point2{mission_goal.x, mission_goal.y}) <=
                  1.0e-6;
          const bool executable =
              lattice_observation.status == LatticePlanStatus::kReachedPlanningGoal ||
              lattice_observation.status == LatticePlanStatus::kViableFrontier ||
              lattice_observation.status == LatticePlanStatus::kRawSafeDetourPrefix;
          if (executable && world_objective_matches &&
              validate_candidate_on_latest_world(candidate, reaches_mission_goal)) {
            const GlobalGuideCandidate pending{
                .guide = candidate,
                .base_generation = guide_update.generation,
                .search_revision = world->revision,
                .objective_goal = world->search_objective.goal,
                .objective_mission_epoch = world->search_objective.mission_epoch,
                .objective_sample_sequence = world->search_objective.sample_sequence,
                .objective_continuous_tracking =
                    world->search_objective.continuous_tracking,
                .objective_available = world->search_objective.available,
                .reaches_mission_goal = reaches_mission_goal,
                .status = lattice_observation.status,
                .expansions = lattice_observation.expansions,
                .guide_length_m = lattice_observation.guide_length_m,
                .endpoint_displacement_m =
                    lattice_observation.frontier_endpoint_displacement_m,
                .reachable_depth_m = lattice_observation.reachable_depth_m,
                .remaining_goal_distance_m =
                    lattice_observation.remaining_goal_distance_m,
                .cost = lattice_observation.cost,
                .fingerprint = routeFingerprint(*candidate),
            };
            if (!pending_global_guide_ ||
                pending_global_guide_->base_generation != guide_update.generation ||
                betterGlobalGuideCandidate(pending, *pending_global_guide_)) {
              pending_global_guide_ = pending;
            }
            if (guide_update.release_reason == GlobalGuideReleaseReason::kExhausted) {
              const GlobalGuideCandidate selected = *pending_global_guide_;
              if (globalGuideCandidateReadyForActivation(
                      selected, false, search_config.minimum_frontier_guide_length_m,
                      search_config.minimum_frontier_endpoint_displacement_m)) {
                if (candidate_objective_matches(selected) &&
                    validate_candidate_on_latest_world(selected.guide,
                                                       selected.reaches_mission_goal)) {
                  guide_acceptance = active_guide_lifecycle_->accept(
                      selected.guide, selected.reaches_mission_goal,
                      publication_world->grid, *publication_world->distances_m,
                      candidate_validation_position);
                }
                pending_global_guide_.reset();
                if (guide_acceptance.accepted) {
                  activated_candidate = selected;
                  guide = active_guide_lifecycle_->guide();
                  guide_update = active_guide_lifecycle_->status();
                  active_guide_expansions = selected.expansions;
                  active_guide_cost = selected.cost;
                }
              }
            }
          }
        }
      } else {
        if (pending_global_guide_ && pending_global_guide_->guide) {
          const GlobalGuideCandidate selected = *pending_global_guide_;
          const bool ready = globalGuideCandidateReadyForActivation(
              selected, true, search_config.minimum_frontier_guide_length_m,
              search_config.minimum_frontier_endpoint_displacement_m);
          if (selected.base_generation == guide_update.generation && ready &&
              candidate_objective_matches(selected) &&
              validate_candidate_on_latest_world(selected.guide,
                                                 selected.reaches_mission_goal)) {
            guide_acceptance = active_guide_lifecycle_->accept(
                selected.guide, selected.reaches_mission_goal, publication_world->grid,
                *publication_world->distances_m, candidate_validation_position);
            if (guide_acceptance.accepted) {
              activated_candidate = selected;
              active_guide_expansions = selected.expansions;
              active_guide_cost = selected.cost;
            }
          }
          if (selected.base_generation != guide_update.generation || ready) {
            pending_global_guide_.reset();
          }
        }
        if (guide_acceptance.accepted) {
          guide = active_guide_lifecycle_->guide();
          guide_update = active_guide_lifecycle_->status();
        } else {
          if (!search_heading.has_value()) {
            search_heading = active_guide_lifecycle_->selectPlanningHeading(
                navigation.state, Point2{mission_goal.x, mission_goal.y});
          }
          guide_heading = *search_heading;
          lattice_observation = planRiskAwareMotionPrimitiveGuide(
              grid, *host_distances, position, guide_heading.heading_rad,
              Point2{mission_goal.x, mission_goal.y}, search_config,
              frontier_blacklist_, &search_session, planning_worker_pool_.get());
          lattice_search_performed = true;
          const auto candidate = std::make_shared<const std::vector<Point2>>(
              std::move(lattice_observation.guide));
          const bool reaches_mission_goal =
              lattice_observation.status == LatticePlanStatus::kReachedPlanningGoal &&
              lattice_observation.reached_mission_goal &&
              lattice_observation.exact_terminal_connector && !candidate->empty() &&
              distance(candidate->back(), Point2{mission_goal.x, mission_goal.y}) <=
                  1.0e-6;
          const bool candidate_available =
              lattice_observation.status == LatticePlanStatus::kReachedPlanningGoal ||
              lattice_observation.status == LatticePlanStatus::kViableFrontier ||
              lattice_observation.status == LatticePlanStatus::kRawSafeDetourPrefix;
          if (candidate_available && world_objective_matches &&
              validate_candidate_on_latest_world(candidate, reaches_mission_goal)) {
            const GlobalGuideCandidate pending{
                .guide = candidate,
                .base_generation = guide_update.generation,
                .search_revision = world->revision,
                .objective_goal = world->search_objective.goal,
                .objective_mission_epoch = world->search_objective.mission_epoch,
                .objective_sample_sequence = world->search_objective.sample_sequence,
                .objective_continuous_tracking =
                    world->search_objective.continuous_tracking,
                .objective_available = world->search_objective.available,
                .reaches_mission_goal = reaches_mission_goal,
                .status = lattice_observation.status,
                .expansions = lattice_observation.expansions,
                .guide_length_m = lattice_observation.guide_length_m,
                .endpoint_displacement_m =
                    lattice_observation.frontier_endpoint_displacement_m,
                .reachable_depth_m = lattice_observation.reachable_depth_m,
                .remaining_goal_distance_m =
                    lattice_observation.remaining_goal_distance_m,
                .cost = lattice_observation.cost,
                .fingerprint = routeFingerprint(*candidate),
            };
            if (!pending_global_guide_ ||
                pending_global_guide_->base_generation != guide_update.generation ||
                betterGlobalGuideCandidate(pending, *pending_global_guide_)) {
              pending_global_guide_ = pending;
            }
            if (globalGuideCandidateReadyForActivation(
                    *pending_global_guide_, true,
                    search_config.minimum_frontier_guide_length_m,
                    search_config.minimum_frontier_endpoint_displacement_m)) {
              const GlobalGuideCandidate selected = *pending_global_guide_;
              if (candidate_objective_matches(selected) &&
                  validate_candidate_on_latest_world(selected.guide,
                                                     selected.reaches_mission_goal)) {
                guide_acceptance = active_guide_lifecycle_->accept(
                    selected.guide, selected.reaches_mission_goal,
                    publication_world->grid, *publication_world->distances_m,
                    candidate_validation_position);
              }
              pending_global_guide_.reset();
              if (guide_acceptance.accepted) {
                activated_candidate = selected;
                active_guide_expansions = selected.expansions;
                active_guide_cost = selected.cost;
              }
            }
          }
          if (guide_acceptance.accepted) {
            guide = active_guide_lifecycle_->guide();
            guide_update = active_guide_lifecycle_->status();
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
                                         Point2{mission_goal.x, mission_goal.y}),
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
      active_route_objective = {};
    } else if (guide.get() != route_source.get()) {
      mppi_route = makeMppiRoute2D(*guide, mission_goal.z,
                                   speed_policy_config_.cruise_speed_mps);
      route_source = guide;
    }
    if (activated_candidate.has_value()) {
      active_route_objective = StaticRouteObjective{
          .goal = activated_candidate->objective_goal,
          .mission_epoch = activated_candidate->objective_mission_epoch,
          .sample_sequence = activated_candidate->objective_sample_sequence,
          .continuous_tracking = activated_candidate->objective_continuous_tracking,
          .available = activated_candidate->objective_available,
      };
    }
    ProductionMppiPreparedEsdf prepared = *publication_world;
    prepared.mppi_route = mppi_route;
    prepared.route_2d_projection = guide;
    prepared.route_objective = active_route_objective;
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
                 lattice_observation.planning_goal.y, mission_goal.z};
      prepared.planning_candidate_endpoint =
          lattice_observation.guide.empty()
              ? prepared.planning_search_start
              : Point3{lattice_observation.guide.back().x,
                       lattice_observation.guide.back().y, mission_goal.z};
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
    prepared.lattice_successor_profiling = lattice_observation.successor_profiling;
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
        prepared.source_stamp_ns =
            std::max(prepared.source_stamp_ns, prepared_esdf_->source_stamp_ns);
        prepared.ready_stamp_ns =
            std::max(prepared.ready_stamp_ns, prepared_esdf_->ready_stamp_ns);
        if (const std::shared_ptr<const ProductionMppiRawWorld2D> raw_world =
                latest_raw_world_.load(std::memory_order_acquire);
            raw_world && raw_world->occupancy) {
          prepared.raw_occupancy = raw_world->occupancy;
        }
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
        "successor_search_batches=%zu successor_search_candidates=%zu "
        "successor_search_batch_max=%zu successor_search_worker_ms=%.3f "
        "expansion_prefetch_batches=%zu expansion_prefetch_entries=%zu "
        "expansion_prefetch_parallel_entries=%zu expansion_prefetch_cache_hits=%zu "
        "expansion_prefetch_discarded_entries=%zu "
        "expansion_prefetch_worker_ms=%.3f "
        "successor_continuation_batches=%zu "
        "successor_continuation_candidates=%zu "
        "successor_continuation_batch_max=%zu "
        "successor_continuation_worker_ms=%.3f "
        "search_session_age_ms=%.2f cycle_detected=%s adaptive_search=%s "
        "soft_tabu_entries=%zu selected_candidate_status=%s "
        "selected_candidate_base_generation=%" PRIu64
        " selected_candidate_search_revision=%" PRIu64
        " selected_candidate_guide_length_m=%.2f "
        "selected_candidate_endpoint_displacement_m=%.2f",
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
        prepared.lattice_successor_profiling.search.collection_calls,
        prepared.lattice_successor_profiling.search.candidates,
        prepared.lattice_successor_profiling.search.maximum_candidates,
        prepared.lattice_successor_profiling.search.worker_ms,
        prepared.lattice_successor_profiling.expansion_prefetch.batches,
        prepared.lattice_successor_profiling.expansion_prefetch.entries,
        prepared.lattice_successor_profiling.expansion_prefetch.parallel_entries,
        prepared.lattice_successor_profiling.expansion_prefetch.cache_hits,
        prepared.lattice_successor_profiling.expansion_prefetch.discarded_entries,
        prepared.lattice_successor_profiling.expansion_prefetch.worker_ms,
        prepared.lattice_successor_profiling.continuation.collection_calls,
        prepared.lattice_successor_profiling.continuation.candidates,
        prepared.lattice_successor_profiling.continuation.maximum_candidates,
        prepared.lattice_successor_profiling.continuation.worker_ms,
        prepared.lattice_search_session_age_ms,
        prepared.no_static_cycle_detected ? "true" : "false",
        prepared.no_static_adaptive_search ? "true" : "false",
        prepared.no_static_soft_tabu_entries,
        latticePlanStatusName(activated_candidate ? activated_candidate->status
                                                  : LatticePlanStatus::kInvalidInput),
        activated_candidate ? activated_candidate->base_generation : 0U,
        activated_candidate ? activated_candidate->search_revision : 0U,
        activated_candidate ? activated_candidate->guide_length_m : 0.0,
        activated_candidate ? activated_candidate->endpoint_displacement_m : 0.0);

    const bool search_incomplete =
        lattice_search_performed && !lattice_observation.search_session_complete;
    if (search_incomplete && !guide_acceptance.accepted &&
        !latest_world_rejected_candidate) {
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
  const std::shared_ptr<const ProductionNavigationObjective> objective =
      navigationObjective();
  const Point3 mission_goal = objective ? objective->goal : mission_goal_;
  mppi::State target{static_cast<float>(mission_goal.x),
                     static_cast<float>(mission_goal.y),
                     static_cast<float>(mission_goal.z)};
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

void ProductionMppiNode::requestGuideRelease(const GlobalGuideReleaseReason reason,
                                             const std::uint64_t guide_generation) {
  if (use_static_map_) {
    requestStaticRouteReplan(reason, guide_generation);
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
