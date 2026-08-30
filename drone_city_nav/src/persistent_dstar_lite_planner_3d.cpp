#include "drone_city_nav/persistent_dstar_lite_planner_3d.hpp"

#include <algorithm>
#include <bit>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <utility>

#include "persistent_dstar_lite_planner_3d_internal.hpp"

namespace drone_city_nav {
namespace {

[[nodiscard]] bool finitePoint(const Point3& point) noexcept {
  return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

[[nodiscard]] bool finiteVector(const Vec3& vector) noexcept {
  return std::isfinite(vector.x) && std::isfinite(vector.y) && std::isfinite(vector.z);
}

} // namespace

bool PersistentPlannerWorld3D::valid() const noexcept {
  const bool observed = observed_occupancy != nullptr;
  const bool known_static = static_occupancy != nullptr;
  if (observed == known_static || producer_instance_id == 0U || revision == 0U ||
      occupied_fingerprint == 0U) {
    return false;
  }
  const GridBounds3D* const world_bounds = bounds();
  return world_bounds != nullptr && world_bounds->resolution_m > 0.0 &&
         world_bounds->width_cells > 0 && world_bounds->height_cells > 0 &&
         world_bounds->depth_cells > 0;
}

const GridBounds3D* PersistentPlannerWorld3D::bounds() const noexcept {
  if (observed_occupancy != nullptr) {
    return &observed_occupancy->bounds();
  }
  return static_occupancy != nullptr ? &static_occupancy->bounds() : nullptr;
}

bool SpatialRouteCandidate3D::valid() const noexcept {
  return points.size() >= 2U && std::isfinite(path_length_m) && path_length_m > 0.0 &&
         std::isfinite(estimated_execution_time_s) &&
         estimated_execution_time_s > 0.0 &&
         std::isfinite(estimated_translation_time_s) &&
         estimated_translation_time_s >= 0.0 &&
         std::isfinite(estimated_stationary_turn_time_s) &&
         estimated_stationary_turn_time_s >= 0.0;
}

bool PlannerUpdate3D::publishable() const noexcept {
  return input_status == PlannerInputStatus3D::kAccepted &&
         improved_incumbent.has_value() && improved_incumbent->valid();
}

bool PlannerUpdate3D::running() const noexcept {
  return input_status == PlannerInputStatus3D::kAccepted &&
         progress == SearchProgress3D::kRunning;
}

PlannerDispatch3D coordinatePlannerUpdate3D(const PlannerUpdate3D& update) noexcept {
  const bool accepted = update.input_status == PlannerInputStatus3D::kAccepted;
  return PlannerDispatch3D{
      .publish_incumbent = update.publishable(),
      .continue_search = update.running(),
      .terminal = accepted && !update.running(),
  };
}

PersistentDStarLitePlanner3D::PersistentDStarLitePlanner3D(
    PersistentPlannerConfig3D config)
    : implementation_{
          std::make_unique<detail::PersistentDStarLitePlanner3DImpl>(config)} {
}

PersistentDStarLitePlanner3D::~PersistentDStarLitePlanner3D() = default;

PersistentDStarLitePlanner3D::PersistentDStarLitePlanner3D(
    PersistentDStarLitePlanner3D&&) noexcept = default;

PersistentDStarLitePlanner3D& PersistentDStarLitePlanner3D::operator=(
    PersistentDStarLitePlanner3D&&) noexcept = default;

PlannerUpdate3D
PersistentDStarLitePlanner3D::plan(const PersistentPlannerRequest3D& request) {
  return implementation_->plan(request);
}

void PersistentDStarLitePlanner3D::reset() noexcept {
  implementation_->reset();
}

const PersistentPlannerConfig3D& PersistentDStarLitePlanner3D::config() const noexcept {
  return implementation_->config();
}

const char* plannerInputStatus3DName(const PlannerInputStatus3D status) noexcept {
  switch (status) {
    case PlannerInputStatus3D::kAccepted:
      return "accepted";
    case PlannerInputStatus3D::kInvalidInput:
      return "invalid_input";
    case PlannerInputStatus3D::kStartUnavailable:
      return "start_unavailable";
    case PlannerInputStatus3D::kGoalUnavailable:
      return "goal_unavailable";
  }
  return "invalid";
}

const char* searchProgress3DName(const SearchProgress3D progress) noexcept {
  switch (progress) {
    case SearchProgress3D::kRunning:
      return "running";
    case SearchProgress3D::kConverged:
      return "converged";
    case SearchProgress3D::kNoRoute:
      return "no_route";
    case SearchProgress3D::kInvalidated:
      return "invalidated";
  }
  return "invalid";
}

const char*
spatialRouteCandidateSource3DName(const SpatialRouteCandidateSource3D source) noexcept {
  switch (source) {
    case SpatialRouteCandidateSource3D::kFeasibilitySearch:
      return "feasibility_search";
    case SpatialRouteCandidateSource3D::kExecutionTimeRefinement:
      return "execution_time_refinement";
  }
  return "invalid";
}

namespace detail {

void Lattice3D::reset() noexcept {
  raw_bounds_ = {};
  width_ = 0;
  height_ = 0;
  depth_ = 0;
}

void DStarLiteSession3D::reset() noexcept {
  queue_token_ = 0U;
  queue_sequence_ = 0U;
  key_modifier_ = 0.0;
  cost_to_goal_heuristic_admissible_ = true;
  open_ = {};
  records_.clear();
  edge_cost_cache_.clear();
  pending_repair_nodes_.clear();
  pending_repair_members_.clear();
}

void FeasiblePathSearch3D::reset() noexcept {
  initialized_ = false;
  queue_sequence_ = 0U;
  open_ = {};
  costs_.clear();
  parents_.clear();
}

void ExecutionTimeRefiner3D::reset() noexcept {
  initialized_ = false;
  complete_ = false;
  start_from_rest_ = false;
  start_ = {};
  goal_.reset();
  spatial_incumbent_.clear();
  goal_cost_s_ = std::numeric_limits<double>::infinity();
  queue_sequence_ = 0U;
  open_ = {};
  costs_.clear();
  parents_.clear();
}

void AnytimePlannerCoordinator3D::reset() noexcept {
  incumbent_.reset();
}

void AnytimePlannerCoordinator3D::retain(SpatialRouteCandidate3D candidate) {
  if (candidate.valid()) {
    incumbent_ = std::move(candidate);
  } else {
    reset();
  }
}

std::optional<SpatialRouteCandidate3D>
AnytimePlannerCoordinator3D::consider(SpatialRouteCandidate3D candidate) {
  constexpr double kObjectiveTolerance{1.0e-9};
  if (!candidate.valid()) {
    return std::nullopt;
  }
  const bool improved =
      !incumbent_.has_value() ||
      candidate.estimated_execution_time_s <
          incumbent_->estimated_execution_time_s -
              kObjectiveTolerance * std::max({1.0, candidate.estimated_execution_time_s,
                                              incumbent_->estimated_execution_time_s});
  if (!improved) {
    return std::nullopt;
  }
  incumbent_ = std::move(candidate);
  return incumbent_;
}

const SpatialRouteCandidate3D* AnytimePlannerCoordinator3D::incumbent() const noexcept {
  return incumbent_ ? std::addressof(*incumbent_) : nullptr;
}

std::size_t PersistentPlannerNode3DHash::operator()(
    const PersistentPlannerNode3D& node) const noexcept {
  std::size_t seed = std::hash<int>{}(node.x);
  seed ^= std::hash<int>{}(node.y) + 0x9e3779b9U + (seed << 6U) + (seed >> 2U);
  seed ^= std::hash<int>{}(node.z) + 0x9e3779b9U + (seed << 6U) + (seed >> 2U);
  return seed;
}

std::size_t PersistentPlannerEdge3DHash::operator()(
    const PersistentPlannerEdge3D& edge) const noexcept {
  std::size_t seed = PersistentPlannerNode3DHash{}(edge.first);
  const std::size_t second = PersistentPlannerNode3DHash{}(edge.second);
  seed ^= second + 0x9e3779b9U + (seed << 6U) + (seed >> 2U);
  return seed;
}

std::size_t PersistentPlannerTimeState3DHash::operator()(
    const PersistentPlannerTimeState3D& state) const noexcept {
  std::size_t seed = PersistentPlannerNode3DHash{}(state.position);
  const auto mix = [&seed](const std::int8_t value) {
    seed ^= std::hash<int>{}(static_cast<int>(value)) + 0x9e3779b9U + (seed << 6U) +
            (seed >> 2U);
  };
  mix(state.incoming.x);
  mix(state.incoming.y);
  mix(state.incoming.z);
  return seed;
}

bool DStarLiteQueueEntryCompare3D::operator()(
    const DStarLiteQueueEntry3D& first,
    const DStarLiteQueueEntry3D& second) const noexcept {
  if (first.key.first != second.key.first) {
    return first.key.first > second.key.first;
  }
  if (first.key.second != second.key.second) {
    return first.key.second > second.key.second;
  }
  return first.sequence > second.sequence;
}

bool PersistentPlannerTimeQueueEntryCompare3D::operator()(
    const PersistentPlannerTimeQueueEntry3D& first,
    const PersistentPlannerTimeQueueEntry3D& second) const noexcept {
  if (first.estimated_total_s != second.estimated_total_s) {
    return first.estimated_total_s > second.estimated_total_s;
  }
  if (first.cost_from_start_s != second.cost_from_start_s) {
    return first.cost_from_start_s > second.cost_from_start_s;
  }
  return first.sequence > second.sequence;
}

PersistentDStarLitePlanner3DImpl::PersistentDStarLitePlanner3DImpl(
    const PersistentPlannerConfig3D& config)
    : config_{config} {
}

const PersistentPlannerConfig3D&
PersistentDStarLitePlanner3DImpl::config() const noexcept {
  return config_;
}

void PersistentDStarLitePlanner3DImpl::reset() noexcept {
  initialized_ = false;
  world_ = {};
  resident_collision_oracle_.reset();
  departure_collision_oracle_.reset();
  lattice_.reset();
  start_ = {};
  last_start_ = {};
  goal_ = {};
  exact_start_ = {};
  exact_goal_ = {};
  mission_epoch_ = 0U;
  dstar_session_.reset();
  feasibility_search_.reset();
  execution_time_refiner_.reset();
  coordinator_.reset();
  lattice_edge_queries_ = 0U;
  raw_edge_validation_checks_ = 0U;
  adaptive_edge_queries_ = 0U;
  adaptive_edges_in_extracted_path_ = 0U;
  maximum_queried_lattice_level_ = 0U;
}

bool PersistentDStarLitePlanner3DImpl::validRequest(
    const PersistentPlannerRequest3D& request) const {
  return finitePoint(request.start) && finitePoint(request.mission_goal) &&
         finiteVector(request.velocity) && request.mission_epoch != 0U &&
         request.world.valid() && config_.minimum_horizontal_step_m > 0.0 &&
         config_.minimum_vertical_step_m > 0.0 &&
         config_.maximum_adaptive_lattice_level <= 10U && config_.time_model.valid() &&
         std::isfinite(config_.minimum_continuous_turn_alignment) &&
         config_.minimum_continuous_turn_alignment >= -1.0 &&
         config_.minimum_continuous_turn_alignment <= 1.0 &&
         config_.goal_tolerance_m >= 0.0 &&
         config_.connector_search_radius_cells <= 32U &&
         (!config_.feasibility_first_enabled ||
          (config_.maximum_feasibility_expansions_per_update > 0U &&
           std::isfinite(config_.maximum_feasibility_compute_time_ms) &&
           config_.maximum_feasibility_compute_time_ms > 0.0 &&
           config_.maximum_feasibility_compute_time_ms <
               config_.maximum_compute_time_ms)) &&
         config_.maximum_expansions_per_update > 0U &&
         config_.maximum_incremental_changed_voxels > 0U &&
         config_.maximum_extracted_path_nodes > 1U &&
         std::isfinite(config_.maximum_compute_time_ms) &&
         config_.maximum_compute_time_ms > 0.0 &&
         config_.physical_footprint.sweep_step_m > 0.0 &&
         pointInsideFlightEnvelope(request.start) &&
         pointInsideFlightEnvelope(request.mission_goal);
}

PlannerUpdate3D
PersistentDStarLitePlanner3DImpl::plan(const PersistentPlannerRequest3D& request) {
  const auto operation_started = std::chrono::steady_clock::now();
  lattice_edge_queries_ = 0U;
  raw_edge_validation_checks_ = 0U;
  adaptive_edge_queries_ = 0U;
  adaptive_edges_in_extracted_path_ = 0U;
  maximum_queried_lattice_level_ = 0U;
  PlannerUpdate3D update;
  PlannerTelemetry3D& telemetry = update.telemetry;
  telemetry.mission_epoch = request.mission_epoch;
  telemetry.planned_on_revision = request.world.revision;
  telemetry.occupied_fingerprint = request.world.occupied_fingerprint;
  if (!validRequest(request)) {
    return update;
  }
  const auto deadline =
      operation_started +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
          std::chrono::duration<double, std::milli>{config_.maximum_compute_time_ms});

  const std::uint64_t previous_producer = world_.producer_instance_id;
  const std::uint64_t previous_mission_epoch = mission_epoch_;
  const Point3 previous_goal = exact_goal_;
  const auto world_update_started = std::chrono::steady_clock::now();
  const PersistentPlannerWorldUpdate3D world_update = updateWorld(request.world);
  telemetry.world_update_ms =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() -
                                                world_update_started)
          .count();
  if (!world_update.accepted) {
    return update;
  }
  update.input_status = PlannerInputStatus3D::kAccepted;
  telemetry.changed_occupied_voxels = world_update.changed_cells.size();
  telemetry.occupied_world_unchanged = world_update.occupied_world_unchanged;

  const std::optional<PersistentPlannerNode3D> start_anchor =
      selectAnchor(request.start, true);
  if (!start_anchor.has_value()) {
    update.input_status = PlannerInputStatus3D::kStartUnavailable;
    return update;
  }
  const std::optional<PersistentPlannerNode3D> goal_anchor =
      selectAnchor(request.mission_goal, false);
  if (!goal_anchor.has_value()) {
    update.input_status = PlannerInputStatus3D::kGoalUnavailable;
    return update;
  }
  const PersistentPlannerTimeState3D time_start_state =
      executionTimeStartState(request, *start_anchor);
  const bool starts_from_rest =
      std::hypot(std::hypot(request.velocity.x, request.velocity.y),
                 request.velocity.z) <= 1.0e-9;

  const bool mission_changed =
      initialized_ &&
      (request.mission_epoch != previous_mission_epoch ||
       distance3D(request.mission_goal, previous_goal) > config_.goal_tolerance_m ||
       *goal_anchor != goal_);
  const bool producer_changed = previous_producer != 0U &&
                                previous_producer != request.world.producer_instance_id;
  if (!initialized_ || world_update.requires_reset || mission_changed) {
    if (mission_changed || producer_changed) {
      coordinator_.reset();
    }
    initializeSearch(request, *start_anchor, *goal_anchor);
  } else {
    const bool feasibility_start_changed = *start_anchor != start_;
    const bool execution_time_start_changed =
        execution_time_refiner_.initialized_ &&
        (execution_time_refiner_.start_ != time_start_state ||
         execution_time_refiner_.start_from_rest_ != starts_from_rest);
    const bool execution_time_goal_changed =
        execution_time_refiner_.initialized_ &&
        distance3D(request.mission_goal, previous_goal) > 1.0e-9;
    telemetry.search_state_reused = true;
    exact_start_ = request.start;
    exact_goal_ = request.mission_goal;
    dstar_session_.key_modifier_ += heuristic(last_start_, *start_anchor);
    start_ = *start_anchor;
    last_start_ = *start_anchor;
    if (!world_update.changed_cells.empty()) {
      scheduleAffectedVertices(world_update.changed_cells,
                               telemetry.affected_lattice_states);
      if (world_update.occupied_cells_removed) {
        // A cost-to-go retained across an obstacle removal can overestimate a
        // newly opened route until every affected label settles. Keep the
        // refinement heuristic strictly Euclidean in that case.
        dstar_session_.cost_to_goal_heuristic_admissible_ = false;
      }
      dstar_session_.repair_generation_ =
          dstar_session_.repair_generation_ == std::numeric_limits<std::uint64_t>::max()
              ? 1U
              : dstar_session_.repair_generation_ + 1U;
    }
    if (!world_update.changed_cells.empty() || feasibility_start_changed) {
      resetFeasibilitySearch();
    }
    if (!world_update.changed_cells.empty() || execution_time_start_changed ||
        execution_time_goal_changed) {
      resetExecutionTimeSearch();
    }
  }

  if (const SpatialRouteCandidate3D* const incumbent = coordinator_.incumbent();
      incumbent != nullptr) {
    const std::optional<std::vector<Point3>> retained =
        rebaseIncumbent(incumbent->points, request.start, request.mission_goal);
    const SpatialRouteCandidateSource3D source = incumbent->source;
    std::optional<SpatialRouteCandidate3D> rebased =
        retained ? makeCandidate(*retained, source, request.velocity) : std::nullopt;
    if (rebased) {
      coordinator_.retain(std::move(*rebased));
      telemetry.incumbent_retained = true;
    } else {
      coordinator_.reset();
    }
  }

  if (config_.feasibility_first_enabled && coordinator_.incumbent() == nullptr) {
    telemetry.feasibility_attempted = true;
    const auto feasibility_deadline = std::min(
        deadline, std::chrono::steady_clock::now() +
                      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                          std::chrono::duration<double, std::milli>{
                              config_.maximum_feasibility_compute_time_ms}));
    std::optional<std::vector<Point3>> path = findFeasiblePath(
        feasibility_deadline, config_.maximum_feasibility_expansions_per_update,
        telemetry.feasibility_expansions);
    std::optional<SpatialRouteCandidate3D> candidate =
        path ? makeCandidate(std::move(*path),
                             SpatialRouteCandidateSource3D::kFeasibilitySearch,
                             request.velocity)
             : std::nullopt;
    if (candidate) {
      telemetry.feasibility_route_found = true;
      update.improved_incumbent = coordinator_.consider(std::move(*candidate));
    }
  }

  const bool repair_complete =
      continueAffectedVertexRepair(deadline, config_.maximum_expansions_per_update,
                                   telemetry.repair_lattice_states_processed);
  telemetry.repair_lattice_states_pending = dstar_session_.pending_repair_nodes_.size();
  telemetry.repair_pending = !repair_complete;
  const std::size_t remaining_spatial_expansions =
      telemetry.repair_lattice_states_processed < config_.maximum_expansions_per_update
          ? config_.maximum_expansions_per_update -
                telemetry.repair_lattice_states_processed
          : 0U;
  bool spatial_search_complete = repair_complete && shortestPathComplete();
  if (!spatial_search_complete && repair_complete &&
      remaining_spatial_expansions > 0U) {
    spatial_search_complete = computeShortestPath(
        deadline, remaining_spatial_expansions, telemetry.expansions);
  }
  bool spatial_route_available{false};
  const auto spatial_start_record = dstar_session_.records_.find(start_);
  spatial_route_available = spatial_search_complete &&
                            spatial_start_record != dstar_session_.records_.end() &&
                            std::isfinite(spatial_start_record->second.g);
  if (spatial_route_available) {
    if (!execution_time_refiner_.initialized_) {
      initializeExecutionTimeSearch(request, time_start_state);
    }
    const std::size_t graph_expansions =
        telemetry.repair_lattice_states_processed + telemetry.expansions;
    const std::size_t refinement_budget =
        graph_expansions < config_.maximum_expansions_per_update
            ? config_.maximum_expansions_per_update - graph_expansions
            : 0U;
    std::optional<std::vector<Point3>> path = continueExecutionTimeSearch(
        deadline, refinement_budget, telemetry.execution_time_search_expansions);
    if (execution_time_refiner_.complete_ && path && !path->empty()) {
      *path = path_postprocessor_.shortcut(
          *path,
          PathPostprocessorContext3D{
              .maximum_shortcut_checks = config_.maximum_shortcut_checks,
              .segment_valid =
                  [this](const Point3& first, const Point3& second,
                         const bool departure) {
                    return departure ? departureSegmentValid(first, second)
                                     : rawSegmentValid(first, second);
                  },
              .time_profile =
                  [this, &request](const std::vector<Point3>& points) {
                    return pathTimeProfile(points, request.velocity);
                  },
          },
          telemetry.shortcut_checks, telemetry.shortcuts_applied);
    }
    if (path && path->size() >= 2U &&
        distance3D(path->back(), request.mission_goal) <= config_.goal_tolerance_m) {
      std::optional<SpatialRouteCandidate3D> refined = makeCandidate(
          std::move(*path), SpatialRouteCandidateSource3D::kExecutionTimeRefinement,
          request.velocity);
      if (refined) {
        std::optional<SpatialRouteCandidate3D> improvement =
            coordinator_.consider(std::move(*refined));
        if (improvement) {
          update.improved_incumbent = std::move(improvement);
        }
      }
    }
  } else if (spatial_search_complete) {
    resetExecutionTimeSearch();
  }

  const bool search_complete =
      spatial_search_complete &&
      (!spatial_route_available || execution_time_refiner_.complete_);
  telemetry.incumbent_available = coordinator_.incumbent() != nullptr;
  if (!search_complete) {
    update.progress = SearchProgress3D::kRunning;
  } else if (telemetry.incumbent_available) {
    update.progress = SearchProgress3D::kConverged;
  } else {
    update.progress = SearchProgress3D::kNoRoute;
  }
  telemetry.execution_time_search_complete = execution_time_refiner_.complete_;
  telemetry.search_generation = dstar_session_.search_generation_;
  telemetry.repair_generation = dstar_session_.repair_generation_;
  telemetry.records = dstar_session_.records_.size();
  telemetry.open_entries = dstar_session_.open_.size();
  telemetry.lattice_edge_queries = lattice_edge_queries_;
  telemetry.raw_edge_validation_checks = raw_edge_validation_checks_;
  telemetry.adaptive_edge_queries = adaptive_edge_queries_;
  telemetry.adaptive_edges_in_extracted_path = adaptive_edges_in_extracted_path_;
  telemetry.maximum_queried_lattice_level = maximum_queried_lattice_level_;
  telemetry.execution_time_search_records = execution_time_refiner_.costs_.size();
  telemetry.execution_time_search_open_entries = execution_time_refiner_.open_.size();
  telemetry.execution_time_search_objective_s =
      std::isfinite(execution_time_refiner_.goal_cost_s_)
          ? execution_time_refiner_.goal_cost_s_
          : 0.0;
  telemetry.search_ms = std::chrono::duration<double, std::milli>(
                            std::chrono::steady_clock::now() - operation_started)
                            .count();
  return update;
}

} // namespace detail
} // namespace drone_city_nav
