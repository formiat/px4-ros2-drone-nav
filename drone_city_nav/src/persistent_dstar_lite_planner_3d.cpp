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

bool PersistentPlannerResult3D::executable() const noexcept {
  return status == PersistentPlannerStatus3D::kReachedMissionGoal &&
         points.size() >= 2U;
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

PersistentPlannerResult3D
PersistentDStarLitePlanner3D::plan(const PersistentPlannerRequest3D& request) {
  return implementation_->plan(request);
}

void PersistentDStarLitePlanner3D::reset() noexcept {
  implementation_->reset();
}

const PersistentPlannerConfig3D& PersistentDStarLitePlanner3D::config() const noexcept {
  return implementation_->config();
}

const char*
persistentPlannerStatus3DName(const PersistentPlannerStatus3D status) noexcept {
  switch (status) {
    case PersistentPlannerStatus3D::kInvalidInput:
      return "invalid_input";
    case PersistentPlannerStatus3D::kReachedMissionGoal:
      return "reached_mission_goal";
    case PersistentPlannerStatus3D::kSearchInProgress:
      return "search_in_progress";
    case PersistentPlannerStatus3D::kNoRoute:
      return "no_route";
    case PersistentPlannerStatus3D::kStartUnavailable:
      return "start_unavailable";
    case PersistentPlannerStatus3D::kGoalUnavailable:
      return "goal_unavailable";
  }
  return "invalid";
}

namespace detail {

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
  raw_bounds_ = {};
  width_ = 0;
  height_ = 0;
  depth_ = 0;
  start_ = {};
  last_start_ = {};
  goal_ = {};
  exact_start_ = {};
  exact_goal_ = {};
  mission_epoch_ = 0U;
  key_modifier_ = 0.0;
  dstar_cost_to_goal_heuristic_admissible_ = true;
  queue_token_ = 0U;
  queue_sequence_ = 0U;
  open_ = {};
  records_.clear();
  edge_cost_cache_.clear();
  pending_repair_nodes_.clear();
  pending_repair_members_.clear();
  resetExecutionTimeSearch();
  incumbent_.clear();
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
         config_.maximum_expansions_per_update > 0U &&
         config_.maximum_incremental_changed_voxels > 0U &&
         config_.maximum_extracted_path_nodes > 1U &&
         std::isfinite(config_.maximum_compute_time_ms) &&
         config_.maximum_compute_time_ms > 0.0 &&
         config_.physical_footprint.sweep_step_m > 0.0 &&
         pointInsideFlightEnvelope(request.start) &&
         pointInsideFlightEnvelope(request.mission_goal);
}

PersistentPlannerResult3D
PersistentDStarLitePlanner3DImpl::plan(const PersistentPlannerRequest3D& request) {
  const auto operation_started = std::chrono::steady_clock::now();
  lattice_edge_queries_ = 0U;
  raw_edge_validation_checks_ = 0U;
  adaptive_edge_queries_ = 0U;
  adaptive_edges_in_extracted_path_ = 0U;
  maximum_queried_lattice_level_ = 0U;
  PersistentPlannerResult3D result{
      .status = PersistentPlannerStatus3D::kInvalidInput,
      .points = {},
      .mission_epoch = request.mission_epoch,
      .planned_on_revision = request.world.revision,
      .occupied_fingerprint = request.world.occupied_fingerprint,
  };
  if (!validRequest(request)) {
    return result;
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
  result.world_update_ms = std::chrono::duration<double, std::milli>(
                               std::chrono::steady_clock::now() - world_update_started)
                               .count();
  if (!world_update.accepted) {
    return result;
  }
  result.changed_occupied_voxels = world_update.changed_cells.size();
  result.occupied_world_unchanged = world_update.occupied_world_unchanged;

  const std::optional<PersistentPlannerNode3D> start_anchor =
      selectAnchor(request.start, true);
  if (!start_anchor.has_value()) {
    result.status = PersistentPlannerStatus3D::kStartUnavailable;
    return result;
  }
  const std::optional<PersistentPlannerNode3D> goal_anchor =
      selectAnchor(request.mission_goal, false);
  if (!goal_anchor.has_value()) {
    result.status = PersistentPlannerStatus3D::kGoalUnavailable;
    return result;
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
      incumbent_.clear();
    }
    initializeSearch(request, *start_anchor, *goal_anchor);
  } else {
    const bool execution_time_start_changed =
        execution_time_search_initialized_ &&
        (execution_time_start_ != time_start_state ||
         execution_time_start_from_rest_ != starts_from_rest);
    const bool execution_time_goal_changed =
        execution_time_search_initialized_ &&
        distance3D(request.mission_goal, previous_goal) > 1.0e-9;
    result.search_state_reused = true;
    exact_start_ = request.start;
    exact_goal_ = request.mission_goal;
    key_modifier_ += heuristic(last_start_, *start_anchor);
    start_ = *start_anchor;
    last_start_ = *start_anchor;
    if (!world_update.changed_cells.empty()) {
      scheduleAffectedVertices(world_update.changed_cells,
                               result.affected_lattice_states);
      if (world_update.occupied_cells_removed) {
        // A cost-to-go retained across an obstacle removal can overestimate a
        // newly opened route until every affected label settles. Keep the
        // refinement heuristic strictly Euclidean in that case.
        dstar_cost_to_goal_heuristic_admissible_ = false;
      }
      repair_generation_ =
          repair_generation_ == std::numeric_limits<std::uint64_t>::max()
              ? 1U
              : repair_generation_ + 1U;
    }
    if (!world_update.changed_cells.empty() || execution_time_start_changed ||
        execution_time_goal_changed) {
      resetExecutionTimeSearch();
    }
  }

  const bool repair_complete =
      continueAffectedVertexRepair(deadline, config_.maximum_expansions_per_update,
                                   result.repair_lattice_states_processed);
  result.repair_lattice_states_pending = pending_repair_nodes_.size();
  result.repair_pending = !repair_complete;
  const std::size_t remaining_expansions =
      result.repair_lattice_states_processed < config_.maximum_expansions_per_update
          ? config_.maximum_expansions_per_update -
                result.repair_lattice_states_processed
          : 0U;
  const bool spatial_search_complete =
      repair_complete && remaining_expansions > 0U &&
      computeShortestPath(deadline, remaining_expansions, result.expansions);
  const auto spatial_start_record = records_.find(start_);
  const bool spatial_route_available = spatial_search_complete &&
                                       spatial_start_record != records_.end() &&
                                       std::isfinite(spatial_start_record->second.g);
  if (spatial_route_available) {
    if (!execution_time_search_initialized_) {
      initializeExecutionTimeSearch(request, time_start_state);
    }
    std::optional<std::vector<Point3>> path =
        continueExecutionTimeSearch(deadline, config_.maximum_expansions_per_update,
                                    result.execution_time_search_expansions);
    if (path.has_value() && !path->empty()) {
      *path = shortcutPath(*path, result.shortcut_checks, result.shortcuts_applied,
                           request.velocity);
    }
    if (path.has_value() && path->size() >= 2U && pathRawValid(*path) &&
        distance3D(path->back(), request.mission_goal) <= config_.goal_tolerance_m) {
      result.status = PersistentPlannerStatus3D::kReachedMissionGoal;
      result.points = std::move(*path);
      incumbent_ = result.points;
    } else if (execution_time_search_complete_) {
      result.status = PersistentPlannerStatus3D::kNoRoute;
    }
  } else if (spatial_search_complete) {
    resetExecutionTimeSearch();
    result.status = PersistentPlannerStatus3D::kNoRoute;
  }

  if (result.status == PersistentPlannerStatus3D::kInvalidInput) {
    const std::optional<std::vector<Point3>> retained =
        rebaseIncumbent(request.start, request.mission_goal);
    if (retained.has_value()) {
      result.status = PersistentPlannerStatus3D::kReachedMissionGoal;
      result.points = *retained;
      result.incumbent_retained = true;
    } else {
      result.status = PersistentPlannerStatus3D::kSearchInProgress;
    }
  }

  result.execution_time_search_complete = execution_time_search_complete_;
  result.search_complete = spatial_search_complete && (!spatial_route_available ||
                                                       execution_time_search_complete_);
  result.search_generation = search_generation_;
  result.repair_generation = repair_generation_;
  result.records = records_.size();
  result.open_entries = open_.size();
  result.lattice_edge_queries = lattice_edge_queries_;
  result.raw_edge_validation_checks = raw_edge_validation_checks_;
  result.adaptive_edge_queries = adaptive_edge_queries_;
  result.adaptive_edges_in_extracted_path = adaptive_edges_in_extracted_path_;
  result.maximum_queried_lattice_level = maximum_queried_lattice_level_;
  result.execution_time_search_records = execution_time_costs_.size();
  result.execution_time_search_open_entries = execution_time_open_.size();
  result.execution_time_search_objective_s =
      std::isfinite(execution_time_goal_cost_s_) ? execution_time_goal_cost_s_ : 0.0;
  populatePathMetrics(result, request.velocity);
  result.search_ms = std::chrono::duration<double, std::milli>(
                         std::chrono::steady_clock::now() - operation_started)
                         .count();
  return result;
}

} // namespace detail
} // namespace drone_city_nav
