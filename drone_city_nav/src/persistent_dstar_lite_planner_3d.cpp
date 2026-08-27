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
  queue_token_ = 0U;
  queue_sequence_ = 0U;
  open_ = {};
  records_.clear();
  edge_cost_cache_.clear();
  incumbent_.clear();
}

bool PersistentDStarLitePlanner3DImpl::validRequest(
    const PersistentPlannerRequest3D& request) const {
  return finitePoint(request.start) && finitePoint(request.mission_goal) &&
         finiteVector(request.velocity) && request.mission_epoch != 0U &&
         request.world.valid() && config_.horizontal_step_m > 0.0 &&
         config_.vertical_step_m > 0.0 && config_.nominal_horizontal_speed_mps > 0.0 &&
         config_.nominal_vertical_speed_mps > 0.0 && config_.goal_tolerance_m >= 0.0 &&
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
    result.search_state_reused = true;
    exact_start_ = request.start;
    exact_goal_ = request.mission_goal;
    key_modifier_ += heuristic(last_start_, *start_anchor);
    start_ = *start_anchor;
    last_start_ = *start_anchor;
    if (!world_update.changed_cells.empty()) {
      updateAffectedVertices(world_update.changed_cells,
                             result.affected_lattice_states);
      repair_generation_ =
          repair_generation_ == std::numeric_limits<std::uint64_t>::max()
              ? 1U
              : repair_generation_ + 1U;
    }
  }

  const auto deadline =
      operation_started +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
          std::chrono::duration<double, std::milli>{config_.maximum_compute_time_ms});
  result.search_complete = computeShortestPath(
      deadline, config_.maximum_expansions_per_update, result.expansions);
  if (result.search_complete) {
    std::vector<Point3> path = extractPath();
    if (!path.empty()) {
      path = shortcutPath(path, result.shortcut_checks, result.shortcuts_applied);
    }
    if (path.size() >= 2U && pathRawValid(path) &&
        distance3D(path.back(), request.mission_goal) <= config_.goal_tolerance_m) {
      result.status = PersistentPlannerStatus3D::kReachedMissionGoal;
      result.points = std::move(path);
      incumbent_ = result.points;
    } else {
      result.status = PersistentPlannerStatus3D::kNoRoute;
    }
  } else {
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

  result.search_generation = search_generation_;
  result.repair_generation = repair_generation_;
  result.records = records_.size();
  result.open_entries = open_.size();
  populatePathMetrics(result);
  result.search_ms = std::chrono::duration<double, std::milli>(
                         std::chrono::steady_clock::now() - operation_started)
                         .count();
  return result;
}

} // namespace detail
} // namespace drone_city_nav
