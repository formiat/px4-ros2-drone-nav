#include "drone_city_nav/persistent_dstar_lite_planner_3d.hpp"

#include <algorithm>
#include <bit>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <numbers>
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
         estimated_stationary_turn_time_s >= 0.0 &&
         std::isfinite(ranked_execution_time_s) && ranked_execution_time_s >= 0.0;
}

double SpatialRouteCandidate3D::objectiveS() const noexcept {
  return ranked_execution_time_s > 0.0 ? ranked_execution_time_s
                                       : estimated_execution_time_s;
}

bool PlannerUpdate3D::publishable() const noexcept {
  return input_status == PlannerInputStatus3D::kAccepted &&
         improved_incumbent.has_value() && improved_incumbent->valid();
}

bool PlannerUpdate3D::running() const noexcept {
  return input_status == PlannerInputStatus3D::kAccepted &&
         progress == SearchProgress3D::kRunning;
}

std::chrono::steady_clock::duration
feasibilitySearchBudget3D(const std::chrono::steady_clock::duration remaining,
                          const std::chrono::steady_clock::duration configured,
                          const bool persistent_search_has_work) noexcept {
  const std::chrono::steady_clock::duration available =
      std::max(remaining, std::chrono::steady_clock::duration::zero());
  // The search runs only while no route exists at all. Until one does, the
  // resident session's repair and expansion work improves a route nobody can
  // fly, so it keeps a reserve to absorb the world and the search takes the
  // rest of the update. The configured time is that reserve, never a ceiling
  // on the search: time to a first route is what the vehicle waits on at
  // every mission waypoint.
  const std::chrono::steady_clock::duration reserve =
      persistent_search_has_work
          ? std::min(std::max(configured, std::chrono::steady_clock::duration::zero()),
                     available / 3)
          : std::chrono::steady_clock::duration::zero();
  return available - reserve;
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
    case SpatialRouteCandidateSource3D::kSpatialSearch:
      return "spatial_search";
    case SpatialRouteCandidateSource3D::kExecutionTimeRefinement:
      return "execution_time_refinement";
  }
  return "invalid";
}

namespace detail {

namespace {

// Whether two routes leave the vehicle's position on the same heading. Only
// the first segment matters: that is the part the vehicle is flying now, and
// a candidate that starts by turning it around discards the motion it has.
[[nodiscard]] bool leavesOnTheSameHeading(const SpatialRouteCandidate3D& incumbent,
                                          const SpatialRouteCandidate3D& candidate) {
  constexpr double kMinimumSegmentM{1.0e-3};
  // Cosine of the angle beyond which a heading change reads as a reversal
  // rather than an adjustment.
  constexpr double kSameHeadingCosine{0.5};
  if (incumbent.points.size() < 2U || candidate.points.size() < 2U) {
    return true;
  }
  const auto heading = [](const SpatialRouteCandidate3D& route) {
    const Vec3 delta{route.points[1U].x - route.points.front().x,
                     route.points[1U].y - route.points.front().y,
                     route.points[1U].z - route.points.front().z};
    const double length =
        std::sqrt(delta.x * delta.x + delta.y * delta.y + delta.z * delta.z);
    return length > kMinimumSegmentM
               ? std::optional<Vec3>{Vec3{delta.x / length, delta.y / length,
                                          delta.z / length}}
               : std::nullopt;
  };
  const std::optional<Vec3> incumbent_heading = heading(incumbent);
  const std::optional<Vec3> candidate_heading = heading(candidate);
  if (!incumbent_heading.has_value() || !candidate_heading.has_value()) {
    return true;
  }
  return incumbent_heading->x * candidate_heading->x +
             incumbent_heading->y * candidate_heading->y +
             incumbent_heading->z * candidate_heading->z >=
         kSameHeadingCosine;
}

} // namespace

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
  if (!incumbent_.has_value()) {
    incumbent_ = std::move(candidate);
    return incumbent_;
  }
  // A route that reaches the goal is not compared with one that does not: the
  // closest-approach route exists only because nothing reached the goal, and
  // being shorter is exactly what it is. Compared on time it displaced every
  // real route, and one recorded flight sat at the end of one for two and a
  // half minutes with the goal thirty-five metres away.
  const bool incumbent_reaches_goal =
      spatialRouteCandidateReachesGoal3D(incumbent_->source);
  const bool candidate_reaches_goal =
      spatialRouteCandidateReachesGoal3D(candidate.source);
  if (incumbent_reaches_goal != candidate_reaches_goal) {
    if (!candidate_reaches_goal) {
      return std::nullopt;
    }
    incumbent_ = std::move(candidate);
    return incumbent_;
  }
  const double scale =
      std::max({1.0, candidate.objectiveS(), incumbent_->objectiveS()});
  if (candidate.objectiveS() >=
      incumbent_->objectiveS() - kObjectiveTolerance * scale) {
    return std::nullopt;
  }
  // A candidate that leaves the vehicle's own position on a different heading
  // than the incumbent turns the vehicle around, and the recorded runs are
  // full of that: a hundred generation changes in four hundred seconds, with
  // twenty-one of them reversing the remaining distance by tens of metres.
  // Such a candidate has to be better by the margin a successor needs, not
  // merely better by a numerical epsilon; one that continues the same heading
  // replaces the incumbent as soon as it is better at all.
  if (!leavesOnTheSameHeading(*incumbent_, candidate) &&
      candidate.objectiveS() >
          incumbent_->objectiveS() - continuity_improvement_margin_s_) {
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

} // namespace detail
} // namespace drone_city_nav
