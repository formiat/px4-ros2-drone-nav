#include "drone_city_nav/persistent_dstar_lite_planner_3d.hpp"

#include <algorithm>
#include <bit>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
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

PersistentDStarLitePlanner3DImpl::PersistentDStarLitePlanner3DImpl(
    const PersistentPlannerConfig3D& config)
    : config_{config},
      lattice_{config_},
      dstar_session_{config_, lattice_},
      feasibility_search_{config_, lattice_},
      execution_time_refiner_{config_, lattice_, dstar_session_},
      coordinator_{config_.continuity_improvement_margin_s} {
}

void PersistentDStarLitePlanner3DImpl::initializeSearch(
    const PersistentPlannerRequest3D& request, const PersistentPlannerNode3D start,
    const PersistentPlannerNode3D goal) {
  initialized_ = true;
  start_ = start;
  last_start_ = start;
  goal_ = goal;
  exact_start_ = request.start;
  exact_goal_ = request.mission_goal;
  mission_epoch_ = request.mission_epoch;
  dstar_session_.begin(start, goal);
  feasibility_search_.reset();
  execution_time_refiner_.reset();
}

FeasiblePathSearch3D::Endpoints3D
PersistentDStarLitePlanner3DImpl::searchEndpoints() const noexcept {
  return FeasiblePathSearch3D::Endpoints3D{
      .start = start_,
      .goal = goal_,
      .exact_start = exact_start_,
      .exact_goal = exact_goal_,
      .departure_waypoint = departure_waypoint_,
  };
}

const PersistentPlannerConfig3D&
PersistentDStarLitePlanner3DImpl::config() const noexcept {
  return config_;
}

void PersistentDStarLitePlanner3DImpl::reset() noexcept {
  initialized_ = false;
  world_ = {};
  lattice_.reset();
  start_ = {};
  last_start_ = {};
  goal_ = {};
  exact_start_ = {};
  departure_waypoint_.reset();
  departure_anchor_skip_ = 0U;
  exact_goal_ = {};
  mission_epoch_ = 0U;
  dstar_session_.reset();
  feasibility_search_.reset();
  execution_time_refiner_.reset();
  coordinator_.reset();
  adaptive_edges_in_extracted_path_ = 0U;
  published_session_id_ = 0U;
  applied_incumbent_rejection_sequence_ = 0U;
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
         std::isfinite(config_.execution_time_refinement_minimum_improvement_s) &&
         config_.execution_time_refinement_minimum_improvement_s >= 0.0 &&
         std::isfinite(config_.execution_time_refinement_minimum_improvement_ratio) &&
         config_.execution_time_refinement_minimum_improvement_ratio >= 0.0 &&
         config_.execution_time_refinement_minimum_improvement_ratio < 1.0 &&
         std::isfinite(config_.feasibility_goal_connector_reach_m) &&
         config_.feasibility_goal_connector_reach_m > 0.0 &&
         std::isfinite(config_.feasibility_clearance_ranking_distance_m) &&
         config_.feasibility_clearance_ranking_distance_m > 0.0 &&
         std::isfinite(config_.clearance_ranking_weight) &&
         config_.clearance_ranking_weight >= 0.0 &&
         std::isfinite(config_.clearance_ranking_distance_m) &&
         config_.clearance_ranking_distance_m > 0.0 &&
         std::isfinite(config_.clearance_ranking_critical_distance_m) &&
         config_.clearance_ranking_critical_distance_m >= 0.0 &&
         config_.clearance_ranking_critical_distance_m <=
             config_.clearance_ranking_distance_m &&
         std::isfinite(config_.clearance_ranking_critical_weight) &&
         config_.clearance_ranking_critical_weight >= 0.0 &&
         config_.physical_footprint.sweep_step_m > 0.0 &&
         lattice_.pointInsideFlightEnvelope(request.start) &&
         lattice_.pointInsideFlightEnvelope(request.mission_goal);
}

PlannerUpdate3D
PersistentDStarLitePlanner3DImpl::plan(const PersistentPlannerRequest3D& request) {
  const auto operation_started = std::chrono::steady_clock::now();
  lattice_.resetEdgeStatistics();
  adaptive_edges_in_extracted_path_ = 0U;
  PlannerUpdate3D update;
  PlannerTelemetry3D& telemetry = update.telemetry;
  telemetry.mission_epoch = request.mission_epoch;
  telemetry.planned_on_revision = request.world.revision;
  telemetry.occupied_fingerprint = request.world.occupied_fingerprint;
  if (!validRequest(request)) {
    telemetry.input_failure = "request_invalid";
    return update;
  }
  // What the shortest-path search is guaranteed, whatever the stages before it
  // spend. Repair, change scheduling and the feasibility search all ran before
  // it against the same budget, and between them they took all of it: D* was
  // measured expanding nothing in over half of the updates, so the route the
  // vehicle flew came from the unranked feasibility branch again and again.
  // Reserving the tail of the update for the search is what lets the ranked
  // route exist at all.
  const auto deadline =
      operation_started +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
          std::chrono::duration<double, std::milli>{config_.maximum_compute_time_ms});
  lattice_.beginEdgeRefinementBudget();
  const auto spatial_search_reserve =
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
          std::chrono::duration<double, std::milli>{
              config_.maximum_compute_time_ms *
              std::clamp(config_.guaranteed_spatial_search_fraction, 0.0, 0.9)});
  const auto preparatory_deadline = deadline - spatial_search_reserve;

  const std::uint64_t previous_producer = world_.producer_instance_id;
  const std::uint64_t previous_mission_epoch = mission_epoch_;
  const Point3 previous_goal = exact_goal_;
  const auto world_update_started = std::chrono::steady_clock::now();
  const PersistentPlannerWorldUpdate3D world_update = updateWorld(request.world);
  telemetry.world_diff_ms = world_update.diff_ms;
  telemetry.world_install_ms = world_update.install_ms;
  telemetry.world_update_ms =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() -
                                                world_update_started)
          .count();
  if (!world_update.accepted) {
    telemetry.input_failure = "world_rejected";
    return update;
  }
  update.input_status = PlannerInputStatus3D::kAccepted;
  if (world_update.resident_world_retained) {
    telemetry.planned_on_revision = world_.revision;
    telemetry.occupied_fingerprint = world_.occupied_fingerprint;
  }
  telemetry.changed_occupied_voxels = world_update.changed_cells.size();
  telemetry.occupied_world_unchanged = world_update.occupied_world_unchanged;

  const PlannerLattice3D::DepartureConnection3D departure =
      lattice_.selectDepartureConnection(request.start, departure_anchor_skip_);
  if (!departure.available()) {
    update.input_status = PlannerInputStatus3D::kStartUnavailable;
    return update;
  }
  const std::optional<PersistentPlannerNode3D>& start_anchor = departure.anchor;
  departure_waypoint_ = departure.waypoint;
  telemetry.departure_waypoint_used = departure.waypoint.has_value();
  const std::optional<PersistentPlannerNode3D> goal_anchor =
      lattice_.selectAnchor(request.mission_goal, false);
  if (!goal_anchor.has_value()) {
    update.input_status = PlannerInputStatus3D::kGoalUnavailable;
    return update;
  }
  const ExecutionTimeRefiner3D::Request3D refinement_request =
      refinementRequest(request, *start_anchor, *goal_anchor);

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
    // The feasibility search keeps its anchor while the vehicle stays within
    // one lattice diagonal of it: a hovering vehicle flips its nearest node
    // with every centimetre of drift, and re-anchoring the search on each flip
    // would restart it forever. The departure segment from the exact start to
    // the anchor is validated on every candidate, and a candidate that fails
    // restarts the search from the current anchor, so a momentarily blocked
    // departure never discards the labels by itself.
    const bool feasibility_start_changed =
        feasibility_search_.initialized() &&
        *start_anchor != feasibility_search_.anchor() &&
        distance3D(request.start, lattice_.pointFor(feasibility_search_.anchor())) >
            std::numbers::sqrt2 * config_.minimum_horizontal_step_m;
    const bool execution_time_start_changed = execution_time_refiner_.startChanged(
        refinement_request.start, refinement_request.start_from_rest);
    const bool execution_time_goal_changed =
        execution_time_refiner_.goalChanged(request.mission_goal);
    telemetry.search_state_reused = true;
    exact_start_ = request.start;
    exact_goal_ = request.mission_goal;
    dstar_session_.rebaseStart(*start_anchor,
                               lattice_.heuristic(last_start_, *start_anchor));
    start_ = *start_anchor;
    last_start_ = *start_anchor;
    if (!world_update.changed_cells.empty()) {
      dstar_session_.scheduleAffectedVertices(world_, world_update.changed_cells,
                                              telemetry.affected_lattice_states);
      const auto& schedule = dstar_session_.scheduleStatistics();
      telemetry.schedule_ms = schedule.total_ms;
      telemetry.schedule_ranking_ms = schedule.ranking_ms;
      telemetry.schedule_edges_forgotten = schedule.edges_forgotten;
      telemetry.schedule_clearances_tightened = schedule.clearances_tightened;
      if (world_update.occupied_cells_removed) {
        // A cost-to-go retained across an obstacle removal can overestimate a
        // newly opened route until every affected label settles. Keep the
        // refinement heuristic strictly Euclidean in that case.
        dstar_session_.markCostToGoalInadmissible();
      }
      dstar_session_.advanceRepairGeneration();
      // The feasibility search keeps its labels across occupied changes and
      // re-validates their edge chains lazily on the resident world; every
      // candidate it returns is raw-validated there as well.
      feasibility_search_.noteWorldChanged();
    }
    if (feasibility_start_changed) {
      feasibility_search_.reset();
    }
    if (execution_time_start_changed || execution_time_goal_changed) {
      execution_time_refiner_.reset();
    } else if (!world_update.changed_cells.empty()) {
      execution_time_refiner_.rebaseWorld();
    }
  }

  // A release reason no longer discards the incumbent. A route released as
  // blocked is blocked at one station, not everywhere, and throwing it away
  // restarted the search from nothing: the feasibility branch then published
  // another first-found route, which the next observed voxel blocked in turn —
  // the churn that gave routes a median life of under two seconds.
  //
  // Evidence decides instead. The incumbent is re-validated against the
  // current world below and reset when it no longer clears the body, which is
  // the same outcome whenever the block is real and on the part still to fly.
  // A consumer that could not enter the incumbent it was delivered says so
  // through the rejection sequence, and that does reset it.
  if (request.incumbent_rejection_sequence > applied_incumbent_rejection_sequence_) {
    // The consumer could not enter the incumbent it was delivered. Keeping it
    // would keep the feasibility search idle and leave the vehicle waiting on
    // an incremental repair that newer evidence may never let converge.
    applied_incumbent_rejection_sequence_ = request.incumbent_rejection_sequence;
    coordinator_.reset();
    execution_time_refiner_.reset();
    feasibility_search_.reset();
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
    const auto feasibility_started = std::chrono::steady_clock::now();
    const bool persistent_search_has_work = dstar_session_.pendingRepairNodes() > 0U ||
                                            !dstar_session_.shortestPathComplete();
    const auto feasibility_deadline = std::min(
        preparatory_deadline,
        feasibility_started +
            feasibilitySearchBudget3D(
                deadline - feasibility_started,
                std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                    std::chrono::duration<double, std::milli>{
                        config_.maximum_feasibility_compute_time_ms}),
                persistent_search_has_work));
    std::optional<std::vector<Point3>> path =
        feasibility_search_.advance(searchEndpoints(), feasibility_deadline,
                                    config_.maximum_feasibility_expansions_per_update,
                                    telemetry.feasibility_expansions);
    // The feasibility branch publishes most of the routes this vehicle flies,
    // so it gets the same simplification and clearance centering the
    // refinement branch has always had. A first-found lattice path handed
    // straight to execution is what put routes against door jambs.
    std::optional<SpatialRouteCandidate3D> candidate =
        path ? makeCandidate(refinePublishedPath(std::move(*path), request, telemetry),
                             SpatialRouteCandidateSource3D::kFeasibilitySearch,
                             request.velocity)
             : std::nullopt;
    if (candidate) {
      telemetry.feasibility_route_found = true;
      update.improved_incumbent = coordinator_.consider(std::move(*candidate));
    }
    telemetry.feasibility_frontier_exhausted = feasibility_search_.frontierExhausted();
    telemetry.feasibility_explored_nodes = feasibility_search_.exploredNodes();
    telemetry.feasibility_closest_goal_distance_m =
        feasibility_search_.closestGoalDistanceM();
    telemetry.feasibility_restarts = feasibility_search_.restartCount();
    telemetry.feasibility_invalidated_labels =
        feasibility_search_.invalidatedLabelCount();
    telemetry.feasibility_adopted_labels = feasibility_search_.adoptedLabelCount();
    telemetry.feasibility_last_invalid_segment =
        feasibility_search_.lastInvalidSegment();
    telemetry.feasibility_anchor = lattice_.pointFor(
        feasibility_search_.initialized() ? feasibility_search_.anchor() : start_);
    telemetry.feasibility_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() -
                                                  feasibility_started)
            .count();
  }

  // Repair and search share the update: while a changing world keeps the
  // repair queue from ever draining, the search still gets half the budget so
  // it can expand and hand out an anytime route on the labels it has. Labels
  // a later repair moves re-enter the queue and the search re-converges.
  const auto repair_started = std::chrono::steady_clock::now();
  const auto repair_deadline =
      dstar_session_.pendingRepairNodes() > 0U && deadline > repair_started
          ? std::min(repair_started + (deadline - repair_started) / 2,
                     preparatory_deadline)
          : preparatory_deadline;
  static_cast<void>(dstar_session_.continueAffectedVertexRepair(
      repair_deadline, config_.maximum_expansions_per_update,
      telemetry.repair_lattice_states_processed));
  dstar_session_.scheduleMovedClearances();
  const auto spatial_search_started = std::chrono::steady_clock::now();
  telemetry.repair_ms =
      std::chrono::duration<double, std::milli>(spatial_search_started - repair_started)
          .count();
  const std::size_t remaining_spatial_expansions =
      telemetry.repair_lattice_states_processed < config_.maximum_expansions_per_update
          ? config_.maximum_expansions_per_update -
                telemetry.repair_lattice_states_processed
          : 0U;
  bool spatial_search_complete = dstar_session_.shortestPathComplete();
  if (!spatial_search_complete && remaining_spatial_expansions > 0U) {
    spatial_search_complete = dstar_session_.computeShortestPath(
        deadline, remaining_spatial_expansions, telemetry.expansions);
  }
  dstar_session_.scheduleMovedClearances();
  const auto refinement_started = std::chrono::steady_clock::now();
  telemetry.spatial_search_ms = std::chrono::duration<double, std::milli>(
                                    refinement_started - spatial_search_started)
                                    .count();
  const bool repair_complete = dstar_session_.pendingRepairNodes() == 0U;
  telemetry.repair_lattice_states_pending = dstar_session_.pendingRepairNodes();
  telemetry.repair_pending = !repair_complete;
  const bool spatial_route_available =
      spatial_search_complete && dstar_session_.startResolved(start_);
  // Seeding the refinement with the feasibility route was tried and is wrong:
  // the refinement treats its seed as an anytime *bound*, so a feasibility
  // route handed to it as a seed makes it declare at once that it cannot
  // improve on it by the admissible margin, and it never expands. The way a
  // ranked route comes to exist is the budget the searches are guaranteed
  // above, not a different seed.
  if (spatial_route_available) {
    if (!execution_time_refiner_.initialized()) {
      execution_time_refiner_.begin(
          refinement_request,
          dstar_session_.extractPath(exact_start_, exact_goal_, departure_waypoint_,
                                     adaptive_edges_in_extracted_path_));
    } else if (!execution_time_refiner_.hasIncumbent()) {
      // The world change dropped the refinement incumbent; the repaired D*
      // route is the new anytime bound.
      execution_time_refiner_.seedIncumbent(
          dstar_session_.extractPath(exact_start_, exact_goal_, departure_waypoint_,
                                     adaptive_edges_in_extracted_path_));
    }
    const std::size_t graph_expansions =
        telemetry.repair_lattice_states_processed + telemetry.expansions;
    const std::size_t guaranteed_refinement_expansions = static_cast<std::size_t>(
        static_cast<double>(config_.maximum_expansions_per_update) *
        std::clamp(config_.guaranteed_refinement_expansion_fraction, 0.0, 1.0));
    const std::size_t refinement_budget =
        std::max(graph_expansions < config_.maximum_expansions_per_update
                     ? config_.maximum_expansions_per_update - graph_expansions
                     : 0U,
                 std::max<std::size_t>(guaranteed_refinement_expansions, 1U));
    std::optional<std::vector<Point3>> path = execution_time_refiner_.advance(
        deadline, refinement_budget, telemetry.execution_time_search_expansions);
    if (path.has_value()) {
      // The published path came from the refinement, so its adaptive-edge count
      // is the one that describes it.
      adaptive_edges_in_extracted_path_ =
          execution_time_refiner_.adaptiveEdgesInExtractedPath();
    }
    if (path && path->size() >= 2U) {
      *path = refinePublishedPath(std::move(*path), request, telemetry);
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
    execution_time_refiner_.reset();
  }
  telemetry.refinement_ms = std::chrono::duration<double, std::milli>(
                                std::chrono::steady_clock::now() - refinement_started)
                                .count();

  // A consumer that opened a new session holds no route of this search: its
  // first update delivers the resident incumbent, later ones improvements only.
  if (request.session_id != 0U && request.session_id != published_session_id_ &&
      !update.improved_incumbent.has_value() && coordinator_.incumbent() != nullptr) {
    update.improved_incumbent = *coordinator_.incumbent();
  }
  if (update.improved_incumbent.has_value()) {
    published_session_id_ = request.session_id;
  }

  // A search that exhausted its frontier without a route says nothing about
  // the world: it says this start could not reach the goal. The nearest
  // reachable node is not always a useful one — it can belong to a component
  // the goal is not in — so the next update starts from the next node the body
  // reaches, and the walk wraps once every one of them has been tried.
  if (coordinator_.incumbent() == nullptr && telemetry.feasibility_frontier_exhausted) {
    const std::size_t anchors = lattice_.departureConnectionCount(request.start);
    departure_anchor_skip_ =
        anchors > 1U ? (departure_anchor_skip_ + 1U) % anchors : 0U;
    telemetry.departure_anchor_skip = departure_anchor_skip_;
  } else if (coordinator_.incumbent() != nullptr) {
    departure_anchor_skip_ = 0U;
  }

  const bool search_complete =
      repair_complete && spatial_search_complete &&
      (!spatial_route_available || execution_time_refiner_.complete());
  telemetry.incumbent_available = coordinator_.incumbent() != nullptr;
  if (!search_complete) {
    update.progress = SearchProgress3D::kRunning;
  } else if (telemetry.incumbent_available) {
    update.progress = SearchProgress3D::kConverged;
  } else {
    update.progress = SearchProgress3D::kNoRoute;
  }
  telemetry.execution_time_search_complete = execution_time_refiner_.complete();
  telemetry.search_generation = dstar_session_.searchGeneration();
  telemetry.repair_generation = dstar_session_.repairGeneration();
  telemetry.records = dstar_session_.records();
  telemetry.open_entries = dstar_session_.openEntries();
  telemetry.lattice_edge_queries = lattice_.edgeQueries();
  telemetry.raw_edge_validation_checks = lattice_.rawEdgeValidationChecks();
  telemetry.schedule_clearances_rederived = lattice_.clearancesRederived();
  telemetry.adaptive_edge_queries = lattice_.adaptiveEdgeQueries();
  telemetry.adaptive_edges_in_extracted_path = adaptive_edges_in_extracted_path_;
  telemetry.maximum_queried_lattice_level = lattice_.maximumQueriedLevel();
  telemetry.execution_time_search_records = execution_time_refiner_.records();
  telemetry.execution_time_search_open_entries = execution_time_refiner_.openEntries();
  telemetry.execution_time_search_objective_s =
      execution_time_refiner_.objectiveSeconds();
  telemetry.search_ms = std::chrono::duration<double, std::milli>(
                            std::chrono::steady_clock::now() - operation_started)
                            .count();
  return update;
}

} // namespace detail
} // namespace drone_city_nav
