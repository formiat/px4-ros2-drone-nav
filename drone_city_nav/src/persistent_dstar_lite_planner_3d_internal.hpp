#pragma once

#include "drone_city_nav/occupied_collision_oracle_3d.hpp"
#include "drone_city_nav/persistent_dstar_lite_planner_3d.hpp"

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace drone_city_nav::detail {

class PersistentDStarLitePlanner3DImpl;

struct PersistentPlannerNode3D {
  int x{0};
  int y{0};
  int z{0};

  [[nodiscard]] bool
  operator==(const PersistentPlannerNode3D&) const noexcept = default;
};

struct PersistentPlannerNode3DHash {
  [[nodiscard]] std::size_t
  operator()(const PersistentPlannerNode3D& node) const noexcept;
};

struct PersistentPlannerEdge3D {
  PersistentPlannerNode3D first{};
  PersistentPlannerNode3D second{};

  [[nodiscard]] bool
  operator==(const PersistentPlannerEdge3D&) const noexcept = default;
};

struct PersistentPlannerEdge3DHash {
  [[nodiscard]] std::size_t
  operator()(const PersistentPlannerEdge3D& edge) const noexcept;
};

struct PersistentPlannerDirection3D {
  std::int8_t x{0};
  std::int8_t y{0};
  std::int8_t z{0};

  [[nodiscard]] bool empty() const noexcept {
    return x == 0 && y == 0 && z == 0;
  }

  [[nodiscard]] bool
  operator==(const PersistentPlannerDirection3D&) const noexcept = default;
};

struct PersistentPlannerTimeState3D {
  PersistentPlannerNode3D position{};
  PersistentPlannerDirection3D incoming{};

  [[nodiscard]] bool
  operator==(const PersistentPlannerTimeState3D&) const noexcept = default;
};

struct PersistentPlannerTimeState3DHash {
  [[nodiscard]] std::size_t
  operator()(const PersistentPlannerTimeState3D& state) const noexcept;
};

struct PersistentPlannerTimeQueueEntry3D {
  double estimated_total_s{std::numeric_limits<double>::infinity()};
  double cost_from_start_s{std::numeric_limits<double>::infinity()};
  PersistentPlannerTimeState3D state{};
  std::uint64_t sequence{0U};
};

struct PersistentPlannerTimeQueueEntryCompare3D {
  [[nodiscard]] bool
  operator()(const PersistentPlannerTimeQueueEntry3D& first,
             const PersistentPlannerTimeQueueEntry3D& second) const noexcept;
};

struct DStarLiteKey3D {
  double first{std::numeric_limits<double>::infinity()};
  double second{std::numeric_limits<double>::infinity()};
};

struct DStarLiteRecord3D {
  double g{std::numeric_limits<double>::infinity()};
  double rhs{std::numeric_limits<double>::infinity()};
  std::uint64_t open_token{0U};
};

struct DStarLiteQueueEntry3D {
  DStarLiteKey3D key{};
  PersistentPlannerNode3D node{};
  std::uint64_t token{0U};
  std::uint64_t sequence{0U};
};

struct DStarLiteQueueEntryCompare3D {
  [[nodiscard]] bool operator()(const DStarLiteQueueEntry3D& first,
                                const DStarLiteQueueEntry3D& second) const noexcept;
};

struct FeasibilityQueueEntry3D {
  double estimated_total_s{std::numeric_limits<double>::infinity()};
  double cost_from_start_s{std::numeric_limits<double>::infinity()};
  std::size_t depth{0U};
  PersistentPlannerNode3D node{};
  std::uint64_t sequence{0U};
};

struct FeasibilityQueueEntryCompare3D {
  [[nodiscard]] bool operator()(const FeasibilityQueueEntry3D& first,
                                const FeasibilityQueueEntry3D& second) const noexcept;
};

struct PersistentPlannerWorldUpdate3D {
  bool accepted{false};
  bool requires_reset{false};
  bool occupied_world_unchanged{false};
  bool occupied_cells_removed{false};
  std::vector<GridIndex3D> changed_cells;
};

using DStarLiteOpenQueue3D =
    std::priority_queue<DStarLiteQueueEntry3D, std::vector<DStarLiteQueueEntry3D>,
                        DStarLiteQueueEntryCompare3D>;
using ExecutionTimeOpenQueue3D =
    std::priority_queue<PersistentPlannerTimeQueueEntry3D,
                        std::vector<PersistentPlannerTimeQueueEntry3D>,
                        PersistentPlannerTimeQueueEntryCompare3D>;
using FeasibilityOpenQueue3D =
    std::priority_queue<FeasibilityQueueEntry3D, std::vector<FeasibilityQueueEntry3D>,
                        FeasibilityQueueEntryCompare3D>;

// The discretized world the searches reason over: which nodes exist, where they
// sit, which segments the physical body can traverse, and what an edge costs.
// All three searches ask the same questions of it, so it answers them itself
// instead of exposing its geometry for a single owner to interpret.
class PlannerLattice3D final {
public:
  explicit PlannerLattice3D(const PersistentPlannerConfig3D& config) noexcept
      : config_{std::addressof(config)} {
  }

  void reset() noexcept;
  // Forgets cached edge costs and query statistics while keeping the geometry.
  void resetEdgeEvidence() noexcept;
  // Starts a new query-statistics window without discarding cached costs.
  void resetEdgeStatistics() noexcept;

  void installWorld(const PersistentPlannerWorld3D& world);
  void configureGridGeometry(const GridBounds3D& bounds);
  [[nodiscard]] bool sameGridGeometry(const GridBounds3D& bounds) const noexcept;
  [[nodiscard]] bool configured() const noexcept;
  [[nodiscard]] const GridBounds3D& bounds() const noexcept;
  // Upper bound on nodes along one traversal of the lattice.
  [[nodiscard]] std::size_t nodeSpan() const noexcept;

  // Drops the cached cost of an evaluated edge. A D* label can depend only on
  // an edge whose cost was evaluated, so repair invalidates exactly those.
  [[nodiscard]] bool forgetEdgeCost(const PersistentPlannerEdge3D& edge);

  [[nodiscard]] bool nodeInside(PersistentPlannerNode3D node) const noexcept;
  [[nodiscard]] int maximumScale() const noexcept;
  [[nodiscard]] std::size_t level(PersistentPlannerNode3D first,
                                  PersistentPlannerNode3D second) const noexcept;
  [[nodiscard]] Point3 pointFor(PersistentPlannerNode3D node) const noexcept;
  [[nodiscard]] PersistentPlannerNode3D nearestNode(const Point3& point) const noexcept;
  [[nodiscard]] std::optional<PersistentPlannerNode3D>
  selectAnchor(const Point3& point, bool start_anchor) const;
  [[nodiscard]] bool pointInsideFlightEnvelope(const Point3& point) const noexcept;
  [[nodiscard]] bool rawSegmentValid(const Point3& first, const Point3& second) const;
  [[nodiscard]] bool departureSegmentValid(const Point3& first,
                                           const Point3& second) const;
  [[nodiscard]] bool nodeValid(PersistentPlannerNode3D node) const;
  // A path is traversable when it stays inside the flight envelope and every
  // segment clears the physical body; the first segment is a departure.
  [[nodiscard]] bool pathTraversable(const std::vector<Point3>& path) const;
  [[nodiscard]] std::vector<PersistentPlannerNode3D>
  adjacentNodes(PersistentPlannerNode3D node) const;

  // Visits every lattice neighbour of a node without materializing a vector;
  // the searches call this in their hot loops.
  template<typename Visitor>
  void forEachAdjacentNode(const PersistentPlannerNode3D node,
                           Visitor&& visitor) const {
    for (std::size_t level = 0U; level <= config_->maximum_adaptive_lattice_level;
         ++level) {
      const int scale = 1 << level;
      if (node.x % scale != 0 || node.y % scale != 0 || node.z % scale != 0) {
        continue;
      }
      for (int z_offset = -1; z_offset <= 1; ++z_offset) {
        for (int y_offset = -1; y_offset <= 1; ++y_offset) {
          for (int x_offset = -1; x_offset <= 1; ++x_offset) {
            if (x_offset == 0 && y_offset == 0 && z_offset == 0) {
              continue;
            }
            const PersistentPlannerNode3D candidate{node.x + x_offset * scale,
                                                    node.y + y_offset * scale,
                                                    node.z + z_offset * scale};
            if (nodeInside(candidate) &&
                pointInsideFlightEnvelope(pointFor(candidate))) {
              visitor(candidate);
            }
          }
        }
      }
    }
  }

  [[nodiscard]] double heuristic(PersistentPlannerNode3D first,
                                 PersistentPlannerNode3D second) const noexcept;
  [[nodiscard]] double rawEdgeCost(PersistentPlannerNode3D first,
                                   PersistentPlannerNode3D second);
  // Uncached edge traversability, for a search that owns its own cost model.
  // Query statistics are recorded the same way as for a cached cost.
  [[nodiscard]] bool edgeTraversable(PersistentPlannerNode3D first,
                                     PersistentPlannerNode3D second);

  [[nodiscard]] std::size_t edgeQueries() const noexcept;
  [[nodiscard]] std::size_t rawEdgeValidationChecks() const noexcept;
  [[nodiscard]] std::size_t adaptiveEdgeQueries() const noexcept;
  [[nodiscard]] std::size_t maximumQueriedLevel() const noexcept;

private:
  const PersistentPlannerConfig3D* config_{nullptr};
  GridBounds3D raw_bounds_{};
  int width_{0};
  int height_{0};
  int depth_{0};
  std::optional<OccupiedCollisionOracle3D> resident_collision_oracle_;
  std::optional<OccupiedCollisionOracle3D> departure_collision_oracle_;
  std::unordered_map<PersistentPlannerEdge3D, double, PersistentPlannerEdge3DHash>
      edge_cost_cache_;
  std::size_t edge_queries_{0U};
  std::size_t raw_edge_validation_checks_{0U};
  std::size_t adaptive_edge_queries_{0U};
  std::size_t maximum_queried_level_{0U};
};

// The incremental backward search. It owns its labels, its open queue, and the
// repair queue that keeps them consistent with world changes, and it answers
// what a route from a start node costs.
class DStarLiteSession3D final {
public:
  DStarLiteSession3D(const PersistentPlannerConfig3D& config,
                     PlannerLattice3D& lattice) noexcept
      : config_{std::addressof(config)},
        lattice_{std::addressof(lattice)} {
  }

  void reset() noexcept;
  // Starts a new backward search rooted at the goal.
  void begin(PersistentPlannerNode3D start, PersistentPlannerNode3D goal);
  // Records that the start moved, keeping the queue keys consistent.
  void rebaseStart(PersistentPlannerNode3D start, double key_offset) noexcept;
  // A repair that could not be scheduled exactly makes the cached cost-to-goal
  // an unsound lower bound until the next full search.
  void markCostToGoalInadmissible() noexcept;
  void advanceRepairGeneration() noexcept;

  // Backward-search cost-to-goal for a node, when the search has established a
  // finite, admissible one. The execution-time refinement reads it as a lower
  // bound instead of duplicating the graph search.
  [[nodiscard]] std::optional<double>
  costToGoal(PersistentPlannerNode3D node) const noexcept;
  [[nodiscard]] bool startResolved(PersistentPlannerNode3D start) const noexcept;

  void scheduleAffectedVertices(const PersistentPlannerWorld3D& world,
                                const std::vector<GridIndex3D>& changed_cells,
                                std::size_t& affected_states);
  [[nodiscard]] bool
  continueAffectedVertexRepair(std::chrono::steady_clock::time_point deadline,
                               std::size_t maximum_vertices,
                               std::size_t& processed_vertices);
  [[nodiscard]] bool shortestPathComplete();
  [[nodiscard]] bool computeShortestPath(std::chrono::steady_clock::time_point deadline,
                                         std::size_t maximum_expansions,
                                         std::size_t& expansions);
  [[nodiscard]] std::vector<Point3> extractPath(const Point3& exact_start,
                                                const Point3& exact_goal,
                                                std::size_t& adaptive_edges);

  [[nodiscard]] std::uint64_t searchGeneration() const noexcept;
  [[nodiscard]] std::uint64_t repairGeneration() const noexcept;
  [[nodiscard]] std::size_t records() const noexcept;
  [[nodiscard]] std::size_t openEntries() const noexcept;
  [[nodiscard]] std::size_t pendingRepairNodes() const noexcept;

private:
  [[nodiscard]] DStarLiteKey3D calculateKey(PersistentPlannerNode3D node);
  void enqueue(PersistentPlannerNode3D node, DStarLiteRecord3D& record);
  void updateVertex(PersistentPlannerNode3D node);
  [[nodiscard]] std::optional<DStarLiteQueueEntry3D> currentTop();

  const PersistentPlannerConfig3D* config_{nullptr};
  PlannerLattice3D* lattice_{nullptr};
  PersistentPlannerNode3D start_{};
  PersistentPlannerNode3D goal_{};
  std::uint64_t search_generation_{0U};
  std::uint64_t repair_generation_{0U};
  std::uint64_t queue_token_{0U};
  std::uint64_t queue_sequence_{0U};
  double key_modifier_{0.0};
  bool cost_to_goal_heuristic_admissible_{true};
  DStarLiteOpenQueue3D open_{};
  std::unordered_map<PersistentPlannerNode3D, DStarLiteRecord3D,
                     PersistentPlannerNode3DHash>
      records_;
  std::deque<PersistentPlannerNode3D> pending_repair_nodes_;
  std::unordered_set<PersistentPlannerNode3D, PersistentPlannerNode3DHash>
      pending_repair_members_;
};

// Resumable best-first search for any raw-traversable path to the goal. It is
// the recovery answer when the optimal backward search has not converged, so it
// owns its own frontier and reports its own progress.
class FeasiblePathSearch3D final {
public:
  struct Endpoints3D {
    PersistentPlannerNode3D start{};
    PersistentPlannerNode3D goal{};
    Point3 exact_start{};
    Point3 exact_goal{};
  };

  FeasiblePathSearch3D(const PersistentPlannerConfig3D& config,
                       PlannerLattice3D& lattice) noexcept
      : config_{std::addressof(config)},
        lattice_{std::addressof(lattice)} {
  }

  void reset() noexcept;
  [[nodiscard]] bool initialized() const noexcept;

  // Expands the frontier until a path is found, the budget is spent, or the
  // deadline passes. Seeds itself from the endpoints on first use.
  [[nodiscard]] std::optional<std::vector<Point3>>
  advance(const Endpoints3D& endpoints, std::chrono::steady_clock::time_point deadline,
          std::size_t maximum_expansions, std::size_t& expansions);

private:
  void initialize(const Endpoints3D& endpoints);
  [[nodiscard]] std::optional<std::vector<Point3>>
  reconstruct(const Endpoints3D& endpoints, PersistentPlannerNode3D terminal) const;

  const PersistentPlannerConfig3D* config_{nullptr};
  PlannerLattice3D* lattice_{nullptr};
  bool initialized_{false};
  std::uint64_t queue_sequence_{0U};
  FeasibilityOpenQueue3D open_{};
  std::unordered_map<PersistentPlannerNode3D, double, PersistentPlannerNode3DHash>
      costs_;
  std::unordered_map<PersistentPlannerNode3D, PersistentPlannerNode3D,
                     PersistentPlannerNode3DHash>
      parents_;
};

// Discrete travel direction of a vector or a lattice edge. Time states are
// direction-aware, so both the refiner and the request that seeds it name the
// same discretization.
[[nodiscard]] PersistentPlannerDirection3D
directionForVector3D(const Vec3& vector) noexcept;

[[nodiscard]] PersistentPlannerDirection3D
directionForEdge3D(PersistentPlannerNode3D first,
                   PersistentPlannerNode3D second) noexcept;

// Refines a spatial route into the fastest executable one by searching over
// direction-aware time states. It is seeded from a spatial incumbent and keeps
// its own frontier across updates, so it owns that state rather than exposing
// it.
class ExecutionTimeRefiner3D final {
public:
  struct Request3D {
    PersistentPlannerTimeState3D start{};
    PersistentPlannerNode3D goal_anchor{};
    Point3 exact_start{};
    Point3 exact_goal{};
    bool start_from_rest{false};
  };

  ExecutionTimeRefiner3D(const PersistentPlannerConfig3D& config,
                         PlannerLattice3D& lattice,
                         const DStarLiteSession3D& session) noexcept
      : config_{std::addressof(config)},
        lattice_{std::addressof(lattice)},
        session_{std::addressof(session)} {
  }

  void reset() noexcept;
  // Starts a refinement for these endpoints, seeded from a spatial route the
  // backward search has already proved.
  void begin(const Request3D& request, const std::vector<Point3>& spatial_route);

  [[nodiscard]] bool initialized() const noexcept;
  [[nodiscard]] bool complete() const noexcept;
  [[nodiscard]] bool startChanged(const PersistentPlannerTimeState3D& start,
                                  bool start_from_rest) const noexcept;
  [[nodiscard]] bool goalChanged(const Point3& exact_goal) const noexcept;
  [[nodiscard]] bool hasIncumbent() const noexcept;
  [[nodiscard]] std::vector<Point3> bestPath();
  [[nodiscard]] std::size_t records() const noexcept;
  [[nodiscard]] std::size_t openEntries() const noexcept;
  [[nodiscard]] double objectiveSeconds() const noexcept;
  [[nodiscard]] std::size_t adaptiveEdgesInExtractedPath() const noexcept;

  [[nodiscard]] std::optional<std::vector<Point3>>
  advance(std::chrono::steady_clock::time_point deadline,
          std::size_t maximum_expansions, std::size_t& expansions);

private:
  void seedIncumbent(const std::vector<Point3>& spatial_route);
  [[nodiscard]] Vec3
  directionVector(PersistentPlannerDirection3D direction) const noexcept;
  [[nodiscard]] double
  heuristic(const PersistentPlannerTimeState3D& state) const noexcept;
  [[nodiscard]] double transitionCost(const PersistentPlannerTimeState3D& first,
                                      const PersistentPlannerTimeState3D& second);
  [[nodiscard]] double
  terminalCost(const PersistentPlannerTimeState3D& state) const noexcept;
  [[nodiscard]] std::vector<Point3> extractPath();

  const PersistentPlannerConfig3D* config_{nullptr};
  PlannerLattice3D* lattice_{nullptr};
  const DStarLiteSession3D* session_{nullptr};
  Request3D request_{};
  bool initialized_{false};
  bool complete_{false};
  std::optional<PersistentPlannerTimeState3D> goal_;
  std::vector<Point3> spatial_incumbent_;
  double goal_cost_s_{std::numeric_limits<double>::infinity()};
  std::uint64_t queue_sequence_{0U};
  std::size_t adaptive_edges_in_extracted_path_{0U};
  ExecutionTimeOpenQueue3D open_{};
  std::unordered_map<PersistentPlannerTimeState3D, double,
                     PersistentPlannerTimeState3DHash>
      costs_;
  std::unordered_map<PersistentPlannerTimeState3D, PersistentPlannerTimeState3D,
                     PersistentPlannerTimeState3DHash>
      parents_;
};

struct PathPostprocessorContext3D {
  std::size_t maximum_shortcut_checks{0U};
  std::function<bool(const Point3&, const Point3&, bool)> segment_valid;
  std::function<FlightPathTimeProfile3D(const std::vector<Point3>&)> time_profile;
};

class PathPostprocessor3D final {
public:
  [[nodiscard]] std::vector<Point3> shortcut(const std::vector<Point3>& path,
                                             const PathPostprocessorContext3D& context,
                                             std::size_t& checks,
                                             std::size_t& applied) const;
};

class AnytimePlannerCoordinator3D final {
public:
  void reset() noexcept;
  void retain(SpatialRouteCandidate3D candidate);
  [[nodiscard]] std::optional<SpatialRouteCandidate3D>
  consider(SpatialRouteCandidate3D candidate);
  [[nodiscard]] const SpatialRouteCandidate3D* incumbent() const noexcept;

private:
  std::optional<SpatialRouteCandidate3D> incumbent_;
};

class PersistentDStarLitePlanner3DImpl final {
public:
  explicit PersistentDStarLitePlanner3DImpl(const PersistentPlannerConfig3D& config);

  [[nodiscard]] PlannerUpdate3D plan(const PersistentPlannerRequest3D& request);
  void reset() noexcept;
  [[nodiscard]] const PersistentPlannerConfig3D& config() const noexcept;

private:
  [[nodiscard]] bool validRequest(const PersistentPlannerRequest3D& request) const;
  [[nodiscard]] PersistentPlannerWorldUpdate3D
  updateWorld(const PersistentPlannerWorld3D& world);
  void installWorld(const PersistentPlannerWorld3D& world);
  // Starts every search over: the backward session, the feasibility frontier,
  // and the execution-time refinement all restart from these endpoints.
  void initializeSearch(const PersistentPlannerRequest3D& request,
                        PersistentPlannerNode3D start, PersistentPlannerNode3D goal);
  [[nodiscard]] FeasiblePathSearch3D::Endpoints3D searchEndpoints() const noexcept;
  [[nodiscard]] std::optional<std::vector<Point3>>
  rebaseIncumbent(const std::vector<Point3>& incumbent, const Point3& start,
                  const Point3& goal) const;
  [[nodiscard]] FlightPathTimeProfile3D
  pathTimeProfile(const std::vector<Point3>& path, const Vec3& initial_velocity) const;
  [[nodiscard]] std::optional<SpatialRouteCandidate3D>
  makeCandidate(std::vector<Point3> path, SpatialRouteCandidateSource3D source,
                const Vec3& initial_velocity) const;
  [[nodiscard]] ExecutionTimeRefiner3D::Request3D
  refinementRequest(const PersistentPlannerRequest3D& request,
                    PersistentPlannerNode3D start_anchor,
                    PersistentPlannerNode3D goal_anchor) const noexcept;
  [[nodiscard]] std::vector<GridIndex3D>
  changedOccupiedCells(const PersistentPlannerWorld3D& previous,
                       const PersistentPlannerWorld3D& current,
                       bool dirty_chunks_complete) const;

  PersistentPlannerConfig3D config_{};
  PersistentPlannerWorld3D world_{};
  PlannerLattice3D lattice_;
  PersistentPlannerNode3D start_{};
  PersistentPlannerNode3D last_start_{};
  PersistentPlannerNode3D goal_{};
  Point3 exact_start_{};
  Point3 exact_goal_{};
  std::uint64_t mission_epoch_{0U};
  bool initialized_{false};
  DStarLiteSession3D dstar_session_;
  FeasiblePathSearch3D feasibility_search_;
  ExecutionTimeRefiner3D execution_time_refiner_;
  PathPostprocessor3D path_postprocessor_{};
  AnytimePlannerCoordinator3D coordinator_;
  std::size_t adaptive_edges_in_extracted_path_{0U};
};

} // namespace drone_city_nav::detail
