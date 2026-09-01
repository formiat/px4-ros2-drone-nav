#pragma once

#include "drone_city_nav/occupied_collision_oracle_3d.hpp"
#include "drone_city_nav/persistent_dstar_lite_planner_3d.hpp"

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <limits>
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

class LatticeState3D final {
public:
  void reset() noexcept;

private:
  friend class PersistentDStarLitePlanner3DImpl;

  GridBounds3D raw_bounds_{};
  int width_{0};
  int height_{0};
  int depth_{0};
};

class DStarLiteSessionState3D final {
public:
  void reset() noexcept;

private:
  friend class PersistentDStarLitePlanner3DImpl;

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
  std::unordered_map<PersistentPlannerEdge3D, double, PersistentPlannerEdge3DHash>
      edge_cost_cache_;
  std::deque<PersistentPlannerNode3D> pending_repair_nodes_;
  std::unordered_set<PersistentPlannerNode3D, PersistentPlannerNode3DHash>
      pending_repair_members_;
};

class FeasiblePathSearchState3D final {
public:
  void reset() noexcept;

private:
  friend class PersistentDStarLitePlanner3DImpl;

  bool initialized_{false};
  std::uint64_t queue_sequence_{0U};
  FeasibilityOpenQueue3D open_{};
  std::unordered_map<PersistentPlannerNode3D, double, PersistentPlannerNode3DHash>
      costs_;
  std::unordered_map<PersistentPlannerNode3D, PersistentPlannerNode3D,
                     PersistentPlannerNode3DHash>
      parents_;
};

class ExecutionTimeRefinementState3D final {
public:
  void reset() noexcept;

private:
  friend class PersistentDStarLitePlanner3DImpl;

  bool initialized_{false};
  bool complete_{false};
  bool start_from_rest_{false};
  PersistentPlannerTimeState3D start_{};
  std::optional<PersistentPlannerTimeState3D> goal_;
  std::vector<Point3> spatial_incumbent_;
  double goal_cost_s_{std::numeric_limits<double>::infinity()};
  std::uint64_t queue_sequence_{0U};
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
  void initializeSearch(const PersistentPlannerRequest3D& request,
                        PersistentPlannerNode3D start, PersistentPlannerNode3D goal);
  [[nodiscard]] bool sameGridGeometry(const GridBounds3D& bounds) const noexcept;
  void configureGridGeometry(const GridBounds3D& bounds);
  [[nodiscard]] bool nodeInside(PersistentPlannerNode3D node) const noexcept;
  [[nodiscard]] int maximumLatticeScale() const noexcept;
  [[nodiscard]] std::size_t latticeLevel(PersistentPlannerNode3D first,
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
  [[nodiscard]] std::vector<PersistentPlannerNode3D>
  adjacentNodes(PersistentPlannerNode3D node) const;
  [[nodiscard]] double heuristic(PersistentPlannerNode3D first,
                                 PersistentPlannerNode3D second) const noexcept;
  [[nodiscard]] double rawEdgeCost(PersistentPlannerNode3D first,
                                   PersistentPlannerNode3D second);
  [[nodiscard]] DStarLiteKey3D calculateKey(PersistentPlannerNode3D node);
  void enqueue(PersistentPlannerNode3D node, DStarLiteRecord3D& record);
  void updateVertex(PersistentPlannerNode3D node);
  void scheduleAffectedVertices(const std::vector<GridIndex3D>& changed_cells,
                                std::size_t& affected_states);
  [[nodiscard]] bool
  continueAffectedVertexRepair(std::chrono::steady_clock::time_point deadline,
                               std::size_t maximum_vertices,
                               std::size_t& processed_vertices);
  [[nodiscard]] std::optional<DStarLiteQueueEntry3D> currentTop();
  [[nodiscard]] bool shortestPathComplete();
  [[nodiscard]] bool computeShortestPath(std::chrono::steady_clock::time_point deadline,
                                         std::size_t maximum_expansions,
                                         std::size_t& expansions);
  [[nodiscard]] std::optional<std::vector<Point3>>
  findFeasiblePath(std::chrono::steady_clock::time_point deadline,
                   std::size_t maximum_expansions, std::size_t& expansions);
  void resetFeasibilitySearch() noexcept;
  void initializeFeasibilitySearch();
  [[nodiscard]] std::vector<Point3> extractPath();
  [[nodiscard]] std::optional<std::vector<Point3>>
  rebaseIncumbent(const std::vector<Point3>& incumbent, const Point3& start,
                  const Point3& goal) const;
  [[nodiscard]] bool pathRawValid(const std::vector<Point3>& path) const;
  [[nodiscard]] FlightPathTimeProfile3D
  pathTimeProfile(const std::vector<Point3>& path, const Vec3& initial_velocity) const;
  [[nodiscard]] std::optional<SpatialRouteCandidate3D>
  makeCandidate(std::vector<Point3> path, SpatialRouteCandidateSource3D source,
                const Vec3& initial_velocity) const;
  void resetExecutionTimeSearch() noexcept;
  [[nodiscard]] PersistentPlannerDirection3D
  directionForVector(const Vec3& vector) const noexcept;
  [[nodiscard]] PersistentPlannerDirection3D
  directionForEdge(PersistentPlannerNode3D first,
                   PersistentPlannerNode3D second) const noexcept;
  [[nodiscard]] Vec3
  directionVector(PersistentPlannerDirection3D direction) const noexcept;
  [[nodiscard]] PersistentPlannerTimeState3D
  executionTimeStartState(const PersistentPlannerRequest3D& request,
                          PersistentPlannerNode3D start_anchor) const noexcept;
  void initializeExecutionTimeSearch(const PersistentPlannerRequest3D& request,
                                     PersistentPlannerTimeState3D start_state);
  void seedExecutionTimeIncumbent();
  [[nodiscard]] bool hasExecutionTimeIncumbent() const noexcept;
  [[nodiscard]] std::vector<Point3> bestExecutionTimePath();
  [[nodiscard]] double
  executionTimeHeuristic(const PersistentPlannerTimeState3D& state) const noexcept;
  [[nodiscard]] double
  executionTimeTransitionCost(const PersistentPlannerTimeState3D& first,
                              const PersistentPlannerTimeState3D& second);
  [[nodiscard]] double
  executionTimeTerminalCost(const PersistentPlannerTimeState3D& state) const noexcept;
  [[nodiscard]] std::optional<std::vector<Point3>>
  continueExecutionTimeSearch(std::chrono::steady_clock::time_point deadline,
                              std::size_t maximum_expansions, std::size_t& expansions);
  [[nodiscard]] std::vector<Point3> extractExecutionTimePath();
  [[nodiscard]] std::vector<GridIndex3D>
  changedOccupiedCells(const PersistentPlannerWorld3D& previous,
                       const PersistentPlannerWorld3D& current) const;

  PersistentPlannerConfig3D config_{};
  PersistentPlannerWorld3D world_{};
  std::optional<OccupiedCollisionOracle3D> resident_collision_oracle_;
  std::optional<OccupiedCollisionOracle3D> departure_collision_oracle_;
  LatticeState3D lattice_{};
  PersistentPlannerNode3D start_{};
  PersistentPlannerNode3D last_start_{};
  PersistentPlannerNode3D goal_{};
  Point3 exact_start_{};
  Point3 exact_goal_{};
  std::uint64_t mission_epoch_{0U};
  bool initialized_{false};
  DStarLiteSessionState3D dstar_session_{};
  FeasiblePathSearchState3D feasibility_search_{};
  ExecutionTimeRefinementState3D execution_time_refiner_{};
  PathPostprocessor3D path_postprocessor_{};
  AnytimePlannerCoordinator3D coordinator_;
  std::size_t lattice_edge_queries_{0U};
  std::size_t raw_edge_validation_checks_{0U};
  std::size_t adaptive_edge_queries_{0U};
  std::size_t adaptive_edges_in_extracted_path_{0U};
  std::size_t maximum_queried_lattice_level_{0U};
};

} // namespace drone_city_nav::detail
