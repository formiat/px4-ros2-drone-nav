#pragma once

#include "drone_city_nav/persistent_dstar_lite_planner_3d.hpp"

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace drone_city_nav::detail {

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

struct PersistentPlannerWorldUpdate3D {
  bool accepted{false};
  bool requires_reset{false};
  bool occupied_world_unchanged{false};
  bool occupied_cells_removed{false};
  std::vector<GridIndex3D> changed_cells;
};

class PersistentDStarLitePlanner3DImpl final {
public:
  explicit PersistentDStarLitePlanner3DImpl(const PersistentPlannerConfig3D& config);

  [[nodiscard]] PersistentPlannerResult3D
  plan(const PersistentPlannerRequest3D& request);
  void reset() noexcept;
  [[nodiscard]] const PersistentPlannerConfig3D& config() const noexcept;

private:
  using OpenQueue =
      std::priority_queue<DStarLiteQueueEntry3D, std::vector<DStarLiteQueueEntry3D>,
                          DStarLiteQueueEntryCompare3D>;
  using TimeOpenQueue =
      std::priority_queue<PersistentPlannerTimeQueueEntry3D,
                          std::vector<PersistentPlannerTimeQueueEntry3D>,
                          PersistentPlannerTimeQueueEntryCompare3D>;

  [[nodiscard]] bool validRequest(const PersistentPlannerRequest3D& request) const;
  [[nodiscard]] PersistentPlannerWorldUpdate3D
  updateWorld(const PersistentPlannerWorld3D& world);
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
  void updateAffectedVertices(const std::vector<GridIndex3D>& changed_cells,
                              std::size_t& affected_states);
  [[nodiscard]] std::optional<DStarLiteQueueEntry3D> currentTop();
  [[nodiscard]] bool shortestPathComplete();
  [[nodiscard]] bool computeShortestPath(std::chrono::steady_clock::time_point deadline,
                                         std::size_t maximum_expansions,
                                         std::size_t& expansions);
  [[nodiscard]] std::vector<Point3> extractPath();
  [[nodiscard]] std::vector<Point3> shortcutPath(const std::vector<Point3>& path,
                                                 std::size_t& checks,
                                                 std::size_t& applied,
                                                 const Vec3& initial_velocity) const;
  [[nodiscard]] std::optional<std::vector<Point3>>
  rebaseIncumbent(const Point3& start, const Point3& goal) const;
  [[nodiscard]] bool pathRawValid(const std::vector<Point3>& path) const;
  [[nodiscard]] FlightPathTimeProfile3D
  pathTimeProfile(const std::vector<Point3>& path, const Vec3& initial_velocity) const;
  void populatePathMetrics(PersistentPlannerResult3D& result,
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
  GridBounds3D raw_bounds_{};
  int width_{0};
  int height_{0};
  int depth_{0};
  PersistentPlannerNode3D start_{};
  PersistentPlannerNode3D last_start_{};
  PersistentPlannerNode3D goal_{};
  Point3 exact_start_{};
  Point3 exact_goal_{};
  std::uint64_t mission_epoch_{0U};
  std::uint64_t search_generation_{0U};
  std::uint64_t repair_generation_{0U};
  std::uint64_t queue_token_{0U};
  std::uint64_t queue_sequence_{0U};
  double key_modifier_{0.0};
  bool initialized_{false};
  bool dstar_cost_to_goal_heuristic_admissible_{true};
  OpenQueue open_{};
  std::unordered_map<PersistentPlannerNode3D, DStarLiteRecord3D,
                     PersistentPlannerNode3DHash>
      records_;
  std::unordered_map<PersistentPlannerEdge3D, double, PersistentPlannerEdge3DHash>
      edge_cost_cache_;
  bool execution_time_search_initialized_{false};
  bool execution_time_search_complete_{false};
  bool execution_time_start_from_rest_{false};
  PersistentPlannerTimeState3D execution_time_start_{};
  std::optional<PersistentPlannerTimeState3D> execution_time_goal_;
  std::vector<Point3> execution_time_spatial_incumbent_;
  double execution_time_goal_cost_s_{std::numeric_limits<double>::infinity()};
  std::uint64_t execution_time_queue_sequence_{0U};
  TimeOpenQueue execution_time_open_{};
  std::unordered_map<PersistentPlannerTimeState3D, double,
                     PersistentPlannerTimeState3DHash>
      execution_time_costs_;
  std::unordered_map<PersistentPlannerTimeState3D, PersistentPlannerTimeState3D,
                     PersistentPlannerTimeState3DHash>
      execution_time_parents_;
  std::vector<Point3> incumbent_;
  std::size_t lattice_edge_queries_{0U};
  std::size_t raw_edge_validation_checks_{0U};
  std::size_t adaptive_edge_queries_{0U};
  std::size_t adaptive_edges_in_extracted_path_{0U};
  std::size_t maximum_queried_lattice_level_{0U};
};

} // namespace drone_city_nav::detail
