#pragma once

#include "drone_city_nav/occupied_collision_oracle_3d.hpp"
#include "drone_city_nav/persistent_dstar_lite_planner_3d.hpp"

#include <array>
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

// One raw cell whose occupancy changed, as the lattice sees it.
struct LatticeChangedCell3D {
  Point3 center{};
  bool occupied_now{false};
};

using LatticeChangesByChunk3D =
    std::unordered_map<OccupancyChunkIndex3D, std::vector<LatticeChangedCell3D>,
                       OccupancyChunkIndex3DHash>;

// Whether a cell centre lies within the swept body of the segment.
using LatticeSegmentTouch3D = std::function<bool(
    const Point3& center, const Point3& first, const Point3& second)>;

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
  // A direct raw connector from `node` to the exact goal, priced with the
  // ranking, competing in the queue with the labelled frontier.
  bool goal_connector{false};
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
  // The request world was older than the resident one; the resident world
  // was kept as the authority.
  bool resident_world_retained{false};
  std::vector<GridIndex3D> changed_cells;
  double diff_ms{0.0};
  double install_ms{0.0};
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
// Euclidean distance from a point to an axis-aligned voxel box; zero inside.
[[nodiscard]] double voxelBoxDistance3D(const Point3& point, const Point3& box_minimum,
                                        const Point3& box_maximum) noexcept;

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
  // Number of level-zero nodes; dense per-node storage is sized by it.
  [[nodiscard]] std::size_t nodeCount() const noexcept;
  [[nodiscard]] std::size_t linearIndex(PersistentPlannerNode3D node) const noexcept;
  [[nodiscard]] PersistentPlannerNode3D nodeAt(std::size_t index) const noexcept;

  // Drops the cached cost of an evaluated edge. A D* label can depend only on
  // an edge whose cost was evaluated, so repair invalidates exactly those.
  [[nodiscard]] bool forgetEdgeCost(const PersistentPlannerEdge3D& edge);
  // Drops the cached cost of an evaluated edge only when the change touching
  // it can move it: a cell that became occupied can block a clear edge but
  // never unblock a blocked one, and a cell that became free can unblock a
  // blocked edge but never block a clear one. Surface flicker in a persistent
  // raw map adds and removes cells in equal numbers; pricing only the edges a
  // change can move keeps repair proportional to real change.
  [[nodiscard]] bool forgetEdgeCostForChange(const PersistentPlannerEdge3D& edge,
                                             bool occupied_cell_added,
                                             bool occupied_cell_removed);
  // Whether labels priced with the previous clearance must be repaired: the
  // ranking factor is a soft cost, so a move that shifts it by less than a
  // small fraction keeps the search consistent enough and is only cached.
  [[nodiscard]] bool rankingRepairRequired(double previous_clearance_m,
                                           double current_clearance_m) const noexcept;
  // Forgets every level-zero edge incident to the node that the change can
  // move (clear edges for an occupied cell that appeared, blocked edges for
  // one that vanished); returns true when anything was forgotten. Coarse
  // scheduling of a very large change set uses it over the nodes of the
  // changed chunks.
  bool forgetNodeEdgesForChange(PersistentPlannerNode3D node, bool occupied_cell_added,
                                bool occupied_cell_removed);
  [[nodiscard]] bool hasEdgeCost(const PersistentPlannerEdge3D& edge) const noexcept;

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
  // Index of the first segment that fails traversal (segment s joins path[s-1]
  // and path[s]; 0 names a point outside the flight envelope or a degenerate
  // path), or nullopt when the path is traversable.
  [[nodiscard]] std::optional<std::size_t>
  firstInvalidSegment(const std::vector<Point3>& path) const;
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
  // Raw flight time scaled by the soft clearance ranking. D* Lite labels and
  // the execution-time refinement rank with it; edge traversability and path
  // validity stay on the raw cost.
  [[nodiscard]] double rankedEdgeCost(PersistentPlannerNode3D first,
                                      PersistentPlannerNode3D second);
  // The same ranking with node clearances derived only within
  // `clearance_reach_m`; a clearance at the reach ranks as that reach. The
  // feasibility-first search uses a short reach so every explored node stays
  // cheap to price.
  [[nodiscard]] double rankedEdgeCost(PersistentPlannerNode3D first,
                                      PersistentPlannerNode3D second,
                                      double clearance_reach_m);
  // Flight time of a straight segment scaled by the worst ranking factor
  // sampled along it, with clearances derived within `clearance_reach_m`.
  [[nodiscard]] double rankedSegmentTimeS(const Point3& first, const Point3& second,
                                          double clearance_reach_m) const;
  // Distance from a node to the nearest raw occupied cell, capped at the
  // clearance ranking distance. It is derived ranking evidence computed from
  // the resident raw grid and cached per node.
  [[nodiscard]] double nodeClearanceM(PersistentPlannerNode3D node);
  // The same clearance at an arbitrary point, uncached.
  [[nodiscard]] double pointClearanceM(const Point3& point) const;
  // Ranking factor of a body clearance (raw clearance less the footprint
  // radius): 1 beyond the ranking distance, growing through the soft band
  // and steeply through the critical band.
  [[nodiscard]] double
  rankingFactorForBodyClearance(double body_clearance_m) const noexcept;
  // The same curve scaled to another reach: unity at and beyond it.
  [[nodiscard]] double rankingFactorForBodyClearance(double body_clearance_m,
                                                     double distance_m) const noexcept;
  // Execution time of a point path with every segment scaled by the worst
  // ranking factor sampled along it; stationary turn time is not scaled.
  [[nodiscard]] double rankedPathTimeS(const std::vector<Point3>& path,
                                       const FlightPathTimeProfile3D& profile) const;
  // Drops every cached node clearance; they are re-derived lazily.
  void resetNodeClearances() noexcept;
  // Drops cached clearances of nodes whose raw surroundings changed.
  // Stamps the chunks an occupied change touched. Cached node clearances are
  // validated lazily against these stamps when they are next consulted, so a
  // scan that changes thousands of cells costs the scheduler one stamp per
  // chunk instead of a reach-box walk per changed cell.
  void noteChangedChunks(
      const std::unordered_set<OccupancyChunkIndex3D, OccupancyChunkIndex3DHash>&
          changed_chunks);
  // Nodes whose lazily re-derived clearance moved their ranking factor past
  // the repair tolerance since the last call; the search repairs the labelled
  // ones among them.
  [[nodiscard]] std::vector<PersistentPlannerNode3D> takeMovedClearances();
  [[nodiscard]] std::size_t clearancesRederived() const noexcept;
  // The current clearance of a node whose clearance was priced before, or
  // nullopt when it never was. A stale cache entry is re-derived first.
  [[nodiscard]] std::optional<double> cachedNodeClearance(PersistentPlannerNode3D node,
                                                          double reach_m);
  // Distance within which an occupied change can alter a node's cached
  // clearance. Zero when clearance ranking is disabled.
  [[nodiscard]] double clearanceRankingReachM() const noexcept;
  // Forgets cached adaptive edges that a changed cell can move (the polarity
  // rule of forgetEdgeCostForChange) and whose swept body `touches` the cell
  // (centre, edge start, edge end). Only edges indexed under a changed chunk
  // are tested, so the cost stays bounded by the cache near the change.
  [[nodiscard]] std::vector<PersistentPlannerEdge3D>
  forgetAdaptiveEdgesTouching(const LatticeChangesByChunk3D& changes,
                              const LatticeSegmentTouch3D& touches);

  // Edge traversability from the shared cost cache, for a search that owns its
  // own cost model. A finite cached cost is exactly a traversable edge.
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
  // Level-zero edges are dense: every node owns the thirteen canonical edges
  // that leave it towards a lexicographically greater neighbour, two state
  // bits each (unknown, clear, blocked). Their raw flight time depends only on
  // the direction, so it is tabulated once per grid geometry. Searches query
  // millions of level-zero edges per second; a hash lookup per query was the
  // dominant planner cost.
  static constexpr std::size_t kLevelZeroDirections{13U};

  struct LevelZeroSlot {
    std::size_t node{0U};
    std::size_t direction{0U};
  };

  std::vector<std::uint32_t> level_zero_edge_states_;
  std::array<double, kLevelZeroDirections> level_zero_edge_time_s_{};
  [[nodiscard]] LevelZeroSlot
  levelZeroSlot(const PersistentPlannerEdge3D& canonical) const noexcept;
  [[nodiscard]] unsigned levelZeroState(LevelZeroSlot slot) const noexcept;
  void setLevelZeroState(LevelZeroSlot slot, unsigned state) noexcept;
  // True when no raw chunk lies within the swept reach of any level-zero edge
  // leaving the node; every such edge is then clear without a swept check.
  [[nodiscard]] bool
  surroundingsUnoccupied(PersistentPlannerNode3D node) const noexcept;
  void markSurroundingLevelZeroEdgesClear(PersistentPlannerNode3D node) noexcept;
  // True when both endpoints' already cached raw clearance exceeds half the
  // edge plus the body extent, which clears the whole swept edge without
  // validation. Uncached clearances are not derived here.
  [[nodiscard]] bool endpointClearanceClears(PersistentPlannerNode3D first,
                                             PersistentPlannerNode3D second);
  // Adaptive (level > 0) edges are sparse and keep a keyed cache.
  std::unordered_map<PersistentPlannerEdge3D, double, PersistentPlannerEdge3DHash>
      adaptive_edge_cost_cache_;

  struct CachedNodeClearance {
    double clearance_m{0.0};
    // The reach the clearance was derived within: the value is exact below
    // it and only a lower bound of the reach at it.
    double cap_m{0.0};
    // The change epoch the clearance is known to be current for.
    std::uint64_t change_epoch{0U};
  };

  std::unordered_map<PersistentPlannerNode3D, CachedNodeClearance,
                     PersistentPlannerNode3DHash>
      node_clearance_cache_;
  // Change epoch of the last occupied change per raw chunk, dense over the
  // chunk grid so a staleness check reads its reach box without hashing; the
  // epoch advances with every noted change set.
  std::vector<std::uint64_t> chunk_change_epoch_;
  // How close occupied evidence lies to a chunk, in chunk rings, dense over
  // the chunk grid and conservative (a ring is recorded when a chunk that far
  // away ever held occupied evidence; zero means none within the ranking
  // reach). A node whose chunk lies beyond the rings a reach spans ranks at
  // unity without a clearance query, so a search through open air stays as
  // cheap as an unranked one, and the short feasibility reach skips more than
  // the full ranking reach.
  std::vector<std::uint8_t> chunk_occupied_ring_;
  int near_occupied_chunk_radius_{0};
  int chunk_columns_{0};
  int chunk_rows_{0};
  int chunk_layers_{0};
  // Sizes the dense per-chunk tables for the raw grid bounds, keeping their
  // contents when the geometry is unchanged.
  void ensureChunkTables(const GridBounds3D& bounds);
  [[nodiscard]] std::optional<std::size_t>
  chunkSlot(const OccupancyChunkIndex3D& chunk) const noexcept;
  void markNearOccupied(const OccupancyChunkIndex3D& chunk) noexcept;
  [[nodiscard]] bool nearOccupied(const Point3& point, double reach_m) const noexcept;
  std::uint64_t change_epoch_{0U};
  std::vector<PersistentPlannerNode3D> moved_clearances_;
  std::size_t clearances_rederived_{0U};
  // Whether a chunk within `reach_m` of the point changed after the given
  // epoch.
  [[nodiscard]] bool clearanceStale(const Point3& point, double reach_m,
                                    std::uint64_t change_epoch) const noexcept;
  [[nodiscard]] double deriveNodeClearance(const Point3& point, double cap_m) const;
  [[nodiscard]] double nodeClearanceWithin(PersistentPlannerNode3D node, double cap_m);
  // Adaptive (level > 0) cached edges keyed by every chunk their
  // margin-expanded extent touches, so an occupied change finds the long
  // edges it can affect without scanning the whole edge cache. Entries of
  // edges forgotten through another chunk are pruned lazily.
  std::unordered_map<OccupancyChunkIndex3D, std::vector<PersistentPlannerEdge3D>,
                     OccupancyChunkIndex3DHash>
      adaptive_edges_by_chunk_;
  // Body margin of a swept edge: footprint extent plus the raw voxel half
  // diagonal. An occupied change outside it cannot touch the edge.
  double adaptive_edge_horizontal_margin_m_{0.0};
  double adaptive_edge_vertical_margin_m_{0.0};
  void indexAdaptiveEdge(const PersistentPlannerEdge3D& edge);
  void unindexAdaptiveEdge(const PersistentPlannerEdge3D& edge);
  template<typename Visitor>
  void forEachChunkTouching(const Point3& first, const Point3& second,
                            Visitor&& visitor) const;
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

  struct ScheduleStatistics {
    double total_ms{0.0};
    double ranking_ms{0.0};
    std::size_t edges_forgotten{0U};
    // Labelled nodes scheduled because a lazily re-derived clearance moved.
    std::size_t clearances_tightened{0U};
  };

  void scheduleAffectedVertices(const PersistentPlannerWorld3D& world,
                                const std::vector<GridIndex3D>& changed_cells,
                                std::size_t& affected_states);

  [[nodiscard]] const ScheduleStatistics& scheduleStatistics() const noexcept {
    return schedule_statistics_;
  }

  [[nodiscard]] bool
  continueAffectedVertexRepair(std::chrono::steady_clock::time_point deadline,
                               std::size_t maximum_vertices,
                               std::size_t& processed_vertices);
  // Queues repair for the labelled nodes whose lazily re-derived clearance
  // moved their ranking factor, and for their labelled neighbours.
  void scheduleMovedClearances();
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
  ScheduleStatistics schedule_statistics_{};
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
  // The lattice node the labels were seeded from. A candidate departs from
  // the exact start straight to it, so the anchor stays valid while that
  // departure segment does, wherever the vehicle drifted meanwhile.
  [[nodiscard]] PersistentPlannerNode3D anchor() const noexcept;
  // True when the last advance() emptied the frontier without a candidate.
  [[nodiscard]] bool frontierExhausted() const noexcept;
  [[nodiscard]] std::size_t exploredNodes() const noexcept;
  // Smallest distance from any expanded node to the exact goal so far.
  [[nodiscard]] double closestGoalDistanceM() const noexcept;
  // Candidates that failed validation since construction, by what failed:
  // full restarts (departure, degenerate candidate) and prefix reseeds
  // (an interior segment whose edge no longer survives the resident world).
  [[nodiscard]] std::size_t restartCount() const noexcept;
  [[nodiscard]] std::size_t prefixReseedCount() const noexcept;
  [[nodiscard]] std::size_t lastInvalidSegment() const noexcept;

  // Expands the frontier until a path is found, the budget is spent, or the
  // deadline passes. Seeds itself from the endpoints on first use and on every
  // restart, so `endpoints.start` is the current start anchor.
  [[nodiscard]] std::optional<std::vector<Point3>>
  advance(const Endpoints3D& endpoints, std::chrono::steady_clock::time_point deadline,
          std::size_t maximum_expansions, std::size_t& expansions);

private:
  static constexpr std::uint32_t kNoParent{std::numeric_limits<std::uint32_t>::max()};

  void initialize(const Endpoints3D& endpoints);
  void ensureLabelStorage();
  [[nodiscard]] bool labelled(std::size_t index) const noexcept;
  // Lattice nodes from the anchor to the terminal, or empty when the parent
  // chain is broken or too long.
  [[nodiscard]] std::vector<PersistentPlannerNode3D>
  reconstructNodes(PersistentPlannerNode3D terminal) const;
  // Exact-start departure, the nodes, and the exact goal. The departure joins
  // the first node the exact start reaches directly; nodes before it are
  // dropped, so a drifted vehicle keeps the labels it can still use.
  [[nodiscard]] std::optional<std::vector<Point3>>
  pathFromNodes(const Endpoints3D& endpoints,
                std::vector<PersistentPlannerNode3D>& nodes) const;
  // Restarts the search seeded with the labels of `prefix`, whose edges were
  // just validated on the resident world.
  void reseedFromPrefix(const Endpoints3D& endpoints,
                        const std::vector<PersistentPlannerNode3D>& prefix);

  const PersistentPlannerConfig3D* config_{nullptr};
  PlannerLattice3D* lattice_{nullptr};
  bool initialized_{false};
  PersistentPlannerNode3D anchor_{};
  std::uint64_t queue_sequence_{0U};
  bool frontier_exhausted_{false};
  double closest_goal_distance_m_{std::numeric_limits<double>::infinity()};
  FeasibilityOpenQueue3D open_{};
  // Dense labels stamped with the generation that wrote them; a reset bumps
  // the generation instead of clearing the arrays.
  std::vector<double> cost_s_;
  std::vector<std::uint32_t> label_generation_;
  std::vector<std::uint32_t> parent_index_;
  std::uint32_t generation_{0U};
  std::size_t explored_{0U};
  std::size_t restart_count_{0U};
  std::size_t prefix_reseed_count_{0U};
  std::size_t last_invalid_segment_{0U};
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
  // Keeps the search across an occupied change; drops blocked incumbents.
  void rebaseWorld();
  // Largest total cost a state may still have to be worth expanding.
  [[nodiscard]] double improvementBoundS() const noexcept;
  [[nodiscard]] std::vector<Point3> bestPath();
  [[nodiscard]] std::size_t records() const noexcept;
  [[nodiscard]] std::size_t openEntries() const noexcept;
  [[nodiscard]] double objectiveSeconds() const noexcept;
  [[nodiscard]] std::size_t adaptiveEdgesInExtractedPath() const noexcept;

  [[nodiscard]] std::optional<std::vector<Point3>>
  advance(std::chrono::steady_clock::time_point deadline,
          std::size_t maximum_expansions, std::size_t& expansions);

  // Installs a raw-valid spatial route as the anytime incumbent and bound.
  void seedIncumbent(const std::vector<Point3>& spatial_route);

private:
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
  // Anchored start of the feasibility search; see plan() for the hysteresis.
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
  // Session that last received the incumbent; see
  // PersistentPlannerRequest3D::session_id.
  std::uint64_t published_session_id_{0U};
  // Session whose incumbent-discard request was already honoured.
  std::uint64_t discarded_session_id_{0U};
  std::uint64_t applied_incumbent_rejection_sequence_{0U};
};

} // namespace drone_city_nav::detail
