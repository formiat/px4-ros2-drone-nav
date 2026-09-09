#pragma once

// The persistent planner's lattice: nodes, edges, the world update contract,
// the lattice itself and the D* Lite session that searches it. The searches
// built on top of them live in persistent_dstar_lite_planner_3d_internal.hpp.

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
#include <span>
#include <tuple>
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

[[nodiscard]] inline bool nodeLess(const PersistentPlannerNode3D& first,
                                   const PersistentPlannerNode3D& second) noexcept {
  return std::tuple{first.z, first.y, first.x} <
         std::tuple{second.z, second.y, second.x};
}

// The undirected edge in its canonical orientation, so both directions of a
// traversal name the same cache entry.
[[nodiscard]] inline PersistentPlannerEdge3D
canonicalEdge(const PersistentPlannerNode3D first,
              const PersistentPlannerNode3D second) noexcept {
  if (nodeLess(first, second)) {
    return PersistentPlannerEdge3D{first, second};
  }
  return PersistentPlannerEdge3D{second, first};
}

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
  // Rebinds only the departure oracle (launch support, proprioceptive seed)
  // to the given world without touching the resident graph evidence.
  void installDepartureEvidence(const PersistentPlannerWorld3D& world);
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
  // move (clear and refined edges for an occupied cell that appeared, blocked
  // edges for one that vanished); returns true when anything was forgotten. Coarse
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

  // Why a departure could or could not be found: what the connector radius
  // held, what the body cleared, and the first leg the body swept into
  // evidence. A vehicle reporting start_unavailable says nothing else about
  // itself; this is what the diagnosis of a pocket needs.
  struct DepartureDiagnostics3D {
    std::size_t candidate_nodes{0U};
    std::size_t valid_nodes{0U};
    std::size_t rejected_legs{0U};
    std::size_t refinement_probes{0U};
    std::size_t refinement_reachable{0U};
    OccupiedCollisionResult3D first_leg_failure{};
    bool first_leg_failure_available{false};
    // No anchor cleared the departure envelope and the hull was asked instead.
    bool hull_fallback{false};
  };

  // Every node in the connector radius the body reaches, nearest first.
  [[nodiscard]] std::vector<PersistentPlannerNode3D>
  admissibleAnchors(const Point3& point, bool start_anchor,
                    DepartureDiagnostics3D* diagnostics = nullptr) const;
  [[nodiscard]] std::optional<PersistentPlannerNode3D>
  selectAnchor(const Point3& point, bool start_anchor) const;

  // How the search leaves the vehicle's exact position for the lattice.
  //
  // Normally that is one segment straight to the nearest admissible node.
  // Where the vehicle has come to rest close to occupied evidence — beside a
  // wall after a blocked-route stop — the swept body sweeps a jamb on every
  // such segment, no node in the connector radius is reachable, and the
  // planner reports start_unavailable for as long as the vehicle stays put:
  // the pocket that ended two of four recorded Urban runs. The lattice is
  // sparse, so the way out is almost always a short step the lattice cannot
  // express. A departure waypoint is that step: a free point off the lattice
  // that the body reaches from where it stands, and from which a node is
  // reachable under the ordinary raw rule. It is a route through free space
  // like any other, not an exemption from the body contract.
  struct DepartureConnection3D {
    std::optional<PersistentPlannerNode3D> anchor;
    // Free points the vehicle flies through before the anchor, in order.
    // Empty in ordinary flight; one point from the departure refinement; a
    // chain from the escape search.
    std::vector<Point3> waypoints;
    DepartureDiagnostics3D diagnostics;

    [[nodiscard]] bool available() const noexcept {
      return anchor.has_value();
    }
  };

  // `skipped_connections` passes over that many admissible connections before
  // returning one. The nearest reachable node is not always a useful one: it
  // can belong to a component the goal is not in, and the search then
  // exhausts itself against a start that was never going to work while
  // another node two metres away would have done. Advancing the skip on
  // evidence of exhaustion turns the single commitment into a bounded walk
  // over the reachable anchors.
  //
  // `preferred` is the anchor the searches are already seeded from. While it
  // stays admissible it is kept, whatever node is nearest now: a vehicle
  // resting near occupied evidence sees its nearest reachable node flip with
  // every raw scan, and each flip beyond a lattice diagonal restarts the
  // searches from nothing. The walk on exhaustion takes precedence.
  [[nodiscard]] DepartureConnection3D selectDepartureConnection(
      const Point3& start, std::size_t skipped_connections = 0U,
      std::optional<PersistentPlannerNode3D> preferred = std::nullopt) const;
  // How many admissible connections the start has, for bounding that walk.
  [[nodiscard]] std::size_t departureConnectionCount(const Point3& start) const;

  struct GoalConnection3D {
    std::optional<PersistentPlannerNode3D> anchor;
    // The point the search ends at: the goal itself, or a free point within
    // the goal tolerance when the goal's own surroundings admit no anchor.
    Point3 endpoint{};
    bool refined{false};

    [[nodiscard]] bool available() const noexcept {
      return anchor.has_value();
    }
  };

  // The lattice node the search ends at and the exact point it reaches. The
  // goal itself is taken while a node within the connector radius reaches it;
  // otherwise the finer grid the departure refinement probes is searched
  // around the goal, within `tolerance_m`, for the nearest free point a node
  // reaches. A goal a fresh scan has just put inside occupied evidence — a
  // point a hand's breadth above a floor the lidar only now sees — would
  // otherwise leave the planner with no search at all until that evidence
  // clears, and a vehicle that has stopped never clears it.
  [[nodiscard]] GoalConnection3D selectGoalConnection(const Point3& goal,
                                                      double tolerance_m) const;
  // Whether the body reaches `target` from `start`, through `waypoint` when
  // one is set. The leg leaving the vehicle carries the departure exemption
  // for contact evidence the body already holds; every later leg is ordinary
  // raw evidence.
  [[nodiscard]] bool departureReachable(const Point3& start,
                                        std::span<const Point3> waypoints,
                                        const Point3& target) const;
  // A short step off the straight segment between two node centres that clears
  // the body on both legs, or nothing. The lattice steps 2 m horizontally
  // against a 0.25 m map, so a doorway can be wide enough for the body and
  // still admit no straight node-to-node segment. Both legs are validated by
  // the ordinary raw rule, so this widens what the search can express, not
  // what the body may touch.
  [[nodiscard]] std::optional<Point3> refineBlockedEdge(PersistentPlannerNode3D first,
                                                        PersistentPlannerNode3D second);
  // The waypoint of a refined edge, for path extraction. Empty unless the
  // edge's cached state still says refined.
  [[nodiscard]] std::optional<Point3>
  edgeWaypoint(PersistentPlannerNode3D first, PersistentPlannerNode3D second) const;
  // Resets the per-update refinement probe budget.
  void beginEdgeRefinementBudget() noexcept;
  [[nodiscard]] bool edgeRefinementBudgetRemaining() const noexcept;
  // Forgets refined edges that an occupied cell that appeared can move: the
  // cell has to touch one of the edge's two legs through its waypoint, which
  // is the geometry the vehicle flies and lies off the straight segment the
  // node walk tests. Only edges indexed under a changed chunk are tested, as
  // for adaptive edges, so the cost stays bounded by the refined edges near
  // the change rather than by a wider node walk for every change.
  [[nodiscard]] std::vector<PersistentPlannerEdge3D>
  forgetRefinedEdgesTouching(const LatticeChangesByChunk3D& changes,
                             const LatticeSegmentTouch3D& touches);

  // The lattice edge a path segment stands for, in path order, or nullopt when
  // the lattice never priced that segment: the departure from the exact start,
  // the connector to the exact goal, a degenerate segment. A segment touching
  // a refined edge's waypoint names that edge. The raw sweep is the authority
  // on a candidate, and a leg it rejects has to reach the cache entry that
  // admitted it, whether that entry is a straight edge or a refined one.
  struct PathSegmentEdge3D {
    PersistentPlannerNode3D from{};
    PersistentPlannerNode3D to{};
  };

  [[nodiscard]] std::optional<PathSegmentEdge3D>
  pricedEdgeForSegment(const std::vector<Point3>& path, std::size_t segment) const;
  // Forgets an edge the raw sweep rejected on the resident world, and records
  // it for the persistent session: labels priced through the edge are stale
  // and the session repairs them. Returns whether anything was cached.
  bool rejectEdgeBySweep(const PersistentPlannerEdge3D& edge);
  [[nodiscard]] std::vector<PersistentPlannerEdge3D> takeSweepRejectedEdges();
  [[nodiscard]] bool pointInsideFlightEnvelope(const Point3& point) const noexcept;
  [[nodiscard]] bool rawSegmentValid(const Point3& first, const Point3& second) const;
  [[nodiscard]] bool departureSegmentValid(const Point3& first,
                                           const Point3& second) const;
  // `hull` judges the leg by the physical body alone rather than the departure
  // envelope: the vehicle leaves a tight spot at hover and upright, so the
  // envelope that contains the hull at every tilt is not what decides whether
  // it may leave at all.
  [[nodiscard]] OccupiedCollisionResult3D
  departureSegmentValidation(const Point3& first, const Point3& second,
                             bool hull = false) const;
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
  // Whether the point and the body's reach around it lie in observed space.
  // A static world is observed everywhere; an observed world answers from its
  // known voxels at the point and one footprint radius along each axis.
  [[nodiscard]] bool pointObserved(const Point3& point) const;
  // Ranking factor of a body clearance (raw clearance less the footprint
  // radius): 1 beyond the ranking distance, growing through the soft band,
  // and by the execution-time ratio the tube law imposes below cruise.
  [[nodiscard]] double
  rankingFactorForBodyClearance(double body_clearance_m) const noexcept;
  // The same curve scaled to another reach: unity at and beyond it.
  [[nodiscard]] double rankingFactorForBodyClearance(double body_clearance_m,
                                                     double distance_m) const noexcept;
  // Worst ranking factor sampled along one segment of a point path: what the
  // segment's translation time is scaled by. It depends on the segment's
  // geometry alone, so a path's factors survive every edit that leaves the
  // segment in place.
  [[nodiscard]] double rankedSegmentFactor(const Point3& first,
                                           const Point3& second) const;
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
  // The same world judged by the physical body alone, for a departure the
  // envelope admits nowhere.
  std::optional<OccupiedCollisionOracle3D> departure_hull_collision_oracle_;
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
  // Level-zero edges whose straight segment does not clear the body but which
  // a short step off it does. Only edges that failed the straight sweep are
  // here, so the map stays small and lives near occupied evidence. The cached
  // edge state, not this map, decides whether a waypoint is current.
  std::unordered_map<PersistentPlannerEdge3D, Point3, PersistentPlannerEdge3DHash>
      refined_edge_waypoints_;
  // Refinement probes spent in the current update. Every edge that fails its
  // straight sweep would otherwise pay for a fan of extra sweeps, and near
  // occupied evidence most edges fail: unbounded, the refinement takes the
  // whole compute budget and the search stops converging.
  std::size_t edge_refinement_probes_{0U};

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
  // Edges the raw sweep rejected since the last take; see rejectEdgeBySweep.
  std::vector<PersistentPlannerEdge3D> sweep_rejected_edges_;
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
  // Refined edges by the chunks their legs touch, for the same reason.
  std::unordered_map<OccupancyChunkIndex3D, std::vector<PersistentPlannerEdge3D>,
                     OccupancyChunkIndex3DHash>
      refined_edges_by_chunk_;
  // Records a refined edge's waypoint and indexes its legs.
  void rememberRefinedWaypoint(const PersistentPlannerEdge3D& edge,
                               const Point3& waypoint);
  // Drops a refined edge's waypoint and its index entries, if any.
  void forgetRefinedWaypoint(const PersistentPlannerEdge3D& edge);
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
  // Repairs the labelled endpoints of every edge the raw sweep rejected since
  // the last call: a label priced through such an edge is stale, and the
  // change scheduling never sees a rejection the sweep made on a candidate.
  void scheduleSweepRejectedEdges();
  void scheduleMovedClearances();
  [[nodiscard]] bool shortestPathComplete();
  [[nodiscard]] bool computeShortestPath(std::chrono::steady_clock::time_point deadline,
                                         std::size_t maximum_expansions,
                                         std::size_t& expansions);
  [[nodiscard]] std::vector<Point3>
  extractPath(const Point3& exact_start, const Point3& exact_goal,
              std::span<const Point3> departure_waypoints, std::size_t& adaptive_edges);

  [[nodiscard]] std::uint64_t searchGeneration() const noexcept;
  [[nodiscard]] std::uint64_t repairGeneration() const noexcept;
  [[nodiscard]] std::size_t records() const noexcept;
  [[nodiscard]] std::size_t openEntries() const noexcept;
  [[nodiscard]] std::size_t pendingRepairNodes() const noexcept;

private:
  [[nodiscard]] DStarLiteKey3D calculateKey(PersistentPlannerNode3D node);
  void enqueue(PersistentPlannerNode3D node, DStarLiteRecord3D& record);
  void updateVertex(PersistentPlannerNode3D node);
  // The optimized D* Lite vertex maintenance (Koenig & Likhachev): a
  // lowered g(u) tightens each predecessor's rhs through the one edge into u
  // instead of rescanning every successor of every predecessor, and a raised
  // g(u) rescans only the predecessors whose rhs went through u.
  void recomputeRhs(PersistentPlannerNode3D node, DStarLiteRecord3D& record);
  void updateVertexQueue(PersistentPlannerNode3D node, DStarLiteRecord3D& record);
  void lowerPredecessors(PersistentPlannerNode3D node, double node_g);
  void raisePredecessors(PersistentPlannerNode3D node, double previous_node_g);
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

} // namespace drone_city_nav::detail
