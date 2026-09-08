#pragma once

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
#include <span>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "persistent_dstar_lite_planner_3d_lattice_internal.hpp"

namespace drone_city_nav::detail {

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
    // See PlannerLattice3D::DepartureConnection3D.
    std::vector<Point3> departure_waypoints;
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
  // Occupied evidence changed on the resident world. Every label is kept; the
  // chain of lattice edges that reached it is re-validated lazily, when the
  // search next touches the label, and the labels behind an edge that no
  // longer survives are dropped and re-entered from their intact neighbours.
  void noteWorldChanged() noexcept;
  // True when the last advance() emptied the frontier without a candidate,
  // whether or not it restarted afterwards within the same call.
  [[nodiscard]] bool frontierExhausted() const noexcept;
  // The lattice nodes the frontier had labelled when it emptied, accumulated
  // over every exhaustion since the last clear: the components of the lattice
  // graph this start reaches and the goal is not in. An escape has to leave
  // them. Cleared by the owner when the vehicle moves on.
  [[nodiscard]] bool closedComponentMarked() const noexcept;
  [[nodiscard]] bool inClosedComponent(PersistentPlannerNode3D node) const noexcept;
  void clearClosedComponent() noexcept;
  [[nodiscard]] std::size_t exploredNodes() const noexcept;
  // Smallest distance from any expanded node to the exact goal so far.
  [[nodiscard]] double closestGoalDistanceM() const noexcept;
  // Candidates that failed validation since construction, by what failed:
  // full restarts (departure, degenerate candidate), and labels dropped
  // because a lattice edge on their chain no longer survives the resident
  // world.
  [[nodiscard]] std::size_t restartCount() const noexcept;
  [[nodiscard]] std::size_t invalidatedLabelCount() const noexcept;
  // Labels whose chain broke and that were re-parented through an intact
  // neighbour instead of being dropped.
  [[nodiscard]] std::size_t adoptedLabelCount() const noexcept;
  [[nodiscard]] std::size_t lastInvalidSegment() const noexcept;

  // Expands the frontier until a path is found, the budget is spent, or the
  // deadline passes. Seeds itself from the endpoints on first use and on every
  // restart, so `endpoints.start` is the current start anchor.
  [[nodiscard]] std::optional<std::vector<Point3>>
  advance(const Endpoints3D& endpoints, std::chrono::steady_clock::time_point deadline,
          std::size_t maximum_expansions, std::size_t& expansions);

private:
  static constexpr std::uint32_t kNoParent{std::numeric_limits<std::uint32_t>::max()};
  // The body of advance(); `exhausted` reports whether the frontier emptied
  // during this call, before any restart.
  [[nodiscard]] std::optional<std::vector<Point3>> advanceFrontier(
      const Endpoints3D& endpoints, std::chrono::steady_clock::time_point deadline,
      std::size_t maximum_expansions, std::size_t& expansions, bool& exhausted);
  // Longest parent chain any walk follows; longer means a loop of links.
  static constexpr std::size_t kMaximumChainWalk{1U << 20U};

  void initialize(const Endpoints3D& endpoints);
  void ensureLabelStorage();
  // Adds every labelled node to the closed component.
  void markClosedComponent();
  [[nodiscard]] bool labelled(std::size_t index) const noexcept;
  // Writes a label reached through a chain validated on the resident world.
  // Refuses a parent that descends from the label.
  [[nodiscard]] bool label(std::size_t index, double cost_from_start_s,
                           std::uint32_t parent, std::uint32_t depth);
  // Queues the label unless an entry with its current cost is already queued.
  void push(std::size_t index, PersistentPlannerNode3D node);
  // Whether the chain of lattice edges from the anchor to the label survives
  // the resident world. Edges are checked from the nearest ancestor already
  // validated on this world. The label behind a broken edge is re-parented
  // through an intact neighbour when one exists; otherwise it and the labels
  // below it on this chain are dropped and queued for re-entry.
  [[nodiscard]] bool chainValid(std::size_t index);
  // Re-parents a label whose parent edge broke through the cheapest adjacent
  // label validated on this epoch, or else through the cheapest labelled
  // neighbour outside the label's own subtree; the next walk validates that
  // parent's chain in turn.
  [[nodiscard]] bool adoptLabel(std::size_t index);
  // Whether the label's parent chain passes through the ancestor.
  [[nodiscard]] bool descendsFrom(std::size_t index,
                                  std::size_t ancestor) const noexcept;
  // Starts a validation epoch: every chain is walked again when next touched.
  // An epoch starts with every occupied change and every rejected edge; both
  // can invalidate chains validated earlier.
  void advanceValidationEpoch() noexcept;
  void invalidateLabel(std::size_t index);
  // Re-opens the intact labelled neighbours of every dropped label, so the
  // region behind a broken edge is re-entered from the labels around it. The
  // cascade stops at the deadline and resumes on the next advance.
  void drainInvalidations(std::chrono::steady_clock::time_point deadline);
  // Lattice nodes from the anchor to the terminal, or empty when the parent
  // chain is broken or too long.
  [[nodiscard]] std::vector<PersistentPlannerNode3D>
  reconstructNodes(PersistentPlannerNode3D terminal) const;
  // Exact-start departure, the nodes with the waypoints of refined edges
  // between them, and the exact goal. The departure joins the first node the
  // exact start reaches directly; nodes before it are dropped, so a drifted
  // vehicle keeps the labels it can still use.
  [[nodiscard]] std::optional<std::vector<Point3>>
  pathFromNodes(const Endpoints3D& endpoints,
                const std::vector<PersistentPlannerNode3D>& nodes) const;

  const PersistentPlannerConfig3D* config_{nullptr};
  PlannerLattice3D* lattice_{nullptr};
  bool initialized_{false};
  PersistentPlannerNode3D anchor_{};
  PersistentPlannerNode3D goal_{};
  std::uint64_t queue_sequence_{0U};
  bool frontier_exhausted_{false};
  double closest_goal_distance_m_{std::numeric_limits<double>::infinity()};
  FeasibilityOpenQueue3D open_{};
  // Dense labels stamped with the generation that wrote them; a reset bumps
  // the generation instead of clearing the arrays. Each label also carries
  // the validation epoch its chain was last walked on and whether an entry
  // with its current cost is queued.
  std::vector<double> cost_s_;
  std::vector<std::uint32_t> label_generation_;
  std::vector<std::uint32_t> parent_index_;
  std::vector<std::uint32_t> depth_;
  std::vector<std::uint32_t> validated_epoch_;
  std::vector<std::uint8_t> queued_;
  std::uint32_t generation_{0U};
  std::uint32_t validation_epoch_{1U};
  std::vector<std::uint32_t> chain_;
  std::vector<std::uint32_t> invalidation_queue_;
  std::vector<std::uint8_t> closed_component_;
  bool closed_component_marked_{false};
  // Edges the raw sweep rejected on the resident world: the sweep is the
  // authority for a candidate, so the search never offers them again on it.
  std::unordered_set<PersistentPlannerEdge3D, PersistentPlannerEdge3DHash>
      rejected_edges_;
  std::size_t explored_{0U};
  std::size_t restart_count_{0U};
  std::size_t invalidated_label_count_{0U};
  std::size_t adopted_label_count_{0U};
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

// Resumable search for a way out of a closed component of the lattice graph.
//
// The lattice is sparse relative to the map: a 2.4 m corridor carries no valid
// node unless its centre happens to fall on the grid, and once its walls are
// observed the nodes inside it vanish from the graph. A vehicle that entered
// such a corridor on a route the graph still had then stands in a region whose
// lattice component the goal is not in; both lattice searches exhaust it and
// can only start over. The way out exists at the body's own scale, and this
// search finds it there: it flood-fills a grid
// `departure_refinement_subdivisions` times finer than the lattice around the
// vehicle, validating every step with the ordinary raw rule (the first with the
// departure exemption), until it reaches a point from which a lattice node
// outside the closed component is reachable. That chain becomes the departure,
// so the body contract is unchanged: every leg is a raw-validated segment.
class EscapeSearch3D final {
public:
  struct Result3D {
    PersistentPlannerNode3D anchor{};
    std::vector<Point3> waypoints;
  };

  EscapeSearch3D(const PersistentPlannerConfig3D& config,
                 PlannerLattice3D& lattice) noexcept;

  void reset() noexcept;
  [[nodiscard]] bool initialized() const noexcept;
  // The fill emptied its frontier without reaching a way out.
  [[nodiscard]] bool exhausted() const noexcept;
  [[nodiscard]] std::size_t exploredCells() const noexcept;
  [[nodiscard]] std::size_t totalProbes() const noexcept;
  // Continues the fill from `start` (re-seeded when the vehicle moved by more
  // than a fine step) until a way out is found, the probe budget is spent, or
  // the deadline passes. `closed` says whether a lattice node belongs to a
  // component the lattice searches already exhausted.
  [[nodiscard]] std::optional<Result3D>
  advance(const Point3& start,
          const std::function<bool(PersistentPlannerNode3D)>& closed,
          std::chrono::steady_clock::time_point deadline, std::size_t maximum_probes,
          std::size_t& probes);

private:
  static constexpr std::uint32_t kNoCell{std::numeric_limits<std::uint32_t>::max()};

  struct CellOffset3D {
    int x{0};
    int y{0};
    int z{0};
  };

  struct QueueEntry3D {
    double cost_m{0.0};
    std::uint32_t cell{0U};

    [[nodiscard]] bool operator>(const QueueEntry3D& other) const noexcept {
      return cost_m != other.cost_m ? cost_m > other.cost_m : cell > other.cell;
    }
  };

  void begin(const Point3& start);
  [[nodiscard]] std::optional<std::uint32_t> cellAt(int x, int y, int z) const noexcept;
  [[nodiscard]] CellOffset3D offsetOf(std::uint32_t cell) const noexcept;
  [[nodiscard]] Point3 pointOf(std::uint32_t cell) const noexcept;
  [[nodiscard]] bool insideMap(const Point3& point) const noexcept;
  // A lattice node one lattice step around the point, outside the closed
  // components, that the body reaches from it.
  [[nodiscard]] std::optional<PersistentPlannerNode3D>
  exitFrom(const Point3& point,
           const std::function<bool(PersistentPlannerNode3D)>& closed,
           std::size_t& probes);

  const PersistentPlannerConfig3D* config_{nullptr};
  PlannerLattice3D* lattice_{nullptr};
  bool initialized_{false};
  bool exhausted_{false};
  Point3 origin_{};
  std::uint32_t origin_cell_{0U};
  int span_{0};
  int vertical_span_{0};
  int side_{0};
  double horizontal_step_m_{0.0};
  double vertical_step_m_{0.0};
  std::vector<std::uint8_t> settled_;
  std::vector<double> cost_m_;
  std::vector<std::uint32_t> parent_;
  std::priority_queue<QueueEntry3D, std::vector<QueueEntry3D>, std::greater<>> open_;
  // Node validity is one sweep per node; cached for the life of the fill.
  std::unordered_map<PersistentPlannerNode3D, bool, PersistentPlannerNode3DHash>
      node_valid_;
  std::size_t explored_{0U};
  std::size_t probes_{0U};
};

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
    // See PlannerLattice3D::DepartureConnection3D.
    std::vector<Point3> departure_waypoints;
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
  // What a shortcut is judged on: the same clearance-ranked execution time the
  // candidates compete on — every segment's translation time scaled by the
  // worst ranking factor sampled along it. Judging a shortcut on raw travel
  // time lets it buy seconds by dragging the route back against the wall the
  // search climbed away from, which is the objective the search was minimising
  // in the first place. The factor is asked per segment because it depends on
  // the segment's geometry alone: a shortcut changes one segment and leaves
  // the rest, and re-sampling the whole route's clearance for every candidate
  // shortcut is what made one update take sixty times its budget.
  std::function<double(const Point3&, const Point3&)> segment_factor;
  // The update's deadline. The pass is part of an anytime search: past the
  // deadline the route publishes as simplified so far, and the next update
  // simplifies further.
  std::optional<std::chrono::steady_clock::time_point> deadline;
};

// Moving a route's interior vertices off the walls they were placed against.
//
// A lattice node lands wherever the grid puts it, so a route through a 2.4 m
// doorway runs within a few centimetres of the jamb: execution then has to
// crawl through it, and the first freshly observed voxel of that jamb blocks
// the route. Sliding each vertex across the passage toward the local clearance
// maximum costs nothing in path length and buys the tube the width it needs.
struct PathClearanceCenteringContext3D {
  std::function<bool(const Point3&, const Point3&, bool)> segment_valid;
  // Raw clearance at a point: distance to the nearest occupied evidence.
  std::function<double(const Point3&)> clearance;
  // Whether a point and the body around it lie in observed space, free or
  // occupied. The clearance above is measured to *observed* evidence only, so
  // beside an unobserved wall it grows without bound and its gradient points
  // into the unknown; a vertex slid that way sits against the jamb the first
  // scan of it reveals. A probe in unobserved space contributes no gradient
  // and a candidate in unobserved space is never taken. Unknown space stays
  // traversable and free of charge: this only decides where an optional
  // geometric refinement may move a vertex. Empty means everything is observed.
  std::function<bool(const Point3&)> observed;
  // Raw clearance a vertex is content with; ascent stops there. In a passage
  // narrower than this the local maximum is the middle of the passage, which
  // is what the ascent finds.
  double target_clearance_m{0.0};
  double probe_step_m{0.25};
  std::size_t maximum_passes{3U};
  // Upper bound on clearance queries, so centering cannot eat a search budget.
  std::size_t maximum_clearance_queries{0U};
  // The update's deadline; centering stops there and the next update resumes.
  std::optional<std::chrono::steady_clock::time_point> deadline;
};

class PathPostprocessor3D final {
public:
  [[nodiscard]] std::vector<Point3> shortcut(const std::vector<Point3>& path,
                                             const PathPostprocessorContext3D& context,
                                             std::size_t& checks,
                                             std::size_t& applied) const;

  // Slides interior vertices across the passage toward more clearance. A move
  // is kept only when it raises the vertex's clearance and both incident
  // segments still validate against raw evidence, so the pass can never turn a
  // valid route into an invalid one.
  [[nodiscard]] std::vector<Point3>
  centerOnClearance(const std::vector<Point3>& path,
                    const PathClearanceCenteringContext3D& context,
                    std::size_t& queries, std::size_t& moved) const;
};

class AnytimePlannerCoordinator3D final {
public:
  explicit AnytimePlannerCoordinator3D(
      double continuity_improvement_margin_s = 0.0) noexcept
      : continuity_improvement_margin_s_{continuity_improvement_margin_s} {
  }

  void reset() noexcept;
  void retain(SpatialRouteCandidate3D candidate);
  // Replaces the incumbent when the candidate is better. A candidate that
  // leaves the vehicle on a different heading has to be better by
  // `continuity_improvement_margin_s`, because taking it turns the vehicle
  // around and discards the motion it already has; one that continues the same
  // heading replaces the incumbent as soon as it is better at all.
  [[nodiscard]] std::optional<SpatialRouteCandidate3D>
  consider(SpatialRouteCandidate3D candidate);
  [[nodiscard]] const SpatialRouteCandidate3D* incumbent() const noexcept;

private:
  double continuity_improvement_margin_s_{0.0};
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
  // Adopts the request's departure evidence into the resident world when the
  // occupied evidence itself is retained.
  void installDepartureEvidence(const PersistentPlannerWorld3D& world);
  // Re-anchors the proprioceptive seed at the request start. The seed is the
  // vehicle's own pose, and the start is that pose now; the world's copy was
  // made when the raw world was captured — a continuation earlier — and a
  // vehicle that has drifted into contact since was refused every departure
  // the node's own validators, seeded at the current pose, still grant.
  // Returns how far the seed lay from the start, negative without a seed.
  double anchorDepartureEvidence(const Point3& start);
  // Starts every search over: the backward session, the feasibility frontier,
  // and the execution-time refinement all restart from these endpoints.
  void initializeSearch(const PersistentPlannerRequest3D& request,
                        PersistentPlannerNode3D start, PersistentPlannerNode3D goal,
                        const Point3& search_goal);
  [[nodiscard]] FeasiblePathSearch3D::Endpoints3D searchEndpoints() const noexcept;
  [[nodiscard]] std::optional<std::vector<Point3>>
  rebaseIncumbent(const std::vector<Point3>& incumbent, const Point3& start,
                  const Point3& goal) const;
  [[nodiscard]] FlightPathTimeProfile3D
  pathTimeProfile(const std::vector<Point3>& path, const Vec3& initial_velocity) const;
  [[nodiscard]] std::optional<SpatialRouteCandidate3D>
  makeCandidate(std::vector<Point3> path, SpatialRouteCandidateSource3D source,
                const Vec3& initial_velocity) const;
  // Shortcut simplification and clearance centering, applied to every path
  // before it becomes a candidate whatever produced it. A lattice zig-zag left
  // in a route costs a stop-and-turn at every corner during execution, and a
  // vertex left against a jamb costs a crawl through the doorway and a route
  // the next observed voxel blocks.
  [[nodiscard]] std::vector<Point3>
  refinePublishedPath(std::vector<Point3> path,
                      const PersistentPlannerRequest3D& request,
                      std::chrono::steady_clock::time_point deadline,
                      PlannerTelemetry3D& telemetry) const;
  [[nodiscard]] ExecutionTimeRefiner3D::Request3D refinementRequest(
      const PersistentPlannerRequest3D& request, PersistentPlannerNode3D start_anchor,
      PersistentPlannerNode3D goal_anchor, const Point3& search_goal) const noexcept;
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
  // The free points the search leaves the vehicle through before the anchor
  // when no lattice node is reachable from where it stands, or when the nodes
  // it reaches belong to a component the goal is not in. Empty in ordinary
  // flight.
  std::vector<Point3> departure_waypoints_;
  // How many of the start's admissible anchors the search has already tried
  // and exhausted itself against. Reset whenever a route is found or the
  // search restarts.
  std::size_t departure_anchor_skip_{0U};
  // The point the searches end at: the mission goal, or the free point the
  // goal connection refined it to within the goal tolerance.
  Point3 exact_goal_{};
  // The mission goal the searches were initialised for.
  Point3 mission_goal_{};
  std::uint64_t mission_epoch_{0U};
  bool initialized_{false};
  DStarLiteSession3D dstar_session_;
  FeasiblePathSearch3D feasibility_search_;
  ExecutionTimeRefiner3D execution_time_refiner_;
  EscapeSearch3D escape_search_;
  // The way out of a closed component the vehicle is flying, trimmed as it
  // passes each point; see EscapeSearch3D. Absent in ordinary flight.
  std::optional<EscapeSearch3D::Result3D> escape_connection_;
  // The feasibility search exhausted the start's component and the escape
  // search has work to do on the next updates.
  bool escape_search_pending_{false};
  // Where the vehicle stood when its component closed; moving away from it
  // discards what was learnt about the component.
  std::optional<Point3> closed_component_origin_;
  PathPostprocessor3D path_postprocessor_{};
  AnytimePlannerCoordinator3D coordinator_;
  std::size_t adaptive_edges_in_extracted_path_{0U};
  // Session that last received the incumbent; see
  // PersistentPlannerRequest3D::session_id.
  std::uint64_t published_session_id_{0U};
  // Session whose incumbent-discard request was already honoured.
  std::uint64_t applied_incumbent_rejection_sequence_{0U};
};

} // namespace drone_city_nav::detail
