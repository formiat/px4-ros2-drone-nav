#pragma once

#include "drone_city_nav/incremental_topological_planner_3d.hpp"
#include "drone_city_nav/risk_aware_lattice_3d.hpp"

#include <cstddef>
#include <optional>
#include <span>

namespace drone_city_nav {

struct IncrementalTopologicalLatticeAdapter3DConfig {
  double maximum_lookahead_m{30.0};
  // A selected strategic path may contain regional boundaries closer than an
  // executable route can be extended with certified overlap. Skip those
  // internal boundaries until this net station advance is available; the
  // local lattice still proves the complete materialized segment raw-safe.
  double minimum_executable_lookahead_m{0.0};
  // A bend inside the local planner's goal-capture neighbourhood is already
  // acquired and cannot be a useful finite segment endpoint. Continue to the
  // first route point outside this radius instead of repeatedly stopping at a
  // regenerated start connector.
  double segment_capture_radius_m{2.0};
  // Consecutive graph edges are locally executable as one segment only while
  // they retain this directional alignment. The adapter stops at a bend
  // instead of letting the local lattice shortcut across it.
  double minimum_collinear_direction_cosine{0.95};
};

struct IncrementalTopologicalLatticeDirective3D {
  Lattice3DStrategicDirective lattice{};
  std::uint64_t strategic_plan_id{0U};
  std::size_t source_segment_index{0U};
  double source_station_m{0.0};
  double target_station_m{0.0};
  double progress_floor_station_m{0.0};
  double projection_distance_m{0.0};
  std::size_t captured_boundaries_skipped{0U};
  std::size_t executable_lookahead_boundaries_skipped{0U};
  bool reaches_topological_target{false};
};

struct TopologicalPolylineProjection3D {
  Point3 point{};
  std::size_t segment_index{0U};
  double segment_fraction{0.0};
  double station_m{0.0};
  double remaining_m{0.0};
  double distance_m{0.0};
};

[[nodiscard]] std::optional<TopologicalPolylineProjection3D>
projectOntoTopologicalPolyline3D(std::span<const Point3> points, const Point3& position,
                                 double minimum_station_m = 0.0);

[[nodiscard]] bool incrementalTopologicalLatticeAdapter3DConfigIsValid(
    const IncrementalTopologicalLatticeAdapter3DConfig& config) noexcept;

[[nodiscard]] std::optional<IncrementalTopologicalLatticeDirective3D>
makeIncrementalTopologicalLatticeDirective3D(
    const IncrementalTopologicalPlan3D& plan, const Point3& position,
    const IncrementalTopologicalLatticeAdapter3DConfig& config = {},
    double minimum_source_station_m = 0.0);

} // namespace drone_city_nav
