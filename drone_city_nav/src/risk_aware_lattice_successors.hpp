#pragma once

#include "drone_city_nav/risk_aware_lattice.hpp"

#include <array>
#include <span>
#include <vector>

namespace drone_city_nav {

class BoundedWorkerPool;

namespace detail {

struct LatticeKey {
  int x{0};
  int y{0};
  int heading{0};

  bool operator==(const LatticeKey&) const = default;
};

struct LatticeKeyHash {
  std::size_t operator()(const LatticeKey& key) const noexcept;
};

struct LatticeSuccessor {
  LatticeKey key{};
  Point2 endpoint{};
  double length_m{0.0};
  double edge_cost{0.0};
  std::array<Point2, 4U> edge_points{};
  std::size_t edge_point_count{0U};
};

struct LatticeSuccessorCollection {
  std::vector<LatticeSuccessor> successors;
  bool roi_boundary_seen{false};
  LatticeSuccessorDiagnostics diagnostics{};
  LatticeSuccessorBatchProfile profile{};
};

struct LatticeSearchRoi {
  double minimum_x{0.0};
  double maximum_x{0.0};
  double minimum_y{0.0};
  double maximum_y{0.0};
};

[[nodiscard]] Point2 latticeCellCenter(const mppi::EsdfGrid& grid,
                                       const LatticeKey& key) noexcept;

[[nodiscard]] LatticeSuccessorCollection collectLatticeSuccessors(
    const mppi::EsdfGrid& grid, std::span<const float> esdf_m, Point2 current,
    const LatticeKey& current_key, LatticeRiskStage stage,
    const RiskAwareLatticeConfig& config, const LatticeSearchRoi& search_roi,
    std::span<const LatticeFrontierBlacklistEntry> frontier_blacklist,
    BoundedWorkerPool* worker_pool);

} // namespace detail
} // namespace drone_city_nav
